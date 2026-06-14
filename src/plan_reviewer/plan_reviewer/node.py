from __future__ import annotations

import base64
import json
import mimetypes
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from gen_bt_interfaces.srv import ReviewPlan
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .renderer import load_json_value, render_plan_review_image

try:
    from langchain_core.messages import HumanMessage
except ModuleNotFoundError:
    HumanMessage = None
try:
    from langchain_google_genai import ChatGoogleGenerativeAI
except ModuleNotFoundError:
    ChatGoogleGenerativeAI = None
try:
    from langchain_openai import ChatOpenAI
except ModuleNotFoundError:
    ChatOpenAI = None
try:
    from langchain_openrouter import ChatOpenRouter
except ModuleNotFoundError:
    ChatOpenRouter = None


REVIEW_PROMPT_TEMPLATE = """You are a plan safety reviewer for a Clearpath Husky A200 ground robot.

Review the generated waypoint plan using the rendered map image and JSON context.
OSM and satellite geometry are reasoning context. Current MoveTo behavior trees execute x,y,yaw map-frame waypoints unless a GPS behavior tree is explicitly selected.

Review criteria:
- Mission fulfillment: requested count/coverage, refinement request honored, obvious omissions.
- Robot constraints: Husky must not drive through water, barriers, steps, unsafe streets, unknown SLAM space, or non-path terrain where avoidable.
- Coordinate sanity: waypoint mode matches map mode, waypoints are visible/in-bounds, no impossible jumps, no lat/lon accidentally treated as x,y.
- Exploration coverage: for explore_area.xml, waypoints should stay inside area_polygon and frontiers should cover the requested area.
- Context use: compare the route against OSM_CONTEXT.linear_features, SATELLITE_MAP, ANNOTATED_SLAM_MAP_IMAGE, robot pose, and mission reasoner capability constraints.

Return strict JSON only, with this shape:
{{
  "status": "pass|warn|reject",
  "summary": "short operator-facing result",
  "findings": [
    {{
      "severity": "info|warning|critical",
      "category": "mission_fulfillment|robot_safety|map_alignment|coordinate_error|unknown_space",
      "waypoint_indices": [1, 2],
      "description": "specific issue",
      "recommended_fix": "specific correction"
    }}
  ],
  "recommended_action": "submit_to_operator|regenerate|block"
}}

PLAN_REVIEW_INPUT_JSON:
{review_input_json}
"""


class PlanReviewerNode(Node):
    def __init__(self) -> None:
        super().__init__('plan_reviewer')
        self.declare_parameter('review_service_name', '/plan_reviewer/review_plan')
        self.declare_parameter('review_artifact_directory', '/tmp/context_gatherer')
        self.declare_parameter('llm_enabled', True)
        self.declare_parameter('provider', 'gemini')
        self.declare_parameter('model_name', 'gemini-2.5-flash')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('max_image_bytes', 5_000_000)

        self._review_artifact_directory = Path(
            str(self.get_parameter('review_artifact_directory').value)
        ).expanduser()
        self._llm_enabled = bool(self.get_parameter('llm_enabled').value)
        self._provider = str(self.get_parameter('provider').value or 'gemini').lower()
        self._model_name = str(self.get_parameter('model_name').value or 'gemini-2.5-flash')
        self._temperature = float(self.get_parameter('temperature').value)
        self._max_image_bytes = int(self.get_parameter('max_image_bytes').value)
        self._llm = None

        service_name = str(self.get_parameter('review_service_name').value)
        self._review_service = self.create_service(
            ReviewPlan,
            service_name,
            self.handle_review_plan,
        )
        self.get_logger().info(
            f'PlanReviewerNode ready (service={service_name}, provider={self._provider}, '
            f'model={self._model_name}, llm_enabled={self._llm_enabled})'
        )

    def handle_review_plan(
        self,
        request: ReviewPlan.Request,
        response: ReviewPlan.Response,
    ) -> ReviewPlan.Response:
        try:
            render_info = render_plan_review_image(
                session_id=request.session_id or 'unknown',
                subtree_id=request.subtree_id or '',
                user_command=request.user_command or '',
                payload_json=request.payload_json or '{}',
                context_snapshot_json=request.context_snapshot_json or '{}',
                attachment_uris=list(request.attachment_uris),
                output_directory=self._review_artifact_directory,
            )
            review_input = self._build_review_input(request, render_info)
            if self._llm_enabled:
                review = self._review_with_llm(review_input, render_info)
            else:
                review = self._fallback_review(review_input, render_info)
        except Exception as exc:
            self.get_logger().error(f'Plan review failed: {exc}')
            response.status_code = response.ERROR
            response.summary = f'Plan review failed: {exc}'
            response.findings_json = '[]'
            response.review_image_uri = ''
            response.recommended_action = 'submit_to_operator'
            return response

        response.status_code = self._status_code(review.get('status', 'warn'), response)
        response.summary = str(review.get('summary') or 'Plan review completed.')
        response.findings_json = json.dumps(review.get('findings') or [], ensure_ascii=False)
        response.review_image_uri = str(render_info.get('image_uri') or '')
        response.recommended_action = str(
            review.get('recommended_action') or self._default_action_for_status(review.get('status'))
        )
        self.get_logger().info(
            f"Plan review result (session={request.session_id}, subtree={request.subtree_id}): "
            f"{review.get('status')} - {response.summary}"
        )
        return response

    def _build_review_input(
        self,
        request: ReviewPlan.Request,
        render_info: Dict[str, Any],
    ) -> Dict[str, Any]:
        context = load_json_value(request.context_snapshot_json) or {}
        payload = load_json_value(request.payload_json) or {}
        contract = load_json_value(request.subtree_contract_json) or {}
        normalized = render_info.get('normalized_plan') or {}
        map_preview = normalized.get('map_preview') if isinstance(normalized, dict) else {}
        map_metadata = map_preview.get('map_metadata') if isinstance(map_preview, dict) else {}

        return {
            'session_id': request.session_id,
            'subtree_id': request.subtree_id,
            'mission_text': request.user_command,
            'operator_feedback': request.operator_feedback,
            'payload_json': payload,
            'subtree_contract_json': contract,
            'attachment_uris': list(request.attachment_uris),
            'review_image_uri': render_info.get('image_uri') or '',
            'map_available': bool(render_info.get('map_available')),
            'map_preview': map_preview or {},
            'map_metadata': map_metadata or {},
            'waypoints': render_info.get('waypoints') or [],
            'area_polygon': render_info.get('area_polygon') or [],
            'frontiers': render_info.get('frontiers') or [],
            'waypoint_pixels': render_info.get('waypoint_pixels') or [],
            'area_polygon_pixels': render_info.get('area_polygon_pixels') or [],
            'frontier_pixels': render_info.get('frontier_pixels') or [],
            'waypoint_area_checks': render_info.get('waypoint_area_checks') or [],
            'render_warnings': render_info.get('render_warnings') or [],
            'context_focus': {
                'OSM_CONTEXT': context.get('OSM_CONTEXT') if isinstance(context, dict) else None,
                'SATELLITE_MAP': context.get('SATELLITE_MAP') if isinstance(context, dict) else None,
                'ANNOTATED_SLAM_MAP_IMAGE': (
                    context.get('ANNOTATED_SLAM_MAP_IMAGE') if isinstance(context, dict) else None
                ),
                'ROBOT_POSE': context.get('ROBOT_POSE') if isinstance(context, dict) else None,
                'GPS_FIX': context.get('GPS_FIX') if isinstance(context, dict) else None,
                'MISSION_REQUEST': context.get('MISSION_REQUEST') if isinstance(context, dict) else None,
                'MISSION_REFINEMENT': context.get('MISSION_REFINEMENT') if isinstance(context, dict) else None,
            },
        }

    def _review_with_llm(
        self,
        review_input: Dict[str, Any],
        render_info: Dict[str, Any],
    ) -> Dict[str, Any]:
        try:
            raw = self._invoke_llm(review_input, str(render_info.get('image_path') or ''))
            parsed = self._parse_review_json(raw)
        except Exception as exc:
            self.get_logger().warning(f'LLM review unavailable, using fallback: {exc}')
            parsed = self._fallback_review(review_input, render_info)
        if not review_input.get('map_available') and parsed.get('status') == 'pass':
            parsed['status'] = 'warn'
            parsed['summary'] = 'Plan review image was unavailable; JSON-only review requires operator attention.'
            parsed.setdefault('findings', []).append(
                {
                    'severity': 'warning',
                    'category': 'map_alignment',
                    'waypoint_indices': [],
                    'description': 'No rendered map image was available for multimodal review.',
                    'recommended_fix': 'Confirm the route against the map before execution.',
                }
            )
            parsed['recommended_action'] = 'submit_to_operator'
        return self._normalize_review(parsed)

    def _invoke_llm(self, review_input: Dict[str, Any], image_path: str) -> str:
        llm = self._get_llm()
        prompt = REVIEW_PROMPT_TEMPLATE.format(
            review_input_json=json.dumps(review_input, ensure_ascii=False, indent=2)
        )
        image_part = self._image_part(image_path)
        if image_part and HumanMessage is not None:
            message = HumanMessage(
                content=[
                    {'type': 'text', 'text': prompt},
                    image_part,
                ]
            )
            result = llm.invoke([message])
        else:
            result = llm.invoke(prompt)
        return str(getattr(result, 'content', result))

    def _get_llm(self):
        if self._llm is not None:
            return self._llm
        if self._provider == 'gemini':
            if ChatGoogleGenerativeAI is None:
                raise RuntimeError('langchain-google-genai is not installed')
            self._llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=self._temperature,
            )
        elif self._provider == 'openai':
            if ChatOpenAI is None:
                raise RuntimeError('langchain-openai is not installed')
            self._llm = ChatOpenAI(model=self._model_name, temperature=self._temperature)
        elif self._provider == 'openrouter':
            if ChatOpenRouter is None:
                raise RuntimeError('langchain-openrouter is not installed')
            self._llm = ChatOpenRouter(model=self._model_name, temperature=self._temperature)
        else:
            raise RuntimeError(f'Unsupported review provider: {self._provider}')
        return self._llm

    def _image_part(self, image_path: str) -> Optional[dict]:
        if not image_path:
            return None
        path = Path(image_path)
        if not path.exists():
            return None
        data = path.read_bytes()
        if len(data) > self._max_image_bytes:
            self.get_logger().warning(
                f'Skipping review image ({len(data)} bytes exceeds {self._max_image_bytes})'
            )
            return None
        mime, _ = mimetypes.guess_type(path.name)
        if mime not in ('image/png', 'image/jpeg'):
            return None
        encoded = base64.b64encode(data).decode('ascii')
        return {'type': 'image_url', 'image_url': {'url': f'data:{mime};base64,{encoded}'}}

    def _fallback_review(
        self,
        review_input: Dict[str, Any],
        render_info: Dict[str, Any],
    ) -> Dict[str, Any]:
        findings = []
        status = 'pass'
        if not review_input.get('map_available'):
            status = 'warn'
            findings.append(
                {
                    'severity': 'warning',
                    'category': 'map_alignment',
                    'waypoint_indices': [],
                    'description': 'No map image was available, so the plan could only be reviewed from JSON.',
                    'recommended_fix': 'Show the route to the operator before execution.',
                }
            )
        for item in render_info.get('waypoint_pixels') or []:
            if item.get('pixel_x') is None or not item.get('in_bounds'):
                status = 'reject'
                findings.append(
                    {
                        'severity': 'critical',
                        'category': 'coordinate_error',
                        'waypoint_indices': [int(item.get('index') or 0)],
                        'description': item.get('reason') or 'Waypoint is not visible on the selected map.',
                        'recommended_fix': 'Regenerate the payload with waypoints inside the map frame.',
                    }
                )
        for item in render_info.get('waypoint_area_checks') or []:
            if item.get('in_area') is False:
                status = 'reject'
                findings.append(
                    {
                        'severity': 'critical',
                        'category': 'mission_fulfillment',
                        'waypoint_indices': [int(item.get('index') or 0)],
                        'description': 'Waypoint is outside the requested exploration polygon.',
                        'recommended_fix': 'Regenerate the payload with all exploration waypoints inside area_polygon.',
                    }
                )
        return {
            'status': status,
            'summary': self._fallback_summary(status),
            'findings': findings,
            'recommended_action': self._default_action_for_status(status),
        }

    @staticmethod
    def _parse_review_json(raw: str) -> Dict[str, Any]:
        text = (raw or '').strip()
        if text.startswith('```'):
            text = text.strip('`').strip()
            if text.lower().startswith('json'):
                text = text[4:].strip()
        start = text.find('{')
        end = text.rfind('}')
        if start >= 0 and end > start:
            text = text[start:end + 1]
        parsed = json.loads(text)
        if not isinstance(parsed, dict):
            raise ValueError('review response was not a JSON object')
        return parsed

    def _normalize_review(self, review: Dict[str, Any]) -> Dict[str, Any]:
        status = str(review.get('status') or 'warn').lower()
        if status not in ('pass', 'warn', 'reject'):
            status = 'warn'
        findings = review.get('findings')
        if not isinstance(findings, list):
            findings = []
        normalized_findings = []
        for finding in findings:
            if isinstance(finding, dict):
                normalized_findings.append(finding)
        action = str(review.get('recommended_action') or self._default_action_for_status(status))
        if action not in ('submit_to_operator', 'regenerate', 'block'):
            action = self._default_action_for_status(status)
        return {
            'status': status,
            'summary': str(review.get('summary') or self._fallback_summary(status)),
            'findings': normalized_findings,
            'recommended_action': action,
        }

    @staticmethod
    def _status_code(status: str, response: ReviewPlan.Response) -> int:
        if status == 'pass':
            return response.PASS
        if status == 'reject':
            return response.REJECT
        return response.WARN

    @staticmethod
    def _default_action_for_status(status: Optional[str]) -> str:
        if status == 'reject':
            return 'regenerate'
        return 'submit_to_operator'

    @staticmethod
    def _fallback_summary(status: str) -> str:
        if status == 'pass':
            return 'No obvious waypoint/map issues found by fallback review.'
        if status == 'reject':
            return 'Fallback review found waypoint projection issues.'
        return 'Fallback review requires operator attention.'


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlanReviewerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('PlanReviewerNode interrupted, shutting down.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['PlanReviewerNode', 'main']
