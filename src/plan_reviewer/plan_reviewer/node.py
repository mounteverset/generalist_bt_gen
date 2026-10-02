from __future__ import annotations

import base64
import json
import mimetypes
import os
from pathlib import Path
import re
import time
from typing import Any, Dict, Optional
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import rclpy
from gen_bt_interfaces.srv import ReviewPlan
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from llm_interface.gemini_direct import direct_gemini_runnable
from .renderer import load_json_value, render_plan_review_image
from .safety_validation import deterministic_plan_findings

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


def _openrouter_generation_metadata(generation_id: str) -> dict[str, Any]:
    api_key = os.environ.get('OPENROUTER_API_KEY')
    if not generation_id or not api_key:
        return {}
    request = Request(
        'https://openrouter.ai/api/v1/generation?'
        + urlencode({'id': generation_id}),
        headers={'Authorization': f'Bearer {api_key}'},
    )
    for attempt in range(15):
        try:
            with urlopen(request, timeout=10) as response:
                data = json.load(response).get('data', {})
            if data.get('provider_name'):
                return data
        except (OSError, ValueError):
            pass
        if attempt < 14:
            time.sleep(1)
    return {}


def _provider_recording_openrouter(**kwargs):
    class ProviderRecordingChatOpenRouter(ChatOpenRouter):
        def _create_chat_result(self, response):
            raw = (
                response
                if isinstance(response, dict)
                else response.model_dump(by_alias=True)
            )
            result = super()._create_chat_result(response)
            usage = raw.get('usage') if isinstance(raw.get('usage'), dict) else {}
            cost = raw.get('cost')
            if cost is None:
                cost = usage.get('cost')
            metadata = (
                _openrouter_generation_metadata(str(raw.get('id') or ''))
                if not raw.get('provider') or cost is None else {}
            )
            provider = raw.get('provider') or metadata.get('provider_name')
            if cost is None:
                cost = metadata.get('total_cost')
            for generation in result.generations:
                if provider:
                    generation.message.response_metadata[
                        'openrouter_provider'
                    ] = provider
                if model := raw.get('model'):
                    generation.message.response_metadata['openrouter_model'] = model
                if service_tier := raw.get('service_tier'):
                    generation.message.response_metadata[
                        'openrouter_service_tier'
                    ] = service_tier
                if cost is not None:
                    generation.message.response_metadata[
                        'openrouter_cost_usd'
                    ] = cost
            return result

    llm = ProviderRecordingChatOpenRouter(**kwargs)
    from openrouter import UNSET
    if 'temperature' not in kwargs:
        llm.temperature = UNSET
    llm.top_p = UNSET
    return llm


REVIEW_PROMPT_TEMPLATE = """You are a plan safety reviewer for the deployed robot described in PLAN_REVIEW_INPUT_JSON.

Review the generated waypoint plan using the rendered map image and JSON context.
OSM and satellite geometry are reasoning context. MoveTo executes x,y,yaw map-frame waypoints; MoveToGPS executes geographic waypoints through FollowGPSWaypoints.
FindAnything results are shown as magenta crosshair rings labeled with the object query; planned waypoints remain blue numbered circles.

Review criteria:
- Mission fulfillment: requested count/coverage, refinement request honored, obvious omissions.
- Object-target fidelity: for find_and_drive_to_nearest_object.xml, map-frame destinations must match FindAnything object markers. When FindAnything has no locations, GPS destinations must match distinct OSM_CONTEXT.tree_features centers; honor singular, nearest, plural, and all-matches wording.
- Platform constraints: Apply the platform type and capabilities from context. Ground robots must avoid water, barriers, steps, unsafe streets, unknown SLAM space, and non-path terrain where avoidable. Water-surface vessels must stay in navigable water and avoid land, barriers, and restricted water.
- Coordinate sanity: waypoint mode matches map mode, waypoints are visible/in-bounds, no impossible jumps, no lat/lon accidentally treated as x,y.
- Exploration coverage: for explore_area.xml, GPS waypoints should stay inside the geographic area overlay when one is supplied, and frontiers should cover the requested area.
- Context use: compare the route against OSM_CONTEXT.linear_features, SATELLITE_MAP, ROBOT_POSE, GPS_FIX, and mission reasoner capability constraints.
- Deterministic findings in review_input are mandatory. For osm_steps, identify the affected waypoint segment and recommend a local detour while preserving safe route sections.

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
        self.declare_parameter('omit_temperature', False)
        self.declare_parameter('max_output_tokens', 4096)
        self.declare_parameter('reasoning_effort', '')
        self.declare_parameter(
            'openrouter_provider_only', Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter('openrouter_allow_fallbacks', False)
        self.declare_parameter('openrouter_seed', -1)
        self.declare_parameter('openrouter_max_retries', 0)
        self.declare_parameter('openrouter_service_tier', '')
        self.declare_parameter('openrouter_timeout_sec', 60.0)
        self.declare_parameter('max_image_bytes', 5_000_000)

        self._review_artifact_directory = Path(
            str(self.get_parameter('review_artifact_directory').value)
        ).expanduser()
        self._llm_enabled = bool(self.get_parameter('llm_enabled').value)
        self._provider = str(self.get_parameter('provider').value or 'gemini').lower()
        self._model_name = str(self.get_parameter('model_name').value or 'gemini-2.5-flash')
        self._temperature = float(self.get_parameter('temperature').value)
        self._omit_temperature = bool(self.get_parameter('omit_temperature').value)
        self._max_output_tokens = int(self.get_parameter('max_output_tokens').value)
        self._reasoning_effort = str(
            self.get_parameter('reasoning_effort').value or ''
        ).strip()
        self._openrouter_provider_only = list(
            self.get_parameter('openrouter_provider_only').value
        )
        self._openrouter_allow_fallbacks = bool(
            self.get_parameter('openrouter_allow_fallbacks').value
        )
        self._openrouter_seed = int(self.get_parameter('openrouter_seed').value)
        self._openrouter_max_retries = int(
            self.get_parameter('openrouter_max_retries').value
        )
        self._openrouter_service_tier = str(
            self.get_parameter('openrouter_service_tier').value or ''
        ).strip()
        self._openrouter_timeout_sec = float(
            self.get_parameter('openrouter_timeout_sec').value
        )
        self._max_image_bytes = int(self.get_parameter('max_image_bytes').value)
        self._llm = None
        evidence_root = os.environ.get('GENERALIST_BT_EVIDENCE_ROOT', '').strip()
        self._evaluation_evidence_root = (
            Path(evidence_root).expanduser().resolve() if evidence_root else None
        )

        service_name = str(self.get_parameter('review_service_name').value)
        self._review_service = self.create_service(
            ReviewPlan,
            service_name,
            self.handle_review_plan,
        )
        self.get_logger().info(
            f'PlanReviewerNode ready (service={service_name}, provider={self._provider}, '
            f'model={self._model_name}, max_output_tokens={self._max_output_tokens}, '
            f'reasoning_effort={self._reasoning_effort or "<disabled>"}, '
            f'llm_enabled={self._llm_enabled})'
        )

    @staticmethod
    def _provider_key(provider: Any) -> str:
        return str(provider or '').strip().lower().replace(' ', '-')

    def _audit_review_request(
        self, session_id: str, prompt: str, image_path: str
    ) -> Optional[tuple[Path, float]]:
        if self._evaluation_evidence_root is None or not session_id:
            return None
        safe_session = re.sub(r'[^A-Za-z0-9._-]', '_', session_id)
        directory = self._evaluation_evidence_root / safe_session / 'llm_calls'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f'{time.time_ns()}_plan_review_request.json'
        path.write_text(
            json.dumps(
                {
                    'session_id': session_id,
                    'stage': 'plan_review',
                    'provider': self._provider,
                    'model': self._model_name,
                    'requested_provider_only': (
                        list(getattr(self, '_openrouter_provider_only', []))
                        if self._provider == 'openrouter'
                        else ['google-gemini-api'] if self._provider == 'gemini' else []
                    ),
                    'allow_fallbacks_requested': (
                        bool(getattr(self, '_openrouter_allow_fallbacks', False))
                        if self._provider == 'openrouter' else None
                    ),
                    'seed': (
                        getattr(self, '_openrouter_seed', -1)
                        if self._provider in {'openrouter', 'gemini'}
                        and getattr(self, '_openrouter_seed', -1) >= 0
                        else None
                    ),
                    'cache_disabled_requested': bool(
                        self._provider == 'openrouter'
                        and getattr(self, '_openrouter_provider_only', [])
                    ),
                    'max_output_tokens': getattr(self, '_max_output_tokens', 4096),
                    'requested_service_tier': (
                        getattr(self, '_openrouter_service_tier', '') or None
                        if self._provider == 'openrouter' else None
                    ),
                    'max_transport_retries': (
                        getattr(self, '_openrouter_max_retries', 0)
                        if self._provider == 'openrouter' else 0
                    ),
                    'reasoning': (
                        {'effort': getattr(self, '_reasoning_effort', '')}
                        if getattr(self, '_reasoning_effort', '')
                        else None
                    ),
                    'prompt': prompt,
                    'attachment_uris': [Path(image_path).resolve().as_uri()]
                    if image_path
                    else [],
                    'started_unix_s': time.time(),
                },
                ensure_ascii=False,
                indent=2,
            )
            + '\n',
            encoding='utf-8',
        )
        return path, time.monotonic()

    def _audit_review_response(
        self,
        audit: Optional[tuple[Path, float]], result=None, error: Any = ''
    ) -> None:
        if audit is None:
            return
        request_path, started = audit
        response_path = request_path.with_name(
            request_path.name.replace('_request.json', '_response.json')
        )
        response_metadata = getattr(result, 'response_metadata', None)
        returned_provider = (
            response_metadata.get('openrouter_provider')
            or response_metadata.get('provider')
            if isinstance(response_metadata, dict) else None
        )
        request = json.loads(request_path.read_text(encoding='utf-8'))
        requested_providers = list(request.get('requested_provider_only') or [])
        provider_verified = (
            self._provider_key(returned_provider)
            in {self._provider_key(provider) for provider in requested_providers}
            if requested_providers
            else None
        )
        returned_service_tier = (
            response_metadata.get('openrouter_service_tier')
            if isinstance(response_metadata, dict) else None
        )
        requested_service_tier = request.get('requested_service_tier')
        response_path.write_text(
            json.dumps(
                {
                    'status': 'error' if error else 'ok',
                    'error': error,
                    'raw_content': str(getattr(result, 'content', result or '')),
                    'response_metadata': response_metadata,
                    'returned_provider': returned_provider,
                    'provider_verified': provider_verified,
                    'returned_service_tier': returned_service_tier,
                    'service_tier_verified': (
                        returned_service_tier == requested_service_tier
                        if requested_service_tier else None
                    ),
                    'usage_metadata': getattr(result, 'usage_metadata', None),
                    'cost_usd': (
                        response_metadata.get('openrouter_cost_usd')
                        if isinstance(response_metadata, dict)
                        and response_metadata.get('openrouter_cost_usd') is not None
                        else response_metadata.get('cost_usd')
                        if isinstance(response_metadata, dict)
                        else None
                    ),
                    'elapsed_s': time.monotonic() - started,
                    'finished_unix_s': time.time(),
                },
                ensure_ascii=False,
                indent=2,
                default=str,
            )
            + '\n',
            encoding='utf-8',
        )

    @staticmethod
    def _exception_details(exc: Exception) -> dict:
        def safe_attr(value, name):
            try:
                return getattr(value, name, None)
            except Exception as nested:
                return f'<unavailable: {type(nested).__name__}: {nested}>'

        details = {
            'type': f'{type(exc).__module__}.{type(exc).__name__}',
            'message': str(exc),
            'repr': repr(exc),
            'args': list(exc.args),
        }
        for name in ('status_code', 'body', 'request_id', 'code'):
            value = safe_attr(exc, name)
            if value is not None:
                details[name] = value
        response = safe_attr(exc, 'raw_response')
        if response is None:
            response = safe_attr(exc, 'response')
        if response is not None and not isinstance(response, str):
            details.setdefault('status_code', safe_attr(response, 'status_code'))
            details.setdefault('body', safe_attr(response, 'text'))
            headers = safe_attr(response, 'headers')
            if headers and not isinstance(headers, str):
                details['response_headers'] = {
                    name: headers[name]
                    for name in ('content-type', 'x-request-id', 'cf-ray')
                    if name in headers
                }
        return details

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
            'object_locations': render_info.get('object_locations') or [],
            'waypoint_pixels': render_info.get('waypoint_pixels') or [],
            'area_polygon_pixels': render_info.get('area_polygon_pixels') or [],
            'frontier_pixels': render_info.get('frontier_pixels') or [],
            'object_location_pixels': render_info.get('object_location_pixels') or [],
            'waypoint_area_checks': render_info.get('waypoint_area_checks') or [],
            'render_warnings': render_info.get('render_warnings') or [],
            'context_snapshot': context if isinstance(context, dict) else {},
            'context_focus': {
                'OSM_CONTEXT': context.get('OSM_CONTEXT') if isinstance(context, dict) else None,
                'SATELLITE_MAP': context.get('SATELLITE_MAP') if isinstance(context, dict) else None,
                'ANNOTATED_SLAM_MAP_IMAGE': (
                    context.get('ANNOTATED_SLAM_MAP_IMAGE') if isinstance(context, dict) else None
                ),
                'FIND_ANYTHING': context.get('FIND_ANYTHING') if isinstance(context, dict) else None,
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
        deterministic_findings = deterministic_plan_findings(review_input)
        review_input['deterministic_findings'] = deterministic_findings
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
        if deterministic_findings:
            parsed['status'] = 'reject'
            parsed['summary'] = (
                'Deterministic safety validation rejected the generated plan.'
            )
            parsed.setdefault('findings', []).extend(deterministic_findings)
            parsed['recommended_action'] = 'block'
        return self._normalize_review(parsed)

    def _invoke_llm(self, review_input: Dict[str, Any], image_path: str) -> str:
        llm = self._get_llm()
        llm_review_input = {
            key: value for key, value in review_input.items() if key != 'context_snapshot'
        }
        prompt = REVIEW_PROMPT_TEMPLATE.format(
            review_input_json=json.dumps(llm_review_input, ensure_ascii=False, indent=2)
        )
        audit = self._audit_review_request(
            str(review_input.get('session_id') or ''), prompt, image_path
        )
        image_part = self._image_part(image_path)
        try:
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
        except Exception as exc:
            self._audit_review_response(audit, error=self._exception_details(exc))
            raise
        self._audit_review_response(audit, result=result)
        return str(getattr(result, 'content', result))

    def _get_llm(self):
        if self._llm is not None:
            return self._llm
        if self._provider == 'gemini':
            self._llm = direct_gemini_runnable(
                model=self._model_name,
                max_output_tokens=self._max_output_tokens,
                seed=max(0, int(getattr(self, '_openrouter_seed', 42))),
                thinking_level=self._reasoning_effort or 'medium',
                temperature=(
                    None if getattr(self, '_omit_temperature', False)
                    else self._temperature
                ),
                timeout_s=max(1.0, self._openrouter_timeout_sec),
            )
        elif self._provider == 'openai':
            if ChatOpenAI is None:
                raise RuntimeError('langchain-openai is not installed')
            kwargs = {
                'model': self._model_name,
                'max_tokens': self._max_output_tokens,
            }
            if not getattr(self, '_omit_temperature', False):
                kwargs['temperature'] = self._temperature
            self._llm = ChatOpenAI(**kwargs)
        elif self._provider == 'openrouter':
            if ChatOpenRouter is None:
                raise RuntimeError('langchain-openrouter is not installed')
            kwargs = {
                'model': self._model_name,
                'max_tokens': self._max_output_tokens,
                'max_retries': max(
                    0, int(getattr(self, '_openrouter_max_retries', 0))
                ),
                'timeout': max(
                    1,
                    int(
                        round(
                            1000.0
                            * float(getattr(self, '_openrouter_timeout_sec', 60.0))
                        )
                    ),
                ),
            }
            if not getattr(self, '_omit_temperature', False):
                kwargs['temperature'] = self._temperature
            providers = list(getattr(self, '_openrouter_provider_only', []))
            if providers:
                kwargs['openrouter_provider'] = {
                    'only': providers,
                    'allow_fallbacks': bool(
                        getattr(self, '_openrouter_allow_fallbacks', False)
                    ),
                    'require_parameters': True,
                    'data_collection': 'deny',
                }
            seed = int(getattr(self, '_openrouter_seed', -1))
            if seed >= 0:
                kwargs['seed'] = seed
            reasoning_effort = str(
                getattr(self, '_reasoning_effort', '') or ''
            ).strip()
            if reasoning_effort:
                kwargs['reasoning'] = {'effort': reasoning_effort}
            service_tier = str(
                getattr(self, '_openrouter_service_tier', '') or ''
            ).strip()
            if service_tier:
                kwargs['model_kwargs'] = {'service_tier': service_tier}
            self._llm = _provider_recording_openrouter(**kwargs)
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
        deterministic_findings = deterministic_plan_findings(review_input)
        if deterministic_findings:
            status = 'reject'
            findings.extend(deterministic_findings)
        if not review_input.get('map_available'):
            if status != 'reject':
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
