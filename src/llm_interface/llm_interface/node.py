from __future__ import annotations

import json
import os
from typing import List, Optional, Tuple

import rclpy
from gen_bt_interfaces.srv import PlanSubtree, SelectBehaviorTree
try:
    from langchain_core.prompts import PromptTemplate
    from langchain_google_genai import ChatGoogleGenerativeAI
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "langchain-core / langchain-google-genai not found. Install them in the "
        "ROS Python environment or disable selection_llm_enabled."
    ) from exc
from rclpy.node import Node


class LLMInterfaceNode(Node):
    """LLM orchestration stub that exposes selection + planning services."""

    PLAN_STATUS_SUCCESS = 0
    PLAN_STATUS_RETRY = 1
    PLAN_STATUS_ESCALATE = 2

    SELECT_FOUND = 0
    SELECT_NO_MATCH = 1
    SELECT_ERROR = 2

    def __init__(self) -> None:
        super().__init__('llm_interface')

        self.declare_parameter('planning_service_name', '/llm_interface/plan_subtree')
        self.declare_parameter('selection_service_name', '/llm_interface/select_behavior_tree')
        self.declare_parameter(
            'default_bt_template',
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
            "  <BehaviorTree ID=\"MainTree\">\n"
            "    <Sequence>\n"
            "      <AlwaysSuccess name=\"Acknowledge\" />\n"
            "    </Sequence>\n"
            "  </BehaviorTree>\n"
            "</root>\n",
        )
        self.declare_parameter('model_name', 'gemini-2.5-pro')
        self.declare_parameter('selection_llm_enabled', True)
        self.declare_parameter('selection_temperature', 0.0)
        self._model_name = self.get_parameter('model_name').value
        self._selection_llm_enabled = bool(
            self.get_parameter('selection_llm_enabled').value
        )
        self._selection_temperature = float(
            self.get_parameter('selection_temperature').value
        )

        planning_topic = self.get_parameter('planning_service_name').value
        selection_topic = self.get_parameter('selection_service_name').value
        self._plan_template = self.get_parameter('default_bt_template').value

        self._plan_service = self.create_service(
            PlanSubtree, planning_topic, self.handle_plan_request
        )
        self._selection_service = self.create_service(
            SelectBehaviorTree, selection_topic, self.handle_selection_request
        )
        self.get_logger().info(
            f"LLMInterfaceNode ready (plan_service={planning_topic}, selection_service={selection_topic})"
        )

    # region Selection
    def handle_selection_request(
        self, request: SelectBehaviorTree.Request, response: SelectBehaviorTree.Response
    ) -> SelectBehaviorTree.Response:
        if len(request.available_trees) != len(request.tree_descriptions):
            response.status_code = self.SELECT_ERROR
            response.reason = (
                'available_trees and tree_descriptions size mismatch '
                f'({len(request.available_trees)} vs {len(request.tree_descriptions)})'
            )
            self.get_logger().warning(response.reason)
            return response

        selection = self._select_tree_via_llm(
            request.user_command, list(request.available_trees), list(request.tree_descriptions)
        )
        if selection is None:
            response.status_code = self.SELECT_NO_MATCH
            response.reason = 'No behavior tree matched the current command.'
            response.selected_tree = ''
            response.confidence = 0.0
            return response

        idx, confidence, reason = selection
        if idx is None:
            response.status_code = self.SELECT_NO_MATCH
            response.reason = 'No behavior tree matched the current command.'
            response.selected_tree = ''
            response.confidence = 0.0
            return response

        response.status_code = self.SELECT_FOUND
        response.selected_tree = request.available_trees[idx]
        response.confidence = confidence
        response.reason = reason
        return response

    def _select_tree_via_llm(
        self, user_command: str, tree_ids: List[str], descriptions: List[str]
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None

        if not self._selection_llm_enabled:
            return self._fallback_selection(tree_ids)

        try:
            chain = self._get_selection_chain()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return self._fallback_selection(tree_ids)

        payload = [
            {"id": tree_id, "description": desc}
            for tree_id, desc in zip(tree_ids, descriptions)
        ]

        try:
            llm_result = chain.invoke(
                {
                    "user_command": user_command,
                    "trees_json": json.dumps(payload, ensure_ascii=False),
                }
            )
        except Exception as exc:
            self.get_logger().error(f"Gemini selection failed: {exc}")
            return self._fallback_selection(tree_ids)

        raw_content = getattr(llm_result, 'content', llm_result)
        cleaned_content = self._strip_code_fence(str(raw_content))
        try:
            parsed = json.loads(cleaned_content)
        except Exception as exc:
            self.get_logger().warning(
                f"Unable to parse Gemini response '{raw_content}': {exc}"
            )
            return self._fallback_selection(tree_ids)

        selected_id = parsed.get('tree_id')
        if selected_id not in tree_ids:
            self.get_logger().warning(
                f"Gemini returned unknown tree '{selected_id}'; falling back."
            )
            return self._fallback_selection(tree_ids)

        confidence = float(parsed.get('confidence', 0.6))
        rationale = parsed.get('rationale', '')
        idx = tree_ids.index(selected_id)
        reason = (
            f"[{self._model_name}] Selected tree '{selected_id}' "
            f"because {rationale or 'it best matched the mission context'}."
        )
        return idx, max(0.0, min(confidence, 1.0)), reason

    def _strip_code_fence(self, text: str) -> str:
        stripped = text.strip()
        if stripped.startswith("```"):
            stripped = stripped.lstrip('`')
            # remove leading language tag if present
            parts = stripped.split('\n', 1)
            stripped = parts[1] if len(parts) > 1 else ''
            if stripped.endswith("```"):
                stripped = stripped.rsplit("```", 1)[0]
        return stripped.strip()

    def _get_selection_chain(self):
        if not hasattr(self, '_selection_chain'):
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    'GEMINI_API_KEY not set; disable selection_llm_enabled or export the key.'
                )
            llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=self._selection_temperature,
                api_key=api_key,
            )
            template = PromptTemplate.from_template(
                """
You are a mission planner for a field robot. Given USER_COMMAND and a JSON array of
behavior trees (each with id + description), choose the best tree or return NONE.
tree_id MUST be exactly one of the string IDs in TREES_JSON (e.g., "demo_tree.xml").
Do NOT answer with numeric indexes.
Respond strictly as JSON: {{"tree_id": "<id or NONE>", "confidence": 0-1, "rationale": "..."}}
USER_COMMAND: {user_command}
TREES_JSON: {trees_json}
"""
            )
            self._selection_chain = template | llm
        return self._selection_chain

    def _fallback_selection(
        self, tree_ids: List[str]
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None
        tree_id = tree_ids[0]
        return 0, 0.5, f"[heuristic] defaulted to '{tree_id}'"

    # endregion

    # region Planning
    def handle_plan_request(
        self, request: PlanSubtree.Request, response: PlanSubtree.Response
    ) -> PlanSubtree.Response:
        response.status_code = self.PLAN_STATUS_SUCCESS
        response.bt_xml = self._synthesize_bt(request.mission_text)
        summary = (
            f"Generated default subtree for mission '{request.mission_text}'. "
            "For full LLM integration, replace this stub with LangChain pipelines."
        )
        response.summary = summary
        response.tool_invocations = '[]'
        response.reason = ''
        self.get_logger().info(
            "PlanSubtree handled mission '%s' (context bytes=%d)",
            request.mission_text,
            len(request.context_snapshot),
        )
        return response

    def _synthesize_bt(self, mission_text: str) -> str:
        mission_comment = f"<!-- Mission: {mission_text} -->"
        if '{mission_text}' in self._plan_template:
            return self._plan_template.replace('{mission_text}', mission_text)
        return mission_comment + '\n' + self._plan_template

    # endregion


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LLMInterfaceNode interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['LLMInterfaceNode', 'main']
