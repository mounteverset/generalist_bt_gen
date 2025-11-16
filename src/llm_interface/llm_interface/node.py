from __future__ import annotations

from typing import List, Optional, Tuple

import rclpy
from gen_bt_interfaces.srv import PlanSubtree, SelectBehaviorTree
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
        self.declare_parameter('model_name', 'gpt-4o-mini')
        self._model_name = self.get_parameter('model_name').value

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
            'LLMInterfaceNode ready (plan_service=%s, selection_service=%s)',
            planning_topic,
            selection_topic,
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
        """Builds a prompt for the LLM, currently simulated with heuristics."""
        if not tree_ids:
            return None

        prompt_lines = [
            f"User command: {user_command}",
            "Available behavior trees:",
        ]
        for idx, (tree_id, description) in enumerate(zip(tree_ids, descriptions)):
            prompt_lines.append(f"{idx}. {tree_id} :: {description}")
        prompt = "\n".join(prompt_lines)

        choice, rationale = self._mock_llm_selection(prompt, tree_ids, descriptions)
        if choice is None:
            return None
        try:
            index = tree_ids.index(choice)
        except ValueError:
            index = 0
        confidence = 0.9 if rationale and 'confident' in rationale.lower() else 0.6
        reason = (
            f"[{self._model_name}] Selected tree '{tree_ids[index]}' "
            f"because {rationale or 'it best matched the mission context'}."
        )
        return index, confidence, reason

    def _mock_llm_selection(
        self, prompt: str, tree_ids: List[str], descriptions: List[str]
    ) -> Tuple[Optional[str], Optional[str]]:
        """Temporary heuristic that simulates an LLM decision."""
        self.get_logger().debug("LLM selection prompt:\\n%s", prompt)
        keywords = [token.lower() for token in prompt.split() if len(token) > 2]
        for tree_id, description in zip(tree_ids, descriptions):
            text = description.lower()
            if any(keyword in text for keyword in keywords):
                return tree_id, "it directly mentions the requested capability, high confident"
        if tree_ids:
            return tree_ids[0], "no explicit match found; defaulting to first entry"
        return None, None

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
