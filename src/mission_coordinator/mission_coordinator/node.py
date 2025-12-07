from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.msg import NodeStatus
from gen_bt_interfaces.action import MissionCommand, GatherContext
from gen_bt_interfaces.srv import (
    CreatePayload,
    OperatorDecision,
    PlanSubtree,
    SelectBehaviorTree,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class CoordinatorParams:
    mission_action_name: str
    llm_plan_service: str
    llm_select_service: str
    bt_executor_action: str
    context_gather_action: str
    create_payload_service: str
    context_snapshot_service: str
    status_topic: str
    active_subtree_topic: str
    pending_plan_topic: str
    operator_decision_service: str
    enable_context_snapshot: bool
    require_operator_accept: bool
    demo_mode: bool
    llm_timeout_sec: float
    gather_timeout_sec: float
    payload_timeout_sec: float
    bt_timeout_sec: float
    spin_period_sec: float
    transcript_directory: Path
    known_trees: list
    mission_namespace: str = 'mission_coordinator'


class MissionCoordinatorNode(Node):
    """Skeleton node that will orchestrate UI, LLM, and BT executor interactions."""

    def __init__(self) -> None:
        super().__init__('mission_coordinator')
        self.params = self._declare_and_get_parameters()
        self.debug_logging = bool(self.declare_parameter('enable_debug_logging', True).value)

        # Publishers that UI clients subscribe to.
        self.status_pub = self.create_publisher(String, self.params.status_topic, 10)
        self.subtree_pub = self.create_publisher(String, self.params.active_subtree_topic, 10)
        self.pending_plan_pub = self.create_publisher(
            String, self.params.pending_plan_topic, 10
        )

        # Action server exposed to UI/mission clients.
        self._mission_server = ActionServer(
            self,
            MissionCommand,
            self.params.mission_action_name,
            execute_callback=self._execute_mission,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._announce_startup()

        # Timer placeholder to keep node alive until action server wiring lands.
        self._spin_timer = self.create_timer(
            self.params.spin_period_sec, self._noop_spin_tick
        )

        self._tree_catalog = self._parse_tree_catalog(self.params.known_trees)
        self._log_debug(f'Loaded {len(self._tree_catalog)} known trees.')
        self._select_tree_client = self.create_client(
            SelectBehaviorTree, self.params.llm_select_service
        )
        self._plan_subtree_client = self.create_client(
            PlanSubtree, self.params.llm_plan_service
        )
        self._context_gather_client = ActionClient(
            self, GatherContext, self.params.context_gather_action
        )
        self._create_payload_client = self.create_client(
            CreatePayload, self.params.create_payload_service
        )
        self._bt_executor_client = ActionClient(
            self, ExecuteTree, self.params.bt_executor_action
        )
        self._pending_decisions: Dict[str, asyncio.Future[bool]] = {}
        self._operator_service = self.create_service(
            OperatorDecision,
            self.params.operator_decision_service,
            self._handle_operator_decision,
        )

    def destroy_node(self) -> None:
        self.get_logger().info('Tearing down MissionCoordinatorNode.')
        if hasattr(self, '_mission_server'):
            self._mission_server.destroy()
        return super().destroy_node()

    # region Parameter helpers
    def _declare_and_get_parameters(self) -> CoordinatorParams:
        def declare(name: str, default):
            return self.declare_parameter(name, default).value

        transcript_dir = Path(
            declare('transcript_directory', '~/.generalist_bt/mission_logs')
        ).expanduser()
        transcript_dir.mkdir(parents=True, exist_ok=True)

        known_trees_default = [
            'demo_tree.xml::Iterates waypoint queue via MoveTo and LogTemperature.'
        ]

        return CoordinatorParams(
            mission_action_name=declare('mission_action_name', '/mission_coordinator/execute_tree'),
            llm_plan_service=declare('llm_plan_service', '/llm_interface/plan_subtree'),
            llm_select_service=declare('llm_select_service', '/llm_interface/select_behavior_tree'),
            bt_executor_action=declare('bt_executor_action', '/bt_executor/execute_tree'),
            context_gather_action=declare('context_gather_action', '/context_gatherer/gather'),
            create_payload_service=declare(
                'create_payload_service', '/llm_interface/create_payload'
            ),
            context_snapshot_service=declare(
                'context_snapshot_service', '/context_gatherer/snapshot'
            ),
            status_topic=declare('status_topic', '/mission_coordinator/status_text'),
            active_subtree_topic=declare(
                'active_subtree_topic', '/mission_coordinator/active_subtree'
            ),
            pending_plan_topic=declare(
                'pending_plan_topic', '/mission_coordinator/pending_plan'
            ),
            operator_decision_service=declare(
                'operator_decision_service', '/mission_coordinator/operator_decision'
            ),
            enable_context_snapshot=bool(declare('enable_context_snapshot', True)),
            require_operator_accept=bool(declare('require_operator_accept', False)),
            demo_mode=bool(declare('demo_mode', True)),
            llm_timeout_sec=float(declare('llm_timeout_sec', 45.0)),
            gather_timeout_sec=float(declare('gather_timeout_sec', 15.0)),
            payload_timeout_sec=float(declare('payload_timeout_sec', 20.0)),
            bt_timeout_sec=float(declare('bt_timeout_sec', 120.0)),
            spin_period_sec=float(declare('spin_period_sec', 0.1)),
            transcript_directory=transcript_dir,
            known_trees=declare('known_trees', known_trees_default) or known_trees_default,
        )

    # endregion

    def _announce_startup(self) -> None:
        self.get_logger().info(
            f"MissionCoordinatorNode ready (demo_mode={self.params.demo_mode}, mission action: {self.params.mission_action_name})"
        )
        self._publish_status('Mission coordinator initialized.')
        self._publish_active_subtree('idle')

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self._log_debug(f'Status update: {text}')

    def _publish_active_subtree(self, subtree: str) -> None:
        msg = String()
        msg.data = subtree
        self.subtree_pub.publish(msg)

    def _publish_pending_plan(self, plan: dict) -> None:
        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False)
        self.pending_plan_pub.publish(msg)
        self._log_debug(f'Published pending plan for session={plan.get("session_id", "")}')

    def _noop_spin_tick(self) -> None:
        """Placeholder timer callback to keep executor ticking."""
        pass

    # region Action server callbacks
    def _goal_callback(self, goal_request: MissionCommand.Goal) -> GoalResponse:
        self.get_logger().info(
            f"Received mission goal (session={getattr(goal_request, 'session_id', '')})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    async def _execute_mission(self, goal_handle) -> MissionCommand.Result:
        goal: MissionCommand.Goal = goal_handle.request
        feedback = MissionCommand.Feedback()
        self._publish_status(f"Received mission: {goal.command}")
        self._publish_active_subtree('selecting_tree')
        self._log_debug(f'Mission goal received: "{goal.command}" (session={goal.session_id})')

        if self.params.demo_mode:
            feedback.stage = 'DEMO'
            feedback.detail = (
                'MissionCoordinatorNode is running in demo mode; no external calls executed.'
            )
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MissionCommand.Result()
                result.accepted = False
                result.outcome_message = 'Mission canceled before execution.'
                return result
            goal_handle.succeed()
            result = MissionCommand.Result()
            result.accepted = True
            result.outcome_message = 'Demo mission acknowledged.'
            return result

        selected_tree = await self._select_behavior_tree(goal)
        if not selected_tree:
            goal_handle.succeed()
            self._publish_status('No suitable behavior tree found.')
            result = MissionCommand.Result()
            result.accepted = False
            result.outcome_message = 'LLM did not provide a valid behavior tree.'
            return result

        self._publish_active_subtree(selected_tree)
        self._publish_status(f'Gathering context for {selected_tree}.')
        gather_result = await self._gather_context(selected_tree, goal)
        if not gather_result or not gather_result.success:
            goal_handle.succeed()
            result = MissionCommand.Result()
            result.accepted = False
            result.outcome_message = (
                gather_result.message if gather_result else 'Context gather failed.'
            )
            self._publish_status(result.outcome_message)
            return result

        self._publish_status(f'Building payload for {selected_tree}.')
        payload_response = await self._create_payload(selected_tree, goal, gather_result)
        if payload_response is None:
            goal_handle.succeed()
            result = MissionCommand.Result()
            result.accepted = False
            result.outcome_message = 'CreatePayload failed.'
            self._publish_status(result.outcome_message)
            return result

        self._publish_active_subtree(selected_tree)
        should_execute = True
        if self._requires_operator_accept(goal):
            should_execute = await self._await_operator_decision(
                selected_tree, goal, payload_response
            )

        if not should_execute:
            goal_handle.succeed()
            result = MissionCommand.Result()
            result.accepted = False
            result.outcome_message = 'Mission canceled by operator.'
            self._publish_status(result.outcome_message)
            return result

        self._log_debug(f'Selected tree {selected_tree}, dispatching to BT executor.')
        node_status, executor_message = await self._execute_behavior_tree(
            selected_tree, goal, payload_response.payload_json
        )

        goal_handle.succeed()
        result = MissionCommand.Result()
        success = node_status == NodeStatus.SUCCESS
        result.accepted = success
        result.outcome_message = executor_message
        if success:
            self._publish_status(f'Behavior tree {selected_tree} completed successfully.')
        else:
            self._publish_status(f'Behavior tree {selected_tree} finished with status {node_status}.')
        return result

    # endregion

    def _parse_tree_catalog(self, entries) -> list[Tuple[str, str]]:
        catalog = []
        if isinstance(entries, list):
            for entry in entries:
                if isinstance(entry, str):
                    if '::' in entry:
                        tree_id, desc = entry.split('::', 1)
                    else:
                        tree_id, desc = entry, ''
                    tree_id = tree_id.strip()
                    if tree_id:
                        catalog.append((tree_id, desc.strip()))
        if not catalog:
            catalog = [('demo_tree.xml', 'Fallback demo tree.')]
        self._log_debug(f'Parsed tree catalog: {catalog}')
        return catalog

    async def _select_behavior_tree(self, goal: MissionCommand.Goal) -> Optional[str]:
        if not self._tree_catalog:
            self._log_debug('Tree catalog empty, cannot select behavior.')
            return None
        self._log_debug('Waiting for SelectBehaviorTree service...')
        if not self._wait_for_service(self._select_tree_client, self.params.llm_select_service):
            return None
        request = SelectBehaviorTree.Request()
        request.session_id = goal.session_id or 'unknown'
        request.user_command = goal.command
        request.available_trees = [tree_id for tree_id, _ in self._tree_catalog]
        request.tree_descriptions = [desc for _, desc in self._tree_catalog]
        request.context_snapshot = goal.context_json or ''
        self._log_debug(f'Calling selection service for session={request.session_id}')
        response = await self._call_service(self._select_tree_client, request)
        if not response or response.status_code != 0:
            self.get_logger().warn('SelectBehaviorTree returned no match.')
            if response:
                self._log_debug(f'Select service response: status={response.status_code}, reason="{response.reason}"')
            return None
        self._log_debug(
            f'Selection response -> tree={response.selected_tree}, confidence={response.confidence}, reason="{response.reason}"'
        )
        return response.selected_tree

    def _context_requirements_for_tree(self, tree_id: str) -> list[str]:
        # Placeholder: pull from catalog/metadata when available.
        return []

    def _subtree_contract_for_tree(self, tree_id: str) -> str:
        # Placeholder: this should be populated from tree metadata.
        return '{}'

    async def _gather_context(self, tree_id: str, goal: MissionCommand.Goal):
        if not self._wait_for_action(
            self._context_gather_client, self.params.context_gather_action
        ):
            return None
        gather_goal = GatherContext.Goal()
        gather_goal.session_id = goal.session_id or 'unknown'
        gather_goal.subtree_id = tree_id
        gather_goal.context_requirements = self._context_requirements_for_tree(tree_id)
        gather_goal.timeout_sec = float(self.params.gather_timeout_sec)
        gather_goal.geo_hint = goal.context_json or ''

        send_future = self._context_gather_client.send_goal_async(
            gather_goal, feedback_callback=self._gather_feedback
        )
        goal_handle = await send_future
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status('Context gatherer rejected the goal.')
            return None
        result_future = goal_handle.get_result_async()
        result = await result_future
        if result is None:
            self._publish_status('Context gatherer result unavailable.')
            return None
        if result.result.success:
            self._log_debug('Context gather success.')
        else:
            self._publish_status(f'Context gather failed: {result.result.message}')
        return result.result

    def _gather_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._publish_status(f'Context gather: {feedback.stage} {feedback.detail}')
        self._log_debug(f'Context gather feedback: {feedback.stage} {feedback.detail}')

    async def _create_payload(
        self, tree_id: str, goal: MissionCommand.Goal, gather_result
    ) -> Optional[CreatePayload.Response]:
        if not self._wait_for_service(
            self._create_payload_client, self.params.create_payload_service
        ):
            return None
        request = CreatePayload.Request()
        request.session_id = goal.session_id or 'unknown'
        request.subtree_id = tree_id
        request.subtree_contract_json = self._subtree_contract_for_tree(tree_id)
        request.context_snapshot_json = gather_result.context_json
        request.attachment_uris = list(gather_result.attachment_uris)
        self._log_debug('Calling CreatePayload service.')
        response = await self._call_service(self._create_payload_client, request)
        if response is None:
            self._publish_status('CreatePayload returned no response.')
            return None
        if response.status_code != response.SUCCESS:
            self._publish_status(f'CreatePayload failed: {response.reason}')
            return None
        self._log_debug(f'CreatePayload succeeded: {response.reasoning}')
        return response

    def _requires_operator_accept(self, goal: MissionCommand.Goal) -> bool:
        if not self.params.require_operator_accept:
            return False
        auto_execute = False
        if goal.context_json:
            try:
                context = json.loads(goal.context_json)
                auto_execute = bool(context.get('auto_execute', False))
            except Exception:
                self._log_debug('Failed to parse context_json for auto_execute flag.')
        return not auto_execute

    async def _await_operator_decision(
        self, tree_id: str, goal: MissionCommand.Goal, payload: CreatePayload.Response
    ) -> bool:
        session_id = goal.session_id or 'unknown'
        plan = {
            'session_id': session_id,
            'tree_id': tree_id,
            'mission': goal.command,
            'summary': payload.reasoning,
            'payload_json': payload.payload_json,
            'tool_trace_json': payload.tool_trace_json,
        }
        self._publish_pending_plan(plan)
        self._publish_status(
            f'Awaiting operator confirmation for session={session_id} (tree={tree_id}).'
        )
        loop = asyncio.get_running_loop()
        future: asyncio.Future[bool] = loop.create_future()
        self._pending_decisions[session_id] = future
        try:
            decision = await future
        finally:
            self._pending_decisions.pop(session_id, None)
        return bool(decision)

    async def _execute_behavior_tree(
        self, tree_id: str, goal: MissionCommand.Goal, payload_json: str
    ) -> Tuple[int, str]:
        self._log_debug('Waiting for bt_executor action server...')
        if not self._wait_for_bt_executor():
            return NodeStatus.FAILURE, 'BT executor unavailable.'
        exec_goal = ExecuteTree.Goal()
        exec_goal.target_tree = tree_id
        exec_goal.payload = payload_json or '{}'

        send_future = self._bt_executor_client.send_goal_async(
            exec_goal, feedback_callback=self._bt_feedback
        )
        goal_handle = await send_future
        if goal_handle is None or not goal_handle.accepted:
            return NodeStatus.FAILURE, 'BT executor rejected the goal.'
        result_future = goal_handle.get_result_async()
        result = await result_future
        if result is None:
            return NodeStatus.FAILURE, 'BT executor result unavailable.'
        self._log_debug(
            f'BT executor finished with status={result.result.node_status.status}, message="{result.result.return_message}"'
        )
        return result.result.node_status.status, result.result.return_message

    def _bt_feedback(self, feedback_msg):
        self._publish_status(f"BT feedback: {feedback_msg.feedback.message}")
        self._log_debug(f'BT feedback: {feedback_msg.feedback.message}')

    def _handle_operator_decision(
        self, request: OperatorDecision.Request, response: OperatorDecision.Response
    ) -> OperatorDecision.Response:
        session_id = request.session_id or 'unknown'
        future = self._pending_decisions.get(session_id)
        if future is None or future.done():
            response.accepted = False
            response.message = f'No pending plan for session {session_id}.'
            self._log_debug(response.message)
            return response
        future.set_result(bool(request.approve))
        response.accepted = True
        response.message = (
            'Execution approved by operator.'
            if request.approve
            else 'Execution rejected by operator.'
        )
        self._publish_status(f'{response.message} (session={session_id}).')
        return response

    def _wait_for_service(self, client, name: str) -> bool:
        for _ in range(10):
            if client.wait_for_service(timeout_sec=1.0):
                return True
            time.sleep(0.1)
        self.get_logger().warn(f'Service {name} unavailable.')
        return False

    def _wait_for_action(self, client, name: str) -> bool:
        for _ in range(10):
            if client.wait_for_server(timeout_sec=1.0):
                return True
            time.sleep(0.1)
        self.get_logger().warn(f'Action server {name} unavailable.')
        return False

    def _wait_for_bt_executor(self) -> bool:
        for _ in range(10):
            if self._bt_executor_client.wait_for_server(timeout_sec=1.0):
                return True
            time.sleep(0.1)
        self.get_logger().warn('BT executor action server unavailable.')
        return False

    async def _call_service(self, client, request):
        return await client.call_async(request)

    def _log_debug(self, message: str) -> None:
        if self.debug_logging:
            self.get_logger().info(f'[trace] {message}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MissionCoordinatorNode interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['MissionCoordinatorNode', 'main']
