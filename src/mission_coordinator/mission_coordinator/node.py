from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.msg import NodeStatus
from gen_bt_interfaces.action import MissionCommand
from gen_bt_interfaces.srv import PlanSubtree, SelectBehaviorTree
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class CoordinatorParams:
    mission_action_name: str
    llm_plan_service: str
    llm_select_service: str
    bt_executor_action: str
    context_snapshot_service: str
    status_topic: str
    active_subtree_topic: str
    enable_context_snapshot: bool
    require_operator_accept: bool
    demo_mode: bool
    llm_timeout_sec: float
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
        self._bt_executor_client = ActionClient(
            self, ExecuteTree, self.params.bt_executor_action
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
            context_snapshot_service=declare(
                'context_snapshot_service', '/context_gatherer/snapshot'
            ),
            status_topic=declare('status_topic', '/mission_coordinator/status_text'),
            active_subtree_topic=declare(
                'active_subtree_topic', '/mission_coordinator/active_subtree'
            ),
            enable_context_snapshot=bool(declare('enable_context_snapshot', True)),
            require_operator_accept=bool(declare('require_operator_accept', False)),
            demo_mode=bool(declare('demo_mode', True)),
            llm_timeout_sec=float(declare('llm_timeout_sec', 45.0)),
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
        self._log_debug(f'Selected tree {selected_tree}, dispatching to BT executor.')
        node_status, executor_message = await self._execute_behavior_tree(selected_tree, goal)

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

    async def _execute_behavior_tree(
        self, tree_id: str, goal: MissionCommand.Goal
    ) -> Tuple[int, str]:
        self._log_debug('Waiting for bt_executor action server...')
        if not self._wait_for_bt_executor():
            return NodeStatus.FAILURE, 'BT executor unavailable.'
        exec_goal = ExecuteTree.Goal()
        exec_goal.target_tree = tree_id
        payload = {'user_command': goal.command, 
                   'session_id': goal.session_id,
                   'waypoints': '[0.0, 0.0, 0.0]; [1.0, 1.0, 0.0]; [2.0, 2.0, 0.0]'}  # Example payload
        exec_goal.payload = json.dumps(payload, ensure_ascii=False)

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

    def _wait_for_service(self, client, name: str) -> bool:
        for _ in range(10):
            if client.wait_for_service(timeout_sec=1.0):
                return True
            time.sleep(0.1)
        self.get_logger().warn(f'Service {name} unavailable.')
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
