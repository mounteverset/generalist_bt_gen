from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import String

from gen_bt_interfaces.action import MissionCommand


@dataclass
class CoordinatorParams:
    mission_action_name: str
    llm_plan_service: str
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
    mission_namespace: str = 'mission_coordinator'


class MissionCoordinatorNode(Node):
    """Skeleton node that will orchestrate UI, LLM, and BT executor interactions."""

    def __init__(self) -> None:
        super().__init__('mission_coordinator')
        self.params = self._declare_and_get_parameters()

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

        return CoordinatorParams(
            mission_action_name=declare('mission_action_name', '/mission_coordinator/execute_tree'),
            llm_plan_service=declare('llm_plan_service', '/llm_interface/plan_subtree'),
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
        )

    # endregion

    def _announce_startup(self) -> None:
        self.get_logger().info(
            'MissionCoordinatorNode ready (demo_mode=%s, mission action: %s)',
            self.params.demo_mode,
            self.params.mission_action_name,
        )
        self._publish_status('Mission coordinator initialized.')
        self._publish_active_subtree('idle')

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

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
            'Received mission goal (session=%s)', getattr(goal_request, 'session_id', '')
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    async def _execute_mission(self, goal_handle) -> MissionCommand.Result:
        goal: MissionCommand.Goal = goal_handle.request
        feedback = MissionCommand.Feedback()
        feedback.stage = 'DEMO'
        feedback.detail = (
            'MissionCoordinatorNode is running in demo mode; no external calls executed.'
        )
        goal_handle.publish_feedback(feedback)
        self._publish_status(f'Received command: {goal.command}')
        self._publish_active_subtree('demo_subtree')

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

    # endregion


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
