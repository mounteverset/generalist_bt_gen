from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.msg import NodeStatus
from gen_bt_interfaces.action import MissionCommand, GatherContext
from gen_bt_interfaces.srv import (
    CreatePayload,
    GetMissionState,
    MissionControl,
    OperatorDecision,
    PlanSubtree,
    SelectBehaviorTree,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String


@dataclass
class CoordinatorParams:
    mission_action_name: str
    status_service: str
    control_service: str
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
    tree_metadata_file: str
    mission_namespace: str = 'mission_coordinator'


class MissionCoordinatorNode(Node):
    """Skeleton node that will orchestrate UI, LLM, and BT executor interactions."""

    STATE_IDLE = 'IDLE'
    STATE_SELECTING_TREE = 'SELECTING_TREE'
    STATE_GATHERING_CONTEXT = 'GATHERING_CONTEXT'
    STATE_BUILDING_PAYLOAD = 'BUILDING_PAYLOAD'
    STATE_WAITING_APPROVAL = 'WAITING_APPROVAL'
    STATE_EXECUTING = 'EXECUTING'
    STATE_SUCCEEDED = 'SUCCEEDED'
    STATE_FAILED = 'FAILED'
    STATE_CANCELED = 'CANCELED'

    def __init__(self) -> None:
        super().__init__('mission_coordinator')
        self.params = self._declare_and_get_parameters()
        self.debug_logging = bool(self.declare_parameter('enable_debug_logging', True).value)
        self._state_lock = threading.Lock()
        self._active_session_id = ''
        self._lifecycle_state = self.STATE_IDLE
        self._status_snapshot = 'Idle'
        self._active_subtree_snapshot = 'unknown'
        self._pending_plan_snapshot: Dict[str, Any] = {}
        self._last_result_snapshot: Dict[str, Any] = {}
        self._active_bt_goal_handle = None
        self._abort_requested_session = ''
        self._action_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = ReentrantCallbackGroup()

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
            callback_group=self._action_callback_group,
        )

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
        self._pending_decisions: Dict[str, Future] = {}
        self._operator_feedback: Dict[str, str] = {}
        self._status_service = self.create_service(
            GetMissionState,
            self.params.status_service,
            self._handle_status_query,
            callback_group=self._service_callback_group,
        )
        self._control_service = self.create_service(
            MissionControl,
            self.params.control_service,
            self._handle_mission_control,
            callback_group=self._service_callback_group,
        )
        self._operator_service_name = self._build_operator_service_name(
            self.params.operator_decision_service
        )
        self._operator_service = self.create_service(
            OperatorDecision,
            self._operator_service_name,
            self._handle_operator_decision,
            callback_group=self._service_callback_group,
        )
        self._operator_service_alias = None
        if self._operator_service_name != self.params.operator_decision_service:
            self._operator_service_alias = self.create_service(
                OperatorDecision,
                self.params.operator_decision_service,
                self._handle_operator_decision,
                callback_group=self._service_callback_group,
            )
        
        self._announce_startup()

        # Load tree metadata
        self._tree_metadata = self._load_tree_metadata(self.params.tree_metadata_file)

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
            status_service=declare('status_service', '/mission_coordinator/status'),
            control_service=declare('control_service', '/mission_coordinator/control'),
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
            require_operator_accept=bool(declare('require_operator_accept', True)),
            demo_mode=bool(declare('demo_mode', True)),
            llm_timeout_sec=float(declare('llm_timeout_sec', 45.0)),
            gather_timeout_sec=float(declare('gather_timeout_sec', 15.0)),
            payload_timeout_sec=float(declare('payload_timeout_sec', 20.0)),
            bt_timeout_sec=float(declare('bt_timeout_sec', 120.0)),
            spin_period_sec=float(declare('spin_period_sec', 0.1)),
            transcript_directory=transcript_dir,
            known_trees=declare('known_trees', known_trees_default) or known_trees_default,
            tree_metadata_file=declare('tree_metadata_file', ''),
        )

    # endregion

    def _announce_startup(self) -> None:
        self.get_logger().info(
            f"MissionCoordinatorNode ready (demo_mode={self.params.demo_mode}, mission action: {self.params.mission_action_name})"
        )
        self._log_debug(
            f'Control services: status={self.params.status_service}, control={self.params.control_service}, '
            f'operator_primary={self._operator_service_name}, '
            f'operator_alias={self.params.operator_decision_service}'
        )
        self._set_lifecycle_state(self.STATE_IDLE)
        self._publish_status('Mission coordinator initialized.')
        self._publish_active_subtree('idle')
        self._clear_pending_plan()

    def _build_operator_service_name(self, base_name: str) -> str:
        base = (base_name or '/mission_coordinator/operator_decision').strip()
        if not base:
            base = '/mission_coordinator/operator_decision'
        base = base.rstrip('/')
        # Include PID so duplicate launches don't contend on one operator service endpoint.
        return f'{base}/{self.get_name()}_{os.getpid()}'

    def _publish_status(self, text: str) -> None:
        with self._state_lock:
            self._status_snapshot = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self._log_debug(f'Status update: {text}')

    def _publish_active_subtree(self, subtree: str) -> None:
        with self._state_lock:
            self._active_subtree_snapshot = subtree
        msg = String()
        msg.data = subtree
        self.subtree_pub.publish(msg)

    def _publish_pending_plan(self, plan: dict) -> None:
        with self._state_lock:
            self._pending_plan_snapshot = dict(plan)
        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False)
        self.pending_plan_pub.publish(msg)
        self._log_debug(f'Published pending plan for session={plan.get("session_id", "")}')

    def _clear_pending_plan(self) -> None:
        with self._state_lock:
            self._pending_plan_snapshot = {}
        msg = String()
        msg.data = '{}'
        self.pending_plan_pub.publish(msg)
        self._log_debug('Cleared pending plan.')

    def _set_lifecycle_state(self, state: str) -> None:
        with self._state_lock:
            self._lifecycle_state = state
        self._log_debug(f'Lifecycle state -> {state}')

    def _set_last_result(self, payload: Dict[str, Any]) -> None:
        with self._state_lock:
            self._last_result_snapshot = dict(payload)

    def _clear_active_session(self) -> None:
        with self._state_lock:
            self._active_session_id = ''
            self._active_bt_goal_handle = None
            self._abort_requested_session = ''

    def _snapshot_state(self) -> Dict[str, Any]:
        with self._state_lock:
            return {
                'active_session_id': self._active_session_id,
                'lifecycle_state': self._lifecycle_state,
                'status_text': self._status_snapshot,
                'active_subtree': self._active_subtree_snapshot,
                'pending_plan': dict(self._pending_plan_snapshot),
                'last_result': dict(self._last_result_snapshot),
            }

    @staticmethod
    def _normalize_session_id(raw_session_id: str) -> str:
        normalized = (raw_session_id or '').strip()
        return normalized or 'unknown'

    def _noop_spin_tick(self) -> None:
        """Placeholder timer callback to keep executor ticking."""
        pass

    def _handle_status_query(
        self, request: GetMissionState.Request, response: GetMissionState.Response
    ) -> GetMissionState.Response:
        snapshot = self._snapshot_state()
        requested_session = self._normalize_session_id(request.session_id)
        active_session = str(snapshot.get('active_session_id', '') or '')
        response.ok = not active_session or requested_session in {'unknown', active_session}
        response.active_session_id = active_session
        response.lifecycle_state = str(snapshot.get('lifecycle_state', self.STATE_IDLE))
        response.status_text = str(snapshot.get('status_text', ''))
        response.active_subtree = str(snapshot.get('active_subtree', ''))
        response.pending_plan_json = json.dumps(snapshot.get('pending_plan', {}), ensure_ascii=False)
        response.last_result_json = json.dumps(snapshot.get('last_result', {}), ensure_ascii=False)
        return response

    def _handle_mission_control(
        self, request: MissionControl.Request, response: MissionControl.Response
    ) -> MissionControl.Response:
        session_id = self._normalize_session_id(request.session_id)
        note = (request.note or '').strip()

        if request.command == MissionControl.Request.ABORT:
            ok, message = self._request_abort(session_id, note)
            response.accepted = ok
            response.message = message
            return response

        if request.command == MissionControl.Request.APPROVE:
            ok, message = self._apply_operator_decision(session_id, True, note)
            response.accepted = ok
            response.message = message
            return response

        if request.command == MissionControl.Request.REJECT:
            ok, message = self._apply_operator_decision(session_id, False, note)
            response.accepted = ok
            response.message = message
            return response

        response.accepted = False
        response.message = f'Unsupported mission control command: {request.command}'
        return response

    def _request_abort(self, session_id: str, note: str = '') -> tuple[bool, str]:
        with self._state_lock:
            active_session = self._active_session_id
            lifecycle_state = self._lifecycle_state
            bt_goal_handle = self._active_bt_goal_handle

        if not active_session:
            return False, 'No active mission to abort.'
        if session_id not in {'unknown', active_session}:
            return False, f'Abort rejected: session {session_id} is not active.'

        with self._state_lock:
            self._abort_requested_session = active_session
        if note:
            self._operator_feedback[active_session] = note

        # If waiting for approval, complete it as a rejection immediately.
        if lifecycle_state == self.STATE_WAITING_APPROVAL:
            ok, message = self._apply_operator_decision(
                active_session, False, note or 'Abort requested.'
            )
            if ok:
                return True, f'Abort applied during approval wait (session={active_session}).'
            return False, message

        if bt_goal_handle is not None:
            bt_goal_handle.cancel_goal_async()
            self._publish_status(f'Abort requested for session={active_session}.')
            return True, f'Abort requested for session={active_session}.'

        self._publish_status(f'Abort queued for session={active_session}.')
        return True, f'Abort queued for session={active_session}.'

    def _apply_operator_decision(
        self, session_id: str, approve: bool, feedback: str = ''
    ) -> tuple[bool, str]:
        target_session = session_id
        if target_session == 'unknown':
            with self._state_lock:
                pending_sessions = list(self._pending_decisions.keys())
            if len(pending_sessions) == 1:
                target_session = pending_sessions[0]
            elif pending_sessions:
                return False, 'Multiple pending plans exist; provide a session_id.'

        future = self._pending_decisions.get(target_session)
        if future is None or future.done():
            return False, f'No pending plan for session {target_session}.'

        self._operator_feedback[target_session] = (feedback or '').strip()
        future.set_result(bool(approve))
        decision_text = 'approved' if approve else 'rejected'
        self._publish_status(
            f'Execution {decision_text} by operator (session={target_session}).'
        )
        return True, f'Execution {decision_text} by operator.'

    # region Action server callbacks
    def _goal_callback(self, goal_request: MissionCommand.Goal) -> GoalResponse:
        requested_session = self._normalize_session_id(goal_request.session_id)
        with self._state_lock:
            active_session = self._active_session_id
            if active_session:
                self.get_logger().warning(
                    f'Rejecting mission goal for session={requested_session}: '
                    f'session {active_session} is still active.'
                )
                return GoalResponse.REJECT
            self._active_session_id = requested_session
        self.get_logger().info(
            f'Received mission goal (session={requested_session})'
        )
        self._set_lifecycle_state(self.STATE_SELECTING_TREE)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel request received.')
        return CancelResponse.ACCEPT

    async def _execute_mission(self, goal_handle) -> MissionCommand.Result:
        goal: MissionCommand.Goal = goal_handle.request
        session_id = self._normalize_session_id(goal.session_id)
        if goal.session_id != session_id:
            goal.session_id = session_id
        feedback = MissionCommand.Feedback()
        result = MissionCommand.Result()
        selected_tree = ''
        payload_response = None
        aborted = False

        try:
            self._set_lifecycle_state(self.STATE_SELECTING_TREE)
            self._publish_status(f"Received mission: {goal.command}")
            self._publish_active_subtree('selecting_tree')
            self._log_debug(f'Mission goal received: "{goal.command}" (session={session_id})')

            if self.params.demo_mode:
                feedback.stage = 'DEMO'
                feedback.detail = (
                    'MissionCoordinatorNode is running in demo mode; no external calls executed.'
                )
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.accepted = False
                    result.outcome_message = 'Mission canceled before execution.'
                    self._set_lifecycle_state(self.STATE_CANCELED)
                    return result
                goal_handle.succeed()
                result.accepted = True
                result.outcome_message = 'Demo mission acknowledged.'
                self._set_lifecycle_state(self.STATE_SUCCEEDED)
                return result

            selected_tree = await self._select_behavior_tree(goal)
            if not selected_tree:
                goal_handle.succeed()
                self._set_lifecycle_state(self.STATE_FAILED)
                self._publish_status('No suitable behavior tree found.')
                result.accepted = False
                result.outcome_message = 'LLM did not provide a valid behavior tree.'
                return result

            self._publish_active_subtree(selected_tree)
            self._set_lifecycle_state(self.STATE_GATHERING_CONTEXT)
            self._publish_status(f'Gathering context for {selected_tree}.')
            gather_result = await self._gather_context(selected_tree, goal)
            if not gather_result or not gather_result.success:
                goal_handle.succeed()
                self._set_lifecycle_state(self.STATE_FAILED)
                result.accepted = False
                result.outcome_message = (
                    gather_result.message if gather_result else 'Context gather failed.'
                )
                self._publish_status(result.outcome_message)
                return result

            self._set_lifecycle_state(self.STATE_BUILDING_PAYLOAD)
            self._publish_status(f'Building payload for {selected_tree}.')
            payload_response = await self._create_payload(selected_tree, goal, gather_result)
            if payload_response is None:
                goal_handle.succeed()
                self._set_lifecycle_state(self.STATE_FAILED)
                result.accepted = False
                result.outcome_message = 'CreatePayload failed.'
                self._publish_status(result.outcome_message)
                return result

            self._publish_active_subtree(selected_tree)
            should_execute = True
            if self._requires_operator_accept(goal):
                self._set_lifecycle_state(self.STATE_WAITING_APPROVAL)
                plan = self._build_pending_plan(selected_tree, goal, payload_response)
                self._publish_pending_plan(plan)
                should_execute = await self._await_operator_decision(
                    str(plan['session_id']), selected_tree
                )

            if not should_execute:
                goal_handle.succeed()
                with self._state_lock:
                    aborted = self._abort_requested_session == session_id
                if aborted:
                    self._set_lifecycle_state(self.STATE_CANCELED)
                    result.outcome_message = 'Mission aborted by operator.'
                else:
                    self._set_lifecycle_state(self.STATE_CANCELED)
                    result.outcome_message = 'Mission canceled by operator.'
                result.accepted = False
                self._publish_status(result.outcome_message)
                if payload_response is not None and not aborted:
                    await self._handle_rejected_plan(selected_tree, goal, payload_response)
                return result

            self._set_lifecycle_state(self.STATE_EXECUTING)
            self._log_debug(f'Selected tree {selected_tree}, dispatching to BT executor.')
            node_status, executor_message, aborted = await self._execute_behavior_tree(
                selected_tree, goal, payload_response.payload_json
            )

            goal_handle.succeed()
            success = node_status == NodeStatus.SUCCESS and not aborted
            result.accepted = success
            result.outcome_message = executor_message
            if aborted:
                self._set_lifecycle_state(self.STATE_CANCELED)
                self._publish_status(f'Behavior tree {selected_tree} aborted by operator.')
            elif success:
                self._set_lifecycle_state(self.STATE_SUCCEEDED)
                self._publish_status(f'Behavior tree {selected_tree} completed successfully.')
            else:
                self._set_lifecycle_state(self.STATE_FAILED)
                self._publish_status(
                    f'Behavior tree {selected_tree} finished with status {node_status}.'
                )
            return result
        except Exception as exc:
            self.get_logger().error(f'Unhandled mission execution error: {exc}')
            goal_handle.abort()
            result.accepted = False
            result.outcome_message = f'Mission coordinator error: {exc}'
            self._set_lifecycle_state(self.STATE_FAILED)
            self._publish_status(result.outcome_message)
            return result
        finally:
            self._set_last_result(
                {
                    'session_id': session_id,
                    'accepted': bool(result.accepted),
                    'outcome_message': result.outcome_message,
                    'lifecycle_state': self._snapshot_state().get('lifecycle_state', ''),
                }
            )
            self._clear_pending_plan()
            self._clear_active_session()
            self._set_lifecycle_state(self.STATE_IDLE)
            self._publish_active_subtree('idle')

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

    def _load_tree_metadata(self, metadata_file: str) -> Dict[str, dict]:
        """Load tree metadata from YAML file."""
        metadata = {}
        
        if not metadata_file:
            self._log_debug('No tree_metadata_file specified, using empty metadata.')
            return metadata
        
        metadata_path = Path(metadata_file).expanduser()
        if not metadata_path.exists():
            self.get_logger().warn(f'Tree metadata file not found: {metadata_path}')
            return metadata
        
        try:
            with open(metadata_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if data and 'trees' in data:
                for tree in data['trees']:
                    tree_id = tree.get('id')
                    if tree_id:
                        metadata[tree_id] = {
                            'description': tree.get('description', ''),
                            'context_requirements': tree.get('context_requirements', []),
                            'blackboard_contract': tree.get('blackboard_contract', {})
                        }
            
            self.get_logger().info(f'Loaded metadata for {len(metadata)} trees from {metadata_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load tree metadata: {e}')
        
        return metadata

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

    def _context_requirements_for_tree(self, tree_id: str) -> List[str]:
        """Get context requirements from metadata."""
        if tree_id in self._tree_metadata:
            return self._tree_metadata[tree_id].get('context_requirements', [])
        return []

    def _subtree_contract_for_tree(self, tree_id: str) -> str:
        """Get blackboard contract from metadata as JSON string."""
        if tree_id in self._tree_metadata:
            contract = self._tree_metadata[tree_id].get('blackboard_contract', {})
            return json.dumps(contract, ensure_ascii=False)
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
        req_label = ','.join(gather_goal.context_requirements) or 'none'
        self.get_logger().info(
            f'Requesting context gather (session={gather_goal.session_id}, subtree={tree_id}, '
            f'reqs=[{req_label}], timeout={gather_goal.timeout_sec:.1f}s)'
        )

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
            context_size = len(result.result.context_json or '')
            attachments = len(result.result.attachment_uris)
            self.get_logger().info(
                f'Context gather result (session={gather_goal.session_id}, subtree={tree_id}, '
                f'success=True, context_bytes={context_size}, attachments={attachments})'
            )
            self._log_debug('Context gather success.')
        else:
            self.get_logger().warn(
                f'Context gather result (session={gather_goal.session_id}, subtree={tree_id}, '
                f'success=False, message="{result.result.message}")'
            )
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
        request.user_command = goal.command
        request.subtree_contract_json = self._subtree_contract_for_tree(tree_id)
        request.context_snapshot_json = gather_result.context_json
        request.attachment_uris = list(gather_result.attachment_uris)
        context_size = len(request.context_snapshot_json or '')
        self.get_logger().info(
            f'CreatePayload request (session={request.session_id}, subtree={tree_id}, '
            f'context_bytes={context_size}, attachments={len(request.attachment_uris)})'
        )
        self._log_debug('Calling CreatePayload service.')
        response = await self._call_service(self._create_payload_client, request)
        if response is None:
            self._publish_status('CreatePayload returned no response.')
            return None
        if response.status_code != response.SUCCESS:
            self.get_logger().warn(
                f'CreatePayload failed (session={request.session_id}, subtree={tree_id}, '
                f'status={response.status_code}): {response.reasoning}'
            )
            self._publish_status(f'CreatePayload failed: {response.reasoning}')
            return None
        payload_log = self._truncate_for_log(response.payload_json or '{}')
        reasoning_log = self._truncate_for_log(response.reasoning or '')
        self.get_logger().info(
            f'CreatePayload response (session={request.session_id}, subtree={tree_id}, '
            f'status={response.status_code}): reasoning="{reasoning_log}", payload={payload_log}'
        )
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

    def _build_pending_plan(
        self, tree_id: str, goal: MissionCommand.Goal, payload: CreatePayload.Response
    ) -> dict:
        return {
            'session_id': goal.session_id or 'unknown',
            'tree_id': tree_id,
            'mission': goal.command,
            'summary': payload.reasoning,
            'payload_json': payload.payload_json,
            'tool_trace_json': payload.tool_trace_json,
            'operator_decision_service': self._operator_service_name,
        }

    async def _await_operator_decision(self, session_id: str, tree_id: str) -> bool:
        session_id = session_id or 'unknown'
        self._operator_feedback.pop(session_id, None)
        self._publish_status(
            f'Awaiting operator confirmation for session={session_id} (tree={tree_id}).'
        )
        self.get_logger().info(
            f'Waiting for operator decision (session={session_id}, tree={tree_id}).'
        )
        future = Future()
        self._pending_decisions[session_id] = future
        try:
            decision = await future
            self.get_logger().info(
                f'Operator decision received (session={session_id}, approve={bool(decision)}).'
            )
        finally:
            self._pending_decisions.pop(session_id, None)
            self._clear_pending_plan()
        return bool(decision)

    async def _handle_rejected_plan(
        self, tree_id: str, goal: MissionCommand.Goal, payload: CreatePayload.Response
    ) -> None:
        session_id = goal.session_id or 'unknown'
        feedback_text = self._operator_feedback.get(session_id, '')
        if not feedback_text:
            return
        if not self._wait_for_service(self._plan_subtree_client, self.params.llm_plan_service):
            self._publish_status('PlanSubtree service unavailable; skipping LLM replan.')
            return
        request = PlanSubtree.Request()
        request.session_id = session_id
        request.mission_text = goal.command
        request.context_snapshot = goal.context_json or ''
        request.blackboard_state = payload.payload_json
        prior_plan_info = {
            'prior_tree_id': tree_id,
            'prior_reasoning': payload.reasoning,
            'prior_payload_json': payload.payload_json,
            'operator_feedback': feedback_text,
        }
        request.failure_report = json.dumps(prior_plan_info, ensure_ascii=False)
        request.requested_capabilities = []
        self._log_debug(
            f'Calling PlanSubtree for session={session_id} with operator feedback.'
        )
        response = await self._call_service(self._plan_subtree_client, request)
        if response is None:
            self._publish_status('PlanSubtree returned no response for rejected plan.')
            return
        if response.status_code != 0:
            self._publish_status(
                f'PlanSubtree could not refine plan (status={response.status_code}).'
            )
            return
        self._publish_status(
            'LLM generated an updated plan after operator feedback; see logs for details.'
        )
        self._log_debug(
            f'Updated plan summary: {response.summary} (bt_xml length={len(response.bt_xml)})'
        )

    async def _execute_behavior_tree(
        self, tree_id: str, goal: MissionCommand.Goal, payload_json: str
    ) -> Tuple[int, str, bool]:
        self._log_debug('Waiting for bt_executor action server...')
        if not self._wait_for_bt_executor():
            return NodeStatus.FAILURE, 'BT executor unavailable.', False

        session_id = self._normalize_session_id(goal.session_id)
        with self._state_lock:
            if self._abort_requested_session == session_id:
                return NodeStatus.FAILURE, 'Mission aborted by operator.', True

        exec_goal = ExecuteTree.Goal()
        exec_goal.target_tree = tree_id
        exec_goal.payload = payload_json or '{}'

        send_future = self._bt_executor_client.send_goal_async(
            exec_goal, feedback_callback=self._bt_feedback
        )
        goal_handle = await send_future
        if goal_handle is None or not goal_handle.accepted:
            return NodeStatus.FAILURE, 'BT executor rejected the goal.', False
        with self._state_lock:
            self._active_bt_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result = await result_future
        with self._state_lock:
            self._active_bt_goal_handle = None
        if result is None:
            return NodeStatus.FAILURE, 'BT executor result unavailable.', False

        aborted = bool(result.status == GoalStatus.STATUS_CANCELED)
        with self._state_lock:
            if self._abort_requested_session == session_id:
                aborted = True
        if aborted:
            return NodeStatus.FAILURE, 'Mission aborted by operator.', True

        self._log_debug(
            f'BT executor finished with status={result.result.node_status.status}, message="{result.result.return_message}"'
        )
        return result.result.node_status.status, result.result.return_message, False

    def _bt_feedback(self, feedback_msg):
        self._publish_status(f"BT feedback: {feedback_msg.feedback.message}")
        self._log_debug(f'BT feedback: {feedback_msg.feedback.message}')

    def _handle_operator_decision(
        self, request: OperatorDecision.Request, response: OperatorDecision.Response
    ) -> OperatorDecision.Response:
        ok, message = self._apply_operator_decision(
            self._normalize_session_id(request.session_id),
            bool(request.approve),
            request.feedback or '',
        )
        response.accepted = ok
        response.message = message
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

    @staticmethod
    def _truncate_for_log(text: str, limit: int = 1200) -> str:
        if len(text) <= limit:
            return text
        return f'{text[:limit]}...(truncated {len(text) - limit} chars)'

    def _log_debug(self, message: str) -> None:
        if self.debug_logging:
            self.get_logger().info(f'[trace] {message}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('MissionCoordinatorNode interrupted, shutting down.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['MissionCoordinatorNode', 'main']
