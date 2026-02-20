import argparse
import asyncio
from datetime import datetime, timezone
from functools import partial
import json
from pathlib import Path
import sys
from typing import Any, Dict, Optional

from gen_bt_interfaces.srv import MissionControl
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rich.console import Console
from std_msgs.msg import String

from .command_router import CommandRouter, parse_session_and_note
from .mission_gateway import MissionGateway

try:
    from zoneinfo import ZoneInfo
except ModuleNotFoundError:  # pragma: no cover
    ZoneInfo = None  # type: ignore[assignment]


console = Console()


class ChatInterfaceNode(Node):
    """CLI chat UI that proxies user commands to the mission coordinator layer."""

    def __init__(self, json_output: bool = False) -> None:
        super().__init__('chat_interface')

        self.declare_parameter('ui_title', 'Generalist BT Chat')
        self.declare_parameter('mission_coordinator_action', '/mission_coordinator/execute_tree')
        self.declare_parameter('status_service', '/mission_coordinator/status')
        self.declare_parameter('control_service', '/mission_coordinator/control')
        self.declare_parameter('status_topic', '/mission_coordinator/status_text')
        self.declare_parameter('subtree_topic', '/mission_coordinator/active_subtree')
        self.declare_parameter('pending_plan_topic', '/mission_coordinator/pending_plan')
        self.declare_parameter(
            'operator_decision_service', '/mission_coordinator/operator_decision'
        )
        self.declare_parameter('diagnostics_topics', [])
        self.declare_parameter('transcript_directory', '~/.generalist_bt/chat_logs')
        self.declare_parameter('refresh_period', 0.1)
        self.declare_parameter('timestamp_timezone', 'UTC')
        self.declare_parameter('demo_mode', False)

        self.ui_title = self.get_parameter('ui_title').get_parameter_value().string_value
        self.mission_coordinator_action = self.get_parameter('mission_coordinator_action').value
        self.status_service = self.get_parameter('status_service').value
        self.control_service = self.get_parameter('control_service').value
        self.status_topic = self.get_parameter('status_topic').value
        self.subtree_topic = self.get_parameter('subtree_topic').value
        self.pending_plan_topic = self.get_parameter('pending_plan_topic').value
        self.operator_decision_service = self.get_parameter('operator_decision_service').value
        self.transcript_directory = Path(
            self.get_parameter('transcript_directory').value
        ).expanduser()
        self.refresh_period = float(self.get_parameter('refresh_period').value)
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.json_output = bool(json_output)

        tz_name = self.get_parameter('timestamp_timezone').value or 'UTC'
        self._tzinfo = self._resolve_timezone(tz_name)
        self._shutdown_event: asyncio.Event = asyncio.Event()
        self._monitor_task: Optional[asyncio.Task] = None
        self._active_mission_task: Optional[asyncio.Task] = None
        self._active_mission_session_id = ''

        self.status_snapshot = 'Idle'
        self.active_subtree = 'unknown'
        self.pending_plan: Optional[Dict[str, Any]] = None
        self._last_result: Dict[str, Any] = {}

        self._log_handle = self._prepare_transcript()

        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        if self.subtree_topic:
            self.create_subscription(String, self.subtree_topic, self._subtree_callback, 10)
        if self.pending_plan_topic:
            self.create_subscription(
                String, self.pending_plan_topic, self._pending_plan_callback, 10
            )

        diag_topics = self.get_parameter('diagnostics_topics').value
        if isinstance(diag_topics, list):
            for topic in diag_topics:
                if isinstance(topic, str) and topic:
                    self.create_subscription(
                        String,
                        topic,
                        partial(self._diagnostics_callback, topic),
                        10,
                    )

        self.gateway = MissionGateway(
            self,
            mission_action_name=self.mission_coordinator_action,
            status_service_name=self.status_service,
            control_service_name=self.control_service,
            operator_decision_service=self.operator_decision_service,
        )

        self._command_router = CommandRouter(
            {
                'status': self._cmd_status,
                'help': self._cmd_help,
                'approve': self._cmd_approve,
                'cancel': self._cmd_cancel,
                'abort': self._cmd_abort,
                'monitor': self._cmd_monitor,
                'quit': self._cmd_quit,
                'exit': self._cmd_quit,
            }
        )

        self._emit_banner()
        self._emit_system_line('UI ready. Type your instruction or /help for commands.')

    @property
    def shutdown_event(self) -> asyncio.Event:
        return self._shutdown_event

    def close(self) -> None:
        if self._monitor_task and not self._monitor_task.done():
            self._monitor_task.cancel()
        if self._active_mission_task and not self._active_mission_task.done():
            self._active_mission_task.cancel()
        if self._log_handle:
            self._log_handle.close()
            self._log_handle = None  # type: ignore[assignment]

    def _prepare_transcript(self):
        self.transcript_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        log_path = self.transcript_directory / f'chat_{timestamp}.jsonl'
        handle = log_path.open('a', encoding='utf-8')
        self._emit_system_line(f'Logging transcript to {log_path}')
        return handle

    def _resolve_timezone(self, tz_name: str):
        if ZoneInfo:
            try:
                return ZoneInfo(tz_name)
            except Exception:  # pragma: no cover
                pass
        return timezone.utc

    def _write_transcript(self, role: str, content: str, metadata: Optional[dict] = None) -> None:
        entry = {
            'ts': datetime.now(self._tzinfo).isoformat(),
            'role': role,
            'content': content,
        }
        if metadata:
            entry['meta'] = metadata
        if self._log_handle:
            self._log_handle.write(json.dumps(entry) + '\n')
            self._log_handle.flush()

    async def handle_user_input(self, text: str, auto_execute: bool = False) -> None:
        stripped = text.strip()
        if not stripped:
            return
        if stripped.startswith('/'):
            handled = await self._command_router.dispatch(stripped)
            if not handled:
                self._emit_system_line(f'Unknown command: {stripped.split(" ", 1)[0]}')
                self._last_result = {'ok': False, 'error': f'Unknown command: {stripped}'}
            return
        await self._forward_to_mission_coordinator(stripped, auto_execute=auto_execute)

    async def _forward_to_mission_coordinator(
        self, message: str, auto_execute: bool = False
    ) -> None:
        self._emit_user_line(message)
        self._write_transcript(
            'user', message, {'target_action': self.mission_coordinator_action}
        )

        if self._active_mission_task and not self._active_mission_task.done():
            msg = (
                f'Mission already active (session={self._active_mission_session_id}). '
                'Use /status, /approve, /cancel, or /abort.'
            )
            self._emit_system_line(msg)
            self._last_result = {
                'ok': False,
                'error': msg,
                'session_id': self._active_mission_session_id,
            }
            return

        if self.demo_mode:
            await self._emit_demo_response(message)
            return

        self._active_mission_task = asyncio.create_task(
            self._run_mission(message, auto_execute=auto_execute)
        )

    async def _run_mission(self, message: str, auto_execute: bool = False) -> None:
        goal_handle, session_id = await self.gateway.send_mission_goal(
            message,
            auto_execute=auto_execute,
            feedback_callback=self._mission_feedback,
            session_prefix='chatui',
        )
        if not session_id:
            msg = f'Mission coordinator action {self.mission_coordinator_action} unavailable.'
            self._emit_system_line(msg)
            self._write_transcript('system', msg)
            self._last_result = {'ok': False, 'error': msg}
            return

        self._active_mission_session_id = session_id
        dispatch_message = (
            f'Dispatching mission to {self.mission_coordinator_action} '
            f'(session={session_id})'
        )
        self._emit_system_line(dispatch_message)
        self._write_transcript(
            'system',
            dispatch_message,
            {'action': self.mission_coordinator_action, 'session_id': session_id},
        )

        if goal_handle is None or not goal_handle.accepted:
            message = 'Mission coordinator rejected the request.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            self._last_result = {'ok': False, 'error': message, 'session_id': session_id}
            self._active_mission_session_id = ''
            return

        result = await self.gateway.await_mission_result(goal_handle)
        if result is None:
            message = 'Mission coordinator result unavailable.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            self._last_result = {'ok': False, 'error': message, 'session_id': session_id}
            self._active_mission_session_id = ''
            return

        if self._has_valid_pending_plan():
            pending = self.pending_plan or {}
            if str(pending.get('session_id', '')).strip() == session_id:
                self.pending_plan = None

        outcome = {
            'ok': bool(result.result.accepted),
            'accepted': bool(result.result.accepted),
            'message': result.result.outcome_message,
            'session_id': session_id,
        }
        self._last_result = outcome
        self._emit_system_line(
            f"Mission result: accepted={outcome['accepted']}, detail='{outcome['message']}'"
        )
        self._write_transcript('result', outcome['message'], outcome)
        self._active_mission_session_id = ''

    def _mission_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        detail = f'{feedback.stage}: {feedback.detail}'
        self._emit_status_line(detail)
        self._write_transcript('feedback', detail)

    async def _cmd_status(self, _: str) -> None:
        active = self._active_mission_session_id if self._active_mission_session_id else ''
        ok, response, err = await self.gateway.query_state(active)
        if not ok or response is None:
            self._emit_system_line(err or 'Status unavailable.')
            self._last_result = {'ok': False, 'error': err or 'Status unavailable.'}
            return

        pending_plan: Dict[str, Any] = {}
        try:
            pending_plan = json.loads(response.pending_plan_json or '{}')
        except Exception:
            pending_plan = {}
        self.pending_plan = self._normalize_pending_plan(pending_plan)
        self.status_snapshot = response.status_text or self.status_snapshot
        self.active_subtree = response.active_subtree or self.active_subtree

        message = (
            f"state={response.lifecycle_state}, status='{response.status_text}', "
            'active_subtree='
            f"'{response.active_subtree}', session='{response.active_session_id or '-'}'"
        )
        self._emit_system_line(message)
        if self._has_valid_pending_plan():
            self._emit_pending_plan_details()
        self._write_transcript('command', '/status', {'state': response.lifecycle_state})
        self._last_result = {
            'ok': True,
            'state': response.lifecycle_state,
            'status': response.status_text,
            'active_subtree': response.active_subtree,
            'session_id': response.active_session_id,
        }

    async def _cmd_approve(self, payload: str) -> None:
        session_id, feedback = parse_session_and_note(payload)
        if not session_id and self._has_valid_pending_plan():
            session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
        if not session_id:
            message = 'No pending plan to approve. Provide session_id or wait for pending plan.'
            self._emit_system_line(message)
            self._last_result = {'ok': False, 'error': message}
            return

        ok, message = await self.gateway.send_control(
            MissionControl.Request.APPROVE,
            session_id=session_id,
            note=feedback,
        )
        self._emit_system_line(
            message or ('Execution approved.' if ok else 'Approve failed.')
        )
        self._last_result = {'ok': ok, 'message': message, 'session_id': session_id}
        if ok:
            self.pending_plan = None

    async def _cmd_cancel(self, payload: str) -> None:
        session_id, feedback = parse_session_and_note(payload)
        if not session_id and self._has_valid_pending_plan():
            session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
        if not session_id:
            message = 'No pending plan to cancel. Provide session_id or wait for pending plan.'
            self._emit_system_line(message)
            self._last_result = {'ok': False, 'error': message}
            return

        ok, message = await self.gateway.send_control(
            MissionControl.Request.REJECT,
            session_id=session_id,
            note=feedback,
        )
        self._emit_system_line(
            message or ('Execution rejected.' if ok else 'Cancel failed.')
        )
        self._last_result = {'ok': ok, 'message': message, 'session_id': session_id}
        if ok:
            self.pending_plan = None

    async def _cmd_abort(self, payload: str) -> None:
        session_id, note = parse_session_and_note(payload)
        if not session_id:
            session_id = self._active_mission_session_id
        ok, message = await self.gateway.send_control(
            MissionControl.Request.ABORT,
            session_id=session_id,
            note=note,
        )
        self._emit_system_line(message or ('Abort requested.' if ok else 'Abort failed.'))
        self._last_result = {'ok': ok, 'message': message, 'session_id': session_id}

    async def _cmd_quit(self, _: str) -> None:
        if self._monitor_task and not self._monitor_task.done():
            self._monitor_task.cancel()
            self._monitor_task = None
        self._emit_system_line('Shutdown requested.')
        self._shutdown_event.set()

    async def _cmd_help(self, _: str) -> None:
        lines = [
            '/status             - Show mission coordinator state snapshot',
            '/status             - Includes pending payload details when approval is needed',
            '/approve [id] [msg] - Approve pending operator plan',
            '/cancel [id] [msg]  - Reject pending operator plan',
            '/abort [id] [msg]   - Abort active mission',
            '/monitor [seconds]  - Start status monitor (or /monitor off)',
            '/quit               - Exit the chat UI',
        ]
        for line in lines:
            self._emit_system_line(line)

    async def _cmd_monitor(self, payload: str) -> None:
        arg = payload.strip().lower()
        if arg in {'off', 'stop'}:
            if self._monitor_task and not self._monitor_task.done():
                self._monitor_task.cancel()
                self._monitor_task = None
                self._emit_system_line('Status monitor stopped.')
                self._last_result = {'ok': True, 'monitoring': False}
            else:
                self._emit_system_line('Status monitor is not running.')
                self._last_result = {'ok': True, 'monitoring': False}
            return

        if self._monitor_task and not self._monitor_task.done():
            self._emit_system_line(
                'Status monitor already running. Use /monitor off to stop it.'
            )
            self._last_result = {'ok': False, 'error': 'monitor already running'}
            return

        interval = 1.0
        if arg:
            try:
                interval = float(arg)
            except ValueError:
                self._emit_system_line('Usage: /monitor [seconds|off]')
                self._last_result = {'ok': False, 'error': 'invalid monitor interval'}
                return
            if interval <= 0.0:
                self._emit_system_line('Monitor interval must be > 0 seconds.')
                self._last_result = {'ok': False, 'error': 'invalid monitor interval'}
                return

        self._monitor_task = asyncio.create_task(self._monitor_loop(interval))
        self._emit_system_line(f'Status monitor started (interval={interval:.1f}s).')
        self._last_result = {'ok': True, 'monitoring': True, 'interval_sec': interval}

    async def _monitor_loop(self, interval_sec: float) -> None:
        try:
            while rclpy.ok() and not self.shutdown_event.is_set():
                snapshot = self._state_snapshot()
                if self.json_output:
                    console.print_json(data=json.dumps(snapshot, ensure_ascii=False))
                else:
                    pending = snapshot.get('pending_plan')
                    pending_id = ''
                    if isinstance(pending, dict):
                        pending_id = str(pending.get('session_id', ''))
                    self._emit_system_line(
                        f"monitor status='{snapshot['status']}' "
                        f"subtree='{snapshot['active_subtree']}' "
                        f"pending_session='{pending_id or '-'}'"
                    )
                await asyncio.sleep(interval_sec)
        except asyncio.CancelledError:
            pass

    def _status_callback(self, msg: String) -> None:
        self.status_snapshot = msg.data
        self._emit_status_line(msg.data)
        self._write_transcript('status', msg.data, {'topic': self.status_topic})

    def _subtree_callback(self, msg: String) -> None:
        self.active_subtree = msg.data

    def _pending_plan_callback(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            self.pending_plan = self._normalize_pending_plan(parsed)
        except Exception as exc:
            self.get_logger().warning(f'Failed to parse pending_plan JSON: {exc}')
            self.pending_plan = {'raw': msg.data}

        if self._has_valid_pending_plan():
            self._emit_pending_plan_details()
            self._write_transcript(
                'pending_plan',
                'Operator approval needed',
                {'session_id': str(self.pending_plan.get('session_id', ''))},
            )

    def _diagnostics_callback(self, topic_name: str, msg: String) -> None:
        self._emit_diag_line(topic_name, msg.data)
        self._write_transcript('diagnostics', msg.data, {'topic': topic_name})

    def _normalize_pending_plan(self, plan: object) -> Optional[Dict[str, Any]]:
        if not isinstance(plan, dict):
            return None
        normalized = dict(plan)
        if 'reasoning' in normalized and 'summary' not in normalized:
            normalized['summary'] = normalized['reasoning']
        return normalized

    def _has_valid_pending_plan(self) -> bool:
        plan = self.pending_plan
        return isinstance(plan, dict) and bool(str(plan.get('session_id', '')).strip())

    def _emit_pending_plan_details(self) -> None:
        if not self._has_valid_pending_plan():
            return
        plan = self.pending_plan or {}
        session_id = str(plan.get('session_id', '')).strip()
        tree_id = str(plan.get('tree_id', 'unknown')).strip() or 'unknown'
        mission = str(plan.get('mission', '')).strip()
        summary = str(plan.get('summary', '') or plan.get('reasoning', '')).strip()
        self._emit_system_line(
            f'Operator approval required for session {session_id} (tree={tree_id}).'
        )
        if mission:
            self._emit_system_line(f'Mission: {mission}')
        if summary:
            self._emit_system_line(f'Summary: {summary}')
        payload_preview = self._pretty_payload(plan.get('payload_json', '{}'))
        self._emit_system_line(f'Payload:\n{payload_preview}')

    def _pretty_payload(self, raw_payload: object) -> str:
        if raw_payload in (None, ''):
            return '{}'
        if isinstance(raw_payload, dict):
            return json.dumps(raw_payload, indent=2, ensure_ascii=False)
        if isinstance(raw_payload, str):
            stripped = raw_payload.strip()
            if not stripped:
                return '{}'
            try:
                parsed = json.loads(stripped)
                return json.dumps(parsed, indent=2, ensure_ascii=False)
            except Exception:
                return stripped
        return str(raw_payload)

    async def _emit_demo_response(self, message: str) -> None:
        await asyncio.sleep(0.1)
        response = (
            f'[DEMO] Would dispatch "{message}" to '
            f'{self.mission_coordinator_action} and keep chat unlocked.'
        )
        self._emit_system_line(response)
        self._write_transcript(
            'demo',
            response,
            {'mission_coordinator_action': self.mission_coordinator_action},
        )
        self._last_result = {'ok': True, 'demo': True, 'message': response}

    def _emit_banner(self) -> None:
        if not self.json_output:
            console.rule(f'[bold green]{self.ui_title}')

    def _emit_user_line(self, text: str) -> None:
        if self.json_output:
            return
        console.print(f'[cyan]You[/]: {text}')

    def _emit_system_line(self, text: str) -> None:
        if self.json_output:
            return
        console.print(f'[yellow]system[/]: {text}')

    def _emit_status_line(self, text: str) -> None:
        if self.json_output:
            return
        console.print(f'[green]status[/]: {text}')

    def _emit_diag_line(self, topic: str, text: str) -> None:
        if self.json_output:
            return
        console.print(f'[magenta]{topic}[/]: {text}')

    def _state_snapshot(self) -> Dict[str, Any]:
        return {
            'status': self.status_snapshot,
            'active_subtree': self.active_subtree,
            'pending_plan': self.pending_plan,
            'last_result': self._last_result,
            'active_session_id': self._active_mission_session_id,
        }


async def _spin_node(node: ChatInterfaceNode, executor: SingleThreadedExecutor) -> None:
    try:
        while rclpy.ok() and not node.shutdown_event.is_set():
            await asyncio.to_thread(executor.spin_once, timeout_sec=node.refresh_period)
    except (asyncio.CancelledError, KeyboardInterrupt):  # pragma: no cover
        pass


async def _cli_loop(node: ChatInterfaceNode) -> None:
    while rclpy.ok() and not node.shutdown_event.is_set():
        try:
            text = await asyncio.to_thread(input, '> ')
        except (EOFError, KeyboardInterrupt):  # pragma: no cover
            await node._cmd_quit('')
            break
        await node.handle_user_input(text)


async def _run_one_shot(node: ChatInterfaceNode, cli_args: argparse.Namespace) -> None:
    if cli_args.command:
        command_text = cli_args.command.strip()
        if not command_text.startswith('/'):
            command_text = f'/{command_text}'
        await node.handle_user_input(command_text)

    if cli_args.mission:
        await node.handle_user_input(cli_args.mission, auto_execute=cli_args.auto_execute)

    if cli_args.watch:
        await node._cmd_monitor('1.0')
        try:
            while rclpy.ok() and not node.shutdown_event.is_set():
                await asyncio.sleep(0.2)
        except KeyboardInterrupt:  # pragma: no cover
            await node._cmd_quit('')
        return

    if cli_args.json:
        console.print_json(data=json.dumps(node._state_snapshot(), ensure_ascii=False))
    await node._cmd_quit('')


def _parse_cli_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description='Generalist BT terminal chat UI')
    parser.add_argument('--mission', type=str, help='One-shot natural language mission text')
    parser.add_argument(
        '--auto-execute',
        action='store_true',
        help='Set auto_execute=true in mission context JSON for one-shot mission dispatch',
    )
    parser.add_argument(
        '--command',
        type=str,
        help='One-shot slash command (e.g. status, approve)',
    )
    parser.add_argument(
        '--watch',
        action='store_true',
        help='Keep running and continuously monitor status after one-shot execution',
    )
    parser.add_argument('--json', action='store_true', help='Emit one-shot output as JSON')
    return parser.parse_known_args(argv)


async def main_async(cli_args: argparse.Namespace, ros_args: list[str]) -> None:
    rclpy.init(args=ros_args)
    node = ChatInterfaceNode(json_output=bool(cli_args.json))
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    one_shot = bool(cli_args.mission or cli_args.command or cli_args.watch)

    try:
        if one_shot:
            spin_task = asyncio.create_task(_spin_node(node, executor))
            try:
                await _run_one_shot(node, cli_args)
            finally:
                node.shutdown_event.set()
                await spin_task
        else:
            await asyncio.gather(_spin_node(node, executor), _cli_loop(node))
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    argv = list(args) if args is not None else sys.argv[1:]
    cli_args, ros_args = _parse_cli_args(argv)
    asyncio.run(main_async(cli_args, ros_args))


if __name__ == '__main__':  # pragma: no cover
    main()
