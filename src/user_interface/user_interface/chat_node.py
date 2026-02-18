import argparse
import asyncio
import json
import sys
from datetime import datetime, timezone
from functools import partial
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, Optional

import rclpy
from gen_bt_interfaces.action import MissionCommand
from gen_bt_interfaces.srv import OperatorDecision
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rich.console import Console
from std_msgs.msg import String

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
        self.declare_parameter('mission_coordinator_namespace', '/mission_coordinator')
        self.declare_parameter('status_service', '/mission_coordinator/status')
        self.declare_parameter('regen_service', '/mission_coordinator/request_regen')
        self.declare_parameter('llm_prompt_service', '/llm_interface/chat_complete')
        self.declare_parameter('status_topic', '/mission_coordinator/status_text')
        self.declare_parameter('subtree_topic', '/mission_coordinator/active_subtree')
        self.declare_parameter('pending_plan_topic', '/mission_coordinator/pending_plan')
        self.declare_parameter('operator_decision_service', '/mission_coordinator/operator_decision')
        self.declare_parameter('diagnostics_topics', [])
        self.declare_parameter('transcript_directory', '~/.generalist_bt/chat_logs')
        self.declare_parameter('enable_langchain_proxy', False)
        self.declare_parameter('refresh_period', 0.1)
        self.declare_parameter('timestamp_timezone', 'UTC')
        self.declare_parameter('demo_mode', False)

        self.ui_title = self.get_parameter('ui_title').get_parameter_value().string_value
        self.mission_coordinator_action = self.get_parameter('mission_coordinator_action').value
        self.mission_coordinator_namespace = self.get_parameter('mission_coordinator_namespace').value
        self.status_service = self.get_parameter('status_service').value
        self.regen_service = self.get_parameter('regen_service').value
        self.llm_prompt_service = self.get_parameter('llm_prompt_service').value
        self.status_topic = self.get_parameter('status_topic').value
        self.subtree_topic = self.get_parameter('subtree_topic').value
        self.pending_plan_topic = self.get_parameter('pending_plan_topic').value
        self.operator_decision_service = self.get_parameter('operator_decision_service').value
        self.transcript_directory = Path(self.get_parameter('transcript_directory').value).expanduser()
        self.enable_langchain_proxy = bool(self.get_parameter('enable_langchain_proxy').value)
        self.refresh_period = float(self.get_parameter('refresh_period').value)
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.json_output = bool(json_output)

        tz_name = self.get_parameter('timestamp_timezone').value or 'UTC'
        self._tzinfo = self._resolve_timezone(tz_name)
        self._shutdown_event: asyncio.Event = asyncio.Event()

        self.status_snapshot = 'Idle'
        self.active_subtree = 'unknown'
        self.pending_plan: Optional[Dict[str, Any]] = None
        self._session_counter = 0
        self._monitor_task: Optional[asyncio.Task] = None
        self._last_result: Dict[str, Any] = {}

        self._log_handle = self._prepare_transcript()

        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        if self.subtree_topic:
            self.create_subscription(String, self.subtree_topic, self._subtree_callback, 10)
        if self.pending_plan_topic:
            self.create_subscription(String, self.pending_plan_topic, self._pending_plan_callback, 10)

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

        self._mission_action_client = ActionClient(
            self, MissionCommand, self.mission_coordinator_action
        )
        self._operator_decision_clients: Dict[str, Any] = {}
        self._parameter_client = AsyncParameterClient(
            self, self.mission_coordinator_namespace
        )

        self._command_table: Dict[str, Callable[[str], Awaitable[None]]] = {
            'status': self._cmd_status,
            'regen': self._cmd_regen,
            'quit': self._cmd_quit,
            'exit': self._cmd_quit,
            'help': self._cmd_help,
            'llm': self._cmd_llm_prompt,
            'approve': self._cmd_approve,
            'cancel': self._cmd_cancel,
            'trees': self._cmd_trees,
            'monitor': self._cmd_monitor,
        }

        self._emit_banner()
        self._emit_system_line('UI ready. Type your instruction or /help for commands.')

    # region Lifecycle helpers
    @property
    def shutdown_event(self) -> asyncio.Event:
        return self._shutdown_event

    def close(self) -> None:
        if self._monitor_task and not self._monitor_task.done():
            self._monitor_task.cancel()
        if self._log_handle:
            self._log_handle.close()
            self._log_handle = None  # type: ignore[assignment]

    # endregion

    # region Parameter + logging utilities
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
            except Exception:  # pragma: no cover - fallback to UTC
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

    # endregion

    # region Input handling
    async def handle_user_input(self, text: str, auto_execute: bool = False) -> None:
        stripped = text.strip()
        if not stripped:
            return
        if stripped.startswith('/'):
            cmd, *rest = stripped[1:].split(' ', 1)
            payload = rest[0] if rest else ''
            handler = self._command_table.get(cmd.lower())
            if handler:
                await handler(payload)
            else:
                self._emit_system_line(f'Unknown command: /{cmd}')
                self._last_result = {'ok': False, 'error': f'Unknown command /{cmd}'}
        else:
            await self._forward_to_mission_coordinator(stripped, auto_execute=auto_execute)

    async def _forward_to_mission_coordinator(self, message: str, auto_execute: bool = False) -> None:
        self._emit_user_line(message)
        self._write_transcript(
            'user', message, {'target_action': self.mission_coordinator_action}
        )
        if self.demo_mode:
            await self._emit_demo_response(message)
            return
        await self._dispatch_mission(message, auto_execute=auto_execute)

    async def _dispatch_mission(self, command_text: str, auto_execute: bool = False) -> None:
        if not await self._wait_for_mission_server():
            message = (
                f'Mission coordinator action {self.mission_coordinator_action} unavailable.'
            )
            self._emit_system_line(message)
            self._write_transcript('system', message)
            self._last_result = {'ok': False, 'error': message}
            return

        goal_msg = MissionCommand.Goal()
        goal_msg.command = command_text
        goal_msg.session_id = self._next_session_id()
        context = {'auto_execute': bool(auto_execute)} if auto_execute else {}
        goal_msg.context_json = json.dumps(context, ensure_ascii=False)

        dispatch_message = (
            f'Dispatching mission to {self.mission_coordinator_action} '
            f'(session={goal_msg.session_id})'
        )
        self._emit_system_line(dispatch_message)
        self._write_transcript(
            'system',
            dispatch_message,
            {'action': self.mission_coordinator_action, 'session_id': goal_msg.session_id},
        )

        send_future = self._mission_action_client.send_goal_async(
            goal_msg, feedback_callback=self._mission_feedback
        )
        goal_handle = await self._await_rclpy_future(send_future, 'send mission goal')
        if goal_handle is None or not goal_handle.accepted:
            message = 'Mission coordinator rejected the request.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            self._last_result = {'ok': False, 'error': message}
            return

        result_future = goal_handle.get_result_async()
        result = await self._await_rclpy_future(result_future, 'mission result')
        if result is None:
            message = 'Mission coordinator result unavailable.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            self._last_result = {'ok': False, 'error': message}
            return

        outcome = {
            'ok': bool(result.result.accepted),
            'accepted': bool(result.result.accepted),
            'message': result.result.outcome_message,
            'session_id': goal_msg.session_id,
        }
        if self._has_valid_pending_plan():
            pending = self.pending_plan or {}
            if str(pending.get('session_id', '')).strip() == goal_msg.session_id:
                self.pending_plan = None
        self._last_result = outcome
        self._emit_system_line(
            f"Mission result: accepted={outcome['accepted']}, detail='{outcome['message']}'"
        )
        self._write_transcript('result', outcome['message'], outcome)

    async def _wait_for_mission_server(self) -> bool:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None, self._mission_action_client.wait_for_server, 1.0
        )

    async def _send_operator_decision(
        self,
        session_id: str,
        approve: bool,
        feedback: str = '',
        service_name: Optional[str] = None,
    ) -> tuple[bool, str]:
        if not session_id:
            message = 'No session_id provided for operator decision.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            return False, message

        decision_client, decision_service = self._operator_client_for(service_name)
        loop = asyncio.get_running_loop()
        available = await loop.run_in_executor(
            None, decision_client.wait_for_service, 1.0
        )
        if not available:
            message = (
                f'Operator decision service {decision_service} unavailable.'
            )
            self._emit_system_line(message)
            self._write_transcript('system', message)
            return False, message

        request = OperatorDecision.Request()
        request.session_id = session_id
        request.approve = bool(approve)
        request.feedback = feedback or ''

        future = decision_client.call_async(request)
        result = await self._await_rclpy_future(future, 'operator decision')
        if result is None:
            message = 'Operator decision result unavailable.'
            self._emit_system_line(message)
            self._write_transcript('system', message)
            return False, message

        message = result.message or 'Operator decision applied.'
        if not result.accepted:
            if message.startswith('No pending plan for session'):
                self.pending_plan = None
            self._emit_system_line(message)
            self._write_transcript('system', message, {'accepted': False})
            return False, message

        self.pending_plan = None
        self._emit_system_line(message)
        self._write_transcript('system', message, {'accepted': True})
        return True, message

    def _operator_client_for(self, service_name: Optional[str]):
        target = (service_name or self.operator_decision_service or '').strip()
        if not target:
            target = '/mission_coordinator/operator_decision'
        client = self._operator_decision_clients.get(target)
        if client is None:
            client = self.create_client(OperatorDecision, target)
            self._operator_decision_clients[target] = client
        return client, target

    async def _list_available_trees(self) -> list[tuple[str, str]]:
        loop = asyncio.get_running_loop()
        ready = await loop.run_in_executor(None, self._parameter_client.wait_for_services, 1.0)
        if not ready:
            raise RuntimeError(
                f'Could not reach parameter services for {self.mission_coordinator_namespace}.'
            )

        future = self._parameter_client.get_parameters(['known_trees'])
        result = await self._await_rclpy_future(future, 'get known_trees parameter')
        if not result:
            return []

        entries: list[str] = []
        candidates = result if isinstance(result, list) else getattr(result, 'values', [])
        if candidates:
            first = candidates[0]
            if isinstance(first, list):
                entries = [item for item in first if isinstance(item, str)]
            elif isinstance(first, str):
                entries = [first]
            elif hasattr(first, 'string_array_value'):
                entries = list(getattr(first, 'string_array_value', []))
            else:
                value = getattr(first, 'value', None)
                if isinstance(value, list):
                    entries = [item for item in value if isinstance(item, str)]
                elif value is not None and hasattr(value, 'string_array_value'):
                    entries = list(value.string_array_value)

        trees: list[tuple[str, str]] = []
        for entry in entries:
            if not isinstance(entry, str):
                continue
            if '::' in entry:
                tree_id, description = entry.split('::', 1)
            else:
                tree_id, description = entry, ''
            tree_id = tree_id.strip()
            if tree_id:
                trees.append((tree_id, description.strip()))
        return trees

    async def _await_rclpy_future(self, future, context: str):
        while not future.done() and rclpy.ok() and not self.shutdown_event.is_set():
            await asyncio.sleep(0.05)

        if future.cancelled():
            self.get_logger().warning(f'Future canceled while waiting for {context}')
            return None
        if future.exception() is not None:
            self.get_logger().error(f'Future failed while waiting for {context}: {future.exception()}')
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to read future result for {context}: {exc}')
            return None

    def _next_session_id(self) -> str:
        self._session_counter += 1
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        return f'chatui-{timestamp}-{self._session_counter}'

    def _mission_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        detail = f'{feedback.stage}: {feedback.detail}'
        self._emit_status_line(detail)
        self._write_transcript('feedback', detail)

    # endregion

    # region Command handlers
    async def _cmd_status(self, _: str) -> None:
        self._emit_system_line(
            f"status='{self.status_snapshot}', active_subtree='{self.active_subtree}'"
        )
        self._write_transcript('command', '/status')
        self._last_result = {
            'ok': True,
            'status': self.status_snapshot,
            'active_subtree': self.active_subtree,
        }

    async def _cmd_regen(self, _: str) -> None:
        self._emit_system_line(f'Would trigger regeneration via {self.regen_service}')
        self._write_transcript('command', '/regen', {'service': self.regen_service})
        self._last_result = {'ok': True, 'message': 'regen requested (placeholder)'}

    async def _cmd_quit(self, _: str) -> None:
        if self._monitor_task and not self._monitor_task.done():
            self._monitor_task.cancel()
            self._monitor_task = None
        self._emit_system_line('Shutdown requested.')
        self._shutdown_event.set()

    async def _cmd_help(self, _: str) -> None:
        lines = [
            '/status             - Show current status snapshot',
            '/trees              - List known behavior trees from mission coordinator',
            '/approve [feedback] - Approve pending operator plan',
            '/cancel [feedback]  - Reject pending operator plan',
            '/monitor [seconds]  - Start status monitor (or /monitor off)',
            '/regen              - Ask mission coordinator to regenerate BT subtree',
            '/llm msg            - Forward note to llm_interface prompt stream',
            '/quit               - Exit the chat UI',
        ]
        for line in lines:
            self._emit_system_line(line)

    async def _cmd_llm_prompt(self, payload: str) -> None:
        content = payload.strip()
        if not content:
            self._emit_system_line('Usage: /llm <message>')
            self._last_result = {'ok': False, 'error': 'missing /llm message'}
            return
        self._emit_system_line(f'(llm_interface) Would call {self.llm_prompt_service} with payload.')
        self._write_transcript(
            'command',
            '/llm',
            {
                'service': self.llm_prompt_service,
                'payload_preview': content[:120],
            },
        )
        self._last_result = {'ok': True, 'message': 'llm prompt placeholder'}

    async def _cmd_approve(self, payload: str) -> None:
        if not self._has_valid_pending_plan():
            message = 'No pending plan to approve.'
            self._emit_system_line(message)
            self._last_result = {'ok': False, 'error': message}
            return
        plan = self.pending_plan or {}
        ok, message = await self._send_operator_decision(
            str(plan.get('session_id', '')),
            True,
            payload.strip(),
            service_name=str(
                plan.get('operator_decision_service') or self.operator_decision_service
            ),
        )
        self._last_result = {'ok': ok, 'message': message}

    async def _cmd_cancel(self, payload: str) -> None:
        if not self._has_valid_pending_plan():
            message = 'No pending plan to cancel.'
            self._emit_system_line(message)
            self._last_result = {'ok': False, 'error': message}
            return
        plan = self.pending_plan or {}
        ok, message = await self._send_operator_decision(
            str(plan.get('session_id', '')),
            False,
            payload.strip(),
            service_name=str(
                plan.get('operator_decision_service') or self.operator_decision_service
            ),
        )
        self._last_result = {'ok': ok, 'message': message}

    async def _cmd_trees(self, _: str) -> None:
        try:
            trees = await self._list_available_trees()
        except Exception as exc:
            message = f'Failed to list trees: {exc}'
            self._emit_system_line(message)
            self.get_logger().error(message)
            self._last_result = {'ok': False, 'error': message}
            return

        if not trees:
            self._emit_system_line('No behavior trees available from mission coordinator.')
            self._last_result = {'ok': True, 'trees': []}
            return

        self._emit_system_line('Available behavior trees:')
        for tree_id, description in trees:
            if description:
                self._emit_system_line(f'- {tree_id}: {description}')
            else:
                self._emit_system_line(f'- {tree_id}')
        self._last_result = {
            'ok': True,
            'trees': [{'id': tree_id, 'description': description} for tree_id, description in trees],
        }

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
            self._emit_system_line('Status monitor already running. Use /monitor off to stop it.')
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
                        f"monitor status='{snapshot['status']}' subtree='{snapshot['active_subtree']}' "
                        f"pending_session='{pending_id or '-'}'"
                    )
                await asyncio.sleep(interval_sec)
        except asyncio.CancelledError:
            pass

    # endregion

    # region Subscribers + demo cues
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
            session_id = str(self.pending_plan.get('session_id', ''))
            summary = str(self.pending_plan.get('summary', '')).strip()
            if summary:
                self._emit_system_line(
                    f'Operator approval required for session {session_id}: {summary}'
                )
            else:
                self._emit_system_line(
                    f'Operator approval required for session {session_id} '
                    '(use /approve or /cancel).'
                )
            self._write_transcript(
                'pending_plan',
                'Operator approval needed',
                {'session_id': session_id},
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

    async def _emit_demo_response(self, message: str) -> None:
        await asyncio.sleep(0.1)
        response = (
            f'[DEMO] Would dispatch "{message}" to {self.mission_coordinator_action} and await feedback.'
        )
        self._emit_system_line(response)
        self._write_transcript(
            'demo',
            response,
            {'mission_coordinator_action': self.mission_coordinator_action},
        )
        self._last_result = {'ok': True, 'demo': True, 'message': response}

    # endregion

    # region Console helpers
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
        }

    # endregion


async def _spin_node(node: ChatInterfaceNode, executor: SingleThreadedExecutor) -> None:
    try:
        while rclpy.ok() and not node.shutdown_event.is_set():
            await asyncio.to_thread(executor.spin_once, timeout_sec=node.refresh_period)
    except (asyncio.CancelledError, KeyboardInterrupt):  # pragma: no cover - interactive
        pass


async def _cli_loop(node: ChatInterfaceNode) -> None:
    while rclpy.ok() and not node.shutdown_event.is_set():
        try:
            text = await asyncio.to_thread(input, '> ')
        except (EOFError, KeyboardInterrupt):  # pragma: no cover - interactive
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
        except KeyboardInterrupt:  # pragma: no cover - interactive
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
    parser.add_argument('--command', type=str, help='One-shot slash command (e.g. trees, status)')
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
