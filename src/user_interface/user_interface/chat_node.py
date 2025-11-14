import asyncio
import json
from datetime import datetime, timezone
from functools import partial
from pathlib import Path
from typing import Awaitable, Callable, Dict, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rich.console import Console
from std_msgs.msg import String

try:
    from zoneinfo import ZoneInfo
except ModuleNotFoundError:  # pragma: no cover
    ZoneInfo = None  # type: ignore[assignment]


console = Console()


class ChatInterfaceNode(Node):
    """Minimal CLI chat UI that proxies user commands to the mission coordinator layer."""

    def __init__(self) -> None:
        super().__init__('chat_interface')

        self.declare_parameter('ui_title', 'Generalist BT Chat')
        self.declare_parameter('mission_coordinator_action', '/mission_coordinator/execute_tree')
        self.declare_parameter(
            'mission_coordinator_namespace', '/mission_coordinator'
        )
        self.declare_parameter('status_service', '/mission_coordinator/status')
        self.declare_parameter('regen_service', '/mission_coordinator/request_regen')
        self.declare_parameter('llm_prompt_service', '/llm_interface/chat_complete')
        self.declare_parameter('status_topic', '/mission_coordinator/status_text')
        self.declare_parameter('diagnostics_topics', [])
        self.declare_parameter('transcript_directory', '~/.generalist_bt/chat_logs')
        self.declare_parameter('enable_langchain_proxy', False)
        self.declare_parameter('refresh_period', 0.1)
        self.declare_parameter('timestamp_timezone', 'UTC')
        self.declare_parameter('demo_mode', True)

        self.ui_title = self.get_parameter('ui_title').get_parameter_value().string_value
        self.mission_coordinator_action = self.get_parameter(
            'mission_coordinator_action'
        ).value
        self.status_service = self.get_parameter('status_service').value
        self.regen_service = self.get_parameter('regen_service').value
        self.llm_prompt_service = self.get_parameter('llm_prompt_service').value
        self.status_topic = self.get_parameter('status_topic').value
        self.transcript_directory = Path(self.get_parameter('transcript_directory').value).expanduser()
        self.enable_langchain_proxy = bool(self.get_parameter('enable_langchain_proxy').value)
        self.refresh_period = float(self.get_parameter('refresh_period').value)
        self.demo_mode = bool(self.get_parameter('demo_mode').value)

        tz_name = self.get_parameter('timestamp_timezone').value or 'UTC'
        self._tzinfo = self._resolve_timezone(tz_name)
        self._shutdown_event: asyncio.Event = asyncio.Event()

        self._log_handle = self._prepare_transcript()

        # Subscriptions for mission coordinator status + diagnostics.
        self.create_subscription(String, self.status_topic, self._status_callback, 10)
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

        self._command_table: Dict[str, Callable[[str], Awaitable[None]]] = {
            'status': self._cmd_status,
            'regen': self._cmd_regen,
            'quit': self._cmd_quit,
            'exit': self._cmd_quit,
            'help': self._cmd_help,
            'llm': self._cmd_llm_prompt,
        }

        self._emit_banner()
        self._emit_system_line('UI ready. Type your instruction or /help for commands.')

    # region Lifecycle helpers
    @property
    def shutdown_event(self) -> asyncio.Event:
        return self._shutdown_event

    def close(self) -> None:
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
    async def handle_user_input(self, text: str) -> None:
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
        else:
            await self._forward_to_mission_coordinator(stripped)

    async def _forward_to_mission_coordinator(self, message: str) -> None:
        self._emit_user_line(message)
        self._write_transcript(
            'user', message, {'target_action': self.mission_coordinator_action}
        )
        if self.demo_mode:
            await self._emit_demo_response(message)
        else:
            self._emit_system_line(
                'Ready to send message to mission coordinator action '
                f'{self.mission_coordinator_action}'
            )

    # endregion

    # region Command handlers
    async def _cmd_status(self, _: str) -> None:
        self._emit_system_line(f'Would call status service: {self.status_service}')
        self._write_transcript('command', '/status', {'service': self.status_service})

    async def _cmd_regen(self, _: str) -> None:
        self._emit_system_line(f'Would trigger regeneration via {self.regen_service}')
        self._write_transcript('command', '/regen', {'service': self.regen_service})

    async def _cmd_quit(self, _: str) -> None:
        self._emit_system_line('Shutdown requested.')
        self._shutdown_event.set()

    async def _cmd_help(self, _: str) -> None:
        lines = [
            '/status  - Request mission coordinator status update',
            '/regen   - Ask mission coordinator to regenerate BT subtree',
            '/llm msg - Forward note to llm_interface prompt stream',
            '/quit    - Exit the chat UI',
        ]
        for line in lines:
            self._emit_system_line(line)

    async def _cmd_llm_prompt(self, payload: str) -> None:
        content = payload.strip()
        if not content:
            self._emit_system_line('Usage: /llm <message>')
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

    # endregion

    # region Subscribers + demo cues
    def _status_callback(self, msg: String) -> None:
        self._emit_status_line(msg.data)
        self._write_transcript('status', msg.data, {'topic': self.status_topic})

    def _diagnostics_callback(self, topic_name: str, msg: String) -> None:
        self._emit_diag_line(topic_name, msg.data)
        self._write_transcript('diagnostics', msg.data, {'topic': topic_name})

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

    # endregion

    # region Console helpers
    def _emit_banner(self) -> None:
        console.rule(f'[bold green]{self.ui_title}')

    def _emit_user_line(self, text: str) -> None:
        console.print(f'[cyan]You[/]: {text}')

    def _emit_system_line(self, text: str) -> None:
        console.print(f'[yellow]system[/]: {text}')

    def _emit_status_line(self, text: str) -> None:
        console.print(f'[green]status[/]: {text}')

    def _emit_diag_line(self, topic: str, text: str) -> None:
        console.print(f'[magenta]{topic}[/]: {text}')

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


async def main_async(args=None) -> None:
    rclpy.init(args=args)
    node = ChatInterfaceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        await asyncio.gather(_spin_node(node, executor), _cli_loop(node))
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    asyncio.run(main_async(args=args))


if __name__ == '__main__':  # pragma: no cover
    main()
