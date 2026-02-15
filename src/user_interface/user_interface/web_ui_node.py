import asyncio
import json
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Deque, Dict, List, Optional

import rclpy
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from gen_bt_interfaces.action import MissionCommand
from gen_bt_interfaces.srv import OperatorDecision
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import uvicorn

try:
    from zoneinfo import ZoneInfo
except ImportError:  # pragma: no cover - Python < 3.9
        ZoneInfo = None  # type: ignore[assignment]


TEMPLATES_DIR = Path(__file__).resolve().parent / 'templates'


class WebInterfaceNode(Node):
    """Web-based chat UI that exposes mission coordinator hooks via HTTP."""

    def __init__(self) -> None:
        super().__init__('chat_web_interface')

        self.declare_parameter('ui_title', 'Generalist BT Chat')
        self.declare_parameter(
            'mission_coordinator_action', '/mission_coordinator/execute_tree'
        )
        self.declare_parameter(
            'mission_coordinator_namespace', '/mission_coordinator'
        )
        self.declare_parameter('status_service', '/mission_coordinator/status')
        self.declare_parameter('regen_service', '/mission_coordinator/request_regen')
        self.declare_parameter('llm_prompt_service', '/llm_interface/chat_complete')
        self.declare_parameter('status_topic', '/mission_coordinator/status_text')
        self.declare_parameter('subtree_topic', '/mission_coordinator/active_subtree')
        self.declare_parameter('pending_plan_topic', '/mission_coordinator/pending_plan')
        self.declare_parameter('diagnostics_topics', [])
        self.declare_parameter('transcript_directory', '~/.generalist_bt/chat_logs')
        self.declare_parameter('enable_langchain_proxy', False)
        self.declare_parameter('refresh_period', 0.1)
        self.declare_parameter('timestamp_timezone', 'UTC')
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter(
            'operator_decision_service', '/mission_coordinator/operator_decision'
        )

        self.ui_title = self.get_parameter('ui_title').value
        self.mission_coordinator_action = self.get_parameter(
            'mission_coordinator_action'
        ).value
        self.status_service = self.get_parameter('status_service').value
        self.regen_service = self.get_parameter('regen_service').value
        self.llm_prompt_service = self.get_parameter('llm_prompt_service').value
        self.status_topic = self.get_parameter('status_topic').value
        self.subtree_topic = self.get_parameter('subtree_topic').value
        self.pending_plan_topic = self.get_parameter('pending_plan_topic').value
        self.transcript_directory = Path(self.get_parameter('transcript_directory').value).expanduser()
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        # log demo mode parameter
        self.get_logger().info(f'Demo mode is set to {self.demo_mode}')
        self.refresh_period = float(self.get_parameter('refresh_period').value)
        self.web_host = self.get_parameter('web_host').value or '0.0.0.0'
        self.web_port = int(self.get_parameter('web_port').value)
        self.operator_decision_service = self.get_parameter(
            'operator_decision_service'
        ).value

        tz_name = self.get_parameter('timestamp_timezone').value or 'UTC'
        self._tzinfo = self._resolve_timezone(tz_name)
        self._shutdown_event: asyncio.Event = asyncio.Event()
        self._log_handle = self._prepare_transcript()

        self.status_snapshot = 'Idle'
        self.active_subtree = 'unknown'
        self.diagnostics: Dict[str, str] = {}
        self.messages: Deque[Dict[str, str]] = deque(maxlen=200)
        self.pending_plan: Optional[Dict] = None

        self.create_subscription(String, self.status_topic, self._status_callback, 10)
        if self.subtree_topic:
            self.create_subscription(String, self.subtree_topic, self._subtree_callback, 10)
        if self.pending_plan_topic:
            self.create_subscription(
                String, self.pending_plan_topic, self._pending_plan_callback, 10
            )

        diagnostics_topics = self.get_parameter('diagnostics_topics').value
        if isinstance(diagnostics_topics, list):
            for topic in diagnostics_topics:
                if isinstance(topic, str) and topic:
                    self.create_subscription(
                        String,
                        topic,
                        lambda msg, t=topic: self._diagnostics_callback(t, msg),
                        10,
                    )

        self._record_message('system', 'Web UI initialized.')

        self._mission_action_client = ActionClient(
            self, MissionCommand, self.mission_coordinator_action
        )
        self._session_counter = 0
        self._operator_decision_client = self.create_client(
            OperatorDecision, self.operator_decision_service
        )

    # region Utilities
    @property
    def shutdown_event(self) -> asyncio.Event:
        return self._shutdown_event

    def _resolve_timezone(self, tz_name: str):
        if ZoneInfo:
            try:
                return ZoneInfo(tz_name)
            except Exception:  # pragma: no cover
                pass
        return timezone.utc

    def _prepare_transcript(self):
        self.transcript_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        log_path = self.transcript_directory / f'web_chat_{timestamp}.jsonl'
        handle = log_path.open('a', encoding='utf-8')
        self.get_logger().info(f'Logging transcript to {log_path}')
        return handle

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

    def _record_message(self, role: str, content: str) -> None:
        ts = datetime.now(self._tzinfo).isoformat()
        self.messages.append({'role': role, 'content': content, 'ts': ts})
        self._write_transcript(role, content)

    async def handle_user_input(self, text: str, auto_execute: bool = False) -> None:
        stripped = text.strip()
        if not stripped:
            return
        if stripped.startswith('/'):
            await self._handle_command(stripped)
        else:
            self._record_message('user', stripped)
            if self.demo_mode:
                await self._demo_response(stripped)
            else:
                await self._dispatch_mission(stripped, auto_execute=auto_execute)

    async def _handle_command(self, text: str) -> None:
        cmd, *rest = text[1:].split(' ', 1)
        payload = rest[0] if rest else ''
        command = cmd.lower()
        if command == 'status':
            self._record_message('system', f'Would invoke {self.status_service}')
        elif command == 'regen':
            self._record_message('system', f'Would invoke {self.regen_service}')
        elif command == 'llm':
            if not payload.strip():
                self._record_message('system', 'Usage: /llm <message>')
            else:
                self._record_message(
                    'system',
                    f'(llm_interface) Would call {self.llm_prompt_service} with payload.',
                )
        else:
            self._record_message('system', f'Unknown command {text}')

    async def _demo_response(self, message: str) -> None:
        await asyncio.sleep(0.1)
        self._record_message(
            'demo',
            f'Pretend to dispatch "{message}" to {self.mission_coordinator_action} (demo mode).',
        )

    async def _dispatch_mission(self, command_text: str, auto_execute: bool = False) -> None:
        if not await self._wait_for_mission_server():
            self._record_message(
                'system',
                f'Mission coordinator action {self.mission_coordinator_action} unavailable.',
            )
            return

        goal_msg = MissionCommand.Goal()
        goal_msg.command = command_text
        goal_msg.session_id = self._next_session_id()
        context = {}
        if auto_execute:
            context['auto_execute'] = True
        goal_msg.context_json = json.dumps(context, ensure_ascii=False)

        self._record_message(
            'system',
            f'Dispatching mission to {self.mission_coordinator_action} (session={goal_msg.session_id})',
        )
        send_future = self._mission_action_client.send_goal_async(
            goal_msg, feedback_callback=self._mission_feedback
        )
        goal_handle = await self._await_rclpy_future(send_future)
        if goal_handle is None or not goal_handle.accepted:
            self._record_message('system', 'Mission coordinator rejected the request.')
            return

        result_future = goal_handle.get_result_async()
        result = await self._await_rclpy_future(result_future)
        if result is None:
            self._record_message('system', 'Mission coordinator result unavailable.')
            return
        self._record_message(
            'system',
            f"Mission result: accepted={result.result.accepted}, detail='{result.result.outcome_message}'",
        )

    async def _wait_for_mission_server(self) -> bool:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None, self._mission_action_client.wait_for_server, 1.0
        )

    async def _send_operator_decision(
        self, session_id: str, approve: bool, feedback: str = ''
    ) -> tuple[bool, str]:
        if not session_id:
            message = 'No session_id provided for operator decision.'
            self._record_message('system', message)
            return False, message

        loop = asyncio.get_running_loop()
        available = await loop.run_in_executor(
            None, self._operator_decision_client.wait_for_service, 1.0
        )
        if not available:
            message = (
                f'Operator decision service {self.operator_decision_service} unavailable.'
            )
            self._record_message('system', message)
            return False, message

        request = OperatorDecision.Request()
        request.session_id = session_id
        request.approve = bool(approve)
        request.feedback = feedback or ''
        future = self._operator_decision_client.call_async(request)
        result = await self._await_rclpy_future(future)
        if result is None:
            message = 'Operator decision result unavailable.'
            self._record_message('system', message)
            return False, message

        if not result.accepted:
            message = result.message or 'Operator decision not accepted.'
            self._record_message('system', message)
            return False, message

        message = result.message or 'Operator decision applied.'
        self._record_message('system', message)
        # Clear cached pending plan once a decision has been applied.
        self.pending_plan = None
        return True, message

    async def _await_rclpy_future(self, future):
        while not future.done():
            await asyncio.sleep(0.05)
        if future.cancelled():
            return None
        if future.exception():
            self.get_logger().error(f'Action future error: {future.exception()}')
            return None
        return future.result()

    def _next_session_id(self) -> str:
        self._session_counter += 1
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        return f'webui-{timestamp}-{self._session_counter}'

    def _mission_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._record_message(
            'feedback', f"{feedback.stage}: {feedback.detail}"
        )

    def close(self) -> None:
        if self._log_handle:
            self._log_handle.close()
            self._log_handle = None  # type: ignore[assignment]

    # endregion

    # region ROS Callbacks
    def _status_callback(self, msg: String) -> None:
        self.status_snapshot = msg.data

    def _subtree_callback(self, msg: String) -> None:
        self.active_subtree = msg.data

    def _diagnostics_callback(self, topic: str, msg: String) -> None:
        self.diagnostics[topic] = msg.data

    def _pending_plan_callback(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
            self.pending_plan = self._normalize_pending_plan(parsed)
        except Exception:
            self.pending_plan = {'raw': msg.data}

    def _normalize_pending_plan(self, plan: object) -> Optional[Dict]:
        if not isinstance(plan, dict):
            return None
        normalized = dict(plan)
        if 'reasoning' in normalized and 'summary' not in normalized:
            normalized['summary'] = normalized['reasoning']
        return normalized

    def _has_valid_pending_plan(self) -> bool:
        plan = self.pending_plan
        return isinstance(plan, dict) and bool(str(plan.get('session_id', '')).strip())

    # endregion

    def build_app(self) -> FastAPI:
        app = FastAPI(title=self.ui_title)
        templates = Jinja2Templates(directory=str(TEMPLATES_DIR))

        @app.get('/', response_class=HTMLResponse)
        async def index(request: Request):
            context = {
                'request': request,
                'ui_title': self.ui_title,
                'status': self.status_snapshot,
                'active_subtree': self.active_subtree,
            }
            return templates.TemplateResponse('index.html', context)

        @app.get('/state')
        async def state():
            return JSONResponse(
                {
                    'status': self.status_snapshot,
                    'active_subtree': self.active_subtree,
                    'messages': list(self.messages),
                    'diagnostics': self.diagnostics,
                    'pending_plan': self.pending_plan,
                }
            )

        @app.post('/command')
        async def command(payload: Dict[str, str]):
            text = payload.get('text', '')
            if not text.strip():
                raise HTTPException(status_code=400, detail='Command text cannot be empty')
            auto_execute = bool(payload.get('auto_execute', False))
            await self.handle_user_input(text, auto_execute=auto_execute)
            return {'ok': True}

        @app.post('/approve')
        async def approve(payload: Dict[str, str]):
            plan = self.pending_plan
            if not self._has_valid_pending_plan():
                raise HTTPException(status_code=400, detail='No pending plan to approve')
            feedback = (payload or {}).get('feedback', '')
            ok, message = await self._send_operator_decision(
                str(plan.get('session_id')), True, feedback
            )
            if not ok:
                raise HTTPException(status_code=400, detail=message)
            return {'ok': True, 'message': message}

        @app.post('/cancel')
        async def cancel(payload: Dict[str, str]):
            plan = self.pending_plan
            if not self._has_valid_pending_plan():
                raise HTTPException(status_code=400, detail='No pending plan to cancel')
            feedback = (payload or {}).get('feedback', '')
            ok, message = await self._send_operator_decision(
                str(plan.get('session_id')), False, feedback
            )
            if not ok:
                raise HTTPException(status_code=400, detail=message)
            return {'ok': True, 'message': message}

        app.state.ros_node = self
        return app


async def _spin_ros(node: WebInterfaceNode, executor: SingleThreadedExecutor) -> None:
    try:
        while rclpy.ok() and not node.shutdown_event.is_set():
            await asyncio.to_thread(executor.spin_once, timeout_sec=node.refresh_period)
    except (asyncio.CancelledError, KeyboardInterrupt):  # pragma: no cover
        pass


async def _serve_http(node: WebInterfaceNode, app: FastAPI) -> None:
    config = uvicorn.Config(
        app,
        host=node.web_host,
        port=node.web_port,
        log_level='info',
        loop='asyncio',
        access_log=False,
    )
    server = uvicorn.Server(config)
    node.get_logger().info(
        f'Starting web UI at http://{node.web_host}:{node.web_port} (demo_mode={node.demo_mode})'
    )
    await server.serve()
    node.shutdown_event.set()


async def main_async(args=None) -> None:
    rclpy.init(args=args)
    node = WebInterfaceNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    app = node.build_app()

    try:
        await asyncio.gather(_spin_ros(node, executor), _serve_http(node, app))
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    asyncio.run(main_async(args=args))


if __name__ == '__main__':  # pragma: no cover
    main()
