import asyncio
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Deque, Dict, Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.templating import Jinja2Templates
from gen_bt_interfaces.srv import MissionControl
from pydantic import BaseModel
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import uvicorn

from .command_router import CommandRouter, parse_session_and_note
from .mission_gateway import MissionGateway

try:
    from zoneinfo import ZoneInfo
except ImportError:  # pragma: no cover
    ZoneInfo = None  # type: ignore[assignment]


TEMPLATES_DIR = Path(__file__).resolve().parent / 'templates'


class CommandPayload(BaseModel):
    text: str
    auto_execute: bool = False


class DecisionPayload(BaseModel):
    session_id: str = ''
    feedback: str = ''


class WebInterfaceNode(Node):
    """Web-based chat UI that exposes mission coordinator hooks via HTTP."""

    def __init__(self) -> None:
        super().__init__('chat_web_interface')

        self.declare_parameter('ui_title', 'Generalist BT Chat')
        self.declare_parameter('mission_coordinator_action', '/mission_coordinator/execute_tree')
        self.declare_parameter('status_service', '/mission_coordinator/status')
        self.declare_parameter('control_service', '/mission_coordinator/control')
        self.declare_parameter('status_topic', '/mission_coordinator/status_text')
        self.declare_parameter('subtree_topic', '/mission_coordinator/active_subtree')
        self.declare_parameter('pending_plan_topic', '/mission_coordinator/pending_plan')
        self.declare_parameter('diagnostics_topics', [])
        self.declare_parameter('transcript_directory', '~/.generalist_bt/chat_logs')
        self.declare_parameter('refresh_period', 0.1)
        self.declare_parameter('timestamp_timezone', 'UTC')
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 8080)
        self.declare_parameter(
            'operator_decision_service', '/mission_coordinator/operator_decision'
        )

        self.ui_title = self.get_parameter('ui_title').value
        self.mission_coordinator_action = self.get_parameter('mission_coordinator_action').value
        self.status_service = self.get_parameter('status_service').value
        self.control_service = self.get_parameter('control_service').value
        self.status_topic = self.get_parameter('status_topic').value
        self.subtree_topic = self.get_parameter('subtree_topic').value
        self.pending_plan_topic = self.get_parameter('pending_plan_topic').value
        self.transcript_directory = Path(
            self.get_parameter('transcript_directory').value
        ).expanduser()
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.refresh_period = float(self.get_parameter('refresh_period').value)
        self.web_host = self.get_parameter('web_host').value or '0.0.0.0'
        self.web_port = int(self.get_parameter('web_port').value)
        self.operator_decision_service = self.get_parameter('operator_decision_service').value

        tz_name = self.get_parameter('timestamp_timezone').value or 'UTC'
        self._tzinfo = self._resolve_timezone(tz_name)
        self._shutdown_event: asyncio.Event = asyncio.Event()

        self.status_snapshot = 'Idle'
        self.active_subtree = 'unknown'
        self.diagnostics: Dict[str, str] = {}
        self.messages: Deque[Dict[str, str]] = deque(maxlen=200)
        self.pending_plan: Optional[Dict] = None

        self._active_mission_task: Optional[asyncio.Task] = None
        self._active_mission_session_id = ''

        self._log_handle = self._prepare_transcript()

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
            }
        )

        self._record_message('system', 'Web UI initialized.')

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

    def close(self) -> None:
        if self._active_mission_task and not self._active_mission_task.done():
            self._active_mission_task.cancel()
        if self._log_handle:
            self._log_handle.close()
            self._log_handle = None  # type: ignore[assignment]

    async def handle_user_input(self, text: str, auto_execute: bool = False) -> dict:
        stripped = text.strip()
        if not stripped:
            return {'ok': False, 'error': 'empty_input'}

        if stripped.startswith('/'):
            handled = await self._command_router.dispatch(stripped)
            if not handled:
                message = f'Unknown command {stripped.split(" ", 1)[0]}'
                self._record_message('system', message)
                return {'ok': False, 'message': message}
            return {'ok': True}

        self._record_message('user', stripped)
        return await self._start_mission(stripped, auto_execute=auto_execute)

    async def _start_mission(self, command_text: str, auto_execute: bool = False) -> dict:
        if self._active_mission_task and not self._active_mission_task.done():
            message = (
                f'Mission already active (session={self._active_mission_session_id}). '
                'Use /status, /approve, /cancel, or /abort.'
            )
            self._record_message('system', message)
            return {
                'ok': False,
                'message': message,
                'session_id': self._active_mission_session_id,
            }

        if self.demo_mode:
            await asyncio.sleep(0.1)
            message = (
                f'Pretend to dispatch "{command_text}" to {self.mission_coordinator_action} '
                '(demo mode).'
            )
            self._record_message('demo', message)
            return {'ok': True, 'demo': True}

        goal_handle, session_id = await self.gateway.send_mission_goal(
            command_text,
            auto_execute=auto_execute,
            feedback_callback=self._mission_feedback,
            session_prefix='webui',
        )
        if not session_id:
            message = f'Mission coordinator action {self.mission_coordinator_action} unavailable.'
            self._record_message('system', message)
            return {'ok': False, 'message': message}

        self._active_mission_session_id = session_id
        self._record_message(
            'system',
            f'Dispatching mission to {self.mission_coordinator_action} (session={session_id})',
        )

        if goal_handle is None or not goal_handle.accepted:
            message = 'Mission coordinator rejected the request.'
            self._record_message('system', message)
            self._active_mission_session_id = ''
            return {'ok': False, 'message': message, 'session_id': session_id}

        self._active_mission_task = asyncio.create_task(
            self._wait_for_mission_result(goal_handle, session_id)
        )
        return {'ok': True, 'session_id': session_id}

    async def _wait_for_mission_result(self, goal_handle, session_id: str) -> None:
        result = await self.gateway.await_mission_result(goal_handle)
        if result is None:
            self._record_message('system', 'Mission coordinator result unavailable.')
            self._active_mission_session_id = ''
            return
        if self._has_valid_pending_plan():
            plan = self.pending_plan or {}
            if str(plan.get('session_id', '')).strip() == session_id:
                self.pending_plan = None
        self._record_message(
            'system',
            f'Mission result: accepted={result.result.accepted}, '
            f"detail='{result.result.outcome_message}'",
        )
        self._active_mission_session_id = ''

    def _mission_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._record_message('feedback', f'{feedback.stage}: {feedback.detail}')

    async def _cmd_status(self, _: str) -> None:
        ok, response, err = await self.gateway.query_state(self._active_mission_session_id)
        if not ok or response is None:
            self._record_message('system', err or 'Status unavailable.')
            return
        self.status_snapshot = response.status_text or self.status_snapshot
        self.active_subtree = response.active_subtree or self.active_subtree
        try:
            parsed_pending = json.loads(response.pending_plan_json or '{}')
            self.pending_plan = self._normalize_pending_plan(parsed_pending)
        except Exception:
            pass
        self._record_message(
            'system',
            (
                f"state={response.lifecycle_state}, status='{response.status_text}', "
                'active_subtree='
                f"'{response.active_subtree}', session='{response.active_session_id or '-'}'"
            ),
        )
        if self._has_valid_pending_plan():
            self._record_pending_plan_details()

    async def _cmd_help(self, _: str) -> None:
        for line in [
            '/status             - Show state + pending payload details',
            '/approve [id] [msg] - Approve pending operator plan',
            '/cancel [id] [msg]  - Reject pending operator plan',
            '/abort [id] [msg]   - Abort active mission',
        ]:
            self._record_message('system', line)

    async def _cmd_approve(self, payload: str) -> None:
        session_id, feedback = parse_session_and_note(payload)
        if not session_id and self._has_valid_pending_plan():
            session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
        if not session_id:
            self._record_message('system', 'No pending plan to approve.')
            return
        ok, message = await self.gateway.send_control(
            MissionControl.Request.APPROVE,
            session_id=session_id,
            note=feedback,
        )
        self._record_message(
            'system', message or ('Execution approved.' if ok else 'Approve failed.')
        )
        if ok:
            self.pending_plan = None

    async def _cmd_cancel(self, payload: str) -> None:
        session_id, feedback = parse_session_and_note(payload)
        if not session_id and self._has_valid_pending_plan():
            session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
        if not session_id:
            self._record_message('system', 'No pending plan to cancel.')
            return
        ok, message = await self.gateway.send_control(
            MissionControl.Request.REJECT,
            session_id=session_id,
            note=feedback,
        )
        self._record_message(
            'system', message or ('Execution rejected.' if ok else 'Cancel failed.')
        )
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
        self._record_message('system', message or ('Abort requested.' if ok else 'Abort failed.'))

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
            return
        if self._has_valid_pending_plan():
            self._record_pending_plan_details()

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

    def _record_pending_plan_details(self) -> None:
        if not self._has_valid_pending_plan():
            return
        plan = self.pending_plan or {}
        session_id = str(plan.get('session_id', '')).strip()
        tree_id = str(plan.get('tree_id', 'unknown')).strip() or 'unknown'
        mission = str(plan.get('mission', '')).strip()
        summary = str(plan.get('summary', '') or plan.get('reasoning', '')).strip()
        self._record_message(
            'system',
            f'Operator approval required for session {session_id} (tree={tree_id}).',
        )
        if mission:
            self._record_message('system', f'Mission: {mission}')
        if summary:
            self._record_message('system', f'Summary: {summary}')
        payload_preview = self._pretty_payload(plan.get('payload_json', '{}'))
        self._record_message('system', f'Payload:\n{payload_preview}')

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
                    'active_session_id': self._active_mission_session_id,
                }
            )

        @app.post('/command')
        async def command(payload: CommandPayload):
            text = payload.text
            if not text.strip():
                raise HTTPException(status_code=400, detail='Command text cannot be empty')
            outcome = await self.handle_user_input(text, auto_execute=payload.auto_execute)
            if not outcome.get('ok', False):
                raise HTTPException(
                    status_code=400, detail=outcome.get('message', 'Command failed')
                )
            return outcome

        @app.post('/approve')
        async def approve(payload: DecisionPayload):
            session_id = (payload.session_id or '').strip()
            if not session_id and self._has_valid_pending_plan():
                session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
            if not session_id:
                raise HTTPException(status_code=400, detail='No pending plan to approve')
            ok, message = await self.gateway.send_control(
                MissionControl.Request.APPROVE,
                session_id=session_id,
                note=payload.feedback or '',
            )
            if not ok:
                raise HTTPException(status_code=400, detail=message or 'Failed to approve plan')
            self.pending_plan = None
            return {'ok': True, 'message': message}

        @app.post('/cancel')
        async def cancel(payload: DecisionPayload):
            session_id = (payload.session_id or '').strip()
            if not session_id and self._has_valid_pending_plan():
                session_id = str((self.pending_plan or {}).get('session_id', '')).strip()
            if not session_id:
                raise HTTPException(status_code=400, detail='No pending plan to cancel')
            ok, message = await self.gateway.send_control(
                MissionControl.Request.REJECT,
                session_id=session_id,
                note=payload.feedback or '',
            )
            if not ok:
                raise HTTPException(status_code=400, detail=message or 'Failed to cancel plan')
            self.pending_plan = None
            return {'ok': True, 'message': message}

        @app.post('/abort')
        async def abort(payload: DecisionPayload):
            session_id = (payload.session_id or '').strip() or self._active_mission_session_id
            ok, message = await self.gateway.send_control(
                MissionControl.Request.ABORT,
                session_id=session_id,
                note=payload.feedback or '',
            )
            if not ok:
                raise HTTPException(
                    status_code=400, detail=message or 'Failed to abort mission'
                )
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
