from __future__ import annotations

import asyncio
from datetime import datetime, timezone
import json
from typing import Optional

from gen_bt_interfaces.action import MissionCommand
from gen_bt_interfaces.srv import GetMissionState, MissionControl, OperatorDecision
from rclpy.action import ActionClient


class MissionGateway:
    """Shared ROS gateway for chat/web user interfaces."""

    def __init__(
        self,
        node,
        mission_action_name: str,
        status_service_name: str,
        control_service_name: str,
        operator_decision_service: str,
    ) -> None:
        self._node = node
        self._mission_action_name = mission_action_name
        self._status_service_name = status_service_name
        self._control_service_name = control_service_name
        self._operator_decision_service = operator_decision_service
        self._mission_action_client = ActionClient(
            node, MissionCommand, mission_action_name
        )
        self._status_client = node.create_client(GetMissionState, status_service_name)
        self._control_client = node.create_client(MissionControl, control_service_name)
        self._operator_decision_clients = {}
        self._session_counter = 0

    @property
    def mission_action_name(self) -> str:
        return self._mission_action_name

    @property
    def status_service_name(self) -> str:
        return self._status_service_name

    @property
    def control_service_name(self) -> str:
        return self._control_service_name

    async def send_mission_goal(
        self,
        command_text: str,
        auto_execute: bool = False,
        feedback_callback=None,
        session_prefix: str = 'chatui',
    ) -> tuple[Optional[object], str]:
        available = await self._wait_for_action_server(self._mission_action_client)
        if not available:
            return None, ''

        session_id = self._next_session_id(session_prefix)
        goal_msg = MissionCommand.Goal()
        goal_msg.command = command_text
        goal_msg.session_id = session_id
        context = {'auto_execute': bool(auto_execute)} if auto_execute else {}
        goal_msg.context_json = json.dumps(context, ensure_ascii=False)
        send_future = self._mission_action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )
        goal_handle = await self._await_future(send_future, 'send mission goal')
        return goal_handle, session_id

    async def await_mission_result(self, goal_handle) -> Optional[object]:
        if goal_handle is None:
            return None
        result_future = goal_handle.get_result_async()
        return await self._await_future(result_future, 'mission result')

    async def query_state(
        self, session_id: str = ''
    ) -> tuple[bool, Optional[GetMissionState.Response], str]:
        ready = await self._wait_for_service(self._status_client)
        if not ready:
            return False, None, f'Status service {self._status_service_name} unavailable.'
        request = GetMissionState.Request()
        request.session_id = session_id or ''
        response = await self._await_future(
            self._status_client.call_async(request), 'status query'
        )
        if response is None:
            return False, None, 'Status response unavailable.'
        return True, response, ''

    async def send_control(
        self, command: int, session_id: str = '', note: str = ''
    ) -> tuple[bool, str]:
        ready = await self._wait_for_service(self._control_client)
        if not ready:
            return False, f'Control service {self._control_service_name} unavailable.'
        request = MissionControl.Request()
        request.session_id = session_id or ''
        request.command = int(command)
        request.note = note or ''
        response = await self._await_future(
            self._control_client.call_async(request), 'mission control'
        )
        if response is None:
            return False, 'Control response unavailable.'
        return bool(response.accepted), response.message or ''

    async def send_operator_decision(
        self,
        session_id: str,
        approve: bool,
        feedback: str = '',
        service_name: Optional[str] = None,
    ) -> tuple[bool, str]:
        target = (service_name or self._operator_decision_service or '').strip()
        if not target:
            target = '/mission_coordinator/operator_decision'
        client = self._operator_decision_clients.get(target)
        if client is None:
            client = self._node.create_client(OperatorDecision, target)
            self._operator_decision_clients[target] = client
        ready = await self._wait_for_service(client)
        if not ready:
            return False, f'Operator decision service {target} unavailable.'
        request = OperatorDecision.Request()
        request.session_id = session_id or ''
        request.approve = bool(approve)
        request.feedback = feedback or ''
        response = await self._await_future(
            client.call_async(request), 'operator decision'
        )
        if response is None:
            return False, 'Operator decision response unavailable.'
        return bool(response.accepted), response.message or ''

    def _next_session_id(self, prefix: str) -> str:
        self._session_counter += 1
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        safe_prefix = (prefix or 'chatui').strip() or 'chatui'
        return f'{safe_prefix}-{timestamp}-{self._session_counter}'

    async def _wait_for_action_server(self, client, timeout_sec: float = 1.0) -> bool:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, client.wait_for_server, timeout_sec)

    async def _wait_for_service(self, client, timeout_sec: float = 1.0) -> bool:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, client.wait_for_service, timeout_sec)

    async def _await_future(self, future, context: str):
        while not future.done():
            await asyncio.sleep(0.05)
        if future.cancelled():
            return None
        if future.exception():
            self._node.get_logger().error(
                f'Future failed while waiting for {context}: {future.exception()}'
            )
            return None
        return future.result()
