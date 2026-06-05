from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from gen_bt_interfaces.srv import ExtractMissionRequirements, ValidateMission
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission_reasoner.reasoner import (
    ERROR,
    MissionReasoner,
    ValidationResult,
    parse_tree_catalog,
)


class MissionReasonerNode(Node):
    """ROS service wrapper for deterministic mission capability validation."""

    def __init__(self) -> None:
        super().__init__(
            'mission_reasoner',
            automatically_declare_parameters_from_overrides=True,
        )
        if not self.has_parameter('validate_mission_service'):
            self.declare_parameter(
                'validate_mission_service', '/mission_reasoner/validate_mission'
            )
        if not self.has_parameter('mission_requirements_service'):
            self.declare_parameter(
                'mission_requirements_service',
                '/llm_interface/extract_mission_requirements',
            )
        if not self.has_parameter('system_description_file'):
            self.declare_parameter(
                'system_description_file',
                '/home/luke/generalist_bt_gen/config/system_description.yaml',
            )
        if not self.has_parameter('enable_debug_logging'):
            self.declare_parameter('enable_debug_logging', False)
        if not self.has_parameter('enable_llm_extraction'):
            self.declare_parameter('enable_llm_extraction', True)
        if not self.has_parameter('llm_extraction_timeout_sec'):
            self.declare_parameter('llm_extraction_timeout_sec', 45.0)
        if not self.has_parameter('llm_service_wait_timeout_sec'):
            self.declare_parameter('llm_service_wait_timeout_sec', 2.0)

        self._debug_logging = bool(self.get_parameter('enable_debug_logging').value)
        self._enable_llm_extraction = bool(
            self.get_parameter('enable_llm_extraction').value
        )
        self._llm_extraction_timeout_sec = float(
            self.get_parameter('llm_extraction_timeout_sec').value
        )
        self._llm_service_wait_timeout_sec = float(
            self.get_parameter('llm_service_wait_timeout_sec').value
        )
        self._system_description_file = str(
            self.get_parameter('system_description_file').value
        )
        self._reasoner = self._load_reasoner(self._system_description_file)
        service_name = str(self.get_parameter('validate_mission_service').value)
        requirements_service = str(
            self.get_parameter('mission_requirements_service').value
        )
        self._callback_group = ReentrantCallbackGroup()
        self._requirements_client = self.create_client(
            ExtractMissionRequirements,
            requirements_service,
            callback_group=self._callback_group,
        )
        self._service = self.create_service(
            ValidateMission,
            service_name,
            self._handle_validate_mission,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'MissionReasonerNode ready (service={service_name}, '
            f'requirements_service={requirements_service}, '
            f'llm_extraction={self._enable_llm_extraction}, '
            f'llm_timeout={self._llm_extraction_timeout_sec:.1f}s, '
            f'system_description={self._system_description_file})'
        )

    def _load_reasoner(self, system_description_file: str) -> MissionReasoner:
        path = Path(system_description_file).expanduser()
        if not path.exists():
            raise FileNotFoundError(f'System description file not found: {path}')
        return MissionReasoner.from_yaml_file(str(path))

    def _handle_validate_mission(
        self, request: ValidateMission.Request, response: ValidateMission.Response
    ) -> ValidateMission.Response:
        try:
            tree_catalog = parse_tree_catalog(request.tree_catalog_json)
            extracted_requirements = self._extract_requirements(request)
            result = self._reasoner.validate(
                request.user_command,
                tree_catalog,
                context_json=request.context_json,
                extracted_requirements=extracted_requirements,
            )
        except Exception as exc:
            self.get_logger().error(f'Mission validation failed: {exc}')
            result = ValidationResult(
                status_code=ERROR,
                message=f'Mission validation error: {exc}',
                reasoning={'error': str(exc)},
            )
        response.status_code = int(result.status_code)
        response.message = result.message
        response.clarification_question = result.clarification_question
        response.reasoning_json = json.dumps(result.reasoning, ensure_ascii=False)
        response.matched_capabilities = list(result.matched_capabilities)
        response.missing_capabilities = list(result.missing_capabilities)
        response.candidate_trees = list(result.candidate_trees)
        if self._debug_logging:
            self.get_logger().info(
                f'Validation response status={response.status_code}, '
                f'candidates={response.candidate_trees}, missing={response.missing_capabilities}'
            )
        return response

    def _extract_requirements(
        self, request: ValidateMission.Request
    ) -> Optional[Dict[str, Any]]:
        if not self._enable_llm_extraction:
            return None
        if not self._requirements_client.wait_for_service(
            timeout_sec=self._llm_service_wait_timeout_sec
        ):
            self.get_logger().warning(
                'Mission requirement extraction service unavailable after '
                f'{self._llm_service_wait_timeout_sec:.1f}s; using deterministic rules.'
            )
            return None
        llm_request = ExtractMissionRequirements.Request()
        llm_request.session_id = request.session_id
        llm_request.user_command = request.user_command
        llm_request.context_json = request.context_json
        llm_request.capability_catalog_json = self._capability_catalog_json()
        llm_request.tree_catalog_json = request.tree_catalog_json

        future = self._requirements_client.call_async(llm_request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        started_at = time.monotonic()
        if not event.wait(timeout=self._llm_extraction_timeout_sec):
            future.cancel()
            self.get_logger().warning(
                'Mission requirement extraction timed out after '
                f'{self._llm_extraction_timeout_sec:.1f}s for command '
                f"'{request.user_command}'; using deterministic rules."
            )
            return None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Mission requirement extraction failed: {exc}; using deterministic rules.'
            )
            return None
        elapsed = time.monotonic() - started_at
        if self._debug_logging:
            self.get_logger().info(
                f'Mission requirement extraction completed in {elapsed:.2f}s.'
            )
        if response is None or response.status_code != response.SUCCESS:
            reason = getattr(response, 'reasoning', '') if response else ''
            self.get_logger().warning(
                f'Mission requirement extraction returned no usable result: {reason}'
            )
            return None
        try:
            parsed = json.loads(response.requirements_json or '{}')
        except Exception as exc:
            self.get_logger().warning(
                f'Mission requirement extraction JSON invalid: {exc}'
            )
            return None
        if not isinstance(parsed, dict):
            return None
        parsed['_source'] = 'llm_interface'
        parsed['_reasoning'] = response.reasoning
        return parsed

    def _capability_catalog_json(self) -> str:
        capabilities = self._reasoner.system_description.get('capabilities', {})
        payload = {
            'supported': capabilities.get('supported', []) or [],
            'unsupported': capabilities.get('unsupported', {}) or {},
            'command_capability_rules': self._reasoner.system_description.get(
                'command_capability_rules', []
            )
            or [],
        }
        return json.dumps(payload, ensure_ascii=False)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionReasonerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('MissionReasonerNode interrupted, shutting down.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['MissionReasonerNode', 'main']
