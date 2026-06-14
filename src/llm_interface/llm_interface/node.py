from __future__ import annotations

import base64
import concurrent.futures
import json
import math
import mimetypes
import os
import re
from pathlib import Path
from typing import List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from gen_bt_interfaces.srv import (
    CreatePayload,
    ExtractMissionRequirements,
    PlanSubtree,
    SelectBehaviorTree,
)
try:
    from langchain_core.prompts import PromptTemplate
    from langchain_core.messages import HumanMessage
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "langchain-core not found. Install it in the ROS Python environment."
    ) from exc
try:
    from langchain_google_genai import ChatGoogleGenerativeAI
except ModuleNotFoundError:
    ChatGoogleGenerativeAI = None
try:
    from langchain_openai import ChatOpenAI
except ModuleNotFoundError:
    ChatOpenAI = None
try:
    from langchain_openrouter import ChatOpenRouter
except ModuleNotFoundError:
    ChatOpenRouter = None
from rclpy.node import Node

DEFAULT_LLM_PROVIDER = 'gemini'
SUPPORTED_LLM_PROVIDERS = {'gemini', 'openai', 'openrouter'}

DEFAULT_SELECTION_PROMPT = (
    "You are a mission planner for a field robot. Given USER_COMMAND and a JSON array of\n"
    "behavior trees (each with id + description), choose the best tree or return NONE.\n"
    "tree_id MUST be exactly one of the string IDs in TREES_JSON "
    "(e.g., \"temperature_logging.xml\").\n"
    "Do NOT answer with numeric indexes.\n"
    "Respond strictly as JSON: {{\"tree_id\": \"<id or NONE>\", \"confidence\": 0-1, \"rationale\": \"...\"}}\n"
    "USER_COMMAND: {user_command}\n"
    "TREES_JSON: {trees_json}\n"
)

DEFAULT_REQUIREMENTS_PROMPT = (
    "You extract robot mission requirements from natural language. Return only JSON.\n"
    "Use capability IDs exactly as they appear in CAPABILITY_CATALOG_JSON when possible.\n"
    "Do not decide whether the mission is safe or executable; only classify what the user asks for.\n\n"
    "USER_COMMAND: {user_command}\n\n"
    "CONTEXT_JSON: {context_json}\n\n"
    "CAPABILITY_CATALOG_JSON: {capability_catalog_json}\n\n"
    "TREE_CATALOG_JSON: {tree_catalog_json}\n\n"
    "Return JSON with this shape:\n"
    "{{\n"
    "  \"required_capabilities\": [\"capability.id\"],\n"
    "  \"mission_intents\": [\"intent_id\"],\n"
    "  \"constraints\": {{\"range_m\": null, \"duration_s\": null}},\n"
    "  \"ambiguities\": [\"short operator-facing ambiguity\"],\n"
    "  \"rationale\": \"brief explanation\"\n"
    "}}\n"
)

DEFAULT_PAYLOAD_PROMPT = (
    "You are a robot mission planner. Given CONTEXT (sensor data) and CONTRACT "
    "(required blackboard keys), generate a JSON payload for the behavior tree.\n\n"
    "SUBTREE_ID: {subtree_id}\n\n"
    "USER_COMMAND: {user_command}\n\n"
    "OPERATOR_REFINEMENT_NOTES:\n"
    "{refinement_notes}\n\n"
    "SESSION_REFINEMENT_HISTORY_JSON:\n"
    "{refinement_history_json}\n\n"
    "PRIOR_REJECTED_PLAN_REASONING:\n"
    "{prior_plan_reasoning}\n\n"
    "PRIOR_REJECTED_PAYLOAD_JSON:\n"
    "{prior_plan_payload_json}\n\n"
    "CONTEXT (raw sensor data):\n"
    "{context_json}\n\n"
    "CONTRACT (required blackboard keys with types/defaults):\n"
    "{contract_json}\n\n"
    "ATTACHMENT_URIS: {attachment_uris}\n\n"
    "INSTRUCTIONS:\n"
    "1. Map context data to blackboard keys specified in the contract\n"
    "2. Use contract defaults when context is missing\n"
    "3. Infer reasonable values from context (e.g., extract waypoints from GPS trail)\n"
    "4. If images are provided, use them to extract map/goal information\n"
    "5. Return ONLY valid JSON matching the contract schema\n"
    "6. Do NOT include markdown code fences\n\n"
    "7. If OPERATOR_REFINEMENT_NOTES are present, treat them as corrections to the prior plan\n"
    "   and update the payload to address them explicitly\n\n"
    "8. If SESSION_REFINEMENT_HISTORY_JSON is present, use each entry to match rejection feedback\n"
    "   with the payload and reasoning that caused it, and avoid repeating the same failures\n\n"
    "9. If PRIOR_REJECTED_PLAN_REASONING or PRIOR_REJECTED_PAYLOAD_JSON are present, use them\n"
    "   as the rejected baseline and revise the payload instead of repeating the same mistake\n\n"
    "OUTPUT (JSON only):"
)

DEFAULT_PAYLOAD_NORMALIZER_PROMPT = (
    "You are a strict JSON schema enforcer for robot mission payloads.\n"
    "Compare MODEL_OUTPUT against CONTRACT_SCHEMA and correct/normalize it to strictly match.\n"
    "Use defaults in the contract when values are missing. If a value is missing and no default\n"
    "is provided, infer from the ORIGINAL_PROMPT context when reasonable; otherwise set null.\n"
    "If the MODEL_OUTPUT already matches the schema, return it unchanged.\n"
    "Return ONLY a valid JSON object (no markdown).\n\n"
    "ORIGINAL_PROMPT:\n"
    "{payload_prompt}\n\n"
    "CONTRACT_SCHEMA:\n"
    "{contract_json}\n\n"
    "MODEL_OUTPUT:\n"
    "{raw_payload}\n\n"
    "NORMALIZED_OUTPUT (JSON only):"
)


class LLMInterfaceNode(Node):
    """LLM orchestration stub that exposes selection + planning services."""

    PLAN_STATUS_SUCCESS = 0
    PLAN_STATUS_RETRY = 1
    PLAN_STATUS_ESCALATE = 2

    SELECT_FOUND = 0
    SELECT_NO_MATCH = 1
    SELECT_ERROR = 2

    def __init__(self) -> None:
        super().__init__(
            'llm_interface',
            automatically_declare_parameters_from_overrides=True,
        )

        if not self.has_parameter('planning_service_name'):
            self.declare_parameter('planning_service_name', '/llm_interface/plan_subtree')
        if not self.has_parameter('selection_service_name'):
            self.declare_parameter('selection_service_name', '/llm_interface/select_behavior_tree')
        if not self.has_parameter('create_payload_service_name'):
            self.declare_parameter('create_payload_service_name', '/llm_interface/create_payload')
        if not self.has_parameter('mission_requirements_service_name'):
            self.declare_parameter(
                'mission_requirements_service_name',
                '/llm_interface/extract_mission_requirements',
            )
        if not self.has_parameter('default_bt_template'):
            self.declare_parameter(
                'default_bt_template',
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n"
                "  <BehaviorTree ID=\"MainTree\">\n"
                "    <Sequence>\n"
                "      <AlwaysSuccess name=\"Acknowledge\" />\n"
                "    </Sequence>\n"
                "  </BehaviorTree>\n"
                "</root>\n",
            )
        if not self.has_parameter('model_name'):
            self.declare_parameter('model_name', 'gemini-2.5-flash')
        if not self.has_parameter('provider'):
            self.declare_parameter('provider', DEFAULT_LLM_PROVIDER)
        if not self.has_parameter('selection_provider'):
            self.declare_parameter(
                'selection_provider', self.get_parameter('provider').value
            )
        if not self.has_parameter('payload_provider'):
            self.declare_parameter(
                'payload_provider', self.get_parameter('provider').value
            )
        if not self.has_parameter('payload_normalizer_provider'):
            self.declare_parameter(
                'payload_normalizer_provider', self.get_parameter('provider').value
            )
        if not self.has_parameter('mission_requirements_provider'):
            self.declare_parameter(
                'mission_requirements_provider', self.get_parameter('provider').value
            )
        if not self.has_parameter('openrouter_app_url'):
            self.declare_parameter('openrouter_app_url', '')
        if not self.has_parameter('openrouter_app_title'):
            self.declare_parameter('openrouter_app_title', 'generalist_bt_gen')
        if not self.has_parameter('selection_model_name'):
            self.declare_parameter(
                'selection_model_name', self.get_parameter('model_name').value
            )
        if not self.has_parameter('payload_model_name'):
            self.declare_parameter(
                'payload_model_name', self.get_parameter('model_name').value
            )
        if not self.has_parameter('payload_normalizer_model_name'):
            self.declare_parameter(
                'payload_normalizer_model_name', self.get_parameter('model_name').value
            )
        if not self.has_parameter('mission_requirements_model_name'):
            self.declare_parameter(
                'mission_requirements_model_name', self.get_parameter('model_name').value
            )
        if not self.has_parameter('selection_llm_enabled'):
            self.declare_parameter('selection_llm_enabled', True)
        if not self.has_parameter('mission_requirements_llm_enabled'):
            self.declare_parameter('mission_requirements_llm_enabled', True)
        if not self.has_parameter('selection_temperature'):
            self.declare_parameter('selection_temperature', 0.0)
        if not self.has_parameter('selection_timeout_sec'):
            self.declare_parameter('selection_timeout_sec', 10.0)
        if not self.has_parameter('mission_requirements_temperature'):
            self.declare_parameter('mission_requirements_temperature', 0.0)
        if not self.has_parameter('selection_reasoning_effort'):
            self.declare_parameter('selection_reasoning_effort', '')
        if not self.has_parameter('selection_reasoning_summary'):
            self.declare_parameter('selection_reasoning_summary', '')
        if not self.has_parameter('mission_requirements_reasoning_effort'):
            self.declare_parameter('mission_requirements_reasoning_effort', '')
        if not self.has_parameter('mission_requirements_reasoning_summary'):
            self.declare_parameter('mission_requirements_reasoning_summary', '')
        if not self.has_parameter('payload_reasoning_effort'):
            self.declare_parameter('payload_reasoning_effort', '')
        if not self.has_parameter('payload_reasoning_summary'):
            self.declare_parameter('payload_reasoning_summary', '')
        if not self.has_parameter('payload_normalizer_reasoning_effort'):
            self.declare_parameter('payload_normalizer_reasoning_effort', '')
        if not self.has_parameter('payload_normalizer_reasoning_summary'):
            self.declare_parameter('payload_normalizer_reasoning_summary', '')
        if not self.has_parameter('payload_multimodal_enabled'):
            self.declare_parameter('payload_multimodal_enabled', True)
        if not self.has_parameter('payload_max_attachment_bytes'):
            self.declare_parameter('payload_max_attachment_bytes', 5_000_000)
        if not self.has_parameter('payload_max_attachments'):
            self.declare_parameter('payload_max_attachments', 6)
        if not self.has_parameter('payload_attachment_allowlist'):
            self.declare_parameter(
                'payload_attachment_allowlist',
                ['image/png', 'image/jpeg']
                )
        if not self.has_parameter('payload_schema_enforced'):
            self.declare_parameter('payload_schema_enforced', False)
        if not self.has_parameter('payload_schema_max_retries'):
            self.declare_parameter('payload_schema_max_retries', 1)
        if not self.has_parameter('prompts.selection'):
            self.declare_parameter('prompts.selection', DEFAULT_SELECTION_PROMPT)
        if not self.has_parameter('prompts.mission_requirements'):
            self.declare_parameter(
                'prompts.mission_requirements', DEFAULT_REQUIREMENTS_PROMPT
            )
        if not self.has_parameter('prompts.payload.default'):
            self.declare_parameter('prompts.payload.default', DEFAULT_PAYLOAD_PROMPT)
        if not self.has_parameter('prompts.payload_normalizer'):
            self.declare_parameter(
                'prompts.payload_normalizer', DEFAULT_PAYLOAD_NORMALIZER_PROMPT
            )
        self._provider = self._normalize_provider_name(
            str(self.get_parameter('provider').value)
        )
        self._selection_provider = self._normalize_provider_name(
            str(self.get_parameter('selection_provider').value or self._provider)
        )
        self._payload_provider = self._normalize_provider_name(
            str(self.get_parameter('payload_provider').value or self._provider)
        )
        self._payload_normalizer_provider = self._normalize_provider_name(
            str(
                self.get_parameter('payload_normalizer_provider').value or self._provider
            )
        )
        self._mission_requirements_provider = self._normalize_provider_name(
            str(
                self.get_parameter('mission_requirements_provider').value
                or self._provider
            )
        )
        self._model_name = str(self.get_parameter('model_name').value)
        self._selection_model_name = str(
            self.get_parameter('selection_model_name').value or self._model_name
        )
        self._payload_model_name = str(
            self.get_parameter('payload_model_name').value or self._model_name
        )
        self._payload_normalizer_model_name = str(
            self.get_parameter('payload_normalizer_model_name').value or self._model_name
        )
        self._mission_requirements_model_name = str(
            self.get_parameter('mission_requirements_model_name').value or self._model_name
        )
        self._selection_llm_enabled = bool(
            self.get_parameter('selection_llm_enabled').value
        )
        self._mission_requirements_llm_enabled = bool(
            self.get_parameter('mission_requirements_llm_enabled').value
        )
        self._selection_temperature = float(
            self.get_parameter('selection_temperature').value
        )
        self._selection_timeout_sec = float(
            self.get_parameter('selection_timeout_sec').value
        )
        self._mission_requirements_temperature = float(
            self.get_parameter('mission_requirements_temperature').value
        )
        self._selection_reasoning_effort = str(
            self.get_parameter('selection_reasoning_effort').value or ''
        ).strip()
        self._selection_reasoning_summary = str(
            self.get_parameter('selection_reasoning_summary').value or ''
        ).strip()
        self._mission_requirements_reasoning_effort = str(
            self.get_parameter('mission_requirements_reasoning_effort').value or ''
        ).strip()
        self._mission_requirements_reasoning_summary = str(
            self.get_parameter('mission_requirements_reasoning_summary').value or ''
        ).strip()
        self._payload_reasoning_effort = str(
            self.get_parameter('payload_reasoning_effort').value or ''
        ).strip()
        self._payload_reasoning_summary = str(
            self.get_parameter('payload_reasoning_summary').value or ''
        ).strip()
        self._payload_normalizer_reasoning_effort = str(
            self.get_parameter('payload_normalizer_reasoning_effort').value or ''
        ).strip()
        self._payload_normalizer_reasoning_summary = str(
            self.get_parameter('payload_normalizer_reasoning_summary').value or ''
        ).strip()
        self._payload_multimodal_enabled = bool(
            self.get_parameter('payload_multimodal_enabled').value
        )
        self._payload_max_attachment_bytes = int(
            self.get_parameter('payload_max_attachment_bytes').value
        )
        self._payload_max_attachments = int(
            self.get_parameter('payload_max_attachments').value
        )
        self._payload_attachment_allowlist = list(
            self.get_parameter('payload_attachment_allowlist').value
        )
        self._payload_schema_enforced = bool(
            self.get_parameter('payload_schema_enforced').value
        )
        self._payload_schema_max_retries = int(
            self.get_parameter('payload_schema_max_retries').value
        )
        self._selection_prompt_template = str(
            self.get_parameter('prompts.selection').value
        )
        self._mission_requirements_prompt_template = str(
            self.get_parameter('prompts.mission_requirements').value
        )
        self._payload_prompt_default = str(
            self.get_parameter('prompts.payload.default').value
        )
        self._payload_normalizer_prompt = str(
            self.get_parameter('prompts.payload_normalizer').value
        )
        self._openrouter_app_url = str(
            self.get_parameter('openrouter_app_url').value or ''
        ).strip()
        self._openrouter_app_title = str(
            self.get_parameter('openrouter_app_title').value or ''
        ).strip()
        payload_prompts = self.get_parameters_by_prefix('prompts.payload')
        self._payload_prompts = {
            name: param.value
            for name, param in payload_prompts.items()
            if name != 'default' and isinstance(param.value, str)
        }

        planning_topic = self.get_parameter('planning_service_name').value
        selection_topic = self.get_parameter('selection_service_name').value
        payload_topic = self.get_parameter('create_payload_service_name').value
        requirements_topic = self.get_parameter('mission_requirements_service_name').value
        self._plan_template = self.get_parameter('default_bt_template').value

        self._plan_service = self.create_service(
            PlanSubtree, planning_topic, self.handle_plan_request
        )
        self._selection_service = self.create_service(
            SelectBehaviorTree, selection_topic, self.handle_selection_request
        )
        self._requirements_service = self.create_service(
            ExtractMissionRequirements,
            requirements_topic,
            self.handle_extract_mission_requirements,
        )
        self._payload_service = self.create_service(
            CreatePayload, payload_topic, self.handle_create_payload
        )
        self.get_logger().info(
            "LLMInterfaceNode ready "
            f"(plan={planning_topic}, select={selection_topic}, "
            f"requirements={requirements_topic}, payload={payload_topic}, "
            f"selection_provider={self._selection_provider}, "
            f"selection_model={self._selection_model_name}, "
            f"selection_timeout_sec={self._selection_timeout_sec:.1f}, "
            f"mission_requirements_provider={self._mission_requirements_provider}, "
            f"mission_requirements_model={self._mission_requirements_model_name}, "
            f"payload_provider={self._payload_provider}, "
            f"payload_model={self._payload_model_name}, "
            f"payload_normalizer_provider={self._payload_normalizer_provider}, "
            f"payload_normalizer_model={self._payload_normalizer_model_name}, "
            f"selection_reasoning_effort={self._selection_reasoning_effort or '<disabled>'}, "
            "mission_requirements_reasoning_effort="
            f"{self._mission_requirements_reasoning_effort or '<disabled>'}, "
            f"payload_reasoning_effort={self._payload_reasoning_effort or '<disabled>'}, "
            "payload_normalizer_reasoning_effort="
            f"{self._payload_normalizer_reasoning_effort or '<disabled>'})"
        )

    @staticmethod
    def _normalize_provider_name(provider_name: str) -> str:
        normalized = (provider_name or DEFAULT_LLM_PROVIDER).strip().lower()
        normalized = {
            'google': 'gemini',
            'google_genai': 'gemini',
            'google-genai': 'gemini',
            'open_ai': 'openai',
            'open_router': 'openrouter',
        }.get(normalized, normalized)
        if normalized not in SUPPORTED_LLM_PROVIDERS:
            supported = ", ".join(sorted(SUPPORTED_LLM_PROVIDERS))
            raise ValueError(
                f"Unsupported llm provider '{provider_name}'. Supported providers: {supported}."
            )
        return normalized

    def _provider_display_name(self, provider_name: str, model_name: str) -> str:
        return f"{provider_name}:{model_name}"

    def _reasoning_config(
        self, reasoning_effort: str = '', reasoning_summary: str = ''
    ) -> Optional[dict]:
        reasoning = {}
        effort = (reasoning_effort or '').strip()
        summary = (reasoning_summary or '').strip()
        if effort:
            reasoning['effort'] = effort
        if summary:
            reasoning['summary'] = summary
        return reasoning or None

    def _json_collection_count(self, payload: str, key: str = '') -> int:
        if not payload:
            return 0
        try:
            parsed = json.loads(payload)
        except Exception:
            return 0
        if key and isinstance(parsed, dict):
            parsed = parsed.get(key, [])
        if isinstance(parsed, (list, dict)):
            return len(parsed)
        return 0

    def _text_preview(self, text: str, limit: int = 160) -> str:
        normalized = re.sub(r'\s+', ' ', text or '').strip()
        if len(normalized) <= limit:
            return normalized
        return normalized[: limit - 3] + '...'

    def _create_chat_llm(
        self,
        provider_name: str,
        model_name: str,
        temperature: float,
        purpose: str,
        reasoning_effort: str = '',
        reasoning_summary: str = '',
    ):
        provider_name = self._normalize_provider_name(provider_name)
        reasoning = self._reasoning_config(reasoning_effort, reasoning_summary)

        if provider_name == 'gemini':
            if ChatGoogleGenerativeAI is None:
                raise RuntimeError(
                    "langchain-google-genai not found. Install it to use the gemini provider."
                )
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    f"GEMINI_API_KEY not set; export it to use the {purpose} gemini model."
                )
            return ChatGoogleGenerativeAI(
                model=model_name,
                temperature=temperature,
                api_key=api_key,
            )

        if provider_name == 'openai':
            if ChatOpenAI is None:
                raise RuntimeError(
                    "langchain-openai not found. Install it to use the openai provider."
                )
            if not os.environ.get('OPENAI_API_KEY'):
                raise RuntimeError(
                    f"OPENAI_API_KEY not set; export it to use the {purpose} openai model."
                )
            kwargs = {
                'model': model_name,
                'temperature': temperature,
            }
            if reasoning:
                kwargs['reasoning'] = reasoning
            return ChatOpenAI(**kwargs)

        if ChatOpenRouter is None:
            raise RuntimeError(
                "langchain-openrouter not found. Install it to use the openrouter provider."
            )
        if not os.environ.get('OPENROUTER_API_KEY'):
            raise RuntimeError(
                f"OPENROUTER_API_KEY not set; export it to use the {purpose} openrouter model."
            )
        kwargs = {
            'model': model_name,
            'temperature': temperature,
        }
        if reasoning:
            kwargs['reasoning'] = reasoning
        if self._openrouter_app_url:
            kwargs['app_url'] = self._openrouter_app_url
        if self._openrouter_app_title:
            kwargs['app_title'] = self._openrouter_app_title
        if purpose == 'selection':
            kwargs['timeout'] = max(1, int(math.ceil(self._selection_timeout_sec)))
        return ChatOpenRouter(**kwargs)

    # region Selection
    def handle_selection_request(
        self, request: SelectBehaviorTree.Request, response: SelectBehaviorTree.Response
    ) -> SelectBehaviorTree.Response:
        self.get_logger().info(
            'Received behavior tree selection request '
            f"(session={request.session_id or '<none>'}, "
            f"candidates={list(request.available_trees)})"
        )
        if len(request.available_trees) != len(request.tree_descriptions):
            response.status_code = self.SELECT_ERROR
            response.reason = (
                'available_trees and tree_descriptions size mismatch '
                f'({len(request.available_trees)} vs {len(request.tree_descriptions)})'
            )
            self.get_logger().warning(response.reason)
            return response

        selection = self._select_tree_via_llm(
            request.user_command, list(request.available_trees), list(request.tree_descriptions)
        )
        if selection is None:
            response.status_code = self.SELECT_NO_MATCH
            response.reason = 'No behavior tree matched the current command.'
            response.selected_tree = ''
            response.confidence = 0.0
            return response

        idx, confidence, reason = selection
        if idx is None:
            response.status_code = self.SELECT_NO_MATCH
            response.reason = 'No behavior tree matched the current command.'
            response.selected_tree = ''
            response.confidence = 0.0
            return response

        response.status_code = self.SELECT_FOUND
        response.selected_tree = request.available_trees[idx]
        response.confidence = confidence
        response.reason = reason
        self.get_logger().info(
            'Behavior tree selection completed '
            f"(session={request.session_id or '<none>'}, "
            f"selected={response.selected_tree}, confidence={response.confidence:.2f})"
        )
        return response

    def _select_tree_via_llm(
        self, user_command: str, tree_ids: List[str], descriptions: List[str]
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None

        if not self._selection_llm_enabled:
            self.get_logger().info(
                f'Selection LLM disabled; using fallback for candidates={tree_ids}'
            )
            return self._fallback_selection(tree_ids, descriptions, user_command)

        try:
            chain = self._get_selection_chain()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return self._fallback_selection(tree_ids, descriptions, user_command)

        payload = [
            {"id": tree_id, "description": desc}
            for tree_id, desc in zip(tree_ids, descriptions)
        ]

        self.get_logger().info(
            'Invoking behavior tree selection LLM '
            f"({self._provider_display_name(self._selection_provider, self._selection_model_name)}, "
            f"candidates={tree_ids}, timeout={self._selection_timeout_sec:.1f}s)"
        )
        try:
            llm_result = self._invoke_chain_with_timeout(
                chain,
                {
                    "user_command": user_command,
                    "trees_json": json.dumps(payload, ensure_ascii=False),
                },
                self._selection_timeout_sec,
                'behavior tree selection LLM',
            )
        except Exception as exc:
            self.get_logger().error(
                f"{self._provider_display_name(self._selection_provider, self._selection_model_name)} "
                f"selection failed: {exc}"
            )
            return self._fallback_selection(tree_ids, descriptions, user_command)

        raw_content = getattr(llm_result, 'content', llm_result)
        cleaned_content = self._prepare_llm_json_text(raw_content)
        try:
            parsed = json.loads(cleaned_content)
        except Exception as exc:
            self.get_logger().warning(
                "Unable to parse "
                f"{self._provider_display_name(self._selection_provider, self._selection_model_name)} "
                f"response '{raw_content}': {exc}"
            )
            return self._fallback_selection(tree_ids, descriptions, user_command)

        selected_id = parsed.get('tree_id')
        if selected_id not in tree_ids:
            self.get_logger().warning(
                f"{self._selection_provider} returned unknown tree '{selected_id}'; "
                "falling back."
            )
            return self._fallback_selection(tree_ids, descriptions, user_command)

        confidence = float(parsed.get('confidence', 0.6))
        rationale = parsed.get('rationale', '')
        idx = tree_ids.index(selected_id)
        reason = (
            f"[{self._provider_display_name(self._selection_provider, self._selection_model_name)}] "
            f"Selected tree '{selected_id}' "
            f"because {rationale or 'it best matched the mission context'}."
        )
        return idx, max(0.0, min(confidence, 1.0)), reason

    def _invoke_chain_with_timeout(
        self,
        chain,
        variables: dict,
        timeout_sec: float,
        label: str,
    ):
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        future = executor.submit(chain.invoke, variables)
        try:
            result = future.result(timeout=max(0.1, timeout_sec))
        except concurrent.futures.TimeoutError as exc:
            future.cancel()
            executor.shutdown(wait=False, cancel_futures=True)
            raise TimeoutError(f'{label} timed out after {timeout_sec:.1f}s') from exc
        except Exception:
            executor.shutdown(wait=False, cancel_futures=True)
            raise
        executor.shutdown(wait=False)
        return result

    def _strip_code_fence(self, text: str) -> str:
        stripped = text.strip()
        if stripped.startswith("```"):
            stripped = stripped.lstrip('`')
            # remove leading language tag if present
            parts = stripped.split('\n', 1)
            stripped = parts[1] if len(parts) > 1 else ''
            if stripped.endswith("```"):
                stripped = stripped.rsplit("```", 1)[0]
        return stripped.strip()

    def _prepare_llm_json_text(self, raw_content) -> str:
        text = self._llm_content_to_text(raw_content)
        cleaned = self._strip_code_fence(text)
        json_candidate = self._extract_json_snippet(cleaned)
        return json_candidate or cleaned

    def _llm_content_to_text(self, raw_content) -> str:
        if raw_content is None:
            return ''
        if isinstance(raw_content, str):
            return raw_content
        if isinstance(raw_content, list):
            parts = [self._llm_content_to_text(item) for item in raw_content]
            return '\n'.join(part for part in parts if part).strip()
        if isinstance(raw_content, dict):
            for key in ('text', 'output_text'):
                value = raw_content.get(key)
                if isinstance(value, str) and value.strip():
                    return value
            for key in ('content', 'parts'):
                nested = raw_content.get(key)
                if nested is None:
                    continue
                flattened = self._llm_content_to_text(nested)
                if flattened:
                    return flattened
            return str(raw_content)

        text_attr = getattr(raw_content, 'text', None)
        if isinstance(text_attr, str) and text_attr.strip():
            return text_attr
        content_attr = getattr(raw_content, 'content', None)
        if content_attr is not None and content_attr is not raw_content:
            flattened = self._llm_content_to_text(content_attr)
            if flattened:
                return flattened
        return str(raw_content)

    def _extract_json_snippet(self, text: str) -> str:
        candidate = (text or '').strip()
        if not candidate:
            return ''
        decoder = json.JSONDecoder()
        try:
            _, end = decoder.raw_decode(candidate)
            return candidate[:end]
        except Exception:
            pass

        for index, char in enumerate(candidate):
            if char not in '{[':
                continue
            try:
                _, end = decoder.raw_decode(candidate[index:])
                return candidate[index:index + end]
            except Exception:
                continue
        return ''

    def _get_selection_chain(self):
        if not hasattr(self, '_selection_chain'):
            llm = self._create_chat_llm(
                provider_name=self._selection_provider,
                model_name=self._selection_model_name,
                temperature=self._selection_temperature,
                purpose='selection',
                reasoning_effort=self._selection_reasoning_effort,
                reasoning_summary=self._selection_reasoning_summary,
            )
            template = PromptTemplate.from_template(self._selection_prompt_template)
            self._selection_chain = template | llm
        return self._selection_chain

    def _fallback_selection(
        self,
        tree_ids: List[str],
        descriptions: Optional[List[str]] = None,
        user_command: str = '',
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None
        descriptions = descriptions or [''] * len(tree_ids)
        command = (user_command or '').lower()
        command_tokens = {
            token for token in re.findall(r'[a-z0-9]+', command)
            if len(token) > 2
        }

        best_idx = 0
        best_score = -1
        for idx, tree_id in enumerate(tree_ids):
            haystack = f'{tree_id} {descriptions[idx] if idx < len(descriptions) else ""}'.lower()
            tokens = set(re.findall(r'[a-z0-9]+', haystack))
            score = len(command_tokens & tokens)
            if any(word in command for word in ('temperature', 'temp', 'thermal')):
                score += 5 if 'temperature' in haystack else 0
            if any(word in command for word in ('photo', 'picture', 'image', 'rgb', 'camera')):
                score += 5 if any(word in haystack for word in ('photo', 'photograph', 'image', 'rgb')) else 0
            if any(word in command for word in ('explore', 'survey', 'area', 'search')):
                score += 5 if 'explore' in haystack else 0
            if score > best_score:
                best_idx = idx
                best_score = score

        tree_id = tree_ids[best_idx]
        confidence = 0.65 if best_score > 0 else 0.5
        return best_idx, confidence, f"[heuristic] selected '{tree_id}'"

    # endregion

    # region Mission Requirement Extraction
    def handle_extract_mission_requirements(
        self,
        request: ExtractMissionRequirements.Request,
        response: ExtractMissionRequirements.Response,
    ) -> ExtractMissionRequirements.Response:
        capability_count = self._json_collection_count(
            request.capability_catalog_json, 'supported'
        )
        unsupported_count = self._json_collection_count(
            request.capability_catalog_json, 'unsupported'
        )
        tree_count = self._json_collection_count(request.tree_catalog_json, 'trees')
        self.get_logger().info(
            'Received mission requirement extraction request '
            f"(session={request.session_id or '<none>'}, "
            f"command='{self._text_preview(request.user_command)}', "
            f"context_bytes={len(request.context_json or '')}, "
            f"supported_capabilities={capability_count}, "
            f"unsupported_capabilities={unsupported_count}, trees={tree_count})"
        )
        requirements, reasoning = self._extract_requirements_via_llm(
            user_command=request.user_command,
            context_json=request.context_json,
            capability_catalog_json=request.capability_catalog_json,
            tree_catalog_json=request.tree_catalog_json,
        )
        if requirements is None:
            response.status_code = response.ERROR
            response.requirements_json = '{}'
            response.reasoning = reasoning or 'Mission requirement extraction failed.'
            self.get_logger().warning(
                'Mission requirement extraction request failed '
                f"(session={request.session_id or '<none>'}, reason='{response.reasoning}')"
            )
            return response

        response.status_code = response.SUCCESS
        response.requirements_json = json.dumps(requirements, ensure_ascii=False)
        response.reasoning = reasoning
        self.get_logger().info(
            'Mission requirement extraction request completed '
            f"(session={request.session_id or '<none>'}, "
            f"required_capabilities={requirements.get('required_capabilities', [])}, "
            f"mission_intents={requirements.get('mission_intents', [])}, "
            f"ambiguities={len(requirements.get('ambiguities', []) or [])}, "
            f"constraint_keys={sorted((requirements.get('constraints') or {}).keys())})"
        )
        return response

    def _extract_requirements_via_llm(
        self,
        user_command: str,
        context_json: str,
        capability_catalog_json: str,
        tree_catalog_json: str,
    ) -> Tuple[Optional[dict], str]:
        if not self._mission_requirements_llm_enabled:
            self.get_logger().info('Mission requirement LLM extraction is disabled.')
            return None, 'Mission requirement LLM extraction is disabled.'

        try:
            chain = self._get_mission_requirements_chain()
        except RuntimeError as exc:
            self.get_logger().warning(str(exc))
            return None, str(exc)

        provider_display = self._provider_display_name(
            self._mission_requirements_provider,
            self._mission_requirements_model_name,
        )
        self.get_logger().info(
            'Invoking mission requirement LLM '
            f"({provider_display}, "
            f"command_bytes={len(user_command or '')}, "
            f"context_bytes={len(context_json or '')}, "
            f"capability_catalog_bytes={len(capability_catalog_json or '')}, "
            f"tree_catalog_bytes={len(tree_catalog_json or '')})"
        )
        try:
            llm_result = chain.invoke(
                {
                    'user_command': user_command or '',
                    'context_json': context_json or '{}',
                    'capability_catalog_json': capability_catalog_json or '{}',
                    'tree_catalog_json': tree_catalog_json or '{}',
                }
            )
        except Exception as exc:
            message = (
                f"{provider_display} mission requirement extraction failed: {exc}"
            )
            self.get_logger().warning(message)
            return None, message

        raw_content = getattr(llm_result, 'content', llm_result)
        cleaned = self._prepare_llm_json_text(raw_content)
        self.get_logger().info(
            'Mission requirement LLM returned content '
            f"(raw_chars={len(self._llm_content_to_text(raw_content))}, "
            f"json_candidate_chars={len(cleaned)})"
        )
        try:
            parsed = json.loads(cleaned)
        except Exception as exc:
            message = f"Unable to parse mission requirements response '{raw_content}': {exc}"
            self.get_logger().warning(message)
            return None, message
        if not isinstance(parsed, dict):
            return None, 'Mission requirements response was not a JSON object.'

        requirements = self._normalize_requirements_payload(parsed)
        self.get_logger().info(
            'Normalized mission requirements '
            f"(required_capabilities={requirements['required_capabilities']}, "
            f"mission_intents={requirements['mission_intents']}, "
            f"ambiguities={requirements['ambiguities']}, "
            f"constraints={requirements['constraints']})"
        )
        reasoning = str(
            parsed.get('rationale')
            or parsed.get('reasoning')
            or 'Extracted mission requirements from user command.'
        )
        return requirements, reasoning

    def _normalize_requirements_payload(self, parsed: dict) -> dict:
        required_capabilities = parsed.get('required_capabilities', [])
        if not isinstance(required_capabilities, list):
            required_capabilities = []
        mission_intents = parsed.get('mission_intents', [])
        if not isinstance(mission_intents, list):
            mission_intents = []
        ambiguities = parsed.get('ambiguities', [])
        if not isinstance(ambiguities, list):
            ambiguities = []
        constraints = parsed.get('constraints', {})
        if not isinstance(constraints, dict):
            constraints = {}
        return {
            'required_capabilities': [
                str(item).strip() for item in required_capabilities if str(item).strip()
            ],
            'mission_intents': [
                str(item).strip() for item in mission_intents if str(item).strip()
            ],
            'constraints': constraints,
            'ambiguities': [
                str(item).strip() for item in ambiguities if str(item).strip()
            ],
            'rationale': str(parsed.get('rationale', '') or parsed.get('reasoning', '')),
        }

    def _get_mission_requirements_chain(self):
        if not hasattr(self, '_mission_requirements_chain'):
            llm = self._create_chat_llm(
                provider_name=self._mission_requirements_provider,
                model_name=self._mission_requirements_model_name,
                temperature=self._mission_requirements_temperature,
                purpose='mission requirement extraction',
                reasoning_effort=self._mission_requirements_reasoning_effort,
                reasoning_summary=self._mission_requirements_reasoning_summary,
            )
            template = PromptTemplate.from_template(
                self._mission_requirements_prompt_template
            )
            self._mission_requirements_chain = template | llm
        return self._mission_requirements_chain

    # endregion

    # region Payload Generation
    def handle_create_payload(
        self, request: CreatePayload.Request, response: CreatePayload.Response
    ) -> CreatePayload.Response:
        """Transform context snapshot into blackboard payload using LLM."""
        try:
            # Parse inputs
            context = json.loads(request.context_snapshot_json) if request.context_snapshot_json else {}
            contract = json.loads(request.subtree_contract_json) if request.subtree_contract_json else {}
            
            self.get_logger().info(
                f"CreatePayload request for subtree='{request.subtree_id}' (session={request.session_id})"
            )
            
            # Generate payload via LLM
            payload_dict = self._generate_payload_via_llm(
                context=context,
                contract=contract,
                subtree_id=request.subtree_id,
                user_command=request.user_command,
                attachment_uris=list(request.attachment_uris)
            )
            payload_errors = self._generated_payload_errors(
                request.subtree_id,
                payload_dict,
                contract,
                context,
            )
            if payload_errors:
                response.status_code = response.ERROR
                response.payload_json = "{}"
                response.reasoning = "Generated payload failed validation: " + "; ".join(payload_errors)
                response.tool_trace_json = "[]"
                self.get_logger().warning(response.reasoning)
                return response
            refinement_notes = self._extract_refinement_notes(context, request.user_command)
            refinement_history_json = self._extract_refinement_history_json(
                context, request.user_command
            )
            prior_reasoning = self._extract_prior_plan_reasoning(context, request.user_command)
            prior_payload_json = self._extract_prior_plan_payload_json(
                context, request.user_command
            )
            reasoning = f"Generated {len(payload_dict)} blackboard entries from context"
            if refinement_notes:
                reasoning += " with operator refinement feedback"
            if refinement_history_json:
                reasoning += " using session refinement history"
            if prior_reasoning or prior_payload_json:
                reasoning += " using the rejected prior plan as context"
            
            response.status_code = response.SUCCESS
            response.payload_json = json.dumps(payload_dict, ensure_ascii=False)
            response.reasoning = reasoning
            response.tool_trace_json = "[]"  # TODO: Add MCP tool traces when available
            
        except json.JSONDecodeError as e:
            response.status_code = response.ERROR
            response.payload_json = "{}"
            response.reasoning = f"Invalid JSON in request: {e}"
            self.get_logger().error(response.reasoning)
            
        except Exception as e:
            self.get_logger().error(f"CreatePayload failed: {e}")
            response.status_code = response.RETRY
            response.payload_json = "{}"
            response.reasoning = str(e)
        
        return response

    def _generate_payload_via_llm(
        self,
        context: dict,
        contract: dict,
        subtree_id: str,
        user_command: str,
        attachment_uris: list,
    ) -> dict:
        """Use LangChain to map context to blackboard entries."""
        
        # If LLM is disabled, use heuristic fallback
        if not self._selection_llm_enabled:
            return self._fallback_payload(context, contract)
        
        try:
            prompt_text = self._render_payload_prompt(
                subtree_id=subtree_id,
                user_command=user_command,
                context=context,
                contract=contract,
                attachment_uris=attachment_uris,
            )

            if self._payload_multimodal_enabled and attachment_uris:
                raw_content = self._invoke_multimodal_payload(prompt_text, attachment_uris)
            else:
                chain = self._get_payload_chain()
                llm_result = chain.invoke({"prompt": prompt_text})
                raw_content = getattr(llm_result, 'content', llm_result)

            cleaned = self._prepare_llm_json_text(raw_content)
            payload = self._safe_parse_payload(cleaned)
            if payload is None:
                if self._payload_schema_enforced:
                    payload = self._normalize_payload_with_llm(
                        prompt_text, contract, cleaned
                    )
                if payload is None:
                    raise ValueError("Payload was not valid JSON.")
            payload = self._coerce_payload_to_contract(payload, contract)

            if self._payload_schema_enforced:
                matches, errors = self._payload_matches_contract(payload, contract)
                if not matches:
                    schema_errors = "; ".join(errors) or "unknown issues"
                    self.get_logger().warning(
                        f"Payload failed schema check ({schema_errors}). Running normalizer."
                    )
                    normalized = self._normalize_payload_with_llm(
                        prompt_text, contract, cleaned
                    )
                    if normalized is None:
                        self.get_logger().warning(
                            "Payload normalizer failed; falling back to defaults."
                        )
                        return self._fallback_payload(context, contract)
                    payload = self._coerce_payload_to_contract(normalized, contract)
                    matches, errors = self._payload_matches_contract(payload, contract)
                    if not matches:
                        schema_errors = "; ".join(errors) or "unknown issues"
                        self.get_logger().warning(
                            f"Normalized payload still mismatched contract ({schema_errors}); "
                            "falling back to defaults."
                        )
                        return self._fallback_payload(context, contract)

            self.get_logger().info(f"LLM generated payload with {len(payload)} keys")
            return payload

        except Exception as e:
            self.get_logger().warning(f"LLM payload generation failed: {e}, using fallback")
            return self._fallback_payload(context, contract)

    def _fallback_payload(self, context: dict, contract: dict) -> dict:
        """Heuristic payload when LLM is unavailable."""
        payload = {}
        
        # Apply contract defaults
        for key, spec in contract.items():
            if isinstance(spec, dict) and 'default' in spec:
                payload[key] = spec['default']
        
        # Direct context passthrough for common keys
        if 'ROBOT_POSE' in context:
            payload['current_pose'] = context['ROBOT_POSE']
        if 'GPS_FIX' in context:
            payload['gps_location'] = context['GPS_FIX']
        payload = self._coerce_payload_to_contract(payload, contract)
        
        self.get_logger().info(f"Fallback payload generated with {len(payload)} keys")
        return payload

    def _coerce_payload_to_contract(self, payload: dict, contract: dict) -> dict:
        if not isinstance(payload, dict) or not isinstance(contract, dict):
            return payload
        coerced = dict(payload)
        waypoints_spec = contract.get('waypoints')
        if isinstance(waypoints_spec, dict):
            converted = self._coerce_waypoints_field(coerced)
            if converted is not None:
                coerced['waypoints'] = converted
        for key in ('frontiers',):
            if isinstance(contract.get(key), dict):
                converted = self._coerce_waypoints_value(coerced.get(key))
                if converted is not None:
                    coerced[key] = converted
        for key in ('area_polygon', 'area_polygon_geo', 'frontiers_geo'):
            if isinstance(contract.get(key), dict):
                converted = self._coerce_point_list_value(coerced.get(key))
                if converted is not None:
                    coerced[key] = converted
        return coerced

    def _coerce_waypoints_field(self, payload: dict) -> Optional[str]:
        if not isinstance(payload, dict):
            return None
        # Preferred key first, then common LLM variants.
        source = payload.get('waypoints')
        if source is None:
            for alt_key in ('route', 'points', 'targets', 'goal_points'):
                if alt_key in payload:
                    source = payload.get(alt_key)
                    break
        return self._coerce_waypoints_value(source)

    def _coerce_waypoints_value(self, value) -> Optional[str]:
        if value is None:
            return None
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return None
            # Already close to contract: normalize separators.
            normalized = text.replace('\n', ';').replace('|', ';')
            matches = re.findall(
                r'(-?\d+(?:\.\d+)?)\s*[,;]\s*(-?\d+(?:\.\d+)?)\s*[,;]\s*(-?\d+(?:\.\d+)?)',
                normalized,
            )
            if matches:
                return '; '.join(f"{x},{y},{yaw}" for x, y, yaw in matches)
            return None
        if isinstance(value, dict):
            nested = (
                value.get('waypoints')
                if 'waypoints' in value
                else value.get('points')
            )
            if nested is not None:
                return self._coerce_waypoints_value(nested)
            triplet = self._waypoint_triplet_from_item(value)
            if triplet is None:
                return None
            return f"{triplet[0]},{triplet[1]},{triplet[2]}"
        if isinstance(value, list):
            triples: List[str] = []
            for item in value:
                triplet = self._waypoint_triplet_from_item(item)
                if triplet is None:
                    continue
                triples.append(f"{triplet[0]},{triplet[1]},{triplet[2]}")
            if triples:
                return '; '.join(triples)
        return None

    def _waypoint_triplet_from_item(self, item) -> Optional[Tuple[str, str, str]]:
        if isinstance(item, str):
            converted = self._coerce_waypoints_value(item)
            if not converted:
                return None
            first = converted.split(';', 1)[0].strip()
            parts = [p.strip() for p in first.split(',')]
            if len(parts) != 3:
                return None
            return parts[0], parts[1], parts[2]
        if isinstance(item, dict):
            x = item.get('x', item.get('X'))
            y = item.get('y', item.get('Y'))
            if x is None or y is None:
                position = item.get('position')
                if isinstance(position, dict):
                    x = position.get('x', position.get('X', x))
                    y = position.get('y', position.get('Y', y))
            if x is None or y is None:
                pose = item.get('pose')
                if isinstance(pose, dict):
                    position = pose.get('position')
                    if isinstance(position, dict):
                        x = position.get('x', position.get('X', x))
                        y = position.get('y', position.get('Y', y))

            yaw = item.get('yaw', item.get('theta', item.get('heading')))
            if yaw is None:
                orientation = item.get('orientation')
                if not isinstance(orientation, dict):
                    pose = item.get('pose')
                    if isinstance(pose, dict):
                        orientation = pose.get('orientation')
                yaw = self._yaw_from_quaternion(orientation)
            if yaw is None:
                yaw = 0.0
            if x is None or y is None:
                return None
            return str(self._as_float(x, 0.0)), str(self._as_float(y, 0.0)), str(
                self._as_float(yaw, 0.0)
            )
        if isinstance(item, list) and len(item) >= 2:
            x = self._as_float(item[0], 0.0)
            y = self._as_float(item[1], 0.0)
            yaw = self._as_float(item[2], 0.0) if len(item) >= 3 else 0.0
            return str(x), str(y), str(yaw)
        return None

    def _coerce_point_list_value(self, value) -> Optional[str]:
        if value is None:
            return None
        if isinstance(value, str):
            text = value.strip()
            if not text:
                return None
            pairs: List[str] = []
            for token in text.replace('\n', ';').replace('|', ';').split(';'):
                pair = self._point_pair_from_item(token)
                if pair is not None:
                    pairs.append(f"{pair[0]},{pair[1]}")
            return '; '.join(pairs) if pairs else None
        if isinstance(value, dict):
            nested = value.get('points') or value.get('vertices') or value.get('coordinates')
            if nested is not None:
                return self._coerce_point_list_value(nested)
            pair = self._point_pair_from_item(value)
            if pair is None:
                return None
            return f"{pair[0]},{pair[1]}"
        if isinstance(value, list):
            pairs: List[str] = []
            for item in value:
                pair = self._point_pair_from_item(item)
                if pair is not None:
                    pairs.append(f"{pair[0]},{pair[1]}")
            return '; '.join(pairs) if pairs else None
        return None

    def _point_pair_from_item(self, item) -> Optional[Tuple[str, str]]:
        if isinstance(item, str):
            values = re.findall(r'-?\d+(?:\.\d+)?', item)
            if len(values) < 2:
                return None
            return str(self._as_float(values[0], 0.0)), str(self._as_float(values[1], 0.0))
        if isinstance(item, dict):
            source = item
            if isinstance(item.get('position'), dict):
                source = item['position']
            elif isinstance(item.get('pose'), dict):
                pose = item['pose']
                if isinstance(pose.get('position'), dict):
                    source = pose['position']
                else:
                    source = pose
            x = source.get('x', source.get('X'))
            y = source.get('y', source.get('Y'))
            if x is None or y is None:
                x = item.get('lat', item.get('latitude', source.get('lat', source.get('latitude'))))
                y = item.get(
                    'lon',
                    item.get(
                        'longitude',
                        item.get('lng', source.get('lon', source.get('longitude', source.get('lng')))),
                    ),
                )
            if x is None or y is None:
                return None
            return str(self._as_float(x, 0.0)), str(self._as_float(y, 0.0))
        if isinstance(item, list) and len(item) >= 2:
            return str(self._as_float(item[0], 0.0)), str(self._as_float(item[1], 0.0))
        return None

    @staticmethod
    def _as_float(value, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _yaw_from_quaternion(orientation) -> Optional[float]:
        if not isinstance(orientation, dict):
            return None
        if not all(k in orientation for k in ('x', 'y', 'z', 'w')):
            return None
        x = LLMInterfaceNode._as_float(orientation.get('x'), 0.0)
        y = LLMInterfaceNode._as_float(orientation.get('y'), 0.0)
        z = LLMInterfaceNode._as_float(orientation.get('z'), 0.0)
        w = LLMInterfaceNode._as_float(orientation.get('w'), 1.0)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_payload_chain(self):
        """Lazy-load the payload generation chain."""
        if not hasattr(self, '_payload_chain'):
            llm = self._create_chat_llm(
                provider_name=self._payload_provider,
                model_name=self._payload_model_name,
                temperature=0.0,
                purpose='payload',
                reasoning_effort=self._payload_reasoning_effort,
                reasoning_summary=self._payload_reasoning_summary,
            )
            template = PromptTemplate.from_template("""{prompt}""")
            self._payload_chain = template | llm
        return self._payload_chain

    def _get_payload_normalizer_chain(self):
        """Lazy-load the payload normalizer chain."""
        if not hasattr(self, '_payload_normalizer_chain'):
            llm = self._create_chat_llm(
                provider_name=self._payload_normalizer_provider,
                model_name=self._payload_normalizer_model_name,
                temperature=0.0,
                purpose='payload normalization',
                reasoning_effort=self._payload_normalizer_reasoning_effort,
                reasoning_summary=self._payload_normalizer_reasoning_summary,
            )
            template = PromptTemplate.from_template(self._payload_normalizer_prompt)
            self._payload_normalizer_chain = template | llm
        return self._payload_normalizer_chain

    def _get_payload_llm(self):
        if not hasattr(self, '_payload_llm'):
            self._payload_llm = self._create_chat_llm(
                provider_name=self._payload_provider,
                model_name=self._payload_model_name,
                temperature=0.0,
                purpose='payload',
                reasoning_effort=self._payload_reasoning_effort,
                reasoning_summary=self._payload_reasoning_summary,
            )
        return self._payload_llm

    def _safe_parse_payload(self, raw: str) -> Optional[dict]:
        try:
            payload = json.loads(raw)
        except Exception as exc:
            self.get_logger().warning(f"Payload JSON parse failed: {exc}")
            return None
        if not isinstance(payload, dict):
            self.get_logger().warning("Payload is not a JSON object.")
            return None
        return payload

    def _generated_payload_errors(
        self,
        subtree_id: str,
        payload: dict,
        contract: dict,
        context: dict,
    ) -> List[str]:
        matches, errors = self._payload_matches_contract(payload, contract)
        validation_errors = list(errors if not matches else [])
        if subtree_id == 'explore_area.xml':
            validation_errors.extend(self._explore_area_payload_errors(payload, context))
        return validation_errors

    def _explore_area_payload_errors(self, payload: dict, context: dict) -> List[str]:
        errors: List[str] = []
        if not isinstance(payload, dict):
            return ["explore_area payload is not an object"]
        if (
            self._has_satellite_context(context)
            and not isinstance(context.get('ANNOTATED_SLAM_MAP_IMAGE'), dict)
            and not self._has_map_execution_anchor(context)
        ):
            errors.append(
                "satellite-only exploration payloads require both GPS_FIX and ROBOT_POSE to anchor map-frame waypoints"
            )
        if self._coordinate_string_looks_geographic(payload.get('waypoints'), context):
            errors.append(
                "explore_area waypoints appear to be lat/lon values; waypoints must be executable map-frame x,y,yaw"
            )
        return errors

    @staticmethod
    def _has_satellite_context(context: dict) -> bool:
        return isinstance(context, dict) and isinstance(context.get('SATELLITE_MAP'), dict)

    @staticmethod
    def _has_map_execution_anchor(context: dict) -> bool:
        if not isinstance(context, dict):
            return False
        return isinstance(context.get('ROBOT_POSE'), dict) and isinstance(context.get('GPS_FIX'), dict)

    def _coordinate_string_looks_geographic(self, raw_points, context: dict) -> bool:
        bounds = self._satellite_bounds(context)
        if not bounds:
            return False
        points = self._parse_coordinate_pairs(raw_points)
        if not points:
            return False
        north, south, east, west = bounds
        return all(south <= x <= north and west <= y <= east for x, y in points)

    @staticmethod
    def _satellite_bounds(context: dict) -> Optional[Tuple[float, float, float, float]]:
        if not isinstance(context, dict):
            return None
        satellite = context.get('SATELLITE_MAP')
        if not isinstance(satellite, dict):
            return None
        metadata = satellite.get('map_metadata')
        if not isinstance(metadata, dict):
            metadata = {}
        bounds = metadata.get('bounds') or satellite.get('bounds')
        if not isinstance(bounds, dict):
            return None
        try:
            north = float(bounds.get('north'))
            south = float(bounds.get('south'))
            east = float(bounds.get('east'))
            west = float(bounds.get('west'))
        except (TypeError, ValueError):
            return None
        if north <= south or east <= west:
            return None
        return north, south, east, west

    def _parse_coordinate_pairs(self, raw_points) -> List[Tuple[float, float]]:
        normalized = self._coerce_point_list_value(raw_points)
        if not normalized:
            return []
        points: List[Tuple[float, float]] = []
        for token in normalized.split(';'):
            values = [part.strip() for part in token.split(',')]
            if len(values) < 2:
                continue
            try:
                points.append((float(values[0]), float(values[1])))
            except ValueError:
                continue
        return points

    def _payload_matches_contract(self, payload: dict, contract: dict) -> Tuple[bool, List[str]]:
        if not isinstance(contract, dict) or not contract:
            return True, []
        if not isinstance(payload, dict):
            return False, ["payload is not an object"]
        errors: List[str] = []
        for key, spec in contract.items():
            if not isinstance(spec, dict):
                continue
            is_required = bool(spec.get('required', False))
            if is_required and key not in payload:
                errors.append(f"missing required key '{key}'")
                continue
            if key not in payload:
                continue
            value = payload[key]
            schema = spec.get('schema')
            if isinstance(schema, dict):
                errors.extend(self._validate_schema(key, value, schema))
                continue
            expected_type = spec.get('type')
            if isinstance(expected_type, str) and not self._value_matches_type(value, expected_type):
                errors.append(f"key '{key}' expected {expected_type}")
        return len(errors) == 0, errors

    def _validate_schema(self, key: str, value, schema: dict) -> List[str]:
        errors: List[str] = []
        schema_type = schema.get('type')
        if isinstance(schema_type, str) and not self._value_matches_type(value, schema_type):
            errors.append(f"key '{key}' expected schema type {schema_type}")
            return errors
        if schema_type == 'string':
            pattern = schema.get('pattern')
            if isinstance(pattern, str):
                try:
                    import re
                    if not re.fullmatch(pattern, str(value).strip()):
                        errors.append(f"key '{key}' failed pattern match")
                except re.error as exc:
                    self.get_logger().warning(
                        f"Invalid regex pattern for key '{key}': {exc}"
                    )
        if schema_type == 'object' and isinstance(value, dict):
            required_keys = schema.get('required')
            if isinstance(required_keys, list):
                for req in required_keys:
                    if req not in value:
                        errors.append(f"key '{key}' missing required '{req}'")
            properties = schema.get('properties')
            if isinstance(properties, dict):
                for prop_name, prop_schema in properties.items():
                    if prop_name not in value or not isinstance(prop_schema, dict):
                        continue
                    prop_type = prop_schema.get('type')
                    if isinstance(prop_type, str) and not self._value_matches_type(
                        value[prop_name], prop_type
                    ):
                        errors.append(
                            f"key '{key}.{prop_name}' expected {prop_type}"
                        )
        if schema_type == 'array' and isinstance(value, list):
            item_schema = schema.get('items')
            if isinstance(item_schema, dict):
                for idx, item in enumerate(value):
                    item_errors = self._validate_schema(f"{key}[{idx}]", item, item_schema)
                    errors.extend(item_errors)
        return errors

    def _value_matches_type(self, value, expected_type: str) -> bool:
        normalized = expected_type.strip().lower()
        if normalized in ('bool', 'boolean'):
            return isinstance(value, bool)
        if normalized in ('int', 'integer'):
            return isinstance(value, int) and not isinstance(value, bool)
        if normalized in ('double', 'float', 'number'):
            return isinstance(value, (int, float)) and not isinstance(value, bool)
        if normalized == 'string':
            return isinstance(value, str)
        if normalized == 'array':
            return isinstance(value, list)
        if normalized == 'object':
            return isinstance(value, dict)
        return True

    def _normalize_payload_with_llm(
        self, prompt_text: str, contract: dict, raw_payload: str
    ) -> Optional[dict]:
        if not self._selection_llm_enabled:
            return None
        if not isinstance(contract, dict) or not contract:
            return None
        chain = self._get_payload_normalizer_chain()
        contract_json = json.dumps(contract, indent=2)
        for attempt in range(max(1, self._payload_schema_max_retries)):
            try:
                llm_result = chain.invoke(
                    {
                        "payload_prompt": prompt_text,
                        "contract_json": contract_json,
                        "raw_payload": raw_payload,
                    }
                )
            except Exception as exc:
                self.get_logger().warning(f"Payload normalizer failed: {exc}")
                continue
            cleaned = self._prepare_llm_json_text(getattr(llm_result, 'content', llm_result))
            normalized = self._safe_parse_payload(cleaned)
            if normalized is None:
                continue
            matches, errors = self._payload_matches_contract(normalized, contract)
            if matches:
                return normalized
            schema_errors = "; ".join(errors) or "unknown issues"
            self.get_logger().warning(
                f"Normalizer output still mismatched ({schema_errors})."
            )
        return None

    def _render_payload_prompt(
        self,
        subtree_id: str,
        user_command: str,
        context: dict,
        contract: dict,
        attachment_uris: list,
    ) -> str:
        attachment_summary = ", ".join(attachment_uris) if attachment_uris else "none"
        template = self._payload_prompts.get(subtree_id, self._payload_prompt_default)
        refinement_notes = self._extract_refinement_notes(context, user_command)
        refinement_history_json = self._extract_refinement_history_json(
            context, user_command
        )
        prior_plan_reasoning = self._extract_prior_plan_reasoning(context, user_command)
        prior_plan_payload_json = self._extract_prior_plan_payload_json(
            context, user_command
        )
        values = {
            'subtree_id': subtree_id,
            'user_command': user_command or '',
            'refinement_notes': refinement_notes or 'none',
            'refinement_history_json': refinement_history_json or '[]',
            'prior_plan_reasoning': prior_plan_reasoning or 'none',
            'prior_plan_payload_json': prior_plan_payload_json or 'none',
            'context_json': json.dumps(context, indent=2),
            'contract_json': json.dumps(contract, indent=2),
            'attachment_uris': attachment_summary,
        }
        try:
            return template.format(**values)
        except Exception as exc:
            self.get_logger().error(
                f"Invalid payload prompt template for subtree '{subtree_id}': {exc}. "
                "Falling back to default."
            )
            return self._payload_prompt_default.format(**values)

    def _extract_refinement_notes(self, context: dict, user_command: str) -> str:
        notes: List[str] = []
        if isinstance(context, dict):
            refinement = self._extract_mission_refinement_context(context)
            if isinstance(refinement, dict):
                operator_feedback = refinement.get('operator_feedback')
                if isinstance(operator_feedback, str) and operator_feedback.strip():
                    notes.append(operator_feedback.strip())
                prompt = refinement.get('prompt')
                if isinstance(prompt, str) and prompt.strip():
                    notes.append(f"Refinement prompt: {prompt.strip()}")
            direct_feedback = context.get('operator_feedback')
            if isinstance(direct_feedback, str) and direct_feedback.strip():
                notes.append(direct_feedback.strip())

        if isinstance(user_command, str) and 'OPERATOR_REFINEMENT_FEEDBACK:' in user_command:
            _, _, feedback_block = user_command.partition('OPERATOR_REFINEMENT_FEEDBACK:')
            cleaned_feedback = feedback_block.strip()
            if cleaned_feedback:
                notes.append(cleaned_feedback)

        deduped: List[str] = []
        seen = set()
        for note in notes:
            normalized = note.strip()
            if not normalized or normalized in seen:
                continue
            seen.add(normalized)
            deduped.append(normalized)
        return "\n\n".join(deduped)

    def _extract_prior_plan_reasoning(self, context: dict, user_command: str) -> str:
        reasoning_values: List[str] = []
        if isinstance(context, dict):
            refinement = self._extract_mission_refinement_context(context)
            if isinstance(refinement, dict):
                prior_reasoning = refinement.get('prior_reasoning')
                if isinstance(prior_reasoning, str) and prior_reasoning.strip():
                    reasoning_values.append(prior_reasoning.strip())

        extracted = self._extract_tagged_block(
            user_command, 'PRIOR_REJECTED_PLAN_REASONING:', 'PRIOR_REJECTED_PAYLOAD_JSON:'
        )
        if extracted:
            reasoning_values.append(extracted)
        return self._dedupe_joined_blocks(reasoning_values)

    def _extract_prior_plan_payload_json(self, context: dict, user_command: str) -> str:
        payload_values: List[str] = []
        if isinstance(context, dict):
            refinement = self._extract_mission_refinement_context(context)
            if isinstance(refinement, dict):
                prior_payload_json = refinement.get('prior_payload_json')
                if isinstance(prior_payload_json, str) and prior_payload_json.strip():
                    payload_values.append(prior_payload_json.strip())

        extracted = self._extract_tagged_block(
            user_command,
            'PRIOR_REJECTED_PAYLOAD_JSON:',
            'Refine the mission payload to address the rejected plan details above.',
        )
        if extracted:
            payload_values.append(extracted)
        return self._dedupe_joined_blocks(payload_values)

    def _extract_refinement_history_json(self, context: dict, user_command: str) -> str:
        history_entries = []
        if isinstance(context, dict):
            refinement = self._extract_mission_refinement_context(context)
            if isinstance(refinement, dict):
                history = refinement.get('history')
                if isinstance(history, list):
                    history_entries.extend(history)

        extracted = self._extract_tagged_block(
            user_command,
            'SESSION_REFINEMENT_HISTORY_JSON:',
            'Refine the mission payload to address the rejected plan details above.',
        )
        if extracted:
            try:
                parsed = json.loads(extracted)
            except Exception:
                pass
            else:
                if isinstance(parsed, list):
                    history_entries.extend(parsed)

        normalized_entries = []
        seen = set()
        for entry in history_entries:
            if not isinstance(entry, dict):
                continue
            serialized = json.dumps(entry, ensure_ascii=False, sort_keys=True)
            if serialized in seen:
                continue
            seen.add(serialized)
            normalized_entries.append(entry)
        if not normalized_entries:
            return ''
        return json.dumps(normalized_entries, ensure_ascii=False, indent=2)

    def _extract_mission_refinement_context(self, context: dict) -> Optional[dict]:
        if not isinstance(context, dict):
            return None
        request_hints = context.get('REQUEST_HINTS')
        if not isinstance(request_hints, dict):
            return None
        refinement = request_hints.get('MISSION_REFINEMENT')
        if not isinstance(refinement, dict):
            return None
        return refinement

    def _extract_tagged_block(
        self, text: str, start_tag: str, end_tag: str = ''
    ) -> str:
        if not isinstance(text, str) or start_tag not in text:
            return ''
        _, _, remainder = text.partition(start_tag)
        candidate = remainder.strip()
        if end_tag and end_tag in candidate:
            candidate, _, _ = candidate.partition(end_tag)
        return candidate.strip()

    def _dedupe_joined_blocks(self, values: List[str]) -> str:
        deduped: List[str] = []
        seen = set()
        for value in values:
            normalized = (value or '').strip()
            if not normalized or normalized in seen:
                continue
            seen.add(normalized)
            deduped.append(normalized)
        return "\n\n".join(deduped)

    def _invoke_multimodal_payload(self, prompt_text: str, attachment_uris: list) -> str:
        llm = self._get_payload_llm()
        parts = [{"type": "text", "text": prompt_text}]

        used = 0
        for uri in attachment_uris:
            if used >= self._payload_max_attachments:
                self.get_logger().warning(
                    f"Skipping attachment '{uri}' "
                    f"(payload_max_attachments={self._payload_max_attachments})"
                )
                continue
            image_part = self._attachment_to_image_part(uri)
            if image_part is None:
                continue
            parts.append(image_part)
            used += 1

        if used == 0:
            return llm.invoke(prompt_text).content

        message = HumanMessage(content=parts)
        result = llm.invoke([message])
        return getattr(result, 'content', result)

    def _attachment_to_image_part(self, uri: str) -> Optional[dict]:
        path = self._resolve_attachment_path(uri)
        if not path:
            return None

        mime, _ = mimetypes.guess_type(path.name)
        if mime not in self._payload_attachment_allowlist:
            self.get_logger().warning(
                f"Skipping attachment '{uri}' "
                f"(mime '{mime}' not in allowlist)"
            )
            return None

        data = path.read_bytes()
        if len(data) > self._payload_max_attachment_bytes:
            self.get_logger().warning(
                f"Skipping attachment '{uri}' "
                f"({len(data)} bytes exceeds limit {self._payload_max_attachment_bytes})"
            )
            return None

        encoded = base64.b64encode(data).decode('ascii')
        data_uri = f"data:{mime};base64,{encoded}"
        return {"type": "image_url", "image_url": {"url": data_uri}}

    def _resolve_attachment_path(self, uri: str) -> Optional[Path]:
        parsed = urlparse(uri)
        if parsed.scheme in ('', 'file'):
            raw_path = parsed.path if parsed.scheme == 'file' else uri
            path = Path(raw_path)
            if not path.is_absolute():
                path = (Path.cwd() / path).resolve()
            if not path.exists():
                self.get_logger().warning(f"Attachment path not found: {path}")
                return None
            return path

        self.get_logger().warning(
            f"Unsupported attachment URI scheme '{parsed.scheme}'"
        )
        return None

    # endregion

    # region Planning
    def handle_plan_request(
        self, request: PlanSubtree.Request, response: PlanSubtree.Response
    ) -> PlanSubtree.Response:
        response.status_code = self.PLAN_STATUS_SUCCESS
        response.bt_xml = self._synthesize_bt(request.mission_text)
        summary = (
            f"Generated default subtree for mission '{request.mission_text}'. "
            "For full LLM integration, replace this stub with LangChain pipelines."
        )
        response.summary = summary
        response.tool_invocations = '[]'
        response.reason = ''
        self.get_logger().info(
            f"PlanSubtree handled mission '{request.mission_text}' "
            f"(context bytes={len(request.context_snapshot)})"
        )
        return response

    def _synthesize_bt(self, mission_text: str) -> str:
        mission_comment = f"<!-- Mission: {mission_text} -->"
        if '{mission_text}' in self._plan_template:
            return self._plan_template.replace('{mission_text}', mission_text)
        return mission_comment + '\n' + self._plan_template

    # endregion


def main(args=None):
    rclpy.init(args=args)
    node = LLMInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LLMInterfaceNode interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ['LLMInterfaceNode', 'main']
