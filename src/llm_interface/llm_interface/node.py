from __future__ import annotations

import base64
import json
import mimetypes
import os
from pathlib import Path
from typing import List, Optional, Tuple
from urllib.parse import urlparse

import rclpy
from gen_bt_interfaces.srv import CreatePayload, PlanSubtree, SelectBehaviorTree
try:
    from langchain_core.prompts import PromptTemplate
    from langchain_core.messages import HumanMessage
    from langchain_google_genai import ChatGoogleGenerativeAI
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "langchain-core / langchain-google-genai not found. Install them in the "
        "ROS Python environment or disable selection_llm_enabled."
    ) from exc
from rclpy.node import Node

DEFAULT_SELECTION_PROMPT = (
    "You are a mission planner for a field robot. Given USER_COMMAND and a JSON array of\n"
    "behavior trees (each with id + description), choose the best tree or return NONE.\n"
    "tree_id MUST be exactly one of the string IDs in TREES_JSON (e.g., \"demo_tree.xml\").\n"
    "Do NOT answer with numeric indexes.\n"
    "Respond strictly as JSON: {{\"tree_id\": \"<id or NONE>\", \"confidence\": 0-1, \"rationale\": \"...\"}}\n"
    "USER_COMMAND: {user_command}\n"
    "TREES_JSON: {trees_json}\n"
)

DEFAULT_PAYLOAD_PROMPT = (
    "You are a robot mission planner. Given CONTEXT (sensor data) and CONTRACT "
    "(required blackboard keys), generate a JSON payload for the behavior tree.\n\n"
    "SUBTREE_ID: {subtree_id}\n\n"
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

        self.declare_parameter('planning_service_name', '/llm_interface/plan_subtree')
        self.declare_parameter('selection_service_name', '/llm_interface/select_behavior_tree')
        self.declare_parameter('create_payload_service_name', '/llm_interface/create_payload')
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
        self.declare_parameter('model_name', 'gemini-2.5-flash')
        self.declare_parameter('selection_llm_enabled', True)
        self.declare_parameter('selection_temperature', 0.0)
        self.declare_parameter('payload_multimodal_enabled', True)
        self.declare_parameter('payload_max_attachment_bytes', 5_000_000)
        self.declare_parameter('payload_max_attachments', 4)
        self.declare_parameter(
            'payload_attachment_allowlist', ['image/png', 'image/jpeg']
        )
        self.declare_parameter('payload_schema_enforced', False)
        self.declare_parameter('payload_schema_max_retries', 1)
        if not self.has_parameter('prompts.selection'):
            self.declare_parameter('prompts.selection', DEFAULT_SELECTION_PROMPT)
        if not self.has_parameter('prompts.payload.default'):
            self.declare_parameter('prompts.payload.default', DEFAULT_PAYLOAD_PROMPT)
        if not self.has_parameter('prompts.payload_normalizer'):
            self.declare_parameter(
                'prompts.payload_normalizer', DEFAULT_PAYLOAD_NORMALIZER_PROMPT
            )
        self._model_name = self.get_parameter('model_name').value
        self._selection_llm_enabled = bool(
            self.get_parameter('selection_llm_enabled').value
        )
        self._selection_temperature = float(
            self.get_parameter('selection_temperature').value
        )
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
        self._payload_prompt_default = str(
            self.get_parameter('prompts.payload.default').value
        )
        self._payload_normalizer_prompt = str(
            self.get_parameter('prompts.payload_normalizer').value
        )
        payload_prompts = self.get_parameters_by_prefix('prompts.payload')
        self._payload_prompts = {
            name: param.value
            for name, param in payload_prompts.items()
            if name != 'default' and isinstance(param.value, str)
        }

        planning_topic = self.get_parameter('planning_service_name').value
        selection_topic = self.get_parameter('selection_service_name').value
        payload_topic = self.get_parameter('create_payload_service_name').value
        self._plan_template = self.get_parameter('default_bt_template').value

        self._plan_service = self.create_service(
            PlanSubtree, planning_topic, self.handle_plan_request
        )
        self._selection_service = self.create_service(
            SelectBehaviorTree, selection_topic, self.handle_selection_request
        )
        self._payload_service = self.create_service(
            CreatePayload, payload_topic, self.handle_create_payload
        )
        self.get_logger().info(
            f"LLMInterfaceNode ready (plan={planning_topic}, select={selection_topic}, payload={payload_topic})"
        )

    # region Selection
    def handle_selection_request(
        self, request: SelectBehaviorTree.Request, response: SelectBehaviorTree.Response
    ) -> SelectBehaviorTree.Response:
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
        return response

    def _select_tree_via_llm(
        self, user_command: str, tree_ids: List[str], descriptions: List[str]
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None

        if not self._selection_llm_enabled:
            return self._fallback_selection(tree_ids)

        try:
            chain = self._get_selection_chain()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return self._fallback_selection(tree_ids)

        payload = [
            {"id": tree_id, "description": desc}
            for tree_id, desc in zip(tree_ids, descriptions)
        ]

        try:
            llm_result = chain.invoke(
                {
                    "user_command": user_command,
                    "trees_json": json.dumps(payload, ensure_ascii=False),
                }
            )
        except Exception as exc:
            self.get_logger().error(f"Gemini selection failed: {exc}")
            return self._fallback_selection(tree_ids)

        raw_content = getattr(llm_result, 'content', llm_result)
        cleaned_content = self._strip_code_fence(str(raw_content))
        try:
            parsed = json.loads(cleaned_content)
        except Exception as exc:
            self.get_logger().warning(
                f"Unable to parse Gemini response '{raw_content}': {exc}"
            )
            return self._fallback_selection(tree_ids)

        selected_id = parsed.get('tree_id')
        if selected_id not in tree_ids:
            self.get_logger().warning(
                f"Gemini returned unknown tree '{selected_id}'; falling back."
            )
            return self._fallback_selection(tree_ids)

        confidence = float(parsed.get('confidence', 0.6))
        rationale = parsed.get('rationale', '')
        idx = tree_ids.index(selected_id)
        reason = (
            f"[{self._model_name}] Selected tree '{selected_id}' "
            f"because {rationale or 'it best matched the mission context'}."
        )
        return idx, max(0.0, min(confidence, 1.0)), reason

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

    def _get_selection_chain(self):
        if not hasattr(self, '_selection_chain'):
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    'GEMINI_API_KEY not set; disable selection_llm_enabled or export the key.'
                )
            llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=self._selection_temperature,
                api_key=api_key,
            )
            template = PromptTemplate.from_template(self._selection_prompt_template)
            self._selection_chain = template | llm
        return self._selection_chain

    def _fallback_selection(
        self, tree_ids: List[str]
    ) -> Optional[Tuple[int, float, str]]:
        if not tree_ids:
            return None
        tree_id = tree_ids[0]
        return 0, 0.5, f"[heuristic] defaulted to '{tree_id}'"

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
                attachment_uris=list(request.attachment_uris)
            )
            
            response.status_code = response.SUCCESS
            response.payload_json = json.dumps(payload_dict, ensure_ascii=False)
            response.reasoning = f"Generated {len(payload_dict)} blackboard entries from context"
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
        self, context: dict, contract: dict, subtree_id: str, attachment_uris: list
    ) -> dict:
        """Use LangChain to map context to blackboard entries."""
        
        # If LLM is disabled, use heuristic fallback
        if not self._selection_llm_enabled:
            return self._fallback_payload(context, contract)
        
        try:
            prompt_text = self._render_payload_prompt(
                subtree_id=subtree_id,
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

            cleaned = self._strip_code_fence(str(raw_content))
            payload = self._safe_parse_payload(cleaned)
            if payload is None:
                if self._payload_schema_enforced:
                    payload = self._normalize_payload_with_llm(
                        prompt_text, contract, cleaned
                    )
                if payload is None:
                    raise ValueError("Payload was not valid JSON.")

            if self._payload_schema_enforced:
                matches, errors = self._payload_matches_contract(payload, contract)
                if not matches:
                    self.get_logger().warning(
                        "Payload failed schema check (%s). Running normalizer.",
                        "; ".join(errors) or "unknown issues",
                    )
                    normalized = self._normalize_payload_with_llm(
                        prompt_text, contract, cleaned
                    )
                    if normalized is None:
                        self.get_logger().warning(
                            "Payload normalizer failed; falling back to defaults."
                        )
                        return self._fallback_payload(context, contract)
                    payload = normalized

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
        
        self.get_logger().info(f"Fallback payload generated with {len(payload)} keys")
        return payload

    def _get_payload_chain(self):
        """Lazy-load the payload generation chain."""
        if not hasattr(self, '_payload_chain'):
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    'GEMINI_API_KEY not set; disable selection_llm_enabled or export the key.'
                )
            
            llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=0.0,  # Deterministic for payload generation
                api_key=api_key,
            )
            template = PromptTemplate.from_template("""{prompt}""")
            self._payload_chain = template | llm
        
        return self._payload_chain

    def _get_payload_normalizer_chain(self):
        """Lazy-load the payload normalizer chain."""
        if not hasattr(self, '_payload_normalizer_chain'):
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    'GEMINI_API_KEY not set; disable selection_llm_enabled or export the key.'
                )
            llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=0.0,
                api_key=api_key,
            )
            template = PromptTemplate.from_template(self._payload_normalizer_prompt)
            self._payload_normalizer_chain = template | llm
        return self._payload_normalizer_chain

    def _get_payload_llm(self):
        if not hasattr(self, '_payload_llm'):
            api_key = os.environ.get('GEMINI_API_KEY')
            if not api_key:
                raise RuntimeError(
                    'GEMINI_API_KEY not set; disable selection_llm_enabled or export the key.'
                )
            self._payload_llm = ChatGoogleGenerativeAI(
                model=self._model_name,
                temperature=0.0,
                api_key=api_key,
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
                        "Invalid regex pattern for key '%s': %s", key, exc
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
            cleaned = self._strip_code_fence(str(getattr(llm_result, 'content', llm_result)))
            normalized = self._safe_parse_payload(cleaned)
            if normalized is None:
                continue
            matches, errors = self._payload_matches_contract(normalized, contract)
            if matches:
                return normalized
            self.get_logger().warning(
                "Normalizer output still mismatched (%s).",
                "; ".join(errors) or "unknown issues",
            )
        return None

    def _render_payload_prompt(
        self, subtree_id: str, context: dict, contract: dict, attachment_uris: list
    ) -> str:
        attachment_summary = ", ".join(attachment_uris) if attachment_uris else "none"
        template = self._payload_prompts.get(subtree_id, self._payload_prompt_default)
        values = {
            'subtree_id': subtree_id,
            'context_json': json.dumps(context, indent=2),
            'contract_json': json.dumps(contract, indent=2),
            'attachment_uris': attachment_summary,
        }
        try:
            return template.format(**values)
        except Exception as exc:
            self.get_logger().error(
                "Invalid payload prompt template for subtree '%s': %s. Falling back to default.",
                subtree_id,
                exc,
            )
            return self._payload_prompt_default.format(**values)

    def _invoke_multimodal_payload(self, prompt_text: str, attachment_uris: list) -> str:
        llm = self._get_payload_llm()
        parts = [{"type": "text", "text": prompt_text}]

        used = 0
        for uri in attachment_uris:
            if used >= self._payload_max_attachments:
                self.get_logger().warning(
                    "Skipping attachment '%s' (payload_max_attachments=%d)",
                    uri,
                    self._payload_max_attachments,
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
                "Skipping attachment '%s' (mime '%s' not in allowlist)",
                uri,
                mime,
            )
            return None

        data = path.read_bytes()
        if len(data) > self._payload_max_attachment_bytes:
            self.get_logger().warning(
                "Skipping attachment '%s' (%d bytes exceeds limit %d)",
                uri,
                len(data),
                self._payload_max_attachment_bytes,
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
                self.get_logger().warning("Attachment path not found: %s", path)
                return None
            return path

        self.get_logger().warning("Unsupported attachment URI scheme '%s'", parsed.scheme)
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
            "PlanSubtree handled mission '%s' (context bytes=%d)",
            request.mission_text,
            len(request.context_snapshot),
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
