from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

import yaml


ACCEPT = 0
CLARIFY = 1
REFUSE = 2
ERROR = 3


@dataclass
class ValidationResult:
    status_code: int
    message: str
    clarification_question: str = ''
    reasoning: Dict[str, Any] = field(default_factory=dict)
    matched_capabilities: List[str] = field(default_factory=list)
    missing_capabilities: List[str] = field(default_factory=list)
    candidate_trees: List[str] = field(default_factory=list)


class MissionReasoner:
    """Deterministic capability gate for mission commands."""

    def __init__(self, system_description: Mapping[str, Any]) -> None:
        self.system_description = dict(system_description or {})
        capabilities = self.system_description.get('capabilities', {})
        self.supported_capabilities = set(capabilities.get('supported', []) or [])
        self.unsupported_capabilities = dict(capabilities.get('unsupported', {}) or {})
        self.command_rules = list(
            self.system_description.get('command_capability_rules', []) or []
        )
        self.clarification_rules = list(
            self.system_description.get('clarification_rules', []) or []
        )
        self.platform = dict(self.system_description.get('platform', {}) or {})

    @classmethod
    def from_yaml_file(cls, path: str) -> 'MissionReasoner':
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return cls(data)

    def validate(
        self,
        user_command: str,
        tree_catalog: Sequence[Mapping[str, Any]],
        context_json: str = '',
        extracted_requirements: Optional[Mapping[str, Any]] = None,
    ) -> ValidationResult:
        command = (user_command or '').strip()
        if not command:
            return ValidationResult(
                status_code=CLARIFY,
                message='Mission command is empty.',
                clarification_question='What mission should the robot perform?',
            )

        context = self._parse_context(context_json)
        requested_from_command = self._capabilities_from_command(command)
        requested = set(requested_from_command)
        llm_requirements = dict(extracted_requirements or {})
        requested_from_llm = self._capabilities_from_extracted_requirements(
            llm_requirements
        )
        requested.update(requested_from_llm)
        debug_info = {
            'context_keys': sorted(context.keys()) if isinstance(context, dict) else [],
            'tree_count': len(tree_catalog),
            'tree_ids': [
                str(tree.get('id', '')).strip()
                for tree in tree_catalog
                if tree.get('id')
            ],
            'requested_from_command_rules': sorted(requested_from_command),
            'requested_from_llm': sorted(requested_from_llm),
        }
        explicit_missing = sorted(
            capability
            for capability in requested
            if capability in self.unsupported_capabilities
            or capability not in self.supported_capabilities
        )
        if explicit_missing:
            result = self._refusal(
                command, requested, explicit_missing, llm_requirements
            )
            result.reasoning['debug_info'] = debug_info
            return result

        clarification = self._clarification_question(command, context)
        if clarification:
            return ValidationResult(
                status_code=CLARIFY,
                message='Mission needs clarification before behavior-tree selection.',
                clarification_question=clarification,
                reasoning={
                    'requested_capabilities': sorted(requested),
                    'llm_requirements': llm_requirements,
                    'context_keys': sorted(context.keys()) if isinstance(context, dict) else [],
                    'debug_info': debug_info,
                },
                matched_capabilities=sorted(requested & self.supported_capabilities),
            )

        candidates = self._candidate_trees(requested, tree_catalog)
        if not candidates:
            return ValidationResult(
                status_code=REFUSE,
                message='No behavior tree in the catalogue satisfies the requested capabilities.',
                reasoning={
                    'requested_capabilities': sorted(requested),
                    'llm_requirements': llm_requirements,
                    'available_trees': [
                        tree.get('id', '') for tree in tree_catalog if tree.get('id')
                    ],
                    'debug_info': debug_info,
                },
                matched_capabilities=sorted(requested & self.supported_capabilities),
                missing_capabilities=sorted(requested),
            )

        limit_refusal = self._validate_static_limits(command, llm_requirements)
        if limit_refusal:
            limit_refusal.matched_capabilities = sorted(
                requested & self.supported_capabilities
            )
            limit_refusal.candidate_trees = candidates
            limit_refusal.reasoning['debug_info'] = debug_info
            return limit_refusal

        return ValidationResult(
            status_code=ACCEPT,
            message='Mission is compatible with declared robot capabilities.',
            reasoning={
                'requested_capabilities': sorted(requested),
                'llm_requirements': llm_requirements,
                'candidate_trees': candidates,
                'debug_info': debug_info,
            },
            matched_capabilities=sorted(requested & self.supported_capabilities),
            candidate_trees=candidates,
        )

    def _capabilities_from_command(self, command: str) -> set[str]:
        normalized = self._normalize(command)
        capabilities: set[str] = set()
        for rule in self.command_rules:
            capability = str(rule.get('capability', '')).strip()
            if not capability:
                continue
            keywords = rule.get('keywords', []) or []
            if any(self._keyword_matches(normalized, keyword) for keyword in keywords):
                capabilities.add(capability)
        return capabilities

    def _capabilities_from_extracted_requirements(
        self, extracted_requirements: Mapping[str, Any]
    ) -> set[str]:
        capabilities: set[str] = set()
        raw_capabilities = extracted_requirements.get('required_capabilities', [])
        if not isinstance(raw_capabilities, list):
            return capabilities
        for item in raw_capabilities:
            capability = str(item).strip()
            if capability:
                capabilities.add(capability)
        return capabilities

    def _candidate_trees(
        self, requested_capabilities: set[str], tree_catalog: Sequence[Mapping[str, Any]]
    ) -> List[str]:
        candidates: List[str] = []
        for tree in tree_catalog:
            tree_id = str(tree.get('id', '')).strip()
            if not tree_id:
                continue
            required = set(tree.get('required_capabilities', []) or [])
            if not required:
                candidates.append(tree_id)
                continue
            if (
                requested_capabilities
                and requested_capabilities.issubset(required)
                and required.issubset(self.supported_capabilities)
            ):
                candidates.append(tree_id)
                continue
            if not requested_capabilities and required.issubset(self.supported_capabilities):
                candidates.append(tree_id)
        return candidates

    def _refusal(
        self,
        command: str,
        requested: set[str],
        missing: Sequence[str],
        llm_requirements: Optional[Mapping[str, Any]] = None,
    ) -> ValidationResult:
        explanations = []
        for capability in missing:
            explanations.append(
                self.unsupported_capabilities.get(
                    capability, f'Capability {capability} is not declared as supported.'
                )
            )
        message = 'Cannot execute mission: ' + ' '.join(explanations)
        return ValidationResult(
            status_code=REFUSE,
            message=message,
            reasoning={
                'command': command,
                'requested_capabilities': sorted(requested),
                'llm_requirements': dict(llm_requirements or {}),
                'unsupported_explanations': explanations,
            },
            matched_capabilities=sorted(requested & self.supported_capabilities),
            missing_capabilities=sorted(missing),
        )

    def _clarification_question(self, command: str, context: Any) -> str:
        normalized = self._normalize(command)
        context_keys = set(context.keys()) if isinstance(context, dict) else set()
        for rule in self.clarification_rules:
            keywords = rule.get('keywords', []) or []
            if not any(self._keyword_matches(normalized, keyword) for keyword in keywords):
                continue
            if bool(rule.get('requires_area_definition', False)):
                if self._has_area_definition(command, context):
                    continue
                return str(rule.get('question', '')).strip()
            required_keys = set(rule.get('requires_any_context_keys', []) or [])
            if required_keys and context_keys.intersection(required_keys):
                continue
            return str(rule.get('question', '')).strip()
        return ''

    def _has_area_definition(self, command: str, context: Any) -> bool:
        if isinstance(context, Mapping):
            area_keys = {
                'target_area',
                'area_polygon',
                'area_polygon_geo',
                'field_boundary',
                'mission_boundary',
                'boundary',
                'bounds',
                'bbox',
                'geo_hint',
            }
            if any(key in context for key in area_keys):
                return True
            request_hints = context.get('REQUEST_HINTS')
            if isinstance(request_hints, Mapping) and any(key in request_hints for key in area_keys):
                return True

        normalized = self._normalize(command)
        coordinate_pairs = re.findall(
            r'-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?',
            command or '',
        )
        if len(coordinate_pairs) >= 3:
            return True
        if re.search(r'\b\d+(?:\.\d+)?\s*(m|meter|meters)\s*(radius|around|from|within)\b', normalized):
            return True
        if re.search(r'\b(radius|within|inside)\s+\d+(?:\.\d+)?\s*(m|meter|meters)\b', normalized):
            return True
        if any(
            phrase in normalized
            for phrase in (
                'polygon',
                'rectangle',
                'bounded by',
                'inside the',
                'within the',
                'around the',
                'field',
                'parking lot',
                'courtyard',
                'orchard',
                'garden',
                'loading bay',
                'zone',
            )
        ):
            return True
        return False

    def _validate_static_limits(
        self, command: str, extracted_requirements: Mapping[str, Any]
    ) -> Optional[ValidationResult]:
        max_range = self._float_platform_value('max_range_m')
        if max_range is None:
            return None
        requested_range = self._extract_range_m(command)
        extracted_range = self._extract_constraint_float(
            extracted_requirements, 'range_m'
        )
        if extracted_range is not None:
            requested_range = max(requested_range or 0.0, extracted_range)
        if requested_range is not None and requested_range > max_range:
            return ValidationResult(
                status_code=REFUSE,
                message=(
                    f'Mission range ({requested_range:g} m) exceeds platform '
                    f'endurance ({max_range:g} m).'
                ),
                reasoning={
                    'requested_range_m': requested_range,
                    'max_range_m': max_range,
                },
                missing_capabilities=['platform.range'],
            )
        return None

    def _extract_constraint_float(
        self, extracted_requirements: Mapping[str, Any], key: str
    ) -> Optional[float]:
        constraints = extracted_requirements.get('constraints', {})
        if not isinstance(constraints, Mapping):
            return None
        value = constraints.get(key)
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _float_platform_value(self, key: str) -> Optional[float]:
        value = self.platform.get(key)
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _extract_range_m(self, command: str) -> Optional[float]:
        normalized = self._normalize(command)
        matches = re.findall(r'(\d+(?:\.\d+)?)\s*(km|kilometer|kilometers|m|meter|meters)', normalized)
        if not matches:
            return None
        distances = []
        for raw_value, unit in matches:
            value = float(raw_value)
            if unit.startswith('km') or unit.startswith('kilometer'):
                value *= 1000.0
            distances.append(value)
        return max(distances) if distances else None

    def _parse_context(self, context_json: str) -> Any:
        if not context_json:
            return {}
        try:
            parsed = json.loads(context_json)
        except Exception:
            return {'raw_context': context_json}
        return parsed if parsed is not None else {}

    def _normalize(self, text: str) -> str:
        return re.sub(r'\s+', ' ', text.lower()).strip()

    def _keyword_matches(self, normalized_text: str, keyword: str) -> bool:
        normalized_keyword = self._normalize(str(keyword))
        if not normalized_keyword:
            return False
        if ' ' in normalized_keyword:
            return normalized_keyword in normalized_text
        return re.search(rf'\b{re.escape(normalized_keyword)}\b', normalized_text) is not None


def parse_tree_catalog(tree_catalog_json: str) -> List[Dict[str, Any]]:
    if not tree_catalog_json:
        return []
    parsed = json.loads(tree_catalog_json)
    if isinstance(parsed, dict):
        parsed = parsed.get('trees', [])
    if not isinstance(parsed, list):
        raise ValueError('tree_catalog_json must be a list or object with a trees list')
    return [dict(item) for item in parsed if isinstance(item, dict)]
