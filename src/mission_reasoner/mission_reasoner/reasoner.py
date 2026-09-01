from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

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
        try:
            import yaml
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                'PyYAML is required only when loading MissionReasoner from YAML.'
            ) from exc
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

        context_admission = self._validate_context_admission(context)
        if context_admission is not None:
            context_admission.matched_capabilities = sorted(
                requested & self.supported_capabilities
            )
            context_admission.reasoning['debug_info'] = debug_info
            return context_admission

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
        context_keys = self._context_key_inventory(context)
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

    @classmethod
    def _context_key_inventory(cls, context: Any) -> set[str]:
        """Return semantic keys from the full structured context envelope."""
        keys: set[str] = set()

        def visit(value: Any) -> None:
            if isinstance(value, Mapping):
                for raw_key, nested in value.items():
                    key = str(raw_key).strip().lower()
                    if key:
                        keys.add(key)
                    visit(nested)
            elif isinstance(value, list):
                for item in value:
                    if isinstance(item, str):
                        keys.add(item.strip().lower())
                    else:
                        visit(item)

        visit(context)
        if keys.intersection({'routes', 'route_definition', 'ordered_waypoints'}):
            keys.update({'route', 'waypoints'})
        if keys.intersection({'named_points', 'waypoint', 'waypoints'}):
            keys.add('waypoints')
        if keys.intersection({'target_areas', 'area_polygon', 'area_polygon_geo'}):
            keys.add('target_area')
        return keys

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

    def _validate_context_admission(
        self, context: Any
    ) -> Optional[ValidationResult]:
        if not isinstance(context, Mapping):
            return None

        if context.get('evidence_quality') == 'insufficient_text_only':
            return ValidationResult(
                status_code=CLARIFY,
                message='The requested named area has no spatial definition.',
                clarification_question=(
                    'What boundary, polygon, radius, or named map area should be used?'
                ),
                reasoning={'guard': 'context.area_definition'},
            )

        route_length = self._context_number(
            context, 'route_length_m', 'ROUTE_LENGTH_M'
        )
        context_max_range = self._context_number(
            context, 'platform_max_range_m', 'PLATFORM_MAX_RANGE_M'
        )
        max_range = context_max_range or self._float_platform_value('max_range_m')
        if (
            route_length is not None
            and max_range is not None
            and route_length > max_range
        ):
            return ValidationResult(
                status_code=REFUSE,
                message=(
                    f'Mission route ({route_length:g} m) exceeds platform '
                    f'endurance ({max_range:g} m).'
                ),
                reasoning={
                    'guard': 'context.range',
                    'route_length_m': route_length,
                    'max_range_m': max_range,
                },
                missing_capabilities=['platform.range'],
            )

        battery = self._context_number(context, 'battery_percent', 'BATTERY_PERCENT')
        battery_state = self._context_mapping(
            context, 'battery_state', 'BATTERY_STATE'
        )
        if battery is None and battery_state:
            battery = self._mapping_number(battery_state, 'percentage', 'percent')
        threshold = self._context_number(
            context,
            'minimum_start_battery_percent',
            'MINIMUM_START_BATTERY_PERCENT',
        )
        if (
            battery is not None
            and threshold is not None
            and battery < threshold
        ):
            return ValidationResult(
                status_code=REFUSE,
                message=(
                    f'Current battery ({battery:g}%) is below the declared '
                    f'{threshold:g}% mission admission threshold.'
                ),
                reasoning={
                    'guard': 'context.battery_admission',
                    'battery_percent': battery,
                    'minimum_start_battery_percent': threshold,
                },
                missing_capabilities=['platform.energy'],
            )

        map_point = self._context_mapping(
            context, 'map_checkpoint_K', 'MAP_CHECKPOINT_K'
        )
        gps_point = self._context_mapping(
            context,
            'gps_checkpoint_K_converted_to_map',
            'GPS_CHECKPOINT_K_CONVERTED_TO_MAP',
        )
        tolerance = self._context_number(
            context,
            'allowed_position_disagreement_m',
            'ALLOWED_POSITION_DISAGREEMENT_M',
        )
        if map_point and gps_point and tolerance is not None:
            distance = self._point_distance(map_point, gps_point)
            if distance is not None and distance > tolerance:
                return ValidationResult(
                    status_code=CLARIFY,
                    message='Map and GPS evidence disagree for checkpoint K.',
                    clarification_question=(
                        f'Checkpoint K differs by {distance:.1f} m between map and '
                        'GPS evidence. Which source should be corrected?'
                    ),
                    reasoning={
                        'guard': 'context.cross_source_consistency',
                        'disagreement_m': distance,
                        'allowed_disagreement_m': tolerance,
                    },
                )

        p9 = self._context_mapping(context, 'P9')
        allowed_polygon = context.get('allowed_polygon') or context.get(
            'ALLOWED_POLYGON'
        )
        if p9 and isinstance(allowed_polygon, list):
            point = self._xy(p9)
            if point is not None and not self._point_in_polygon(
                point, allowed_polygon
            ):
                return ValidationResult(
                    status_code=REFUSE,
                    message='P9 lies outside the allowed mission polygon.',
                    reasoning={
                        'guard': 'context.geofence',
                        'point': list(point),
                        'allowed_polygon': allowed_polygon,
                    },
                    missing_capabilities=['navigation.safe_target'],
                )

        slam = self._context_mapping(
            context, 'annotated_slam_map', 'ANNOTATED_SLAM_MAP_IMAGE'
        )
        osm = self._context_mapping(context, 'osm_context', 'OSM_CONTEXT')
        tolerance = self._context_number(
            context, 'cross_source_tolerance_m', 'CROSS_SOURCE_TOLERANCE_M'
        )
        slam_polygon = slam.get('derived_area_polygon') if slam else None
        osm_polygon = osm.get('area_polygon') if osm else None
        if (
            isinstance(slam_polygon, list)
            and isinstance(osm_polygon, list)
            and tolerance is not None
        ):
            slam_center = self._polygon_center(slam_polygon)
            osm_center = self._polygon_center(osm_polygon)
            if slam_center is not None and osm_center is not None:
                distance = math.dist(slam_center, osm_center)
                if distance > tolerance:
                    return ValidationResult(
                        status_code=CLARIFY,
                        message='Spatial context sources disagree.',
                        clarification_question=(
                            f'The map sources disagree by {distance:.1f} m. '
                            'Which area definition is authoritative?'
                        ),
                        reasoning={
                            'guard': 'context.cross_source_consistency',
                            'disagreement_m': distance,
                            'allowed_disagreement_m': tolerance,
                        },
                    )
        return None

    @staticmethod
    def _context_mapping(
        context: Mapping[str, Any], *keys: str
    ) -> Mapping[str, Any]:
        for key in keys:
            value = context.get(key)
            if isinstance(value, Mapping):
                return value
        return {}

    @staticmethod
    def _mapping_number(
        mapping: Mapping[str, Any], *keys: str
    ) -> Optional[float]:
        for key in keys:
            try:
                return float(mapping.get(key))
            except (TypeError, ValueError):
                continue
        return None

    @classmethod
    def _context_number(
        cls, context: Mapping[str, Any], *keys: str
    ) -> Optional[float]:
        return cls._mapping_number(context, *keys)

    @staticmethod
    def _xy(mapping: Mapping[str, Any]) -> Optional[tuple[float, float]]:
        try:
            return float(mapping['x']), float(mapping['y'])
        except (KeyError, TypeError, ValueError):
            return None

    @classmethod
    def _point_distance(
        cls, first: Mapping[str, Any], second: Mapping[str, Any]
    ) -> Optional[float]:
        first_xy = cls._xy(first)
        second_xy = cls._xy(second)
        if first_xy is None or second_xy is None:
            return None
        return math.dist(first_xy, second_xy)

    @staticmethod
    def _point_in_polygon(
        point: tuple[float, float], polygon: Sequence[Sequence[float]]
    ) -> bool:
        try:
            vertices = [(float(item[0]), float(item[1])) for item in polygon]
        except (IndexError, TypeError, ValueError):
            return False
        if len(vertices) < 3:
            return False
        x, y = point
        inside = False
        previous_x, previous_y = vertices[-1]
        for current_x, current_y in vertices:
            cross = (x - current_x) * (previous_y - current_y) - (
                y - current_y
            ) * (previous_x - current_x)
            if (
                abs(cross) < 1e-9
                and min(current_x, previous_x) - 1e-9
                <= x
                <= max(current_x, previous_x) + 1e-9
                and min(current_y, previous_y) - 1e-9
                <= y
                <= max(current_y, previous_y) + 1e-9
            ):
                return True
            if (current_y > y) != (previous_y > y):
                crossing_x = (
                    (previous_x - current_x)
                    * (y - current_y)
                    / (previous_y - current_y)
                    + current_x
                )
                if x < crossing_x:
                    inside = not inside
            previous_x, previous_y = current_x, current_y
        return inside

    @staticmethod
    def _polygon_center(
        polygon: Sequence[Sequence[float]],
    ) -> Optional[tuple[float, float]]:
        try:
            vertices = [(float(item[0]), float(item[1])) for item in polygon]
        except (IndexError, TypeError, ValueError):
            return None
        if not vertices:
            return None
        return (
            sum(item[0] for item in vertices) / len(vertices),
            sum(item[1] for item in vertices) / len(vertices),
        )

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
