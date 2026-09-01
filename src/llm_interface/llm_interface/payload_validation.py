"""Pure payload-contract checks shared by the ROS node and offline evaluation."""

from __future__ import annotations

import math
import re
from typing import Any, Mapping, Optional


def value_matches_type(value: Any, expected_type: str) -> bool:
    normalized = expected_type.strip().lower()
    if normalized in ("bool", "boolean"):
        return isinstance(value, bool)
    if normalized in ("int", "integer"):
        return isinstance(value, int) and not isinstance(value, bool)
    if normalized in ("double", "float", "number"):
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if normalized == "string":
        return isinstance(value, str)
    if normalized == "array":
        return isinstance(value, list)
    if normalized == "object":
        return isinstance(value, dict)
    return True


def validate_schema(key: str, value: Any, schema: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    schema_type = schema.get("type")
    if isinstance(schema_type, str) and not value_matches_type(value, schema_type):
        return [f"key '{key}' expected schema type {schema_type}"]
    if schema_type == "string" and isinstance(schema.get("pattern"), str):
        try:
            if not re.fullmatch(str(schema["pattern"]), str(value).strip()):
                errors.append(f"key '{key}' failed pattern match")
        except re.error as exc:
            errors.append(f"key '{key}' has invalid contract regex: {exc}")
    if schema_type == "object" and isinstance(value, dict):
        for required in schema.get("required", []) or []:
            if required not in value:
                errors.append(f"key '{key}' missing required '{required}'")
        for name, child_schema in (schema.get("properties", {}) or {}).items():
            if name in value and isinstance(child_schema, Mapping):
                errors.extend(validate_schema(f"{key}.{name}", value[name], child_schema))
    if schema_type == "array" and isinstance(value, list):
        child_schema = schema.get("items")
        if isinstance(child_schema, Mapping):
            for index, item in enumerate(value):
                errors.extend(validate_schema(f"{key}[{index}]", item, child_schema))
    return errors


def payload_matches_contract(
    payload: Any, contract: Mapping[str, Any]
) -> tuple[bool, list[str]]:
    if not contract:
        return True, []
    if not isinstance(payload, dict):
        return False, ["payload is not an object"]
    errors: list[str] = []
    for key, spec in contract.items():
        if not isinstance(spec, Mapping):
            continue
        if spec.get("required") and key not in payload:
            errors.append(f"missing required key '{key}'")
            continue
        if key not in payload:
            continue
        schema = spec.get("schema")
        if isinstance(schema, Mapping):
            errors.extend(validate_schema(key, payload[key], schema))
        elif isinstance(spec.get("type"), str) and not value_matches_type(
            payload[key], str(spec["type"])
        ):
            errors.append(f"key '{key}' expected {spec['type']}")
    return not errors, errors


def _context_value(context: Mapping[str, Any], *names: str) -> Any:
    for name in names:
        if name in context:
            return context[name]
    return None


def _available_source(value: Any) -> bool:
    return isinstance(value, Mapping) and value.get("status") not in (
        "unavailable",
        "placeholder",
    )


def _satellite_bounds(context: Mapping[str, Any]) -> Optional[tuple[float, float, float, float]]:
    satellite = _context_value(context, "SATELLITE_MAP", "satellite_map")
    if not _available_source(satellite):
        return None
    metadata = satellite.get("map_metadata", {})
    bounds = metadata.get("bounds") or satellite.get("bounds")
    if not isinstance(bounds, Mapping):
        return None
    try:
        north = float(bounds["north"])
        south = float(bounds["south"])
        east = float(bounds["east"])
        west = float(bounds["west"])
    except (KeyError, TypeError, ValueError):
        return None
    if north <= south or east <= west:
        return None
    return north, south, east, west


def _parse_coordinate_pairs(raw: Any) -> list[tuple[float, float]]:
    if not isinstance(raw, str):
        return []
    points: list[tuple[float, float]] = []
    for token in raw.split(";"):
        fields = [part.strip() for part in token.split(",")]
        if len(fields) < 2:
            continue
        try:
            points.append((float(fields[0]), float(fields[1])))
        except ValueError:
            continue
    return points


def _waypoint_payload_errors(payload: Any, context: Mapping[str, Any]) -> list[str]:
    if not isinstance(payload, dict):
        return ["waypoint payload is not an object"]
    satellite = _context_value(context, "SATELLITE_MAP", "satellite_map")
    robot_pose = _context_value(context, "ROBOT_POSE", "robot_pose")
    gps_fix = _context_value(context, "GPS_FIX", "gps_fix")
    annotated_map = _context_value(
        context, "ANNOTATED_SLAM_MAP_IMAGE", "annotated_slam_map"
    )
    if (
        _available_source(satellite)
        and not _available_source(annotated_map)
        and not (isinstance(robot_pose, Mapping) and isinstance(gps_fix, Mapping))
    ):
        return [
            "geographic waypoint planning requires map execution context; "
            "SATELLITE_MAP without both GPS_FIX and ROBOT_POSE cannot anchor map-frame waypoints"
        ]
    bounds = _satellite_bounds(context)
    points = _parse_coordinate_pairs(payload.get("waypoints"))
    if bounds and points:
        north, south, east, west = bounds
        if all(south <= x <= north and west <= y <= east for x, y in points):
            return [
                "waypoints appear to be lat/lon values; waypoints must be executable map-frame x,y,yaw"
            ]
    return []


def _parse_gps_waypoint(token: str) -> Optional[tuple[float, float, float, float]]:
    try:
        fields = [float(value.strip()) for value in token.split(",")]
    except ValueError:
        return None
    if len(fields) == 2:
        return fields[0], fields[1], 0.0, 0.0
    if len(fields) == 3:
        return fields[0], fields[1], 0.0, fields[2]
    if len(fields) == 4:
        return fields[0], fields[1], fields[2], fields[3]
    return None


def _gps_waypoint_payload_errors(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["GPS waypoint payload is not an object"]
    raw = payload.get("gps_waypoints")
    if not isinstance(raw, str) or not raw.strip():
        return ["gps_waypoints must contain at least one geographic waypoint"]
    errors: list[str] = []
    for index, token in enumerate(raw.split(";"), start=1):
        waypoint = _parse_gps_waypoint(token.strip())
        if waypoint is None:
            errors.append(f"gps_waypoints entry {index} is malformed")
            continue
        latitude, longitude, altitude, yaw = waypoint
        if not -90.0 <= latitude <= 90.0:
            errors.append(f"gps_waypoints entry {index} latitude is outside [-90, 90]")
        if not -180.0 <= longitude <= 180.0:
            errors.append(f"gps_waypoints entry {index} longitude is outside [-180, 180]")
        if not all(math.isfinite(value) for value in (latitude, longitude, altitude, yaw)):
            errors.append(f"gps_waypoints entry {index} contains a non-finite value")
    return errors


def generated_payload_errors(
    payload: Any,
    contract: Mapping[str, Any],
    context: Mapping[str, Any] | None = None,
) -> list[str]:
    _, errors = payload_matches_contract(payload, contract)
    result = list(errors)
    if "waypoints" in contract:
        result.extend(_waypoint_payload_errors(payload, context or {}))
    if "gps_waypoints" in contract:
        result.extend(_gps_waypoint_payload_errors(payload))
    return result
