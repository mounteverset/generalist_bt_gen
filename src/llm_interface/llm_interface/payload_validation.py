"""Pure payload-contract checks shared by the ROS node and offline evaluation."""

from __future__ import annotations

import math
import re
from typing import Any, Mapping, Optional


# ponytail: tune this clearance using measured OSM/GPS error on the robot.
STAIR_CLEARANCE_M = 5.0


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
    if not isinstance(raw, str) or not raw.strip():
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


def _segment_distance_m(
    first: tuple[float, float],
    second: tuple[float, float],
    third: tuple[float, float],
    fourth: tuple[float, float],
) -> float:
    latitude_scale = 111_195.0
    longitude_scale = latitude_scale * math.cos(math.radians(first[0]))

    def xy(point: tuple[float, float]) -> tuple[float, float]:
        return (
            (point[1] - first[1]) * longitude_scale,
            (point[0] - first[0]) * latitude_scale,
        )

    a, b, c, d = (xy(point) for point in (first, second, third, fourth))

    def cross(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
        return 0.0

    def point_to_segment(p, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        fraction = 0.0
        if length_squared:
            projection = ((p[0] - start[0]) * dx + (p[1] - start[1]) * dy)
            fraction = max(0.0, min(1.0, projection / length_squared))
        return math.hypot(p[0] - start[0] - fraction * dx, p[1] - start[1] - fraction * dy)

    return min(
        point_to_segment(a, c, d), point_to_segment(b, c, d),
        point_to_segment(c, a, b), point_to_segment(d, a, b),
    )


def gps_stairway_errors(payload: Any, context: Mapping[str, Any]) -> list[str]:
    raw = payload.get("gps_waypoints") if isinstance(payload, dict) else None
    if not isinstance(raw, str):
        return []
    osm = _context_value(context, "OSM_CONTEXT", "osm_context")
    if not isinstance(osm, Mapping):
        return []
    if osm.get("status") == "unavailable":
        return ["OSM_CONTEXT unavailable; cannot verify GPS route against stairways"]
    steps = osm.get("steps_features")
    if not isinstance(steps, list):
        return [
            "OSM_CONTEXT has no stairway inventory; gather fresh OSM context "
            "before GPS navigation"
        ]
    route = [_parse_gps_waypoint(token.strip()) for token in raw.split(";") if token.strip()]
    if not route or any(
        point is None or not all(math.isfinite(value) for value in point)
        for point in route
    ):
        return []
    center = osm.get("center")
    try:
        center_lat = float(center["lat"])
        center_lon = float(center["lon"])
        radius_m = float(osm["radius_m"])
    except (KeyError, TypeError, ValueError):
        return ["OSM_CONTEXT lacks stairway query coverage; cannot validate GPS route"]
    if (
        not all(math.isfinite(value) for value in (center_lat, center_lon, radius_m))
        or radius_m <= STAIR_CLEARANCE_M
    ):
        return ["OSM_CONTEXT has invalid stairway query coverage"]
    longitude_scale = 111_195.0 * math.cos(math.radians(center_lat))
    for index, point in enumerate(route, start=1):
        distance_m = math.hypot(
            (point[0] - center_lat) * 111_195.0,
            (point[1] - center_lon) * longitude_scale,
        )
        if distance_m > radius_m - STAIR_CLEARANCE_M:
            return [f"gps_waypoints entry {index} lies outside OSM stairway query coverage"]

    for stair in steps:
        if not isinstance(stair, Mapping):
            return ["OSM stairway inventory is malformed; cannot validate GPS route"]
        coordinates = stair.get("coordinates")
        stair_points: list[tuple[float, float]] = []
        try:
            stair_points = [
                (float(point["lat"]), float(point["lon"])) for point in coordinates
            ]
        except (KeyError, TypeError, ValueError):
            pass
        if len(stair_points) < 2 or not all(
            math.isfinite(value) for point in stair_points for value in point
        ):
            return [f"OSM steps way {stair.get('osm_id', 'unknown')} lacks valid geometry"]
        for index in range(max(1, len(route) - 1)):
            start = route[index][:2]
            end = route[min(index + 1, len(route) - 1)][:2]
            for first, second in zip(stair_points, stair_points[1:]):
                if _segment_distance_m(start, end, first, second) <= STAIR_CLEARANCE_M:
                    return [
                        f"gps_waypoints segment {index + 1} approaches OSM steps way "
                        f"{stair.get('osm_id', 'unknown')} within {STAIR_CLEARANCE_M:g} m"
                    ]
    return []


def _object_route_errors(payload: Any, context: Mapping[str, Any]) -> list[str]:
    if not isinstance(payload, Mapping):
        return []
    map_waypoints = payload.get("waypoints")
    if isinstance(map_waypoints, str) and map_waypoints and not map_waypoints.strip():
        return ["waypoints must contain map coordinates or be exactly empty"]
    if map_waypoints:
        return []
    raw_gps = payload.get("gps_waypoints")
    if not raw_gps:
        return ["object route needs FindAnything map waypoints or OSM tree GPS waypoints"]
    hints = _context_value(context, "REQUEST_HINTS")
    mission = hints.get("MISSION_REQUEST") if isinstance(hints, Mapping) else None
    mission_text = str(mission.get("mission_text", "")) if isinstance(mission, Mapping) else ""
    if mission_text and not re.search(r"\btrees?\b", mission_text, re.IGNORECASE):
        return ["OSM tree fallback is only valid for tree requests"]

    find_anything = _context_value(context, "FIND_ANYTHING")
    if isinstance(find_anything, Mapping):
        queries = find_anything.get("queries")
        if find_anything.get("locations") or any(
            isinstance(query, Mapping) and query.get("locations")
            for query in (queries if isinstance(queries, list) else [])
        ):
            return ["FindAnything locations are available; use them as map-frame targets"]

    osm = _context_value(context, "OSM_CONTEXT", "osm_context")
    trees = osm.get("tree_features", []) if isinstance(osm, Mapping) else []
    centers: list[tuple[float, float]] = []
    for tree in (trees if isinstance(trees, list) else []):
        center = tree.get("center") if isinstance(tree, Mapping) else None
        if not isinstance(center, Mapping):
            continue
        try:
            point = float(center["lat"]), float(center["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point):
            centers.append(point)
    if not centers:
        return ["FindAnything unavailable and OSM_CONTEXT has no mapped tree coordinates"]

    used: set[int] = set()
    for index, token in enumerate(str(raw_gps).split(";"), start=1):
        point = _parse_gps_waypoint(token.strip())
        if point is None:
            continue
        match = next((number for number, center in enumerate(centers)
                      if abs(point[0] - center[0]) <= 1e-5
                      and abs(point[1] - center[1]) <= 1e-5), None)
        if match is None or match in used:
            return [f"gps_waypoints entry {index} must be a distinct OSM tree center"]
        used.add(match)
    return []


def osm_five_tree_route(context: Mapping[str, Any], user_command: str) -> str:
    """Supply C3's five OSM targets when the model produced no route."""
    if not (re.search(r"\b(?:five|5)\b", user_command, re.IGNORECASE)
            and re.search(r"\btrees?\b", user_command, re.IGNORECASE)):
        return ""
    osm = _context_value(context, "OSM_CONTEXT", "osm_context")
    trees = osm.get("tree_features") if isinstance(osm, Mapping) else None
    if not isinstance(trees, list):
        return ""
    points: list[tuple[float, float]] = []
    for tree in trees:
        center = tree.get("center") if isinstance(tree, Mapping) else None
        if not isinstance(center, Mapping):
            continue
        try:
            point = float(center["lat"]), float(center["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point) and point not in points:
            points.append(point)
        if len(points) == 5:
            break
    if len(points) < 5:
        return ""
    route = "; ".join(f"{lat:.8f},{lon:.8f},0.0" for lat, lon in points)
    return route if not _object_route_errors(
        {"waypoints": "", "gps_waypoints": route}, context
    ) else ""


def generated_payload_errors(
    payload: Any,
    contract: Mapping[str, Any],
    context: Mapping[str, Any] | None = None,
) -> list[str]:
    _, errors = payload_matches_contract(payload, contract)
    result = list(errors)
    if "waypoints" in contract and isinstance(payload, Mapping) and payload.get("waypoints"):
        result.extend(_waypoint_payload_errors(payload, context or {}))
    if "gps_waypoints" in contract and (
        "waypoints" not in contract
        or (isinstance(payload, Mapping) and payload.get("gps_waypoints"))
    ):
        result.extend(_gps_waypoint_payload_errors(payload))
        result.extend(gps_stairway_errors(payload, context or {}))
    if "waypoints" in contract and "gps_waypoints" in contract:
        result.extend(_object_route_errors(payload, context or {}))
    return result
