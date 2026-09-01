"""Deterministic spatial checks that run before an LLM review can approve a plan."""

from __future__ import annotations

from typing import Any, Mapping, Sequence


def point_in_polygon(
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


def _context_value(context: Mapping[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in context:
            return context[key]
    return None


def _waypoint_xy(waypoint: Mapping[str, Any]) -> tuple[float, float] | None:
    try:
        return float(waypoint["x"]), float(waypoint["y"])
    except (KeyError, TypeError, ValueError):
        return None


def deterministic_plan_findings(
    review_input: Mapping[str, Any],
) -> list[dict[str, Any]]:
    context = review_input.get("context_snapshot")
    if not isinstance(context, Mapping):
        context = {}
    findings: list[dict[str, Any]] = []
    allowed = _context_value(context, "allowed_polygon", "ALLOWED_POLYGON")
    water_geofence = _context_value(
        context, "water_geofence", "WATER_GEOFENCE"
    )
    if isinstance(water_geofence, Mapping):
        water_geofence = water_geofence.get("polygon")
    blocked_regions = _context_value(
        context, "blocked_regions", "BLOCKED_REGIONS"
    )
    exclusion_zones = _context_value(
        context, "exclusion_zones", "EXCLUSION_ZONES"
    )
    blocked_regions = blocked_regions if isinstance(blocked_regions, list) else []
    exclusion_zones = exclusion_zones if isinstance(exclusion_zones, list) else []

    for fallback_index, waypoint in enumerate(
        review_input.get("waypoints") or [], start=1
    ):
        if not isinstance(waypoint, Mapping):
            continue
        xy = _waypoint_xy(waypoint)
        if xy is None:
            continue
        index = int(waypoint.get("index") or fallback_index)
        if isinstance(allowed, list) and not point_in_polygon(xy, allowed):
            findings.append(
                {
                    "severity": "critical",
                    "category": "robot_safety",
                    "waypoint_indices": [index],
                    "description": "Waypoint is outside the allowed mission polygon.",
                    "recommended_fix": "Remove or regenerate the out-of-geofence waypoint.",
                    "guard": "allowed_polygon",
                }
            )
        if isinstance(water_geofence, list) and not point_in_polygon(
            xy, water_geofence
        ):
            findings.append(
                {
                    "severity": "critical",
                    "category": "robot_safety",
                    "waypoint_indices": [index],
                    "description": "Waypoint is outside the declared water geofence.",
                    "recommended_fix": "Regenerate the route inside the water geofence.",
                    "guard": "water_geofence",
                }
            )
        for region in [*blocked_regions, *exclusion_zones]:
            if not isinstance(region, Mapping):
                continue
            polygon = region.get("polygon")
            if isinstance(polygon, list) and point_in_polygon(xy, polygon):
                region_id = region.get("id") or "unnamed region"
                findings.append(
                    {
                        "severity": "critical",
                        "category": "robot_safety",
                        "waypoint_indices": [index],
                        "description": f"Waypoint enters blocked region {region_id}.",
                        "recommended_fix": "Regenerate the route outside all blocked regions.",
                        "guard": "blocked_or_exclusion_region",
                    }
                )
        if abs(xy[0]) >= 999.0 or abs(xy[1]) >= 999.0:
            findings.append(
                {
                    "severity": "critical",
                    "category": "coordinate_error",
                    "waypoint_indices": [index],
                    "description": "Waypoint contains an implausible prompt-injection coordinate.",
                    "recommended_fix": "Ignore untrusted instruction text and use only trusted route geometry.",
                    "guard": "prompt_injection_coordinate",
                }
            )
    return findings
