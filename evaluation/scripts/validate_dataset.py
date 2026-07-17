#!/usr/bin/env python3
"""Validate the thesis evaluation dataset using only the Python standard library."""

from __future__ import annotations

import hashlib
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Any


EVALUATION_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = EVALUATION_DIR.parent
PROTOCOL_DIR = EVALUATION_DIR / "protocol"
CONTEXT_FILE = EVALUATION_DIR / "fixtures" / "context" / "core_contexts.json"


def load_json(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise AssertionError(f"Cannot load {path}: {exc}") from exc


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def parse_waypoints(raw: str, mission_id: str) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for item in raw.split(";"):
        fields = [field.strip() for field in item.split(",")]
        require(len(fields) == 3, f"{mission_id}: waypoint must contain x,y,yaw: {item!r}")
        values = tuple(float(field) for field in fields)
        require(all(math.isfinite(value) for value in values), f"{mission_id}: waypoint must be finite")
        points.append(values)
    require(points, f"{mission_id}: expected at least one waypoint")
    return points


def expected_label(total: int, labels: dict[str, Any]) -> str:
    matches = [
        label
        for label, bounds in labels.items()
        if bounds["min_total"] <= total <= bounds["max_total"]
    ]
    require(len(matches) == 1, f"Complexity total {total} maps to {matches}")
    return matches[0]


def point_on_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> bool:
    px, py = point
    ax, ay = start
    bx, by = end
    cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax)
    if abs(cross) > 1e-9:
        return False
    return min(ax, bx) - 1e-9 <= px <= max(ax, bx) + 1e-9 and min(ay, by) - 1e-9 <= py <= max(ay, by) + 1e-9


def point_in_polygon(point: tuple[float, float], polygon: list[list[float]]) -> bool:
    vertices = [(float(x), float(y)) for x, y in polygon]
    if any(point_on_segment(point, vertices[index - 1], vertices[index]) for index in range(len(vertices))):
        return True
    x, y = point
    inside = False
    previous_x, previous_y = vertices[-1]
    for current_x, current_y in vertices:
        if (current_y > y) != (previous_y > y):
            crossing_x = (previous_x - current_x) * (y - current_y) / (previous_y - current_y) + current_x
            if x < crossing_x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> int:
    value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if abs(value) < 1e-9:
        return 0
    return 1 if value > 0 else 2


def segments_intersect(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    if o1 != o2 and o3 != o4:
        return True
    return (
        (o1 == 0 and point_on_segment(c, a, b))
        or (o2 == 0 and point_on_segment(d, a, b))
        or (o3 == 0 and point_on_segment(a, c, d))
        or (o4 == 0 and point_on_segment(b, c, d))
    )


def route_intersects_polygon(route: list[tuple[float, float, float]], polygon: list[list[float]]) -> bool:
    vertices = [(float(x), float(y)) for x, y in polygon]
    if any(point_in_polygon((point[0], point[1]), polygon) for point in route):
        return True
    edges = list(zip(vertices, vertices[1:] + vertices[:1]))
    for first, second in zip(route, route[1:]):
        segment_start = (first[0], first[1])
        segment_end = (second[0], second[1])
        if any(segments_intersect(segment_start, segment_end, edge_start, edge_end) for edge_start, edge_end in edges):
            return True
    return False


def validate() -> list[str]:
    warnings: list[str] = []

    # Parse every JSON file first so malformed supporting files cannot be missed.
    for path in sorted(EVALUATION_DIR.rglob("*.json")):
        load_json(path)

    core = load_json(PROTOCOL_DIR / "core_missions.json")
    complexity = load_json(PROTOCOL_DIR / "complexity_rubric.json")
    scoring = load_json(PROTOCOL_DIR / "scoring_rubric.json")
    contexts = load_json(CONTEXT_FILE)
    safety = load_json(PROTOCOL_DIR / "safety_cases.json")
    variants = load_json(PROTOCOL_DIR / "context_variants.json")

    missions = core["missions"]
    design = core["design"]
    require(len(missions) == 9, "Core dataset must contain exactly nine missions")
    require(design["mission_count"] == len(missions), "Declared mission count does not match")
    require(design["planned_primary_outputs"] == len(missions) * 3 * design["primary_conditions"], "Primary output count is inconsistent")

    mission_ids = [mission["id"] for mission in missions]
    require(len(mission_ids) == len(set(mission_ids)), "Mission IDs must be unique")
    require(set(mission_ids) == {"S1", "S2", "S3", "M1", "M2", "M3", "C1", "C2", "C3"}, "Unexpected mission ID set")

    platform_counts = Counter(mission["platform"] for mission in missions)
    label_counts = Counter(mission["complexity"]["label"] for mission in missions)
    require(dict(platform_counts) == design["platform_distribution"], "Platform distribution does not match design")
    require(dict(label_counts) == design["complexity_distribution"], "Complexity distribution does not match design")
    require(abs(platform_counts["husky"] - platform_counts["blueboat"]) <= 1, "Platform split should be balanced within one mission")

    fixture_ids = set(contexts["fixtures"])
    require(fixture_ids == set(mission_ids), "Context fixtures must match mission IDs exactly")
    common_rubric_ids = set(scoring["common_elements"])

    blocked_blueboat = 0
    placeholder_artifacts = 0
    paraphrase_ids: set[str] = set()
    paraphrase_texts: set[str] = set()

    reference_routes: dict[str, list[tuple[float, float, float]]] = {}
    for mission in missions:
        mission_id = mission["id"]
        label = mission["complexity"]["label"]
        scores = mission["complexity"]["scores"]
        require(set(scores) == set(complexity["dimensions"]), f"{mission_id}: complexity dimensions do not match rubric")
        require(all(isinstance(value, int) and 0 <= value <= 2 for value in scores.values()), f"{mission_id}: complexity scores must be integers from 0 to 2")
        total = sum(scores.values())
        require(total == mission["complexity"]["total"], f"{mission_id}: complexity total is wrong")
        require(expected_label(total, complexity["labels"]) == label, f"{mission_id}: complexity label is wrong")

        paraphrases = mission["paraphrases"]
        require(len(paraphrases) == 3, f"{mission_id}: exactly three paraphrases are required")
        for index, paraphrase in enumerate(paraphrases, start=1):
            expected_id = f"{mission_id}-P{index}"
            require(paraphrase["id"] == expected_id, f"{mission_id}: expected paraphrase ID {expected_id}")
            require(paraphrase["id"] not in paraphrase_ids, f"Duplicate paraphrase ID {paraphrase['id']}")
            normalized = re.sub(r"\s+", " ", paraphrase["text"].strip().lower())
            require(normalized not in paraphrase_texts, f"Duplicate paraphrase text in {mission_id}")
            require(len(normalized) >= 10, f"{paraphrase['id']}: paraphrase is too short")
            paraphrase_ids.add(paraphrase["id"])
            paraphrase_texts.add(normalized)

        context_ref = mission["context_fixture"]
        require(context_ref == f"core_contexts.json#/fixtures/{mission_id}", f"{mission_id}: context reference is inconsistent")
        require(mission["expected"]["outcome"] == "plan", f"{mission_id}: core missions must expect a plan")
        require(bool(mission["expected"]["tree_id"]), f"{mission_id}: expected tree is missing")
        payload = mission["expected"]["canonical_payload"]
        require(isinstance(payload, dict) and payload, f"{mission_id}: canonical payload is empty")
        require("waypoints" in payload, f"{mission_id}: canonical payload needs waypoints")
        reference_routes[mission_id] = parse_waypoints(payload["waypoints"], mission_id)
        require(set(mission["semantic_rubric_elements"]).issubset(common_rubric_ids), f"{mission_id}: unknown semantic rubric element")
        require(len(mission["reference_behavior"]) >= 1, f"{mission_id}: reference behavior is empty")

        if mission["platform"] == "husky":
            require(mission["support_status"] == "implemented_catalogue", f"{mission_id}: Husky core case should use implemented catalogue")
        else:
            if mission["support_status"] == "blocked_blueboat_implementation":
                blocked_blueboat += 1
                require("blocking_requirement" in mission, f"{mission_id}: blocked mission needs blocking requirement")

    fixture_data = contexts["fixtures"]

    # Validate canonical route coordinates against the fixed context, not only syntax.
    for mission_id, point_id in (("S1", "P1"), ("S2", "W1")):
        expected_point = fixture_data[mission_id]["named_points"][point_id]
        actual = reference_routes[mission_id][0]
        require(actual == (expected_point["x"], expected_point["y"], expected_point["yaw"]), f"{mission_id}: canonical point differs from context")

    for mission_id, order_key in (("S3", "route_order"), ("M2", "route_order")):
        expected_route = [
            (
                fixture_data[mission_id]["named_points"][point_id]["x"],
                fixture_data[mission_id]["named_points"][point_id]["y"],
                fixture_data[mission_id]["named_points"][point_id]["yaw"],
            )
            for point_id in fixture_data[mission_id][order_key]
        ]
        require(reference_routes[mission_id] == expected_route, f"{mission_id}: canonical route differs from named-point context")

    m1_expected = [
        (point["x"], point["y"], point["yaw"])
        for point in fixture_data["M1"]["routes"]["R1"]["ordered_waypoints"]
    ]
    require(reference_routes["M1"] == m1_expected, "M1: canonical route differs from R1")

    m3_polygon = fixture_data["M3"]["target_areas"]["north_monitoring_zone"]["polygon"]
    require(all(point_in_polygon((point[0], point[1]), m3_polygon) for point in reference_routes["M3"]), "M3: canonical route leaves north zone")

    c1_context = fixture_data["C1"]
    c1_polygon = c1_context["target_areas"]["marked_field"]["polygon"]
    margin = c1_context["target_areas"]["marked_field"]["required_boundary_margin_m"]
    xs = [point[0] for point in c1_polygon]
    ys = [point[1] for point in c1_polygon]
    require(
        all(min(xs) + margin <= point[0] <= max(xs) - margin and min(ys) + margin <= point[1] <= max(ys) - margin for point in reference_routes["C1"]),
        "C1: canonical route violates the boundary margin",
    )
    for blocked_region in c1_context["blocked_regions"]:
        require(not route_intersects_polygon(reference_routes["C1"], blocked_region["polygon"]), f"C1: route intersects {blocked_region['id']}")

    for mission_id in ("M2", "C2", "C3"):
        geofence = fixture_data[mission_id]["water_geofence"]["polygon"]
        require(all(point_in_polygon((point[0], point[1]), geofence) for point in reference_routes[mission_id]), f"{mission_id}: canonical route leaves water geofence")

    c2_context = fixture_data["C2"]
    min_offset = c2_context["shore_offset_m"]["minimum"]
    max_offset = c2_context["shore_offset_m"]["maximum"]
    require(all(min_offset <= point[1] <= max_offset for point in reference_routes["C2"]), "C2: canonical route violates shore offset")
    for exclusion in c2_context["exclusion_zones"]:
        require(not route_intersects_polygon(reference_routes["C2"], exclusion["polygon"]), f"C2: route intersects {exclusion['id']}")
    c2_target = c2_context["target_sampling_interval_m"]
    c2_distances = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(reference_routes["C2"], reference_routes["C2"][1:])
    ]
    require(all(0.7 * c2_target <= distance <= 1.3 * c2_target for distance in c2_distances), "C2: canonical sample spacing exceeds ±30% tolerance")
    c2_duration_min = sum(c2_distances) / c2_context["planning_speed_m_s"] / 60.0
    require(c2_duration_min <= c2_context["maximum_duration_min"], "C2: canonical route exceeds duration limit")

    c3_target = fixture_data["C3"]["target_sampling_interval_m"]
    c3_distances = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(reference_routes["C3"], reference_routes["C3"][1:])
    ]
    require(all(abs(distance - c3_target) <= 1e-9 for distance in c3_distances), "C3: canonical sample spacing differs from target")

    def count_placeholders(value: Any) -> int:
        if isinstance(value, dict):
            own = 1 if value.get("status") == "placeholder" else 0
            return own + sum(count_placeholders(item) for item in value.values())
        if isinstance(value, list):
            return sum(count_placeholders(item) for item in value)
        return 0

    placeholder_artifacts = count_placeholders(contexts)
    if blocked_blueboat:
        warnings.append(f"{blocked_blueboat} BlueBoat core missions remain blocked from scored runs")
    if placeholder_artifacts:
        warnings.append(f"{placeholder_artifacts} context image artifacts still have placeholder status")

    safety_ids = [case["id"] for case in safety["cases"]]
    require(len(safety_ids) == len(set(safety_ids)), "Safety case IDs must be unique")
    require(len(safety_ids) >= 8, "At least eight adverse cases are expected")
    valid_outcomes = {"plan", "clarification", "refusal"}
    for case in safety["cases"]:
        require(case["expected"]["outcome"] in valid_outcomes, f"{case['id']}: invalid expected outcome")
        require(bool(case["failure_if"]), f"{case['id']}: failure condition is missing")

    variant_ids: set[str] = set()
    for mission_group in variants["selected_missions"]:
        require(mission_group["mission_id"] in mission_ids, "Context variant references unknown mission")
        require(len(mission_group["variants"]) == 5, f"{mission_group['mission_id']}: expected five context conditions")
        conditions = {variant["condition"] for variant in mission_group["variants"]}
        require(conditions == {"complete", "text_only", "missing_one_source", "contradictory", "irrelevant_distractor"}, f"{mission_group['mission_id']}: context condition set is incomplete")
        for variant in mission_group["variants"]:
            require(variant["id"] not in variant_ids, f"Duplicate context variant ID {variant['id']}")
            require(variant["expected_outcome"] in valid_outcomes, f"{variant['id']}: invalid expected outcome")
            variant_ids.add(variant["id"])

    snapshot = core["source_snapshot"]
    snapshot_files = {
        "tree_metadata_sha256": REPOSITORY_DIR / "config" / "tree_metadata.yaml",
        "system_description_sha256": REPOSITORY_DIR / "config" / "system_description.yaml",
    }
    for key, path in snapshot_files.items():
        if path.exists():
            require(sha256(path) == snapshot[key], f"Source snapshot changed: {path}")
        else:
            warnings.append(f"Cannot verify source hash outside repository layout: {path}")

    return warnings


def main() -> int:
    try:
        warnings = validate()
    except (AssertionError, KeyError, TypeError, ValueError) as exc:
        print(f"DATASET INVALID: {exc}", file=sys.stderr)
        return 1

    print("DATASET VALID")
    print("Core missions: 9; paraphrases: 27; planned E1 outputs: 189")
    for warning in warnings:
        print(f"WARNING: {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
