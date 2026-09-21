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

from evaluation_coordinates import gps_route_to_map


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
    execution_scoring = load_json(PROTOCOL_DIR / "execution_scoring.json")
    contexts = load_json(CONTEXT_FILE)
    safety = load_json(PROTOCOL_DIR / "safety_cases.json")
    variants = load_json(PROTOCOL_DIR / "context_variants.json")
    runtime = load_json(PROTOCOL_DIR / "runtime_contract.json")
    node_manifest = load_json(PROTOCOL_DIR / "bt_node_manifest.json")
    model_conditions = load_json(PROTOCOL_DIR / "model_conditions.json")
    choice_space = load_json(PROTOCOL_DIR / "choice_space_variants.json")
    m1_distractors = load_json(PROTOCOL_DIR / "m1_action_distractors.json")

    missions = core["missions"]
    design = core["design"]
    require(len(missions) == 9, "Core dataset must contain exactly nine missions")
    require(design["mission_count"] == len(missions), "Declared mission count does not match")
    require(design["planned_primary_outputs"] == len(missions) * 3 * design["primary_conditions"], "Primary output count is inconsistent")
    require(
        design["currently_supported_primary_outputs"] + design["blocked_blueboat_outputs"]
        == design["planned_primary_outputs"],
        "Supported and blocked E1 output counts do not sum to the design total",
    )

    mission_ids = [mission["id"] for mission in missions]
    require(len(mission_ids) == len(set(mission_ids)), "Mission IDs must be unique")
    require(set(mission_ids) == {"S1", "S2", "S3", "M1", "M2", "M3", "C1", "C2", "C3"}, "Unexpected mission ID set")

    platform_counts = Counter(mission["platform"] for mission in missions)
    label_counts = Counter(mission["complexity"]["label"] for mission in missions)
    require(dict(platform_counts) == design["platform_distribution"], "Platform distribution does not match design")
    require(dict(label_counts) == design["complexity_distribution"], "Complexity distribution does not match design")
    require(dict(platform_counts) == {"husky": 7, "blueboat": 2}, "Core dataset must contain seven Husky and two BlueBoat missions")

    fixture_ids = set(contexts["fixtures"])
    require(fixture_ids == set(mission_ids), "Context fixtures must match mission IDs exactly")
    trees_by_id = {tree["id"]: tree for tree in runtime["tree_catalogue"]}
    for mission in missions:
        mission_id = mission["id"]
        tree_id = mission["expected"]["tree_id"]
        require(tree_id in trees_by_id, f"{mission_id}: expected tree is absent from the runtime catalogue")
        expected_context = trees_by_id[tree_id].get("context_requirements", [])
        require(
            mission["requirements"]["required_context"] == expected_context,
            f"{mission_id}: required context differs from {tree_id}",
        )
        require(
            contexts["fixtures"][mission_id]["available_context"] == expected_context,
            f"{mission_id}: available context differs from {tree_id}",
        )
    common_rubric_ids = set(scoring["common_elements"])
    require(
        set(scoring["common_semantic_scale"]) == {"0", "1", "2"},
        "Semantic scale must contain exactly 0, 1, and 2",
    )
    require(
        scoring["semantic_scoring"]["semantic_pass"].startswith("Every applicable"),
        "Semantic pass rule is missing",
    )
    require(
        scoring["effort_metrics"]["manual_repair_budget_s"] == 300,
        "Manual repair budget must be 300 seconds",
    )
    require(
        set(scoring["effort_metrics"]["manual_repair_statuses"])
        == {"not_needed", "corrected", "attempted_failed", "not_attempted", "not_applicable"},
        "Manual repair statuses are incomplete",
    )
    require(
        scoring["scored_run_settings"]
        == {
            "repetitions": 1,
            "seed": 42,
            "timeout_s": 60,
            "max_transport_retries": 3,
            "m3_refinement_attempts": 1,
        },
        "Scored-run settings differ from the fixed protocol",
    )
    for name, protocol in (
        ("scoring_rubric.json", scoring),
        ("execution_scoring.json", execution_scoring),
        ("context_variants.json", variants),
        ("safety_cases.json", safety),
    ):
        require(
            protocol.get("freeze_status") in {"draft", "frozen"},
            f"{name}: freeze status is invalid",
        )
    require(
        set(execution_scoring["integration_checks_by_platform"])
        == {"husky", "blueboat"},
        "E5 integration checks must cover both platforms",
    )
    require(
        len(execution_scoring["portability_components"]) == 8
        and len(set(execution_scoring["portability_components"])) == 8,
        "E5 must define eight unique portability components",
    )
    require(
        execution_scoring["primary_trial_plan"]
        == {
            "mission_ids": ["S1", "S2", "S3", "M1", "M2", "M3", "C1", "C2", "C3"],
            "evidence_level": "physical",
            "repetitions_per_mission": 3,
            "planned_trials": 27,
            "planning_method": "M3",
            "planning_model": "gpt-5.6-sol",
            "planning_paraphrase": "P2",
            "simulation_trials": "Excluded from the primary result.",
        },
        "E5 primary trial plan differs from the thesis protocol",
    )
    require(
        execution_scoring["mission_requirements"]
        == {
            "S1": {
                "platform": "husky",
                "required_waypoints": 1,
                "required_measurements": 1,
                "required_photos": 0,
            },
            "S2": {
                "platform": "blueboat",
                "required_waypoints": 1,
                "required_measurements": 1,
                "required_photos": 0,
            },
            "S3": {
                "platform": "husky",
                "required_waypoints": 3,
                "required_measurements": 3,
                "required_photos": 0,
            },
            "M1": {
                "platform": "husky",
                "required_waypoints": 3,
                "required_measurements": 0,
                "required_photos": 4,
            },
            "M2": {
                "platform": "blueboat",
                "required_waypoints": 4,
                "required_measurements": 4,
                "required_photos": 0,
            },
            "M3": {
                "platform": "husky",
                "required_waypoints": 3,
                "required_measurements": 3,
                "required_photos": 0,
            },
            "C1": {
                "platform": "husky",
                "required_waypoints": 8,
                "required_measurements": 0,
                "required_photos": 0,
            },
            "C2": {
                "platform": "husky",
                "required_waypoints": 381,
                "required_measurements": 381,
                "required_photos": 0,
            },
            "C3": {
                "platform": "husky",
                "required_waypoints": 5,
                "required_measurements": 0,
                "required_photos": 0,
            },
        },
        "E5 mission denominators differ from the thesis protocol",
    )

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
        require(
            [item.get("specificity") for item in paraphrases] == ["low", "medium", "high"],
            f"{mission_id}: P1/P2/P3 must have low/medium/high specificity",
        )
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
        contract = trees_by_id[mission["expected"]["tree_id"]].get("blackboard_contract", {})
        required_keys = {key for key, value in contract.items() if value.get("required")}
        require(
            required_keys <= set(payload),
            f"{mission_id}: canonical payload misses required keys {sorted(required_keys - set(payload))}",
        )
        require(
            set(payload) <= set(contract),
            f"{mission_id}: canonical payload contains unknown keys {sorted(set(payload) - set(contract))}",
        )
        if mission["platform"] == "blueboat":
            require("gps_waypoints" in payload and "waypoints" not in payload, f"{mission_id}: BlueBoat requires GPS waypoints")
        if "gps_waypoints" in payload:
            reference_routes[mission_id] = gps_route_to_map(
                payload["gps_waypoints"], contexts["fixtures"][mission_id]
            )
        else:
            require("waypoints" in payload, f"{mission_id}: canonical payload needs waypoints")
            reference_routes[mission_id] = parse_waypoints(payload["waypoints"], mission_id)
        if mission_id in {"S1", "S2", "S3", "M2"}:
            route = payload.get("gps_waypoints", payload.get("waypoints"))
            require(
                all(route in item["text"] for item in paraphrases),
                f"{mission_id}: every paraphrase must state the exact supplied coordinates",
            )
        require(set(mission["semantic_rubric_elements"]).issubset(common_rubric_ids), f"{mission_id}: unknown semantic rubric element")
        require(len(mission["reference_behavior"]) >= 1, f"{mission_id}: reference behavior is empty")

        if mission["platform"] == "husky":
            require(mission["support_status"] == "implemented_catalogue", f"{mission_id}: Husky core case should use implemented catalogue")
        else:
            if mission["support_status"] == "blocked_blueboat_implementation":
                blocked_blueboat += 1
                require("blocking_requirement" in mission, f"{mission_id}: blocked mission needs blocking requirement")

    fixture_data = contexts["fixtures"]
    default_gps = (48.284180, 11.608129)
    for mission_id, context in fixture_data.items():
        if "GPS_FIX" not in context["available_context"]:
            continue
        gps_fix = context.get("gps_fix", {})
        require(
            abs(gps_fix.get("latitude", math.inf) - default_gps[0]) < 1e-9
            and abs(gps_fix.get("longitude", math.inf) - default_gps[1]) < 1e-9,
            f"{mission_id}: GPS_FIX must use the frozen Hollerner Lake default",
        )
    c2_osm = fixture_data["C2"]["osm_context"]
    require(
        c2_osm.get("provider") == "overpass"
        and c2_osm.get("source_endpoint") == "https://overpass-api.de/api/interpreter"
        and c2_osm.get("center") == {"lat": default_gps[0], "lon": default_gps[1]}
        and c2_osm.get("radius_m") == 1200.0,
        "C2: OSM context must use the context gatherer's Hollerner Lake request",
    )
    require(
        c2_osm.get("feature_counts", {}).get("linear") == len(c2_osm.get("linear_features", []))
        and c2_osm.get("feature_counts", {}).get("point") == len(c2_osm.get("point_features", []))
        and c2_osm.get("feature_counts", {}).get("area") == len(c2_osm.get("area_features", []))
        and any(area.get("name") == "Hollerner See" for area in c2_osm.get("area_features", [])),
        "C2: OSM context is incomplete or does not contain Hollerner See",
    )

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

    geofence = fixture_data["M2"]["water_geofence"]["polygon"]
    require(all(point_in_polygon((point[0], point[1]), geofence) for point in reference_routes["M2"]), "M2: canonical route leaves water geofence")

    c2_context = fixture_data["C2"]
    c2_target = c2_context["target_sampling_interval_m"]
    c2_distances = [
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(reference_routes["C2"], reference_routes["C2"][1:])
    ]
    require(all(0.6 * c2_target <= distance <= 1.3 * c2_target for distance in c2_distances), "C2: canonical straight-line spacing is inconsistent with the 10 m path interval")
    require(math.hypot(reference_routes["C2"][-1][0] - reference_routes["C2"][0][0], reference_routes["C2"][-1][1] - reference_routes["C2"][0][1]) < 1.0, "C2: canonical route is not a roundtrip")
    require(c2_context["osm_context"]["linear_features"], "C2: OSM route geometry is missing")

    c3_context = fixture_data["C3"]
    c3_locations = [
        (location["point"]["x"], location["point"]["y"])
        for location in c3_context["find_anything"]["locations"]
    ]
    c3_route = [(point[0], point[1]) for point in reference_routes["C3"]]
    require(len(c3_route) == 5 and set(c3_route) == set(c3_locations), "C3: canonical route must use all five FindAnything tree locations")
    require(core["missions"][-1]["expected"]["canonical_payload"].get("waypoint_frame_id") == "map", "C3: canonical route must use the FindAnything map frame")

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
        supported_outputs = (
            (len(missions) - blocked_blueboat)
            * design["paraphrases_per_mission"]
            * design["primary_conditions"]
        )
        require(
            supported_outputs == design["currently_supported_primary_outputs"],
            "Declared supported E1 output count is wrong",
        )
        warnings.append(
            f"Only {supported_outputs} of {design['planned_primary_outputs']} E1 outputs "
            "are currently supported; blocked BlueBoat cases must not enter success rates"
        )
    if placeholder_artifacts:
        warnings.append(f"{placeholder_artifacts} context image artifacts still have placeholder status")

    safety_ids = [case["id"] for case in safety["cases"]]
    require(len(safety_ids) == len(set(safety_ids)), "Safety case IDs must be unique")
    require(len(safety_ids) >= 8, "At least eight adverse cases are expected")
    valid_outcomes = {"plan", "clarification", "refusal"}
    for case in safety["cases"]:
        require(case["expected"]["outcome"] in valid_outcomes, f"{case['id']}: invalid expected outcome")
        require(bool(case["failure_if"]), f"{case['id']}: failure condition is missing")
        require(
            case["status"] in {
                "ready_current_husky",
                "ready_current_blueboat",
            },
            f"{case['id']}: safety guard status is not resolved",
        )

    variant_ids: set[str] = set()
    require(
        variants.get("reference_model_key") in model_conditions["models"],
        "E2 reference model is unknown",
    )
    for mission_group in variants["selected_missions"]:
        require(mission_group["mission_id"] in mission_ids, "Context variant references unknown mission")
        require(len(mission_group["variants"]) == 6, f"{mission_group['mission_id']}: expected six context conditions")
        conditions = {variant["condition"] for variant in mission_group["variants"]}
        require(conditions == {"complete", "text_only", "missing_one_source", "contradictory", "irrelevant_distractor", "stale"}, f"{mission_group['mission_id']}: context condition set is incomplete")
        for variant in mission_group["variants"]:
            require(variant["id"] not in variant_ids, f"Duplicate context variant ID {variant['id']}")
            require(variant["expected_outcome"] in valid_outcomes, f"{variant['id']}: invalid expected outcome")
            require(isinstance(variant.get("operations"), list), f"{variant['id']}: machine-readable operations are missing")
            for operation in variant["operations"]:
                require(operation.get("op") in {"set", "remove"}, f"{variant['id']}: invalid context operation")
                require(bool(operation.get("path")), f"{variant['id']}: context operation path is missing")
            variant_ids.add(variant["id"])

    snapshot = core["source_snapshot"]
    snapshot_files = {
        "tree_metadata_sha256": REPOSITORY_DIR / "config" / "tree_metadata.yaml",
        "system_description_sha256": REPOSITORY_DIR / "config" / "system_description.yaml",
        "blueboat_system_description_sha256": REPOSITORY_DIR / "config" / "system_description_blueboat.yaml",
        "bt_node_manifest_sha256": PROTOCOL_DIR / "bt_node_manifest.json",
    }
    for key, path in snapshot_files.items():
        if path.exists():
            require(sha256(path) == snapshot[key], f"Source snapshot changed: {path}")
        else:
            warnings.append(f"Cannot verify source hash outside repository layout: {path}")

    require(
        runtime["source_hashes"] == {
            key: snapshot[key] for key in snapshot_files
        },
        "Runtime contract hashes differ from the dataset source snapshot",
    )
    implementation_files = {
        "evaluation_coordinates_sha256": EVALUATION_DIR / "scripts" / "evaluation_coordinates.py",
        "mission_reasoner_sha256": REPOSITORY_DIR / "src" / "mission_reasoner" / "mission_reasoner" / "reasoner.py",
        "payload_validation_sha256": REPOSITORY_DIR / "src" / "llm_interface" / "llm_interface" / "payload_validation.py",
        "plan_safety_validation_sha256": REPOSITORY_DIR / "src" / "plan_reviewer" / "plan_reviewer" / "safety_validation.py",
        "evaluation_core_sha256": EVALUATION_DIR / "scripts" / "evaluation_core.py",
        "evaluation_runner_sha256": EVALUATION_DIR / "scripts" / "run_evaluation.py",
        "btgenbot2_importer_sha256": EVALUATION_DIR / "scripts" / "import_btgenbot2_batch.py",
        "btgenbot2_server_sha256": EVALUATION_DIR / "colab" / "btgenbot2_server.py",
        "factory_helper_source_sha256": REPOSITORY_DIR / "src" / "bt_executor" / "src" / "bt_factory_check.cpp",
    }
    for key, path in implementation_files.items():
        require(
            runtime["implementation_hashes"].get(key) == sha256(path),
            f"Runtime contract has stale implementation hash: {path}",
        )
    require(
        runtime["bt_node_manifest"] == node_manifest,
        "Runtime contract contains a stale BT node manifest",
    )
    require(
        {tree["id"] for tree in runtime["tree_catalogue"]}
        == {
            "temperature_logging.xml",
            "gps_waypoint_navigation.xml",
            "gps_temperature_logging.xml",
            "blueboat_temperature_logging.xml",
            "navigate_and_photograph.xml",
            "find_and_drive_to_nearest_object.xml",
            "explore_area.xml",
            "blueboat_temperature_logging.xml",
        },
        "Runtime tree catalogue does not contain the seven current tree templates",
    )
    registered_nodes = set(node_manifest["registered_nodes"])
    registration_source = (
        REPOSITORY_DIR / "src" / "robot_actions" / "src" / "plugin_registration.cpp"
    ).read_text(encoding="utf-8")
    compiled_node_names = set(
        re.findall(
            r'factory\.registerNodeType<[^>]+>\s*\(\s*"([^"]+)"',
            registration_source,
        )
    )
    require(
        registered_nodes == compiled_node_names,
        "BT node manifest differs from plugin_registration.cpp",
    )
    require(
        {
            "MoveToGPS",
            "ParseGpsWaypoints",
            "PublishWaypointMarkers",
            "TakePhoto",
        }.issubset(registered_nodes),
        "Registered-node manifest omits current aliases or GPS nodes",
    )
    require(
        not {"FindAnything", "FindObjectLocation"}.intersection(registered_nodes),
        "Pre-BT context gathering nodes must not appear as registered BT actions",
    )
    require(
        not {"CheckBattery", "ReturnToHome", "Delay"}.intersection(registered_nodes),
        "Evaluation node manifest includes unregistered custom nodes",
    )
    require(
        len(model_conditions["models"]) == 3,
        "Expected exactly three general-purpose model conditions",
    )
    m1_scale = choice_space["m1_action_library"]
    require(
        choice_space.get("freeze_status") in {"draft", "frozen"},
        "Choice-space freeze status is invalid",
    )
    require(
        m1_distractors.get("freeze_status") in {"draft", "frozen"},
        "M1 distractor freeze status is invalid",
    )
    require(
        m1_scale["base_action_node_count"]
        == len(m1_scale["base_node_descriptions"]),
        "M1 scale protocol must match the evaluation-visible custom action nodes",
    )
    require(
        set(m1_scale["base_node_descriptions"]).issubset(registered_nodes),
        "M1 base-node descriptions must reference registered nodes",
    )
    require(
        all(
            "Effect:" in description and "Applicability:" in description
            for description in m1_scale["base_node_descriptions"].values()
        ),
        "M1 base nodes require comparable effect and applicability descriptions",
    )
    m1_variant_sizes = {
        variant["id"]: variant["action_node_count"]
        for variant in m1_scale["variants"]
    }
    require(
        m1_variant_sizes
        == {"M1-N12": 12, "M1-N24": 24, "M1-N50": 50, "M1-N100": 100},
        "M1 scale variants must contain the frozen 12/24/50/100 levels",
    )
    require(
        all(
            variant["distractor_count"]
            == variant["action_node_count"] - m1_scale["base_action_node_count"]
            for variant in m1_scale["variants"]
        ),
        "M1 scale variant distractor counts are inconsistent",
    )
    distractor_entries = m1_distractors["distractors"]
    distractor_names = [entry["name"] for entry in distractor_entries]
    require(len(distractor_entries) == 90, "M1 requires exactly 90 distractors")
    require(
        len(distractor_names) == len(set(distractor_names)),
        "M1 distractor names must be unique",
    )
    require(
        not set(distractor_names).intersection(registered_nodes),
        "M1 distractors collide with real registered nodes",
    )
    profiles = m1_distractors["port_profiles"]
    for entry in distractor_entries:
        require(
            entry.get("category") in {"semantic_near", "adjacent_capability"},
            f"{entry.get('name')}: invalid distractor category",
        )
        require(bool(entry.get("effect")), f"{entry['name']}: effect is missing")
        require(
            bool(entry.get("applicability")),
            f"{entry['name']}: applicability is missing",
        )
        require(
            entry.get("port_profile") in profiles,
            f"{entry['name']}: unknown port profile",
        )
        if entry["category"] == "semantic_near":
            require(
                entry.get("confuses_with") in registered_nodes,
                f"{entry['name']}: semantic-near target is not a real node",
            )
    require(
        Counter(entry["category"] for entry in distractor_entries)
        == {"semantic_near": 45, "adjacent_capability": 45},
        "M1 distractors must be balanced by category",
    )
    for variant in m1_scale["variants"]:
        selected = distractor_entries[: variant["distractor_count"]]
        counts = Counter(entry["category"] for entry in selected)
        require(
            abs(counts["semantic_near"] - counts["adjacent_capability"]) <= 1,
            f"{variant['id']}: distractor prefix is not category-balanced",
        )
    require(
        all(re.fullmatch(r"[A-Z][A-Za-z0-9]*", name) for name in distractor_names),
        "M1 added actions must follow the compiled nodes' PascalCase convention",
    )
    require(
        choice_space["m3_tree_catalogue"]["method"] == "M3"
        and choice_space["m3_tree_catalogue"].get("reference_model_key")
        in model_conditions["models"]
        and {
            variant["id"]
            for variant in choice_space["m3_tree_catalogue"]["variants"]
        }
        == {"CS1", "CS2", "CS3"},
        "M3 tree-catalogue variants are incomplete",
    )

    return warnings


def main() -> int:
    try:
        warnings = validate()
    except (AssertionError, KeyError, TypeError, ValueError) as exc:
        print(f"DATASET INVALID: {exc}", file=sys.stderr)
        return 1

    print("DATASET VALID")
    print("Core missions: 9; paraphrases: 27; designed E1 outputs: 189")
    print("Currently supported E1 outputs: 189; BlueBoat outputs are available offline")
    for warning in warnings:
        print(f"WARNING: {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
