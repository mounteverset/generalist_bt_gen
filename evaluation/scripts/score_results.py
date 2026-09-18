#!/usr/bin/env python3
"""Aggregate frozen evaluation artifacts and optional human/E5 records."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
import random
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Mapping

from evaluation_core import EVALUATION, PROTOCOL, load_json, write_json
from run_evaluation import protocol_hashes

MANUAL_CORRECTION_TYPES = (
    "syntax",
    "interface",
    "tree_selection",
    "control_flow",
    "parameter",
    "spatial_route",
    "sensing_or_output",
    "constraint",
    "missing_catalogue_item",
)


def nested(value: Mapping[str, Any], *path: str) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def model_calls(result: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    calls: list[Mapping[str, Any]] = []
    if isinstance(result.get("api_call"), Mapping):
        calls.append(result["api_call"])
    for stage in result.get("stages", []):
        if not isinstance(stage, Mapping):
            continue
        name = str(stage.get("stage", ""))
        if name in {"requirements", "selection"} or name.startswith("payload_attempt_"):
            calls.append(stage)
    return calls


def token_total(calls: Iterable[Mapping[str, Any]], kind: str) -> int | None:
    keys = (
        ("prompt_tokens", "input_tokens")
        if kind == "input"
        else ("completion_tokens", "output_tokens")
    )
    total = 0
    found = False
    for call in calls:
        sources = (call.get("usage", {}), call.get("generation_metadata", {}))
        for source in sources:
            if not isinstance(source, Mapping):
                continue
            value = next((source.get(key) for key in keys if source.get(key) is not None), None)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                total += int(value)
                found = True
                break
    return total if found else None


def transport_retry_count(calls: Iterable[Mapping[str, Any]]) -> int:
    return sum(
        max(0, len(call.get("attempts", [])) - 1)
        for call in calls
        if isinstance(call.get("attempts"), list)
    )


def automatic_refinement_count(result: Mapping[str, Any]) -> int:
    return sum(
        1
        for stage in result.get("stages", [])
        if isinstance(stage, Mapping)
        and str(stage.get("stage", "")).startswith("payload_attempt_")
        and str(stage.get("stage")) != "payload_attempt_1"
    )


def syntax_valid(result: Mapping[str, Any], method: str) -> bool | None:
    if method == "M3":
        checks: list[bool] = []
        for stage in result.get("stages", []):
            if not isinstance(stage, Mapping):
                continue
            for key in (
                "requirements_json_parse_errors",
                "selection_json_parse_errors",
                "payload_json_parse_errors",
            ):
                if key in stage:
                    checks.append(not stage[key])
        return all(checks) if checks else None
    decision = nested(result, "validation", "decision_json_syntax_valid")
    xml = nested(result, "validation", "xml", "syntax_valid")
    checks = [value for value in (decision, xml) if isinstance(value, bool)]
    return all(checks) if checks else None


def unsafe_parameter(result: Mapping[str, Any]) -> bool | None:
    if result.get("decision_outcome") != "plan":
        return None
    spatial = nested(result, "validation", "spatial_review")
    semantics = nested(result, "validation", "automated_semantics")
    problems: list[bool] = []
    if isinstance(spatial, Mapping) and isinstance(spatial.get("approved"), bool):
        problems.append(not spatial["approved"])
    if isinstance(semantics, Mapping) and isinstance(
        semantics.get("concrete_values_present"), bool
    ):
        problems.append(not semantics["concrete_values_present"])
    final_errors = result.get("final_validation_errors")
    if isinstance(final_errors, list):
        problems.append(bool(final_errors))
    return any(problems) if problems else False


def discover(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    expected_protocol_hashes = protocol_hashes()
    for result_path in sorted(root.glob("*/result.json")):
        request_path = result_path.with_name("request.json")
        if not request_path.is_file():
            continue
        request = load_json(request_path)
        result = load_json(result_path)
        xml_validation = nested(result, "validation", "xml") or {}
        action_metrics = nested(result, "validation", "action_library_metrics") or {}
        scale = request.get("scale_condition") or result.get("scale_condition") or {}
        calls = model_calls(result)
        latency = sum(float(call.get("latency_s", 0.0) or 0.0) for call in calls)
        variant_id = request.get("variant_id") or ""
        base_condition = f"{request['method']}:{request['model_key']}"
        condition = (
            f"{base_condition}:{variant_id}"
            if variant_id and request["experiment"] in {"E2", "E3"}
            else base_condition
        )
        mission = request.get("mission", {})
        paraphrase = request.get("paraphrase", {})
        reference = request.get("evaluation_reference") or {}
        expected_outcome = request["expected_outcome"]
        decision_outcome = result.get("decision_outcome")
        primary_success = (
            result.get("correct_outcome")
            if request["experiment"] in {"E2", "E4"} and expected_outcome != "plan"
            else result.get("automated_task_success")
        )
        refinement_count = automatic_refinement_count(result)
        first_task_success = result.get("first_attempt_task_success")
        if first_task_success is None and refinement_count == 0:
            first_task_success = result.get("automated_task_success")
        false_acceptance = (
            decision_outcome == "plan"
            if request["experiment"] == "E4"
            and expected_outcome in {"clarification", "refusal"}
            else None
        )
        rows.append(
            {
                "condition_id": request["condition_id"],
                "experiment": request["experiment"],
                "method": request["method"],
                "model_key": request["model_key"],
                "base_condition": base_condition,
                "condition": condition,
                "mission_id": request["mission_id"],
                "platform": mission.get("platform"),
                "complexity": nested(mission, "complexity", "label"),
                "paraphrase_id": request["paraphrase_id"],
                "specificity": paraphrase.get("specificity"),
                "variant_id": variant_id,
                "context_condition": reference.get("condition"),
                "adverse_type": reference.get("adverse_type"),
                "action_node_count": scale.get("action_node_count"),
                "tree_count": scale.get("tree_count"),
                "scale_value": scale.get("action_node_count", scale.get("tree_count")),
                "repetition": request["repetition"],
                "status": result.get("status"),
                "scored": result.get("scored") is True,
                "scored_protocol": request.get("scored_protocol") is True,
                "protocol_hashes_match": request.get("protocol_hashes")
                == expected_protocol_hashes,
                "expected_outcome": expected_outcome,
                "decision_outcome": decision_outcome,
                "correct_outcome": result.get("correct_outcome"),
                "automated_task_success": result.get("automated_task_success"),
                "primary_success": primary_success,
                "first_attempt_valid": result.get("first_attempt_valid"),
                "first_attempt_task_success": first_task_success,
                "tree_match": result.get("tree_match"),
                "syntax_valid": syntax_valid(result, request["method"]),
                "interface_valid_static": xml_validation.get("interface_valid_static"),
                "factory_load": xml_validation.get("factory_load", "not_applicable"),
                "factory_load_pass": (
                    xml_validation.get("factory_load") == "pass"
                    if xml_validation.get("factory_load") in {"pass", "fail"}
                    else None
                ),
                "payload_valid": (
                    result.get("final_payload_valid")
                    if request["method"] == "M3" and expected_outcome == "plan"
                    else None
                ),
                "required_node_recall": action_metrics.get("required_node_recall"),
                "distractor_use": action_metrics.get("distractor_use"),
                "distractor_use_count": action_metrics.get("distractor_use_count"),
                "distractor_nodes_used": ";".join(
                    action_metrics.get("used_distractor_nodes", [])
                ),
                "invented_node_count": action_metrics.get("invented_node_count"),
                "invented_nodes": ";".join(action_metrics.get("invented_nodes", [])),
                "port_error_count": action_metrics.get("port_error_count"),
                "planning_call_count": len(calls),
                "automatic_refinement_count": refinement_count,
                "transport_retry_count": transport_retry_count(calls),
                "input_tokens": token_total(calls, "input"),
                "output_tokens": token_total(calls, "output"),
                "cost_usd": float(result.get("cost_usd", 0.0) or 0.0),
                "latency_s": round(latency, 3),
                "clarification_outcome_correct": (
                    decision_outcome == "clarification"
                    if request["experiment"] == "E4"
                    and expected_outcome == "clarification"
                    else None
                ),
                "refusal_outcome_correct": (
                    decision_outcome == "refusal"
                    if request["experiment"] == "E4" and expected_outcome == "refusal"
                    else None
                ),
                "false_acceptance": false_acceptance,
                "unsafe_parameter": (
                    unsafe_parameter(result) if request["experiment"] == "E4" else None
                ),
                "human_semantic_total": None,
                "human_semantic_max": None,
                "human_semantic_fraction": None,
                "human_semantic_pass": None,
                "adjudicated_task_success": None,
                "adequate_failure_response": None,
                "correct_clarification": None,
                "correct_refusal": None,
                "guard_detection": None,
                "manual_repair_status": (
                    "not_applicable"
                    if request["experiment"] == "E1"
                    and result.get("status")
                    in {"dry_run", "blocked", "transport_error", "protocol_error"}
                    else None
                ),
                "manual_correction_count": None,
                "manual_correction_types": "",
                "manual_time_s": None,
                "corrected_artifact_path": None,
                "post_repair_validation_path": None,
                "post_repair_deterministic_pass": None,
                "post_repair_semantic_pass": None,
                "success_after_manual_repair": None,
                "result_path": str(result_path.resolve()),
            }
        )
    return rows


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{line_number}: expected a JSON object")
        rows.append(value)
    return rows


def validate_scores(scores: Any, elements: list[str], label: str) -> dict[str, int]:
    if not isinstance(scores, Mapping) or set(scores) != set(elements):
        raise ValueError(f"{label}: scores must contain exactly {elements}")
    if not all(isinstance(value, int) and 0 <= value <= 2 for value in scores.values()):
        raise ValueError(f"{label}: every score must be an integer from 0 to 2")
    return {name: int(scores[name]) for name in elements}


def validate_correction(
    correction: Any,
    allowed_types: set[str],
    experiment: str,
    label: str,
    response_directory: Path,
    elements: list[str],
) -> dict[str, Any]:
    if not isinstance(correction, Mapping):
        raise ValueError(f"{label}: correction record is missing")
    status = correction.get("status")
    allowed_statuses = {
        "not_needed",
        "corrected",
        "attempted_failed",
        "not_attempted",
        "not_applicable",
    }
    if status not in allowed_statuses:
        raise ValueError(f"{label}: unknown correction status {status!r}")
    if experiment != "E1" and status != "not_applicable":
        raise ValueError(f"{label}: manual repair is scored only in E1")
    count = correction.get("manual_correction_count")
    seconds = correction.get("manual_time_s")
    types = correction.get("manual_correction_types", [])
    artifact_path = correction.get("corrected_artifact_path")
    validation_path = correction.get("post_repair_validation_path")
    deterministic_pass = correction.get("post_repair_deterministic_pass")
    raw_post_scores = correction.get("post_repair_scores")
    post_scores = (
        validate_scores(raw_post_scores, elements, f"{label} post-repair")
        if raw_post_scores is not None
        else None
    )
    semantic_pass = (
        all(value == 2 for value in post_scores.values()) if post_scores else None
    )
    if (
        not isinstance(types, list)
        or len(types) != len(set(types))
        or not set(types).issubset(allowed_types)
    ):
        raise ValueError(f"{label}: invalid manual correction type")
    if status == "not_needed":
        if count != 0 or seconds != 0:
            raise ValueError(f"{label}: not_needed requires 0 corrections and 0 seconds")
    elif status == "corrected":
        if not isinstance(count, int) or count < 1:
            raise ValueError(f"{label}: corrected requires at least one correction")
        if not types:
            raise ValueError(f"{label}: corrected requires at least one correction type")
        if not isinstance(seconds, (int, float)) or not 0 <= seconds <= 300:
            raise ValueError(f"{label}: corrected time must be within 0 to 300 seconds")
        if deterministic_pass is not True or semantic_pass is not True:
            raise ValueError(f"{label}: corrected requires both post-repair checks to pass")
        for name, value in (
            ("corrected artifact", artifact_path),
            ("post-repair validation", validation_path),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(f"{label}: {name} path is missing")
            resolved = Path(value)
            if not resolved.is_absolute():
                resolved = response_directory / resolved
            if not resolved.is_file():
                raise ValueError(f"{label}: {name} does not exist: {resolved}")
    elif status == "attempted_failed":
        if not isinstance(count, int) or count < 0:
            raise ValueError(f"{label}: attempted_failed needs a correction count")
        if not isinstance(seconds, (int, float)) or not 0 <= seconds <= 300:
            raise ValueError(f"{label}: attempted time must be within 0 to 300 seconds")
        if count and not types:
            raise ValueError(f"{label}: recorded corrections require a correction type")
        if deterministic_pass is True and semantic_pass is True:
            raise ValueError(f"{label}: attempted_failed cannot have both checks pass")
    elif status in {"not_attempted", "not_applicable"}:
        if count is not None or seconds is not None:
            raise ValueError(f"{label}: {status} effort values must be null")
    return {**correction, "post_repair_semantic_pass": semantic_pass}


def quadratic_weighted_kappa(pairs: list[tuple[int, int]]) -> float | None:
    if not pairs:
        return None
    matrix = [[0, 0, 0] for _ in range(3)]
    for first, second in pairs:
        matrix[first][second] += 1
    total = len(pairs)
    first_counts = [sum(row) for row in matrix]
    second_counts = [sum(matrix[row][column] for row in range(3)) for column in range(3)]
    observed = sum(
        ((row - column) ** 2 / 4) * matrix[row][column]
        for row in range(3)
        for column in range(3)
    ) / total
    expected = sum(
        ((row - column) ** 2 / 4)
        * first_counts[row]
        * second_counts[column]
        / total**2
        for row in range(3)
        for column in range(3)
    )
    return 1.0 if expected == 0 and observed == 0 else (None if expected == 0 else 1 - observed / expected)


def apply_reviews(
    rows: list[dict[str, Any]], responses_path: Path, key_path: Path, rubric: Mapping[str, Any]
) -> dict[str, Any]:
    key = load_json(key_path)
    by_condition = {row["condition_id"]: row for row in rows}
    agreement_pairs: list[tuple[int, int]] = []
    reviewed: set[str] = set()
    allowed_types = set(rubric["effort_metrics"]["manual_correction_types"])
    for response in load_jsonl(responses_path):
        review_id = response.get("review_id")
        if review_id not in key:
            raise ValueError(f"Unknown review_id {review_id!r}")
        condition_id = key[review_id]["condition_id"]
        if condition_id not in by_condition:
            continue
        if condition_id in reviewed:
            raise ValueError(f"Duplicate review response for {review_id}")
        reviewed.add(condition_id)
        elements = list(key[review_id]["applicable_elements"])
        reviewers = response.get("reviewers")
        minimum = 2 if key[review_id].get("second_review_required") else 1
        if not isinstance(reviewers, list) or len(reviewers) != minimum:
            raise ValueError(f"{review_id}: exactly {minimum} reviewer record(s) required")
        reviewer_ids = [item.get("reviewer_id") for item in reviewers if isinstance(item, Mapping)]
        if len(reviewer_ids) != minimum or any(not value for value in reviewer_ids) or len(set(reviewer_ids)) != minimum:
            raise ValueError(f"{review_id}: reviewer IDs must be present and unique")
        reviewer_scores = [
            validate_scores(item.get("scores"), elements, f"{review_id} reviewer")
            for item in reviewers
            if isinstance(item, Mapping)
        ]
        if len(reviewer_scores) != len(reviewers):
            raise ValueError(f"{review_id}: malformed reviewer record")
        disagreement = any(scores != reviewer_scores[0] for scores in reviewer_scores[1:])
        if len(reviewer_scores) >= 2:
            agreement_pairs.extend(
                (reviewer_scores[0][element], reviewer_scores[1][element])
                for element in elements
            )
        if disagreement:
            final_scores = validate_scores(
                response.get("adjudicated_scores"), elements, f"{review_id} adjudication"
            )
        else:
            final_scores = reviewer_scores[0]
        row = by_condition[condition_id]
        total = sum(final_scores.values())
        maximum = 2 * len(elements)
        semantic_pass = all(value == 2 for value in final_scores.values())
        row.update(
            {
                "human_semantic_total": total,
                "human_semantic_max": maximum,
                "human_semantic_fraction": round(total / maximum, 6),
                "human_semantic_pass": semantic_pass,
                "adjudicated_task_success": bool(row["primary_success"] and semantic_pass),
            }
        )
        if row["experiment"] == "E4":
            adequate = bool(row["correct_outcome"] and semantic_pass)
            row["adequate_failure_response"] = (
                adequate if row["expected_outcome"] != "plan" else None
            )
            row["correct_clarification"] = (
                adequate if row["expected_outcome"] == "clarification" else None
            )
            row["correct_refusal"] = (
                adequate if row["expected_outcome"] == "refusal" else None
            )
            row["guard_detection"] = bool(row["primary_success"] and semantic_pass)
        correction = validate_correction(
            response.get("correction"),
            allowed_types,
            row["experiment"],
            review_id,
            responses_path.parent,
            elements,
        )
        row.update(
            {
                "manual_repair_status": correction["status"],
                "manual_correction_count": correction.get("manual_correction_count"),
                "manual_correction_types": ";".join(
                    correction.get("manual_correction_types", [])
                ),
                "manual_time_s": correction.get("manual_time_s"),
                "corrected_artifact_path": correction.get("corrected_artifact_path"),
                "post_repair_validation_path": correction.get(
                    "post_repair_validation_path"
                ),
                "post_repair_deterministic_pass": correction.get(
                    "post_repair_deterministic_pass"
                ),
                "post_repair_semantic_pass": correction.get(
                    "post_repair_semantic_pass"
                ),
                "success_after_manual_repair": (
                    correction["status"] in {"not_needed", "corrected"}
                    if row["experiment"] == "E1"
                    else None
                ),
            }
        )
        if row["experiment"] == "E1":
            needs_repair = not row["adjudicated_task_success"]
            if correction["status"] == "not_needed" and needs_repair:
                raise ValueError(f"{review_id}: failing artifact cannot be marked not_needed")
            if correction["status"] in {"corrected", "attempted_failed", "not_attempted"} and not needs_repair:
                raise ValueError(f"{review_id}: passing artifact does not require repair")
    exact = (
        sum(first == second for first, second in agreement_pairs) / len(agreement_pairs)
        if agreement_pairs
        else None
    )
    return {
        "reviewed_artifacts": len(reviewed),
        "missing_review_artifacts": sum(
            row["status"] not in {"dry_run", "blocked", "transport_error", "protocol_error"}
            and row["condition_id"] not in reviewed
            for row in rows
        ),
        "double_scored_element_count": len(agreement_pairs),
        "exact_element_agreement": round(exact, 6) if exact is not None else None,
        "quadratic_weighted_kappa": (
            round(value, 6) if (value := quadratic_weighted_kappa(agreement_pairs)) is not None else None
        ),
    }


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def cluster_bootstrap_rate(
    rows: list[dict[str, Any]], field: str, samples: int = 10000
) -> tuple[float | None, float | None]:
    clusters: dict[str, list[bool]] = defaultdict(list)
    for row in rows:
        if isinstance(row.get(field), bool):
            clusters[row["mission_id"]].append(row[field])
    if len(clusters) < 2:
        return None, None
    identifiers = sorted(clusters)
    rng = random.Random(42)
    estimates: list[float] = []
    for _ in range(samples):
        selected = [rng.choice(identifiers) for _ in identifiers]
        values = [value for identifier in selected for value in clusters[identifier]]
        estimates.append(sum(values) / len(values))
    return percentile(estimates, 0.025), percentile(estimates, 0.975)


def rate(rows: list[dict[str, Any]], field: str) -> tuple[int, int, float | None]:
    values = [row[field] for row in rows if isinstance(row.get(field), bool)]
    successes = sum(values)
    return successes, len(values), successes / len(values) if values else None


def numeric(rows: list[dict[str, Any]], field: str) -> list[float]:
    return [
        float(row[field])
        for row in rows
        if isinstance(row.get(field), (int, float)) and not isinstance(row[field], bool)
    ]


def median_iqr(values: list[float]) -> tuple[float | None, float | None, float | None]:
    if not values:
        return None, None, None
    return percentile(values, 0.5), percentile(values, 0.25), percentile(values, 0.75)


RATE_FIELDS = (
    "primary_success",
    "correct_outcome",
    "syntax_valid",
    "interface_valid_static",
    "factory_load_pass",
    "payload_valid",
    "tree_match",
    "first_attempt_valid",
    "first_attempt_task_success",
    "automated_task_success",
    "human_semantic_pass",
    "adjudicated_task_success",
    "clarification_outcome_correct",
    "refusal_outcome_correct",
    "adequate_failure_response",
    "correct_clarification",
    "correct_refusal",
    "false_acceptance",
    "unsafe_parameter",
    "guard_detection",
    "distractor_use",
    "success_after_manual_repair",
)


def summarize_group(
    rows: list[dict[str, Any]], breakdown: str, group: str
) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "breakdown": breakdown,
        "group": group,
        "experiment": rows[0]["experiment"],
        "n_artifacts": len(rows),
        "n_missions": len({row["mission_id"] for row in rows}),
        "n_complete": sum(row["status"] == "complete" for row in rows),
        "n_validation_failed": sum(row["status"] == "validation_failed" for row in rows),
        "n_transport_error": sum(row["status"] == "transport_error" for row in rows),
        "n_protocol_error": sum(row["status"] == "protocol_error" for row in rows),
        "n_blocked": sum(row["status"] == "blocked" for row in rows),
    }
    for field in RATE_FIELDS:
        successes, denominator, value = rate(rows, field)
        summary[f"{field}_successes"] = successes
        summary[f"{field}_n"] = denominator
        summary[f"{field}_rate"] = round(value, 6) if value is not None else None
    low, high = cluster_bootstrap_rate(rows, "primary_success")
    summary["primary_success_cluster_ci95_low"] = round(low, 6) if low is not None else None
    summary["primary_success_cluster_ci95_high"] = round(high, 6) if high is not None else None
    factory = [row["factory_load"] for row in rows]
    summary["factory_pass"] = factory.count("pass")
    summary["factory_fail"] = factory.count("fail")
    summary["factory_not_run"] = factory.count("not_run")
    summary["factory_not_applicable"] = factory.count("not_applicable")
    for field in (
        "latency_s",
        "planning_call_count",
        "automatic_refinement_count",
        "transport_retry_count",
        "input_tokens",
        "output_tokens",
        "cost_usd",
        "required_node_recall",
        "distractor_use_count",
        "invented_node_count",
        "port_error_count",
        "human_semantic_fraction",
    ):
        values = numeric(rows, field)
        summary[f"mean_{field}"] = round(sum(values) / len(values), 6) if values else None
        summary[f"sum_{field}"] = round(sum(values), 6) if values else None
        summary[f"{field}_n"] = len(values)
    correction_statuses = Counter(
        row["manual_repair_status"] for row in rows if row["manual_repair_status"]
    )
    for status in (
        "not_needed",
        "corrected",
        "attempted_failed",
        "not_attempted",
        "not_applicable",
    ):
        summary[f"manual_{status}"] = correction_statuses[status]
    correction_types = Counter(
        name
        for row in rows
        for name in str(row["manual_correction_types"]).split(";")
        if name
    )
    for correction_type in MANUAL_CORRECTION_TYPES:
        summary[f"manual_{correction_type}_artifact_count"] = correction_types[
            correction_type
        ]
    for field in ("manual_correction_count", "manual_time_s"):
        values = numeric(rows, field)
        median, q1, q3 = median_iqr(values)
        summary[f"median_{field}"] = round(median, 3) if median is not None else None
        summary[f"q1_{field}"] = round(q1, 3) if q1 is not None else None
        summary[f"q3_{field}"] = round(q3, 3) if q3 is not None else None
        summary[f"{field}_n"] = len(values)
        attempted = numeric(
            [
                row
                for row in rows
                if row["manual_repair_status"] in {"corrected", "attempted_failed"}
            ],
            field,
        )
        median, q1, q3 = median_iqr(attempted)
        summary[f"median_attempted_{field}"] = (
            round(median, 3) if median is not None else None
        )
        summary[f"q1_attempted_{field}"] = round(q1, 3) if q1 is not None else None
        summary[f"q3_attempted_{field}"] = round(q3, 3) if q3 is not None else None
        summary[f"attempted_{field}_n"] = len(attempted)
    return summary


def aggregate_by(
    rows: list[dict[str, Any]], fields: tuple[str, ...], breakdown: str
) -> list[dict[str, Any]]:
    groups: dict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[tuple(row.get(field) for field in fields)].append(row)
    return [
        summarize_group(members, breakdown, " | ".join(str(value) for value in key))
        for key, members in sorted(groups.items(), key=lambda item: str(item[0]))
    ]


def breakdowns(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    e1 = [row for row in rows if row["experiment"] == "E1"]
    for field in ("platform", "specificity", "complexity"):
        if e1:
            output.extend(aggregate_by(e1, ("condition", field), f"E1_{field}"))
    e2 = [row for row in rows if row["experiment"] == "E2"]
    if e2:
        output.extend(
            aggregate_by(
                e2,
                ("base_condition", "context_condition", "expected_outcome"),
                "E2_context",
            )
        )
    e3 = [row for row in rows if row["experiment"] == "E3"]
    if e3:
        output.extend(
            aggregate_by(
                e3,
                ("base_condition", "variant_id", "scale_value"),
                "E3_scale",
            )
        )
    e4 = [row for row in rows if row["experiment"] == "E4"]
    if e4:
        output.extend(
            aggregate_by(e4, ("condition", "adverse_type"), "E4_adverse_type")
        )
        output.extend(
            aggregate_by(e4, ("condition", "expected_outcome"), "E4_expected_outcome")
        )
    return output


def exact_sign_flip_p(differences: list[float]) -> float | None:
    nonzero = [value for value in differences if value != 0]
    if not nonzero:
        return 1.0 if differences else None
    observed = abs(sum(nonzero) / len(nonzero))
    extreme = 0
    total = 2 ** len(nonzero)
    for signs in itertools.product((-1, 1), repeat=len(nonzero)):
        permuted = abs(sum(sign * value for sign, value in zip(signs, nonzero)) / len(nonzero))
        extreme += permuted >= observed - 1e-12
    return extreme / total


def paired_difference_interval(differences: list[float]) -> tuple[float | None, float | None]:
    if len(differences) < 2:
        return None, None
    rng = random.Random(42)
    estimates = [
        sum(rng.choice(differences) for _ in differences) / len(differences)
        for _ in range(10000)
    ]
    return percentile(estimates, 0.025), percentile(estimates, 0.975)


def comparison_allowed(first: dict[str, Any], second: dict[str, Any]) -> bool:
    experiment = first["experiment"]
    if experiment != second["experiment"]:
        return False
    if experiment == "E1":
        return (
            first["model_key"] == second["model_key"]
            and {first["method"], second["method"]} == {"M1", "M3"}
        )
    if experiment == "E2":
        return (
            first["base_condition"] == second["base_condition"]
            and "complete" in {first["context_condition"], second["context_condition"]}
        )
    if experiment == "E3":
        if first["base_condition"] != second["base_condition"]:
            return False
        values = sorted(
            {
                row["scale_value"]
                for row in (first, second)
                if isinstance(row.get("scale_value"), (int, float))
            }
        )
        all_values = first.get("all_scale_values", [])
        return len(values) == 2 and all_values.index(values[1]) - all_values.index(values[0]) == 1
    return False


def paired_comparisons(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if isinstance(row["primary_success"], bool):
            grouped[(row["experiment"], row["condition"])].append(row)
    scale_values: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        if row["experiment"] == "E3" and isinstance(row.get("scale_value"), (int, float)):
            scale_values[row["base_condition"]].append(row["scale_value"])
    metadata: dict[tuple[str, str], dict[str, Any]] = {}
    values: dict[tuple[str, str], dict[tuple[Any, ...], bool]] = {}
    for group, members in grouped.items():
        first = dict(members[0])
        first["all_scale_values"] = sorted(set(scale_values[first["base_condition"]]))
        metadata[group] = first
        values[group] = {
            (row["mission_id"], row["paraphrase_id"], row["repetition"]): row[
                "primary_success"
            ]
            for row in members
        }
    output: list[dict[str, Any]] = []
    for first_group, second_group in itertools.combinations(sorted(values), 2):
        first_meta, second_meta = metadata[first_group], metadata[second_group]
        if not comparison_allowed(first_meta, second_meta):
            continue
        shared = sorted(set(values[first_group]) & set(values[second_group]))
        if not shared:
            continue
        mission_differences: dict[str, list[float]] = defaultdict(list)
        for key in shared:
            mission_differences[key[0]].append(
                float(values[first_group][key]) - float(values[second_group][key])
            )
        cluster_differences = [
            sum(items) / len(items) for items in mission_differences.values()
        ]
        effect = sum(cluster_differences) / len(cluster_differences)
        low, high = paired_difference_interval(cluster_differences)
        output.append(
            {
                "experiment": first_group[0],
                "metric": "condition_appropriate_success",
                "condition_a": first_group[1],
                "condition_b": second_group[1],
                "paired_artifact_n": len(shared),
                "paired_mission_n": len(cluster_differences),
                "a_success_rate": round(
                    sum(values[first_group][key] for key in shared) / len(shared), 6
                ),
                "b_success_rate": round(
                    sum(values[second_group][key] for key in shared) / len(shared), 6
                ),
                "paired_risk_difference_a_minus_b": round(effect, 6),
                "cluster_ci95_low": round(low, 6) if low is not None else None,
                "cluster_ci95_high": round(high, 6) if high is not None else None,
                "exact_mission_sign_flip_p_two_sided": round(
                    exact_sign_flip_p(cluster_differences), 8
                ),
            }
        )
    return output


def catalogue_coverage(core: Mapping[str, Any], runtime: Mapping[str, Any]) -> list[dict[str, Any]]:
    trees = {tree["id"]: tree for tree in runtime["tree_catalogue"]}
    records: list[dict[str, Any]] = []
    for method in ("M1", "M2", "M3"):
        for mission in core["missions"]:
            tree = trees.get(mission["expected"]["tree_id"])
            covered = mission.get("support_status") == "implemented_catalogue" and tree is not None
            if method == "M3" and covered:
                covered = set(mission["requirements"]["required_capabilities"]).issubset(
                    tree.get("required_capabilities", [])
                )
            records.append(
                {
                    "method": method,
                    "platform": mission["platform"],
                    "mission_id": mission["id"],
                    "covered": covered,
                }
            )
    output: list[dict[str, Any]] = []
    for method in ("M1", "M2", "M3"):
        for platform in ("all", "husky", "blueboat"):
            selected = [
                row
                for row in records
                if row["method"] == method
                and (platform == "all" or row["platform"] == platform)
            ]
            count = sum(row["covered"] for row in selected)
            output.append(
                {
                    "method": method,
                    "platform": platform,
                    "covered_missions": count,
                    "mission_n": len(selected),
                    "coverage_rate": round(count / len(selected), 6),
                }
            )
    return output


def score_execution(
    path: Path,
    protocol: Mapping[str, Any],
    protocol_sha256: str | None = None,
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
]:
    source = load_json(path)
    if source.get("protocol_version") != protocol["protocol_version"]:
        raise ValueError("Execution record protocol version does not match")
    if protocol_sha256 and source.get("execution_scoring_sha256") != protocol_sha256:
        raise ValueError("Execution record protocol hash does not match")
    required = set(protocol["trial_required_fields"])
    trials: list[dict[str, Any]] = []
    seen: set[str] = set()
    trial_keys: set[tuple[str, str, int]] = set()
    for trial in source.get("trials", []):
        missing = required - set(trial)
        if missing:
            raise ValueError(f"Execution trial is missing {sorted(missing)}")
        if trial["trial_id"] in seen:
            raise ValueError(f"Duplicate execution trial {trial['trial_id']}")
        seen.add(trial["trial_id"])
        mission_requirements = protocol["mission_requirements"].get(trial["mission_id"])
        if mission_requirements is None:
            raise ValueError(f"{trial['trial_id']}: mission is outside the E5 protocol")
        if trial["platform"] != mission_requirements["platform"]:
            raise ValueError(f"{trial['trial_id']}: platform does not match the mission")
        if not isinstance(trial["planning_condition_id"], str) or not trial["planning_condition_id"]:
            raise ValueError(f"{trial['trial_id']}: planning condition is missing")
        if trial["planning_passed"] is not True:
            raise ValueError(f"{trial['trial_id']}: E5 requires a planning-pass artifact")
        if trial.get("started") is not True:
            raise ValueError(f"{trial['trial_id']}: only started trials belong in the trial array")
        if not isinstance(trial.get("repetition"), int) or trial["repetition"] < 1:
            raise ValueError(f"{trial['trial_id']}: repetition must be a positive integer")
        trial_key = (trial["mission_id"], trial["evidence_level"], trial["repetition"])
        if trial_key in trial_keys:
            raise ValueError(f"{trial['trial_id']}: duplicate mission/evidence/repetition")
        trial_keys.add(trial_key)
        if not isinstance(trial.get("duration_s"), (int, float)) or trial["duration_s"] < 0:
            raise ValueError(f"{trial['trial_id']}: duration_s must be non-negative")
        if trial["evidence_level"] not in protocol["evidence_levels"]:
            raise ValueError(f"{trial['trial_id']}: invalid evidence level")
        if trial["terminal_outcome"] not in protocol["terminal_outcomes"]:
            raise ValueError(f"{trial['trial_id']}: invalid terminal outcome")
        if trial["factory_load"] not in protocol["factory_load_values"]:
            raise ValueError(f"{trial['trial_id']}: invalid factory-load value")
        if trial["platform"] not in protocol["integration_checks_by_platform"]:
            raise ValueError(f"{trial['trial_id']}: unknown platform")
        expected_checks = set(protocol["integration_checks_by_platform"][trial["platform"]])
        checks = trial["integration_checks"]
        if not isinstance(checks, Mapping) or set(checks) != expected_checks:
            raise ValueError(
                f"{trial['trial_id']}: integration checks must contain exactly {sorted(expected_checks)}"
            )
        if any(value not in protocol["integration_check_values"] for value in checks.values()):
            raise ValueError(f"{trial['trial_id']}: invalid integration-check value")
        ratios: dict[str, float | None] = {}
        for name, numerator, denominator in (
            ("route_completion", "reached_waypoints", "required_waypoints"),
            ("measurement_coverage", "valid_measurements", "required_measurements"),
            ("photo_coverage", "valid_photos", "required_photos"),
        ):
            required_count = trial[denominator]
            actual_count = trial[numerator]
            if required_count != mission_requirements[denominator]:
                raise ValueError(f"{trial['trial_id']}: {denominator} differs from the frozen mission requirement")
            if not isinstance(required_count, int) or required_count < 0:
                raise ValueError(f"{trial['trial_id']}: invalid {denominator}")
            if not isinstance(actual_count, int) or actual_count < 0:
                raise ValueError(f"{trial['trial_id']}: invalid {numerator}")
            ratios[name] = min(actual_count / required_count, 1.0) if required_count else None
        applicable = [value for value in ratios.values() if value is not None]
        if not applicable:
            raise ValueError(f"{trial['trial_id']}: no task-quality denominator")
        interventions = trial["operator_interventions"]
        incidents = trial["safety_incidents"]
        if not isinstance(interventions, list) or not isinstance(incidents, list):
            raise ValueError(f"{trial['trial_id']}: interventions and incidents must be arrays")
        if any(item.get("type") not in protocol["intervention_types"] for item in interventions):
            raise ValueError(f"{trial['trial_id']}: invalid intervention type")
        if any(item.get("type") not in protocol["safety_incident_types"] for item in incidents):
            raise ValueError(f"{trial['trial_id']}: invalid safety incident type")
        task_quality = min(applicable)
        completion = trial["terminal_outcome"] == "completed" and task_quality == 1.0
        trials.append(
            {
                **trial,
                **ratios,
                "task_quality": task_quality,
                "mission_completion": completion,
                "autonomous_success": completion and not interventions and not incidents,
                "operator_intervention_count": len(interventions),
                "safety_incident_count": len(incidents),
            }
        )
    not_started: list[dict[str, Any]] = []
    not_started_keys: set[tuple[str, str, int]] = set()
    not_started_required = set(protocol["not_started_required_fields"])
    for record in source.get("not_started", []):
        missing = not_started_required - set(record)
        if missing:
            raise ValueError(f"Non-started trial is missing {sorted(missing)}")
        if record["trial_id"] in seen:
            raise ValueError(f"Duplicate execution trial {record['trial_id']}")
        seen.add(record["trial_id"])
        mission_requirements = protocol["mission_requirements"].get(record["mission_id"])
        if mission_requirements is None or record["platform"] != mission_requirements["platform"]:
            raise ValueError(f"{record['trial_id']}: mission and platform do not match")
        if record["evidence_level"] not in protocol["evidence_levels"]:
            raise ValueError(f"{record['trial_id']}: invalid evidence level")
        if not isinstance(record["repetition"], int) or record["repetition"] < 1:
            raise ValueError(f"{record['trial_id']}: repetition must be a positive integer")
        if not isinstance(record["reason"], str) or not record["reason"].strip():
            raise ValueError(f"{record['trial_id']}: non-start reason is missing")
        key = (record["mission_id"], record["evidence_level"], record["repetition"])
        if key in trial_keys or key in not_started_keys:
            raise ValueError(f"{record['trial_id']}: duplicate mission/evidence/repetition")
        not_started_keys.add(key)
        not_started.append(dict(record))
    summaries: list[dict[str, Any]] = []
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for trial in trials:
        groups[(trial["platform"], trial["evidence_level"])].append(trial)
    for (platform, evidence), members in sorted(groups.items()):
        completion = sum(item["mission_completion"] for item in members)
        autonomous = sum(item["autonomous_success"] for item in members)
        summary = {
                "platform": platform,
                "evidence_level": evidence,
                "trial_n": len(members),
                "mission_completions": completion,
                "mission_completion_rate": round(completion / len(members), 6),
                "autonomous_successes": autonomous,
                "autonomous_success_rate": round(autonomous / len(members), 6),
                "mean_task_quality": round(
                    sum(item["task_quality"] for item in members) / len(members), 6
                ),
                "mean_duration_s": round(
                    sum(float(item["duration_s"]) for item in members) / len(members), 3
                ),
                "operator_interventions": sum(
                    item["operator_intervention_count"] for item in members
                ),
                "safety_incidents": sum(item["safety_incident_count"] for item in members),
            }
        for check in protocol["integration_checks_by_platform"][platform]:
            for value in protocol["integration_check_values"]:
                summary[f"integration_{check}_{value}"] = sum(
                    item["integration_checks"][check] == value for item in members
                )
        summaries.append(summary)
    primary = protocol["primary_trial_plan"]
    planned = {
        (mission_id, primary["evidence_level"], repetition)
        for mission_id in primary["mission_ids"]
        for repetition in range(1, primary["repetitions_per_mission"] + 1)
    }
    observed = planned & trial_keys
    planned_not_started = planned & not_started_keys
    unaccounted = planned - observed - planned_not_started
    summaries.append(
        {
            "platform": "all",
            "evidence_level": "primary_plan_coverage",
            "trial_n": len(observed),
            "planned_trials": len(planned),
            "missing_planned_trials": len(planned - observed),
            "not_started_planned_trials": len(planned_not_started),
            "unaccounted_planned_trials": len(unaccounted),
            "primary_plan_complete": observed == planned,
            "primary_plan_accounted": not unaccounted,
        }
    )
    expected_components = set(protocol["portability_components"])
    portability = source.get("portability", [])
    if portability:
        found = {item.get("component") for item in portability}
        if found != expected_components or len(found) != len(portability):
            raise ValueError("Portability records must contain every component exactly once")
        for item in portability:
            if item.get("classification") not in protocol["portability_classes"]:
                raise ValueError(f"Invalid portability class for {item.get('component')}")
            if not item.get("evidence"):
                raise ValueError(f"Missing portability evidence for {item.get('component')}")
    class_counts = Counter(item.get("classification") for item in portability)
    portability_summary = [
        {
            "component_n": len(portability),
            "shared_unchanged": class_counts["shared_unchanged"],
            "shared_configured": class_counts["shared_configured"],
            "platform_specific": class_counts["platform_specific"],
            "shared_count": class_counts["shared_unchanged"] + class_counts["shared_configured"],
            "shared_proportion": round(
                (class_counts["shared_unchanged"] + class_counts["shared_configured"])
                / len(portability),
                6,
            ) if portability else None,
        }
    ]
    return trials, summaries, portability_summary, not_started


def write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    values = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not values:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(key for row in values for key in row))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(values)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument("--output-dir", type=Path, default=EVALUATION / "results")
    parser.add_argument("--reviews", type=Path)
    parser.add_argument("--review-key", type=Path)
    parser.add_argument("--execution-file", type=Path)
    parser.add_argument(
        "--include-unscored",
        action="store_true",
        help="Include pilot artifacts. Never use this flag for final thesis tables.",
    )
    args = parser.parse_args()

    discovered = discover(args.input_dir)
    rows = discovered if args.include_unscored else [row for row in discovered if row["scored"]]
    if not rows and not args.execution_file:
        qualifier = "" if args.include_unscored else " scored"
        print(f"No{qualifier} evaluation artifacts found in {args.input_dir}.", file=sys.stderr)
        return 2
    if not args.include_unscored:
        stale = [
            row["condition_id"]
            for row in rows
            if not row["scored_protocol"] or not row["protocol_hashes_match"]
        ]
        if stale:
            raise ValueError(
                f"{len(stale)} scored artifact(s) lack the current frozen protocol hashes"
            )
        if rows and not args.reviews:
            parser.error("--reviews and --review-key are required for complete scored results")
    rubric = load_json(PROTOCOL / "scoring_rubric.json")
    if tuple(rubric["effort_metrics"]["manual_correction_types"]) != MANUAL_CORRECTION_TYPES:
        raise ValueError("Scorer correction types differ from scoring_rubric.json")
    review_summary: dict[str, Any] | None = None
    if args.reviews:
        if not args.review_key:
            parser.error("--review-key is required with --reviews")
        review_summary = apply_reviews(rows, args.reviews, args.review_key, rubric)
        if not args.include_unscored and review_summary["missing_review_artifacts"]:
            raise ValueError(
                f"{review_summary['missing_review_artifacts']} scored artifact(s) lack human review"
            )
    summaries = aggregate_by(rows, ("experiment", "condition"), "condition")
    detailed = breakdowns(rows)
    comparisons = paired_comparisons(rows)
    core = load_json(PROTOCOL / "core_missions.json")
    runtime = load_json(PROTOCOL / "runtime_contract.json")
    coverage = catalogue_coverage(core, runtime)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "runs.csv", rows)
    write_csv(args.output_dir / "summary.csv", summaries)
    write_csv(args.output_dir / "breakdowns.csv", detailed)
    write_csv(args.output_dir / "paired_comparisons.csv", comparisons)
    write_csv(args.output_dir / "catalogue_coverage.csv", coverage)
    execution_summary: dict[str, Any] | None = None
    if args.execution_file:
        execution_protocol = load_json(PROTOCOL / "execution_scoring.json")
        execution_protocol_sha256 = hashlib.sha256(
            (PROTOCOL / "execution_scoring.json").read_bytes()
        ).hexdigest()
        trials, trial_summaries, portability, not_started = score_execution(
            args.execution_file, execution_protocol, execution_protocol_sha256
        )
        write_csv(args.output_dir / "execution_runs.csv", trials)
        write_csv(args.output_dir / "execution_not_started.csv", not_started)
        write_csv(args.output_dir / "execution_summary.csv", trial_summaries)
        write_csv(args.output_dir / "portability_summary.csv", portability)
        execution_summary = {
            "trial_count": len(trials),
            "not_started_count": len(not_started),
            "protocol_version": execution_protocol["protocol_version"],
            "protocol_freeze_status": execution_protocol["freeze_status"],
            "protocol_sha256": execution_protocol_sha256,
            "groups": trial_summaries,
            "portability": portability,
        }
    write_json(
        args.output_dir / "summary.json",
        {
            "input_dir": str(args.input_dir.resolve()),
            "scoring_rubric_version": rubric["rubric_version"],
            "scoring_rubric_sha256": hashlib.sha256(
                (PROTOCOL / "scoring_rubric.json").read_bytes()
            ).hexdigest(),
            "scorer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "raw_outputs_are_never_repaired_in_place": True,
            "include_unscored": args.include_unscored,
            "artifacts_discovered": len(discovered),
            "artifacts_analyzed": len(rows),
            "review": review_summary,
            "catalogue_coverage": coverage,
            "groups": summaries,
            "breakdowns": detailed,
            "paired_comparisons": comparisons,
            "execution": execution_summary,
        },
    )
    print(f"Analyzed {len(rows)} artifacts ({len(discovered)} discovered) into {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
