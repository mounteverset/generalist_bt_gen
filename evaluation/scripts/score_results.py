#!/usr/bin/env python3
"""Aggregate evaluation artifacts without changing or repairing model outputs."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Mapping

from evaluation_core import EVALUATION, load_json, write_json


def wilson(successes: int, total: int, z: float = 1.96) -> tuple[float, float]:
    if total == 0:
        return 0.0, 0.0
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    margin = (
        z
        * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total))
        / denominator
    )
    return max(0.0, center - margin), min(1.0, center + margin)


def exact_mcnemar_p(b: int, c: int) -> float:
    discordant = b + c
    if discordant == 0:
        return 1.0
    lower = min(b, c)
    probability = sum(
        math.comb(discordant, index) * (0.5**discordant)
        for index in range(lower + 1)
    )
    return min(1.0, 2.0 * probability)


def nested(value: Mapping[str, Any], *path: str) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def discover(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for result_path in sorted(root.glob("*/result.json")):
        request_path = result_path.with_name("request.json")
        if not request_path.is_file():
            continue
        request = load_json(request_path)
        result = load_json(result_path)
        xml_validation = nested(result, "validation", "xml") or {}
        action_metrics = (
            nested(result, "validation", "action_library_metrics") or {}
        )
        scale_condition = request.get("scale_condition") or result.get(
            "scale_condition"
        ) or {}
        stages = result.get("stages", [])
        latency = sum(
            float(stage.get("latency_s", 0.0) or 0.0)
            for stage in stages
            if isinstance(stage, Mapping)
        )
        if isinstance(result.get("api_call"), Mapping):
            latency += float(result["api_call"].get("latency_s", 0.0) or 0.0)
        variant_id = request.get("variant_id") or ""
        condition = f"{request['method']}:{request['model_key']}"
        if variant_id:
            condition += f":{variant_id}"
        row = {
            "condition_id": request["condition_id"],
            "experiment": request["experiment"],
            "method": request["method"],
            "model_key": request["model_key"],
            "condition": condition,
            "mission_id": request["mission_id"],
            "paraphrase_id": request["paraphrase_id"],
            "variant_id": variant_id,
            "action_node_count": scale_condition.get("action_node_count"),
            "distractor_count": scale_condition.get("distractor_count"),
            "repetition": request["repetition"],
            "status": result.get("status"),
            "scored": result.get("scored") is True,
            "expected_outcome": request["expected_outcome"],
            "decision_outcome": result.get("decision_outcome"),
            "correct_outcome": result.get("correct_outcome"),
            "automated_task_success": result.get("automated_task_success"),
            "primary_success": (
                result.get("correct_outcome")
                if request["experiment"] == "E4"
                and request["expected_outcome"] != "plan"
                else result.get("automated_task_success")
            ),
            "first_attempt_valid": result.get("first_attempt_valid"),
            "tree_match": result.get("tree_match"),
            "syntax_valid": xml_validation.get("syntax_valid"),
            "interface_valid_static": xml_validation.get("interface_valid_static"),
            "factory_load": xml_validation.get("factory_load", "not_applicable"),
            "required_node_recall": action_metrics.get("required_node_recall"),
            "distractor_use": action_metrics.get("distractor_use"),
            "distractor_use_count": action_metrics.get("distractor_use_count"),
            "distractor_nodes_used": ";".join(
                action_metrics.get("used_distractor_nodes", [])
            ),
            "invented_node_count": action_metrics.get("invented_node_count"),
            "invented_nodes": ";".join(action_metrics.get("invented_nodes", [])),
            "port_error_count": action_metrics.get("port_error_count"),
            "cost_usd": float(result.get("cost_usd", 0.0) or 0.0),
            "latency_s": round(latency, 3),
            "result_path": str(result_path.resolve()),
        }
        rows.append(row)
    return rows


def write_csv(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        if row["status"] in ("dry_run", "blocked"):
            continue
        groups[(row["experiment"], row["condition"])].append(row)
    summary: list[dict[str, Any]] = []
    for (experiment, condition), members in sorted(groups.items()):
        evaluable = [row for row in members if isinstance(row["primary_success"], bool)]
        correct = sum(row["primary_success"] is True for row in evaluable)
        valid_rows = [
            row for row in members if isinstance(row["first_attempt_valid"], bool)
        ]
        first_valid = sum(row["first_attempt_valid"] is True for row in valid_rows)
        low, high = wilson(correct, len(evaluable))
        recalls = [
            float(row["required_node_recall"])
            for row in members
            if isinstance(row["required_node_recall"], (int, float))
        ]
        distractor_rows = [
            row for row in members if isinstance(row["distractor_use"], bool)
        ]
        summary.append(
            {
                "experiment": experiment,
                "condition": condition,
                "n_artifacts": len(members),
                "n_evaluable": len(evaluable),
                "n_non_evaluable": len(members) - len(evaluable),
                "n_complete": sum(row["status"] == "complete" for row in members),
                "n_validation_failed": sum(
                    row["status"] == "validation_failed" for row in members
                ),
                "n_transport_error": sum(
                    row["status"] == "transport_error" for row in members
                ),
                "n_protocol_error": sum(
                    row["status"] == "protocol_error" for row in members
                ),
                "primary_successes": correct,
                "primary_success_rate": round(correct / len(evaluable), 6)
                if evaluable
                else None,
                "primary_success_rate_ci95_low": round(low, 6) if evaluable else None,
                "primary_success_rate_ci95_high": round(high, 6) if evaluable else None,
                "first_attempt_valid_n": len(valid_rows),
                "first_attempt_valid": first_valid,
                "first_attempt_valid_rate": round(first_valid / len(valid_rows), 6)
                if valid_rows
                else None,
                "mean_required_node_recall": round(
                    sum(recalls) / len(recalls), 6
                )
                if recalls
                else None,
                "distractor_use_n": len(distractor_rows),
                "distractor_use_rate": round(
                    sum(row["distractor_use"] is True for row in distractor_rows)
                    / len(distractor_rows),
                    6,
                )
                if distractor_rows
                else None,
                "invented_node_total": sum(
                    int(row["invented_node_count"] or 0) for row in members
                ),
                "port_error_total": sum(
                    int(row["port_error_count"] or 0) for row in members
                ),
                "factory_pass": sum(
                    row["factory_load"] == "pass" for row in members
                ),
                "factory_fail": sum(
                    row["factory_load"] == "fail" for row in members
                ),
                "factory_not_run": sum(
                    row["factory_load"] == "not_run" for row in members
                ),
                "cost_usd": round(sum(row["cost_usd"] for row in members), 8),
                "mean_latency_s": round(
                    sum(row["latency_s"] for row in members) / len(members), 3
                ),
            }
        )
    return summary


def paired_comparisons(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    values: dict[str, dict[tuple[Any, ...], bool]] = defaultdict(dict)
    for row in rows:
        if not isinstance(row["primary_success"], bool):
            continue
        key = (
            row["experiment"],
            row["mission_id"],
            row["paraphrase_id"],
            row["variant_id"],
            row["repetition"],
        )
        values[row["condition"]][key] = row["primary_success"]
    comparisons: list[dict[str, Any]] = []
    for first, second in itertools.combinations(sorted(values), 2):
        shared = sorted(set(values[first]) & set(values[second]))
        if not shared:
            continue
        first_only = sum(values[first][key] and not values[second][key] for key in shared)
        second_only = sum(values[second][key] and not values[first][key] for key in shared)
        comparisons.append(
            {
                "condition_a": first,
                "condition_b": second,
                "paired_n": len(shared),
                "a_correct_b_wrong": first_only,
                "a_wrong_b_correct": second_only,
                "exact_mcnemar_p_two_sided": round(
                    exact_mcnemar_p(first_only, second_only), 8
                ),
            }
        )
    return comparisons


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument("--output-dir", type=Path, default=EVALUATION / "results")
    parser.add_argument(
        "--include-unscored",
        action="store_true",
        help="Include pilot artifacts. Never use this flag for final thesis tables.",
    )
    args = parser.parse_args()

    discovered_rows = discover(args.input_dir)
    rows = (
        discovered_rows
        if args.include_unscored
        else [row for row in discovered_rows if row["scored"]]
    )
    if not rows:
        qualifier = "" if args.include_unscored else " scored"
        print(
            f"No{qualifier} evaluation artifacts found in {args.input_dir}.",
            file=sys.stderr,
        )
        return 2
    summaries = aggregate(rows)
    comparisons = paired_comparisons(rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "runs.csv", rows)
    write_csv(args.output_dir / "summary.csv", summaries)
    write_csv(args.output_dir / "paired_comparisons.csv", comparisons)
    write_json(
        args.output_dir / "summary.json",
        {
            "input_dir": str(args.input_dir.resolve()),
            "raw_outputs_are_never_repaired": True,
            "include_unscored": args.include_unscored,
            "artifacts_discovered": len(discovered_rows),
            "artifacts_analyzed": len(rows),
            "groups": summaries,
            "paired_comparisons": comparisons,
        },
    )
    print(
        f"Analyzed {len(rows)} artifacts "
        f"({len(discovered_rows)} discovered) into {args.output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
