#!/usr/bin/env python3
"""Export blinded final artifacts and a validated-response template."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from evaluation_core import EVALUATION, PROTOCOL, load_json, write_json


def final_artifact(result: dict[str, Any]) -> Any:
    if isinstance(result.get("api_call"), dict):
        return result["api_call"].get("content")
    if result.get("decision_outcome") != "plan":
        reasoner = next(
            (
                stage
                for stage in result.get("stages", [])
                if stage.get("stage") == "mission_reasoner"
            ),
            {},
        )
        return {
            "outcome": reasoner.get("outcome", result.get("decision_outcome")),
            "message": reasoner.get("message"),
            "clarification_question": reasoner.get("clarification_question"),
            "reasoning": reasoner.get("reasoning"),
        }
    return {
        "selected_tree": result.get("selected_tree"),
        "payload": result.get("final_payload"),
    }


def applicable_elements(request: dict[str, Any]) -> list[str]:
    if request["expected_outcome"] != "plan":
        return ["failure_quality"]
    declared = request.get("mission", {}).get("semantic_rubric_elements", [])
    if declared:
        return list(declared)
    return [
        "intent",
        "platform",
        "tree_or_structure",
        "spatial_plan",
        "constraints",
        "completeness",
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument(
        "--packet", type=Path, default=EVALUATION / "results" / "blind_review.jsonl"
    )
    parser.add_argument(
        "--responses",
        type=Path,
        default=EVALUATION / "results" / "blind_review_responses.jsonl",
    )
    parser.add_argument(
        "--key", type=Path, default=EVALUATION / "results" / "blind_review_key.json"
    )
    parser.add_argument("--salt", default="generalist-bt-thesis-review-v2")
    parser.add_argument(
        "--include-unscored",
        action="store_true",
        help="Include pilot artifacts. Never use this flag for final thesis review.",
    )
    args = parser.parse_args()

    rubric = load_json(PROTOCOL / "scoring_rubric.json")
    packet: list[dict[str, Any]] = []
    responses: list[dict[str, Any]] = []
    key: dict[str, Any] = {}
    for result_path in sorted(args.input_dir.glob("*/result.json")):
        request_path = result_path.with_name("request.json")
        if not request_path.is_file():
            continue
        request = load_json(request_path)
        result = load_json(result_path)
        if not args.include_unscored and result.get("scored") is not True:
            continue
        if result.get("status") in {"dry_run", "blocked", "transport_error", "protocol_error"}:
            continue
        review_id = "R-" + hashlib.sha256(
            f"{args.salt}|{request['condition_id']}".encode("utf-8")
        ).hexdigest()[:12]
        complexity = request.get("mission", {}).get("complexity", {}).get("label")
        second_review = (
            request["experiment"] == "E4"
            or complexity == "complex"
            or int(hashlib.sha256(review_id.encode("utf-8")).hexdigest(), 16) % 5 == 0
        )
        elements = applicable_elements(request)
        packet.append(
            {
                "review_id": review_id,
                "mission_id": request["mission_id"],
                "mission_text": request["paraphrase"]["text"],
                "context": request["context"],
                "expected_outcome": request["expected_outcome"],
                "evaluation_reference": request.get("evaluation_reference"),
                "artifact": final_artifact(result),
                "applicable_elements": elements,
                "rubric": {
                    name: rubric["common_elements"][name] for name in elements
                },
                "scale": rubric["common_semantic_scale"],
                "second_review_required": second_review,
            }
        )
        responses.append(
            {
                "review_id": review_id,
                "reviewers": [],
                "adjudicated_scores": None,
                "correction": {
                    "status": None if request["experiment"] == "E1" else "not_applicable",
                    "manual_correction_count": None,
                    "manual_correction_types": [],
                    "manual_time_s": None,
                    "corrected_artifact_path": None,
                    "post_repair_validation_path": None,
                    "post_repair_deterministic_pass": None,
                    "post_repair_scores": None,
                    "notes": "",
                },
            }
        )
        key[review_id] = {
            "condition_id": request["condition_id"],
            "method": request["method"],
            "model_key": request["model_key"],
            "result_path": str(result_path.resolve()),
            "applicable_elements": elements,
            "second_review_required": second_review,
        }
    if not packet:
        qualifier = "" if args.include_unscored else " scored"
        print(
            f"No reviewable{qualifier} artifacts found in {args.input_dir}.",
            file=sys.stderr,
        )
        return 2
    args.packet.parent.mkdir(parents=True, exist_ok=True)
    args.packet.write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in packet),
        encoding="utf-8",
    )
    args.responses.write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in responses),
        encoding="utf-8",
    )
    write_json(args.key, key)
    print(f"Wrote {len(packet)} reviewable blinded artifacts to {args.packet}")
    print(f"Wrote the response template to {args.responses}")
    print(f"Keep the separate key private until scoring is complete: {args.key}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
