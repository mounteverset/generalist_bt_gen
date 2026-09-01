#!/usr/bin/env python3
"""Create a reviewer packet with method and model labels removed."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from evaluation_core import EVALUATION, PROTOCOL, load_json, write_json


def raw_artifact(result: dict[str, Any]) -> Any:
    if isinstance(result.get("api_call"), dict):
        return result["api_call"].get("content")
    stages = result.get("stages", [])
    return [
        {
            "stage": stage.get("stage"),
            "content": stage.get("content"),
            "payload": stage.get("payload"),
            "selection": stage.get("selection"),
            "outcome": stage.get("outcome"),
            "message": stage.get("message"),
            "clarification_question": stage.get("clarification_question"),
            "errors": stage.get("errors"),
        }
        for stage in stages
        if any(
            stage.get(field) is not None
            for field in (
                "content",
                "payload",
                "selection",
                "outcome",
                "message",
                "clarification_question",
                "errors",
            )
        )
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument(
        "--packet", type=Path, default=EVALUATION / "results" / "blind_review.jsonl"
    )
    parser.add_argument(
        "--key", type=Path, default=EVALUATION / "results" / "blind_review_key.json"
    )
    parser.add_argument("--salt", default="generalist-bt-thesis-review-v1")
    parser.add_argument(
        "--include-unscored",
        action="store_true",
        help="Include pilot artifacts. Never use this flag for final thesis review.",
    )
    args = parser.parse_args()

    rubric = load_json(PROTOCOL / "scoring_rubric.json")
    packet: list[dict[str, Any]] = []
    key: dict[str, Any] = {}
    for result_path in sorted(args.input_dir.glob("*/result.json")):
        request_path = result_path.with_name("request.json")
        if not request_path.is_file():
            continue
        request = load_json(request_path)
        result = load_json(result_path)
        if not args.include_unscored and result.get("scored") is not True:
            continue
        if result.get("status") in {
            "dry_run",
            "blocked",
            "transport_error",
            "protocol_error",
        }:
            continue
        review_id = "R-" + hashlib.sha256(
            f"{args.salt}|{request['condition_id']}".encode("utf-8")
        ).hexdigest()[:12]
        packet.append(
            {
                "review_id": review_id,
                "mission_id": request["mission_id"],
                "mission_text": request["paraphrase"]["text"],
                "context": request["context"],
                "expected_outcome": request["expected_outcome"],
                "artifact": raw_artifact(result),
                "rubric": rubric["common_elements"],
            }
        )
        key[review_id] = {
            "condition_id": request["condition_id"],
            "method": request["method"],
            "model_key": request["model_key"],
            "result_path": str(result_path.resolve()),
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
    write_json(args.key, key)
    print(f"Wrote {len(packet)} reviewable blinded artifacts to {args.packet}")
    print(f"Keep the separate key private until scoring is complete: {args.key}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
