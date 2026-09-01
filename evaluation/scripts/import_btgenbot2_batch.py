#!/usr/bin/env python3
"""Import raw BTGenBot-2 Colab outputs into existing M2 dry-run artifacts."""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from evaluation_core import EVALUATION, PROTOCOL, load_json, write_json
from run_evaluation import btgenbot_revision_errors, direct_result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("batch_output", type=Path)
    parser.add_argument("--artifact-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument("--factory-helper", type=Path)
    parser.add_argument("--scored", action="store_true")
    args = parser.parse_args()

    runtime = load_json(PROTOCOL / "runtime_contract.json")
    model_conditions = load_json(PROTOCOL / "model_conditions.json")
    outputs = {}
    with args.batch_output.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                item = json.loads(line)
                outputs[str(item["request_id"])] = item

    imported = 0
    for request_path in sorted(args.artifact_dir.glob("*/request.json")):
        request = load_json(request_path)
        condition = request.get("condition_id")
        if request.get("method") != "M2" or condition not in outputs:
            continue
        output = outputs[condition]
        if "raw_text" not in output:
            raise ValueError(f"{condition}: batch output does not preserve raw_text")
        metadata = output.get("metadata", {})
        if args.scored and (
            revision_errors := btgenbot_revision_errors(metadata, model_conditions)
        ):
            raise ValueError(f"{condition}: {'; '.join(revision_errors)}")
        result_path = request_path.with_name("result.json")
        if result_path.exists():
            write_json(request_path.with_name("dry_run_result.json"), load_json(result_path))
        evaluation = direct_result(
            "M2",
            output["raw_text"],
            request["mission"],
            request["expected_outcome"],
            runtime,
            args.factory_helper,
            request["context"],
            decision_envelope=request["experiment"] == "E4",
        )
        evaluation.update(
            {
                "condition_id": condition,
                "status": "complete"
                if evaluation.get("correct_outcome")
                else "validation_failed",
                "scored": args.scored,
                "finished_at": datetime.now(timezone.utc).isoformat(),
                "cost_usd": 0.0,
                "api_call": {
                    "content": output["raw_text"],
                    "raw_response": output,
                    "raw_output_preserved": True,
                    "generation_metadata": metadata,
                    "provider": "colab_batch",
                    "cost_usd": 0.0,
                },
            }
        )
        write_json(result_path, evaluation)
        imported += 1
    print(f"Imported {imported} BTGenBot-2 outputs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
