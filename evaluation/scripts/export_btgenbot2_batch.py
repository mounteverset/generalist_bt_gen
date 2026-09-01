#!/usr/bin/env python3
"""Export M2 requests from a dry run into the Colab JSONL batch format."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from evaluation_core import EVALUATION, load_json


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=EVALUATION / "raw_outputs")
    parser.add_argument(
        "--output",
        type=Path,
        default=EVALUATION / "results" / "btgenbot2_requests.jsonl",
    )
    args = parser.parse_args()

    requests = []
    for result_path in sorted(args.input_dir.glob("*/result.json")):
        request_path = result_path.with_name("request.json")
        if not request_path.is_file():
            continue
        request = load_json(request_path)
        result = load_json(result_path)
        if request.get("method") != "M2":
            continue
        body = result.get("api_call", {}).get("request", {}).get("body")
        if not isinstance(body, dict):
            continue
        requests.append(
            {
                "request_id": request["condition_id"],
                "task": body["task"],
                "actions": body["actions"],
                "max_new_tokens": body.get("max_new_tokens", 2048),
                "seed": body.get("seed", request["seed"]),
            }
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in requests),
        encoding="utf-8",
    )
    print(f"Wrote {len(requests)} frozen requests to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
