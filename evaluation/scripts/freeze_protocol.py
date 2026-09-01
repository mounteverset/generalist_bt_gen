#!/usr/bin/env python3
"""Freeze evaluation inputs that are otherwise duplicated across prompts and scripts."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

import yaml


REPO = Path(__file__).resolve().parents[2]
PROTOCOL = REPO / "evaluation" / "protocol"
TREE_METADATA = REPO / "config" / "tree_metadata.yaml"
SYSTEM_DESCRIPTION = REPO / "config" / "system_description.yaml"
NODE_MANIFEST = PROTOCOL / "bt_node_manifest.json"
CORE_MISSIONS = PROTOCOL / "core_missions.json"
OUTPUT = PROTOCOL / "runtime_contract.json"
IMPLEMENTATION_FILES = {
    "mission_reasoner_sha256": REPO
    / "src"
    / "mission_reasoner"
    / "mission_reasoner"
    / "reasoner.py",
    "payload_validation_sha256": REPO
    / "src"
    / "llm_interface"
    / "llm_interface"
    / "payload_validation.py",
    "plan_safety_validation_sha256": REPO
    / "src"
    / "plan_reviewer"
    / "plan_reviewer"
    / "safety_validation.py",
    "evaluation_core_sha256": REPO
    / "evaluation"
    / "scripts"
    / "evaluation_core.py",
    "evaluation_runner_sha256": REPO
    / "evaluation"
    / "scripts"
    / "run_evaluation.py",
    "btgenbot2_server_sha256": REPO
    / "evaluation"
    / "colab"
    / "btgenbot2_server.py",
    "factory_helper_source_sha256": REPO
    / "src"
    / "bt_executor"
    / "src"
    / "bt_factory_check.cpp",
}


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def repository_commit() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def build_snapshot(commit: str) -> dict[str, Any]:
    hashes = {
        "tree_metadata_sha256": sha256(TREE_METADATA),
        "system_description_sha256": sha256(SYSTEM_DESCRIPTION),
        "bt_node_manifest_sha256": sha256(NODE_MANIFEST),
    }
    return {
        "contract_version": "1.0.0",
        "repository_commit": commit,
        "source_hashes": hashes,
        "implementation_hashes": {
            key: sha256(path) for key, path in IMPLEMENTATION_FILES.items()
        },
        "tree_catalogue": load_yaml(TREE_METADATA).get("trees", []),
        "system_description": load_yaml(SYSTEM_DESCRIPTION),
        "bt_node_manifest": load_json(NODE_MANIFEST),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--commit",
        help="Commit represented by this snapshot. Defaults to the current HEAD.",
    )
    parser.add_argument(
        "--update-dataset-snapshot",
        action="store_true",
        help="Also update source_snapshot in core_missions.json.",
    )
    args = parser.parse_args()

    commit = args.commit or repository_commit()
    snapshot = build_snapshot(commit)
    write_json(OUTPUT, snapshot)

    if args.update_dataset_snapshot:
        core = load_json(CORE_MISSIONS)
        core["source_snapshot"] = {
            "repository_commit": commit,
            **snapshot["source_hashes"],
        }
        write_json(CORE_MISSIONS, core)

    print(f"Wrote {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
