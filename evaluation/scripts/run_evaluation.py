#!/usr/bin/env python3
"""
Thesis Evaluation Runner — E1 Core Method and Model Comparison.

Generates prompts and calls LLM APIs for all 9 missions × 3 paraphrases × 7 conditions
(M1+M3 across 3 models, plus M2 BTGenBot-2) = 189 primary outputs.

All general-purpose models route through OpenRouter (single API key).
BTGenBot-2 routes through a Colab endpoint.

Usage:
  # Dry-run: generate all prompts without calling APIs
  python3 evaluation/scripts/run_evaluation.py --dry-run

  # Run all 5 Husky missions (all methods, all models)
  OPENROUTER_API_KEY=*** python3 evaluation/scripts/run_evaluation.py --methods M1,M3 --models all --platform husky

  # Single mission test run
  python3 evaluation/scripts/run_evaluation.py --mission S1 --paraphrase S1-P1 --models gpt-5.6-sol --platform husky

  # M2 (BTGenBot-2) via Colab endpoint
  python3 evaluation/scripts/run_evaluation.py --methods M2 --colab-url https://xxxx.ngrok.io --platform husky

  # Full E1: M1+M3+M2
  OPENROUTER_API_KEY=*** python3 evaluation/scripts/run_evaluation.py \\
    --methods M1,M3,M2 --models all --colab-url https://xxxx.ngrok.io --platform husky

Environment:
  OPENROUTER_API_KEY  — OpenRouter API key (required for M1/M3)
  OLLAMA_HOST         — Optional: local Ollama for Gemma fallback (default http://localhost:11434)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# ---------- paths ----------
REPO = Path(__file__).resolve().parents[2]
EVAL_DIR = REPO / "evaluation"
PROTOCOL = EVAL_DIR / "protocol"
FIXTURES = EVAL_DIR / "fixtures" / "context"
PROMPTS_DIR = EVAL_DIR / "prompts"
RAW_OUTPUTS = EVAL_DIR / "raw_outputs"

# ---------- model config ----------
# All general-purpose models route through OpenRouter.
# Short keys are used on the CLI; OpenRouter IDs are used in API calls.
MODELS = {
    "gpt-5.6-sol": {
        "openrouter_id": "openai/gpt-5.6-sol",
        "label": "Large (GPT-5.6-Sol)",
        "temperature": 0.0,
        "max_tokens": 8192,
        "reasoning": {"effort": "xhigh"},
    },
    "gemini-3.5-flash": {
        "openrouter_id": "google/gemini-3.5-flash",
        "label": "Medium (Gemini 3.5 Flash)",
        "temperature": 0.0,
        "max_tokens": 8192,
    },
    "gemma-4-26b": {
        "openrouter_id": "google/gemma-4-26b-a4b-it",
        "label": "Small (Gemma 4 26B)",
        "temperature": 0.0,
        "max_tokens": 4096,
        "fallback_ollama": True,  # also runnable locally
    },
}

OPENROUTER_BASE = "https://openrouter.ai/api/v1"

# ---------- helpers ----------

def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)

def save_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)

def load_prompt(name: str) -> str:
    path = PROMPTS_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"Prompt template not found: {path}")
    return path.read_text(encoding="utf-8")

def serialize_context(context: dict) -> str:
    clean = {}
    for key, value in context.items():
        if key in ("evidence_quality", "implementation_note", "agreement_rule"):
            continue
        clean[key] = value
    return json.dumps(clean, indent=2, ensure_ascii=False)

# ---------- prompt builders ----------

def build_m1_prompt(mission: dict, paraphrase: dict, context: dict) -> tuple[str, str]:
    system = load_prompt("m1_system.txt")
    user_parts = [
        "## Mission",
        paraphrase["text"],
        "",
        "## Context",
        serialize_context(context),
        "",
        "## Instructions",
        "Generate the complete BehaviorTree.CPP XML for this mission.",
        "Use ONLY the action nodes listed in the system prompt. Output ONLY valid XML.",
    ]
    if "forbidden_assumptions" in mission.get("requirements", {}):
        user_parts.insert(1, "## Constraints")
        for c in mission["requirements"]["forbidden_assumptions"]:
            user_parts.insert(2, f"- {c}")
        user_parts.insert(3, "")
    return system, "\n".join(user_parts)

def build_m3_selection_prompt(mission: dict, paraphrase: dict, context: dict) -> tuple[str, str]:
    system = load_prompt("m3_selection_system.txt")
    user = "\n".join([
        "## Mission",
        paraphrase["text"],
        "",
        "## Context",
        serialize_context(context),
        "",
        "## Task",
        "Select the most appropriate tree from the catalogue, or determine that no suitable tree exists.",
    ])
    return system, user

def build_m3_payload_prompt(mission: dict, paraphrase: dict, context: dict, tree_id: str) -> tuple[str, str]:
    system = load_prompt("m3_payload_system.txt")
    user_parts = [
        "## Mission",
        paraphrase["text"],
        "",
        "## Selected Tree",
        tree_id,
        "",
        "## Context",
        serialize_context(context),
        "",
        "## Task",
        f"Generate a valid blackboard payload for {tree_id} that fulfills this mission.",
    ]
    if "constraints" in mission.get("requirements", {}):
        user_parts.insert(1, "## Constraints")
        for c in mission["requirements"]["constraints"]:
            user_parts.insert(2, f"- {c}")
    return system, "\n".join(user_parts)

# ---------- M2 (BTGenBot-2) action spec ----------

BTGENBOT2_ACTIONS = "\n".join([
    "MoveTo(pose: string, action_name: string)",
    "ParseWaypoints(raw_waypoints: string, waypoint_queue: string, waypoint_count: string)",
    "LoopString(queue: string, value: string, if_empty: string)",
    "LogTemperature(logfile_path: string)",
    "TakePhoto(image_topic: string, output_directory: string, filename_prefix: string, timeout_ms: int, filepath: string)",
    "DistanceTraveled(interval_m: double, odom_topic: string, odom_timeout_ms: int, distance_accumulated_m: double)",
    "KeepRunningUntilFailure()",
    "FindObjectLocation(object_query: string, object_pose: string)",
    "CheckBattery(min_percent: double, battery_percent: double, low_battery: bool)",
    "ReturnToHome(home_pose: string, action_name: string)",
    "Sequence",
    "Parallel(success_count: int, failure_count: int)",
    "Fallback",
    "Delay(delay_ms: int)",
])

def build_m2_prompt(mission: dict, paraphrase: dict) -> tuple[str, str]:
    task = paraphrase["text"]
    extras = []
    if "constraints" in mission.get("requirements", {}):
        extras.append("Constraints: " + "; ".join(mission["requirements"]["constraints"]))
    if "forbidden_assumptions" in mission.get("requirements", {}):
        extras.append("; ".join(mission["requirements"]["forbidden_assumptions"]))
    if extras:
        task = f"{task} {' '.join(extras)}"
    return task, BTGENBOT2_ACTIONS

# ---------- artifact saving ----------

def save_artifact(run_id: str, files: dict[str, Any]) -> Path:
    path = RAW_OUTPUTS / run_id
    path.mkdir(parents=True, exist_ok=True)
    for name, content in files.items():
        save_json(path / name, content)
    return path

# ---------- API callers ----------

def call_openrouter(model_cfg: dict, system: str, user: str, dry_run: bool) -> dict:
    """Call any model via OpenRouter (OpenAI-compatible API)."""
    api_key = os.environ.get("OPENROUTER_API_KEY", "")
    if not api_key and not dry_run:
        return {"error": "OPENROUTER_API_KEY not set", "raw_response": None}

    if dry_run:
        return {"dry_run": True, "model": model_cfg["openrouter_id"], "provider": "openrouter"}

    body: dict[str, Any] = {
        "model": model_cfg["openrouter_id"],
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": user},
        ],
        "temperature": model_cfg["temperature"],
        "max_tokens": model_cfg["max_tokens"],
    }
    if "reasoning" in model_cfg:
        body["reasoning"] = model_cfg["reasoning"]

    req = urllib.request.Request(
        f"{OPENROUTER_BASE}/chat/completions",
        data=json.dumps(body).encode(),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as resp:
            result = json.loads(resp.read())
        choice = result["choices"][0]
        content = choice["message"].get("content", "")
        # OpenRouter may include reasoning in a separate field
        if "reasoning" in choice["message"] and not content:
            content = choice["message"]["reasoning"]
        return {
            "raw_response": result,
            "content": content,
            "model": result.get("model", model_cfg["openrouter_id"]),
            "usage": result.get("usage", {}),
        }
    except Exception as exc:
        return {"error": str(exc), "raw_response": None}


def call_ollama(model_key: str, system: str, user: str, dry_run: bool) -> dict:
    """Call local Ollama (Gemma fallback)."""
    if dry_run:
        return {"dry_run": True, "model": model_key, "provider": "ollama"}

    try:
        result = subprocess.run(
            ["ollama", "run", model_key, "--format", "json"],
            input=json.dumps({
                "model": model_key,
                "messages": [
                    {"role": "system", "content": system},
                    {"role": "user", "content": user},
                ],
                "stream": False,
                "options": {"temperature": 0.0, "num_predict": 4096},
            }),
            capture_output=True, text=True, timeout=300,
        )
        if result.returncode != 0:
            return {"error": result.stderr, "raw_response": None}
        data = json.loads(result.stdout)
        return {
            "raw_response": data,
            "content": data.get("message", {}).get("content", ""),
            "model": model_key,
            "usage": data.get("eval_count"),
        }
    except subprocess.TimeoutExpired:
        return {"error": "Timeout after 300s", "raw_response": None}
    except Exception as exc:
        return {"error": str(exc), "raw_response": None}


def call_colab(colab_url: str, task: str, actions: str, dry_run: bool) -> dict:
    """Call BTGenBot-2 via Colab endpoint."""
    if dry_run:
        return {"dry_run": True, "model": "BTGenBot-2", "provider": "colab"}

    body = {"task": task, "actions": actions, "max_new_tokens": 2048}
    req = urllib.request.Request(
        f"{colab_url.rstrip('/')}/generate",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as resp:
            result = json.loads(resp.read())
        return {
            "raw_response": result,
            "content": result.get("xml", ""),
            "model": "BTGenBot-2",
        }
    except Exception as exc:
        return {"error": str(exc), "raw_response": None}


def call_llm(model_key: str, system: str, user: str, dry_run: bool) -> dict:
    """Route model key → API call. Prefers OpenRouter, falls back to Ollama for Gemma."""
    cfg = MODELS[model_key]
    start = time.monotonic()

    # Try OpenRouter first (all models available there)
    api_key = os.environ.get("OPENROUTER_API_KEY", "")
    if api_key or dry_run:
        result = call_openrouter(cfg, system, user, dry_run)
    elif cfg.get("fallback_ollama"):
        result = call_ollama(model_key, system, user, dry_run)
    else:
        result = {"error": "OPENROUTER_API_KEY not set and no local fallback available",
                  "raw_response": None}

    result["latency_s"] = round(time.monotonic() - start, 3)
    result["model_key"] = model_key
    return result

# ---------- main ----------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Thesis E1 Evaluation Runner")
    p.add_argument("--dry-run", action="store_true", help="Generate prompts without API calls")
    p.add_argument("--methods", default="M1,M3", help="Comma-separated: M1,M3,M2")
    p.add_argument("--models", default="all", help="Comma-separated model keys or 'all'")
    p.add_argument("--platform", default="all", help="husky, blueboat, or all")
    p.add_argument("--mission", help="Single mission ID (e.g., S1)")
    p.add_argument("--paraphrase", help="Single paraphrase ID (e.g., S1-P1)")
    p.add_argument("--colab-url", help="BTGenBot-2 Colab endpoint URL (for M2)")
    p.add_argument("--output-dir", default=str(RAW_OUTPUTS), help="Output directory")
    p.add_argument("--log", default=str(EVAL_DIR / "run_log.jsonl"), help="Run log path")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    missions_data = load_json(PROTOCOL / "core_missions.json")
    contexts = load_json(FIXTURES / "core_contexts.json")

    methods = [m.strip().upper() for m in args.methods.split(",")]
    if args.models == "all":
        model_keys = list(MODELS.keys())
    else:
        model_keys = [m.strip() for m in args.models.split(",")]

    unknown = [m for m in model_keys if m not in MODELS]
    if unknown:
        print(f"Unknown models: {unknown}", file=sys.stderr)
        print(f"Available: {list(MODELS.keys())}", file=sys.stderr)
        return 1

    missions = missions_data["missions"]
    if args.platform != "all":
        missions = [m for m in missions if m["platform"] == args.platform]
    if args.mission:
        missions = [m for m in missions if m["id"] == args.mission]

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    total = 0
    skipped = 0
    errors = 0

    for mission in missions:
        mid = mission["id"]
        platform = mission["platform"]

        if mission["support_status"] != "implemented_catalogue":
            print(f"SKIP {mid}: {mission['support_status']}")
            skipped += 3 * (len(methods) * len(model_keys) + (1 if "M2" in methods else 0))
            continue

        paraphrases = mission["paraphrases"]
        if args.paraphrase:
            paraphrases = [p for p in paraphrases if p["id"] == args.paraphrase]

        context = contexts["fixtures"].get(mid, {})

        for paraphrase in paraphrases:
            pid = paraphrase["id"]

            for model_key in model_keys:
                cfg = MODELS[model_key]

                # --- M1: Direct BT XML Generation ---
                if "M1" in methods:
                    total += 1
                    run_id = f"E1_M1_{model_key}_{pid}_{uuid.uuid4().hex[:8]}"
                    print(f"RUN M1 {pid} [{cfg['label']}] ", end="", flush=True)

                    system, user = build_m1_prompt(mission, paraphrase, context)
                    metadata = {
                        "run_id": run_id, "experiment": "E1", "method": "M1",
                        "mission_id": mid, "paraphrase_id": pid,
                        "model_key": model_key, "model_label": cfg["label"],
                        "openrouter_id": cfg["openrouter_id"],
                        "platform": platform, "complexity": mission["complexity"]["label"],
                        "temperature": cfg["temperature"], "max_tokens": cfg["max_tokens"],
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                    save_artifact(run_id, {
                        "system_prompt.json": {"content": system},
                        "user_prompt.json": {"content": user},
                        "metadata.json": metadata,
                    })

                    result = call_llm(model_key, system, user, args.dry_run)
                    result["run_id"] = run_id
                    result["method"] = "M1"
                    with log_path.open("a") as log:
                        log.write(json.dumps(result) + "\n")

                    status = "ERROR" if result.get("error") else ("DRY-RUN" if result.get("dry_run") else "OK")
                    extra = f" ({result.get('latency_s', '?')}s)" if status == "OK" else f": {result.get('error', '')}"
                    print(f"{status}{extra}")
                    if result.get("error"):
                        errors += 1

                    save_artifact(run_id, {"result.json": result})

                # --- M3: Catalogue Selection + Payload ---
                if "M3" in methods:
                    # Step 1: Tree selection
                    total += 1
                    sel_id = f"E1_M3_sel_{model_key}_{pid}_{uuid.uuid4().hex[:8]}"
                    print(f"RUN M3-select {pid} [{cfg['label']}] ", end="", flush=True)

                    system, user = build_m3_selection_prompt(mission, paraphrase, context)
                    metadata_sel = {
                        "run_id": sel_id, "experiment": "E1", "method": "M3", "stage": "selection",
                        "mission_id": mid, "paraphrase_id": pid,
                        "model_key": model_key, "model_label": cfg["label"],
                        "openrouter_id": cfg["openrouter_id"],
                        "platform": platform, "complexity": mission["complexity"]["label"],
                        "temperature": cfg["temperature"],
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                    save_artifact(sel_id, {
                        "system_prompt.json": {"content": system},
                        "user_prompt.json": {"content": user},
                        "metadata.json": metadata_sel,
                    })

                    sel_result = call_llm(model_key, system, user, args.dry_run)
                    sel_result["run_id"] = sel_id
                    sel_result["method"] = "M3_selection"
                    with log_path.open("a") as log:
                        log.write(json.dumps(sel_result) + "\n")

                    if sel_result.get("error"):
                        print(f"ERROR: {sel_result['error']}")
                        errors += 1
                        continue
                    elif sel_result.get("dry_run"):
                        print("DRY-RUN")
                    else:
                        print(f"OK ({sel_result.get('latency_s', '?')}s)")

                    save_artifact(sel_id, {"result.json": sel_result})

                    # Step 2: Parse selection → generate payload
                    try:
                        sel_data = json.loads(sel_result.get("content", "{}"))
                        tree_id = sel_data.get("tree_id")
                        action = sel_data.get("action")
                    except json.JSONDecodeError:
                        tree_id = None
                        action = "parse_error"

                    if action in ("refuse", "clarify", "parse_error") or tree_id is None:
                        combined = {
                            "run_id": sel_id, "method": "M3",
                            "mission_id": mid, "paraphrase_id": pid,
                            "model_key": model_key,
                            "selection": sel_data if isinstance(sel_data, dict) else {"raw": sel_result.get("content")},
                            "payload": None,
                            "status": action or "no_tree_selected",
                        }
                    else:
                        # Step 2: Payload generation
                        pyld_id = f"E1_M3_pyld_{model_key}_{pid}_{uuid.uuid4().hex[:8]}"
                        print(f"  M3-payload {pid} [{cfg['label']}] tree={tree_id} ", end="", flush=True)

                        system_p, user_p = build_m3_payload_prompt(mission, paraphrase, context, tree_id)
                        metadata_pyld = {
                            "run_id": pyld_id, "experiment": "E1", "method": "M3", "stage": "payload",
                            "mission_id": mid, "paraphrase_id": pid,
                            "model_key": model_key, "model_label": cfg["label"],
                            "openrouter_id": cfg["openrouter_id"],
                            "selected_tree": tree_id, "selection_run_id": sel_id,
                            "temperature": cfg["temperature"],
                            "timestamp": datetime.now(timezone.utc).isoformat(),
                        }
                        save_artifact(pyld_id, {
                            "system_prompt.json": {"content": system_p},
                            "user_prompt.json": {"content": user_p},
                            "metadata.json": metadata_pyld,
                        })

                        pyld_result = call_llm(model_key, system_p, user_p, args.dry_run)
                        pyld_result["run_id"] = pyld_id
                        pyld_result["method"] = "M3_payload"
                        pyld_result["selected_tree"] = tree_id
                        with log_path.open("a") as log:
                            log.write(json.dumps(pyld_result) + "\n")

                        if pyld_result.get("error"):
                            print(f"ERROR: {pyld_result['error']}")
                            errors += 1
                        elif pyld_result.get("dry_run"):
                            print("DRY-RUN")
                        else:
                            print(f"OK ({pyld_result.get('latency_s', '?')}s)")

                        save_artifact(pyld_id, {"result.json": pyld_result})

                        combined = {
                            "run_id": sel_id, "method": "M3",
                            "mission_id": mid, "paraphrase_id": pid,
                            "model_key": model_key,
                            "selection": sel_data if isinstance(sel_data, dict) else {"raw": sel_result.get("content")},
                            "payload": pyld_result.get("content"),
                            "status": "ok",
                        }

                    save_artifact(sel_id, {"m3_combined.json": combined})

            # --- M2: BTGenBot-2 via Colab ---
            if "M2" in methods:
                if not args.colab_url:
                    print(f"SKIP M2 {pid}: --colab-url required")
                    skipped += 1
                else:
                    total += 1
                    run_id = f"E1_M2_btgenbot2_{pid}_{uuid.uuid4().hex[:8]}"
                    print(f"RUN M2 {pid} [BTGenBot-2] ", end="", flush=True)

                    task, actions = build_m2_prompt(mission, paraphrase)
                    metadata = {
                        "run_id": run_id, "experiment": "E1", "method": "M2",
                        "mission_id": mid, "paraphrase_id": pid,
                        "model_key": "BTGenBot-2", "model_label": "Specialist (fine-tuned)",
                        "platform": platform, "complexity": mission["complexity"]["label"],
                        "colab_url": args.colab_url,
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                    save_artifact(run_id, {
                        "btgenbot2_prompt.json": {"task": task, "actions": actions},
                        "metadata.json": metadata,
                    })

                    result = call_colab(args.colab_url, task, actions, args.dry_run)
                    result["run_id"] = run_id
                    result["method"] = "M2"
                    with log_path.open("a") as log:
                        log.write(json.dumps(result) + "\n")

                    status = "ERROR" if result.get("error") else ("DRY-RUN" if result.get("dry_run") else "OK")
                    extra = f" ({result.get('latency_s', '?')}s)" if status == "OK" else f": {result.get('error', '')}"
                    print(f"{status}{extra}")
                    if result.get("error"):
                        errors += 1

                    save_artifact(run_id, {"result.json": result})

    # Summary
    print(f"\n{'='*60}")
    print(f"Total runs: {total} | Skipped (blocked): {skipped} | Errors: {errors}")
    print(f"Log: {log_path}")

    if args.dry_run:
        print("\nDry run complete. Prompts saved to raw_outputs/.")
        print("Run with OPENROUTER_API_KEY set to execute against LLMs.")

    return 0 if errors == 0 else 1

if __name__ == "__main__":
    raise SystemExit(main())
