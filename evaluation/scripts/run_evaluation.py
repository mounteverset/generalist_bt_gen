#!/usr/bin/env python3
"""Reproducible E1-E4 thesis evaluation runner.

The runner preserves every raw first output. Transport retries and M3 semantic
refinement are logged separately so first-attempt validity is never inflated.
"""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import mimetypes
import os
import platform as host_platform
import random
import socket
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional

from evaluation_core import (
    EVALUATION,
    FIXTURES,
    PROTOCOL,
    REPO,
    analyze_xml_node_usage,
    apply_context_operations,
    build_m1_prompt,
    build_m2_prompt,
    build_m3_payload_prompt,
    build_m3_requirements_prompt,
    build_m3_selection_prompt,
    canonical_json,
    compare_payload_to_reference,
    condition_id,
    context_variant,
    extract_xml_parameters,
    load_json,
    materialize_m1_action_library,
    multimodal_preflight,
    parse_decision,
    parse_json_object,
    parse_payload_response,
    review_payload,
    run_factory_check,
    score_xml_against_mission,
    sha256,
    tree_by_id,
    validate_xml_interface,
    write_json,
)

sys.path.insert(0, str(REPO / "src" / "mission_reasoner"))
sys.path.insert(0, str(REPO / "src" / "llm_interface"))
from mission_reasoner.reasoner import ACCEPT, CLARIFY, REFUSE, MissionReasoner
from llm_interface.payload_validation import generated_payload_errors


OPENROUTER_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_OUTPUT = EVALUATION / "raw_outputs"
DEFAULT_LOG = EVALUATION / "run_log.jsonl"
TRANSIENT_HTTP_CODES = {408, 409, 429, 502, 503, 504}
CONDITION_HASH_KEYS = (
    "runtime_contract_sha256",
    "model_conditions_sha256",
    "core_dataset_sha256",
    "context_fixtures_sha256",
    "context_variants_sha256",
    "choice_space_variants_sha256",
    "m1_action_distractors_sha256",
    "safety_cases_sha256",
    "scoring_rubric_sha256",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def protocol_hashes() -> dict[str, str]:
    return {
        "runtime_contract_sha256": sha256(PROTOCOL / "runtime_contract.json"),
        "model_conditions_sha256": sha256(PROTOCOL / "model_conditions.json"),
        "core_dataset_sha256": sha256(PROTOCOL / "core_missions.json"),
        "context_fixtures_sha256": sha256(FIXTURES / "core_contexts.json"),
        "context_variants_sha256": sha256(PROTOCOL / "context_variants.json"),
        "choice_space_variants_sha256": sha256(PROTOCOL / "choice_space_variants.json"),
        "m1_action_distractors_sha256": sha256(PROTOCOL / "m1_action_distractors.json"),
        "safety_cases_sha256": sha256(PROTOCOL / "safety_cases.json"),
        "scoring_rubric_sha256": sha256(PROTOCOL / "scoring_rubric.json"),
    }


def normalize_content(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return "".join(
            str(item.get("text", "")) if isinstance(item, dict) else str(item)
            for item in value
        )
    return "" if value is None else str(value)


def redacted_headers(headers: Mapping[str, str]) -> dict[str, str]:
    return {
        key: ("<redacted>" if key.lower() == "authorization" else value)
        for key, value in headers.items()
    }


def response_cost(response: Mapping[str, Any]) -> float:
    candidates = [
        response.get("cost"),
        (response.get("usage") or {}).get("cost")
        if isinstance(response.get("usage"), Mapping)
        else None,
    ]
    for value in candidates:
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return 0.0


def retry_delay(headers: Mapping[str, str], attempt: int) -> float:
    raw = headers.get("Retry-After") or headers.get("retry-after")
    try:
        return min(60.0, max(0.0, float(raw)))
    except (TypeError, ValueError):
        return min(30.0, 2.0 ** attempt + random.random())


def http_json(
    url: str,
    body: Mapping[str, Any],
    headers: Mapping[str, str],
    *,
    timeout_s: int,
    max_transport_retries: int,
    dry_run: bool,
) -> dict[str, Any]:
    request_record = {
        "url": url,
        "headers": redacted_headers(headers),
        "body": body,
        "timeout_s": timeout_s,
    }
    if dry_run:
        return {
            "dry_run": True,
            "request": request_record,
            "attempts": [],
            "content": "",
            "raw_response": None,
            "latency_s": 0.0,
            "cost_usd": 0.0,
        }

    attempts: list[dict[str, Any]] = []
    started = time.monotonic()
    for attempt_index in range(max_transport_retries + 1):
        attempt_started = time.monotonic()
        request = urllib.request.Request(
            url,
            data=json.dumps(body).encode("utf-8"),
            headers=dict(headers),
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout_s) as response:
                raw_body = response.read().decode("utf-8")
                parsed = json.loads(raw_body)
                attempts.append(
                    {
                        "attempt": attempt_index + 1,
                        "http_status": response.status,
                        "latency_s": round(time.monotonic() - attempt_started, 3),
                        "response_headers": dict(response.headers.items()),
                    }
                )
                return {
                    "request": request_record,
                    "attempts": attempts,
                    "raw_response": parsed,
                    "latency_s": round(time.monotonic() - started, 3),
                    "cost_usd": response_cost(parsed),
                }
        except urllib.error.HTTPError as exc:
            response_headers = dict(exc.headers.items()) if exc.headers else {}
            raw_error = exc.read().decode("utf-8", errors="replace")
            retryable = exc.code in TRANSIENT_HTTP_CODES
            delay = retry_delay(response_headers, attempt_index)
            attempts.append(
                {
                    "attempt": attempt_index + 1,
                    "http_status": exc.code,
                    "error_body": raw_error,
                    "retryable": retryable,
                    "retry_delay_s": delay if retryable else 0.0,
                    "latency_s": round(time.monotonic() - attempt_started, 3),
                    "response_headers": response_headers,
                }
            )
            if not retryable or attempt_index >= max_transport_retries:
                break
            time.sleep(delay)
        except (urllib.error.URLError, TimeoutError, socket.timeout) as exc:
            delay = retry_delay({}, attempt_index)
            attempts.append(
                {
                    "attempt": attempt_index + 1,
                    "error": str(exc),
                    "retryable": True,
                    "retry_delay_s": delay,
                    "latency_s": round(time.monotonic() - attempt_started, 3),
                }
            )
            if attempt_index >= max_transport_retries:
                break
            time.sleep(delay)
        except (json.JSONDecodeError, ValueError) as exc:
            attempts.append(
                {
                    "attempt": attempt_index + 1,
                    "error": f"invalid JSON response: {exc}",
                    "retryable": False,
                    "latency_s": round(time.monotonic() - attempt_started, 3),
                }
            )
            break
    return {
        "request": request_record,
        "attempts": attempts,
        "raw_response": None,
        "error": attempts[-1].get("error")
        or f"HTTP {attempts[-1].get('http_status')}",
        "content": "",
        "latency_s": round(time.monotonic() - started, 3),
        "cost_usd": 0.0,
    }


def image_message_content(user_text: str, image_paths: Iterable[Path]) -> list[dict[str, Any]]:
    content: list[dict[str, Any]] = [{"type": "text", "text": user_text}]
    for path in image_paths:
        media_type = mimetypes.guess_type(path.name)[0] or "image/png"
        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
        content.append(
            {
                "type": "image_url",
                "image_url": {"url": f"data:{media_type};base64,{encoded}"},
            }
        )
    return content


def call_openrouter(
    model: Mapping[str, Any],
    system: str,
    user: str,
    *,
    image_paths: Iterable[Path],
    seed: int,
    dry_run: bool,
    timeout_s: int,
    max_transport_retries: int,
) -> dict[str, Any]:
    api_key = os.environ.get("OPENROUTER_API_KEY", "")
    if not api_key and not dry_run:
        return {"error": "OPENROUTER_API_KEY is not set", "cost_usd": 0.0}
    paths = list(image_paths)
    user_content: Any = image_message_content(user, paths) if paths else user
    body: dict[str, Any] = {
        "model": model["model_id"],
        "messages": [
            {"role": "system", "content": system},
            {"role": "user", "content": user_content},
        ],
        "max_tokens": model["max_tokens"],
        "seed": seed,
    }
    if model.get("temperature") is not None:
        body["temperature"] = model["temperature"]
    if model.get("reasoning"):
        body["reasoning"] = model["reasoning"]
    providers = model.get("provider_only", [])
    if providers:
        body["provider"] = {
            "only": providers,
            "allow_fallbacks": False,
            "require_parameters": True,
            "data_collection": "deny",
        }
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
        "X-OpenRouter-Cache": "false",
        "X-OpenRouter-Metadata": "enabled",
        "HTTP-Referer": "https://github.com/generalist-bt-gen/evaluation",
        "X-Title": "Generalist BT thesis evaluation",
    }
    result = http_json(
        OPENROUTER_URL,
        body,
        headers,
        timeout_s=timeout_s,
        max_transport_retries=max_transport_retries,
        dry_run=dry_run,
    )
    raw = result.get("raw_response")
    if isinstance(raw, Mapping):
        choices = raw.get("choices") or []
        if choices:
            message = choices[0].get("message", {})
            result["content"] = normalize_content(message.get("content"))
            result["finish_reason"] = choices[0].get("finish_reason")
        result["returned_model"] = raw.get("model")
        result["returned_provider"] = raw.get("provider")
        result["usage"] = raw.get("usage", {})
    result["model_id"] = model["model_id"]
    result["requested_provider_only"] = providers
    returned_provider = result.get("returned_provider")
    result["provider_verified"] = (
        str(returned_provider).lower().replace(" ", "-") in providers
        if returned_provider and providers
        else None
    )
    result["seed"] = seed
    result["cache_disabled_requested"] = True
    return result


def call_colab(
    colab_url: str,
    task: str,
    actions: str,
    *,
    seed: int,
    dry_run: bool,
    timeout_s: int,
    max_transport_retries: int,
) -> dict[str, Any]:
    token = os.environ.get("COLAB_EVAL_TOKEN", "")
    if not token and not dry_run:
        return {"error": "COLAB_EVAL_TOKEN is not set", "cost_usd": 0.0}
    body = {
        "task": task,
        "actions": actions,
        "max_new_tokens": 2048,
        "seed": seed,
    }
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
    }
    result = http_json(
        f"{colab_url.rstrip('/')}/generate",
        body,
        headers,
        timeout_s=timeout_s,
        max_transport_retries=max_transport_retries,
        dry_run=dry_run,
    )
    raw = result.get("raw_response")
    if isinstance(raw, Mapping):
        result["content"] = normalize_content(
            raw.get("raw_text", raw.get("xml", ""))
        )
        result["extracted_xml"] = raw.get("extracted_xml", raw.get("xml"))
        result["generation_metadata"] = raw.get("metadata", {})
        result["raw_output_preserved"] = "raw_text" in raw
    result["model_id"] = "BTGenBot-2"
    result["seed"] = seed
    return result


def method_outcome(action: Optional[str]) -> str:
    outcomes = {
        "plan": "plan",
        "select": "plan",
        "clarify": "clarification",
        "refuse": "refusal",
    }
    return outcomes.get(str(action), "invalid")


def btgenbot_revision_errors(
    metadata: Mapping[str, Any], model_conditions: Mapping[str, Any]
) -> list[str]:
    """Verify that a Colab result came from the model revisions in the protocol."""
    local_model = model_conditions["local_model"]
    expected = {
        "base_model": local_model.get("base_model"),
        "base_revision": local_model.get("base_revision"),
        "adapter_model": local_model.get("adapter"),
        "adapter_revision": local_model.get("adapter_revision"),
    }
    errors = [
        f"{field} does not match the frozen protocol"
        for field, expected_value in expected.items()
        if not expected_value or metadata.get(field) != expected_value
    ]
    if metadata.get("revision_freeze_status") != "frozen":
        errors.append("model revisions were not frozen by the Colab runtime")
    return errors


def direct_result(
    method: str,
    raw_output: str,
    mission: Mapping[str, Any],
    expected_outcome: str,
    runtime: Mapping[str, Any],
    factory_helper: Optional[Path],
    context: Mapping[str, Any],
    decision_envelope: bool = False,
) -> dict[str, Any]:
    decision = None
    decision_json_syntax_valid: Optional[bool] = None
    if decision_envelope:
        decision, errors = parse_decision(raw_output)
        decision_json_syntax_valid = decision is not None
        outcome = method_outcome(decision.get("action") if decision else None)
        if outcome != "plan":
            task_success = not errors and outcome == expected_outcome
            return {
                "decision": decision,
                "decision_outcome": outcome,
                "expected_outcome": expected_outcome,
                "correct_outcome": outcome == expected_outcome,
                "automated_task_success": task_success,
                "first_attempt_valid": not errors,
                "first_attempt_task_success": task_success,
                "validation": {
                    "decision_json_syntax_valid": decision_json_syntax_valid,
                    "decision_errors": errors,
                },
            }
        raw_xml = decision.get("xml", "") if decision else ""
        if not isinstance(raw_xml, str) or not raw_xml:
            errors.append("plan decision has no XML artifact")
            raw_xml = ""
    else:
        errors = []
        raw_xml = raw_output

    if errors:
        return {
            "decision": decision,
            "decision_outcome": "invalid_plan",
            "expected_outcome": expected_outcome,
            "correct_outcome": False,
            "automated_task_success": False,
            "first_attempt_valid": False,
            "first_attempt_task_success": False,
            "validation": {
                "decision_json_syntax_valid": decision_json_syntax_valid,
                "decision_errors": errors,
            },
        }
    xml_validation = validate_xml_interface(
        raw_xml, runtime["bt_node_manifest"], initial_blackboard=()
    )
    factory = run_factory_check(raw_xml, factory_helper)
    xml_validation.update(factory)
    semantic = score_xml_against_mission(raw_xml, mission)
    scale_condition = runtime.get("evaluation_condition", {})
    node_usage = analyze_xml_node_usage(
        raw_xml,
        runtime["bt_node_manifest"],
        scale_condition.get("distractor_names", []),
    )
    port_errors = [
        error
        for error in xml_validation.get("errors", [])
        if "undeclared port" in error or "missing required port" in error
    ]
    action_library_metrics = {
        **node_usage,
        "required_node_recall": semantic.get("required_node_recall"),
        "distractor_use": bool(node_usage["used_distractor_nodes"]),
        "distractor_use_count": len(node_usage["used_distractor_nodes"]),
        "invented_node_count": len(node_usage["invented_nodes"]),
        "port_errors": port_errors,
        "port_error_count": len(port_errors),
    }
    extracted_parameters = extract_xml_parameters(raw_xml)
    spatial_review = review_payload(extracted_parameters, context, mission)
    first_valid = bool(
        xml_validation["syntax_valid"]
        and xml_validation["interface_valid_static"]
        and factory.get("factory_load") == "pass"
    )
    task_success = bool(
        first_valid
        and expected_outcome == "plan"
        and semantic["required_nodes_present"]
        and semantic["concrete_values_present"]
        and spatial_review["approved"]
        and not action_library_metrics["distractor_use"]
    )
    return {
        "decision": decision,
        "decision_outcome": "plan" if first_valid else "invalid_plan",
        "expected_outcome": expected_outcome,
        "correct_outcome": first_valid and expected_outcome == "plan",
        "automated_task_success": task_success,
        "first_attempt_valid": first_valid,
        "first_attempt_task_success": task_success,
        "validation": {
            "decision_json_syntax_valid": decision_json_syntax_valid,
            "xml": xml_validation,
            "automated_semantics": semantic,
            "action_library_metrics": action_library_metrics,
            "extracted_parameters": extracted_parameters,
            "spatial_review": spatial_review,
        },
    }


def reasoner_result_dict(result: Any) -> dict[str, Any]:
    status = {ACCEPT: "plan", CLARIFY: "clarification", REFUSE: "refusal"}.get(
        result.status_code, "error"
    )
    return {
        "outcome": status,
        "status_code": result.status_code,
        "message": result.message,
        "clarification_question": result.clarification_question,
        "reasoning": result.reasoning,
        "matched_capabilities": result.matched_capabilities,
        "missing_capabilities": result.missing_capabilities,
        "candidate_trees": result.candidate_trees,
    }


def requirements_shape_errors(value: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    expected = {
        "required_capabilities": list,
        "mission_intents": list,
        "constraints": dict,
        "ambiguities": list,
        "rationale": str,
    }
    for key, expected_type in expected.items():
        if not isinstance(value.get(key), expected_type):
            errors.append(f"requirements field '{key}' must be {expected_type.__name__}")
    return errors


def execute_m3(
    item: Mapping[str, Any],
    model: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    seed: int,
    image_paths: Iterable[Path],
    dry_run: bool,
    timeout_s: int,
    max_transport_retries: int,
    refinement_attempts: int,
    scored: bool,
) -> dict[str, Any]:
    mission = item["mission"]
    mission_text = item["paraphrase"]["text"]
    context = item["context"]
    stages: list[dict[str, Any]] = []

    requirements_system, requirements_user = build_m3_requirements_prompt(
        mission_text, context, runtime
    )
    requirements_call = call_openrouter(
        model,
        requirements_system,
        requirements_user,
        image_paths=image_paths,
        seed=seed,
        dry_run=dry_run,
        timeout_s=timeout_s,
        max_transport_retries=max_transport_retries,
    )
    stages.append({"stage": "requirements", **requirements_call})
    if requirements_call.get("error"):
        return {"status": "transport_error", "stages": stages}
    if scored and requirements_call.get("provider_verified") is not True:
        return {
            "status": "protocol_error",
            "reason": "requirements provider could not be verified",
            "stages": stages,
        }
    extracted_requirements: dict[str, Any] = {}
    requirements_errors: list[str] = []
    requirements_json_parse_errors: list[str] = []
    if not dry_run:
        parsed, requirements_errors = parse_json_object(requirements_call["content"])
        requirements_json_parse_errors = list(requirements_errors)
        if parsed:
            extracted_requirements = parsed
            requirements_errors.extend(requirements_shape_errors(parsed))

    reasoner = MissionReasoner(runtime["system_description"])
    gate = reasoner.validate(
        mission_text,
        runtime["tree_catalogue"],
        canonical_json(context),
        extracted_requirements,
    )
    gate_record = reasoner_result_dict(gate)
    gate_record["requirements_json_parse_errors"] = requirements_json_parse_errors
    gate_record["requirements_errors"] = requirements_errors
    stages.append({"stage": "mission_reasoner", **gate_record})

    deterministic_outcome = gate_record["outcome"]
    if deterministic_outcome != "plan":
        correct_outcome = deterministic_outcome == item["expected_outcome"]
        task_success = bool(
            not requirements_errors
            and correct_outcome
        )
        return {
            "status": "complete" if correct_outcome else "validation_failed",
            "decision_outcome": deterministic_outcome,
            "expected_outcome": item["expected_outcome"],
            "correct_outcome": correct_outcome,
            "automated_task_success": task_success,
            "first_attempt_valid": not requirements_errors,
            "first_attempt_task_success": task_success,
            "stages": stages,
        }

    candidates = gate.candidate_trees
    selection_system, selection_user = build_m3_selection_prompt(
        mission_text, context, runtime, candidates
    )
    selection_call = call_openrouter(
        model,
        selection_system,
        selection_user,
        image_paths=image_paths,
        seed=seed,
        dry_run=dry_run,
        timeout_s=timeout_s,
        max_transport_retries=max_transport_retries,
    )
    stages.append({"stage": "selection", **selection_call})
    if selection_call.get("error"):
        return {"status": "transport_error", "stages": stages}
    if scored and selection_call.get("provider_verified") is not True:
        return {
            "status": "protocol_error",
            "reason": "selection provider could not be verified",
            "stages": stages,
        }
    selection: Optional[dict[str, Any]] = None
    selection_errors: list[str] = []
    selection_json_parse_errors: list[str] = []
    if dry_run:
        expected_tree = mission.get("expected", {}).get("tree_id")
        selected_tree_id = (
            expected_tree if expected_tree in candidates else (candidates[0] if candidates else None)
        )
    else:
        selection, selection_errors = parse_json_object(selection_call["content"])
        selection_json_parse_errors = list(selection_errors)
        selected_tree_id = selection.get("tree_id") if selection else None
        if selection and not isinstance(selection.get("rationale"), str):
            selection_errors.append("selection rationale must be a string")
        confidence = selection.get("confidence") if selection else None
        if not isinstance(confidence, (int, float)) or isinstance(confidence, bool):
            selection_errors.append("selection confidence must be numeric")
        elif not 0.0 <= float(confidence) <= 1.0:
            selection_errors.append("selection confidence must be within [0, 1]")
        if selected_tree_id not in candidates:
            selection_errors.append("selected tree is not in deterministic candidate set")
    stages.append(
        {
            "stage": "selection_validation",
            "selection": selection,
            "errors": selection_errors,
            "selection_json_parse_errors": selection_json_parse_errors,
            "candidate_trees": candidates,
        }
    )
    if selection_errors or not selected_tree_id:
        return {
            "status": "validation_failed",
            "decision_outcome": "invalid_plan",
            "expected_outcome": item["expected_outcome"],
            "correct_outcome": False,
            "automated_task_success": False,
            "first_attempt_valid": False,
            "first_attempt_task_success": False,
            "stages": stages,
        }

    tree = tree_by_id(runtime, selected_tree_id)
    if tree is None:
        return {"status": "protocol_error", "stages": stages}
    contract = tree.get("blackboard_contract", {})
    payload_system, payload_user = build_m3_payload_prompt(
        mission_text, context, tree
    )
    payload_call = call_openrouter(
        model,
        payload_system,
        payload_user,
        image_paths=image_paths,
        seed=seed,
        dry_run=dry_run,
        timeout_s=timeout_s,
        max_transport_retries=max_transport_retries,
    )
    stages.append({"stage": "payload_attempt_1", **payload_call})
    if payload_call.get("error"):
        return {"status": "transport_error", "stages": stages}
    if scored and payload_call.get("provider_verified") is not True:
        return {
            "status": "protocol_error",
            "reason": "payload provider could not be verified",
            "stages": stages,
        }
    if dry_run:
        return {
            "status": "dry_run",
            "decision_outcome": "plan",
            "expected_outcome": item["expected_outcome"],
            "correct_outcome": None,
            "first_attempt_valid": None,
            "first_attempt_task_success": None,
            "selected_tree": selected_tree_id,
            "stages": stages,
        }

    first_raw_output = payload_call["content"]
    payload, payload_errors = parse_payload_response(first_raw_output)
    parse_errors = list(payload_errors)
    contract_errors: list[str] = []
    mission_review_errors: list[str] = []
    if payload is not None:
        contract_errors = generated_payload_errors(payload, contract, context)
        mission_review_errors = review_payload(payload, context, mission)["errors"]
        payload_errors.extend(contract_errors)
        payload_errors.extend(mission_review_errors)
    first_attempt_valid = bool(
        not requirements_errors
        and not selection_errors
        and payload is not None
        and not parse_errors
        and not contract_errors
    )
    first_reference = (
        compare_payload_to_reference(payload, mission)
        if payload is not None and mission.get("expected", {}).get("canonical_payload")
        else {}
    )
    exact_reference_required = (
        mission.get("complexity", {}).get("scores", {}).get("spatial_reasoning", 0)
        < 2
    )
    first_attempt_task_success = bool(
        first_attempt_valid
        and not mission_review_errors
        and selected_tree_id == mission.get("expected", {}).get("tree_id")
        and (
            not exact_reference_required
            or not first_reference
            or first_reference.get("reference_match")
        )
    )
    stages.append(
        {
            "stage": "payload_validation_1",
            "payload": payload,
            "errors": payload_errors,
            "parse_errors": parse_errors,
            "payload_json_parse_errors": parse_errors,
            "contract_errors": contract_errors,
            "mission_review_errors": mission_review_errors,
        }
    )

    final_payload = payload
    final_errors = list(payload_errors)
    final_parse_errors = list(parse_errors)
    final_contract_errors = list(contract_errors)
    final_mission_review_errors = list(mission_review_errors)
    prior_raw = first_raw_output
    for refinement_index in range(refinement_attempts):
        if not final_errors:
            break
        payload_system, payload_user = build_m3_payload_prompt(
            mission_text,
            context,
            tree,
            prior_output=prior_raw,
            validation_errors=final_errors,
        )
        refinement_call = call_openrouter(
            model,
            payload_system,
            payload_user,
            image_paths=image_paths,
            seed=seed + refinement_index + 1,
            dry_run=False,
            timeout_s=timeout_s,
            max_transport_retries=max_transport_retries,
        )
        attempt_number = refinement_index + 2
        stages.append(
            {"stage": f"payload_attempt_{attempt_number}", **refinement_call}
        )
        if refinement_call.get("error"):
            break
        if scored and refinement_call.get("provider_verified") is not True:
            final_errors.append("refinement provider could not be verified")
            break
        prior_raw = refinement_call["content"]
        final_payload, final_parse_errors = parse_payload_response(prior_raw)
        final_errors = list(final_parse_errors)
        final_contract_errors = []
        final_mission_review_errors = []
        if final_payload is not None:
            final_contract_errors = generated_payload_errors(
                final_payload, contract, context
            )
            final_mission_review_errors = review_payload(
                final_payload, context, mission
            )["errors"]
            final_errors.extend(final_contract_errors)
            final_errors.extend(final_mission_review_errors)
        stages.append(
            {
                "stage": f"payload_validation_{attempt_number}",
                "payload": final_payload,
                "errors": final_errors,
                "payload_json_parse_errors": final_parse_errors,
                "contract_errors": final_contract_errors,
                "mission_review_errors": final_mission_review_errors,
            }
        )

    final_valid = final_payload is not None and not final_errors
    reference = (
        compare_payload_to_reference(final_payload, mission)
        if final_payload is not None and mission.get("expected", {}).get("canonical_payload")
        else {}
    )
    actual_outcome = "plan" if final_valid else "invalid_plan"
    return {
        "status": "complete" if final_valid else "validation_failed",
        "decision_outcome": actual_outcome,
        "expected_outcome": item["expected_outcome"],
        "correct_outcome": actual_outcome == item["expected_outcome"],
        "automated_task_success": bool(
            final_valid
            and selected_tree_id == mission.get("expected", {}).get("tree_id")
            and (
                not exact_reference_required
                or not reference
                or reference.get("reference_match")
            )
        ),
        "first_attempt_valid": first_attempt_valid,
        "first_attempt_task_success": first_attempt_task_success,
        "selected_tree": selected_tree_id,
        "expected_tree": mission.get("expected", {}).get("tree_id"),
        "tree_match": selected_tree_id == mission.get("expected", {}).get("tree_id"),
        "final_payload": final_payload,
        "final_payload_valid": bool(
            final_payload is not None
            and not final_parse_errors
            and not final_contract_errors
        ),
        "final_spatial_valid": bool(
            final_payload is not None and not final_mission_review_errors
        ),
        "final_validation_errors": final_errors,
        "reference_comparison": reference,
        "stages": stages,
    }


def get_cost(record: Mapping[str, Any]) -> float:
    total = 0.0
    for stage in record.get("stages", []):
        try:
            total += float(stage.get("cost_usd", 0.0))
        except (TypeError, ValueError):
            pass
    if "api_call" in record:
        try:
            total += float(record["api_call"].get("cost_usd", 0.0))
        except (TypeError, ValueError):
            pass
    return total


def find_mission(core: Mapping[str, Any], mission_id: str) -> dict[str, Any]:
    for mission in core["missions"]:
        if mission["id"] == mission_id:
            return mission
    raise KeyError(mission_id)


def build_work_items(
    experiment: str,
    core: Mapping[str, Any],
    contexts: Mapping[str, Any],
    variants: Mapping[str, Any],
    safety: Mapping[str, Any],
    choice_space: Mapping[str, Any],
) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    if experiment == "E1":
        for mission in core["missions"]:
            for paraphrase in mission["paraphrases"]:
                items.append(
                    {
                        "mission": mission,
                        "paraphrase": paraphrase,
                        "context": contexts["fixtures"][mission["id"]],
                        "expected_outcome": mission["expected"]["outcome"],
                        "variant_id": None,
                    }
                )
    elif experiment == "E3":
        method_variants = (
            (
                "M1",
                choice_space["m1_action_library"]["variants"],
            ),
            (
                "M3",
                [
                    variant
                    for variant in choice_space["m3_tree_catalogue"]["variants"]
                    if variant.get("status") == "ready"
                ],
            ),
        )
        for mission in core["missions"]:
            for paraphrase in mission["paraphrases"]:
                for method, scale_variants in method_variants:
                    if method == "M1" and mission["platform"] != "husky":
                        continue
                    for scale_variant in scale_variants:
                        items.append(
                            {
                                "mission": mission,
                                "paraphrase": paraphrase,
                                "context": contexts["fixtures"][mission["id"]],
                                "expected_outcome": mission["expected"]["outcome"],
                                "variant_id": scale_variant["id"],
                                "variant_method": method,
                            }
                        )
    elif experiment == "E2":
        for group in variants["selected_missions"]:
            mission = find_mission(core, group["mission_id"])
            base_context = contexts["fixtures"][mission["id"]]
            for variant in group["variants"]:
                materialized, _ = context_variant(
                    variants, mission["id"], variant["id"], base_context
                )
                for paraphrase in mission["paraphrases"]:
                    items.append(
                        {
                            "mission": mission,
                            "paraphrase": paraphrase,
                            "context": materialized,
                            "expected_outcome": variant["expected_outcome"],
                            "variant_id": variant["id"],
                            "evaluation_reference": variant,
                        }
                    )
    elif experiment == "E4":
        for case in safety["cases"]:
            expected = dict(case["expected"])
            pseudo_mission = {
                "id": case["id"],
                "title": case["title"],
                "platform": case["platform"],
                "support_status": (
                    "blocked_blueboat_implementation"
                    if case["status"] == "blocked_blueboat_implementation"
                    else "implemented_catalogue"
                ),
                "complexity": {"label": "adverse"},
                "requirements": {},
                "expected": {
                    "outcome": expected["outcome"],
                    "tree_id": expected.get("tree_id"),
                    "canonical_payload": {},
                },
            }
            items.append(
                {
                    "mission": pseudo_mission,
                    "paraphrase": {"id": f"{case['id']}-P1", "text": case["mission"]},
                    "context": dict(case.get("context_overrides", {})),
                    "expected_outcome": expected["outcome"],
                    "variant_id": case["id"],
                    "safety_case": case,
                    "evaluation_reference": case,
                }
            )
    else:
        raise ValueError(f"Unsupported experiment {experiment}")
    return items


def narrow_runtime_for_e3(
    runtime: Mapping[str, Any],
    item: Mapping[str, Any],
    choice_space: Mapping[str, Any],
    distractor_catalogue: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], Optional[str]]:
    variant = item.get("variant_id")
    method = item.get("variant_method")
    if method == "M1":
        narrowed, condition = materialize_m1_action_library(
            runtime,
            choice_space,
            distractor_catalogue,
            str(variant),
            item["paraphrase"]["id"],
        )
        return narrowed, condition, None
    if method != "M3":
        return copy.deepcopy(dict(runtime)), {}, f"Unsupported E3 method {method}"

    variant_config = next(
        (
            entry
            for entry in choice_space["m3_tree_catalogue"]["variants"]
            if entry.get("id") == variant
        ),
        None,
    )
    if variant_config is None:
        return copy.deepcopy(dict(runtime)), {}, f"Unknown M3 variant {variant}"
    if variant_config.get("status") == "blocked":
        return (
            copy.deepcopy(dict(runtime)),
            {},
            str(variant_config.get("blocker", "M3 variant is blocked")),
        )

    narrowed = copy.deepcopy(dict(runtime))
    expected = item["mission"].get("expected", {}).get("tree_id")
    catalogue = runtime["tree_catalogue"]
    if variant == "CS1":
        chosen = [tree for tree in catalogue if tree.get("id") == expected]
        distractor = next(
            (tree for tree in catalogue if tree.get("id") != expected), None
        )
        if distractor:
            chosen.append(distractor)
        narrowed["tree_catalogue"] = chosen
    elif variant != "CS2":
        return narrowed, {}, f"M3 variant {variant} has no materialization rule"

    tree_ids = [tree["id"] for tree in narrowed["tree_catalogue"]]
    catalogue_hash = hashlib.sha256(canonical_json(tree_ids).encode("utf-8")).hexdigest()
    condition = {
        "method": "M3",
        "variant_id": variant,
        "tree_count": len(tree_ids),
        "tree_ids": tree_ids,
        "tree_catalogue_sha256": catalogue_hash,
    }
    narrowed["evaluation_condition"] = condition
    return narrowed, condition, None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Thesis evaluation runner")
    parser.add_argument("--experiment", choices=("E1", "E2", "E3", "E4"), default="E1")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scored", action="store_true")
    parser.add_argument(
        "--methods",
        help="Comma-separated M1,M2,M3. Defaults to M3 for E2, M1,M3 for E3, and all methods otherwise.",
    )
    parser.add_argument("--models", default="all")
    parser.add_argument("--platform", choices=("all", "husky", "blueboat"), default="all")
    parser.add_argument("--mission")
    parser.add_argument("--paraphrase")
    parser.add_argument("--variant")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--colab-url")
    parser.add_argument("--multimodal", action="store_true")
    parser.add_argument("--factory-helper")
    parser.add_argument("--m3-refinement-attempts", type=int, default=1)
    parser.add_argument("--max-transport-retries", type=int, default=3)
    parser.add_argument("--timeout-s", type=int, default=60)
    parser.add_argument("--max-cost-usd", type=float)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def preflight_scored(
    args: argparse.Namespace,
    core: Mapping[str, Any],
    model_conditions: Mapping[str, Any],
    methods: Iterable[str],
    models: Iterable[str],
    choice_space: Mapping[str, Any],
    m1_distractors: Mapping[str, Any],
) -> list[str]:
    errors: list[str] = []
    if not args.scored:
        return errors
    if core.get("freeze_status") != "frozen":
        errors.append("core_missions.json is not frozen")
    scoring = load_json(PROTOCOL / "scoring_rubric.json")
    if scoring.get("freeze_status") != "frozen":
        errors.append("scoring_rubric.json is not frozen")
    if args.experiment == "E2":
        if load_json(PROTOCOL / "context_variants.json").get("freeze_status") != "frozen":
            errors.append("context_variants.json is not frozen")
    if args.experiment == "E4":
        if load_json(PROTOCOL / "safety_cases.json").get("freeze_status") != "frozen":
            errors.append("safety_cases.json is not frozen")
    if any(method in {"M1", "M2"} for method in methods):
        helper = Path(args.factory_helper) if args.factory_helper else None
        if helper is None or not helper.is_file() or not os.access(helper, os.X_OK):
            errors.append("M1 and M2 scored runs require an executable --factory-helper")
    fixed = scoring.get("scored_run_settings", {})
    for name, actual in (
        ("repetitions", args.repetitions),
        ("seed", args.seed),
        ("timeout_s", args.timeout_s),
        ("max_transport_retries", args.max_transport_retries),
        ("m3_refinement_attempts", args.m3_refinement_attempts),
    ):
        if fixed.get(name) != actual:
            errors.append(f"{name} must equal the frozen scoring value {fixed.get(name)}")
    if args.experiment == "E3" and choice_space.get("freeze_status") != "frozen":
        errors.append("choice_space_variants.json is not frozen")
    if (
        args.experiment == "E3"
        and "M1" in methods
        and m1_distractors.get("freeze_status") != "frozen"
    ):
        errors.append("m1_action_distractors.json is not frozen")
    for key in models:
        model = model_conditions["models"][key]
        if model.get("provider_freeze_status") != "frozen":
            errors.append(f"{key}: provider is not frozen")
        if len(model.get("provider_only", [])) != 1:
            errors.append(f"{key}: exactly one provider_only entry is required")
    if "M2" in methods:
        local = model_conditions["local_model"]
        if local.get("freeze_status") != "frozen":
            errors.append("BTGenBot-2 base and adapter revisions are not frozen")
        if not args.colab_url and not args.dry_run:
            errors.append("M2 requires --colab-url")
    return errors


def conditions_per_item(
    methods: Iterable[str], model_keys: Iterable[str], repetitions: int
) -> int:
    general_methods = sum(method in ("M1", "M3") for method in methods)
    local_methods = sum(method == "M2" for method in methods)
    return (
        general_methods * len(list(model_keys)) + local_methods
    ) * repetitions


def main() -> int:
    args = parse_args()
    if args.repetitions < 1:
        print("--repetitions must be at least 1", file=sys.stderr)
        return 2
    if not 1 <= args.timeout_s <= 60:
        print("--timeout-s must be between 1 and 60", file=sys.stderr)
        return 2

    core = load_json(PROTOCOL / "core_missions.json")
    contexts = load_json(FIXTURES / "core_contexts.json")
    variants = load_json(PROTOCOL / "context_variants.json")
    safety = load_json(PROTOCOL / "safety_cases.json")
    runtime = load_json(PROTOCOL / "runtime_contract.json")
    model_conditions = load_json(PROTOCOL / "model_conditions.json")
    choice_space = load_json(PROTOCOL / "choice_space_variants.json")
    m1_distractors = load_json(PROTOCOL / "m1_action_distractors.json")

    default_methods = {
        "E2": "M3",
        "E3": "M1,M3",
    }.get(args.experiment, "M1,M2,M3")
    method_text = args.methods or default_methods
    methods = [value.strip().upper() for value in method_text.split(",") if value.strip()]
    unknown_methods = sorted(set(methods) - {"M1", "M2", "M3"})
    if unknown_methods:
        print(f"Unknown methods: {unknown_methods}", file=sys.stderr)
        return 2
    if args.experiment == "E2" and methods != ["M3"]:
        print(
            "E2 is a within-method context ablation for M3; use --methods M3.",
            file=sys.stderr,
        )
        return 2
    if args.experiment == "E3" and not set(methods).issubset({"M1", "M3"}):
        print(
            "E3 scales the M1 action library and M3 tree catalogue; use M1 and/or M3.",
            file=sys.stderr,
        )
        return 2
    if args.models == "all":
        model_keys = (
            [variants["reference_model_key"]]
            if args.experiment == "E2"
            else list(model_conditions["models"])
        )
    else:
        model_keys = [value.strip() for value in args.models.split(",") if value.strip()]
    unknown_models = sorted(set(model_keys) - set(model_conditions["models"]))
    if unknown_models:
        print(f"Unknown models: {unknown_models}", file=sys.stderr)
        return 2
    scored_errors = preflight_scored(
        args,
        core,
        model_conditions,
        methods,
        model_keys,
        choice_space,
        m1_distractors,
    )
    if scored_errors:
        print("SCORED RUN BLOCKED:", file=sys.stderr)
        for error in scored_errors:
            print(f"- {error}", file=sys.stderr)
        return 2

    items = build_work_items(
        args.experiment,
        core,
        contexts,
        variants,
        safety,
        choice_space,
    )
    if args.experiment == "E3":
        items = [item for item in items if item.get("variant_method") in methods]
    if args.platform != "all":
        items = [
            item for item in items if item["mission"]["platform"] == args.platform
        ]
    if args.mission:
        items = [item for item in items if item["mission"]["id"] == args.mission]
    if args.paraphrase:
        items = [
            item for item in items if item["paraphrase"]["id"] == args.paraphrase
        ]
    if args.variant:
        items = [item for item in items if item.get("variant_id") == args.variant]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    manifest_path = args.manifest or (
        EVALUATION / "manifests" / "generated" / f"{args.experiment}_{utc_now().replace(':', '-')}.json"
    )
    manifest = {
        "experiment": args.experiment,
        "started_at": utc_now(),
        "scored": args.scored,
        "dry_run": args.dry_run,
        "methods": methods,
        "models": model_keys,
        "repetitions": args.repetitions,
        "seed_base": args.seed,
        **protocol_hashes(),
        "choice_space_freeze_status": choice_space.get("freeze_status"),
        "m1_action_distractors_freeze_status": m1_distractors.get(
            "freeze_status"
        ),
        "execution_scoring_sha256": sha256(PROTOCOL / "execution_scoring.json"),
        "repository_commit": runtime["repository_commit"],
        "host": {
            "python": sys.version,
            "platform": host_platform.platform(),
        },
        "output_dir": str(args.output_dir.resolve()),
        "blocked_platform_rule": "BlueBoat cases are recorded as blocked and never counted as supported runs until its implementation contract is frozen.",
        "multimodal": args.multimodal,
    }
    write_json(manifest_path, manifest)

    protocol_fingerprint = condition_id(
        [
            manifest[key]
            for key in CONDITION_HASH_KEYS
        ]
    )

    total = skipped = failed = completed = 0
    cumulative_cost = 0.0
    factory_helper = Path(args.factory_helper) if args.factory_helper else None
    for item in items:
        mission = item["mission"]
        active_methods = (
            [item["variant_method"]]
            if item.get("variant_method")
            else methods
        )
        item_condition_count = conditions_per_item(
            active_methods, model_keys, args.repetitions
        )
        if mission["support_status"] != "implemented_catalogue":
            skipped += item_condition_count
            print(f"BLOCKED {mission['id']}: {mission['support_status']}")
            continue
        run_runtime, scale_condition, e3_blocker = (
            narrow_runtime_for_e3(
                runtime,
                item,
                choice_space,
                m1_distractors,
            )
            if args.experiment == "E3"
            else (dict(runtime), {}, None)
        )
        if e3_blocker:
            skipped += item_condition_count
            print(f"BLOCKED {mission['id']} {item.get('variant_id')}: {e3_blocker}")
            continue
        image_paths: list[Path] = []
        if args.multimodal:
            image_paths, image_errors = multimodal_preflight(item["context"])
            if image_errors:
                skipped += item_condition_count
                print(
                    f"BLOCKED {mission['id']} multimodal: "
                    + "; ".join(image_errors)
                )
                continue

        for method in active_methods:
            if method == "M2":
                condition_models = ["btgenbot2"]
            elif args.experiment == "E3" and method == "M3":
                condition_models = [
                    choice_space["m3_tree_catalogue"]["reference_model_key"]
                ]
            else:
                condition_models = model_keys
            for model_key in condition_models:
                for repetition in range(1, args.repetitions + 1):
                    total += 1
                    seed = args.seed + repetition - 1
                    identifier = condition_id(
                        [
                            args.experiment,
                            method,
                            model_key,
                            mission["id"],
                            item["paraphrase"]["id"],
                            item.get("variant_id") or "base",
                            repetition,
                            protocol_fingerprint,
                            scale_condition.get("action_library_sha256")
                            or scale_condition.get("tree_catalogue_sha256")
                            or "base",
                        ]
                    )
                    artifact_dir = args.output_dir / identifier
                    result_path = artifact_dir / "result.json"
                    if result_path.exists() and not args.force:
                        skipped += 1
                        print(f"RESUME {identifier}: result already exists")
                        continue
                    if (
                        args.max_cost_usd is not None
                        and cumulative_cost >= args.max_cost_usd
                    ):
                        print("BUDGET LIMIT REACHED")
                        manifest["finished_at"] = utc_now()
                        manifest["cumulative_cost_usd"] = cumulative_cost
                        manifest["stopped_by_budget"] = True
                        write_json(manifest_path, manifest)
                        return 0

                    artifact_dir.mkdir(parents=True, exist_ok=True)
                    request_record = {
                        "condition_id": identifier,
                        "experiment": args.experiment,
                        "method": method,
                        "model_key": model_key,
                        "mission_id": mission["id"],
                        "paraphrase_id": item["paraphrase"]["id"],
                        "variant_id": item.get("variant_id"),
                        "scale_condition": scale_condition or None,
                        "repetition": repetition,
                        "seed": seed,
                        "expected_outcome": item["expected_outcome"],
                        "evaluation_reference": item.get("evaluation_reference"),
                        "mission": mission,
                        "paraphrase": item["paraphrase"],
                        "context": item["context"],
                        "context_modality": "multimodal" if image_paths else "structured_text",
                        "runtime_contract_sha256": manifest["runtime_contract_sha256"],
                        "protocol_hashes": {
                            key: manifest[key] for key in CONDITION_HASH_KEYS
                        },
                        "scored_protocol": args.scored,
                        "run_settings": {
                            "repetitions": args.repetitions,
                            "seed": args.seed,
                            "timeout_s": args.timeout_s,
                            "max_transport_retries": args.max_transport_retries,
                            "m3_refinement_attempts": args.m3_refinement_attempts,
                        },
                        "started_at": utc_now(),
                    }
                    write_json(artifact_dir / "request.json", request_record)
                    print(
                        f"RUN {args.experiment} {method} {mission['id']} "
                        f"{item['paraphrase']['id']} "
                        f"{item.get('variant_id') or 'base'} [{model_key}] r{repetition}"
                    )

                    if method == "M3":
                        model = model_conditions["models"][model_key]
                        result = execute_m3(
                            item,
                            model,
                            run_runtime,
                            seed=seed,
                            image_paths=image_paths,
                            dry_run=args.dry_run,
                            timeout_s=args.timeout_s,
                            max_transport_retries=args.max_transport_retries,
                            refinement_attempts=args.m3_refinement_attempts,
                            scored=args.scored,
                        )
                    elif method == "M1":
                        system, user = build_m1_prompt(
                            mission,
                            item["paraphrase"],
                            item["context"],
                            run_runtime,
                            adverse=args.experiment == "E4",
                        )
                        api_call = call_openrouter(
                            model_conditions["models"][model_key],
                            system,
                            user,
                            image_paths=image_paths,
                            seed=seed,
                            dry_run=args.dry_run,
                            timeout_s=args.timeout_s,
                            max_transport_retries=args.max_transport_retries,
                        )
                        if args.dry_run:
                            evaluation = {
                                "status": "dry_run",
                                "correct_outcome": None,
                                "first_attempt_valid": None,
                            }
                        elif api_call.get("error"):
                            evaluation = {"status": "transport_error"}
                        elif args.scored and api_call.get("provider_verified") is not True:
                            evaluation = {
                                "status": "protocol_error",
                                "reason": "OpenRouter provider could not be verified",
                            }
                        else:
                            evaluation = direct_result(
                                method,
                                api_call["content"],
                                mission,
                                item["expected_outcome"],
                                run_runtime,
                                factory_helper,
                                item["context"],
                                decision_envelope=args.experiment == "E4",
                            )
                            evaluation["status"] = (
                                "complete"
                                if evaluation.get("correct_outcome")
                                else "validation_failed"
                            )
                        result = {"api_call": api_call, **evaluation}
                    else:
                        if not args.colab_url and not args.dry_run:
                            result = {
                                "status": "blocked",
                                "reason": "M2 requires --colab-url",
                            }
                        else:
                            task, actions = build_m2_prompt(
                                mission,
                                item["paraphrase"],
                                item["context"],
                                run_runtime,
                                adverse=args.experiment == "E4",
                            )
                            api_call = call_colab(
                                args.colab_url or "https://colab.invalid",
                                task,
                                actions,
                                seed=seed,
                                dry_run=args.dry_run,
                                timeout_s=args.timeout_s,
                                max_transport_retries=args.max_transport_retries,
                            )
                            if args.dry_run:
                                evaluation = {
                                    "status": "dry_run",
                                    "correct_outcome": None,
                                    "first_attempt_valid": None,
                                }
                            elif api_call.get("error"):
                                evaluation = {"status": "transport_error"}
                            elif args.scored and not api_call.get(
                                "raw_output_preserved"
                            ):
                                evaluation = {
                                    "status": "protocol_error",
                                    "reason": "Colab response did not preserve raw_text",
                                }
                            elif args.scored and (
                                revision_errors := btgenbot_revision_errors(
                                    api_call.get("generation_metadata", {}),
                                    model_conditions,
                                )
                            ):
                                evaluation = {
                                    "status": "protocol_error",
                                    "reason": "; ".join(revision_errors),
                                }
                            else:
                                evaluation = direct_result(
                                    method,
                                    api_call["content"],
                                    mission,
                                    item["expected_outcome"],
                                    run_runtime,
                                    factory_helper,
                                    item["context"],
                                    decision_envelope=args.experiment == "E4",
                                )
                                evaluation["status"] = (
                                    "complete"
                                    if evaluation.get("correct_outcome")
                                    else "validation_failed"
                                )
                            result = {"api_call": api_call, **evaluation}

                    result.update(
                        {
                            "condition_id": identifier,
                            "finished_at": utc_now(),
                            "scored": args.scored,
                            "scale_condition": scale_condition or None,
                        }
                    )
                    cost = get_cost(result)
                    result["cost_usd"] = cost
                    cumulative_cost += cost
                    write_json(result_path, result)
                    with args.log.open("a", encoding="utf-8") as log:
                        log.write(
                            json.dumps(
                                {
                                    "condition_id": identifier,
                                    "experiment": args.experiment,
                                    "method": method,
                                    "model_key": model_key,
                                    "mission_id": mission["id"],
                                    "paraphrase_id": item["paraphrase"]["id"],
                                    "variant_id": item.get("variant_id"),
                                    "action_node_count": scale_condition.get(
                                        "action_node_count"
                                    ),
                                    "distractor_count": scale_condition.get(
                                        "distractor_count"
                                    ),
                                    "repetition": repetition,
                                    "status": result.get("status"),
                                    "correct_outcome": result.get("correct_outcome"),
                                    "first_attempt_valid": result.get(
                                        "first_attempt_valid"
                                    ),
                                    "automated_task_success": result.get(
                                        "automated_task_success"
                                    ),
                                    "cost_usd": cost,
                                    "result_path": str(result_path.resolve()),
                                },
                                ensure_ascii=False,
                            )
                            + "\n"
                        )
                    if result.get("status") == "blocked":
                        skipped += 1
                    elif result.get("status") in ("transport_error", "protocol_error"):
                        failed += 1
                    else:
                        completed += 1

    manifest.update(
        {
            "finished_at": utc_now(),
            "conditions_started": total,
            "conditions_completed": completed,
            "conditions_failed": failed,
            "conditions_skipped_or_blocked": skipped,
            "cumulative_cost_usd": round(cumulative_cost, 8),
        }
    )
    write_json(manifest_path, manifest)
    print(
        f"Finished: {completed} completed, {failed} failed, {skipped} skipped/blocked; "
        f"cost recorded ${cumulative_cost:.6f}"
    )
    print(f"Manifest: {manifest_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
