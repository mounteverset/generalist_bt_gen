from __future__ import annotations

import json
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "evaluation" / "scripts"))

from evaluation_core import load_json, materialize_m1_action_library
from run_evaluation import (
    btgenbot_revision_errors,
    build_work_items,
    call_openrouter,
    direct_result,
)


RUNTIME = load_json(REPO / "evaluation" / "protocol" / "runtime_contract.json")
CORE = load_json(REPO / "evaluation" / "protocol" / "core_missions.json")
CONTEXTS = load_json(REPO / "evaluation" / "fixtures" / "context" / "core_contexts.json")
CONTEXT_VARIANTS = load_json(REPO / "evaluation" / "protocol" / "context_variants.json")
SAFETY = load_json(REPO / "evaluation" / "protocol" / "safety_cases.json")
CHOICE_SPACE = load_json(REPO / "evaluation" / "protocol" / "choice_space_variants.json")
M1_DISTRACTORS = load_json(
    REPO / "evaluation" / "protocol" / "m1_action_distractors.json"
)


def test_openrouter_dry_request_disables_cache_and_provider_fallbacks():
    model = {
        "model_id": "test/model",
        "temperature": 0.0,
        "max_tokens": 100,
        "provider_only": ["test-provider"],
    }
    result = call_openrouter(
        model,
        "system",
        "user",
        image_paths=[],
        seed=42,
        dry_run=True,
        timeout_s=60,
        max_transport_retries=3,
    )

    request = result["request"]
    assert request["headers"]["X-OpenRouter-Cache"] == "false"
    assert request["headers"]["X-OpenRouter-Metadata"] == "enabled"
    assert request["headers"]["Authorization"] == "<redacted>"
    assert request["body"]["provider"] == {
        "only": ["test-provider"],
        "allow_fallbacks": False,
        "require_parameters": True,
        "data_collection": "deny",
    }
    assert request["body"]["seed"] == 42
    result = call_openrouter(
        dict(model, temperature=None), "system", "user", image_paths=[],
        seed=42, dry_run=True, timeout_s=60, max_transport_retries=3,
    )
    assert "temperature" not in result["request"]["body"]


def test_e4_decision_envelope_scores_refusal_without_xml():
    result = direct_result(
        "M1",
        json.dumps(
            {
                "action": "refuse",
                "rationale": "The platform cannot fly.",
            }
        ),
        {"expected": {"tree_id": None, "canonical_payload": {}}},
        "refusal",
        RUNTIME,
        None,
        {},
        decision_envelope=True,
    )

    assert result["correct_outcome"] is True
    assert result["decision_outcome"] == "refusal"


def test_btgenbot_revisions_must_match_the_frozen_protocol():
    conditions = {
        "local_model": {
            "base_model": "base",
            "base_revision": "base-sha",
            "adapter": "adapter",
            "adapter_revision": "adapter-sha",
        }
    }
    matching = {
        "base_model": "base",
        "base_revision": "base-sha",
        "adapter_model": "adapter",
        "adapter_revision": "adapter-sha",
        "revision_freeze_status": "frozen",
    }

    assert btgenbot_revision_errors(matching, conditions) == []
    assert btgenbot_revision_errors(
        dict(matching, adapter_revision="different"), conditions
    )


def test_e3_work_items_keep_m1_and_m3_scale_conditions_separate():
    items = build_work_items(
        "E3", CORE, CONTEXTS, CONTEXT_VARIANTS, SAFETY, CHOICE_SPACE
    )
    selected = [
        item
        for item in items
        if item["mission"]["id"] == "S1"
        and item["paraphrase"]["id"] == "S1-P1"
    ]
    assert {
        item["variant_id"]
        for item in selected
        if item["variant_method"] == "M1"
    } == {"M1-N12", "M1-N24", "M1-N50", "M1-N100"}
    assert {
        item["variant_id"]
        for item in selected
        if item["variant_method"] == "M3"
    } == {"CS1", "CS2"}


def test_e3_m1_distractor_use_is_recorded_and_fails_task_success():
    scaled, _ = materialize_m1_action_library(
        RUNTIME, CHOICE_SPACE, M1_DISTRACTORS, "M1-N24", "S1-P1"
    )
    selected = next(mission for mission in CORE["missions"] if mission["id"] == "S1")
    raw = """<root BTCPP_format="4" main_tree_to_execute="T">
    <BehaviorTree ID="T"><Sequence>
      <ParseWaypoints raw_waypoints="10.0,5.0,0.0" waypoint_queue="{q}" waypoint_count="{n}"/>
      <LoopString queue="{q}" value="{p}" if_empty="SUCCESS"><Sequence>
        <MoveTo pose="{p}"/>
        <LogTemperature logfile_path="/tmp/evaluation/S1_temperature.txt"/>
      </Sequence></LoopString>
      <MoveToLocation location_name="P1"/>
    </Sequence></BehaviorTree></root>"""
    result = direct_result(
        "M1",
        raw,
        selected,
        "plan",
        scaled,
        Path("/bin/true"),
        CONTEXTS["fixtures"]["S1"],
    )
    metrics = result["validation"]["action_library_metrics"]
    assert result["first_attempt_valid"] is True
    assert metrics["used_distractor_nodes"] == ["MoveToLocation"]
    assert metrics["distractor_use"] is True
    assert result["automated_task_success"] is False
