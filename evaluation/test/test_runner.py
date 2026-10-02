from __future__ import annotations

import json
import sys
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "evaluation" / "scripts"))

import run_evaluation as runner
from build_cli_trials import _project_osm_context
from evaluation_core import load_json, materialize_m1_action_library
from run_evaluation import (
    btgenbot_revision_errors,
    build_work_items,
    call_btgenbot_openai,
    call_m1_model,
    call_openrouter,
    direct_result,
    e1_scoring_identity,
    protocol_hashes,
    openai_usage_cost,
)


RUNTIME = load_json(REPO / "evaluation" / "protocol" / "runtime_contract.json")
CORE = load_json(REPO / "evaluation" / "protocol" / "core_missions.json")
CONTEXTS = load_json(REPO / "evaluation" / "fixtures" / "context" / "core_contexts.json")
CONTEXT_VARIANTS = load_json(REPO / "evaluation" / "protocol" / "context_variants.json")
E2_CONTEXTS = load_json(REPO / "evaluation" / "fixtures" / "context" / "e2_contexts.json")
SAFETY = load_json(REPO / "evaluation" / "protocol" / "safety_cases.json")
CHOICE_SPACE = load_json(REPO / "evaluation" / "protocol" / "choice_space_variants.json")
M1_DISTRACTORS = load_json(
    REPO / "evaluation" / "protocol" / "m1_action_distractors.json"
)


def test_e2_osm_projection_keeps_only_mission_relevant_features():
    projected = _project_osm_context(
        CONTEXTS["fixtures"]["M3"],
        CONTEXT_VARIANTS["osm_context_projection"],
    )["osm_context"]

    assert set(projected) >= {
        "area_features", "linear_features", "point_features", "steps_features"
    }
    assert not set(projected) & {
        "tree_features", "waste_basket_features", "mission_target_features"
    }
    assert {item["kind"] for item in projected["area_features"]} == {"water"}
    assert {item["kind"] for item in projected["point_features"]} == {"barrier"}


def test_e2_missing_satellite_does_not_attach_the_transport_override():
    satellite = E2_CONTEXTS["fixtures"]["M3-CV2"]["satellite_map"]

    assert satellite["status"] == "unavailable"
    assert "path" not in satellite


def test_e1_scoring_identities_match_the_189_row_workbook_order():
    conditions = [
        ("M1", "gpt-5.6-sol"),
        ("M1", "gemini-3.8-flash"),
        ("M1", "gemma-4-26b"),
        ("M2", "btgenbot2"),
        ("M3", "gpt-5.6-sol"),
        ("M3", "gemini-3.8-flash"),
        ("M3", "gemma-4-26b"),
    ]
    identities = [
        e1_scoring_identity(method, model, mission, f"{mission}-P{paraphrase}", 1)
        for mission in ("S1", "S2", "S3", "M1", "M2", "M3", "C1", "C2", "C3")
        for paraphrase in (1, 2, 3)
        for method, model in conditions
    ]

    assert [number for number, _ in identities] == list(range(1, 190))
    assert identities[0][1] == "E1-M1-gpt-5.6-sol-S1-P1-r1"
    assert identities[-1][1] == "E1-M3-gemma-4-26b-C3-P3-r1"


def test_protocol_hashes_include_only_each_experiments_inputs():
    common = {
        "runtime_contract_sha256",
        "model_conditions_sha256",
        "core_dataset_sha256",
        "context_fixtures_sha256",
        "scoring_rubric_sha256",
    }
    assert set(protocol_hashes("E1")) == common
    assert set(protocol_hashes("E2")) == common | {"context_variants_sha256"}
    assert set(protocol_hashes("E3")) == common | {
        "choice_space_variants_sha256",
        "m1_action_distractors_sha256",
        "e3_gpt_flex_amendment_sha256",
    }
    assert set(protocol_hashes("E4")) == common | {"safety_cases_sha256"}


def test_e4_gpt_flex_manifest_is_frozen_and_openai_pinned():
    manifest = load_json(
        REPO / "evaluation" / "protocol" / "e4_m3_gpt_flex_trials.json"
    )
    model = manifest["models"]["gpt-5.6-sol"]

    assert manifest["freeze_status"] == "frozen"
    assert len(manifest["trials"]) == 10
    assert model["model_id"] == "openai/gpt-5.6-sol:floor"
    assert model["provider_only"] == ["openai"]
    assert model["allow_fallbacks"] is True
    assert model["accepted_service_tiers"] == ["flex", "default", None]


def test_openrouter_dry_request_allows_same_provider_tier_fallbacks():
    model = {
        "model_id": "test/model:floor",
        "temperature": 0.0,
        "max_tokens": 100,
        "provider_only": ["test-provider"],
        "allow_fallbacks": True,
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
        "allow_fallbacks": True,
        "require_parameters": True,
        "data_collection": "deny",
    }
    assert request["body"]["seed"] == 42
    assert "service_tier" not in request["body"]
    assert result["requested_service_tier"] is None
    result = call_openrouter(
        dict(model, temperature=None), "system", "user", image_paths=[],
        seed=42, dry_run=True, timeout_s=60, max_transport_retries=3,
    )
    assert "temperature" not in result["request"]["body"]


def test_openai_direct_dry_request_uses_native_parameters():
    model = {
        "model_id": "openai/gpt-5.6-sol",
        "openai_model_id": "gpt-5.6-sol",
        "m1_api_backend": "openai",
        "temperature": None,
        "max_tokens": 65536,
        "reasoning": {"effort": "xhigh"},
    }
    result = call_m1_model(
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
    assert request["url"] == "https://api.openai.com/v1/chat/completions"
    assert request["headers"]["Authorization"] == "<redacted>"
    assert request["body"]["model"] == "gpt-5.6-sol"
    assert request["body"]["max_completion_tokens"] == 65536
    assert request["body"]["reasoning_effort"] == "xhigh"
    assert request["body"]["store"] is False
    assert "provider" not in request["body"]
    assert "temperature" not in request["body"]
    assert result["api_backend"] == "openai"


def test_gemini_direct_dry_request_uses_native_parameters():
    model = {
        "model_id": "google/gemini-3.8-flash",
        "gemini_model_id": "gemini-3.8-flash",
        "m1_api_backend": "gemini",
        "temperature": 0.0,
        "gemini_temperature": None,
        "max_tokens": 65536,
        "reasoning": {"effort": "high"},
        "m1_reasoning": {"effort": "medium"},
    }
    result = call_m1_model(
        model,
        "system",
        "user",
        image_paths=[],
        seed=42,
        dry_run=True,
        timeout_s=300,
        max_transport_retries=3,
    )

    request = result["request"]
    assert request["url"] == (
        "https://generativelanguage.googleapis.com/v1beta/models/"
        "gemini-3.8-flash:generateContent"
    )
    assert request["headers"]["x-goog-api-key"] == "<redacted>"
    config = request["body"]["generationConfig"]
    assert config["maxOutputTokens"] == 65536
    assert config["thinkingConfig"]["thinkingLevel"] == "MEDIUM"
    assert config["seed"] == 42
    assert "temperature" not in config
    assert request["body"]["systemInstruction"] == {
        "parts": [{"text": "system"}]
    }
    assert result["api_backend"] == "gemini"


def test_gemini_direct_response_records_usage_and_provider(monkeypatch):
    monkeypatch.setattr(
        runner,
        "http_json",
        lambda *args, **kwargs: {
            "raw_response": {
                "candidates": [
                    {
                        "content": {"parts": [{"text": "<root/>"}]},
                        "finishReason": "STOP",
                    }
                ],
                "modelVersion": "gemini-3.8-flash",
                "usageMetadata": {
                    "promptTokenCount": 100,
                    "candidatesTokenCount": 20,
                    "thoughtsTokenCount": 30,
                    "totalTokenCount": 150,
                },
            },
            "cost_usd": None,
        },
    )
    result = call_m1_model(
        {
            "model_id": "google/gemini-3.8-flash",
            "gemini_model_id": "gemini-3.8-flash",
            "m1_api_backend": "gemini",
            "temperature": None,
            "max_tokens": 65536,
            "reasoning": {"effort": "high"},
            "gemini_pricing_usd_per_million": {
                "input": 0.75,
                "cached_input": 0.075,
                "output": 3.75,
            },
        },
        "system",
        "user",
        image_paths=[],
        seed=42,
        dry_run=False,
        timeout_s=300,
        max_transport_retries=3,
    )

    assert result["content"] == "<root/>"
    assert result["usage"]["completion_tokens"] == 50
    assert result["provider_verified"] is True
    assert result["cost_usd"] == 0.0002625


def test_openai_usage_cost_uses_cached_token_rate():
    cost = openai_usage_cost(
        {
            "prompt_tokens": 1000,
            "completion_tokens": 50,
            "prompt_tokens_details": {"cached_tokens": 100},
        },
        {"input": 4.0, "cached_input": 0.4, "output": 20.0},
    )
    assert cost == 0.00464


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
    conditions["local_model"]["revision_metadata_required"] = False
    assert btgenbot_revision_errors({}, conditions) == []
    assert btgenbot_revision_errors(
        dict(matching, adapter_revision="different"), conditions
    )


def test_btgenbot_openai_records_revision_metadata(monkeypatch):
    metadata = {"revision_freeze_status": "frozen"}

    monkeypatch.setattr(
        "run_evaluation.http_json",
        lambda *args, **kwargs: {
            "raw_response": {
                "model": "adapter",
                "choices": [{"message": {"content": "<root/>"}}],
                "metadata": metadata,
            }
        },
    )

    result = call_btgenbot_openai(
        "http://jetson/v1/chat/completions",
        {"adapter": "adapter"},
        "task",
        "actions",
        adverse=False,
        seed=42,
        dry_run=False,
        timeout_s=60,
        max_transport_retries=3,
    )

    assert result["generation_metadata"] == metadata


def test_btgenbot_openai_uses_official_user_message_shape():
    result = call_btgenbot_openai(
        "http://jetson/v1/chat/completions",
        {"adapter": "adapter", "max_tokens": 8192, "temperature": 1.0},
        "Concise behavior summary.",
        "[MoveTo (parameters: location)]",
        adverse=False,
        seed=42,
        dry_run=True,
        timeout_s=60,
        max_transport_retries=0,
    )

    body = result["request"]["body"]
    assert body["messages"][1]["content"] == (
        "Concise behavior summary.\n\nActions: [MoveTo (parameters: location)]"
    )
    assert body["max_tokens"] == 8192
    assert body["temperature"] == 1.0
    assert '<root BTCPP_format="4" main_tree_to_execute="MainTree">' in body[
        "messages"
    ][0]["content"]
    assert '<BehaviorTree ID="MainTree">' in body["messages"][0]["content"]


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
    } == {"M1-N11", "M1-N50", "M1-N100"}
    assert {
        item["variant_id"]
        for item in selected
        if item["variant_method"] == "M3"
    } == {"CS1", "CS2"}


def test_e3_m1_distractor_use_is_recorded_and_fails_task_success():
    scaled, _ = materialize_m1_action_library(
        RUNTIME, CHOICE_SPACE, M1_DISTRACTORS, "M1-N50", "S1-P1"
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


def test_m1_direct_gps_action_can_pass_without_parser_helper():
    selected = next(mission for mission in CORE["missions"] if mission["id"] == "S1")
    raw = """<root BTCPP_format="4" main_tree_to_execute="T">
    <BehaviorTree ID="T"><Sequence>
      <MoveToGPS gps_pose="48.2846166,11.6071509"/>
      <LogTemperature/>
    </Sequence></BehaviorTree></root>"""

    result = direct_result(
        "M1",
        raw,
        selected,
        "plan",
        RUNTIME,
        Path("/bin/true"),
        CONTEXTS["fixtures"]["S1"],
    )

    assert result["first_attempt_valid"] is True
    assert result["automated_task_success"] is True


def test_m1_scores_first_complete_xml_inside_surrounding_text():
    selected = next(mission for mission in CORE["missions"] if mission["id"] == "S1")
    raw = """Here is the requested tree:
```xml
<root BTCPP_format="4" main_tree_to_execute="T">
  <BehaviorTree ID="T"><Sequence>
    <MoveToGPS gps_pose="48.2846166,11.6071509"/>
    <LogTemperature/>
  </Sequence></BehaviorTree>
</root>
```
The tree navigates before logging."""

    result = direct_result(
        "M1",
        raw,
        selected,
        "plan",
        RUNTIME,
        Path("/bin/true"),
        CONTEXTS["fixtures"]["S1"],
    )

    assert result["first_attempt_valid"] is True
    assert result["automated_task_success"] is True
    assert result["xml_extraction"] == {
        "complete_root_found": True,
        "surrounding_text_removed": True,
        "validation_source": "first_complete_root",
    }
