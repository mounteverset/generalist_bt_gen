from __future__ import annotations

import json
import hashlib
import sys
import tempfile
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "evaluation" / "scripts"))

from evaluation_core import load_json
from score_results import apply_reviews, discover, score_execution, validate_correction


def write_json(path: Path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def test_scoring_pipeline_counts_effort_reviews_and_execution():
    rubric = load_json(REPO / "evaluation" / "protocol" / "scoring_rubric.json")
    execution_protocol = load_json(
        REPO / "evaluation" / "protocol" / "execution_scoring.json"
    )
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        artifact = root / "artifact"
        artifact.mkdir()
        write_json(
            artifact / "request.json",
            {
                "condition_id": "condition-1",
                "experiment": "E1",
                "method": "M1",
                "model_key": "model",
                "mission_id": "S1",
                "paraphrase_id": "S1-P1",
                "variant_id": None,
                "repetition": 1,
                "expected_outcome": "plan",
                "mission": {
                    "platform": "husky",
                    "complexity": {"label": "simple"},
                },
                "paraphrase": {"specificity": "low"},
            },
        )
        write_json(
            artifact / "result.json",
            {
                "scored": True,
                "status": "complete",
                "decision_outcome": "plan",
                "correct_outcome": True,
                "automated_task_success": True,
                "first_attempt_valid": True,
                "first_attempt_task_success": True,
                "cost_usd": 0.01,
                "api_call": {
                    "latency_s": 1.25,
                    "attempts": [{}, {}],
                    "usage": {"prompt_tokens": 10, "completion_tokens": 5},
                },
                "validation": {
                    "xml": {
                        "syntax_valid": True,
                        "interface_valid_static": True,
                        "factory_load": "pass",
                    }
                },
            },
        )
        rows = discover(root)
        assert len(rows) == 1
        assert rows[0]["planning_call_count"] == 1
        assert rows[0]["transport_retry_count"] == 1
        assert rows[0]["input_tokens"] == 10
        assert rows[0]["output_tokens"] == 5

        key = root / "key.json"
        responses = root / "responses.jsonl"
        write_json(
            key,
            {
                "R-1": {
                    "condition_id": "condition-1",
                    "applicable_elements": ["intent"],
                    "second_review_required": False,
                }
            },
        )
        responses.write_text(
            json.dumps(
                {
                    "review_id": "R-1",
                    "reviewers": [{"reviewer_id": "A", "scores": {"intent": 2}}],
                    "adjudicated_scores": None,
                    "correction": {
                        "status": "not_needed",
                        "manual_correction_count": 0,
                        "manual_correction_types": [],
                        "manual_time_s": 0,
                        "corrected_artifact_path": None,
                        "post_repair_validation_path": None,
                        "post_repair_deterministic_pass": None,
                        "post_repair_scores": None,
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        summary = apply_reviews(rows, responses, key, rubric)
        assert summary["reviewed_artifacts"] == 1
        assert rows[0]["human_semantic_pass"] is True
        assert rows[0]["success_after_manual_repair"] is True

        execution_file = root / "execution.json"
        integration_checks = {
            name: "pass"
            for name in execution_protocol["integration_checks_by_platform"]["husky"]
        }
        portability = [
            {
                "component": name,
                "classification": "shared_unchanged",
                "evidence": "src/example",
            }
            for name in execution_protocol["portability_components"]
        ]
        write_json(
            execution_file,
            {
                "protocol_version": execution_protocol["protocol_version"],
                "execution_scoring_sha256": hashlib.sha256(
                    (REPO / "evaluation" / "protocol" / "execution_scoring.json").read_bytes()
                ).hexdigest(),
                "trials": [
                    {
                        "trial_id": "S1-physical-1",
                        "mission_id": "S1",
                        "platform": "husky",
                        "evidence_level": "physical",
                        "repetition": 1,
                        "planning_condition_id": "planning-S1",
                        "planning_passed": True,
                        "started": True,
                        "terminal_outcome": "completed",
                        "duration_s": 12.0,
                        "factory_load": "pass",
                        "integration_checks": integration_checks,
                        "required_waypoints": 1,
                        "reached_waypoints": 1,
                        "required_measurements": 1,
                        "valid_measurements": 1,
                        "required_photos": 0,
                        "valid_photos": 0,
                        "operator_interventions": [],
                        "safety_incidents": [],
                    }
                ],
                "not_started": [],
                "portability": portability,
            },
        )
        trials, summaries, portability_summary, not_started = score_execution(
            execution_file,
            execution_protocol,
            hashlib.sha256(
                (REPO / "evaluation" / "protocol" / "execution_scoring.json").read_bytes()
            ).hexdigest(),
        )
        assert trials[0]["mission_completion"] is True
        assert trials[0]["autonomous_success"] is True
        assert summaries[-1]["missing_planned_trials"] == 26
        assert summaries[-1]["unaccounted_planned_trials"] == 26
        assert not not_started
        assert portability_summary[0]["shared_proportion"] == 1.0


def test_corrected_repair_requires_saved_artifact_and_two_passes():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        repaired = root / "repaired.xml"
        repaired.write_text("<root/>", encoding="utf-8")
        validation = root / "validation.json"
        validation.write_text("{}", encoding="utf-8")
        correction = {
            "status": "corrected",
            "manual_correction_count": 2,
            "manual_correction_types": ["tree_selection", "spatial_route"],
            "manual_time_s": 84,
            "corrected_artifact_path": repaired.name,
            "post_repair_validation_path": validation.name,
            "post_repair_deterministic_pass": True,
            "post_repair_scores": {"intent": 2},
        }
        validated = validate_correction(
            correction,
            {"tree_selection", "spatial_route"},
            "E1",
            "review",
            root,
            ["intent"],
        )
        assert validated["status"] == "corrected"
        repaired.unlink()
        try:
            validate_correction(
                correction,
                {"tree_selection", "spatial_route"},
                "E1",
                "review",
                root,
                ["intent"],
            )
        except ValueError:
            pass
        else:
            raise AssertionError("Missing repaired artifact was accepted")


def test_e1_rows_are_reused_as_e2_complete_and_e3_cs2():
    from score_results import paired_comparisons, reused_e1_rows

    def row(experiment, condition_id, variant, success, **extra):
        values = {
            "condition_id": condition_id,
            "experiment": experiment,
            "method": "M3",
            "model_key": "gpt-5.6-sol",
            "mission_id": "M3",
            "paraphrase_id": "M3-P2",
            "repetition": 1,
            "variant_id": variant,
            "base_condition": "M3:gpt-5.6-sol",
            "condition": f"M3:gpt-5.6-sol:{variant}" if variant else "M3:gpt-5.6-sol",
            "primary_success": success,
            "context_condition": None,
            "scale_value": None,
        }
        values.update(extra)
        return values

    gemma_e1 = row("E1", "gemma-e1", "", False)
    gemma_e1["model_key"] = "gemma-4-26b"
    gemma_e1["base_condition"] = "M3:gemma-4-26b"
    gemma_e1["condition"] = "M3:gemma-4-26b"
    gemma_e3 = row("E3", "gemma-e3", "CS1", True, scale_value=2)
    gemma_e3["model_key"] = "gemma-4-26b"
    gemma_e3["base_condition"] = "M3:gemma-4-26b"
    gemma_e3["condition"] = "M3:gemma-4-26b:CS1"
    rows = [
        row("E1", "e1", "", True),
        row("E2", "e2", "M3-CV2", False, context_condition="missing_one_source"),
        row("E3", "e3", "CS1", True, scale_value=2),
        gemma_e1,
        gemma_e3,
    ]
    clones = reused_e1_rows(rows)
    e2 = next(clone for clone in clones if clone["experiment"] == "E2")
    e3 = [clone for clone in clones if clone["experiment"] == "E3"]
    assert e2["variant_id"] == "M3-CV0"
    assert e2["context_condition"] == "complete"
    assert e2["reused_from"] == "e1"
    assert {clone["model_key"] for clone in e3} == {"gpt-5.6-sol", "gemma-4-26b"}
    assert {clone["scale_value"] for clone in e3} == {7}
    comparisons = paired_comparisons(rows + clones)
    assert {item["experiment"] for item in comparisons} == {"E2", "E3"}
    e2 = next(item for item in comparisons if item["experiment"] == "E2")
    assert e2["paired_artifact_n"] == 1
