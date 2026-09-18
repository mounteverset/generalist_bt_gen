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
                        "trial_id": "S1-sim-1",
                        "mission_id": "S1",
                        "platform": "husky",
                        "evidence_level": "simulation",
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
        assert summaries[-1]["missing_planned_trials"] == 5
        assert summaries[-1]["unaccounted_planned_trials"] == 5
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
