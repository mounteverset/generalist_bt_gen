from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'evaluation/scripts'))
sys.path.insert(0, str(ROOT / 'src/user_interface'))

from evaluation_core import load_json
from import_m3_cli_results import (
    FIRST_OUTPUT_ADJUDICATIONS,
    _audit_stages,
    _review_pass,
    import_trial,
)
from run_e1_m3_cli import (
    DEFAULT_MANIFEST,
    _write_trial_protocol,
    load_manifest,
)
from score_results import discover
from user_interface.evaluation_mode import load_context_fixture


def test_review_warning_is_accepted_for_operator_decision():
    assert _review_pass({'status': 'ok', 'content': '{"status":"warn"}'})


def test_projection_only_rejection_is_scored_as_warning():
    stage = {
        'status': 'ok',
        'content': json.dumps({
            'status': 'reject',
            'findings': [{
                'severity': 'critical',
                'category': 'coordinate_error',
                'description': 'Waypoints could not be projected onto the map overlay.',
            }],
        }),
    }
    assert _review_pass(stage)


def test_m3_extra_transit_logging_rejection_is_scored_as_warning():
    stage = {
        'status': 'ok',
        'content': json.dumps({
            'status': 'reject',
            'findings': [{
                'severity': 'critical',
                'category': 'mission_fulfillment',
                'description': 'The plan provides four locations instead of exactly three.',
            }],
        }),
    }
    assert _review_pass(stage, {'id': 'M3'})


def test_real_safety_rejection_remains_rejected():
    stage = {
        'status': 'ok',
        'content': json.dumps({
            'status': 'reject',
            'findings': [{
                'severity': 'critical',
                'category': 'robot_safety',
                'description': 'The route crosses water.',
            }],
        }),
    }
    assert not _review_pass(stage, {'id': 'M3'})


def test_direct_openai_transport_is_provider_verified(tmp_path):
    _write(
        tmp_path / 'llm_calls/1_payload_request.json',
        {
            'stage': 'payload',
            'model': 'gpt-5.6-sol',
            'provider': 'openai',
            'requested_provider_only': [],
        },
    )
    _write(
        tmp_path / 'llm_calls/1_payload_response.json',
        {
            'status': 'ok',
            'error': '',
            'raw_content': '{}',
            'returned_provider': None,
            'provider_verified': None,
        },
    )
    stage = _audit_stages(tmp_path)[0]
    assert stage['returned_provider'] == 'OpenAI'
    assert stage['provider_verified'] is True


def test_openrouter_flex_tier_is_recorded(tmp_path):
    _audit_call(tmp_path, 1, 'payload', '{}', service_tier='flex')

    stage = _audit_stages(tmp_path)[0]
    assert stage['requested_service_tier'] == 'flex'
    assert stage['returned_service_tier'] == 'flex'
    assert stage['service_tier_verified'] is True


def _write(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')


def _audit_call(
    directory: Path,
    call_id: int,
    stage: str,
    content: str,
    *,
    provider_verified: bool = True,
    model: str = 'openai/gpt-5.6-sol',
    provider: str = 'openrouter',
    provider_only: list[str] | None = None,
    service_tier: str | None = None,
) -> None:
    provider_only = ['openai'] if provider_only is None else provider_only
    _write(
        directory / 'llm_calls' / f'{call_id}_{stage}_request.json',
        {
            'stage': stage,
            'model': model,
            'provider': provider,
            'requested_provider_only': provider_only,
            'requested_service_tier': service_tier,
            'prompt': f'{stage} prompt',
            'attachment_uris': [],
        },
    )
    _write(
        directory / 'llm_calls' / f'{call_id}_{stage}_response.json',
        {
            'status': 'ok',
            'error': '',
            'raw_content': content,
            'returned_provider': 'OpenAI',
            'provider_verified': provider_verified,
            'returned_service_tier': service_tier,
            'service_tier_verified': True if service_tier else None,
            'usage_metadata': {
                'input_tokens': 10,
                'output_tokens': 5,
                'total_tokens': 15,
            },
            'response_metadata': {'model': model},
            'elapsed_s': 1.0,
        },
    )


def test_import_preserves_raw_evidence_and_feeds_scorer(tmp_path, monkeypatch):
    manifest = load_manifest()
    trial = manifest['trials'][0]
    raw_root = tmp_path / 'raw'
    raw = raw_root / trial['trial_id']
    raw.mkdir(parents=True)
    _write_trial_protocol(
        DEFAULT_MANIFEST,
        manifest,
        trial,
        raw_root,
        ['ros2', 'launch'],
        ['ros2', 'run'],
    )
    fixture = load_context_fixture(
        str(ROOT / 'evaluation/fixtures/context/core_contexts.json')
        + '#/fixtures/S1'
    )
    _write(
        raw / 'request.json',
        {
            'trial_id': trial['trial_id'],
            'mission': trial['mission'],
            'execution_requested': False,
            'goal_context': {
                'evaluation_source': {
                    key: fixture[key]
                    for key in (
                        'source_path', 'json_pointer', 'source_sha256', 'fixture_sha256'
                    )
                },
                'evaluation_context': fixture['context'],
            },
        },
    )
    mission = next(
        item for item in load_json(ROOT / 'evaluation/protocol/core_missions.json')['missions']
        if item['id'] == trial['mission_id']
    )
    tree = mission['expected']['tree_id']
    payload = dict(mission['expected']['canonical_payload'])
    payload['logfile_path'] = '/tmp/equivalent_temperature_log.txt'
    _write(
        raw / 'pending_plan.json',
        {
            'tree_id': tree,
            'payload_json': json.dumps(payload),
            'plan_review': {
                'status': 'pass',
                'summary': 'Plan satisfies deterministic and model review.',
                'findings': [],
            },
        },
    )
    _write(
        raw / 'result.json',
        {
            'trial_id': trial['trial_id'],
            'planning_status': 'captured',
            'selected_tree': tree,
            'execution_requested': False,
        },
    )
    _audit_call(raw, 1, 'requirements', json.dumps({'required_capabilities': []}))
    _audit_call(raw, 2, 'payload', json.dumps(payload))
    _audit_call(raw, 3, 'plan_review', json.dumps({'status': 'pass'}))
    before = {
        str(path.relative_to(raw)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in raw.rglob('*') if path.is_file()
    }

    output = tmp_path / 'normalized'
    import_trial(raw, output, trial, manifest, DEFAULT_MANIFEST)

    after = {
        str(path.relative_to(raw)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in raw.rglob('*') if path.is_file()
    }
    assert after == before
    request = load_json(output / trial['trial_id'] / 'request.json')
    result = load_json(output / trial['trial_id'] / 'result.json')
    assert request['experiment_number'] == 5
    assert request['condition_id'] == 'E1-M3-gpt-5.6-sol-S1-P1-r1'
    assert request['context'] == fixture['context']
    assert result['status'] == 'complete'
    assert result['automated_task_success'] is True
    assert result['provider_verified'] is True
    assert len(result['stages']) == 3

    rows = discover(output)
    assert rows[0]['experiment_number'] == 5
    assert rows[0]['provider_verified'] is True
    assert rows[0]['planning_call_count'] == 3
    assert rows[0]['input_tokens'] == 30
    assert rows[0]['output_tokens'] == 15
    assert rows[0]['plan_review_status'] == 'pass'
    assert rows[0]['syntax_valid'] is True

    _audit_call(raw, 4, 'payload', '{}')
    _audit_call(raw, 5, 'plan_review', json.dumps({'status': 'reject'}))
    _write(
        raw / 'pending_plan.json',
        {'tree_id': tree, 'payload_json': '{}', 'plan_review': {'status': 'reject'}},
    )
    monkeypatch.setitem(
        FIRST_OUTPUT_ADJUDICATIONS,
        trial['trial_id'],
        {'accepted_review_status': 'warn', 'reason': 'test first-output decision'},
    )
    adjudicated_output = tmp_path / 'adjudicated'
    import_trial(raw, adjudicated_output, trial, manifest, DEFAULT_MANIFEST)
    adjudicated = load_json(adjudicated_output / trial['trial_id'] / 'result.json')
    assert adjudicated['automated_task_success'] is True
    assert adjudicated['final_payload'] == payload
    assert adjudicated['scoring_adjudication']['selected_stage'] == 'payload_attempt_1'

    payload_response = raw / 'llm_calls/2_payload_response.json'
    response = load_json(payload_response)
    response['provider_verified'] = False
    _write(payload_response, response)
    failed_output = tmp_path / 'provider_failure'
    import_trial(raw, failed_output, trial, manifest, DEFAULT_MANIFEST)
    failed = load_json(failed_output / trial['trial_id'] / 'result.json')
    assert failed['status'] == 'protocol_error'
    assert failed['automated_task_success'] is False
    assert failed['provider_verified'] is False

    protocol_path = raw / 'trial_protocol.json'
    protocol = load_json(protocol_path)
    protocol['input_paths']['context_fixture'] = str(tmp_path / 'removed-context.json')
    _write(protocol_path, protocol)
    with pytest.raises(ValueError, match='frozen input changed or disappeared'):
        import_trial(raw, tmp_path / 'strict_stale', trial, manifest, DEFAULT_MANIFEST)
    import_trial(
        raw,
        tmp_path / 'rescored_stale',
        trial,
        manifest,
        DEFAULT_MANIFEST,
        rescore_stored_evidence=True,
    )


def _non_plan_trial(tmp_path: Path, manifest_name: str, trial_index: int, decision: str):
    """Write raw CLI evidence for a reasoner clarification or refusal."""
    manifest_path = ROOT / 'evaluation/protocol' / manifest_name
    manifest = load_manifest(manifest_path)
    trial = manifest['trials'][trial_index]
    raw_root = tmp_path / 'raw'
    raw = raw_root / trial['trial_id']
    raw.mkdir(parents=True)
    _write_trial_protocol(
        manifest_path, manifest, trial, raw_root, ['ros2', 'launch'], ['ros2', 'run']
    )
    path, _, pointer = trial['context_fixture'].partition('#')
    fixture = load_context_fixture(f'{ROOT / path}#{pointer}')
    _write(
        raw / 'request.json',
        {
            'trial_id': trial['trial_id'],
            'mission': trial['mission'],
            'execution_requested': False,
            'goal_context': {
                'evaluation_source': {
                    key: fixture[key]
                    for key in (
                        'source_path', 'json_pointer', 'source_sha256', 'fixture_sha256'
                    )
                },
                'evaluation_context': fixture['context'],
            },
        },
    )
    _write(raw / 'mission_decision.json', {'session_id': trial['trial_id'], 'decision': decision})
    _write(
        raw / 'result.json',
        {'trial_id': trial['trial_id'], 'planning_status': 'failed_before_plan'},
    )
    model_config = manifest['models'][trial['model']]
    transport = model_config['transport']
    _audit_call(
        raw, 1, 'requirements', json.dumps({'required_capabilities': []}),
        model=(
            'gpt-5.6-sol' if transport == 'openai' else 'openai/gpt-5.6-sol'
        ),
        provider=transport,
        provider_only=list(model_config['provider_only']),
        service_tier=model_config.get('service_tier'),
    )
    output = tmp_path / 'normalized'
    import_trial(raw, output, trial, manifest, manifest_path)
    return (
        load_json(output / trial['trial_id'] / 'request.json'),
        load_json(output / trial['trial_id'] / 'result.json'),
        discover(output),
    )


def test_e4_refusal_counts_as_correct_outcome(tmp_path):
    request, result, rows = _non_plan_trial(tmp_path, 'e4_m3_cli_trials.json', 0, 'refuse')
    assert request['experiment'] == 'E4'
    assert request['expected_outcome'] == 'refusal'
    assert result['decision_outcome'] == 'refusal'
    assert result['correct_outcome'] is True
    assert result['status'] == 'complete'
    assert rows[0]['primary_success'] is True
    assert rows[0]['false_acceptance'] is False


def test_e2_wrong_non_plan_class_fails(tmp_path):
    # M3-CV1 (text only) expects clarification; a refusal is the wrong class.
    request, result, rows = _non_plan_trial(tmp_path, 'e2_m3_cli_trials.json', 0, 'refuse')
    assert request['experiment'] == 'E2'
    assert request['evaluation_reference']['condition'] == 'text_only'
    assert result['correct_outcome'] is False
    assert rows[0]['primary_success'] is False


def test_e4_plan_review_rejection_counts_as_expected_refusal(tmp_path):
    manifest_path = ROOT / 'evaluation/protocol/e4_m3_cli_trials.json'
    manifest = load_manifest(manifest_path)
    trial = next(item for item in manifest['trials'] if item['mission_id'] == 'A07')
    raw_root = tmp_path / 'raw'
    raw = raw_root / trial['trial_id']
    raw.mkdir(parents=True)
    _write_trial_protocol(
        manifest_path, manifest, trial, raw_root, ['ros2', 'launch'], ['ros2', 'run']
    )
    path, _, pointer = trial['context_fixture'].partition('#')
    fixture = load_context_fixture(f'{ROOT / path}#{pointer}')
    _write(
        raw / 'request.json',
        {
            'trial_id': trial['trial_id'],
            'mission': trial['mission'],
            'execution_requested': False,
            'goal_context': {
                'evaluation_source': {
                    key: fixture[key]
                    for key in (
                        'source_path', 'json_pointer', 'source_sha256', 'fixture_sha256'
                    )
                },
                'evaluation_context': fixture['context'],
            },
        },
    )
    review = {
        'status': 'reject',
        'findings': [{'severity': 'critical', 'category': 'robot_safety',
                      'description': 'P9 lies outside the allowed mission polygon.'}],
    }
    _write(
        raw / 'pending_plan.json',
        {'tree_id': 'navigate_and_photograph.xml', 'payload_json': '{}', 'plan_review': review},
    )
    _write(raw / 'result.json', {'trial_id': trial['trial_id'], 'planning_status': 'captured'})
    _audit_call(raw, 1, 'requirements', json.dumps({'required_capabilities': []}))
    _audit_call(raw, 2, 'payload', '{}')
    _audit_call(raw, 3, 'plan_review', json.dumps(review))
    output = tmp_path / 'normalized'
    import_trial(raw, output, trial, manifest, manifest_path)
    result = load_json(output / trial['trial_id'] / 'result.json')
    assert result['decision_outcome'] == 'refusal'
    assert result['correct_outcome'] is True
    assert discover(output)[0]['false_acceptance'] is False
