import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_e1_m3_cli', ROOT / 'evaluation/scripts/run_e1_m3_cli.py'
)
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


def test_manifest_produces_complete_platform_specific_matrix():
    manifest = RUNNER.load_manifest()

    assert len(manifest['trials']) == 81
    for model in manifest['models']:
        husky = RUNNER.selected_trials(manifest, model, 'husky')
        blueboat = RUNNER.selected_trials(manifest, model, 'blueboat')
        assert len(husky) == 21
        assert len(blueboat) == 6
        assert RUNNER.launch_command(manifest, model, 'husky')[-1].endswith(
            '/config/system_description.yaml'
        )
        assert RUNNER.launch_command(manifest, model, 'blueboat')[-1].endswith(
            '/config/system_description_blueboat.yaml'
        )

    example = next(
        trial for trial in manifest['trials']
        if trial['mission_id'] == 'C2'
        and trial['paraphrase_id'] == 'C2-P3'
        and trial['model'] == 'gemma-4-26b'
    )
    assert example['experiment_number'] == 168
    assert example['trial_id'] == 'E1-C2-P3-method3-gemma-r1'
    assert example['scoring_condition_id'] == 'E1-M3-gemma-4-26b-C2-P3-r1'


def test_trial_cost_summary_records_each_call(tmp_path):
    trial = tmp_path / 'trial'
    calls = trial / 'llm_calls'
    calls.mkdir(parents=True)
    (calls / '1_payload_response.json').write_text(
        '{"status":"ok","cost_usd":0.01}', encoding='utf-8'
    )
    (calls / '2_review_response.json').write_text(
        '{"status":"ok","response_metadata":{"openrouter_cost_usd":0.02}}',
        encoding='utf-8',
    )

    summary = RUNNER._trial_cost_summary(trial)

    assert summary['cost_reporting_complete'] is True
    assert summary['total_cost_usd'] == 0.03
    assert (trial / 'cost_summary.json').is_file()


def test_selected_trials_can_filter_exact_trial_ids():
    manifest = RUNNER.load_manifest()
    trial_id = 'E1-C1-P1-method3-gpt56sol-r1'

    selected = RUNNER.selected_trials(
        manifest, 'gpt-5.6-sol', 'husky', {trial_id}
    )

    assert [trial['trial_id'] for trial in selected] == [trial_id]
