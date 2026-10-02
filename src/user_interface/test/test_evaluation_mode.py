import hashlib
import json

import pytest

from user_interface.evaluation_mode import load_context_fixture, validate_trial_id


def test_load_context_fixture_builds_runtime_context_and_attachments(tmp_path):
    fixture_root = tmp_path / 'fixtures'
    context_dir = fixture_root / 'context'
    artifact = fixture_root / 'artifacts' / 'map.png'
    context_dir.mkdir(parents=True)
    artifact.parent.mkdir()
    artifact.write_bytes(b'png')
    metadata = fixture_root / 'artifacts' / 'map.json'
    metadata.write_text(
        json.dumps(
            {
                'image_width_px': 100,
                'image_height_px': 100,
                'meters_per_px': 0.8,
                'bounds': {
                    'north': 48.1,
                    'south': 47.9,
                    'east': 11.1,
                    'west': 10.9,
                },
            }
        ),
        encoding='utf-8',
    )
    source = context_dir / 'contexts.json'
    source.write_text(
        json.dumps(
            {
                'fixtures': {
                    'M1': {
                        'available_context': ['GPS_FIX', 'SATELLITE_MAP'],
                        'gps_fix': {'latitude': 48.0, 'longitude': 11.0},
                        'annotated_slam_map': {
                            'path': 'artifacts/map.png',
                            'sha256': hashlib.sha256(b'png').hexdigest(),
                        },
                        'satellite_map': {
                            'path': 'artifacts/map.png',
                            'metadata_path': 'artifacts/map.json',
                            'sha256': hashlib.sha256(b'png').hexdigest(),
                        },
                    }
                }
            }
        ),
        encoding='utf-8',
    )

    loaded = load_context_fixture(f'{source}#/fixtures/M1')

    assert loaded['context']['GPS_FIX']['latitude'] == 48.0
    assert 'ANNOTATED_SLAM_MAP_IMAGE' not in loaded['context']
    assert loaded['context']['SATELLITE_MAP']['uri'] == artifact.resolve().as_uri()
    assert loaded['context']['SATELLITE_MAP']['metadata_uri'] == metadata.resolve().as_uri()
    assert loaded['context']['SATELLITE_MAP']['map_metadata']['meters_per_px'] == 0.8
    assert loaded['attachment_uris'] == [artifact.resolve().as_uri()]
    assert len(loaded['fixture_sha256']) == 64


def test_trial_id_rejects_paths():
    assert validate_trial_id(
        'E1-M1-P2-method3-gemma-r1'
    ) == 'E1-M1-P2-method3-gemma-r1'
    with pytest.raises(ValueError):
        validate_trial_id('../escape')
