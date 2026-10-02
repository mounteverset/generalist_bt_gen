from __future__ import annotations

import copy
import hashlib
import json
import re
from pathlib import Path
from typing import Any


_CONTEXT_KEYS = {
    'robot_pose': 'ROBOT_POSE',
    'gps_fix': 'GPS_FIX',
    'annotated_slam_map': 'ANNOTATED_SLAM_MAP_IMAGE',
    'satellite_map': 'SATELLITE_MAP',
    'osm_context': 'OSM_CONTEXT',
    'find_anything': 'FIND_ANYTHING',
    'rgb360_sweep': 'RGB360SWEEP',
}
_IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png'}
_SAFE_TRIAL_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')


def validate_trial_id(value: str) -> str:
    trial_id = (value or '').strip()
    if not _SAFE_TRIAL_ID.fullmatch(trial_id):
        raise ValueError(
            'trial ID must start with an alphanumeric character and contain only '
            'letters, numbers, dots, underscores, or hyphens'
        )
    return trial_id


def load_context_fixture(reference: str) -> dict[str, Any]:
    path_text, separator, pointer = (reference or '').partition('#')
    if not path_text:
        raise ValueError('context fixture must include a JSON file path')
    path = Path(path_text).expanduser().resolve()
    raw = path.read_bytes()
    document = json.loads(raw)
    fixture = _resolve_json_pointer(document, pointer if separator else '')
    if not isinstance(fixture, dict):
        raise ValueError('context fixture JSON pointer must select an object')

    available = set(fixture.get('available_context', []))
    context = {
        _CONTEXT_KEYS.get(key, key): copy.deepcopy(value)
        for key, value in fixture.items()
        if key not in _CONTEXT_KEYS or _CONTEXT_KEYS[key] in available
    }
    artifact_root = path.parent.parent if path.parent.name == 'context' else path.parent
    attachments: list[str] = []
    _resolve_artifacts(context, artifact_root, attachments)
    fixture_json = json.dumps(fixture, sort_keys=True, separators=(',', ':')).encode()
    return {
        'source_path': str(path),
        'json_pointer': pointer if separator else '',
        'source_sha256': hashlib.sha256(raw).hexdigest(),
        'fixture_sha256': hashlib.sha256(fixture_json).hexdigest(),
        'context': context,
        'attachment_uris': list(dict.fromkeys(attachments)),
    }


def _resolve_json_pointer(document: Any, pointer: str) -> Any:
    if not pointer:
        return document
    if not pointer.startswith('/'):
        raise ValueError('context fixture fragment must be a JSON pointer starting with /')
    value = document
    for token in pointer[1:].split('/'):
        token = token.replace('~1', '/').replace('~0', '~')
        if isinstance(value, list):
            value = value[int(token)]
        elif isinstance(value, dict):
            value = value[token]
        else:
            raise ValueError(f'JSON pointer cannot descend through {type(value).__name__}')
    return value


def _resolve_artifacts(value: Any, artifact_root: Path, attachments: list[str]) -> None:
    if isinstance(value, list):
        for item in value:
            _resolve_artifacts(item, artifact_root, attachments)
        return
    if not isinstance(value, dict):
        return

    for item in list(value.values()):
        _resolve_artifacts(item, artifact_root, attachments)

    path_value = value.get('path')
    if isinstance(path_value, str) and Path(path_value).suffix.lower() in _IMAGE_SUFFIXES:
        artifact = Path(path_value).expanduser()
        if not artifact.is_absolute():
            artifact = artifact_root / artifact
        artifact = artifact.resolve()
        if not artifact.is_relative_to(artifact_root.resolve()):
            raise ValueError(f'fixture image escapes artifact root: {artifact}')
        if not artifact.is_file():
            raise FileNotFoundError(f'fixture image not found: {artifact}')
        declared_hash = str(value.get('sha256') or '').strip().lower()
        actual_hash = hashlib.sha256(artifact.read_bytes()).hexdigest()
        if declared_hash and declared_hash != actual_hash:
            raise ValueError(f'fixture image SHA-256 mismatch: {artifact}')
        uri = artifact.as_uri()
        value['uri'] = uri
        attachments.append(uri)

    metadata_value = value.get('metadata_path')
    if isinstance(metadata_value, str):
        metadata = Path(metadata_value).expanduser()
        if not metadata.is_absolute():
            metadata = artifact_root / metadata
        metadata = metadata.resolve()
        if not metadata.is_relative_to(artifact_root.resolve()):
            raise ValueError(f'fixture metadata escapes artifact root: {metadata}')
        if not metadata.is_file():
            raise FileNotFoundError(f'fixture metadata not found: {metadata}')
        metadata_document = json.loads(metadata.read_text(encoding='utf-8'))
        if not isinstance(metadata_document, dict):
            raise ValueError(f'fixture metadata must contain a JSON object: {metadata}')
        value['metadata_uri'] = metadata.as_uri()
        value['map_metadata'] = metadata_document
