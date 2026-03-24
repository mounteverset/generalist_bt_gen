import json
from typing import Any, Callable, Dict, Iterable, List, Optional


MAP_CONTEXT_KEYS = ('ANNOTATED_SLAM_MAP_IMAGE', 'SLAM_MAP_IMAGE')


def load_json_value(raw_value: Any) -> Any:
    if raw_value in (None, ''):
        return None
    if isinstance(raw_value, (dict, list, int, float, bool)):
        return raw_value
    if not isinstance(raw_value, str):
        return None
    stripped = raw_value.strip()
    if not stripped:
        return None
    try:
        return json.loads(stripped)
    except Exception:
        return None


def extract_waypoints(payload_value: Any) -> List[Dict[str, Any]]:
    payload = load_json_value(payload_value)
    if not isinstance(payload, dict):
        return []
    return _extract_waypoint_list(payload.get('waypoints'))


def build_map_preview(
    context_value: Any,
    attachment_uris: Iterable[str],
    artifact_url_builder: Callable[[str], str],
) -> Optional[Dict[str, Any]]:
    context = load_json_value(context_value)
    if isinstance(context, dict):
        for key in MAP_CONTEXT_KEYS:
            candidate = _map_preview_from_entry(context.get(key), key, artifact_url_builder)
            if candidate:
                return candidate

    for uri in attachment_uris:
        if not isinstance(uri, str):
            continue
        lowered = uri.lower()
        if lowered.endswith('.png') and 'slam_map' in lowered:
            return {
                'source_key': 'attachment',
                'uri': uri,
                'image_url': artifact_url_builder(uri),
                'map_metadata': {},
            }
    return None


def normalize_pending_plan(
    plan: Any,
    artifact_url_builder: Callable[[str], str],
) -> Optional[Dict[str, Any]]:
    if not isinstance(plan, dict):
        return None

    normalized = dict(plan)
    if 'reasoning' in normalized and 'summary' not in normalized:
        normalized['summary'] = normalized['reasoning']

    payload_object = load_json_value(normalized.get('payload_json'))
    if payload_object is not None:
        normalized['payload_object'] = payload_object

    context_object = load_json_value(normalized.get('context_snapshot_json'))
    if context_object is not None:
        normalized['context_snapshot'] = context_object

    normalized['waypoints'] = extract_waypoints(payload_object or normalized.get('payload_json'))
    normalized['waypoint_count'] = len(normalized['waypoints'])

    attachment_uris = normalized.get('attachment_uris', [])
    if not isinstance(attachment_uris, list):
        attachment_uris = []
    normalized['attachment_uris'] = [uri for uri in attachment_uris if isinstance(uri, str)]

    map_preview = build_map_preview(
        context_object or normalized.get('context_snapshot_json'),
        normalized['attachment_uris'],
        artifact_url_builder,
    )
    if map_preview:
        normalized['map_preview'] = map_preview

    return normalized


def _extract_waypoint_list(raw_waypoints: Any) -> List[Dict[str, Any]]:
    waypoints: List[Dict[str, Any]] = []
    if raw_waypoints in (None, ''):
        return waypoints

    if isinstance(raw_waypoints, str):
        for token in raw_waypoints.split(';'):
            parsed = _waypoint_from_string(token)
            if parsed:
                waypoints.append(parsed)
        return _finalize_waypoints(waypoints)

    if isinstance(raw_waypoints, list):
        for item in raw_waypoints:
            parsed = _waypoint_from_value(item)
            if parsed:
                waypoints.append(parsed)
        return _finalize_waypoints(waypoints)

    if isinstance(raw_waypoints, dict):
        nested = raw_waypoints.get('waypoints')
        if nested is None:
            for key in ('items', 'poses', 'points', 'targets'):
                if key in raw_waypoints:
                    nested = raw_waypoints.get(key)
                    break
        if nested is not None:
            return _extract_waypoint_list(nested)
        parsed = _waypoint_from_dict(raw_waypoints)
        if parsed:
            return _finalize_waypoints([parsed])

    return waypoints


def _waypoint_from_value(value: Any) -> Optional[Dict[str, Any]]:
    if isinstance(value, str):
        return _waypoint_from_string(value)
    if isinstance(value, dict):
        return _waypoint_from_dict(value)
    return None


def _waypoint_from_string(raw_waypoint: str) -> Optional[Dict[str, Any]]:
    cleaned = raw_waypoint.strip()
    if not cleaned:
        return None
    for token in '[]()':
        cleaned = cleaned.replace(token, '')
    values = []
    for part in cleaned.split(','):
        stripped = part.strip()
        if not stripped:
            continue
        try:
            values.append(float(stripped))
        except ValueError:
            return None
    if len(values) < 2:
        return None
    return {
        'x': values[0],
        'y': values[1],
        'yaw_rad': values[2] if len(values) >= 3 else 0.0,
        'raw': raw_waypoint.strip(),
    }


def _waypoint_from_dict(raw_waypoint: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    position = raw_waypoint
    if isinstance(raw_waypoint.get('position'), dict):
        position = raw_waypoint['position']
    elif isinstance(raw_waypoint.get('pose'), dict):
        pose = raw_waypoint['pose']
        if isinstance(pose.get('position'), dict):
            position = pose['position']
        else:
            position = pose

    x = _coerce_float(position.get('x'))
    y = _coerce_float(position.get('y'))
    if x is None or y is None:
        return None

    yaw = 0.0
    for key in ('yaw_rad', 'yaw', 'theta', 'heading'):
        candidate = _coerce_float(raw_waypoint.get(key))
        if candidate is not None:
            yaw = candidate
            break
        candidate = _coerce_float(position.get(key))
        if candidate is not None:
            yaw = candidate
            break

    parsed = {'x': x, 'y': y, 'yaw_rad': yaw}
    frame_id = raw_waypoint.get('frame_id') or position.get('frame_id')
    if isinstance(frame_id, str) and frame_id.strip():
        parsed['frame_id'] = frame_id.strip()
    return parsed


def _finalize_waypoints(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    finalized: List[Dict[str, Any]] = []
    for index, waypoint in enumerate(waypoints, start=1):
        item = dict(waypoint)
        item['index'] = index
        finalized.append(item)
    return finalized


def _coerce_float(value: Any) -> Optional[float]:
    if value in (None, ''):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _map_preview_from_entry(
    entry: Any,
    source_key: str,
    artifact_url_builder: Callable[[str], str],
) -> Optional[Dict[str, Any]]:
    if not isinstance(entry, dict):
        return None
    uri = entry.get('uri')
    if not isinstance(uri, str) or not uri.strip():
        return None
    map_metadata = entry.get('map_metadata')
    if not isinstance(map_metadata, dict):
        map_metadata = {}
    return {
        'source_key': source_key,
        'uri': uri,
        'image_url': artifact_url_builder(uri),
        'width': entry.get('width') or map_metadata.get('width'),
        'height': entry.get('height') or map_metadata.get('height'),
        'resolution': entry.get('resolution') or map_metadata.get('resolution_m_per_px'),
        'frame_id': entry.get('frame_id') or map_metadata.get('frame_id'),
        'timestamp': entry.get('timestamp') or map_metadata.get('timestamp'),
        'map_metadata': map_metadata,
        'robot_pose': map_metadata.get('robot_pose', {}),
    }
