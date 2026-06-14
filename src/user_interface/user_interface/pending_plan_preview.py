import json
from typing import Any, Callable, Dict, Iterable, List, Optional


MAP_CONTEXT_KEYS = ('SATELLITE_MAP', 'ANNOTATED_SLAM_MAP_IMAGE')


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


def extract_area_polygon(payload_value: Any) -> List[Dict[str, Any]]:
    payload = load_json_value(payload_value)
    if not isinstance(payload, dict):
        return []
    return _extract_waypoint_list(payload.get('area_polygon'))


def extract_frontiers(payload_value: Any) -> List[Dict[str, Any]]:
    payload = load_json_value(payload_value)
    if not isinstance(payload, dict):
        return []
    return _extract_waypoint_list(payload.get('frontiers'))


def build_map_preview(
    context_value: Any,
    attachment_uris: Iterable[str],
    artifact_url_builder: Callable[[str], str],
    waypoints: Optional[List[Dict[str, Any]]] = None,
) -> Optional[Dict[str, Any]]:
    context = load_json_value(context_value)
    if isinstance(context, dict):
        for key in MAP_CONTEXT_KEYS:
            candidate = _map_preview_from_context_entry(
                context.get(key),
                key,
                artifact_url_builder,
                waypoints or [],
            )
            if candidate:
                return candidate

    for uri in attachment_uris:
        if not isinstance(uri, str):
            continue
        lowered = uri.lower()
        if lowered.endswith('.png') and ('slam_map' in lowered or 'satellite_map' in lowered):
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

    attachment_uris = normalized.get('attachment_uris', [])
    if not isinstance(attachment_uris, list):
        attachment_uris = []
    normalized['attachment_uris'] = [uri for uri in attachment_uris if isinstance(uri, str)]

    payload_for_preview = payload_object or normalized.get('payload_json')
    raw_waypoints = extract_waypoints(payload_for_preview)
    raw_area_polygon = _extract_payload_point_list(payload_for_preview, 'area_polygon')
    raw_area_polygon_geo = _extract_payload_point_list(payload_for_preview, 'area_polygon_geo')
    raw_frontiers = _extract_payload_point_list(payload_for_preview, 'frontiers')
    raw_frontiers_geo = _extract_payload_point_list(payload_for_preview, 'frontiers_geo')
    map_selection_points = raw_waypoints
    if raw_area_polygon_geo or raw_frontiers_geo:
        map_selection_points = raw_area_polygon_geo + raw_frontiers_geo
    map_preview = build_map_preview(
        context_object or normalized.get('context_snapshot_json'),
        normalized['attachment_uris'],
        artifact_url_builder,
        map_selection_points,
    )
    if map_preview:
        normalized['map_preview'] = map_preview

    normalized['waypoints'] = _normalize_waypoints_for_preview(
        raw_waypoints,
        map_preview,
    )
    normalized['waypoint_count'] = len(normalized['waypoints'])
    normalized['area_polygon'] = _normalize_waypoints_for_preview(
        _select_overlay_points(raw_area_polygon, raw_area_polygon_geo, map_preview),
        map_preview,
    )
    normalized['frontiers'] = _normalize_waypoints_for_preview(
        _select_overlay_points(raw_frontiers, raw_frontiers_geo, map_preview),
        map_preview,
    )
    normalized['area_polygon_count'] = len(normalized['area_polygon'])
    normalized['frontier_count'] = len(normalized['frontiers'])

    return normalized


def _extract_payload_point_list(payload_value: Any, key: str) -> List[Dict[str, Any]]:
    payload = load_json_value(payload_value)
    if not isinstance(payload, dict):
        return []
    return _extract_waypoint_list(payload.get(key))


def _select_overlay_points(
    planar_points: List[Dict[str, Any]],
    geographic_points: List[Dict[str, Any]],
    map_preview: Optional[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    if (map_preview or {}).get('coordinate_mode') == 'geographic' and geographic_points:
        return geographic_points
    return planar_points or geographic_points


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

    latitude = _coerce_float(
        raw_waypoint.get('lat', raw_waypoint.get('latitude', position.get('lat', position.get('latitude'))))
    )
    longitude = _coerce_float(
        raw_waypoint.get('lon', raw_waypoint.get('longitude', raw_waypoint.get('lng', position.get(
            'lon', position.get('longitude', position.get('lng'))))))
    )
    x = _coerce_float(position.get('x'))
    y = _coerce_float(position.get('y'))
    if latitude is None or longitude is None:
        if x is None or y is None:
            return None

    parsed = {'yaw_rad': yaw}
    if latitude is not None and longitude is not None:
        parsed['lat'] = latitude
        parsed['lon'] = longitude
    else:
        parsed['x'] = x
        parsed['y'] = y
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


def _map_preview_from_context_entry(
    entry: Any,
    source_key: str,
    artifact_url_builder: Callable[[str], str],
    waypoints: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    primary = _map_preview_from_entry(entry, source_key, artifact_url_builder)
    if source_key != 'SATELLITE_MAP' or not isinstance(entry, dict):
        return primary

    artifacts = entry.get('map_artifacts')
    if not isinstance(artifacts, list) or not waypoints:
        return primary

    candidates: List[Dict[str, Any]] = []
    if primary:
        candidates.append(primary)
    for artifact in artifacts:
        candidate = _map_preview_from_entry(artifact, source_key, artifact_url_builder)
        if candidate:
            candidates.append(candidate)

    return _best_satellite_artifact_for_waypoints(candidates, waypoints) or primary


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
    coordinate_mode = _coordinate_mode_for_preview(source_key, map_metadata)
    return {
        'source_key': source_key,
        'uri': uri,
        'image_url': artifact_url_builder(uri),
        'width': entry.get('width') or map_metadata.get('width') or map_metadata.get('image_width_px'),
        'height': entry.get('height') or map_metadata.get('height') or map_metadata.get('image_height_px'),
        'resolution': (
            entry.get('resolution')
            or map_metadata.get('resolution_m_per_px')
            or map_metadata.get('meters_per_px')
        ),
        'frame_id': entry.get('frame_id') or map_metadata.get('frame_id'),
        'timestamp': entry.get('timestamp') or map_metadata.get('timestamp'),
        'zoom': entry.get('zoom') or map_metadata.get('zoom'),
        'map_metadata': map_metadata,
        'coordinate_mode': coordinate_mode,
        'robot_pose': map_metadata.get('robot_pose', {}),
        'role': entry.get('role'),
        'reason': entry.get('reason'),
    }


def _best_satellite_artifact_for_waypoints(
    candidates: List[Dict[str, Any]],
    waypoints: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    scored: List[tuple] = []
    for order, candidate in enumerate(candidates):
        metadata = candidate.get('map_metadata')
        if not isinstance(metadata, dict) or not _has_valid_bounds(metadata.get('bounds')):
            continue
        normalized_waypoints = _normalize_waypoints_for_preview(waypoints, candidate)
        if not normalized_waypoints:
            continue
        if not all(
            _waypoint_in_geographic_bounds(waypoint, metadata.get('bounds'))
            for waypoint in normalized_waypoints
        ):
            continue
        zoom = _coerce_float(candidate.get('zoom')) or -1.0
        meters_per_px = (
            _coerce_float(candidate.get('resolution'))
            or _coerce_float(metadata.get('meters_per_px'))
            or _coerce_float(metadata.get('resolution_m_per_px'))
            or _bounds_area_degrees(metadata.get('bounds'))
        )
        scored.append((zoom, -float(meters_per_px), -order, candidate))
    if not scored:
        return None
    return max(scored, key=lambda item: item[:3])[3]


def _waypoint_in_geographic_bounds(waypoint: Dict[str, Any], bounds: Any) -> bool:
    if not isinstance(bounds, dict):
        return False
    latitude = _coerce_float(waypoint.get('lat'))
    longitude = _coerce_float(waypoint.get('lon'))
    north = _coerce_float(bounds.get('north'))
    south = _coerce_float(bounds.get('south'))
    east = _coerce_float(bounds.get('east'))
    west = _coerce_float(bounds.get('west'))
    if None in (latitude, longitude, north, south, east, west):
        return False
    return south <= latitude <= north and west <= longitude <= east


def _bounds_area_degrees(bounds: Any) -> float:
    if not isinstance(bounds, dict):
        return float('inf')
    north = _coerce_float(bounds.get('north'))
    south = _coerce_float(bounds.get('south'))
    east = _coerce_float(bounds.get('east'))
    west = _coerce_float(bounds.get('west'))
    if None in (north, south, east, west):
        return float('inf')
    return abs((north - south) * (east - west))


def _normalize_waypoints_for_preview(
    waypoints: List[Dict[str, Any]],
    map_preview: Optional[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    coordinate_mode = (map_preview or {}).get('coordinate_mode')
    if coordinate_mode != 'geographic':
        return waypoints

    normalized: List[Dict[str, Any]] = []
    for waypoint in waypoints:
        item = dict(waypoint)
        if not _has_lat_lon(item):
            x = _coerce_float(item.get('x'))
            y = _coerce_float(item.get('y'))
            if _looks_like_lat_lon(x, y):
                item['lat'] = x
                item['lon'] = y
        item['coordinate_mode'] = 'geographic'
        normalized.append(item)
    return normalized


def _coordinate_mode_for_preview(source_key: str, map_metadata: Dict[str, Any]) -> str:
    if source_key == 'SATELLITE_MAP' or _has_valid_bounds(map_metadata.get('bounds')):
        return 'geographic'
    return 'planar'


def _has_valid_bounds(bounds: Any) -> bool:
    if not isinstance(bounds, dict):
        return False
    north = _coerce_float(bounds.get('north'))
    south = _coerce_float(bounds.get('south'))
    east = _coerce_float(bounds.get('east'))
    west = _coerce_float(bounds.get('west'))
    return (
        north is not None and south is not None and east is not None and west is not None and
        north > south and east > west
    )


def _has_lat_lon(waypoint: Dict[str, Any]) -> bool:
    return _coerce_float(waypoint.get('lat')) is not None and _coerce_float(waypoint.get('lon')) is not None


def _looks_like_lat_lon(latitude: Optional[float], longitude: Optional[float]) -> bool:
    return (
        latitude is not None and longitude is not None and
        -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0
    )
