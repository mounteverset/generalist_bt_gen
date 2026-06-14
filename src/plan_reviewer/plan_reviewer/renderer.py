from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse

from PIL import Image, ImageDraw, ImageFont

from user_interface.pending_plan_preview import normalize_pending_plan


def load_json_value(raw_value: Any) -> Any:
    if raw_value in (None, ''):
        return None
    if isinstance(raw_value, (dict, list, int, float, bool)):
        return raw_value
    if not isinstance(raw_value, str):
        return None
    try:
        return json.loads(raw_value)
    except Exception:
        return None


def render_plan_review_image(
    *,
    session_id: str,
    subtree_id: str,
    user_command: str,
    payload_json: str,
    context_snapshot_json: str,
    attachment_uris: Iterable[str],
    output_directory: Path,
) -> Dict[str, Any]:
    plan = {
        'session_id': session_id,
        'tree_id': subtree_id,
        'mission': user_command,
        'payload_json': payload_json,
        'context_snapshot_json': context_snapshot_json,
        'attachment_uris': list(attachment_uris or []),
    }
    normalized = normalize_pending_plan(plan, lambda uri: uri) or plan
    map_preview = normalized.get('map_preview')
    waypoints = list(normalized.get('waypoints') or [])
    area_polygon = list(normalized.get('area_polygon') or [])
    frontiers = list(normalized.get('frontiers') or [])
    render_info: Dict[str, Any] = {
        'normalized_plan': normalized,
        'waypoints': waypoints,
        'area_polygon': area_polygon,
        'frontiers': frontiers,
        'map_available': False,
        'image_uri': '',
        'image_path': '',
        'waypoint_pixels': [],
        'area_polygon_pixels': [],
        'frontier_pixels': [],
        'waypoint_area_checks': [],
        'render_warnings': [],
    }

    if not isinstance(map_preview, dict):
        render_info['render_warnings'].append('No map preview was available.')
        return render_info

    source_path = resolve_artifact_path(str(map_preview.get('uri') or ''))
    if source_path is None:
        render_info['render_warnings'].append(
            f"Map artifact was not readable: {map_preview.get('uri') or ''}"
        )
        return render_info

    with Image.open(source_path) as source:
        image = source.convert('RGBA')

    width, height = image.size
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    waypoint_pixels = [
        waypoint_pixel(waypoint, map_preview, width, height)
        for waypoint in waypoints
    ]
    area_polygon_pixels = [
        waypoint_pixel(point, map_preview, width, height)
        for point in area_polygon
    ]
    frontier_pixels = [
        waypoint_pixel(point, map_preview, width, height)
        for point in frontiers
    ]
    render_info['waypoint_pixels'] = waypoint_pixels
    render_info['area_polygon_pixels'] = area_polygon_pixels
    render_info['frontier_pixels'] = frontier_pixels

    polygon_line_points = [
        (item['pixel_x'], item['pixel_y'])
        for item in area_polygon_pixels
        if item.get('pixel_x') is not None and item.get('pixel_y') is not None
    ]
    if len(polygon_line_points) >= 3:
        draw.polygon(polygon_line_points, fill=(14, 165, 233, 45))
        draw.line(
            polygon_line_points + [polygon_line_points[0]],
            fill=(14, 165, 233, 240),
            width=max(3, width // 180),
        )
        render_info['waypoint_area_checks'] = [
            {
                'index': item.get('index'),
                'in_area': (
                    item.get('pixel_x') is not None
                    and item.get('pixel_y') is not None
                    and point_in_polygon(
                        float(item['pixel_x']),
                        float(item['pixel_y']),
                        polygon_line_points,
                    )
                ),
            }
            for item in waypoint_pixels
        ]

    line_points = [
        (item['pixel_x'], item['pixel_y'])
        for item in waypoint_pixels
        if item.get('pixel_x') is not None and item.get('pixel_y') is not None
    ]
    if len(line_points) >= 2:
        draw.line(line_points, fill=(37, 99, 235, 230), width=max(3, width // 220))

    radius = max(7, min(width, height) // 48)
    for item in waypoint_pixels:
        x = item.get('pixel_x')
        y = item.get('pixel_y')
        if x is None or y is None:
            continue
        fill = (37, 99, 235, 245) if item.get('in_bounds') else (245, 158, 11, 245)
        outline = (255, 255, 255, 250)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=fill, outline=outline, width=2)
        label = str(item.get('index', ''))
        bbox = draw.textbbox((0, 0), label, font=font)
        text_w = bbox[2] - bbox[0]
        text_h = bbox[3] - bbox[1]
        draw.text((x - text_w / 2, y - text_h / 2), label, fill=(255, 255, 255, 255), font=font)

    frontier_radius = max(6, min(width, height) // 60)
    for item in frontier_pixels:
        x = item.get('pixel_x')
        y = item.get('pixel_y')
        if x is None or y is None:
            continue
        points = [
            (x, y - frontier_radius),
            (x + frontier_radius, y),
            (x, y + frontier_radius),
            (x - frontier_radius, y),
        ]
        draw.polygon(points, fill=(45, 212, 191, 245), outline=(17, 24, 39, 250))

    robot_pixel = robot_pose_pixel(normalized, map_preview, width, height)
    if robot_pixel:
        x, y = robot_pixel
        size = max(10, min(width, height) // 36)
        draw.polygon(
            [(x, y - size), (x - size * 0.75, y + size * 0.75), (x + size * 0.75, y + size * 0.75)],
            fill=(220, 38, 38, 245),
            outline=(255, 255, 255, 250),
        )

    draw_metadata_banner(draw, image.size, normalized, map_preview, font)

    output_directory.mkdir(parents=True, exist_ok=True)
    safe_session = ''.join(ch if ch.isalnum() or ch in ('-', '_') else '_' for ch in session_id or 'unknown')
    output_path = output_directory / f'plan_review_{safe_session}_{int(time.time() * 1000)}.png'
    image.convert('RGB').save(output_path, format='PNG')

    render_info.update(
        {
            'map_available': True,
            'image_uri': output_path.as_uri(),
            'image_path': str(output_path),
        }
    )
    return render_info


def resolve_artifact_path(uri: str) -> Optional[Path]:
    if not uri:
        return None
    parsed = urlparse(uri)
    if parsed.scheme in ('', 'file'):
        path = Path(parsed.path if parsed.scheme == 'file' else uri)
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
        return path if path.exists() else None
    return None


def waypoint_pixel(
    waypoint: Dict[str, Any],
    map_preview: Dict[str, Any],
    width: int,
    height: int,
) -> Dict[str, Any]:
    result = {
        'index': waypoint.get('index'),
        'waypoint': waypoint,
        'pixel_x': None,
        'pixel_y': None,
        'in_bounds': False,
        'reason': '',
    }
    if map_preview.get('coordinate_mode') == 'geographic':
        bounds = (map_preview.get('map_metadata') or {}).get('bounds')
        lat = coerce_float(waypoint.get('lat'))
        lon = coerce_float(waypoint.get('lon'))
        pixel = geographic_pixel(lat, lon, bounds, width, height)
    else:
        metadata = map_preview.get('map_metadata') or {}
        x = coerce_float(waypoint.get('x'))
        y = coerce_float(waypoint.get('y'))
        pixel = planar_pixel(x, y, metadata, width, height)

    if pixel is None:
        result['reason'] = 'Waypoint could not be projected onto the selected map.'
        return result

    px, py = pixel
    result['pixel_x'] = px
    result['pixel_y'] = py
    result['in_bounds'] = 0 <= px < width and 0 <= py < height
    if not result['in_bounds']:
        result['reason'] = 'Projected waypoint is outside the map image bounds.'
    return result


def geographic_pixel(
    lat: Optional[float],
    lon: Optional[float],
    bounds: Any,
    width: int,
    height: int,
) -> Optional[Tuple[float, float]]:
    if lat is None or lon is None or not isinstance(bounds, dict):
        return None
    north = coerce_float(bounds.get('north'))
    south = coerce_float(bounds.get('south'))
    east = coerce_float(bounds.get('east'))
    west = coerce_float(bounds.get('west'))
    if None in (north, south, east, west) or north == south or east == west:
        return None
    px = (lon - west) / (east - west) * width
    py = (north - lat) / (north - south) * height
    return px, py


def planar_pixel(
    x: Optional[float],
    y: Optional[float],
    metadata: Dict[str, Any],
    width: int,
    height: int,
) -> Optional[Tuple[float, float]]:
    if x is None or y is None:
        return None
    resolution = coerce_float(metadata.get('resolution_m_per_px') or metadata.get('meters_per_px'))
    if resolution in (None, 0):
        resolution = 1.0
    origin = metadata.get('origin') if isinstance(metadata.get('origin'), dict) else {}
    origin_x = coerce_float(origin.get('x')) or 0.0
    origin_y = coerce_float(origin.get('y')) or 0.0
    px = (x - origin_x) / resolution
    py = height - ((y - origin_y) / resolution)
    return px, py


def robot_pose_pixel(
    normalized: Dict[str, Any],
    map_preview: Dict[str, Any],
    width: int,
    height: int,
) -> Optional[Tuple[float, float]]:
    metadata = map_preview.get('map_metadata') or {}
    pose = metadata.get('robot_pose') if isinstance(metadata.get('robot_pose'), dict) else {}
    if map_preview.get('coordinate_mode') == 'geographic':
        context = normalized.get('context_snapshot') if isinstance(normalized.get('context_snapshot'), dict) else {}
        gps = context.get('GPS_FIX') if isinstance(context.get('GPS_FIX'), dict) else {}
        lat = coerce_float(pose.get('lat') or gps.get('latitude') or gps.get('lat'))
        lon = coerce_float(pose.get('lon') or pose.get('longitude') or gps.get('longitude') or gps.get('lon'))
        return geographic_pixel(lat, lon, metadata.get('bounds'), width, height)

    x = coerce_float(pose.get('x'))
    y = coerce_float(pose.get('y'))
    return planar_pixel(x, y, metadata, width, height)


def draw_metadata_banner(
    draw: ImageDraw.ImageDraw,
    size: Tuple[int, int],
    normalized: Dict[str, Any],
    map_preview: Dict[str, Any],
    font: ImageFont.ImageFont,
) -> None:
    width, _height = size
    metadata = map_preview.get('map_metadata') or {}
    lines = [
        f"Mission: {str(normalized.get('mission') or '')[:90]}",
        (
            f"Map: {map_preview.get('source_key', 'unknown')} "
            f"mode={map_preview.get('coordinate_mode', 'unknown')} "
            f"waypoints={len(normalized.get('waypoints') or [])} "
            f"polygon={len(normalized.get('area_polygon') or [])} "
            f"frontiers={len(normalized.get('frontiers') or [])}"
        ),
    ]
    if isinstance(metadata.get('bounds'), dict):
        bounds = metadata['bounds']
        lines.append(
            f"Bounds N {bounds.get('north')} S {bounds.get('south')} "
            f"E {bounds.get('east')} W {bounds.get('west')}"
        )
    elif isinstance(metadata.get('origin'), dict):
        origin = metadata['origin']
        lines.append(
            f"Origin x={origin.get('x')} y={origin.get('y')} "
            f"res={metadata.get('resolution_m_per_px') or metadata.get('meters_per_px')}"
        )

    line_height = 14
    banner_height = 10 + line_height * len(lines)
    draw.rectangle((0, 0, width, banner_height), fill=(17, 24, 39, 215))
    y = 5
    for line in lines:
        draw.text((8, y), line, fill=(255, 255, 255, 255), font=font)
        y += line_height


def coerce_float(value: Any) -> Optional[float]:
    if value in (None, ''):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside
