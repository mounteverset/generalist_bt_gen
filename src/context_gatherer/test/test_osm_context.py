from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
CONTEXT_GATHERER_NODE = (
    REPO_ROOT / 'src' / 'context_gatherer' / 'src' / 'context_gatherer_node.cpp'
)


def classify_overpass_element(element):
    tags = element.get('tags') or {}
    if 'barrier' in tags:
        return 'barrier'
    if tags.get('highway') == 'steps':
        return 'steps'
    if 'highway' in tags:
        return 'route'
    if tags.get('natural') == 'water':
        return 'water'
    if 'natural' in tags:
        return 'avoid_natural'
    if 'landuse' in tags or 'leisure' in tags:
        return 'area'
    if 'amenity' in tags or 'emergency' in tags:
        return 'landmark'
    return ''


def highway_kind(highway):
    if highway == 'steps':
        return 'steps'
    if highway in ('residential', 'unclassified', 'tertiary', 'living_street'):
        return 'street'
    if highway in ('path', 'track', 'service', 'footway', 'cycleway', 'pedestrian'):
        return 'path'
    return 'route'


def element_center_point(element):
    center = element.get('center')
    if isinstance(center, dict) and 'lat' in center and 'lon' in center:
        return {'lat': center['lat'], 'lon': center['lon']}
    if 'lat' in element and 'lon' in element:
        return {'lat': element['lat'], 'lon': element['lon']}
    geometry = element.get('geometry')
    if isinstance(geometry, list) and geometry:
        points = [point for point in geometry if 'lat' in point and 'lon' in point]
        if points:
            return {
                'lat': sum(point['lat'] for point in points) / len(points),
                'lon': sum(point['lon'] for point in points) / len(points),
            }
    return None


def sampled_geometry_coordinates(element, max_coordinates):
    geometry = element.get('geometry')
    if not isinstance(geometry, list):
        return [], False, 0
    points = [point for point in geometry if 'lat' in point and 'lon' in point]
    original_count = len(points)
    if not points:
        return [], False, 0
    limit = max(2, max_coordinates)
    truncated = original_count > limit
    output_count = limit if truncated else original_count
    coordinates = []
    last_index = -1
    for index in range(output_count):
        point_index = index
        if truncated:
            point_index = round(index * (original_count - 1) / (output_count - 1))
        if point_index == last_index:
            continue
        last_index = point_index
        point = points[point_index]
        coordinates.append({'lat': point['lat'], 'lon': point['lon']})
    return coordinates, truncated, original_count


def build_osm_context_like_context_gatherer(overpass, max_linear=40, max_coordinates=80):
    linear_features = []
    point_features = []
    area_features = []
    path_id = 1
    street_id = 1
    point_id = 1
    area_id = 1
    skipped_linear = 0
    truncated = False

    for element in overpass.get('elements') or []:
        tags = element.get('tags') or {}
        kind = classify_overpass_element(element)
        osm_type = element.get('type')
        if (
            osm_type == 'way'
            and isinstance(tags.get('highway'), str)
            and 'geometry' in element
        ):
            if len(linear_features) >= max_linear:
                skipped_linear += 1
                truncated = True
                continue
            feature_kind = highway_kind(tags['highway'])
            if feature_kind == 'street':
                label = f'S{street_id}'
                street_id += 1
            else:
                label = f'P{path_id}'
                path_id += 1
            coordinates, coordinates_truncated, original_count = sampled_geometry_coordinates(
                element, max_coordinates
            )
            if not coordinates:
                continue
            feature = {
                'label': label,
                'kind': feature_kind,
                'coordinates': coordinates,
                'original_coordinate_count': original_count,
                'recommended_use': (
                    'avoid' if feature_kind == 'steps' else 'candidate_route_verify'
                ),
            }
            if coordinates_truncated:
                feature['coordinates_truncated'] = True
                truncated = True
            center = element_center_point(element)
            if center:
                feature['center'] = center
            linear_features.append(feature)
            continue

        if kind in ('barrier', 'landmark') and element_center_point(element):
            point_features.append({'label': f'M{point_id}', 'kind': kind})
            point_id += 1
            continue

        if kind in ('water', 'avoid_natural', 'area'):
            area_features.append({'label': f'A{area_id}', 'kind': kind})
            area_id += 1

    return {
        'linear_features': linear_features,
        'point_features': point_features,
        'area_features': area_features,
        'limits': {
            'skipped_linear_features': skipped_linear,
            'truncated': truncated,
        },
    }


def test_overpass_query_requests_full_geometry_for_highway_ways():
    source = CONTEXT_GATHERER_NODE.read_text()

    assert ');out geom;' in source
    assert ');out geom center;' not in source


def test_way_geometry_fixture_builds_linear_features_with_coordinates():
    overpass = {
        'elements': [
            {
                'type': 'way',
                'id': 101,
                'tags': {'highway': 'path', 'surface': 'compacted'},
                'geometry': [
                    {'lat': 48.20280, 'lon': 11.64480},
                    {'lat': 48.20290, 'lon': 11.64495},
                    {'lat': 48.20305, 'lon': 11.64510},
                ],
            },
            {
                'type': 'way',
                'id': 102,
                'tags': {'highway': 'residential', 'name': 'Bauhofstrasse'},
                'geometry': [
                    {'lat': 48.20310, 'lon': 11.64520},
                    {'lat': 48.20320, 'lon': 11.64530},
                ],
            },
            {
                'type': 'node',
                'id': 201,
                'lat': 48.20285,
                'lon': 11.64470,
                'tags': {'barrier': 'bollard'},
            },
            {
                'type': 'way',
                'id': 301,
                'tags': {'natural': 'water'},
                'geometry': [
                    {'lat': 48.20350, 'lon': 11.64600},
                    {'lat': 48.20360, 'lon': 11.64620},
                ],
            },
        ]
    }

    context = build_osm_context_like_context_gatherer(overpass)

    assert [feature['label'] for feature in context['linear_features']] == ['P1', 'S1']
    assert context['linear_features'][0]['kind'] == 'path'
    assert context['linear_features'][0]['recommended_use'] == 'candidate_route_verify'
    assert context['linear_features'][0]['coordinates'][0] == {
        'lat': 48.20280,
        'lon': 11.64480,
    }
    assert context['linear_features'][0]['original_coordinate_count'] == 3
    assert context['linear_features'][0]['center']['lat'] == pytest.approx(48.20291666666667)
    assert context['linear_features'][0]['center']['lon'] == pytest.approx(11.64495)
    assert context['point_features'][0]['kind'] == 'barrier'
    assert context['area_features'][0]['kind'] == 'water'


def test_center_only_highway_fixture_exposes_previous_geometry_loss():
    overpass = {
        'elements': [
            {
                'type': 'way',
                'id': 101,
                'center': {'lat': 48.20290, 'lon': 11.64495},
                'nodes': [1, 2, 3],
                'tags': {'highway': 'path', 'surface': 'compacted'},
            }
        ]
    }

    context = build_osm_context_like_context_gatherer(overpass)

    assert context['linear_features'] == []
