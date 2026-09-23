import math
from pathlib import Path

import pytest
import yaml


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
    if tags.get('natural') == 'tree':
        return 'tree'
    if (
        tags.get('natural') in ('tree_row', 'coastline')
        or tags.get('waterway') == 'stream'
    ):
        return 'environmental_line'
    if 'natural' in tags:
        return 'avoid_natural'
    if is_area_element(element) and any(
        key in tags for key in ('landuse', 'leisure', 'tourism', 'amenity', 'surface')
    ):
        return 'area'
    if 'amenity' in tags or 'emergency' in tags:
        return 'landmark'
    return ''


def is_area_element(element):
    if element.get('type') == 'relation':
        return (element.get('tags') or {}).get('type') == 'multipolygon'
    if element.get('type') != 'way':
        return False
    nodes = element.get('nodes')
    if isinstance(nodes, list) and len(nodes) >= 4:
        return nodes[0] == nodes[-1]
    geometry = element.get('geometry')
    return (
        isinstance(geometry, list)
        and len(geometry) >= 4
        and geometry[0] == geometry[-1]
    )


def area_rings(element):
    def ring(geometry, nodes=None, role='outer', way_id=None):
        coordinates = []
        for index, point in enumerate(geometry):
            if 'lat' not in point or 'lon' not in point:
                continue
            coordinate = {'lat': point['lat'], 'lon': point['lon']}
            if isinstance(nodes, list) and index < len(nodes):
                coordinate['osm_node_id'] = nodes[index]
            coordinates.append(coordinate)
        result = {
            'role': role,
            'coordinates': coordinates,
            'original_coordinate_count': len(coordinates),
        }
        if way_id is not None:
            result['osm_way_id'] = way_id
        return result

    rings = []
    if isinstance(element.get('geometry'), list):
        rings.append(ring(element['geometry'], element.get('nodes')))
    for member in element.get('members') or []:
        if isinstance(member.get('geometry'), list):
            rings.append(
                ring(
                    member['geometry'],
                    member.get('nodes'),
                    member.get('role', ''),
                    member.get('ref'),
                )
            )
    return rings


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
    bounds = element.get('bounds')
    if isinstance(bounds, dict) and all(
        key in bounds for key in ('minlat', 'maxlat', 'minlon', 'maxlon')
    ):
        return {
            'lat': (bounds['minlat'] + bounds['maxlat']) / 2.0,
            'lon': (bounds['minlon'] + bounds['maxlon']) / 2.0,
        }
    points = element_geometry_points(element)
    if points:
        return {
            'lat': sum(point['lat'] for point in points) / len(points),
            'lon': sum(point['lon'] for point in points) / len(points),
        }
    return None


def element_geometry_points(element):
    points = []
    geometry = element.get('geometry')
    if isinstance(geometry, list):
        points.extend(
            point for point in geometry if 'lat' in point and 'lon' in point
        )
    for member in element.get('members') or []:
        member_geometry = member.get('geometry')
        if isinstance(member_geometry, list):
            points.extend(
                point
                for point in member_geometry
                if 'lat' in point and 'lon' in point
            )
    return points


def element_bounds(element):
    bounds = element.get('bounds')
    if isinstance(bounds, dict) and all(
        key in bounds for key in ('minlat', 'maxlat', 'minlon', 'maxlon')
    ):
        return {
            'north': bounds['maxlat'],
            'south': bounds['minlat'],
            'east': bounds['maxlon'],
            'west': bounds['minlon'],
        }
    points = element_geometry_points(element)
    if not points:
        return {}
    return {
        'north': max(point['lat'] for point in points),
        'south': min(point['lat'] for point in points),
        'east': max(point['lon'] for point in points),
        'west': min(point['lon'] for point in points),
    }


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


def haversine_distance_m(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    delta_lat = lat2 - lat1
    delta_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(delta_lat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2) ** 2
    )
    return 6371000 * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def build_osm_context_like_context_gatherer(
    overpass,
    max_linear=40,
    max_coordinates=80,
    center_lat=48.284180,
    center_lon=11.608129,
):
    linear_features = []
    point_features = []
    steps_features = []
    area_features = []
    tree_features = []
    waste_basket_features = []
    mission_target_features = []
    environmental_features = []
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
        if osm_type == 'way' and kind == 'steps':
            steps_features.append(
                {
                    'osm_id': element['id'],
                    'tags': tags,
                    'recommended_use': 'avoid',
                    'coordinates': [
                        {
                            **point,
                            **(
                                {'osm_node_id': element['nodes'][index]}
                                if index < len(element.get('nodes') or []) else {}
                            ),
                        }
                        for index, point in enumerate(element.get('geometry') or [])
                    ],
                }
            )
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
                'tags': tags,
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

        is_mission_target = tags.get('amenity') in {
            'bench',
            'toilets',
            'shelter',
            'drinking_water',
            'bicycle_parking',
            'recycling',
            'shower',
            'bbq',
        } or tags.get('leisure') == 'picnic_table'
        if is_mission_target:
            center = element_center_point(element)
            if center:
                mission_target_features.append(
                    {
                        'osm_id': element['id'],
                        'center': center,
                        'distance_m': haversine_distance_m(
                            center_lat,
                            center_lon,
                            center['lat'],
                            center['lon'],
                        ),
                        'kind': 'mission_target',
                        'tags': tags,
                    }
                )

        is_tree = kind == 'tree'
        is_waste_basket = tags.get('amenity') == 'waste_basket'
        if is_tree or is_waste_basket:
            center = element_center_point(element)
            if center:
                feature = {
                    'osm_id': element['id'],
                    'center': center,
                    'distance_m': haversine_distance_m(
                        center_lat,
                        center_lon,
                        center['lat'],
                        center['lon'],
                    ),
                    'kind': 'tree' if is_tree else 'waste_basket',
                }
                (tree_features if is_tree else waste_basket_features).append(feature)
            if is_tree:
                continue

        if kind == 'environmental_line':
            center = element_center_point(element)
            if center:
                environmental_features.append(
                    {
                        'osm_id': element['id'],
                        'center': center,
                        'distance_m': haversine_distance_m(
                            center_lat,
                            center_lon,
                            center['lat'],
                            center['lon'],
                        ),
                        'kind': kind,
                        'tags': tags,
                    }
                )
            continue

        if kind in ('barrier', 'landmark') and element_center_point(element):
            point_features.append(
                {'label': f'M{point_id}', 'kind': kind, 'tags': tags}
            )
            point_id += 1
            continue

        if kind in ('water', 'avoid_natural', 'area') and is_area_element(element):
            points = element_geometry_points(element)
            if not points:
                center = element_center_point(element)
                points = [center] if center else []
            area_features.append(
                {
                    'label': f'A{area_id}',
                    'kind': kind,
                    'osm_id': element['id'],
                    'tags': tags,
                    'rings': area_rings(element),
                    'distance_m': min(
                        haversine_distance_m(
                            center_lat, center_lon, point['lat'], point['lon']
                        )
                        for point in points
                    ),
                }
            )
            area_id += 1

    tree_features.sort(key=lambda feature: feature['distance_m'])
    waste_basket_features.sort(key=lambda feature: feature['distance_m'])
    mission_target_features.sort(key=lambda feature: feature['distance_m'])
    environmental_features.sort(key=lambda feature: feature['distance_m'])
    skipped_tree = max(0, len(tree_features) - 100)
    skipped_waste_basket = max(0, len(waste_basket_features) - 20)
    skipped_mission_target = max(0, len(mission_target_features) - 40)
    skipped_environmental = max(0, len(environmental_features) - 40)
    tree_features = tree_features[:100]
    waste_basket_features = waste_basket_features[:20]
    mission_target_features = mission_target_features[:40]
    environmental_features = environmental_features[:40]
    area_features.sort(
        key=lambda feature: (
            0 if feature['kind'] == 'water' and feature['tags'].get('name') else 1,
            feature['distance_m'],
        )
    )
    skipped_area = max(0, len(area_features) - max_linear)
    area_features = area_features[:max_linear]
    for index, feature in enumerate(tree_features, 1):
        feature['label'] = f'T{index}'
    for index, feature in enumerate(waste_basket_features, 1):
        feature['label'] = f'B{index}'
    for index, feature in enumerate(mission_target_features, 1):
        feature['label'] = f'MT{index}'
    for index, feature in enumerate(environmental_features, 1):
        feature['label'] = f'E{index}'

    return {
        'linear_features': linear_features,
        'point_features': point_features,
        'steps_features': steps_features,
        'area_features': area_features,
        'tree_features': tree_features,
        'waste_basket_features': waste_basket_features,
        'mission_target_features': mission_target_features,
        'environmental_features': environmental_features,
        'limits': {
            'skipped_linear_features': skipped_linear,
            'skipped_area_features': skipped_area,
            'skipped_tree_features': skipped_tree,
            'skipped_waste_basket_features': skipped_waste_basket,
            'skipped_mission_target_features': skipped_mission_target,
            'skipped_environmental_features': skipped_environmental,
            'truncated': (
                truncated
                or skipped_tree > 0
                or skipped_waste_basket > 0
                or skipped_mission_target > 0
                or skipped_environmental > 0
                or skipped_area > 0
            ),
        },
    }


def test_overpass_query_requests_full_geometry_for_highway_ways():
    source = CONTEXT_GATHERER_NODE.read_text()

    assert ');out geom;' in source
    assert ');out geom center;' not in source


def test_stairs_survive_route_limit_with_tags_and_node_locations():
    source = CONTEXT_GATHERER_NODE.read_text()
    stair = {
        'type': 'way',
        'id': 485792666,
        'nodes': [4785581767, 4785581766],
        'tags': {
            'highway': 'steps', 'step_count': '36', 'ramp': 'no',
            'surface': 'concrete',
        },
        'geometry': [
            {'lat': 48.2848323, 'lon': 11.6070147},
            {'lat': 48.2848002, 'lon': 11.6070885},
        ],
    }

    context = build_osm_context_like_context_gatherer(
        {'elements': [stair]}, max_linear=0, max_coordinates=2
    )

    assert r'[\"highway\"=\"steps\"]' in source
    assert context['linear_features'] == []
    assert context['steps_features'] == [
        {
            'osm_id': 485792666,
            'tags': stair['tags'],
            'recommended_use': 'avoid',
            'coordinates': [
                {**point, 'osm_node_id': node_id}
                for point, node_id in zip(stair['geometry'], stair['nodes'])
            ],
        }
    ]


def test_general_areas_keep_boundary_nodes_and_tags_for_payload_generation():
    source = CONTEXT_GATHERER_NODE.read_text()
    pitch = {
        'type': 'way',
        'id': 1127832739,
        'nodes': [10311392033, 10311392032, 10311392031, 10311392030, 10311392033],
        'tags': {'leisure': 'pitch', 'sport': 'volleyball', 'surface': 'sand', 'lit': 'no'},
        'geometry': [
            {'lat': 48.2845565, 'lon': 11.6074886},
            {'lat': 48.2846475, 'lon': 11.6075825},
            {'lat': 48.2845406, 'lon': 11.6078163},
            {'lat': 48.2844496, 'lon': 11.6077224},
            {'lat': 48.2845565, 'lon': 11.6074886},
        ],
    }
    parking = {
        'type': 'way',
        'id': 2,
        'nodes': [10, 11, 12, 10],
        'tags': {'amenity': 'parking', 'surface': 'gravel'},
        'geometry': [
            {'lat': 48.2847, 'lon': 11.6080},
            {'lat': 48.2848, 'lon': 11.6080},
            {'lat': 48.2848, 'lon': 11.6081},
            {'lat': 48.2847, 'lon': 11.6080},
        ],
    }
    relation = {
        'type': 'relation',
        'id': 3,
        'tags': {'type': 'multipolygon', 'landuse': 'grass'},
        'members': [
            {
                'type': 'way',
                'ref': 99,
                'role': 'inner',
                'geometry': [
                    {'lat': 48.2841, 'lon': 11.6081},
                    {'lat': 48.2842, 'lon': 11.6081},
                    {'lat': 48.2841, 'lon': 11.6081},
                ],
            }
        ],
    }
    far_areas = [
        {
            'type': 'way',
            'id': 100 + index,
            'tags': {'landuse': 'grass'},
            'geometry': [
                {'lat': 48.29 + index * 0.00001, 'lon': 11.61},
                {'lat': 48.2901 + index * 0.00001, 'lon': 11.61},
                {'lat': 48.2901 + index * 0.00001, 'lon': 11.6101},
                {'lat': 48.29 + index * 0.00001, 'lon': 11.61},
            ],
        }
        for index in range(45)
    ]

    context = build_osm_context_like_context_gatherer(
        {'elements': far_areas + [pitch, parking, relation]}, max_coordinates=2
    )

    for key in ('landuse', 'leisure', 'natural', 'amenity', 'tourism'):
        assert f'[\\"{key}\\"]' in source
    assert r'[\"surface\"][!\"highway\"]' in source
    assert 'std::numeric_limits<int>::max(), rings_truncated' in source
    assert 'coordinate["osm_node_id"]' in source
    assert len(context['area_features']) == 40
    assert context['limits']['skipped_area_features'] == 8
    pitch_feature = next(
        item for item in context['area_features'] if item['osm_id'] == pitch['id']
    )
    assert pitch_feature['tags'] == pitch['tags']
    assert pitch_feature['rings'][0]['coordinates'] == [
        {**point, 'osm_node_id': node_id}
        for point, node_id in zip(pitch['geometry'], pitch['nodes'])
    ]
    assert any(item['osm_id'] == parking['id'] for item in context['area_features'])
    relation_feature = next(
        item for item in context['area_features'] if item['osm_id'] == relation['id']
    )
    assert relation_feature['rings'][0]['role'] == 'inner'
    assert relation_feature['rings'][0]['osm_way_id'] == 99
    assert relation_feature['rings'][0]['coordinates'] == relation['members'][0]['geometry']


def test_osm_context_keeps_nearest_trees_and_waste_baskets():
    source = CONTEXT_GATHERER_NODE.read_text()
    center_lat = 48.284180
    center_lon = 11.608129
    trees = [
        {
            'type': 'node',
            'id': index,
            'lat': center_lat + index * 0.00001,
            'lon': center_lon,
            'tags': {'natural': 'tree'},
        }
        for index in range(1, 106)
    ]
    bins = [
        {
            'type': 'node',
            'id': 1000 + index,
            'lat': center_lat,
            'lon': center_lon + index * 0.00001,
            'tags': {'amenity': 'waste_basket'},
        }
        for index in range(1, 26)
    ]

    context = build_osm_context_like_context_gatherer(
        {'elements': list(reversed(trees + bins))},
        center_lat=center_lat,
        center_lon=center_lon,
    )

    assert '[\\"natural\\"=\\"tree\\"]' in source
    assert len(context['tree_features']) == 100
    assert len(context['waste_basket_features']) == 20
    assert context['tree_features'][0]['osm_id'] == 1
    assert context['tree_features'][-1]['osm_id'] == 100
    assert context['waste_basket_features'][0]['osm_id'] == 1001
    assert context['waste_basket_features'][-1]['osm_id'] == 1020
    assert context['limits']['skipped_tree_features'] == 5
    assert context['limits']['skipped_waste_basket_features'] == 5


def test_osm_context_requests_targets_traversability_obstacles_and_environment():
    source = CONTEXT_GATHERER_NODE.read_text()
    target_values = {
        'bench',
        'toilets',
        'shelter',
        'drinking_water',
        'bicycle_parking',
        'recycling',
        'shower',
        'bbq',
        'picnic_table',
    }
    elements = [
        {
            'type': 'node',
            'id': 1,
            'lat': 48.2842,
            'lon': 11.6082,
            'tags': {'amenity': 'bench'},
        },
        {
            'type': 'node',
            'id': 2,
            'lat': 48.2843,
            'lon': 11.6083,
            'tags': {'leisure': 'picnic_table'},
        },
        {
            'type': 'way',
            'id': 3,
            'tags': {
                'highway': 'path',
                'surface': 'compacted',
                'smoothness': 'good',
                'width': '2',
                'incline': '3%',
                'trail_visibility': 'excellent',
                'access': 'yes',
            },
            'geometry': [
                {'lat': 48.2841, 'lon': 11.6080},
                {'lat': 48.2842, 'lon': 11.6081},
            ],
        },
        {
            'type': 'way',
            'id': 4,
            'tags': {
                'barrier': 'fence',
                'access': 'private',
                'locked': 'yes',
                'maxwidth:physical': '1.5',
            },
            'geometry': [
                {'lat': 48.2840, 'lon': 11.6080},
                {'lat': 48.2840, 'lon': 11.6082},
            ],
        },
        {
            'type': 'way',
            'id': 5,
            'tags': {'natural': 'tree_row'},
            'geometry': [
                {'lat': 48.2844, 'lon': 11.6080},
                {'lat': 48.2845, 'lon': 11.6081},
            ],
        },
        {
            'type': 'way',
            'id': 6,
            'tags': {'waterway': 'stream'},
            'geometry': [
                {'lat': 48.2845, 'lon': 11.6082},
                {'lat': 48.2846, 'lon': 11.6083},
            ],
        },
        {
            'type': 'way',
            'id': 7,
            'tags': {'natural': 'wetland', 'wetland': 'reedbed'},
            'geometry': [
                {'lat': 48.2846, 'lon': 11.6084},
                {'lat': 48.2847, 'lon': 11.6085},
                {'lat': 48.2846, 'lon': 11.6086},
                {'lat': 48.2846, 'lon': 11.6084},
            ],
        },
    ]

    context = build_osm_context_like_context_gatherer({'elements': elements})

    assert all(value in source for value in target_values)
    assert r'[\"waterway\"=\"stream\"]' in source
    assert r'[\"barrier\"]' in source
    assert 'tree_row|tree_group|wetland|coastline' in source
    assert len(context['mission_target_features']) == 2
    assert {item['osm_id'] for item in context['environmental_features']} == {5, 6}
    assert any(item['kind'] == 'avoid_natural' for item in context['area_features'])
    assert context['linear_features'][0]['tags']['surface'] == 'compacted'
    assert context['linear_features'][0]['tags']['trail_visibility'] == 'excellent'
    obstacle = next(item for item in context['point_features'] if item['kind'] == 'barrier')
    assert obstacle['tags']['locked'] == 'yes'
    assert obstacle['tags']['maxwidth:physical'] == '1.5'


def test_overpass_request_handles_public_server_load_shedding():
    source = CONTEXT_GATHERER_NODE.read_text()

    assert 'overpass_timeout_sec_ + 20.0' in source
    assert 'http_code == 429' in source
    assert 'std::this_thread::sleep_for(15s);' in source
    assert 'response.contains("remark")' in source
    assert 'error_message += ": " + response_text.substr(0, 500);' in source


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
                    {'lat': 48.20350, 'lon': 11.64640},
                    {'lat': 48.20350, 'lon': 11.64600},
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


def test_relation_member_geometry_preserves_lake_extent():
    relation = {
        'type': 'relation',
        'id': 10867274,
        'bounds': {
            'minlat': 48.2796683,
            'minlon': 11.5941991,
            'maxlat': 48.2886447,
            'maxlon': 11.6084286,
        },
        'members': [
            {
                'type': 'way',
                'role': 'outer',
                'geometry': [
                    {'lat': 48.2796683, 'lon': 11.5941991},
                    {'lat': 48.2886447, 'lon': 11.6084286},
                ],
            }
        ],
        'tags': {'name': 'Hollerner See', 'natural': 'water'},
    }

    assert element_center_point(relation) == {
        'lat': pytest.approx(48.2841565),
        'lon': pytest.approx(11.60131385),
    }
    assert element_bounds(relation) == {
        'north': 48.2886447,
        'south': 48.2796683,
        'east': 11.6084286,
        'west': 11.5941991,
    }
    assert len(element_geometry_points(relation)) == 2


def test_lake_context_defaults_cover_hollerner_and_use_official_failover():
    source = CONTEXT_GATHERER_NODE.read_text()
    params = yaml.safe_load(
        (
            REPO_ROOT
            / 'src'
            / 'context_gatherer'
            / 'config'
            / 'context_gatherer_params.yaml'
        ).read_text()
    )['context_gatherer']['ros__parameters']

    assert 'return 1200.0;' in source
    assert params['overpass_timeout_sec'] == 25.0
    assert params['overpass_max_linear_features'] == 40
    assert (
        'https://gall.openstreetmap.de/api/interpreter'
        in params['overpass_fallback_endpoints']
    )
    assert 'lake_route_relevance' in source


def test_overpass_timeout_retries_with_stairs_and_water_but_marks_reduced_scope():
    source = CONTEXT_GATHERER_NODE.read_text()
    query = source.split('json fetch_overpass_context(', 1)[1].split(
        'json get_or_fetch_overpass_context(', 1
    )[0]

    assert 'server_timed_out = server_timed_out || http_code == 504' in query
    assert 'fetch_overpass_context(lat, lon, radius_m, true)' in query
    assert query.index(r'[\"highway\"=\"steps\"]') < query.index('if (!reduced_query)')
    assert query.index(r'[\"barrier\"]') < query.index('if (!reduced_query)')
    assert r'[\"natural\"=\"water\"]' in query
    assert '"safety_critical" : "full"' in query
    assert '{"query_scope", overpass.value("__query_scope", "full")}' in source


def test_robot_pose_uses_configured_sim_odometry_topic():
    source = CONTEXT_GATHERER_NODE.read_text()
    params = yaml.safe_load(
        (
            REPO_ROOT
            / 'src'
            / 'context_gatherer'
            / 'config'
            / 'context_gatherer_params.yaml'
        ).read_text()
    )['context_gatherer']['ros__parameters']

    assert 'declare_parameter<std::string>("odom_topic", "/target/odometry/fused")' in source
    assert 'create_subscription<nav_msgs::msg::Odometry>(\n      odom_topic_' in source
    assert params['odom_topic'] == '/target/odometry/fused'


def test_gps_subscription_uses_sensor_data_qos():
    source = CONTEXT_GATHERER_NODE.read_text()

    assert (
        'create_subscription<sensor_msgs::msg::NavSatFix>(\n'
        '        gps_fix_topic_, rclcpp::SensorDataQoS()'
    ) in source


def test_rgb_context_auto_selects_sweep_camera():
    source = CONTEXT_GATHERER_NODE.read_text()
    params = yaml.safe_load(
        (
            REPO_ROOT
            / 'src'
            / 'context_gatherer'
            / 'config'
            / 'context_gatherer_params.yaml'
        ).read_text()
    )['context_gatherer']['ros__parameters']
    assert '"/okvis/rgb2/image_raw", 10' in source
    assert params['rgb360_sweep_camera_topic'] == 'auto'


def test_required_gps_fix_is_validated_and_aborts_with_reason():
    source = CONTEXT_GATHERER_NODE.read_text()
    params = yaml.safe_load(
        (
            REPO_ROOT
            / 'src'
            / 'context_gatherer'
            / 'config'
            / 'context_gatherer_params.yaml'
        ).read_text()
    )['context_gatherer']['ros__parameters']

    assert 'snapshot.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX' in source
    assert 'snapshot.latitude == 0.0 && snapshot.longitude == 0.0' in source
    assert 'snapshot.age_sec > gps_fix_max_age_sec_' in source
    assert 'Required GPS_FIX context unavailable on ' in source
    assert 'goal_handle->abort(result);' in source
    assert params['gps_fix_max_age_sec'] == 5.0
