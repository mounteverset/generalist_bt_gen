import json

from user_interface.pending_plan_preview import (
    extract_area_polygon,
    extract_frontiers,
    extract_waypoints,
    normalize_pending_plan,
)


def test_extract_waypoints_from_semicolon_string():
    payload = {'waypoints': '1.0,2.0,0.0; 3.5,4.25,1.57'}

    waypoints = extract_waypoints(payload)

    assert len(waypoints) == 2
    assert waypoints[0]['index'] == 1
    assert waypoints[0]['x'] == 1.0
    assert waypoints[0]['y'] == 2.0
    assert waypoints[1]['yaw_rad'] == 1.57


def test_extract_explore_area_polygon_and_frontiers_from_payload():
    payload = {
        'area_polygon': '0.0,0.0; 5.0,0.0; 5.0,4.0; 0.0,4.0',
        'frontiers': '1.0,1.0,0.0; 4.0,3.0,1.57',
    }

    polygon = extract_area_polygon(payload)
    frontiers = extract_frontiers(payload)

    assert len(polygon) == 4
    assert polygon[2]['x'] == 5.0
    assert polygon[2]['y'] == 4.0
    assert len(frontiers) == 2
    assert frontiers[1]['yaw_rad'] == 1.57


def test_normalize_pending_plan_builds_map_preview():
    plan = {
        'session_id': 'webui-123',
        'tree_id': 'temperature_logging.xml',
        'summary': 'Visit generated waypoints.',
        'payload_json': json.dumps({'waypoints': '1.0,2.0,0.0; 3.0,4.0,1.57'}),
        'context_snapshot_json': json.dumps(
            {
                'ANNOTATED_SLAM_MAP_IMAGE': {
                    'uri': 'file:///tmp/context_gatherer/annotated_slam_map.png',
                    'width': 400,
                    'height': 300,
                    'frame_id': 'map',
                    'resolution': 0.05,
                    'map_metadata': {
                        'width': 400,
                        'height': 300,
                        'resolution_m_per_px': 0.05,
                        'origin': {'x': -10.0, 'y': -5.0, 'yaw_rad': 0.0},
                        'robot_pose': {
                            'available': True,
                            'x': 0.5,
                            'y': 1.0,
                            'yaw_rad': 0.2,
                        },
                    },
                }
            }
        ),
        'attachment_uris': ['file:///tmp/context_gatherer/annotated_slam_map.png'],
    }

    normalized = normalize_pending_plan(plan, lambda uri: f'/artifacts?uri={uri}')

    assert normalized is not None
    assert normalized['waypoint_count'] == 2
    assert normalized['waypoints'][1]['index'] == 2
    assert normalized['payload_object']['waypoints'].startswith('1.0,2.0')
    assert normalized['map_preview']['image_url'].startswith('/artifacts?uri=')
    assert normalized['map_preview']['frame_id'] == 'map'
    assert normalized['map_preview']['map_metadata']['origin']['x'] == -10.0
    assert len(normalized['map_previews']) == 1
    assert normalized['map_previews'][0]['frame_id'] == 'map'


def test_extract_waypoints_from_lat_lon_dicts():
    payload = {
        'waypoints': [
            {'lat': 48.20286111111111, 'lon': 11.64486111111111, 'yaw': 0.25},
            {'latitude': 48.2025, 'longitude': 11.6452, 'heading': 1.57},
        ]
    }

    waypoints = extract_waypoints(payload)

    assert len(waypoints) == 2
    assert waypoints[0]['lat'] == 48.20286111111111
    assert waypoints[0]['lon'] == 11.64486111111111
    assert waypoints[1]['lat'] == 48.2025
    assert waypoints[1]['lon'] == 11.6452
    assert waypoints[1]['yaw_rad'] == 1.57


def test_normalize_pending_plan_builds_satellite_map_preview_with_lat_lon_waypoints():
    plan = {
        'session_id': 'webui-geo-123',
        'tree_id': 'temperature_logging.xml',
        'summary': 'Visit generated geographic waypoints.',
        'payload_json': json.dumps(
            {
                'waypoints': (
                    '48.20286111111111,11.64486111111111,0.0; '
                    '48.20251111111111,11.64511111111111,1.57'
                )
            }
        ),
        'context_snapshot_json': json.dumps(
            {
                'SATELLITE_MAP': {
                    'uri': 'file:///tmp/context_gatherer/satellite_map.png',
                    'zoom': 18,
                    'map_metadata': {
                        'image_width_px': 512,
                        'image_height_px': 512,
                        'bounds': {
                            'north': 48.2032,
                            'south': 48.2022,
                            'east': 11.6454,
                            'west': 11.6444,
                        },
                        'meters_per_px': 0.75,
                        'robot_location': {
                            'lat': 48.20286111111111,
                            'lon': 11.64486111111111,
                            'label': 'Robot',
                        },
                    },
                }
            }
        ),
        'attachment_uris': ['file:///tmp/context_gatherer/satellite_map.png'],
    }

    normalized = normalize_pending_plan(plan, lambda uri: f'/artifacts?uri={uri}')

    assert normalized is not None
    assert normalized['waypoint_count'] == 2
    assert normalized['map_preview']['source_key'] == 'SATELLITE_MAP'
    assert normalized['map_preview']['coordinate_mode'] == 'geographic'
    assert normalized['map_preview']['zoom'] == 18
    assert normalized['waypoints'][0]['lat'] == 48.20286111111111
    assert normalized['waypoints'][0]['lon'] == 11.64486111111111
    assert normalized['waypoints'][0]['coordinate_mode'] == 'geographic'


def test_satellite_preview_prefers_detail_artifact_containing_all_waypoints():
    plan = {
        'session_id': 'webui-geo-detail',
        'tree_id': 'temperature_logging.xml',
        'summary': 'Visit generated geographic waypoints.',
        'payload_json': json.dumps(
            {
                'waypoints': (
                    '48.20286,11.64486,0.0; '
                    '48.20284,11.64490,1.57'
                )
            }
        ),
        'context_snapshot_json': json.dumps(
            {
                'SATELLITE_MAP': {
                    'uri': 'file:///tmp/context_gatherer/satellite_overview.png',
                    'zoom': 15,
                    'map_metadata': {
                        'image_width_px': 768,
                        'image_height_px': 768,
                        'bounds': {
                            'north': 48.206,
                            'south': 48.200,
                            'east': 11.650,
                            'west': 11.640,
                        },
                        'meters_per_px': 3.0,
                    },
                    'map_artifacts': [
                        {
                            'role': 'overview',
                            'uri': 'file:///tmp/context_gatherer/satellite_overview.png',
                            'zoom': 15,
                            'map_metadata': {
                                'image_width_px': 768,
                                'image_height_px': 768,
                                'bounds': {
                                    'north': 48.206,
                                    'south': 48.200,
                                    'east': 11.650,
                                    'west': 11.640,
                                },
                                'meters_per_px': 3.0,
                            },
                        },
                        {
                            'role': 'detail_start',
                            'uri': 'file:///tmp/context_gatherer/satellite_detail.png',
                            'zoom': 18,
                            'map_metadata': {
                                'image_width_px': 768,
                                'image_height_px': 768,
                                'bounds': {
                                    'north': 48.2030,
                                    'south': 48.2027,
                                    'east': 11.6451,
                                    'west': 11.6447,
                                },
                                'meters_per_px': 0.4,
                            },
                        },
                    ],
                }
            }
        ),
        'attachment_uris': [],
    }

    normalized = normalize_pending_plan(plan, lambda uri: f'/artifacts?uri={uri}')

    assert normalized is not None
    assert normalized['map_preview']['zoom'] == 18
    assert normalized['map_preview']['role'] == 'detail_start'
    assert normalized['map_preview']['uri'].endswith('satellite_detail.png')
    assert [preview['zoom'] for preview in normalized['map_previews']] == [18, 15]
    assert normalized['map_previews'][0]['role'] == 'detail_start'
    assert normalized['map_previews'][1]['role'] == 'overview'


def test_normalize_explore_preview_uses_geo_overlays_for_satellite_map():
    plan = {
        'session_id': 'webui-explore-geo',
        'tree_id': 'explore_area.xml',
        'summary': 'Explore the requested polygon.',
        'payload_json': json.dumps(
            {
                'waypoints': '1.0,1.0,0.0; 2.0,2.0,1.57',
                'area_polygon': '0.0,0.0; 5.0,0.0; 5.0,5.0; 0.0,5.0',
                'frontiers': '1.0,1.0,0.0; 4.0,4.0,1.57',
                'area_polygon_geo': (
                    '48.2027,11.6447; 48.2027,11.6451; '
                    '48.2030,11.6451; 48.2030,11.6447'
                ),
                'frontiers_geo': '48.2028,11.6448; 48.2029,11.6450',
            }
        ),
        'context_snapshot_json': json.dumps(
            {
                'SATELLITE_MAP': {
                    'uri': 'file:///tmp/context_gatherer/satellite_detail.png',
                    'zoom': 18,
                    'map_metadata': {
                        'image_width_px': 768,
                        'image_height_px': 768,
                        'bounds': {
                            'north': 48.2031,
                            'south': 48.2026,
                            'east': 11.6452,
                            'west': 11.6446,
                        },
                        'meters_per_px': 0.4,
                    },
                }
            }
        ),
        'attachment_uris': [],
    }

    normalized = normalize_pending_plan(plan, lambda uri: f'/artifacts?uri={uri}')

    assert normalized is not None
    assert normalized['map_preview']['source_key'] == 'SATELLITE_MAP'
    assert normalized['map_preview']['coordinate_mode'] == 'geographic'
    assert normalized['waypoint_count'] == 2
    assert normalized['area_polygon_count'] == 4
    assert normalized['frontier_count'] == 2
    assert normalized['area_polygon'][0]['lat'] == 48.2027
    assert normalized['area_polygon'][0]['coordinate_mode'] == 'geographic'
    assert normalized['frontiers'][1]['lon'] == 11.6450


def test_satellite_preview_falls_back_when_detail_misses_waypoints():
    plan = {
        'session_id': 'webui-geo-overview',
        'tree_id': 'temperature_logging.xml',
        'summary': 'Visit generated geographic waypoints.',
        'payload_json': json.dumps(
            {
                'waypoints': (
                    '48.20286,11.64486,0.0; '
                    '48.20450,11.64800,1.57'
                )
            }
        ),
        'context_snapshot_json': json.dumps(
            {
                'SATELLITE_MAP': {
                    'uri': 'file:///tmp/context_gatherer/satellite_overview.png',
                    'zoom': 15,
                    'map_metadata': {
                        'image_width_px': 768,
                        'image_height_px': 768,
                        'bounds': {
                            'north': 48.206,
                            'south': 48.200,
                            'east': 11.650,
                            'west': 11.640,
                        },
                        'meters_per_px': 3.0,
                    },
                    'map_artifacts': [
                        {
                            'role': 'detail_start',
                            'uri': 'file:///tmp/context_gatherer/satellite_detail.png',
                            'zoom': 18,
                            'map_metadata': {
                                'image_width_px': 768,
                                'image_height_px': 768,
                                'bounds': {
                                    'north': 48.2030,
                                    'south': 48.2027,
                                    'east': 11.6451,
                                    'west': 11.6447,
                                },
                                'meters_per_px': 0.4,
                            },
                        },
                    ],
                }
            }
        ),
        'attachment_uris': [],
    }

    normalized = normalize_pending_plan(plan, lambda uri: f'/artifacts?uri={uri}')

    assert normalized is not None
    assert normalized['map_preview']['zoom'] == 15
    assert normalized['map_preview']['uri'].endswith('satellite_overview.png')
    assert len(normalized['map_previews']) == 2
