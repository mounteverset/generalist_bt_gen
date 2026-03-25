import json

from user_interface.pending_plan_preview import extract_waypoints, normalize_pending_plan


def test_extract_waypoints_from_semicolon_string():
    payload = {'waypoints': '1.0,2.0,0.0; 3.5,4.25,1.57'}

    waypoints = extract_waypoints(payload)

    assert len(waypoints) == 2
    assert waypoints[0]['index'] == 1
    assert waypoints[0]['x'] == 1.0
    assert waypoints[0]['y'] == 2.0
    assert waypoints[1]['yaw_rad'] == 1.57


def test_normalize_pending_plan_builds_map_preview():
    plan = {
        'session_id': 'webui-123',
        'tree_id': 'demo_tree.xml',
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
        'tree_id': 'demo_tree.xml',
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
