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
