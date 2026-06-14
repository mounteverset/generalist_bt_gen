import json
from pathlib import Path

from PIL import Image

from plan_reviewer.renderer import render_plan_review_image


def test_render_satellite_waypoints_to_pixels(tmp_path):
    image_path = tmp_path / 'satellite_map.png'
    Image.new('RGB', (100, 100), 'white').save(image_path)
    context = {
        'SATELLITE_MAP': {
            'uri': image_path.as_uri(),
            'map_metadata': {
                'image_width_px': 100,
                'image_height_px': 100,
                'bounds': {
                    'north': 10.0,
                    'south': 0.0,
                    'east': 20.0,
                    'west': 10.0,
                },
            },
        }
    }
    payload = {
        'waypoints': [
            {'lat': 9.0, 'lon': 11.0, 'yaw_rad': 0.0},
            {'lat': 5.0, 'lon': 15.0, 'yaw_rad': 0.0},
        ]
    }

    result = render_plan_review_image(
        session_id='session-1',
        subtree_id='navigate.xml',
        user_command='visit two points',
        payload_json=json.dumps(payload),
        context_snapshot_json=json.dumps(context),
        attachment_uris=[image_path.as_uri()],
        output_directory=tmp_path,
    )

    assert result['map_available'] is True
    assert Path(result['image_path']).exists()
    assert result['image_path'].startswith(str(tmp_path))
    assert result['waypoint_pixels'][0]['pixel_x'] == 10.0
    assert result['waypoint_pixels'][0]['pixel_y'] == 10.0
    assert result['waypoint_pixels'][1]['pixel_x'] == 50.0
    assert result['waypoint_pixels'][1]['pixel_y'] == 50.0
    assert all(item['in_bounds'] for item in result['waypoint_pixels'])


def test_out_of_bounds_waypoint_is_reported(tmp_path):
    image_path = tmp_path / 'satellite_map.png'
    Image.new('RGB', (100, 100), 'white').save(image_path)
    context = {
        'SATELLITE_MAP': {
            'uri': str(image_path),
            'map_metadata': {
                'image_width_px': 100,
                'image_height_px': 100,
                'bounds': {
                    'north': 10.0,
                    'south': 0.0,
                    'east': 20.0,
                    'west': 10.0,
                },
            },
        }
    }
    payload = {'waypoints': [{'lat': 50.0, 'lon': 50.0, 'yaw_rad': 0.0}]}

    result = render_plan_review_image(
        session_id='session-1',
        subtree_id='navigate.xml',
        user_command='visit one point',
        payload_json=json.dumps(payload),
        context_snapshot_json=json.dumps(context),
        attachment_uris=[],
        output_directory=tmp_path,
    )

    assert result['map_available'] is True
    assert result['waypoint_pixels'][0]['in_bounds'] is False
    assert 'outside' in result['waypoint_pixels'][0]['reason']


def test_missing_map_returns_json_only_warning(tmp_path):
    result = render_plan_review_image(
        session_id='session-1',
        subtree_id='navigate.xml',
        user_command='visit one point',
        payload_json=json.dumps({'waypoints': '1.0,2.0,0.0'}),
        context_snapshot_json='{}',
        attachment_uris=[],
        output_directory=tmp_path,
    )

    assert result['map_available'] is False
    assert result['image_uri'] == ''
    assert result['render_warnings']


def test_render_explore_area_overlays_and_flags_outside_waypoint(tmp_path):
    image_path = tmp_path / 'slam_map.png'
    Image.new('RGB', (100, 100), 'white').save(image_path)
    context = {
        'ANNOTATED_SLAM_MAP_IMAGE': {
            'uri': image_path.as_uri(),
            'width': 100,
            'height': 100,
            'frame_id': 'map',
            'map_metadata': {
                'width': 100,
                'height': 100,
                'resolution_m_per_px': 1.0,
                'origin': {'x': 0.0, 'y': 0.0, 'yaw_rad': 0.0},
                'robot_pose': {'x': 1.0, 'y': 1.0, 'yaw_rad': 0.0},
            },
        }
    }
    payload = {
        'waypoints': '2.0,2.0,0.0; 20.0,20.0,1.57',
        'area_polygon': '0.0,0.0; 10.0,0.0; 10.0,10.0; 0.0,10.0',
        'frontiers': '2.0,2.0,0.0; 8.0,8.0,1.57',
    }

    result = render_plan_review_image(
        session_id='session-explore',
        subtree_id='explore_area.xml',
        user_command='explore the polygon',
        payload_json=json.dumps(payload),
        context_snapshot_json=json.dumps(context),
        attachment_uris=[image_path.as_uri()],
        output_directory=tmp_path,
    )

    assert result['map_available'] is True
    assert len(result['area_polygon_pixels']) == 4
    assert len(result['frontier_pixels']) == 2
    assert result['waypoint_area_checks'][0]['in_area'] is True
    assert result['waypoint_area_checks'][1]['in_area'] is False
