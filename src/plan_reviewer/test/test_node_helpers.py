import json
import sys
import types
from types import SimpleNamespace


def _install_ros_stubs() -> None:
    if 'rclpy' not in sys.modules:
        sys.modules['rclpy'] = types.ModuleType('rclpy')
    if 'rclpy.node' not in sys.modules:
        node_module = types.ModuleType('rclpy.node')
        node_module.Node = object
        sys.modules['rclpy.node'] = node_module
    if 'rclpy.executors' not in sys.modules:
        executor_module = types.ModuleType('rclpy.executors')
        executor_module.MultiThreadedExecutor = object
        sys.modules['rclpy.executors'] = executor_module
    srv_module = sys.modules.get('gen_bt_interfaces.srv')
    if srv_module is None:
        srv_module = types.ModuleType('gen_bt_interfaces.srv')
        sys.modules['gen_bt_interfaces.srv'] = srv_module

    class DummyReviewPlan:
        class Request:
            pass

        class Response:
            PASS = 0
            WARN = 1
            REJECT = 2
            ERROR = 3

    srv_module.ReviewPlan = DummyReviewPlan


_install_ros_stubs()

from plan_reviewer.node import REVIEW_PROMPT_TEMPLATE, PlanReviewerNode


def test_review_input_includes_context_contract_and_image_uri():
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    request = SimpleNamespace(
        session_id='session-7',
        subtree_id='navigate.xml',
        user_command='inspect the loading bay',
        operator_feedback='avoid the north gate',
        payload_json=json.dumps({'waypoints': '1.0,2.0,0.0'}),
        context_snapshot_json=json.dumps(
            {
                'OSM_CONTEXT': {'linear_features': [{'name': 'service road'}]},
                'SATELLITE_MAP': {'uri': '/tmp/satellite.png', 'map_metadata': {'bounds': {}}},
                'ROBOT_POSE': {'x': 0.0, 'y': 0.0},
            }
        ),
        attachment_uris=['/tmp/satellite.png'],
        subtree_contract_json=json.dumps({'waypoints': {'type': 'string'}}),
    )
    render_info = {
        'image_uri': 'file:///tmp/context_gatherer/plan_review.png',
        'map_available': True,
        'normalized_plan': {'map_preview': {'map_metadata': {'bounds': {'north': 1}}}},
        'waypoints': [{'index': 1, 'x': 1.0, 'y': 2.0}],
        'waypoint_pixels': [{'index': 1, 'pixel_x': 10, 'pixel_y': 20, 'in_bounds': True}],
        'render_warnings': [],
    }

    review_input = node._build_review_input(request, render_info)

    assert review_input['mission_text'] == 'inspect the loading bay'
    assert review_input['payload_json']['waypoints'] == '1.0,2.0,0.0'
    assert review_input['subtree_contract_json']['waypoints']['type'] == 'string'
    assert review_input['context_focus']['OSM_CONTEXT']['linear_features'][0]['name'] == 'service road'
    assert review_input['review_image_uri'].endswith('plan_review.png')


def test_prompt_mentions_move_to_map_frame_and_strict_json():
    prompt = REVIEW_PROMPT_TEMPLATE.format(review_input_json='{}')

    assert 'strict JSON only' in prompt
    assert 'MoveTo behavior trees execute x,y,yaw map-frame waypoints' in prompt
    assert 'OSM_CONTEXT.linear_features' in prompt


def test_fallback_review_rejects_explore_waypoint_outside_area_polygon():
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    review_input = {'map_available': True}
    render_info = {
        'waypoint_pixels': [
            {'index': 1, 'pixel_x': 10.0, 'pixel_y': 20.0, 'in_bounds': True}
        ],
        'waypoint_area_checks': [{'index': 1, 'in_area': False}],
    }

    review = node._fallback_review(review_input, render_info)

    assert review['status'] == 'reject'
    assert review['recommended_action'] == 'regenerate'
    assert review['findings'][0]['category'] == 'mission_fulfillment'
    assert review['findings'][0]['waypoint_indices'] == [1]
