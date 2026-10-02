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
    if 'rclpy.parameter' not in sys.modules:
        parameter_module = types.ModuleType('rclpy.parameter')

        class DummyParameter:
            class Type:
                STRING_ARRAY = object()

        parameter_module.Parameter = DummyParameter
        sys.modules['rclpy.parameter'] = parameter_module
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

import plan_reviewer.node as node_module
from plan_reviewer.node import REVIEW_PROMPT_TEMPLATE, PlanReviewerNode


def test_evaluation_audit_preserves_review_prompt_and_response(tmp_path):
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    node._evaluation_evidence_root = tmp_path
    node._provider = 'openrouter'
    node._model_name = 'google/gemma'
    node._max_output_tokens = 4096
    node._openrouter_provider_only = ['darkbloom']
    node._openrouter_allow_fallbacks = True
    node._openrouter_seed = 42
    node._openrouter_service_tier = 'flex'

    audit = node._audit_review_request('E1-C1-P2-method3-gemma-r1', 'review prompt', '')
    node._audit_review_response(
        audit,
        result=SimpleNamespace(
            content='{"status":"pass"}',
            usage_metadata={'input_tokens': 12},
            response_metadata={
                'model': 'google/gemma',
                'openrouter_provider': 'Darkbloom',
                'openrouter_service_tier': 'flex',
                'openrouter_cost_usd': 0.0045,
            },
        ),
    )

    files = sorted(
        (tmp_path / 'E1-C1-P2-method3-gemma-r1' / 'llm_calls').glob('*.json')
    )
    request = json.loads(files[0].read_text())
    response = json.loads(files[1].read_text())
    assert request['prompt'] == 'review prompt'
    assert request['requested_provider_only'] == ['darkbloom']
    assert request['requested_service_tier'] == 'flex'
    assert request['seed'] == 42
    assert request['cache_disabled_requested'] is True
    assert request['max_output_tokens'] == 4096
    assert request['reasoning'] is None
    assert response['raw_content'] == '{"status":"pass"}'
    assert response['usage_metadata']['input_tokens'] == 12
    assert response['returned_provider'] == 'Darkbloom'
    assert response['provider_verified'] is True
    assert response['returned_service_tier'] == 'flex'
    assert response['service_tier_verified'] is True
    assert response['cost_usd'] == 0.0045


def test_evaluation_audit_preserves_review_error_details(tmp_path):
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    node._evaluation_evidence_root = tmp_path
    node._provider = 'openrouter'
    node._model_name = 'google/gemma'

    class ErrorResponse:
        status_code = 502
        headers = {'content-type': 'application/json', 'x-request-id': 'req-456'}

        @property
        def text(self):
            raise RuntimeError('response not read')

    error = RuntimeError('provider failed')
    error.response = ErrorResponse()
    audit = node._audit_review_request('pilot-r5', 'review prompt', '')
    node._audit_review_response(audit, error=node._exception_details(error))

    path = next((tmp_path / 'pilot-r5' / 'llm_calls').glob('*_response.json'))
    saved = json.loads(path.read_text())
    assert saved['error']['status_code'] == 502
    assert saved['error']['body'].startswith('<unavailable: RuntimeError:')
    assert saved['error']['response_headers']['x-request-id'] == 'req-456'


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
                'FIND_ANYTHING': {
                    'query': 'loading bay sign',
                    'locations': [{'frame_id': 'target/map', 'point': {'x': 1.0, 'y': 2.0}}],
                },
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
        'object_locations': [{'index': 1, 'query': 'loading bay sign', 'x': 1.0, 'y': 2.0}],
        'waypoint_pixels': [{'index': 1, 'pixel_x': 10, 'pixel_y': 20, 'in_bounds': True}],
        'object_location_pixels': [
            {'index': 1, 'pixel_x': 10, 'pixel_y': 20, 'in_bounds': True}
        ],
        'render_warnings': [],
    }

    review_input = node._build_review_input(request, render_info)

    assert review_input['mission_text'] == 'inspect the loading bay'
    assert review_input['payload_json']['waypoints'] == '1.0,2.0,0.0'
    assert review_input['subtree_contract_json']['waypoints']['type'] == 'string'
    assert review_input['context_focus']['OSM_CONTEXT']['linear_features'][0]['name'] == 'service road'
    assert review_input['context_snapshot']['ROBOT_POSE']['x'] == 0.0
    assert review_input['context_focus']['FIND_ANYTHING']['query'] == 'loading bay sign'
    assert review_input['object_locations'][0]['query'] == 'loading bay sign'
    assert review_input['review_image_uri'].endswith('plan_review.png')


def test_llm_prompt_keeps_focused_context_without_duplicate_snapshot():
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    captured = []

    class FakeLlm:
        def invoke(self, prompt):
            captured.append(prompt)
            return SimpleNamespace(content='{"status":"pass"}')

    node._get_llm = lambda: FakeLlm()
    node._audit_review_request = lambda *args: None
    node._image_part = lambda image_path: None

    node._invoke_llm(
        {
            'session_id': 'session-8',
            'context_snapshot': {'OSM_CONTEXT': {'large': 'duplicate'}},
            'context_focus': {'OSM_CONTEXT': {'large': 'kept'}},
        },
        '',
    )

    prompt_input = json.loads(captured[0].split('PLAN_REVIEW_INPUT_JSON:\n', 1)[1])
    assert 'context_snapshot' not in prompt_input
    assert prompt_input['context_focus']['OSM_CONTEXT']['large'] == 'kept'


def test_openrouter_reviewer_limits_output_tokens(monkeypatch):
    calls = []

    class FakeOpenRouter:
        def __init__(self, **kwargs):
            calls.append(kwargs)

        def _create_chat_result(self, response):
            return SimpleNamespace(
                generations=[
                    SimpleNamespace(message=SimpleNamespace(response_metadata={}))
                ]
            )

    monkeypatch.setattr(node_module, 'ChatOpenRouter', FakeOpenRouter)
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    node._provider = 'openrouter'
    node._model_name = 'google/gemma-4-26b-a4b-it'
    node._temperature = 0.0
    node._max_output_tokens = 4096
    node._reasoning_effort = 'high'
    node._openrouter_provider_only = ['darkbloom']
    node._openrouter_allow_fallbacks = True
    node._openrouter_seed = 42
    node._openrouter_max_retries = 0
    node._openrouter_timeout_sec = 60.0
    node._openrouter_service_tier = 'flex'
    node._llm = None

    llm = node._get_llm()
    assert isinstance(llm, FakeOpenRouter)
    assert calls == [
        {
            'model': 'google/gemma-4-26b-a4b-it',
            'temperature': 0.0,
            'max_tokens': 4096,
            'max_retries': 0,
            'timeout': 60000,
            'model_kwargs': {'service_tier': 'flex'},
            'seed': 42,
            'reasoning': {'effort': 'high'},
            'openrouter_provider': {
                'only': ['darkbloom'],
                'allow_fallbacks': True,
                'require_parameters': True,
                'data_collection': 'deny',
            },
        }
    ]
    result = llm._create_chat_result(
        {'provider': 'Darkbloom', 'service_tier': 'flex'}
    )
    assert result.generations[0].message.response_metadata == {
        'openrouter_provider': 'Darkbloom',
        'openrouter_service_tier': 'flex',
    }


def test_openrouter_reviewer_falls_back_to_generation_metadata(monkeypatch):
    class FakeOpenRouter:
        def __init__(self, **kwargs):
            pass

        def _create_chat_result(self, response):
            return SimpleNamespace(
                generations=[
                    SimpleNamespace(message=SimpleNamespace(response_metadata={}))
                ]
            )

    monkeypatch.setattr(node_module, 'ChatOpenRouter', FakeOpenRouter)
    monkeypatch.setattr(
        node_module,
        '_openrouter_generation_metadata',
        lambda generation_id: {
            'provider_name': 'Makora',
            'total_cost': 0.0045,
        },
    )
    llm = node_module._provider_recording_openrouter(model='google/gemma')

    result = llm._create_chat_result({'id': 'gen-1'})

    assert result.generations[0].message.response_metadata == {
        'openrouter_provider': 'Makora',
        'openrouter_cost_usd': 0.0045,
    }


def test_prompt_mentions_move_to_map_frame_and_strict_json():
    prompt = REVIEW_PROMPT_TEMPLATE.format(review_input_json='{}')

    assert 'strict JSON only' in prompt
    assert 'MoveTo executes x,y,yaw map-frame waypoints' in prompt
    assert 'OSM_CONTEXT.linear_features' in prompt
    assert 'magenta crosshair rings' in prompt


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


def test_fallback_review_rejects_waypoint_in_blocked_region():
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    review_input = {
        'map_available': True,
        'waypoints': [{'index': 1, 'x': 15.0, 'y': 10.0}],
        'context_snapshot': {
            'blocked_regions': [
                {
                    'id': 'wet_patch',
                    'polygon': [
                        [12.0, 8.0],
                        [18.0, 8.0],
                        [18.0, 12.0],
                        [12.0, 12.0],
                    ],
                }
            ]
        },
    }
    render_info = {'waypoint_pixels': [], 'waypoint_area_checks': []}

    review = node._fallback_review(review_input, render_info)

    assert review['status'] == 'reject'
    assert review['recommended_action'] == 'regenerate'
    assert review['findings'][0]['guard'] == 'blocked_or_exclusion_region'


def test_openrouter_reviewer_can_omit_temperature(monkeypatch):
    calls = []

    class FakeOpenRouter:
        def __init__(self, **kwargs):
            calls.append(kwargs)

    monkeypatch.setattr(node_module, 'ChatOpenRouter', FakeOpenRouter)
    node = PlanReviewerNode.__new__(PlanReviewerNode)
    node._provider = 'openrouter'
    node._model_name = 'openai/gpt-5.6-sol'
    node._temperature = 0.0
    node._omit_temperature = True
    node._max_output_tokens = 8192
    node._reasoning_effort = 'xhigh'
    node._openrouter_provider_only = ['openai']
    node._openrouter_seed = 42
    node._openrouter_max_retries = 0
    node._openrouter_timeout_sec = 60.0
    node._llm = None

    node._get_llm()

    assert 'temperature' not in calls[0]
    assert calls[0]['reasoning'] == {'effort': 'xhigh'}
