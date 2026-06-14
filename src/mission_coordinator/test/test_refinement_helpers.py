import json
import sys
import types
import asyncio
from types import SimpleNamespace


def _install_ros_stubs() -> None:
    if 'rclpy' not in sys.modules:
        sys.modules['rclpy'] = types.ModuleType('rclpy')

    if 'rclpy.node' not in sys.modules:
        node_module = types.ModuleType('rclpy.node')

        class DummyNode:
            pass

        node_module.Node = DummyNode
        sys.modules['rclpy.node'] = node_module

    if 'rclpy.action' not in sys.modules:
        action_module = types.ModuleType('rclpy.action')

        class DummyActionClient:
            pass

        class DummyActionServer:
            pass

        class DummyCancelResponse:
            ACCEPT = 1

        class DummyGoalResponse:
            ACCEPT = 1
            REJECT = 2

        action_module.ActionClient = DummyActionClient
        action_module.ActionServer = DummyActionServer
        action_module.CancelResponse = DummyCancelResponse
        action_module.GoalResponse = DummyGoalResponse
        sys.modules['rclpy.action'] = action_module

    if 'rclpy.callback_groups' not in sys.modules:
        callback_module = types.ModuleType('rclpy.callback_groups')

        class DummyReentrantCallbackGroup:
            pass

        callback_module.ReentrantCallbackGroup = DummyReentrantCallbackGroup
        sys.modules['rclpy.callback_groups'] = callback_module

    if 'rclpy.executors' not in sys.modules:
        executor_module = types.ModuleType('rclpy.executors')

        class DummyMultiThreadedExecutor:
            pass

        executor_module.MultiThreadedExecutor = DummyMultiThreadedExecutor
        sys.modules['rclpy.executors'] = executor_module

    if 'rclpy.task' not in sys.modules:
        task_module = types.ModuleType('rclpy.task')

        class DummyFuture:
            pass

        task_module.Future = DummyFuture
        sys.modules['rclpy.task'] = task_module

    if 'action_msgs.msg' not in sys.modules:
        goal_status_module = types.ModuleType('action_msgs.msg')

        class DummyGoalStatus:
            STATUS_CANCELED = 5

        goal_status_module.GoalStatus = DummyGoalStatus
        sys.modules['action_msgs.msg'] = goal_status_module

    if 'btcpp_ros2_interfaces.action' not in sys.modules:
        bt_action_module = types.ModuleType('btcpp_ros2_interfaces.action')

        class DummyExecuteTree:
            class Goal:
                pass

        bt_action_module.ExecuteTree = DummyExecuteTree
        sys.modules['btcpp_ros2_interfaces.action'] = bt_action_module

    if 'btcpp_ros2_interfaces.msg' not in sys.modules:
        bt_msg_module = types.ModuleType('btcpp_ros2_interfaces.msg')

        class DummyNodeStatus:
            SUCCESS = 1
            FAILURE = 2

        bt_msg_module.NodeStatus = DummyNodeStatus
        sys.modules['btcpp_ros2_interfaces.msg'] = bt_msg_module

    if 'gen_bt_interfaces.action' not in sys.modules:
        gen_action_module = types.ModuleType('gen_bt_interfaces.action')

        class DummyMissionCommand:
            class Goal:
                pass

            class Result:
                pass

            class Feedback:
                pass

        class DummyGatherContext:
            class Goal:
                pass

            class Result:
                pass

            class Feedback:
                pass

        gen_action_module.MissionCommand = DummyMissionCommand
        gen_action_module.GatherContext = DummyGatherContext
        sys.modules['gen_bt_interfaces.action'] = gen_action_module

    gen_srv_module = sys.modules.get('gen_bt_interfaces.srv')
    if gen_srv_module is None:
        gen_srv_module = types.ModuleType('gen_bt_interfaces.srv')
        sys.modules['gen_bt_interfaces.srv'] = gen_srv_module

    class DummySrv:
        class Request:
            pass

        class Response:
            pass

    gen_srv_module.CreatePayload = DummySrv
    gen_srv_module.GetMissionState = DummySrv
    gen_srv_module.MissionControl = DummySrv
    gen_srv_module.OperatorDecision = DummySrv
    gen_srv_module.ReviewPlan = getattr(gen_srv_module, 'ReviewPlan', DummySrv)
    gen_srv_module.SelectBehaviorTree = DummySrv
    gen_srv_module.ValidateMission = DummySrv

    if 'std_msgs.msg' not in sys.modules:
        std_msgs_module = types.ModuleType('std_msgs.msg')

        class DummyString:
            pass

        std_msgs_module.String = DummyString
        sys.modules['std_msgs.msg'] = std_msgs_module


_install_ros_stubs()

from mission_coordinator.node import MissionCoordinatorNode


def test_compose_payload_user_command_appends_operator_feedback():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)

    command = node._compose_payload_user_command(
        'inspect the loading zone',
        'use the left-side route and avoid the blocked entrance',
        prior_payload_json='{"waypoints": "1.0,2.0,0.0"}',
        prior_reasoning='The prior plan inferred only a single waypoint.',
        refinement_history=[
            {
                'iteration': 1,
                'operator_feedback': 'first rejection',
                'prior_payload_json': '{"waypoints": "1.0,2.0,0.0"}',
            }
        ],
    )

    assert command.startswith('inspect the loading zone')
    assert 'OPERATOR_REFINEMENT_FEEDBACK:' in command
    assert 'avoid the blocked entrance' in command
    assert 'PRIOR_REJECTED_PLAN_REASONING:' in command
    assert 'The prior plan inferred only a single waypoint.' in command
    assert 'PRIOR_REJECTED_PAYLOAD_JSON:' in command
    assert '{"waypoints": "1.0,2.0,0.0"}' in command
    assert 'SESSION_REFINEMENT_HISTORY_JSON:' in command
    assert '"iteration": 1' in command


def test_build_context_gather_hint_preserves_existing_context_and_refinement():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    goal = SimpleNamespace(
        session_id='session-42',
        command='inspect the loading zone',
        context_json=json.dumps({'auto_execute': False, 'site': 'north-yard'}),
    )

    hint = json.loads(
        node._build_context_gather_hint(
            'temperature_logging.xml',
            goal,
            operator_feedback='focus on the marked loading bay',
            prior_payload_json='{"waypoints": "1.0,2.0,0.0"}',
            prior_reasoning='The prior payload did not cover the full area.',
            refinement_history=[
                {
                    'iteration': 1,
                    'operator_feedback': 'missed the loading bay',
                    'prior_payload_json': '{"waypoints": "1.0,2.0,0.0"}',
                }
            ],
        )
    )

    assert hint['auto_execute'] is False
    assert hint['site'] == 'north-yard'
    assert hint['MISSION_REQUEST']['session_id'] == 'session-42'
    assert hint['MISSION_REQUEST']['subtree_id'] == 'temperature_logging.xml'
    assert hint['MISSION_REFINEMENT']['requested'] is True
    assert (
        hint['MISSION_REFINEMENT']['operator_feedback']
        == 'focus on the marked loading bay'
    )
    assert (
        hint['MISSION_REFINEMENT']['prior_payload_json']
        == '{"waypoints": "1.0,2.0,0.0"}'
    )
    assert (
        hint['MISSION_REFINEMENT']['prior_reasoning']
        == 'The prior payload did not cover the full area.'
    )
    assert len(hint['MISSION_REFINEMENT']['history']) == 1
    assert hint['MISSION_REFINEMENT']['history'][0]['iteration'] == 1
    assert 'OPERATOR_REFINEMENT_FEEDBACK:' in hint['MISSION_REFINEMENT']['prompt']
    assert 'PRIOR_REJECTED_PLAN_REASONING:' in hint['MISSION_REFINEMENT']['prompt']
    assert 'PRIOR_REJECTED_PAYLOAD_JSON:' in hint['MISSION_REFINEMENT']['prompt']


def test_append_refinement_history_entry_accumulates_session_history():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    node._refinement_history = {}

    first = node._append_refinement_history_entry(
        'session-42',
        'temperature_logging.xml',
        1,
        'missed waypoint 2',
        '{"waypoints": "1.0,2.0,0.0"}',
        'The first payload only covered one waypoint.',
    )
    second = node._append_refinement_history_entry(
        'session-42',
        'temperature_logging.xml',
        2,
        'still too close to the obstacle',
        '{"waypoints": "1.0,2.0,0.0; 3.0,4.0,0.0"}',
        'The second payload improved coverage but path safety was poor.',
    )

    assert len(first) == 1
    assert len(second) == 2
    assert second[0]['operator_feedback'] == 'missed waypoint 2'
    assert second[0]['payload_waypoints'] == '1.0,2.0,0.0'
    assert second[1]['iteration'] == 2
    assert second[1]['payload_waypoints'] == '1.0,2.0,0.0; 3.0,4.0,0.0'


def test_build_reasoner_tree_catalog_includes_capability_metadata():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    node._tree_catalog = [
        ('temperature_logging.xml', 'Fallback temperature logging tree.'),
        ('navigate_and_photograph.xml', 'Navigate and take photos.'),
    ]
    node._tree_metadata = {
        'temperature_logging.xml': {
            'description': 'Metadata description.',
            'mission_intents': ['navigate_waypoints'],
            'required_capabilities': ['navigation.waypoints'],
            'unsupported_requirements': ['locomotion.flight'],
            'selection_constraints': {'max_range_m': 5000},
        },
        'navigate_and_photograph.xml': {
            'mission_intents': ['photograph_route'],
            'required_capabilities': ['navigation.waypoints', 'sensing.rgb_image'],
        },
    }

    catalog = json.loads(node._build_reasoner_tree_catalog_json())['trees']

    assert catalog[0]['id'] == 'temperature_logging.xml'
    assert catalog[0]['description'] == 'Metadata description.'
    assert catalog[0]['required_capabilities'] == ['navigation.waypoints']
    assert catalog[0]['selection_constraints']['max_range_m'] == 5000
    assert catalog[1]['required_capabilities'] == [
        'navigation.waypoints',
        'sensing.rgb_image',
    ]


def test_format_reasoner_rejection_includes_llm_requirements_and_ambiguities():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    response = SimpleNamespace(
        message='No behavior tree in the catalogue satisfies the requested capabilities.',
        reasoning_json=json.dumps(
            {
                'requested_capabilities': [
                    'locomotion.ground',
                    'navigation.waypoints',
                    'sensing.temperature',
                    'payload.parse_waypoints',
                    'mapping.slam',
                ],
                'llm_requirements': {
                    'required_capabilities': [
                        'locomotion.ground',
                        'navigation.waypoints',
                        'sensing.temperature',
                        'payload.parse_waypoints',
                        'mapping.slam',
                    ],
                    'mission_intents': ['navigate_waypoints', 'log_temperature'],
                    'ambiguities': [
                        'The exact coordinates for the 3 waypoints are not specified.',
                        'It is unclear if the robot should actively generate a map.',
                    ],
                    'constraints': {'range_m': None, 'duration_s': None},
                },
                'available_trees': [
                    'temperature_logging.xml',
                    'navigate_and_photograph.xml',
                    'explore_area.xml',
                ],
            }
        ),
        matched_capabilities=[
            'locomotion.ground',
            'mapping.slam',
            'navigation.waypoints',
            'payload.parse_waypoints',
            'sensing.temperature',
        ],
        missing_capabilities=[
            'locomotion.ground',
            'mapping.slam',
            'navigation.waypoints',
            'payload.parse_waypoints',
            'sensing.temperature',
        ],
        candidate_trees=[],
    )

    message = node._format_reasoner_rejection(response)

    assert 'Reasoner details:' in message
    assert 'Required capabilities: locomotion.ground' in message
    assert 'mapping.slam' in message
    assert 'Candidate behavior trees: none' in message
    assert 'temperature_logging.xml' in message
    assert 'Mission intents inferred: navigate_waypoints, log_temperature' in message
    assert 'The exact coordinates for the 3 waypoints are not specified.' in message


def test_build_pending_plan_includes_plan_review():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    node._operator_service_name = '/mission_coordinator/operator_decision_test'
    goal = SimpleNamespace(session_id='session-42', command='inspect the bay')
    gather_result = SimpleNamespace(context_json='{}', attachment_uris=[])
    payload = SimpleNamespace(
        reasoning='Generated a route.',
        payload_json='{"waypoints": "1.0,2.0,0.0"}',
        tool_trace_json='[]',
    )
    review = {
        'status': 'warn',
        'summary': 'Waypoint 1 is close to unknown space.',
        'findings': [{'severity': 'warning'}],
        'review_image_uri': 'file:///tmp/context_gatherer/plan_review.png',
        'recommended_action': 'submit_to_operator',
    }

    plan = node._build_pending_plan(
        'temperature_logging.xml',
        goal,
        gather_result,
        payload,
        plan_review=review,
    )

    assert plan['plan_review']['status'] == 'warn'
    assert plan['plan_review']['review_image_uri'].endswith('plan_review.png')


def test_review_feedback_for_refinement_includes_findings():
    review = {
        'summary': 'Waypoint 2 crosses water.',
        'findings': [
            {
                'severity': 'critical',
                'category': 'robot_safety',
                'waypoint_indices': [2],
                'description': 'Route segment enters water.',
                'recommended_fix': 'Move waypoint 2 onto the service path.',
            }
        ],
        'recommended_action': 'regenerate',
    }

    feedback = MissionCoordinatorNode._review_feedback_for_refinement(review)

    assert 'PLAN_REVIEW_REJECTION:' in feedback
    assert 'Waypoint 2 crosses water.' in feedback
    assert 'PLAN_REVIEW_FINDINGS_JSON:' in feedback
    assert '"robot_safety"' in feedback


def test_refine_rejected_plan_reuses_existing_context_without_regather():
    node = MissionCoordinatorNode.__new__(MissionCoordinatorNode)
    node._refinement_history = {}
    node.STATE_BUILDING_PAYLOAD = 'BUILDING_PAYLOAD'
    statuses = []
    lifecycle_states = []
    create_payload_calls = []

    def publish_status(message):
        statuses.append(message)

    def set_lifecycle_state(state):
        lifecycle_states.append(state)

    async def gather_context(*_args, **_kwargs):
        raise AssertionError('refinement should not re-gather context')

    async def create_payload(
        tree_id,
        goal,
        gather_result,
        operator_feedback='',
        prior_payload_json='',
        prior_reasoning='',
        refinement_history=None,
    ):
        create_payload_calls.append(
            {
                'tree_id': tree_id,
                'goal': goal,
                'gather_result': gather_result,
                'operator_feedback': operator_feedback,
                'prior_payload_json': prior_payload_json,
                'prior_reasoning': prior_reasoning,
                'refinement_history': refinement_history,
            }
        )
        return SimpleNamespace(
            payload_json='{"waypoints": "3.0,4.0,0.0"}',
            reasoning='Refined payload.',
            tool_trace_json='[]',
        )

    node._publish_status = publish_status
    node._set_lifecycle_state = set_lifecycle_state
    node._gather_context = gather_context
    node._create_payload = create_payload
    node._normalize_session_id = lambda session_id: session_id or 'unknown'

    goal = SimpleNamespace(session_id='session-42', command='inspect the bay')
    original_context = SimpleNamespace(
        success=True,
        context_json='{"SATELLITE_MAP": {"uri": "file:///tmp/map.png"}}',
        attachment_uris=['file:///tmp/map.png'],
    )
    prior_payload = SimpleNamespace(
        payload_json='{"waypoints": "1.0,2.0,0.0"}',
        reasoning='Original payload.',
    )

    refined_context, refined_payload = asyncio.run(
        node._refine_rejected_plan(
            'temperature_logging.xml',
            goal,
            original_context,
            'move waypoint away from the curb',
            1,
            prior_payload,
        )
    )

    assert refined_context is original_context
    assert refined_payload.reasoning == 'Refined payload.'
    assert lifecycle_states == ['BUILDING_PAYLOAD']
    assert create_payload_calls[0]['gather_result'] is original_context
    assert create_payload_calls[0]['operator_feedback'] == 'move waypoint away from the curb'
    assert create_payload_calls[0]['prior_payload_json'] == '{"waypoints": "1.0,2.0,0.0"}'
    assert create_payload_calls[0]['prior_reasoning'] == 'Original payload.'
    assert 'using existing context' in statuses[0]
