import json
import sys
import types
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

    if 'gen_bt_interfaces.srv' not in sys.modules:
        gen_srv_module = types.ModuleType('gen_bt_interfaces.srv')

        class DummySrv:
            class Request:
                pass

            class Response:
                pass

        gen_srv_module.CreatePayload = DummySrv
        gen_srv_module.GetMissionState = DummySrv
        gen_srv_module.MissionControl = DummySrv
        gen_srv_module.OperatorDecision = DummySrv
        gen_srv_module.SelectBehaviorTree = DummySrv
        sys.modules['gen_bt_interfaces.srv'] = gen_srv_module

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
            'demo_tree.xml',
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
    assert hint['MISSION_REQUEST']['subtree_id'] == 'demo_tree.xml'
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
        'demo_tree.xml',
        1,
        'missed waypoint 2',
        '{"waypoints": "1.0,2.0,0.0"}',
        'The first payload only covered one waypoint.',
    )
    second = node._append_refinement_history_entry(
        'session-42',
        'demo_tree.xml',
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
