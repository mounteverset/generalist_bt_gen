from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
TREE_PATH = ROOT / 'src' / 'bt_executor' / 'trees' / 'blueboat_temperature_logging.xml'


def test_blueboat_temperature_tree_visits_waypoints_and_cycles_probe():
    root = ET.parse(TREE_PATH).getroot()

    assert root.attrib['main_tree_to_execute'] == 'blueboat_temperature_logging.xml'
    mission = root.find('.//BehaviorTree/Sequence')
    assert mission is not None
    assert [child.tag for child in mission] == [
        'PublishWaypointMarkers',
        'ParseGpsWaypoints',
        'WaitForMavrosGpsFix',
        'SetMavrosMode',
        'WaitForMavrosMode',
        'SetMavrosMode',
        'WaitForMavrosMode',
        'SetMavrosArm',
        'WaitForMavrosMode',
        'LoopString',
    ]
    parser = root.find('.//ParseGpsWaypoints')
    assert parser is not None
    assert parser.attrib['raw_waypoints'] == '{gps_waypoints}'

    gps_fix = root.find('.//WaitForMavrosGpsFix')
    assert gps_fix is not None
    assert gps_fix.attrib['fix_topic'] == '/mavros/global_position/raw/fix'
    assert gps_fix.attrib['timeout_sec'] == '{gps_fix_timeout_sec}'

    set_modes = root.findall('.//SetMavrosMode')
    assert [node.attrib['custom_mode'] for node in set_modes] == ['HOLD', 'GUIDED']
    assert {node.attrib['service_name'] for node in set_modes} == {'/mavros/set_mode'}

    wait_modes = root.findall('.//WaitForMavrosMode')
    assert [node.attrib['desired_mode'] for node in wait_modes] == [
        'HOLD',
        'GUIDED',
        'GUIDED',
    ]
    assert [node.attrib['require_armed'] for node in wait_modes] == [
        'false',
        'false',
        'true',
    ]
    assert {node.attrib['timeout_sec'] for node in wait_modes} == {
        '{guided_mode_timeout_sec}'
    }
    assert {node.attrib['state_topic'] for node in wait_modes} == {'/mavros/state'}

    arm = root.find('.//SetMavrosArm')
    assert arm is not None
    assert arm.attrib['arm'] == 'true'
    assert arm.attrib['service_name'] == '/mavros/cmd/arming'

    loop = root.find('.//LoopString')
    assert loop is not None
    assert loop.attrib['if_empty'] == 'SUCCESS'

    move = root.find('.//MoveToGlobalSetpoint')
    assert move is not None
    assert move.attrib['gps_pose'] == '{active_blueboat_waypoint}'
    assert move.attrib['acceptance_radius_m'] == '{acceptance_radius_m}'
    assert move.attrib['publish_rate_hz'] == '5.0'
    assert move.attrib['setpoint_topic'] == '/mavros/setpoint_position/global'
    assert move.attrib['position_topic'] == '/mavros/global_position/global'
    assert len(move) == 1
    assert move[0].tag == 'Sequence'

    depths = root.findall('.//SetDepth')
    assert [node.attrib['target_depth_cm'] for node in depths] == [
        '{measurement_depth_cm}',
        '{raise_depth_cm}',
        '{raise_depth_cm}',
    ]
    assert {node.attrib['action_name'] for node in depths} == {
        '/green/stepper/set_depth'
    }
    assert {node.attrib['max_depth_cm'] for node in depths} == {'200'}

    temperature = root.find('.//LogTemperature')
    assert temperature is not None
    assert temperature.attrib['service_name'] == '/green/read_temp_cached'

    recovery = root.find('.//Fallback/ForceFailure/SetDepth')
    assert recovery is not None
    assert recovery.attrib['name'] == 'RaiseTemperatureProbeAfterFailure'

    hold = root.find('.//Sleep')
    assert hold is not None
    assert hold.attrib['msec'] == '{station_hold_ms}'
