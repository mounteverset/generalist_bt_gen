from pathlib import Path
import xml.etree.ElementTree as ET


TREE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'trees'
    / 'find_and_drive_to_nearest_object.xml'
)


def test_find_and_drive_tree_executes_planner_generated_object_waypoints():
    root = ET.parse(TREE_PATH).getroot()

    tree_id = 'find_and_drive_to_nearest_object.xml'
    assert root.attrib['main_tree_to_execute'] == tree_id
    behavior_tree = root.find(f"./BehaviorTree[@ID='{tree_id}']")
    assert behavior_tree is not None

    assert behavior_tree.find('.//FindAnything') is None
    assert behavior_tree.find('.//FindObjectLocation') is None

    parse_waypoints = behavior_tree.find('.//ParseWaypoints')
    assert parse_waypoints is not None
    assert parse_waypoints.attrib['raw_waypoints'] == '{waypoints}'
    assert parse_waypoints.attrib['waypoint_queue'] == '{waypoint_queue}'

    loop = behavior_tree.find('.//LoopString')
    assert loop is not None
    assert loop.attrib['queue'] == '{waypoint_queue}'
    assert loop.attrib['value'] == '{active_waypoint}'
    assert loop.attrib['if_empty'] == 'FAILURE'

    move_to = behavior_tree.find('.//MoveTo')
    assert move_to is not None
    assert move_to.attrib['pose'] == '{active_waypoint}'
    assert move_to.attrib['frame_id'] == '{waypoint_frame_id}'
    assert move_to.attrib['action_name'] == '/a200_0000/navigate_to_pose'
