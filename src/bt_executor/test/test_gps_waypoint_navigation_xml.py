from pathlib import Path
import xml.etree.ElementTree as ET


TREE_PATH = Path(__file__).resolve().parents[1] / 'trees' / 'gps_waypoint_navigation.xml'


def test_gps_waypoint_navigation_tree_uses_geographic_action():
    root = ET.parse(TREE_PATH).getroot()

    assert root.attrib['main_tree_to_execute'] == 'gps_waypoint_navigation.xml'
    behavior_tree = root.find("./BehaviorTree[@ID='gps_waypoint_navigation.xml']")
    assert behavior_tree is not None

    parser = behavior_tree.find('.//ParseGpsWaypoints')
    assert parser is not None
    assert parser.attrib['raw_waypoints'] == '{gps_waypoints}'

    move = behavior_tree.find('.//MoveToGPS')
    assert move is not None
    assert move.attrib['gps_pose'] == '{active_gps_waypoint}'
    assert move.attrib['action_name'] == '/a200_0000/follow_gps_waypoints'

    assert behavior_tree.find('.//MoveTo') is None
    assert behavior_tree.find('.//LogTemperature') is None
