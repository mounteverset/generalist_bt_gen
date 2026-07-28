from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]


def test_gps_temperature_tree_uses_typed_gps_navigation_flow():
    tree_path = (
        ROOT
        / 'src'
        / 'bt_executor'
        / 'trees'
        / 'gps_temperature_logging.xml'
    )

    root = ET.parse(tree_path).getroot()
    parse_nodes = list(root.iter('ParseGpsWaypoints'))
    move_nodes = list(root.iter('MoveToGPS'))
    temperature_nodes = list(root.iter('LogTemperature'))

    assert root.attrib['main_tree_to_execute'] == 'gps_temperature_logging.xml'
    assert parse_nodes[0].attrib['raw_waypoints'] == '{gps_waypoints}'
    assert parse_nodes[0].attrib['waypoint_queue'] == '{gps_waypoint_queue}'
    assert move_nodes[0].attrib['gps_pose'] == '{active_gps_waypoint}'
    assert (
        move_nodes[0].attrib['action_name']
        == '/a200_0000/follow_gps_waypoints'
    )
    assert temperature_nodes[0].attrib['logfile_path'] == '{logfile_path}'
