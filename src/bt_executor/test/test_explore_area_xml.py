from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]


def test_explore_area_xml_declares_static_waypoint_flow():
    tree_path = ROOT / 'src' / 'bt_executor' / 'trees' / 'explore_area.xml'

    root = ET.parse(tree_path).getroot()
    tags = [element.tag for element in root.iter()]
    parse_nodes = list(root.iter('ParseWaypoints'))
    move_nodes = list(root.iter('MoveTo'))

    assert root.attrib['main_tree_to_execute'] == 'explore_area.xml'
    assert 'Precondition' in tags
    assert 'LoopString' in tags
    assert parse_nodes[0].attrib['raw_waypoints'] == '{waypoints}'
    assert parse_nodes[0].attrib['waypoint_count'] == '{waypoint_count}'
    assert move_nodes[0].attrib['pose'] == '{active_waypoint}'
    assert move_nodes[0].attrib['action_name'] == '/a200_0000/navigate_to_pose'
