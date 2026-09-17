"""Contract tests for the explore-area behavior tree."""

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_explore_area_xml_declares_geographic_waypoint_flow():
    """The tree should submit the complete GPS route in one action node."""
    tree_path = ROOT / 'src' / 'bt_executor' / 'trees' / 'explore_area.xml'

    root = ET.parse(tree_path).getroot()
    move_nodes = list(root.iter('MoveToGPS'))

    assert root.attrib['main_tree_to_execute'] == 'explore_area.xml'
    assert len(move_nodes) == 1
    assert move_nodes[0].attrib['gps_poses'] == '{gps_waypoints}'
    assert move_nodes[0].attrib['action_name'] == '/follow_gps_waypoints'
    assert not list(root.iter('ParseGpsWaypoints'))
    assert not list(root.iter('LoopString'))
    assert not list(root.iter('ParseWaypoints'))
    assert not list(root.iter('MoveTo'))


def test_explore_area_metadata_contract_matches_tree_input():
    """The catalogue contract should expose exactly the tree's route input."""
    metadata_path = ROOT / 'config' / 'tree_metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    tree = next(
        item for item in metadata['trees'] if item['id'] == 'explore_area.xml'
    )

    contract = tree['blackboard_contract']
    assert contract['gps_waypoints']['required'] is True
    route_description = contract['gps_waypoints']['schema']['description']
    assert 'FollowGPSWaypoints' in route_description
    assert 'waypoints' not in contract
    assert 'area_polygon' not in contract
    assert 'frontiers' not in contract
