from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
TREE_PATH = ROOT / 'src' / 'bt_executor' / 'trees' / 'navigate_and_photograph.xml'


def test_navigate_and_photograph_tree_uses_gps_navigation():
    root = ET.parse(TREE_PATH).getroot()
    tree = root.find("./BehaviorTree[@ID='navigate_and_photograph.xml']")

    parser = tree.find('.//ParseGpsWaypoints')
    move = tree.find('.//MoveToGPS')

    assert parser.attrib['raw_waypoints'] == '{gps_waypoints}'
    assert parser.attrib['waypoint_queue'] == '{gps_waypoint_queue}'
    assert move.attrib['gps_pose'] == '{active_gps_waypoint}'
    assert move.attrib['action_name'] == '/follow_gps_waypoints'
    distance = tree.find('.//DistanceTraveled')
    assert distance is not None
    assert 'odom_topic' not in distance.attrib
    assert tree.find('.//TakePhoto') is not None
    assert tree.find('.//ParseWaypoints') is None
    assert tree.find('.//MoveTo') is None


def test_navigate_and_photograph_odom_topic_is_executor_configuration():
    metadata = yaml.safe_load((ROOT / 'config' / 'tree_metadata.yaml').read_text())
    tree = next(
        item for item in metadata['trees']
        if item['id'] == 'navigate_and_photograph.xml'
    )
    executor_params = yaml.safe_load(
        (
            ROOT
            / 'src'
            / 'generalist_bringup'
            / 'config'
            / 'bt_executor_params.yaml'
        ).read_text()
    )['bt_action_server']['ros__parameters']

    assert 'odom_topic' not in tree['blackboard_contract']
    assert executor_params['distance_traveled_odom_topic']
