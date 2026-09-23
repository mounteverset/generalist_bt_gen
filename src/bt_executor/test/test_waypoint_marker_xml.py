"""Contract tests for mission waypoint visualization."""

from pathlib import Path
import xml.etree.ElementTree as ET


TREES_DIRECTORY = Path(__file__).resolve().parents[1] / 'trees'
GPS_TREES = {
    'blueboat_temperature_logging.xml',
    'gps_temperature_logging.xml',
    'gps_waypoint_navigation.xml',
    'navigate_and_photograph.xml',
    'explore_area.xml',
}
MAP_TREES = {
    'find_and_drive_to_nearest_object.xml',
    'temperature_logging.xml',
}


def test_every_behavior_tree_publishes_or_clears_waypoint_markers():
    for tree_path in sorted(TREES_DIRECTORY.glob('*.xml')):
        root = ET.parse(tree_path).getroot()
        marker_nodes = list(root.iter('PublishWaypointMarkers'))
        assert len(marker_nodes) == 1, tree_path.name


def test_waypoint_marker_nodes_consume_the_matching_payload_contract():
    for tree_name in GPS_TREES:
        root = ET.parse(TREES_DIRECTORY / tree_name).getroot()
        marker = next(root.iter('PublishWaypointMarkers'))
        assert marker.attrib['gps_waypoints'] == '{gps_waypoints}'

    for tree_name in MAP_TREES:
        root = ET.parse(TREES_DIRECTORY / tree_name).getroot()
        marker = next(root.iter('PublishWaypointMarkers'))
        assert marker.attrib['waypoints'] == '{waypoints}'
