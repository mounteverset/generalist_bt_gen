from pathlib import Path
import xml.etree.ElementTree as ET


TREE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'trees'
    / 'find_and_drive_to_nearest_object.xml'
)


def test_find_and_drive_to_nearest_object_tree_connects_perception_to_navigation():
    root = ET.parse(TREE_PATH).getroot()

    tree_id = 'find_and_drive_to_nearest_object.xml'
    assert root.attrib['main_tree_to_execute'] == tree_id
    behavior_tree = root.find(f"./BehaviorTree[@ID='{tree_id}']")
    assert behavior_tree is not None

    find_anything = behavior_tree.find('.//FindAnything')
    assert find_anything is not None
    assert find_anything.attrib['object'] == '{object}'
    assert find_anything.attrib['max_results'] == '1'
    assert find_anything.attrib['pose'] == '{object_pose}'
    assert find_anything.attrib['frame_id'] == '{object_frame_id}'

    move_to = behavior_tree.find('.//MoveTo')
    assert move_to is not None
    assert move_to.attrib['pose'] == '{object_pose}'
    assert move_to.attrib['frame_id'] == '{object_frame_id}'
    assert move_to.attrib['action_name'] == '/a200_0000/navigate_to_pose'
