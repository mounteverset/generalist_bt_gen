from pathlib import Path
import xml.etree.ElementTree as ET


TREE_PATH = Path(__file__).resolve().parents[1] / 'trees' / '360_rgb_sweep.xml'


def test_sweep_uses_pose_frame_and_captures_every_60_degree_heading():
    sequence = ET.parse(TREE_PATH).getroot().find('./BehaviorTree/Sequence')
    assert sequence is not None
    assert sequence.find('GetCurrentPose').attrib['current_frame_id'] == '{sweep_frame_id}'

    movements = sequence.findall('MoveTo')
    assert [node.attrib['name'] for node in movements] == [
        'RotateTo060', 'RotateTo120', 'RotateTo180',
        'RotateTo240', 'RotateTo300', 'ReturnToStartHeading',
    ]
    assert all(node.attrib['frame_id'] == '{sweep_frame_id}' for node in movements)

    captures = sequence.findall('TakePhoto')
    assert [node.attrib['filename_prefix'] for node in captures] == [
        'sweep_000', 'sweep_060', 'sweep_120',
        'sweep_180', 'sweep_240', 'sweep_300',
    ]
    children = list(sequence)
    assert children.index(captures[0]) < children.index(movements[0])
    assert children.index(captures[-1]) < children.index(movements[-1])
