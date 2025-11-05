import json

import pytest

from llm_interface.bt_xml_generator import BehaviorTreeGenerationError, generate_bt_xml


def test_generate_bt_xml_round_trip():
    spec = {
        "tree_id": "InspectField",
        "root": "root_sequence",
        "nodes": [
            {
                "id": "root_sequence",
                "type": "Sequence",
                "children": ["check_weather", "drive_route"],
            },
            {
                "id": "check_weather",
                "type": "Action",
                "plugin": "CheckWeather",
                "inputs": {"location": "field"},
            },
            {
                "id": "drive_route",
                "type": "Action",
                "plugin": "DriveRoute",
                "inputs": {"waypoints": [1, 2, 3]},
            },
        ],
    }

    xml_output = generate_bt_xml(spec)
    assert "InspectField" in xml_output
    assert "CheckWeather" in xml_output
    assert json.dumps([1, 2, 3]) in xml_output


def test_generate_bt_xml_invalid_root():
    spec = {"nodes": []}
    with pytest.raises(BehaviorTreeGenerationError):
        generate_bt_xml(spec)
