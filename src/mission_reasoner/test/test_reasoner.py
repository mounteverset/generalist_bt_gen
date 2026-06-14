import json
from pathlib import Path

import yaml

from mission_reasoner.reasoner import ACCEPT, CLARIFY, REFUSE, MissionReasoner


ROOT = Path(__file__).resolve().parents[3]


def _reasoner() -> MissionReasoner:
    with open(ROOT / 'config' / 'system_description.yaml', 'r', encoding='utf-8') as handle:
        return MissionReasoner(yaml.safe_load(handle))


def _trees():
    with open(ROOT / 'config' / 'tree_metadata.yaml', 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)['trees']


def test_explore_area_metadata_advertises_payload_and_context_contract():
    tree = next(item for item in _trees() if item['id'] == 'explore_area.xml')

    assert 'payload.parse_waypoints' in tree['required_capabilities']
    assert tree['selection_constraints']['requires_target_area'] is True
    assert set(tree['context_requirements']) >= {
        'ROBOT_POSE',
        'ANNOTATED_SLAM_MAP_IMAGE',
        'GPS_FIX',
        'SATELLITE_MAP',
        'OSM_CONTEXT',
        'BATTERY_STATE',
    }
    contract = tree['blackboard_contract']
    assert contract['waypoints']['required'] is True
    assert contract['area_polygon']['required'] is True
    assert contract['frontiers']['required'] is True
    assert contract['area_polygon_geo']['required'] is False
    assert contract['frontiers_geo']['required'] is False


def test_accepts_ground_navigation_and_photos():
    result = _reasoner().validate(
        'Drive to these waypoints and take photos.',
        _trees(),
        context_json=json.dumps({'waypoints': '0,0,0; 1,0,0'}),
    )

    assert result.status_code == ACCEPT
    assert 'navigate_and_photograph.xml' in result.candidate_trees
    assert 'sensing.rgb_image' in result.matched_capabilities


def test_clarifies_document_command_without_target():
    result = _reasoner().validate('Document the area.', _trees())

    assert result.status_code == CLARIFY
    assert 'target area or route' in result.clarification_question


def test_clarifies_explore_command_without_area_definition():
    result = _reasoner().validate('Explore the area.', _trees())

    assert result.status_code == CLARIFY
    assert 'What area should the robot explore' in result.clarification_question


def test_accepts_explore_command_with_polygon_area():
    result = _reasoner().validate(
        'Explore the polygon 0,0; 10,0; 10,5; 0,5.',
        _trees(),
    )

    assert result.status_code == ACCEPT
    assert 'explore_area.xml' in result.candidate_trees
    assert 'mapping.slam' in result.matched_capabilities


def test_refuses_aerial_mission():
    result = _reasoner().validate(
        'Fly over the forest and take aerial photos of the treetops.',
        _trees(),
    )

    assert result.status_code == REFUSE
    assert 'locomotion.flight' in result.missing_capabilities
    assert 'cannot fly' in result.message


def test_refuses_missing_thermal_sensor():
    result = _reasoner().validate('Take thermal images of the field.', _trees())

    assert result.status_code == REFUSE
    assert 'sensing.thermal_image' in result.missing_capabilities
    assert 'thermal' in result.message


def test_refuses_soil_sampling():
    result = _reasoner().validate('Dig soil samples every 10 meters.', _trees())

    assert result.status_code == REFUSE
    assert 'sampling.soil' in result.missing_capabilities


def test_refuses_range_beyond_platform_limit():
    result = _reasoner().validate('Drive a 15 km patrol route.', _trees())

    assert result.status_code == REFUSE
    assert 'platform.range' in result.missing_capabilities
    assert '15000' in result.message


def test_refuses_capability_extracted_by_llm_requirements():
    result = _reasoner().validate(
        'Inspect the site from above.',
        _trees(),
        extracted_requirements={
            'required_capabilities': ['locomotion.flight', 'sensing.rgb_image'],
            'rationale': 'The phrase from above implies aerial inspection.',
        },
    )

    assert result.status_code == REFUSE
    assert 'locomotion.flight' in result.missing_capabilities
    assert result.reasoning['llm_requirements']['required_capabilities'] == [
        'locomotion.flight',
        'sensing.rgb_image',
    ]


def test_refuses_range_extracted_by_llm_requirements():
    result = _reasoner().validate(
        'Do a long perimeter patrol.',
        _trees(),
        extracted_requirements={
            'required_capabilities': ['locomotion.ground', 'navigation.waypoints'],
            'constraints': {'range_m': 15000},
        },
    )

    assert result.status_code == REFUSE
    assert 'platform.range' in result.missing_capabilities
