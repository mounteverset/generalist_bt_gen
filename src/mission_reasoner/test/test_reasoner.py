import json
from pathlib import Path

import yaml

from mission_reasoner.reasoner import ACCEPT, CLARIFY, REFUSE, MissionReasoner


ROOT = Path(__file__).resolve().parents[3]


def _reasoner() -> MissionReasoner:
    with open(ROOT / 'config' / 'system_description.yaml', 'r', encoding='utf-8') as handle:
        return MissionReasoner(yaml.safe_load(handle))


def _blueboat_reasoner() -> MissionReasoner:
    with open(
        ROOT / 'config' / 'system_description_blueboat.yaml',
        'r',
        encoding='utf-8',
    ) as handle:
        return MissionReasoner(yaml.safe_load(handle))


def _trees():
    with open(ROOT / 'config' / 'tree_metadata.yaml', 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)['trees']


def test_explore_area_metadata_advertises_payload_and_context_contract():
    tree = next(item for item in _trees() if item['id'] == 'explore_area.xml')

    assert 'navigation.gps_waypoints' in tree['required_capabilities']
    assert 'payload.parse_gps_waypoints' in tree['required_capabilities']
    assert tree['selection_constraints']['requires_target_area'] is True
    assert set(tree['context_requirements']) >= {
        'ROBOT_POSE',
        'GPS_FIX',
        'OSM_CONTEXT',
        'SATELLITE_MAP',
    }
    contract = tree['blackboard_contract']
    assert contract['gps_waypoints']['required'] is True
    assert contract['area_polygon_geo']['required'] is False
    assert contract['frontiers_geo']['required'] is False


def test_accepts_geographic_coverage_mission():
    result = _reasoner().validate(
        'Plan a Husky coverage route inside this latitude longitude polygon.',
        _trees(),
        extracted_requirements={
            'required_capabilities': [
                'localization.gps',
                'localization.odometry',
                'locomotion.ground',
                'mapping.slam',
                'navigation.gps_waypoints',
                'navigation.waypoints',
                'payload.parse_gps_waypoints',
            ]
        },
    )

    assert result.status_code == ACCEPT
    assert result.candidate_trees == ['explore_area.xml']


def test_accepts_ground_navigation_and_photos():
    result = _reasoner().validate(
        (
            'Drive the Husky along the gravel path towards the north end of the lake. '
            'Take a photograph every 10 metres travelled.'
        ),
        _trees(),
        extracted_requirements={
            'required_capabilities': [
                'localization.gps',
                'localization.odometry',
                'locomotion.ground',
                'navigation.gps_waypoints',
                'navigation.waypoints',
                'sensing.rgb_image',
            ]
        },
    )

    assert result.status_code == ACCEPT
    assert 'navigate_and_photograph.xml' in result.candidate_trees
    assert 'navigation.gps_waypoints' in result.matched_capabilities
    assert 'sensing.rgb_image' in result.matched_capabilities


def test_selects_find_and_drive_tree_for_nearest_object_mission():
    result = _reasoner().validate(
        'Find the nearest red chair and drive to it.',
        _trees(),
    )

    assert result.status_code == ACCEPT
    assert result.candidate_trees == ['find_and_drive_to_nearest_object.xml']
    assert 'perception.object_location' in result.matched_capabilities
    assert 'navigation.waypoints' in result.matched_capabilities


def test_find_and_drive_metadata_routes_find_anything_through_planning_context():
    tree = next(
        item for item in _trees()
        if item['id'] == 'find_and_drive_to_nearest_object.xml'
    )

    assert tree['context_requirements'] == [
        'ROBOT_POSE',
        'ANNOTATED_SLAM_MAP_IMAGE',
        'FIND_ANYTHING',
    ]
    assert 'payload.parse_waypoints' in tree['required_capabilities']
    assert 'waypoints' in tree['blackboard_contract']
    assert 'object' not in tree['blackboard_contract']


def test_accepts_document_command_with_structured_routes_context():
    result = _reasoner().validate(
        'Document the supplied ground route R1 by taking photographs.',
        _trees(),
        context_json=json.dumps(
            {
                'available_context': ['ROUTE_DEFINITION', 'RGB_IMAGE'],
                'routes': {
                    'R1': {
                        'ordered_waypoints': [
                            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                            {'x': 10.0, 'y': 0.0, 'yaw': 0.0},
                        ]
                    }
                },
            }
        ),
    )

    assert result.status_code == ACCEPT
    assert 'navigate_and_photograph.xml' in result.candidate_trees


def test_selects_gps_temperature_tree_for_explicit_geographic_waypoints():
    result = _reasoner().validate(
        'Drive through these GPS waypoints and log temperature.',
        _trees(),
        context_json=json.dumps(
            {'gps_waypoints': '48.2848,11.6077,0.0; 48.2851,11.6074,1.57'}
        ),
    )

    assert result.status_code == ACCEPT
    assert result.candidate_trees == ['gps_temperature_logging.xml']
    assert 'navigation.gps_waypoints' in result.matched_capabilities


def test_blueboat_profile_selects_guided_temperature_tree():
    reasoner = _blueboat_reasoner()
    result = reasoner.validate(
        'Take the BlueBoat through these GPS waypoints and log water temperature.',
        _trees(),
        context_json=json.dumps(
            {'gps_waypoints': '48.2848,11.6077,0.0; 48.2851,11.6074,1.57'}
        ),
    )

    assert 'id' not in reasoner.platform
    assert reasoner.platform['max_range_m'] == 1000
    assert reasoner.platform['max_speed_ms'] == 1.0
    assert reasoner.platform['max_probe_depth_cm'] == 200
    assert reasoner.system_description['autopilot']['mode_transition'] == ['HOLD', 'GUIDED']
    assert reasoner.system_description['autopilot']['required_armed'] is True
    assert reasoner.system_description['autopilot']['connection']['fcu_url'] == (
        'udp://@192.168.2.2:14600'
    )
    assert result.status_code == ACCEPT
    assert result.candidate_trees == ['blueboat_temperature_logging.xml']


def test_selects_gps_temperature_tree_for_named_lake_route():
    result = _reasoner().validate(
        'Drive a roundtrip around Hollerner Lake and log temperature.',
        _trees(),
    )

    assert result.status_code == ACCEPT
    assert result.candidate_trees == ['gps_temperature_logging.xml']
    assert 'navigation.gps_waypoints' in result.matched_capabilities


def test_named_lake_route_includes_plain_gps_navigation_tree():
    result = _reasoner().validate(
        'Drive a roundtrip around Hollerner Lake.',
        _trees(),
    )

    assert result.status_code == ACCEPT
    assert 'gps_waypoint_navigation.xml' in result.candidate_trees
    assert 'navigation.gps_waypoints' in result.matched_capabilities


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


def test_refuses_low_battery_from_context_before_tree_selection():
    result = _reasoner().validate(
        'Cover the marked field now.',
        _trees(),
        context_json=json.dumps(
            {
                'battery_percent': 12.0,
                'minimum_start_battery_percent': 20.0,
            }
        ),
    )

    assert result.status_code == REFUSE
    assert result.reasoning['guard'] == 'context.battery_admission'


def test_clarifies_cross_source_checkpoint_conflict():
    result = _reasoner().validate(
        'Drive to checkpoint K and log temperature.',
        _trees(),
        context_json=json.dumps(
            {
                'map_checkpoint_K': {'x': 20.0, 'y': 10.0},
                'gps_checkpoint_K_converted_to_map': {'x': 220.0, 'y': 110.0},
                'allowed_position_disagreement_m': 5.0,
            }
        ),
    )

    assert result.status_code == CLARIFY
    assert result.reasoning['guard'] == 'context.cross_source_consistency'


def test_refuses_named_target_outside_allowed_polygon():
    result = _reasoner().validate(
        'Go to P9 and take route photographs on the way.',
        _trees(),
        context_json=json.dumps(
            {
                'P9': {'x': 80.0, 'y': 80.0},
                'allowed_polygon': [
                    [0.0, 0.0],
                    [50.0, 0.0],
                    [50.0, 50.0],
                    [0.0, 50.0],
                ],
            }
        ),
    )

    assert result.status_code == REFUSE
    assert result.reasoning['guard'] == 'context.geofence'
