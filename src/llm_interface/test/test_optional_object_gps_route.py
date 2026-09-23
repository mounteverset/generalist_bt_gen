from llm_interface.payload_validation import generated_payload_errors, osm_five_tree_route
from pathlib import Path

import yaml


def test_object_route_accepts_empty_or_valid_gps_leg_and_rejects_invalid_one():
    contract = {
        'waypoints': {'type': 'string', 'required': True},
        'gps_waypoints': {'type': 'string', 'required': True},
    }
    payload = {'waypoints': '8.0,4.0,0.0', 'gps_waypoints': ''}

    assert not generated_payload_errors(payload, contract)
    assert "missing required key 'gps_waypoints'" in generated_payload_errors(
        {'waypoints': payload['waypoints']}, contract
    )
    payload['gps_waypoints'] = '48.2848,11.6077,0.0'
    assert not generated_payload_errors(payload, contract)
    payload['gps_waypoints'] = '91.0,11.6077,0.0'
    assert any('latitude is outside' in error for error in generated_payload_errors(payload, contract))


def test_object_route_uses_distinct_osm_trees_only_when_find_anything_is_empty():
    metadata = yaml.safe_load((Path(__file__).resolve().parents[3] / 'config/tree_metadata.yaml').read_text())
    contract = next(
        tree['blackboard_contract'] for tree in metadata['trees']
        if tree['id'] == 'find_and_drive_to_nearest_object.xml'
    )
    context = {
        'REQUEST_HINTS': {'MISSION_REQUEST': {'mission_text': 'Visit two mapped trees'}},
        'OSM_CONTEXT': {
            'center': {'lat': 48.2848, 'lon': 11.6077},
            'radius_m': 500,
            'steps_features': [],
            'tree_features': [
                {'center': {'lat': 48.2848, 'lon': 11.6077}},
                {'center': {'lat': 48.2849, 'lon': 11.6078}},
            ],
        }
    }
    payload = {'waypoints': '', 'gps_waypoints': '48.2848,11.6077,0.0; 48.2849,11.6078,0.0'}
    assert generated_payload_errors(payload, contract, context) == []

    payload['waypoints'] = ' '
    assert any('exactly empty' in error for error in generated_payload_errors(payload, contract, context))
    payload['waypoints'] = ''

    payload['gps_waypoints'] = ''
    assert any('object route needs' in error for error in generated_payload_errors(payload, contract, context))
    payload['gps_waypoints'] = '48.2848,11.6077,0.0; 48.2848,11.6077,0.0'
    assert any('distinct OSM tree' in error for error in generated_payload_errors(payload, contract, context))
    payload['gps_waypoints'] = '48.2848,11.6077,0.0; 48.2850,11.6080,0.0'
    assert any('distinct OSM tree' in error for error in generated_payload_errors(payload, contract, context))

    context['FIND_ANYTHING'] = {'locations': [{'point': {'x': 8.0, 'y': 4.0}}]}
    assert any('FindAnything locations are available' in error for error in generated_payload_errors(payload, contract, context))
    context.pop('FIND_ANYTHING')
    context['REQUEST_HINTS']['MISSION_REQUEST']['mission_text'] = 'Find the red toolbox'
    assert any('only valid for tree requests' in error for error in generated_payload_errors(payload, contract, context))


def test_empty_five_tree_plan_falls_back_to_osm_centers():
    context = {
        'REQUEST_HINTS': {'MISSION_REQUEST': {'mission_text': 'Visit five mapped trees'}},
        'OSM_CONTEXT': {'tree_features': [
            {'center': {'lat': 48.2848 + i * 0.0001, 'lon': 11.6077}}
            for i in range(5)
        ]},
    }
    route = osm_five_tree_route(context, 'Visit five mapped trees')
    assert len(route.split(';')) == 5
    assert route.startswith('48.28480000,11.60770000,0.0')
    context['FIND_ANYTHING'] = {'locations': [{'point': {'x': 8.0, 'y': 4.0}}]}
    assert osm_five_tree_route(context, 'Visit five mapped trees') == ''
