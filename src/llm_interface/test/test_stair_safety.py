from llm_interface.payload_validation import generated_payload_errors


CONTRACT = {'gps_waypoints': {'type': 'string', 'required': True}}
STAIR = {
    'osm_id': 485792666,
    'tags': {'highway': 'steps', 'ramp': 'no', 'surface': 'concrete'},
    'coordinates': [
        {'lat': 48.2848323, 'lon': 11.6070147, 'osm_node_id': 4785581767},
        {'lat': 48.2848002, 'lon': 11.6070885, 'osm_node_id': 4785581766},
    ],
}
OSM_CONTEXT = {
    'center': {'lat': 48.2848, 'lon': 11.6071},
    'radius_m': 1200.0,
    'steps_features': [STAIR],
}


def test_gps_route_crossing_stairs_is_rejected_even_between_waypoints():
    payload = {
        'gps_waypoints': (
            '48.28485,11.60695,0.0,0.0; 48.28475,11.60715,0.0,0.0'
        )
    }
    context = {'OSM_CONTEXT': OSM_CONTEXT}

    errors = generated_payload_errors(payload, CONTRACT, context)

    assert any('segment 1' in error and '485792666' in error for error in errors)


def test_gps_route_clear_of_stairs_is_accepted():
    payload = {
        'gps_waypoints': (
            '48.28500,11.60670,0.0,0.0; 48.28520,11.60680,0.0,0.0'
        )
    }

    assert generated_payload_errors(
        payload, CONTRACT, {'OSM_CONTEXT': OSM_CONTEXT}
    ) == []


def test_waypoint_near_stairway_is_rejected_with_clearance():
    payload = {'gps_waypoints': '48.2848323,11.6070500,0.0,0.0'}

    errors = generated_payload_errors(payload, CONTRACT, {'OSM_CONTEXT': OSM_CONTEXT})

    assert any('within 5 m' in error for error in errors)


def test_old_or_malformed_stair_inventory_fails_closed():
    payload = {'gps_waypoints': '48.28500,11.60670,0.0,0.0'}

    assert 'gather fresh OSM context' in ' '.join(
        generated_payload_errors(payload, CONTRACT, {'OSM_CONTEXT': {}})
    )
    assert 'OSM_CONTEXT unavailable' in ' '.join(
        generated_payload_errors(
            payload, CONTRACT, {'OSM_CONTEXT': {'status': 'unavailable'}}
        )
    )
    assert 'lacks valid geometry' in ' '.join(
        generated_payload_errors(
            payload, CONTRACT,
            {'OSM_CONTEXT': OSM_CONTEXT | {'steps_features': [{'osm_id': 485792666}]}},
        )
    )


def test_route_outside_stairway_query_coverage_is_rejected():
    payload = {'gps_waypoints': '48.30000,11.60710,0.0,0.0'}

    errors = generated_payload_errors(payload, CONTRACT, {'OSM_CONTEXT': OSM_CONTEXT})

    assert any('outside OSM stairway query coverage' in error for error in errors)
