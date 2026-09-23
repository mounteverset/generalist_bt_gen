from llm_interface.payload_validation import generated_payload_errors


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
