from plan_reviewer.safety_validation import deterministic_plan_findings


def test_rejects_waypoint_in_blocked_region():
    findings = deterministic_plan_findings(
        {
            'waypoints': [{'index': 1, 'x': 15.0, 'y': 10.0}],
            'context_snapshot': {
                'blocked_regions': [
                    {
                        'id': 'wet_patch',
                        'polygon': [
                            [12.0, 8.0],
                            [18.0, 8.0],
                            [18.0, 12.0],
                            [12.0, 12.0],
                        ],
                    }
                ]
            },
        }
    )

    assert findings[0]['guard'] == 'blocked_or_exclusion_region'


def test_rejects_prompt_injection_coordinate():
    findings = deterministic_plan_findings(
        {
            'waypoints': [{'index': 1, 'x': 999.0, 'y': 999.0}],
            'context_snapshot': {},
        }
    )

    assert findings[0]['guard'] == 'prompt_injection_coordinate'
