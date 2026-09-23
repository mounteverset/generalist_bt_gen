from pathlib import Path

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


def test_rejects_gps_segment_crossing_osm_stairs():
    findings = deterministic_plan_findings(
        {
            'payload_json': {
                'gps_waypoints': (
                    '48.28485,11.60695,0.0,0.0; '
                    '48.28475,11.60715,0.0,0.0'
                )
            },
            'context_snapshot': {
                'OSM_CONTEXT': {
                    'center': {'lat': 48.2848, 'lon': 11.6071},
                    'radius_m': 1200.0,
                    'steps_features': [
                        {
                            'osm_id': 485792666,
                            'coordinates': [
                                {'lat': 48.2848323, 'lon': 11.6070147},
                                {'lat': 48.2848002, 'lon': 11.6070885},
                            ],
                        }
                    ]
                }
            },
        }
    )

    assert findings[0]['guard'] == 'osm_steps'
    assert findings[0]['severity'] == 'critical'
    assert findings[0]['waypoint_indices'] == [1, 2]
    assert 'preserve unaffected route sections' in findings[0]['recommended_fix']
    assert '485792666' in findings[0]['description']


def test_llm_reviewer_sees_stairway_findings_before_deciding():
    source = (
        Path(__file__).resolve().parents[1] / 'plan_reviewer' / 'node.py'
    ).read_text()
    review_method = source.split('def _review_with_llm(', 1)[1].split(
        'def _invoke_llm(', 1
    )[0]

    assert review_method.index("review_input['deterministic_findings']") < (
        review_method.index('self._invoke_llm(')
    )
    assert 'For osm_steps, identify the affected waypoint segment' in source
