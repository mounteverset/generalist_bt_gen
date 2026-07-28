# `context_gatherer`

`context_gatherer` serves `/context_gatherer/gather` and builds the structured
context requested by the selected behavior-tree metadata. It combines ROS sensor
state with map helpers and external geographic data, writes generated artifacts
under `/tmp/context_gatherer` by default, and returns JSON plus attachment URIs.

## Supported requirements

| Requirement | Result |
| --- | --- |
| `ROBOT_POSE`, `GPS_FIX`, `BATTERY_STATE` | Latest configured robot-state sample |
| `RGB_IMAGE`, `DEPTH_IMAGE` | Saved image artifact and metadata |
| `ANNOTATED_SLAM_MAP_IMAGE` | Pose-annotated SLAM map |
| `SATELLITE_MAP`, `SATELLITE_TILE` | MapTiler overview/detail artifacts and geographic bounds |
| `OSM_CONTEXT` | Overpass route lines, surfaces/access tags, barriers, water/area geometry, and landmarks |
| `FIND_ANYTHING` | Object-location service result |
| `RGB360SWEEP` | Images produced by the internal `360_rgb_sweep.xml` routine |

## OpenStreetMap route context

`OSM_CONTEXT` is centered on the current GPS fix or the mission's geographic
hint. The configured general Overpass radius is 750 m, but mission semantics can
increase the requested extent:

| Mission hint | Minimum extent |
| --- | --- |
| lake, pond, reservoir, Hollerner See | 1200 m |
| around, loop, perimeter, circumvent | 850 m |
| survey, explore, field, park, area | 350 m |
| other point missions | 150 m, raised to the configured 750 m minimum for Overpass |

The Overpass query includes highway geometry and route-relevant tags such as
`surface`, `smoothness`, `tracktype`, `access`, and barriers, plus water
relations/ways and selected landmarks. Each linear feature retains sampled
latitude/longitude geometry and its original OSM tags.

Responses are bounded by `overpass_max_linear_features` and
`overpass_max_coordinates_per_feature`. For lake missions, linear features are
selected by distance to the retrieved water geometry plus a traversability
penalty before truncation. This prevents nearby source-order roads from
displacing shore paths and useful gravel/compacted route segments. Steps and
poor-surface paths remain present as avoid/verify context rather than being
silently treated as safe.

OSM coordinates are reasoning input, not map-frame goals. GPS trees convert the
selected route into the `gps_waypoints` payload; generic `waypoints` remains
map-frame `x,y,yaw`.

Relevant parameters live in `config/context_gatherer_params.yaml`, including
the primary/fallback Overpass endpoints, timeouts, radius, feature limits, and
MapTiler overview/detail settings.
