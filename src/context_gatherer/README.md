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
| `FIND_ANYTHING` | Object-location service result, with optional WGS84 coordinates from `/toLL` for map-frame detections |
| `RGB360SWEEP` | Images produced by the internal `360_rgb_sweep.xml` routine |

With `rgb360_sweep_camera_topic: auto`, the sweep checks for publishers on
`/a200_0000/sensors/camera_0/color/image` and `/okvis/rgb2/image_raw`. If both
are present, `use_sim_time` selects the simulation camera when true and OKVIS
when false. An explicit camera topic parameter overrides this selection.

`GPS_FIX` is required to contain a current, nonzero fix with a valid receiver
status. Missing, invalid, or older-than-`gps_fix_max_age_sec` samples abort the
gather action with a reason that the mission coordinator publishes to the UI.

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

Closed area ways and multipolygon relations with `landuse`, `leisure`,
`natural`, `amenity`, or `tourism` tags (plus surface-tagged non-highway ways)
are available in `area_features`, nearest first after named water bodies.
Each selected area keeps its tags and complete boundary rings. Way ring
coordinates include OSM node IDs when Overpass supplies them; relation rings
also retain member way IDs and outer/inner roles. Relation member node IDs are
not present in standard `out geom` responses, so their coordinates are kept
without invented IDs.

All queried `highway=steps` ways are retained separately in `steps_features`,
regardless of the linear route limit. Their tags and complete, node-ID-labelled
geometry let GPS payload validation reject route segments within 5 m of stairs.
An older OSM context without `steps_features` is rejected for GPS payloads;
an unavailable context or a route outside the queried radius is rejected too.
Gather fresh context before retrying or refining a route.

Dedicated nearest-first collections expose up to 100 individual trees, 20
waste baskets, and 40 mission targets (benches, toilets, shelters, drinking
water, picnic tables, bicycle parking, recycling, showers, and BBQ sites).
Environmental context includes tree rows/groups, hedges, woods, scrub,
meadows, wetlands/reedbeds, shorelines, and streams.

Responses are bounded by `overpass_max_linear_features` (also the area count
limit). `overpass_max_coordinates_per_feature` applies to sampled linear and
other non-area geometry, not selected area boundaries. For lake missions,
linear features are selected by distance to the retrieved water geometry plus
a traversability penalty before truncation. This prevents nearby source-order roads from
displacing shore paths and useful gravel/compacted route segments. Steps and
poor-surface paths remain present as avoid/verify context rather than being
silently treated as safe.

OSM coordinates are reasoning input, not map-frame goals. GPS trees convert the
selected route into the `gps_waypoints` payload; generic `waypoints` remains
map-frame `x,y,yaw`.

Relevant parameters live in `config/context_gatherer_params.yaml`, including
the primary/fallback Overpass endpoints, timeouts, radius, feature limits, and
MapTiler overview/detail settings.
