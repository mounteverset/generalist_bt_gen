# Subtree Metadata Schema

This file defines metadata for behavior trees, including context requirements and blackboard contracts.
It also declares the static capabilities used by `mission_reasoner` before
behavior-tree selection.

## Schema Format

```yaml
trees:
  - id: "tree_filename.xml"
    description: "Human-readable description of what this tree does"
    mission_intents:
      - "navigate_waypoints"  # Mission classes this tree can satisfy
    required_capabilities:
      - "locomotion.ground"   # Capability IDs declared in system_description.yaml
    unsupported_requirements:
      - "locomotion.flight"   # Optional explicit refusal hints
    selection_constraints:
      max_range_m: 5000       # Optional static limits checked before selection
    context_requirements:
      - REQUIREMENT_NAME  # Enum-like values that context_gatherer understands
    blackboard_contract:
      key_name:
        type: "string|int|double|Pose|PoseArray|..."
        required: true|false
        default: <value>  # Optional default value
```

## Current catalogue

The selectable tree IDs in `config/tree_metadata.yaml` must match both
`mission_coordinator.known_trees` and XML filenames under
`src/bt_executor/trees`.

| Tree ID | Required route field | Coordinate frame |
| --- | --- | --- |
| `temperature_logging.xml` | `waypoints` | map-frame `x,y,yaw` |
| `gps_waypoint_navigation.xml` | `gps_waypoints` | geographic latitude/longitude |
| `gps_temperature_logging.xml` | `gps_waypoints` | geographic latitude/longitude |
| `navigate_and_photograph.xml` | `waypoints` | map-frame `x,y,yaw` |
| `explore_area.xml` | `waypoints`, `area_polygon`, `frontiers` | map frame |

`360_rgb_sweep.xml` is an internal context-capture tree, so it has source XML
but is intentionally not in the selectable metadata/catalogue.

## GPS tree example

```yaml
trees:
  - id: "gps_waypoint_navigation.xml"
    description: "Follow a geographic OSM/GPS route without adding an unrelated sensing task."
    mission_intents:
      - "navigate_waypoints"
      - "navigate_gps_waypoints"
      - "geographic_route"
    required_capabilities:
      - "localization.gps"
      - "localization.odometry"
      - "locomotion.ground"
      - "navigation.waypoints"
      - "navigation.gps_waypoints"
      - "payload.parse_gps_waypoints"
    unsupported_requirements:
      - "locomotion.flight"
    selection_constraints:
      max_range_m: 5000
      requires_target_area_or_route: true
    context_requirements:
      - ROBOT_POSE
      - GPS_FIX
      - OSM_CONTEXT
      - SATELLITE_MAP
    blackboard_contract:
      gps_waypoints:
        type: "string"
        required: true
        schema:
          type: "string"
          description: "Semicolon-separated latitude,longitude[,yaw] points."
          examples:
            - "48.2848,11.6077,0.0; 48.2851,11.6074,1.57"
```

Do not put latitude/longitude into `waypoints`. That key is reserved for
map-frame meters and is consumed by `ParseWaypoints`/`MoveTo`. Geographic
routes use `gps_waypoints` and are consumed by
`ParseGpsWaypoints`/`MoveToGPS`.

## Supported Context Requirements

| Requirement | Description | Data Source |
|-------------|-------------|-------------|
| `ROBOT_POSE` | Current position and orientation | configured pose/odometry topics |
| `RGB_IMAGE` | Latest RGB camera image | configured RGB topic |
| `DEPTH_IMAGE` | Latest depth image | configured depth topic |
| `BATTERY_STATE` | Battery percentage and voltage | configured battery topic |
| `GPS_FIX` | Current latitude/longitude fix | configured NavSatFix topic |
| `ANNOTATED_SLAM_MAP_IMAGE` | SLAM map annotated with the robot pose | `annotated_map_saver` service |
| `SATELLITE_MAP` / `SATELLITE_TILE` | Overview/detail map artifacts with geographic metadata | MapTiler + `satellite_map_annotator` |
| `OSM_CONTEXT` | Routeable lines with surface/access data, water geometry, barriers, and relevant points | OpenStreetMap Overpass API |
| `FIND_ANYTHING` | Object-location candidates | configured FindAnything service |
| `RGB360SWEEP` | Active six-heading RGB context sweep | internal `360_rgb_sweep.xml` execution |

## Capability Fields

| Field | Description |
|-------------|-------------|
| `mission_intents` | Normalized mission classes that the reasoner can use for explanations and future extraction. |
| `required_capabilities` | Capability IDs that must be supported by `config/system_description.yaml` for this tree to remain a candidate. |
| `unsupported_requirements` | Optional common capability mismatches that should produce specific refusal messages. |
| `selection_constraints` | Static limits such as maximum range or required target information. Dynamic checks still belong in context gathering or payload validation. |
