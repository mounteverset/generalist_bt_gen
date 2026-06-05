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

## Example Trees

```yaml
trees:
  - id: "navigate_and_photograph.xml"
    description: "Navigate to waypoints and take photos at regular intervals"
    mission_intents:
      - "navigate_waypoints"
      - "photograph_route"
      - "document_area"
    required_capabilities:
      - "locomotion.ground"
      - "navigation.waypoints"
      - "sensing.rgb_image"
      - "localization.gps"
    unsupported_requirements:
      - "locomotion.flight"
      - "sensing.thermal_image"
    selection_constraints:
      max_range_m: 5000
      requires_target_area_or_route: true
    context_requirements:
      - ROBOT_POSE
      - RGB_IMAGE
      - GPS_FIX
    blackboard_contract:
      waypoints:
        type: "geometry_msgs/PoseArray"
        required: true
      photo_interval_m:
        type: "double"
        required: false
        default: 5.0
      camera_topic:
        type: "string"
        required: false
        default: "/camera/image_raw"

  - id: "explore_area.xml"
    description: "Autonomous exploration of an unknown area with obstacle avoidance"
    mission_intents:
      - "explore_area"
      - "survey_area"
    required_capabilities:
      - "locomotion.ground"
      - "navigation.waypoints"
      - "localization.odometry"
      - "mapping.slam"
    context_requirements:
      - ROBOT_POSE
      - POINTCLOUD
      - BATTERY_STATE
    blackboard_contract:
      exploration_radius_m:
        type: "double"
        required: false
        default: 50.0
      min_battery_percent:
        type: "double"
        required: false
        default: 20.0
      return_home_on_low_battery:
        type: "bool"
        required: false
        default: true

  - id: "demo_tree.xml"
    description: "Simple demo tree for testing the system"
    mission_intents:
      - "navigate_waypoints"
      - "log_temperature"
    required_capabilities:
      - "locomotion.ground"
      - "navigation.waypoints"
      - "sensing.temperature"
      - "payload.parse_waypoints"
    context_requirements:
      - ROBOT_POSE
    blackboard_contract:
      test_message:
        type: "string"
        required: false
        default: "Hello from demo tree"
```

## Supported Context Requirements

| Requirement | Description | Data Source |
|-------------|-------------|-------------|
| `ROBOT_POSE` | Current robot position and orientation | `/odom` or `/tf` |
| `RGB_IMAGE` | Latest RGB camera image | `/camera/image_raw` |
| `DEPTH_IMAGE` | Latest depth image | `/camera/depth/image_raw` |
| `POINTCLOUD` | 3D point cloud | `/velodyne_points` or similar |
| `SATELLITE_MAP` | Annotated satellite or aerial image with geo metadata | External tile/image source + `satellite_map_annotator` |
| `GPS_FIX` | GPS coordinates | `/gps/fix` |
| `BATTERY_STATE` | Battery percentage and voltage | `/battery_state` |
| `SEMANTIC_MAP` | Semantic map query result | Custom service |

## Capability Fields

| Field | Description |
|-------------|-------------|
| `mission_intents` | Normalized mission classes that the reasoner can use for explanations and future extraction. |
| `required_capabilities` | Capability IDs that must be supported by `config/system_description.yaml` for this tree to remain a candidate. |
| `unsupported_requirements` | Optional common capability mismatches that should produce specific refusal messages. |
| `selection_constraints` | Static limits such as maximum range or required target information. Dynamic checks still belong in context gathering or payload validation. |
