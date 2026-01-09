# Subtree Metadata Schema

This file defines metadata for behavior trees, including context requirements and blackboard contracts.

## Schema Format

```yaml
trees:
  - id: "tree_filename.xml"
    description: "Human-readable description of what this tree does"
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
| `GPS_FIX` | GPS coordinates | `/gps/fix` |
| `BATTERY_STATE` | Battery percentage and voltage | `/battery_state` |
| `SEMANTIC_MAP` | Semantic map query result | Custom service |
