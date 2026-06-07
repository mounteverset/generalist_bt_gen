# From BT.CPP Robot Action Plugins to a Running Mission

This guide documents the current path from implemented BehaviorTree.CPP robot
action plugins to a mission that can be selected, parameterized, and executed by
the running ROS 2 stack.

The runtime path is:

```text
robot_actions C++ plugin
  -> BT XML in bt_executor/trees
  -> tree metadata + capability metadata
  -> mission_reasoner validation
  -> llm_interface tree selection + payload generation
  -> mission_coordinator dispatch
  -> bt_executor loads XML + plugin library
  -> robot action/service servers
```

## 1. Implement The BT Action Plugin

Robot BT nodes live in `src/robot_actions`.

For a new node:

1. Add a header under `src/robot_actions/include/robot_actions/`.
2. Add the implementation under `src/robot_actions/src/`.
3. Add the `.cpp` file to the `add_library(robot_actions SHARED ...)` list in
   `src/robot_actions/CMakeLists.txt`.
4. Register the node in `src/robot_actions/src/plugin_registration.cpp`.

Use the existing node patterns:

| Pattern | Base class | Current examples |
| --- | --- | --- |
| ROS action client BT node | `BT::RosActionNode<...>` | `MoveTo` |
| ROS service client BT node | `BT::RosServiceNode<...>` | `LogTemperature`, `RestartNode` |
| ROS topic capture BT node | `BT::SyncActionNode` with an `rclcpp` subscription | `TakePicture` / `TakePhoto` |
| Local synchronous BT utility | `BT::SyncActionNode` | `ParseWaypoints` |

Important constraints:

- The CMake dependency is `behaviortree_cpp`, not `behaviortree_cpp_v4`.
- Keep BT port names stable; the XML tree and payload contract depend on them.
- For ROS action/service nodes, expose service/action names through standard
  BehaviorTree.ROS2 ports or default parameters where possible.
- If the node needs a default ROS interface name, follow the current pattern in
  `plugin_registration.cpp`: read or declare a node parameter and pass it through
  `BT::RosNodeParams.default_port_value`.

Example registration shape:

```cpp
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<robot_actions::MyAction>("MyAction", params);
}
```

After installation, the compiled plugin library is discovered as:

```text
install/robot_actions/lib/librobot_actions.so
```

## 2. Make The Plugin Discoverable By `bt_executor`

`bt_executor` scans plugin directories from its ROS parameters.

Current parameter:

```yaml
plugin_directories:
  - "robot_actions/lib"
```

This package-relative entry is resolved against the installed prefix, so it
finds `install/robot_actions/lib` after `colcon build`.

If you move plugin libraries or split them into another package, update:

- `src/bt_executor/config/bt_executor_params.yaml`
- `src/generalist_bringup/config/bt_executor_params.yaml`

The executor scans each `.so` in those directories and calls `BT::LoadPlugin`.
Successful startup logs include:

```text
Searching plugins in: .../install/robot_actions/lib
Registered plugin: librobot_actions.so
```

## 3. Use The Node In A BT XML File

Behavior tree XML files live in:

```text
src/bt_executor/trees/
```

They are installed to `share/bt_executor/trees` and discovered through:

```yaml
behavior_trees:
  - "bt_executor/trees"
```

The XML `BehaviorTree ID` must match the tree ID used by mission selection and
metadata. For example:

```xml
<root BTCPP_format="4" main_tree_to_execute="temperature_logging.xml">
  <BehaviorTree ID="temperature_logging.xml">
    <Sequence name="TemperatureLoggingMission">
      <ParseWaypoints raw_waypoints="{waypoints}"
                      waypoint_queue="{waypoint_queue}"
                      waypoint_count="{waypoint_count}" />
      <LoopString queue="{waypoint_queue}" value="{active_waypoint}" if_empty="SUCCESS">
        <Sequence name="VisitWaypoint">
          <MoveTo pose="{active_waypoint}" action_name="/a200_0000/navigate_to_pose"/>
          <LogTemperature logfile_path="{logfile_path}" />
        </Sequence>
      </LoopString>
    </Sequence>
  </BehaviorTree>
</root>
```

Current registered node IDs:

| XML node ID | Ports | Runtime dependency |
| --- | --- | --- |
| `MoveTo` | `pose`, optional `frame_id`, standard `action_name` | Nav2 `NavigateToPose` action |
| `TakePicture` / `TakePhoto` | inputs `image_topic`, `output_directory`, `filename_prefix`, `timeout_ms`; output `filepath` | RGB `sensor_msgs/Image` topic |
| `GetCurrentPose` | inputs `odom_topic`, `odom_timeout_ms`; outputs `current_x`, `current_y`, `current_yaw`, `current_pose`, `current_frame_id`, fixed-yaw `sweep_pose_*` strings | Odometry topic |
| `DistanceTraveled` | inputs `interval_m`, `odom_topic`, output `distance_accumulated_m` | Odometry topic |
| `FindObjectLocation` / `FindAnything` | inputs `object`, `max_results`, `default_yaw`, standard `service_name`; outputs `x`, `y`, `yaw`, `pose`, `frame_id` | `language_feature_msgs/FindObjectLocations` service |
| `LogTemperature` | input `logfile_path`, standard `service_name` | `std_srvs/Trigger` temperature service |
| `RestartNode` | input `node_name`, standard `service_name` | `std_srvs/SetBool` restart service |
| `ParseWaypoints` | `raw_waypoints`, `waypoint_queue`, `waypoint_count` | local BT utility |

## 4. Add Tree Metadata For Selection And Payload Generation

Every selectable tree needs an entry in `config/tree_metadata.yaml`.

The metadata connects the tree to three upstream stages:

- `mission_reasoner`: checks static robot capabilities before tree selection.
- `context_gatherer`: collects context required by the selected tree.
- `llm_interface/create_payload`: generates blackboard values that match the
  tree ports.

Minimum shape:

```yaml
trees:
  - id: "my_tree.xml"
    description: "Short human-readable mission description."
    mission_intents:
      - "navigate_waypoints"
    required_capabilities:
      - "locomotion.ground"
      - "navigation.waypoints"
    unsupported_requirements:
      - "locomotion.flight"
    selection_constraints:
      max_range_m: 5000
    context_requirements:
      - ROBOT_POSE
    blackboard_contract:
      waypoints:
        type: "string"
        required: true
        schema:
          type: "string"
          description: "Semicolon-separated x,y,yaw waypoints."
      logfile_path:
        type: "string"
        required: false
        default: "/tmp/temperature_log.txt"
```

Rules:

- `id` must match the XML `BehaviorTree ID` and the filename used by
  `ExecuteTree.Goal.target_tree`.
- `required_capabilities` must use IDs declared in
  `config/system_description.yaml`.
- `context_requirements` must use values supported by `context_gatherer`.
- `blackboard_contract` keys must match the blackboard keys used in XML ports,
  such as `{waypoints}` or `{logfile_path}`.

## 5. Add Robot Capabilities If The Action Introduces New Ones

Capabilities are declared in:

```text
config/system_description.yaml
```

Add new sensors, actions, supported capabilities, unsupported explanations, and
command keyword rules as needed.

Example:

```yaml
actions:
  - id: "photograph"
    capability: "sensing.rgb_image"
    type: "sensing"
    ros_interface: "/camera/capture"
    ros_type: "service"

capabilities:
  supported:
    - "sensing.rgb_image"

command_capability_rules:
  - capability: "sensing.rgb_image"
    keywords: ["photo", "photograph", "picture", "image"]
```

The reasoner may call `llm_interface/extract_mission_requirements` to parse the
command, but the final accept/clarify/refuse decision is made against this file.

## 6. Add The Tree To The Mission Coordinator Catalog

`mission_coordinator` only offers trees listed in its `known_trees` parameter.

Current file:

```text
src/mission_coordinator/config/mission_coordinator_params.yaml
```

Add:

```yaml
known_trees:
  - "my_tree.xml::Short description used by selection."
```

The reasoner receives this catalog plus metadata and may narrow it before the
LLM selection call.

## 7. Ensure Runtime ROS Interfaces Exist

Before running a BT that calls ROS action/service nodes, the backing ROS
interfaces must be available.

For the current temperature logging tree:

| BT node | Required server |
| --- | --- |
| `MoveTo` | `/a200_0000/navigate_to_pose` or the configured `nav2_action_name` |
| `LogTemperature` | `/log_temperature` |
| `TakePicture` / `TakePhoto` | `/a200_0000/sensors/camera_0/color/image` or configured `take_photo_image_topic` |
| `GetCurrentPose` | `/odom` or configured `get_current_pose_odom_topic` |
| `DistanceTraveled` | `/odom` or configured `distance_traveled_odom_topic` |
| `FindObjectLocation` / `FindAnything` | `/language_processor/find_object_locations` or configured `find_anything_service_name` |

Useful checks:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luke/generalist_bt_gen/install/setup.bash

ros2 action list
ros2 service list
```

For the integrated bringup, `dummy_log_temperature_node` provides
`/log_temperature`. Nav2 must come from the Husky simulation or robot bringup.

## 8. Build And Source

Build the packages affected by a new plugin/tree/metadata change:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luke/generalist_bt_gen
colcon build --packages-select robot_actions bt_executor mission_coordinator mission_reasoner llm_interface generalist_bringup
source install/setup.bash
```

If `gen_bt_interfaces` changed, include it and allow overriding the local
underlay:

```bash
colcon build --packages-select gen_bt_interfaces robot_actions bt_executor mission_coordinator mission_reasoner llm_interface generalist_bringup --allow-overriding gen_bt_interfaces
```

If interface generation fails inside `rosidl_generator_rs` while importing
Miniconda `em.py`, use the ROS/system Python path for the build:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luke/generalist_bt_gen/install/setup.bash
export PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/usr/lib/python3/dist-packages
export PYTHONNOUSERSITE=1
colcon build --packages-select gen_bt_interfaces robot_actions bt_executor mission_coordinator mission_reasoner llm_interface generalist_bringup --allow-overriding gen_bt_interfaces
```

## 9. Launch The System

Integrated stack:

```bash
source /opt/ros/jazzy/setup.bash
source /home/luke/generalist_bt_gen/install/setup.bash
ros2 launch generalist_bringup generalist_bringup.launch.py
```

BT executor only:

```bash
ros2 launch bt_executor bt_executor.launch.py
```

Use a timeout for launch files during automated checks because ROS nodes keep
spinning:

```bash
timeout 30s ros2 launch bt_executor bt_executor.launch.py
```

## 10. Send A Mission Through The Full Pipeline

Normal user entry is the CLI or web UI, both talking to
`/mission_coordinator/execute_tree`.

For direct action testing, send a `MissionCommand` goal:

```bash
ros2 action send_goal /mission_coordinator/execute_tree gen_bt_interfaces/action/MissionCommand \
  "{command: 'Drive to two waypoints and log temperature', session_id: 'manual-test-1', context_json: '{\"auto_execute\": true, \"waypoints\": \"1.0,0.0,0.0; 2.0,0.0,0.0\"}'}"
```

Expected pipeline:

1. `mission_reasoner` validates the mission against capabilities.
2. `llm_interface` selects a tree from the candidate catalog.
3. `context_gatherer` collects required context.
4. `llm_interface` creates the blackboard payload.
5. `mission_coordinator` sends `ExecuteTree` to `/bt_executor/execute_tree`.
6. `bt_executor` loads the XML, loads `librobot_actions.so`, injects payload
   keys onto the blackboard, and ticks the tree.

## 11. Directly Test The BT Executor

To isolate BT/plugin problems from LLM/reasoner/context issues, call the executor
directly:

```bash
ros2 action send_goal /bt_executor/execute_tree btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'temperature_logging.xml', payload: '{\"waypoints\":\"1.0,0.0,0.0; 2.0,0.0,0.0\", \"logfile_path\":\"/tmp/temperature_log.txt\"}'}"
```

If this works but the full mission fails, inspect reasoner metadata,
`known_trees`, context requirements, and the blackboard contract.

If this fails, inspect:

- plugin registration logs from `bt_executor`
- XML node names and port names
- required ROS action/service servers
- payload keys written to blackboard

## 12. Troubleshooting Checklist

- Plugin not found:
  - Confirm `install/robot_actions/lib/librobot_actions.so` exists.
  - Confirm `plugin_directories` includes `robot_actions/lib`.
  - Confirm `plugin_registration.cpp` registers the XML node ID.
- XML tree not found:
  - Confirm the XML is under `src/bt_executor/trees`.
  - Rebuild and source the workspace.
  - Confirm `behavior_trees` includes `bt_executor/trees`.
- Reasoner refuses unexpectedly:
  - Check `required_capabilities` in `config/tree_metadata.yaml`.
  - Check supported/unsupported capabilities in `config/system_description.yaml`.
  - Check LLM requirement extraction output in reasoner logs if enabled.
- LLM selects the wrong tree:
  - Check `known_trees` descriptions.
  - Check tree metadata descriptions and mission intents.
  - Check that the reasoner candidate list is not too broad.
- Payload missing BT port values:
  - Check `blackboard_contract` keys.
  - Check XML blackboard references such as `{waypoints}`.
  - Check `CreatePayload` reasoning and payload JSON.
- ROS action/service failure:
  - Use `ros2 action list` and `ros2 service list`.
  - Check configured names such as `nav2_action_name`,
    `log_temperature_service_name`, `take_photo_image_topic`,
    `distance_traveled_odom_topic`, and per-node
    `action_name`/`service_name`/`image_topic` ports.
