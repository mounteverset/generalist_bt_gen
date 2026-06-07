# BT_executor 

This package extends behaviortree.ros2 `TreeExecutionServer` in its own class called `GeneralistBehaviorTreeServer`. We assume for now that the User Command is send via the action goal call. How this information then gets used and processed inside the tree to be written on the blackboard remains as open TODO in this package.

Overriding the following methods with the following purpose:

| Method | Purpose |
| --- | --- |
| `void onTreeCreated(BT::Tree& tree)` | Parse the goal payload JSON, store it as `user_command`, seed the waypoint queue + logfile path on the blackboard. | 
| `std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status)` | Publishes textual status + currently active node and optionally halts the tree when failures are not auto-restarted. | 
| `void onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)` | Publishes a final status string and forwards an actionable result back to mission clients. | 
| `std::optional<std::string> onLoopFeedback()` | Lightweight feedback hook used by the UI to display executor state. | 

Mission payloads are parsed centrally in `executor_utils.*`: all top-level JSON keys are mirrored onto the BT blackboard (nested objects are flattened using dot-notation), so LLM-authored behavior trees can rely on payload-provided parameters without additional C++ glue. Specialized helpers (e.g., waypoint queue extraction) also live there to keep the action server compact.

--- 

### ROS node

bt_executor_node -> Creates an instance of the Action Server Class and spins it

## Action plugin import and BT bundles

We import the robot skills from the `robot_actions` plugin library which extends BT.CPP with ROS 2 action/service nodes (`MoveTo`, `LogTemperature`, ...). The directories holding `.so` plugins are provided through the `plugin_directories` parameter (see below). Behavior tree XML bundles live under `trees/` in this package and are made discoverable through the `behavior_trees` parameter.

## Parameters

Default configuration lives in `config/bt_executor_params.yaml` and can be loaded via:

```bash
ros2 run bt_executor bt_executor_node --ros-args --params-file $(ros2 pkg prefix bt_executor)/share/bt_executor/config/bt_executor_params.yaml
```

Key parameters:

| Name | Description | Default |
| --- | --- | --- |
| `action_name` | Action server name exposed to mission coordinator. | `/bt_executor/execute_tree` |
| `plugin_directories` | Additional directories scanned for ROS BT plugins. Accepts absolute paths or `package/subfolder` entries resolved relative to the package prefix. | `["robot_actions/lib"]` |
| `nav2_action_name` | Default action server name used by the `MoveTo` BT node (can still be overridden per-node via the `action_name` input port). | `"/navigate_to_pose"` |
| `log_temperature_service_name` / `take_photo_image_topic` / `get_current_pose_pose_topic` / `distance_traveled_odom_topic` / `get_current_pose_odom_topic` / `find_anything_service_name` | Default service/topic names used by the respective `robot_actions` sensing/perception nodes. `TakePicture`/`TakePhoto`, `DistanceTraveled`, and `GetCurrentPose` can override image/pose/odometry topics with input ports. `FindObjectLocation`/`FindAnything` can override the service with the standard `service_name` port. | `"/log_temperature"`, `"/a200_0000/sensors/camera_0/color/image"`, `"/a200_0000/pose"`, `"/a200_0000/platform/odom/filtered"`, `"/a200_0000/platform/odom/filtered"`, `"/language_processor/find_object_locations"` |
| `behavior_trees` | List of `package/subfolder` entries that contain BT XML files to pre-register. | `["bt_executor/trees"]` |
| `status_topic` / `active_node_topic` | Topics publishing textual status + active subtree for UI/mission coordinator. | `/mission_coordinator/status_text`, `/mission_coordinator/active_subtree` |
| `enable_debug_logging` | Enables verbose BT execution logs (per-tick blackboard dump + robot action debug traces). Keep `false` for normal operation. | `false` |
| `auto_restart_on_failure` | When `false`, the BT stops on failures so the mission coordinator can react; set `true` for continuous retries. | `false` |
| `feedback_rate_hz` | Frequency for `onLoopFeedback` publishing. | `5.0` Hz |

## Launching

Use the provided launch file to start the executor with the default configuration:

```bash
ros2 launch bt_executor bt_executor.launch.py
```

This loads `config/bt_executor_params.yaml`, which points the server at the tree bundle (`trees/`) and registers the `robot_actions` plugin library discovered under `robot_actions/lib`.

## Temperature Logging Tree (`trees/temperature_logging.xml`)

The default behavior tree, `temperature_logging.xml`, executes a waypoint
temperature logging mission:

1. `ParseWaypoints` converts the raw mission payload string (written to `waypoints_raw` on the blackboard) into a `BT::SharedQueue<std::string>` called `waypoint_queue`.
2. `LoopString` iterates over `waypoint_queue`, handing each `"x,y,yaw"` entry to its child subtree.
3. Each iteration runs `MoveTo` with the current waypoint string, proxying to the Nav2 `NavigateToPose` action.
4. After reaching the waypoint, `LogTemperature` triggers a telemetry logging service call using the `logfile_path` also injected via the payload.

Mission payloads may provide waypoints either as arrays (e.g. `[1.0, 2.0, 0.0]`), objects (`{"x": 1.0, "y": 2.0, "yaw": 0.0}`), or ready-to-use `"x,y,yaw"` strings—the executor normalizes them into a string queue for the tree.

## Context sweep tree (`trees/360_rgb_sweep.xml`)

`360_rgb_sweep.xml` is a bounded active-context routine for future
`context_gatherer` integration. It reads the current map-frame pose with
`GetCurrentPose`, rotates in place with `MoveTo` using map-frame goals,
captures RGB images with `TakePhoto` at fixed yaws 0, 1.046, 2.093, 3.14, and
4.186 radians, then rotates through 5.233 and 6.283 radians without capturing
duplicate images.

Optional payload keys are `pose_topic`, `pose_timeout_ms`, `odom_topic`,
`odom_timeout_ms`, `camera_topic`, `photo_output_directory`, and
`image_timeout_ms`.
