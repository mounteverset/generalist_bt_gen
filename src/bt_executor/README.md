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
| `behavior_trees` | List of `package/subfolder` entries that contain BT XML files to pre-register. | `["bt_executor/trees"]` |
| `status_topic` / `active_node_topic` | Topics publishing textual status + active subtree for UI/mission coordinator. | `/mission_coordinator/status_text`, `/mission_coordinator/active_subtree` |
| `auto_restart_on_failure` | When `false`, the BT stops on failures so the mission coordinator can react; set `true` for continuous retries. | `false` |
| `feedback_rate_hz` | Frequency for `onLoopFeedback` publishing. | `5.0` Hz |

## Launching

Use the provided launch file to start the executor with the default configuration:

```bash
ros2 launch bt_executor bt_executor.launch.py
```

This loads `config/bt_executor_params.yaml`, which points the server at the demo tree bundle (`trees/`) and registers the `robot_actions` plugin library discovered under `robot_actions/lib`.

## Demo tree (`trees/demo_tree.xml`)

The default behavior tree, `MainTree`, executes a waypoint mission:

1. `LoopString` iterates over the `waypoint_queue` that `GeneralistBehaviorTreeServer` populates by parsing the action goal payload (`{"waypoints": [...]}`).
2. Each iteration runs `MoveTo` with the current waypoint string (`"x,y,yaw"`), which proxies to the Nav2 `NavigateToPose` action.
3. After reaching the waypoint, `LogTemperature` triggers a telemetry logging service call using the `logfile_path` also injected via the payload.

Mission payloads may provide waypoints either as arrays (e.g. `[1.0, 2.0, 0.0]`), objects (`{"x": 1.0, "y": 2.0, "yaw": 0.0}`), or ready-to-use `"x,y,yaw"` strings—the executor normalizes them into a string queue for the tree.
