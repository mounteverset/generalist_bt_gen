# BT_executor 

This package extends behaviortree.ros2 `TreeExecutionServer` in its own class called `GeneralistBehaviorTreeServer`. We assume for now that the User Command is send via the action goal call. How this information then gets used and processed inside the tree to be written on the blackboard remains as open TODO in this package.

Overriding the following methods with the following purpose:

| Method | Purpose |
| --- | --- |
| `void onTreeCreated(BT::Tree& tree)` | set the goal payload of the action call goal to "user_command" on the blackboard | 
| `std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status)` | t.b.d | 
| `void onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)` | If the tree failed we trigger the extension of the tree here by returning a custom error message to the action client | 
| `std::optional<std::string> onLoopFeedback()` | The feedback is used to display information in the user interface on the webpage, active node for example, blackboard status stuff like this | 

--- 

### ROS node

bt_executor_node -> Creates an instance of the Action Server Class and spins it

## Action Plugin import and BT import

We import the robot skill plugins from `robot_actions` (configured via action server parameters). LLM-facing logic lives entirely outside the BT, inside `mission_coordinator`, so no `llm_actions` plugin set is required. The behavior tree and its subtrees live inside this package in `trees/`.

## Parameters

Default configuration lives in `config/bt_executor_params.yaml` and can be loaded via:

```bash
ros2 run bt_executor bt_executor_node --ros-args --params-file $(ros2 pkg prefix bt_executor)/share/bt_executor/config/bt_executor_params.yaml
```

Key parameters:

| Name | Description | Default |
| --- | --- | --- |
| `action_name` | Action server name exposed to mission coordinator. | `/bt_executor/execute_tree` |
| `tree_file` | Relative path to the BT XML file to load. | `trees/demo_tree.xml` |
| `plugin_directories` | Shared library directories for `robot_actions` plugins (registered via `registerNodesIntoFactory`). | `[install/robot_actions/lib]` |
| `status_topic` / `active_node_topic` | Topics publishing textual status + active subtree for UI/mission coordinator. | `/mission_coordinator/status_text`, `/mission_coordinator/active_subtree` |
| `auto_restart_on_failure` | When `false`, the BT stops on failures so the mission coordinator can react; set `true` for continuous retries. | `false` |
| `publish_groot_stream` | Enable Groot2 ZMQ logging. | `true` |
| `feedback_rate_hz` | Frequency for `onLoopFeedback` publishing. | `5.0` Hz |
| `log_directory` | Directory where executor logs/telemetry are written. | `~/.generalist_bt/executor_logs` |

## Launching

Use the provided launch file to start the executor with the default configuration:

```bash
ros2 launch bt_executor bt_executor.launch.py
```

This loads `config/bt_executor_params.yaml`, which points the server at the demo tree (`trees/demo_tree.xml`) and registers any plugin libraries found under `robot_actions/lib`. Adjust the YAML to add more plugin directories or behavior-tree bundles as they become available.
| `auto_restart_on_failure` | When `false`, the BT stops on failures so the mission coordinator can react; set `true` for continuous retries. | `false` |
| `plugin_directories` | Shared library directories for `robot_actions` plugins (registered via `registerNodesIntoFactory`). | `install/robot_actions/lib` |
| `status_topic` / `active_node_topic` | Topics publishing textual status + active subtree for the UI/mission coordinator. | `/mission_coordinator/status_text`, `/mission_coordinator/active_subtree` |
