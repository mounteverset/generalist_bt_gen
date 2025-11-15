# BT Executor Implementation Plan (MVP)

Goal: deliver the first version of `bt_executor` that wraps `behaviortree_ros2::TreeExecutionServer`, exposes a ROS 2 action endpoint for `MissionCommand` execution requests, and provides deterministic hooks for tree lifecycle events (blackboard seeding, telemetry, failure detection). Context: `architecture.md`, `project_roadmap.md`, and package README.

## 1. Components & Responsibilities

| Component | Purpose | Notes |
| --- | --- | --- |
| `GeneralistBehaviorTreeServer` (C++) | Subclass of `behaviortree_ros2::TreeExecutionServer` implementing hook overrides (`onTreeCreated`, `onLoopAfterTick`, `onTreeExecutionCompleted`, `onLoopFeedback`). | Responsible for blackboard seeding (`user_command`), failure detection, logging feedback for the mission coordinator / UI. |
| `bt_executor_node` | Thin `rclcpp::Node` that instantiates `GeneralistBehaviorTreeServer`, loads parameters (tree path, plugin directories, action namespace), and spins. | Launchable via `ros2 run` or `ros2 launch`. |
| Tree assets (`trees/*.xml`) | Initial behavior tree(s) distributed with the package. | Stubs for now; must be loadable by the server. |
| Parameters YAML | Defines plugin directories, tree file, action names, telemetry topics, failure-handling toggles. |
| Telemetry publishers | Provide textual status + active node info for UI. | Could reuse `status_topic` / `active_subtree_topic`. |

### Dependency Notes (Task 1)

- **Core libraries**: `behaviortree_cpp_v4` (BT engine), `behaviortree_ros2` (TreeExecutionServer), `rclcpp`, `rclcpp_action`, `std_msgs`, `ament_index_cpp`.
- **Plugin packages**: `robot_actions` (BT skill plugins loaded from shared libraries via TreeExecutionServer params).
- **Interface packages**: `gen_bt_interfaces` (MissionCommand action for coordination), potentially `behaviortree_ros2_msgs` for executor telemetry if needed.
- **Utilities**: `nlohmann_json` (optional for structured feedback), `yaml-cpp` if we parse configs manually (TreeExecutionServer already supports YAML).

## 2. Tasks (Tiny, Testable Steps)

| # | Task | Context/Input | Expected Output |
| --- | --- | --- | --- |
| 1 | Confirm BT.ROS2 dependency graph | README + architecture specs | Notes on required dependencies (`behaviortree_cpp`, `behaviortree_ros2`, `rclcpp`, `std_msgs`, plugin packages). |
| 2 | Author parameter manifest | README + architecture | Create `config/bt_executor_params.yaml` describing tree file path, plugin dirs, status topics, action QoS. ✅ |
| 3 | Scaffold `GeneralistBehaviorTreeServer` class | Task 1 output | Header + source with class definition, constructors, override stubs logging to `RCLCPP_INFO`. ✅ |
| 4 | Implement `onTreeCreated` | Task 3 | Copy mission goal payload to blackboard entry `user_command`; log tree root details. ✅ |
| 5 | Implement `onLoopAfterTick` | Task 3 | Track tick result, detect failure thresholds, optionally request stop/regeneration (placeholder returning empty optional). ✅ |
| 6 | Implement `onLoopFeedback` | Task 3 | Produce JSON or text feedback (active node, status) for mission/UI topics. ✅ |
| 7 | Implement `onTreeExecutionCompleted` | Task 3 | Map BT outcome -> action result codes; include failure reason for mission coordinator. ✅ |
| 8 | Build `bt_executor_node` | Task 3 | `main.cpp` that loads params, instantiates server, spins. ✅ |
| 9 | Wire plugin & tree loading | Task 8 | Parse parameters for plugin directories, XML path; pass to server constructor. ✅ |
|10 | Add telemetry publishers | Task 6 | `rclcpp::Publisher<std_msgs::msg::String>` for `/mission_coordinator/status_text`, `/mission_coordinator/active_subtree`. ✅ |
|11 | Tree asset stub | README guidance | Provide starter XML (e.g., `trees/demo_tree.xml`) to allow server to load. ✅ |
|12 | Config + launch files | Task 2 | Install YAML + `launch/bt_executor.launch.py` referencing tree + plugin params. ✅ |
|13 | Documentation updates | All | Update `README.md` with build/run instructions and hook behavior. ✅ |
|14 | Build/test verification | All | `colcon build --packages-select bt_executor` + simple `ros2 run bt_executor bt_executor_node` smoke test (with demo tree). ✅ |

### Stretch (later phases)
- Integrate regeneration triggers (call mission coordinator service when onLoopAfterTick detects failure).
- Add ROS services to reload tree files or plugin manifests at runtime.
- Export telemetry via `diagnostic_msgs`.

This plan keeps tasks granular so we can iterate quickly while aligning with the architecture’s requirement that `bt_executor` focus purely on BT runtime concerns. Subsequent tasks (LLM repair, mission coordination) happen outside this package.*** End Patch
