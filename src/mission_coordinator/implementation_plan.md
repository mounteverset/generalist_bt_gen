# Mission Coordinator MVP Implementation Plan

Precise, bite-sized tasks for the first implementation of the mission coordinator (aligned with `project_roadmap.md` v0.5). Each task lists the concrete input/context required and the expected outcome so that progress is objectively verifiable.

## 1. Node & Interface Blueprint

| Component | Purpose | Interfaces |
| --- | --- | --- |
| `MissionCoordinatorNode` (rclpy) | Central orchestrator that receives natural-language missions from the UI, calls the LLM layer for clarification/subtree generation, and hands the resulting BT XML to `bt_executor`. | - Provides action server `/mission_coordinator/execute_tree` implementing `MissionCommand.action`.<br>- Publishes `/mission_coordinator/status_text` (`std_msgs/msg/String`).<br>- Publishes `/mission_coordinator/active_subtree` (`std_msgs/msg/String`). |
| `MissionCommand.action` (in `gen_bt_interfaces`) | Contract between UI (`user_interface`) and mission coordinator. | Goal: `string command`, `string session_id`, `string context_json`.<br>Feedback: `string stage`, `string detail`.<br>Result: `bool accepted`, `string outcome_message`. |
| BT executor client | Mission coordinator acts as an action client to `/bt_executor/execute_tree` (`behaviortree_ros2_msgs/action/RunTree`). | Sends BT XML + plugin metadata and monitors feedback/result. |
| LLM interface client | Calls `/llm_interface/plan_subtree` (`llm_interface/srv/PlanSubtree`) to obtain BT XML/subtree metadata. | Provides NL command + optional context snapshot. |
| Context gatherer hook (optional) | Service client to `/context_gatherer/snapshot` to include sensor context before LLM call. | Returns JSON snapshot string or empty on failure. |

### Dependency Notes (Task 1)

- **Action/Service packages**: `gen_bt_interfaces/action/MissionCommand.action`, `std_msgs/msg/String`, `behaviortree_ros2_msgs/action/RunTree` (placeholder name for BT executor interface), `llm_interface/srv/PlanSubtree` (LLM), `context_gatherer/srv/Snapshot` (if enabled).
- **ROS client libraries**: MVP target is `rclpy` for rapid iteration; revisit `rclcpp` later if performance dictates.
- **Supporting packages**: `builtin_interfaces` (for timestamps in logs if needed), `ament_cmake` build, `rosidl_default_generators` for action.

## 2. Task Breakdown

| # | Task | Input | Outcome |
| --- | --- | --- | --- |
| 1 | Confirm dependency graph | Architecture + `project_roadmap.md` + existing `.action/.srv` files | Written note/TODO describing exact message/action packages to depend on (`behaviortree_ros2_msgs`, `llm_interface`, etc.). |
| 2 | Define `MissionCommand.action` | Task 1 notes | File `gen_bt_interfaces/action/MissionCommand.action` committed with fields above. |
| 3 | Wire action build system | Task 2 file | `gen_bt_interfaces/package.xml` + `CMakeLists.txt` updated to build/install the new action (rosidl macros). |
| 4 | Declare ROS parameters | Architecture + UI config | Parameter list (namespace, target service/action names, timeouts, bool flags) recorded in `config/mission_coordinator_params.yaml`. |
| 5 | Create node skeleton | Tasks 1–4 | `src/mission_coordinator/mission_coordinator_node.py` (or .cpp) instantiates node, loads params, sets up publishers, and logs startup. |
| 6 | Implement MissionCommand action server shell | Task 5 | Action server accepts/rejects goals, returns placeholder result without external calls; includes integration test or demo run. |
| 7 | LLM service client helper | Task 5 | Helper method/class that sends `PlanSubtree` request using command + optional context string; handles timeout/errors, returns future. |
| 8 | Context snapshot helper (optional) | Task 5 | Function that decides whether to call `/context_gatherer/snapshot` based on param `enable_context_snapshot`, returning JSON string (empty on failure). |
| 9 | BT executor action client helper | Task 5 | Wrapper that waits for `/bt_executor/execute_tree`, sends BT XML produced by LLM, and exposes callbacks for feedback/result. |
|10 | Pipeline wiring | Tasks 6–9 | Action goal flow: accept → gather context (if enabled) → call LLM → stream status feedback → call BT executor action. Feedback mirrored to `/mission_coordinator/status_text` and `MissionCommand` feedback. |
|11 | Active subtree/status publishers | Task 10 | Publish LLM-provided subtree name and executor feedback states to `/mission_coordinator/active_subtree` and `/mission_coordinator/status_text`. |
|12 | Error handling paths | Task 10 | Ensure failures (LLM/context/executor) send descriptive feedback and complete action result with `accepted=false` + message. |
|13 | Launch/config assets | Tasks 4–12 | Add `config/mission_coordinator_params.yaml` + `launch/mission_coordinator.launch.py` referencing node and parameter file. |
|14 | README + interface docs | Tasks 2–13 | Update `src/mission_coordinator/README.md` to describe new action, parameters, and how to run/test the MVP. |

Scope reminders:
- Only one BT execution per mission (no self-repair/regeneration loops yet).
- All ROS interface names must align with what `user_interface` already exports (`/mission_coordinator/...`).
- Each helper (Tasks 7–9) should include minimal logging/tests before being invoked by Task 10 to keep debugging focused.
