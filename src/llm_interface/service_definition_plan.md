# PlanSubtree Service Definition Plan

_Reference_: `architecture.md` (LLM orchestration) + `mission_coordinator/config/mission_coordinator_params.yaml` (parameter `llm_plan_service`).

## Purpose
Mission coordinator requires a deterministic ROS 2 service to request new BT subtrees or mission plans from the LLM stack. This service must capture mission intent, execution context, and recent failure data, then return BT XML (or identifiers) plus operator-facing summaries.

## Proposed Location
Define `PlanSubtree.srv` inside `gen_bt_interfaces/srv/` so both `mission_coordinator` (client) and `llm_interface` (server) share the contract.

## Request Fields
| Field | Type | Description |
| --- | --- | --- |
| `session_id` | `string` | Correlates requests with UI sessions / transcripts. |
| `mission_text` | `string` | Natural-language command supplied by the user or higher-level planner. |
| `context_snapshot` | `string` (JSON) | Serialized context (map state, robot pose, sensor cues) produced by `context_gatherer`. |
| `blackboard_state` | `string` (JSON) | Optional snapshot of executor blackboard for stateful planning. |
| `failure_report` | `string` (JSON) | Optional structure describing why the previous subtree failed. |
| `requested_capabilities` | `string[]` | Optional hints (e.g., `['NAV', 'LOG_TEMP']`). |

## Response Fields
| Field | Type | Description |
| --- | --- | --- |
| `status_code` | `uint8` | Enum: `0=SUCCESS`, `1=RETRY`, `2=ESCALATE`. |
| `bt_xml` | `string` | Behavior tree XML (full tree or subtree) ready for `bt_executor`. |
| `summary` | `string` | Human-readable plan explanation for UI/mission coordinator logs. |
| `tool_invocations` | `string` (JSON) | Trace of MCP/Tool calls executed by LangChain for auditing. |
| `reason` | `string` | Diagnostic info when `status_code != SUCCESS`. |

## Next Steps
1. Add `PlanSubtree.srv` to `gen_bt_interfaces/srv/` following the field list above.
2. Regenerate interfaces via `colcon build` to expose Python/C++ types.
3. Implement the service server in `llm_interface` and update `mission_coordinator` to call it instead of demo stubs.

---

# BehaviorTreeSelection Service

Mission coordinator also needs a lightweight service to pick an existing subtree (by ID) before attempting regeneration. This allows the LLM layer to quickly rank candidate BTs and avoids regeneration when a suitable tree already exists.

## Purpose
Given a user command + catalog of available BTs (with descriptions/usage instructions), decide which BT to execute. If no suitable subtree exists, respond with a failure code so the mission coordinator can proceed to generation via `PlanSubtree`.

## Proposed Location
Define `SelectBehaviorTree.srv` under `gen_bt_interfaces/srv/`.

## Request Fields
| Field | Type | Description |
| --- | --- | --- |
| `session_id` | `string` | Correlates UI interactions. |
| `user_command` | `string` | High-level command from operator. |
| `available_trees` | `string[]` | List of BT IDs (e.g., XML filenames or registry IDs). |
| `tree_descriptions` | `string[]` | JSON or markdown entries aligned with `available_trees`, containing purpose, required inputs, and usage notes. |
| `context_snapshot` | `string` (JSON) | Optional context to help ranking. |

## Response Fields
| Field | Type | Description |
| --- | --- | --- |
| `status_code` | `uint8` | Enum: `0=FOUND`, `1=NO_MATCH`, `2=ERROR`. |
| `selected_tree` | `string` | Name/ID of the BT to execute when `status_code=FOUND`. |
| `confidence` | `float32` | Confidence score (0–1) for UI display / telemetry. |
| `reason` | `string` | Explanation or failure reason. |

## Workflow Integration
1. Mission coordinator calls `SelectBehaviorTree` with the user command + inventory of known BT XML.
2. If `status_code=FOUND`, it directly sends that BT name to `bt_executor`.
3. If `status_code=NO_MATCH`, mission coordinator escalates to `PlanSubtree` to synthesize a new tree.

## Next Steps
1. Add `SelectBehaviorTree.srv` to `gen_bt_interfaces`.
2. Extend `llm_interface` node with a second service server handling this contract.
3. Update mission coordinator flow to query selection first, fallback to plan generation only when necessary.
