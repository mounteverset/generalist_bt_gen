# `mission_coordinator` package

The mission_coordinator is responsible for sending the user request coming from the web/chat UI via a action call to a LLM, provide context of available sensor sources and information about available robot action behavior trees. 
After receiving a behavior tree decision which tree to execute the mission coordinator calls the `bt_executor` action server with the respective tree to call. The payload is needed context for the execution of the tree, this could for example be waypoints or a tree species which is the target to photograph. The payload will be parsed from JSON to populate the blackboard of the `bt_executor` for execution. 

The mission coordinator should have the possibility to be somewhat configurable during run-time with dynamic reconfigure by the user interface. E.g. the LLM provider should be changeable by the UI, it should exist an option to toggle if the mission_coordinator waits for an acknowledge by the user before sending an action goal to execute a BT. Also at some point we want to use memori inside of the LLM interface app, this should also be an option to select or de-select.

## Parameters & Defaults

Default values live in `config/mission_coordinator_params.yaml` and can be supplied via `--ros-args --params-file`. Key parameters:

| Parameter | Description | Default |
| --- | --- | --- |
| `mission_action_name` | Action name exposed to the UI (`MissionCommand.action`). | `/mission_coordinator/execute_tree` |
| `llm_plan_service` | Service to request BT XML/subtree plans from `llm_interface`. | `/llm_interface/plan_subtree` |
| `bt_executor_action` | Action client target on the BT executor. | `/bt_executor/execute_tree` |
| `context_snapshot_service` | Optional sensor-context capture service. | `/context_gatherer/snapshot` |
| `status_topic` | Status text published for UI consumption. | `/mission_coordinator/status_text` |
| `active_subtree_topic` | Current subtree identifier being executed. | `/mission_coordinator/active_subtree` |
| `operator_decision_service` | Alias service for operator approve/reject; pending plans include a coordinator-specific endpoint to avoid conflicts across duplicate launches. | `/mission_coordinator/operator_decision` |
| `enable_context_snapshot` | Toggle context gathering before LLM call. | `true` |
| `require_operator_accept` | If true, wait for UI acknowledgement before running BT (can be bypassed per mission with `context_json.auto_execute=true`). | `true` |
| `demo_mode` | Skip ROS calls and emit mock responses (dev aid). | `true` |
| `llm_timeout_sec` | Timeout for LLM planning service. | `45.0` |
| `bt_timeout_sec` | Timeout for BT executor action. | `120.0` |
| `spin_period_sec` | Main executor spin period. | `0.1` |
| `transcript_directory` | Directory for mission transcripts/logs. | `~/.generalist_bt/mission_logs` |
