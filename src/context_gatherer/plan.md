# Context Gatherer Implementation Plan (v0.6 target)

Goal: ship a functional `context_gatherer` node that serves `GatherContext.action`, aggregates required sensor/geo context, and unblocks `mission_coordinator` + `llm_interface` to build a subtree payload before calling `bt_executor`.

## Milestones
1) **Package scaffolding** – Create C++ node skeleton with parameters, action server wiring for `GatherContext`. Stub feedback/result so mission_coordinator can connect without data. Add launch + params YAML.
2) **Context requirement registry** – Define enum/strings mapping (e.g., ROBOT_POSE, ODOM, RGB_IMAGE, POINTCLOUD, SATELLITE_TILE, LAST_FAILURE). Configurable per parameter. Reject unknown requirements early.
3) **Core collectors (local ROS)** – Implement helpers for:
   - Pose/TF/odom (map frame) with freshness checks.
   - Battery/velocity/system status (optional).
   - Camera capture via `image_transport` + `cv_bridge` (single frame, timeout).
   - Pointcloud/scan summary (downsampled stats; full cloud stored as file).
   - Recent BT failure note passthrough from goal (if provided in geo_hint or later goal field).
4) **External data (MCP/OSM)** – Add ROS-MCP client to request recent messages when available; add OpenStreetMapMCP tile fetch by GPS (from current pose or geo_hint). Cache tiles under `/tmp/context_gatherer/<session>/`.
5) **Snapshot assembly** – Normalize into JSON schema (timestamp, frame_id, pose, nav_status, sensors[], map, attachments[]). Keep blobs out of JSON; reference URIs pointing to saved files (png/jpg/pcd). Include encoding/shape metadata.
6) **Limits + error handling** – Timeouts per requirement, max bytes per attachment, partial-success result with clear message. Throttle captures and reuse cached data within a short window to avoid sensor spam.
7) **Persistence + cleanup** – Write snapshot JSON + attachments to session-scoped directory; return URIs. Optional retention policy param.
8) **Testing & observability** – Add unit tests for registry and JSON builder; small integration test that spins the action server and returns stub data. Add structured logging for each stage.

## Interface touchpoints
- Uses `gen_bt_interfaces/action/GatherContext.action` (goal fields: session_id, subtree_id, context_requirements[], timeout_sec, geo_hint).
- Emits `context_json` (schema above) + `attachment_uris` on success/partial success.

## Changes needed in other packages
### llm_interface
1) Implement `CreatePayload.srv` server (new in `gen_bt_interfaces/srv/CreatePayload.srv`).
2) Validate `subtree_contract_json` and `context_snapshot_json`, ensuring required keys/attachments exist; return `RETRY` on missing context vs `ERROR` on schema issues.
3) Prompt/tooling: include context JSON + attachment URIs; allow MCP tools to load referenced files if needed. Return `payload_json` with blackboard-ready values and `tool_trace_json`.
4) Streaming/logging: emit short `reasoning` string and persist tool traces alongside existing LangChain logs.

### mission_coordinator
1) Add `GatherContext` action client: build goal from selected subtree metadata (context_requirements + optional geo hint). Stream feedback to status topic/UI.
2) After selection and before BT execution: call `GatherContext`; on success/partial-success, pass `context_json` + `attachment_uris` into `CreatePayload` request along with `subtree_contract_json` (from catalog/metadata).
3) Handle failures: if `GatherContext` fails hard, surface to UI and skip BT; if `CreatePayload` returns `RETRY`, decide whether to re-call gatherer or ask operator.
4) Update params/YAML to include context action name, payload service name, timeouts, and retry policy; wire into the mission flow (disable in demo_mode).
5) Ensure `ExecuteTree` goal payload uses `payload_json` returned by `CreatePayload` and logs it for traceability (without leaking large attachments).

## Deliverables checklist
- C++ node source + launch/params in `context_gatherer`.
- Working action server with staged feedback and JSON output.
- Updated `mission_coordinator` flow calling GatherContext -> CreatePayload -> bt_executor.
- `llm_interface` CreatePayload server with validation and prompt/tooling hook.
