# llm_interface Implementation Plan

_Reference_: `architecture.md` ("LLM orchestration" + `llm_interface` responsibilities) and `project_roadmap.md` (v0.5 requirements).

## Goals
- Provide a ROS 2 service (`/llm_interface/plan_subtree`) that the mission coordinator can call to translate NL mission requests and context into BT XML (or subtree IDs) plus metadata.
- Host LangChain pipelines for multiple providers (OpenAI, Gemini, Claude) with configurable API keys and toolchains.
- Capture context from `context_gatherer` / mission coordinator payloads and pass them to the LLM prompt template.

## Work Breakdown

1. **Service API wiring (ROS 2)**
   - Define `PlanSubtree.srv` (regeneration) and `SelectBehaviorTree.srv` (existing tree selection) in `gen_bt_interfaces`.
   - Implement two service servers inside `llm_interface`: a fast selection path that returns an existing BT ID, and a generation path that emits BT XML when no match exists. Both servers should share authentication/prompt infrastructure and expose latency/status metrics.

2. **LangChain pipeline**
   - Create prompt templates tuned for BT subtree synthesis (LLM instructions, output schema).
   - Configure provider adapters (OpenAI, Gemini, Claude) with retry + rate-limit handling.
   - Integrate MCP tools (filesystem, ROS context) per `architecture.md` guidance to allow richer planning.

3. **Mission coordinator integration**
   - Update `mission_coordinator` to call the new service (replace demo placeholder) and react to `status_code` variants (`SUCCESS`, `RETRY`, `ESCALATE`).
   - Surface response summaries into UI feedback topics.

4. **Telemetry + caching**
   - Store request/response pairs in `~/.generalist_bt/llm_cache` for audits.
   - Emit structured logs / metrics (latency, provider) for monitoring.

5. **Testing + docs**
   - Add unit tests with mocked LLM providers.
   - Document secrets management, provider selection, and service contract.

## Dependencies / Open Questions
- Need API keys + endpoints (see `scripts/setup_cloud_env.sh`).
- Confirm context payload schema with upcoming `context_gatherer` implementation.
- Determine how tool invocations (e.g., plan validators) are surfaced back to mission coordinator.
