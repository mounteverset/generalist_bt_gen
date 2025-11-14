# Chat Interface Package Plan

## Mission
Design `user_interface` as the presentation layer described in `architecture.md`: a chat-style UI that forwards user intents to the orchestrator, shows system status, and collects clarifications. The package stays thin (no planning/execution logic) while offering ergonomic human-in-the-loop controls for the Jazzy + BehaviorTree.ROS2 stack.

## User-Facing Features
1. **Chat Console (MVP)** – Text-only TUI/CLI that streams:
   - User prompts with local timestamp + session metadata.
   - Orchestrator responses (status, BT feedback, LLM reasoning summaries).
   - Inline notifications for regeneration requests, confirmations, or blocking alerts.
2. **Command Shortcuts** – Slash-style helpers (`/pause`, `/resume`, `/status`) mapped to orchestrator services/actions.
3. **Context Panels (stretch)** – Optional panes that tail key ROS topics or LangChain trace snippets for situational awareness.
4. **Conversation Logging** – Persist chat transcript (JSONL + Markdown) for later auditing, referencing timestamped ROS action goals.

## Tech Stack & Integrations
- **ROS 2 Jazzy / rclpy** for the UI node + service/action clients (consistent with other Python orchestration nodes).
- **LangChain REST/Websocket hooks** consumed via `aiohttp` if/when direct chat completions are streamed.
- **Rich / Textual (Python TUI)** for responsive terminal widgets (panels, input, status badges). GTK/Qt left as future work if GUI required.
- **Standard ROS Interfaces**
  - Action client to the orchestrator’s `ExecuteTree` proxy action.
  - Service clients for `/regenerate_tree`, `/list_subtrees`, `/confirm_stop`, etc. (exact names TBD with orchestrator team).
  - Subscriptions to diagnostics topics for status banners.

## Implementation Steps
1. **Skeleton Node** ✅
   - `chat_node.py` now exposes an `AsyncIOExecutor`-backed CLI that reads ROS params (action/service/topic names), writes transcripts, and exposes slash commands for orchestrator interactions without invoking LangChain directly.
2. **Transport Layer**
   - Wrap ROS clients/subscribers + optional WebSocket client for LangChain streams.
   - Provide async queues that the UI thread/widget layer can poll without blocking ROS callbacks.
3. **UI Layer**
   - Implement Rich/Textual layouts (input box, message feed, status panel) **and** FastAPI web surface (✅ basic HTML + polling `/state` now available at `web_ui_node`).
   - Map keyboard shortcuts + slash commands to ROS/LangChain calls.
4. **Persistence** ✅
   - Transcripts stream to `~/.generalist_bt/chat_logs/<timestamp>.jsonl` with timestamps + metadata tags (topic/service touched).
5. **Testing & Tooling**
   - Unit tests for transport helpers (mock ROS clients).
   - Snapshot/Golden tests for transcript formatting.
   - CLI smoke test (`ros2 run user_interface chat_node -- --demo`) with mock orchestrator endpoints.

## Open Questions
- Confirm orchestrator API (topic/action/service names + schemas) to avoid tight coupling.
- Decide whether LangChain streaming comes via orchestrator relays or direct WebSocket tokens from the UI.
- Clarify authentication strategy for cloud LLM traces (env vars vs. keyring).
- Extend `chat_node.py` beyond demo stubs: wire `/status`, `/regen`, `/llm` commands to real ROS services (names still configured in `config/chat_params.yaml`).
- Rich/Textual panes are still TBD; current MVP uses Rich console output.
- Web UI needs websocket/server-sent updates & orchestrator wiring—currently polling JSON + demo responses only.

> Next step: formalize the orchestrator<->UI contract and scaffold the `chat_node.py` entry point inside this package.
