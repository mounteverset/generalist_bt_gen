# `user_interface` Package

Chat-first interfaces for the Field BT mission coordinator. This package currently ships two entry points—`chat_node` (terminal) and `web_ui_node` (browser)—that proxy operator intents to the mission coordinator over ROS 2 services/actions and stream ROS status back to the user. Both UIs are intentionally thin: they format data, persist transcripts, and surface controls, while the coordination node remains the sole component that drives `bt_executor`, `context_gatherer`, `llm_interface`, and related packages.

> **TL;DR**: `user_interface` provides configurable UIs; all integrations happen through parameterized ROS APIs so we can retarget the mission coordinator contract without rewriting the view layer.

## Capabilities
- **Configurable ROS contract** – Every action/service/topic name is read from ROS parameters/YAML so the UI can follow mission coordinator rename changes without recompiling.
- **Terminal chat console (`chat_node`)** – Minimal Rich-based CLI with slash commands (`/status`, `/regen`, `/llm`, `/quit`), per-session JSONL transcript logging, diagnostics subscriptions, and demo-mode stubs for offline testing.
- **Browser control surface (`web_ui_node`)** – FastAPI + Jinja2 backend serving `/` (dashboard), `/state` (JSON snapshots), and `/command` (POST chat). Includes a responsive layout for active subtree, status, diagnostics, and chat history; front-end polls ROS state regularly.
- **Transcript logging** – Both frontends write JSONL logs to `~/.generalist_bt/chat_logs` (path configurable) for auditing / LLM fine-tuning.
- **Demo mode** – When `demo_mode` is `true` (default), no ROS calls are issued; the UI echoes what would happen. Turn it off once mission coordinator services are available.

## Installation / Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select user_interface
source install/setup.bash
```

## Usage
### Terminal Chat
```bash
ros2 run user_interface chat_node \
  --ros-args --params-file $(ros2 pkg prefix user_interface)/share/user_interface/config/chat_params.yaml
```
Type natural-language instructions or slash commands. Example session:
```
/status        # prints “Would call /mission_coordinator/status” (demo mode)
/regen         # prints mission coordinator regen service name
/llm Please summarize logs
Hello robot
/quit
```
When `demo_mode:=false`, these commands should be wired to actual mission coordinator clients.

### Web UI
```bash
ros2 run user_interface web_ui_node \
  --ros-args --params-file $(ros2 pkg prefix user_interface)/share/user_interface/config/chat_params.yaml
```
Then open `http://<host>:8080` (port/host configurable). The page auto-refreshes status/diagnostics/active subtree every 1.5 s. Sending a chat command issues a POST to `/command`; in demo mode it echoes back, otherwise it should forward to the mission coordinator.

## Parameters (shared defaults in `config/chat_params.yaml`)
| Parameter | Description | Default |
| --- | --- | --- |
| `ui_title` | Display name for CLI banner + web title. | `Generalist BT Chat` |
| `mission_coordinator_action` | Action name the UI forwards user intents to. | `/mission_coordinator/execute_tree` |
| `status_service` | Service used by `/status`. | `/mission_coordinator/status` |
| `regen_service` | Service used by `/regen`. | `/mission_coordinator/request_regen` |
| `llm_prompt_service` | Service used by `/llm`. | `/llm_interface/chat_complete` |
| `status_topic` | Topic for textual status updates. | `/mission_coordinator/status_text` |
| `subtree_topic` | Topic that publishes active subtree label (web UI). | `/mission_coordinator/active_subtree` |
| `diagnostics_topics` | List of extra `std_msgs/String` topics to display. | `[ "/diagnostics" ]` |
| `transcript_directory` | Directory for JSONL logs (expanded). | `~/.generalist_bt/chat_logs` |
| `refresh_period` | ROS spin + UI poll period (seconds). | `0.1` |
| `timestamp_timezone` | Zoneinfo name for logging. | `UTC` |
| `demo_mode` | When `true`, ROS clients are stubbed with log messages. | `true` |
| `web_host` | (Web UI) host binding for FastAPI/Uvicorn. | `0.0.0.0` |
| `web_port` | (Web UI) port binding. | `8080` |

Override any parameter through `--ros-args -p param_name:=value` or custom YAML.

## Open TODOs / Next Steps
1. **Mission coordinator gateway** – Implement actual ROS action/service clients that the CLI/Web call, encapsulated in a dedicated class (so UI stays ROS-free).
2. **Live updates for Web UI** – Replace polling with WebSockets/SSE for instant BT state updates, include feedback stream from the mission coordinator.
3. **Authentication / multi-user** – Add optional auth (token or OAuth) for web deployment beyond localhost; handle per-user transcript directories.
4. **Richer layout** – Integrate additional panes (e.g., context_gatherer snapshots, map view) per `architecture.md`.
5. **Testing** – Add pytest coverage for transcript writer, command parsing, and FastAPI handlers (with `TestClient`).
6. **Config packages** – Move default YAML into launch files and add sample `ros2 launch` descriptions once mission coordinator API is stable.
7. **LangChain streaming** – Decide whether `/llm` call goes through the mission coordinator or direct websockets to `llm_interface`; update UI accordingly.

Contributions that tackle any of the above should update `plan_chat_interface.md` so the roadmap stays synchronized.
