# BehaviorTree.ROS2 + LangChain Architecture

## Overview

`generalist_bt_gen` combines ROS 2 Jazzy, BehaviorTree.CPP, BehaviorTree.ROS2, and LangChain to let a field robot execute natural-language tasks and expand its own behavior tree when it detects capability gaps. The executor now subclasses `behaviortree_ros2::TreeExecutionServer`, so BT runs are exposed as ROS 2 Actions with built-in logging, preemption, and plugin management. When the tree cannot complete a command, LangChain orchestrates LLM reasoning, tool calls, and MCP servers to synthesize new BT subtrees that are merged into the running executor without restarting the rest of the stack.

> **Note:** “Expanding the tree” does **not** mean mutating a running `BT::Tree` instance. Instead, the `TreeExecutionServer` node stays alive, but the internal `BT::Tree` is stopped, updated XML is computed, and a **new** tree is created from that XML inside the same executor process. From the ROS 2 Action perspective, this may mean that the **current goal is finished** (e.g., with a special result code) and a **follow-up goal** is started once regeneration completes.

## Orchestration Layer (LLM Pre‑Hook)

To keep concerns clearly separated, the system is split into three layers:

- **UI layer (`chat_interface`)** – Presents information to the user and collects commands; no heavy planning logic.
- **Orchestration layer (`orchestrator`)** – Core logic:
  - receives commands from the UI (e.g., via topics/services or a thin RPC API),
  - calls `llm_interface` for “Thinking” / planning,
  - decides which behavior trees (or subtree IDs) to execute, and in what order,
  - sends goals to `bt_executor` and reacts to feedback/results,
  - may update trees via `bt_updater` when capabilities are missing,
  - determines termination conditions for open‑ended tasks like exploration.
- **Execution layer (`bt_executor` + BT plugins)** – Runs behavior trees against the robot, exposing them as ROS 2 actions.

The orchestration layer sits between UI and execution:

- It can display intermediate status and reasoning in the UI.
- It receives feedback/results from the BT executor and either:
  - forwards them directly to the UI, or
  - uses them as signals to halt / modify / switch behavior trees.

## Technology Stack

| Layer | Tooling | Notes |
| --- | --- | --- |
| Behavior tree runtime | BehaviorTree.CPP 4.x | Core BT engine and plugin API. |
| ROS integration | BehaviorTree.ROS2 (`TreeExecutionServer`) | Wraps BT execution inside an `rclcpp_action::Server`, handles XML/plugin discovery, Groot2 logging, and action feedback. |
| Robotics middleware | ROS 2 Jazzy | Provides rclcpp/rclpy nodes, services, actions, Nav2 + sensor interfaces. |
| LLM orchestration | LangChain (Python) | Supplies prompt templates, multi-provider LLM clients (OpenAI/Gemini/Claude), tool-calling, and MCP server integration for structured plans and BT specs. |
| Cloud LLMs | OpenAI, Google Gemini, Anthropic Claude | Configurable via LangChain; selected per prompt config. |

## Package Responsibilities

| Package | Purpose | Key Dependencies |
| --- | --- | --- |
| `bt_executor` | Extends `TreeExecutionServer` with custom hooks: registers BT plugins once, seeds the global blackboard, listens for failure states, and drives the LLM regeneration loop. On regeneration, it stops the current BT run and calls `createTreeFromText` with updated XML, without restarting the ROS 2 Action server or node. The current action goal is completed with an explicit result (e.g., `NEEDS_EXTENSION` or `TREE_UPDATED`), and the orchestrator is expected to re-issue or continue the mission via a follow-up goal. Publishes action feedback/metrics. | `behaviortree_cpp`, `behaviortree_ros2`, `rclcpp`, `std_srvs`, `yaml-cpp`, `nlohmann_json`. |
| `robot_actions` | ROS-native BT plugins for locomotion, manipulation, and sensing (Nav2 goals, camera triggers, etc.). | `rclcpp`, `behaviortree_cpp`, `nav2_msgs`, `sensor_msgs`, `geometry_msgs`, `action_msgs`. |
| `llm_actions` | BT nodes that gate behavior based on LLM output, update the blackboard, and bridge to the LangChain service (e.g., Thinking node, plan validator). These are optional if the project decides to keep all “Thinking” in the external orchestration layer and use the BT purely for execution. | `rclcpp`, `behaviortree_cpp`, `nlohmann_json`, `llm_interface`. |
| `context_gatherer` | Aggregates camera frames, GPS, point clouds, and semantic context into the format expected by LangChain tools/MPL servers. | `rclcpp`, `sensor_msgs`, `nav_msgs`, `geographic_msgs`, `image_transport`, `cv_bridge`, `nlohmann_json`. |
| `llm_interface` | Python ROS node exposing `LLMQuery.srv`. Uses LangChain PromptTemplates, tool chains, and MCP-backed retrievers to generate BT specs + artifacts. It is called by the orchestrator both as a **pre‑hook** before BT execution (to choose/churn subtrees) and during regeneration when new capabilities are needed. | `rclpy`, `langchain`, `openai`, `google-generativeai`, `anthropic`, `requests`, `python3-yaml`. |
| `bt_updater` | Applies LangChain-generated XML patches to the authoritative BT tree, validates them with TinyXML2, and returns merged XML to the executor, which then recreates the internal `BT::Tree`. | `rclcpp`, `behaviortree_cpp`, `tinyxml2`, `yaml-cpp`. |
| `orchestrator` | Logic layer between UI and executor. Receives high-level commands from `chat_interface`, queries `llm_interface` for plans, selects/creates subtrees via `bt_updater`, sends `ExecuteTree` goals to `bt_executor`, monitors feedback/results, and implements termination logic for open-ended missions (e.g., exploration “enough coverage” or user satisfaction). Forwards relevant status and reasoning back to the UI. | `rclpy`/`rclcpp`, `action_msgs`, `llm_interface`, `bt_updater`, `std_msgs`. |
| `chat_interface` | Pure UI layer for human commands; presents status, histories, and controls. It delegates all planning/decision-making to the `orchestrator` and does **not** talk directly to `bt_executor`. | `rclpy`, `FastAPI`/`Flask`, `websockets`, `std_msgs`. |

Optional packages such as `bt_visualizer` can subscribe to the executor action feedback or Groot2 stream for live dashboards.

## Main Tree Structure

```markdown
// filepath: /home/luke/generalist_bt_gen/architecture.md
ReactiveSequence:
    - ChatInterfaceAbortSignalConditionCheck
    - Thinking Action Node (outputs decision enum)
    - Fallback:
        - Subtrees to execute (preconditioned with skipIf: decision != subtree_enum)
        - FailureFallbackAction (triggers tree-expansion as no subtree was executed or they failed while running)
```

> **Variant:** In the “Thinking outside BT” design, the above root may be simplified to a **pure execution tree** (e.g., a `Sequence`/`Fallback` over robot skills and guards), while the LLM “Thinking Action Node” and `FailureFallbackAction` logic move into the orchestration layer. The BT then executes a specific subtree ID or entry point chosen by the LLM pre‑hook.

## Runtime Flow

```mermaid
sequenceDiagram
    actor User
    participant UI as chat_interface (UI)
    participant Orc as orchestrator (Logic)
    participant Exec as bt_executor (TreeExecutionServer)
    participant Skills as robot_actions / llm_actions
    participant Ctx as context_gatherer
    participant LLM as llm_interface (LangChain)
    participant Upd as bt_updater

    User->>UI: "Explore the field and photograph lonely trees"
    UI-->>Orc: Command (structured request)

    Note over Orc,LLM: Pre-hook "Thinking" in orchestration layer
    Orc->>LLM: LLMQuery.srv(command, available_subtrees, recent_results)
    LLM-->>Orc: plan = { subtree_ids[], maybe_new_subtree_spec, stopping_criteria }

    alt new subtree needed
      Orc->>Upd: UpdateTree.srv(plan.new_subtree_xml)
      Upd-->>Orc: OK + merged XML path
      Orc->>Exec: Notify/trigger reload (if needed)
    end

    loop for subtree in plan.subtree_ids
      Orc->>Exec: Send `ExecuteTree` goal (subtree_id, command context)
      Exec->>Skills: Tick BT via TreeExecutionServer loop
      Skills-->>Exec: RUNNING / SUCCESS / FAILURE
      Exec-->>Orc: Action feedback (node status, telemetry)
      Orc-->>UI: UI-friendly progress / status updates

      alt subtree SUCCESS and stopping_criteria not met
        Orc-->>Orc: Decide whether to call next subtree in plan or replan
      else subtree FAILURE or NEEDS_EXTENSION
        Exec-->>Orc: Action result (status + failure_reason)
        Orc->>LLM: Optional LLMQuery.srv(command, failure_reason, context)
        LLM-->>Orc: updated plan / new subtree request
        Orc-->>Upd: Optional UpdateTree.srv(...)
      end

      alt stopping_criteria met (e.g., enough exploration, user satisfied)
        Orc-->>UI: "Exploration complete" (with reasoning/metrics)
        break
      end
    end
```

## Failure, Regeneration & Open-Ended Goals

1. **Detection (inside BT)** – `bt_executor` detects `FAILURE` or `NEEDS_EXTENSION` and finishes the current action goal with a structured result. The executor and Action server remain active.
2. **Decision (in orchestrator)** – The orchestrator receives this result and decides whether to:
   - Report a terminal failure to the UI, or
   - Request new capabilities or a revised plan from `llm_interface`.
3. **Context capture** – The orchestrator can request snapshots from `context_gatherer` and pass this context into `llm_interface`.
4. **LangChain workflow** – As
