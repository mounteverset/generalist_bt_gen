# llm_interface

LangChain-powered ROS 2 node that exposes planning + tree-selection services for the mission coordinator. The node will:

- Listen on `/llm_interface/plan_subtree` to synthesize new BT XML (see `service_definition_plan.md`).
- Listen on `/llm_interface/select_behavior_tree` to pick the most relevant existing subtree before regeneration (using Gemini 2.5 via LangChain).
- Call the configured LLM provider + MCP tools to translate mission text + context snapshots into BT XML.
- Log requests/responses for auditability and provide summaries back to the UI layer.

See `implementation_plan.md` for the delivery roadmap.
