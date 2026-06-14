# llm_interface

LangChain-powered ROS 2 node that exposes requirement extraction, planning, tree-selection, and payload services for the mission pipeline. The node will:

- Listen on `/llm_interface/plan_subtree` to synthesize new BT XML (see `service_definition_plan.md`).
- Listen on `/llm_interface/extract_mission_requirements` to classify free-form mission commands into structured capability requirements for `mission_reasoner`.
- Listen on `/llm_interface/select_behavior_tree` to pick the most relevant existing subtree before regeneration using the configured LangChain provider.
- Call the configured LLM provider + MCP tools to translate mission text + context snapshots into BT XML.
- Log requests/responses for auditability and provide summaries back to the UI layer.

## Provider configuration

The active provider is selected via ROS parameters in `config/llm_interface_params.yaml`:

- `provider`: global default provider for all llm_interface calls.
- `mission_requirements_provider`: provider for mission requirement extraction.
- `selection_provider`: provider for behavior-tree selection.
- `payload_provider`: provider for payload generation.
- `payload_normalizer_provider`: provider for payload schema correction.
- `selection_reasoning_effort`, `mission_requirements_reasoning_effort`,
  `payload_reasoning_effort`, `payload_normalizer_reasoning_effort`: optional
  reasoning effort strings passed to OpenAI/OpenRouter LangChain clients. Empty
  values disable reasoning configuration for that purpose.
- Matching `*_reasoning_summary` parameters can request provider-supported
  reasoning summaries, for example `auto` or `detailed`.

Supported providers:

- `gemini` using `GEMINI_API_KEY`
- `openai` using `OPENAI_API_KEY`
- `openrouter` using `OPENROUTER_API_KEY`

To run through OpenRouter, set the provider fields to `openrouter` and use an OpenRouter model id such as `openai/gpt-4o-mini` or `anthropic/claude-sonnet-4.5`. Optional app attribution headers can be set with:

- `openrouter_app_url`
- `openrouter_app_title`

See `implementation_plan.md` for the delivery roadmap.
