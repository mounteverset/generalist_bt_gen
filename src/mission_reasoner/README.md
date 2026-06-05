# `mission_reasoner` package

`mission_reasoner` validates an operator mission before behavior-tree
selection. It compares the natural-language command, optional goal context, BT
catalogue metadata, and `config/system_description.yaml`.

When `enable_llm_extraction` is true, the node first calls
`/llm_interface/extract_mission_requirements` to parse the command into
structured requirements. The LLM output is treated as semantic extraction only;
the accept/clarify/refuse decision is still made by deterministic validation
against the system description.

## Interface

Service: `/mission_reasoner/validate_mission` (`ValidateMission.srv`)

Responses:

| Status | Meaning |
| --- | --- |
| `ACCEPT` | At least one BT candidate is compatible with the declared platform capabilities. |
| `CLARIFY` | The command is potentially supported but lacks a target or parameter needed before selection. |
| `REFUSE` | The command requests a capability absent from the platform description. |
| `ERROR` | The reasoner cannot validate the mission due to invalid configuration or input. |

If the LLM service is unavailable, times out, or returns invalid JSON, the
reasoner logs a warning and falls back to deterministic command rules.

Relevant parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enable_llm_extraction` | `true` | Enables the semantic extraction call before deterministic validation. |
| `mission_requirements_service` | `/llm_interface/extract_mission_requirements` | LLM extraction service name. |
| `llm_service_wait_timeout_sec` | `2.0` | Startup/service-discovery wait before falling back. |
| `llm_extraction_timeout_sec` | `45.0` | Maximum time to wait for the extraction response. Large remote models can exceed 10 seconds on cold calls. |
