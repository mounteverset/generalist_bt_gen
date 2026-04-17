# Ongoing Research Presentation Outline

## Motivation

### Behavior Trees suit robotic mission
- Behavior Trees are attractive for robot control because they are modular, interpretable, and robust in safety-critical settings.
- Traditional mission planning slow and technical

### Multi-Modal Context is available:

- Multimodal context matters for outdoor missions: maps, GPS, images, robot pose, sensor readings, and mission history can change the correct interpretation.

### Validation Problem
- Recent literature is dominated by BT generation from scratch, but that shifts validation and safety burden onto every new LLM output.
- In robotics, validation is the bottleneck: generated trees may be syntactically valid but still unsafe, brittle, or poorly aligned with available robot skills or desired user mission
- Existing work also tends to use weak context, often only the text instruction, while real outdoor missions depend on richer environment and robot-state information
- Outdoor and field scenarios raise the cost (expensive equipment, high risk) of wrong decisions, making fully generative control approaches harder to justify
- This thesis investigates how LLMs support interpretation and adaptation while keeping execution grounded in validated BT assets

## BT Terminology

- BT Signals, Actions, Control Nodes, Input/Output Ports
- Blackboard, Blackboard contract, Payload


## Problem Formulation

- Goal: transform a natural-language mission into reliable Behavior Tree selection and parametrization for an outdoor robot.
- using existing BT templates and robot skills, rather than generating unrestricted end-to-end robot missions
- Research gap from the literature:
  - most systems generate new BTs instead of selecting from a trusted library
  - adaptation is often reactive, after failure has already happened
  - multi-source context for outdoor deployment is rarely modeled explicitly
- Input sources include:
  - user mission text
  - subtree metadata (tree description, context requirements and blackboard contracts)
  - robot capabilities
  - multimodal runtime context from sensors and environment representations
- Core research question:
  - "How can LLMs convert natural-language mission descriptions and multimodal context into dependable BT selection and payload generation?"
- Main technical challenges:
  - structuring heterogeneous context in a platform-independent format
  - matching mission intent to available subtree templates
  - extracting valid parameters for BT blackboards
  - preserving reliability, traceability, and operator control
- Why existing BTs are the right abstraction:
  - validated trees preserve prior engineering effort and known safety behavior
  - selection from a library is easier to audit than free-form tree generation
  - constrained parametrization reduces the search space for the LLM
  - reusable BT templates support cross-platform deployment better than one-off generated trees

## Approach

- Use a modular ROS 2 architecture with clear separation between planning, context gathering, and execution.
- Keep "thinking" outside the Behavior Tree:
  - `mission_coordinator` handles orchestration (mission command -> llm calls -> user feedback -> bt execution)
  - `llm_interface` handles interpretation, tree selection, and payload generation
  - `bt_executor` remains focused on BT execution, has a library of pre-built BT's available, converts payload into blackboard 
- Represent subtree capabilities explicitly through metadata:
  - tree description
  - required context
  - blackboard contract
- Treat the LLM primarily as a situation-aware selector and parameter tuner, not as the final authority for robot control structure.
- Use `context_gatherer` to assemble mission-relevant context into structured JSON plus attachment references.
- Use LLM calls to:
  - interpret the command
  - select an existing subtree
  - generate and normalize blackboard payload values
- improve reliability over generation-first methods:
  - reuse pre-validated execution structures
  - localize LLM responsibility to higher-level reasoning and parameter extraction
  - make operator review and fallback handling easier
- Support user interaction through CLI/web UI and optional operator approval before execution.

- Current progress:
  - natural-language command flow from UI to mission coordinator to bt execution is in place
  - subtree selection and payload-related schema is working
  - context gathering infrastructure is defined and partially implemented, additional context can modularly be added depending on the mission requirements
  - full self-updating / repairing trees are explicitly out of scope for the current thesis
  - pending plan review before execution
  - operator feedback (w/ full session history of payloads and feedbacks)

## Testing strategy
 - Missions:
    - 
  - Prompts:
    - 


## Potential Future Directions

- Improve robustness with confidence scoring, fallback policies, and operator-in-the-loop confirmations.
- Expand context gathering with richer GPS, map, point cloud, and semantic scene understanding.
- Natural language mission boundaries, constraints ("only drive on field you are currently on", "only use trails, no offroading")
- Compare multiple LLM providers and prompting strategies for reliability and latency.
- Generalize the framework across platforms, including both ground robots and surface vehicles.
- Investigate controlled subtree extension or repair after execution failure.
- Build a benchmark suite of outdoor mission scenarios for reproducible evaluation.

## Open Questions
- Focus on non-technical operators? Need user studies
- 