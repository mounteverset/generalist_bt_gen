# Thesis evaluation

This folder is the machine-readable evaluation source of truth. It separates
the direct-generation baselines from the thesis pipeline and preserves every
raw first model output.

## Scope and current status

- The E1 core dataset and context fixtures are frozen; later-experiment protocol files remain separate.
- It contains nine missions with three paraphrases each.
- Seven Husky missions are implemented in the current catalogue.
- Two BlueBoat missions specify the multi-platform evaluation. The capability
  profile, six BlueBoat node interfaces, and tree are in the runtime snapshot;
  both cases are available for offline scored runs. E5 uses physical execution
  evidence from both platforms.
- The designed E1 matrix contains 189 outputs: 27 prompts times six
  general-model method conditions plus BTGenBot-2.
- All 189 planned E1 outputs are supported offline: 27 prompts times seven
  conditions. Physical execution evidence is reported separately.
- No user study is planned.

The runner refuses a scored run while the dataset, providers, or BTGenBot-2
revisions are not frozen. Preliminary and dry runs remain available.

## Mission instruction specificity

Dataset version `0.4.1` assigns P1, P2, and P3 to low, medium, and high
instruction specificity, recorded in each paraphrase's `specificity` field.
P1 states the goal and refers to supplied settings; P2 names the main steps
and settings; P3 expands the steps and restates relevant context facts.
All three share the same context, mission constraints, reference behavior,
acceptable outcomes, and complexity score. Meaning preservation applies to
the instruction together with that context, not to the sentence alone.
Details absent from the shared context stay explicit at every level.
Longer instructions must not introduce extra goals, constraints, coordinates,
or preferred solutions. Equivalence requires semantic review; the validator
checks the three level labels and their order, not natural-language meaning.

This design tests sensitivity to instruction detail and explicit restatement
of available context. Compare levels within each mission; the 27 instructions
are still nine tasks. Wording and explicitness vary together, so differences
cannot be attributed to instruction length alone.

Earlier draft outputs retain their original wording. Condition IDs include the
mission, context, safety, model, runtime, and scoring protocol hashes, so an
input change creates a new ID. Do not pool older outputs with revised inputs.

## Methods

- `M1`: a general-purpose model directly generates complete BT.CPP XML.
- `M2`: the fine-tuned BTGenBot-2 model directly generates complete BT.CPP XML.
- `M3`: the thesis pipeline extracts requirements, applies the real
  `MissionReasoner`, checks context admission, constrains tree selection,
  generates a payload, validates the payload contract, reviews spatial and
  safety properties, and records any refinement separately.

M1 and M2 receive the same fixed facts as M3. Their prompts require external
mission inputs such as waypoint lists and output paths to be embedded directly
in XML. Blackboard references are valid only for values produced by an earlier
node. This avoids giving the direct baselines an unresolved payload that they
have no mechanism to return.

The action and tree catalogues are generated from
`protocol/runtime_contract.json`. Static copied catalogues were removed because
they had drifted from the implementation.

## Experiments

- `E1`: supported mission and model/method comparison.
- `E2`: M3 context ablation with complete, text-only, missing-source, stale,
  contradictory, and irrelevant-distractor variants. GPT-5.6-Sol is the fixed
  reference model. The two missions and six conditions produce 12 outputs;
  ten are new calls and two reuse identical E1 conditions.
- `E3`: method-specific choice-space scaling on S1, M3, and C2. M1 defines
  libraries of 11, 50, and 100 custom-node identifiers. M3 separately varies
  the tree catalogue between the expected tree plus one incompatible tree and
  all seven current trees. M1 produces 27 outputs. The post-hoc M3 model
  extension produces 18 outputs, including nine reused E1 conditions.
- `E4`: adverse requests and safety/failure quality. Clarification and refusal
  are correct outcomes for predefined cases.
- `E5`: one physical execution per mission and general-purpose model, for at
  most 27 trials. This is not produced by the offline runner and must be
  reported separately.

## Important files

- `protocol/core_missions.json`: missions, paraphrases, expected trees,
  canonical payloads, complexity, and support status.
- `fixtures/context/core_contexts.json`: fixed facts supplied to all methods.
- `protocol/runtime_contract.json`: frozen JSON snapshot of tree metadata,
  platform capabilities, and registered node interfaces.
- `protocol/bt_node_manifest.json`: current custom nodes, aliases, controls, and
  ports.
- `protocol/model_conditions.json`: model, decoding, provider, and revision
  freeze state.
- `protocol/context_variants.json`: machine-readable E2 context operations.
- `protocol/choice_space_variants.json`: E3 conditions and confound controls.
- `protocol/m1_action_distractors.json`: 90 evaluation-only node signatures,
  paired between semantically similar and adjacent robot-mission capabilities.
- `protocol/safety_cases.json`: E4 expected plans, clarifications, and refusals.
- `protocol/scoring_rubric.json`: pass rules, denominators, review coverage,
  correction effort, and statistical reporting.
- `protocol/execution_scoring.json`: E5 trial and portability record rules.
- `protocol/e5_physical_runbook.md`: physical preflight, trial order, abort rules,
  and evidence layout.
- `scripts/run_evaluation.py`: E1–E4 runner.
- `scripts/import_m3_cli_results.py`: validates immutable Method 3 CLI evidence
  and writes scorer-compatible copies.
- `scripts/score_results.py`: per-run tables, mission-cluster intervals, paired
  effects, human ratings, correction effort, and optional E5 summaries.
- `scripts/export_blind_review.py`: removes model and method labels for manual
  semantic review and creates the response template.
- `scripts/render_route_review.py`: renders each artifact's route, as the robot's
  parsers read it, with the reference route and context geometry on an interactive
  OpenStreetMap page. `--review-key` hides method, model, and scores for blind review.
- `protocol/review_single_reviewer_amendment.json`: replaces the rubric's second
  reviewer, adjudication, and agreement rules with a single blinded reviewer.
- `colab/btgenbot2_server.ipynb`: authenticated BTGenBot-2 endpoint and batch
  workflow.

## E2-E4 under the reduced scope

`protocol/e2_e4_reduced_scope_amendment.json` (frozen 2026-09-28) fixes the
remaining experiments before any E2-E4 output exists:

- M3 runs through the ROS pipeline and the evaluation CLI in E2-E4, as in E1.
  `scripts/build_cli_trials.py` writes the E2 and E4 context fixtures, the E3 CS1
  coordinator parameter files, and `protocol/e{2,3,4}_m3_cli_trials.json`
  (10, 9, and 30 new-call trials). Run them with
  `scripts/run_e1_m3_cli.py --manifest protocol/e2_m3_cli_trials.json` and import
  them with `scripts/import_m3_cli_results.py --manifest ...`.
- E2 and E3 use only P2. The E1 M3 GPT-5.6-Sol P2 outputs supply the E2 complete
  condition. Matching E1 M3 P2 outputs supply E3 CS2 for all three models;
  `score_results.py` clones them automatically.
- E2 sends raw evidence without condition labels. Its changed conditions retain
  OSM water, paths, barriers, and stairs; M3 uses the E2-only northern-zone
  satellite image, and C2-CV2 retains GPS, OSM, and satellite evidence.
- E3 M1 runs through `run_evaluation.py` (27 outputs); E4 M1 likewise (30 outputs).
  The runner rejects M3 for E2-E4 and M2 for E4.
- E3 GPT-5.6-Sol prefers OpenAI Flex through OpenRouter's `:floor` variant for
  M1 and M3. The provider remains pinned to `openai`; regular OpenAI service is
  an accepted fallback and the returned tier is evidence, not a scoring gate.
  Each request may take 900 seconds, with three transport retries. Use a
  5400-second CLI evaluation timeout for each M3 trial.
- E3 outputs receive no human rating; `export_blind_review.py` skips them.
- `protocol/e3_m3_model_extension_amendment_20260929.json` records that adding
  Gemini and Gemma to E3 M3 is a post-hoc descriptive extension.
- The mission coordinator writes `mission_decision.json` into the trial evidence
  folder, so the importer can score clarifications and refusals. Rebuild the
  workspace before running E2-E4.

## Validation and dry runs

Run these commands from the repository root:

```bash
python3 evaluation/scripts/validate_dataset.py

python3 evaluation/scripts/run_evaluation.py \
  --experiment E1 \
  --dry-run \
  --methods M1,M3 \
  --models gpt-5.6-sol \
  --mission S1 \
  --paraphrase S1-P1

python3 evaluation/scripts/run_evaluation.py \
  --experiment E3 \
  --dry-run \
  --methods M1 \
  --models gpt-5.6-sol \
  --mission S1 \
  --paraphrase S1-P2 \
  --variant M1-N100
```

The normal output directory is ignored by Git. After each batch, condition
folders are renamed to `MISSION-PARAPHRASE__artifact-id`, for example
`S1-P1__45268faedca8b4a5`. Each folder contains `request.json` and `result.json`.
A run manifest records dataset
hashes, repository snapshot, seeds, host details, counts, and costs. Existing
condition IDs are skipped unless `--force` is used.

E1 hashes only its frozen common inputs: runtime contract, model conditions,
core missions, core context fixtures, and scoring rubric. Draft E2 context
variants, E3 choice-space files, and E4 safety cases cannot invalidate E1
artifacts.

Use `--repetitions`, `--seed`, `--max-cost-usd`, `--max-trials 10`, and
`--max-transport-retries` for controlled batches. Existing results do not count
toward `--max-trials`. Each result and run manifest records the known cost and
whether provider cost reporting was complete. Transport retries preserve one
model output; M3 validation/refinement attempts are separate stages.
`first_attempt_valid` is always computed from the original raw output.
For direct XML methods, the runner preserves that full output and validates the
first complete `<root>...</root>` document found within it. It removes only
surrounding prose or a Markdown fence and never repairs the extracted XML. Each
M1 and M2 artifact folder also stores that exact block as `response.xml`.

## M3 CLI evaluation mode

Set one permanent evidence root before starting bringup and the trial CLI. Both
processes must inherit the same value.

```bash
export GENERALIST_BT_EVIDENCE_ROOT="$PWD/evaluation/results/m3_cli"
ros2 launch generalist_bringup generalist_bringup.launch.py
```

In another sourced terminal, submit one frozen plan-only trial:

```bash
export GENERALIST_BT_EVIDENCE_ROOT="$PWD/evaluation/results/m3_cli"
ros2 run user_interface chat_node \
  --trial-id E1-M1-P2-method3-gemma-pilot-r1 \
  --context-fixture "$PWD/evaluation/fixtures/context/core_contexts.json#/fixtures/M1" \
  --mission "Drive the Husky on the path to the northern end of the lake. Take a photograph every 10 metres travelled and save the images to the evaluation output directory."
```

Evaluation mode verifies fixture image hashes, replays the frozen context,
captures each LLM prompt and raw response, saves the pending plan, and rejects
execution after capture. It refuses `--auto-execute`. The trial ID does not
select a model; load the intended frozen LLM parameter file before bringup.
Use `method1`, `method2`, and `method3` in trial IDs so method names cannot be
confused with mission IDs such as `M1` and `M3`.

For scored Gemma runs, set `openrouter_provider_only: ['makora']` under both
`llm_interface` and `plan_reviewer` in that parameter file. This disables
provider fallback and records the requested provider, returned provider, and
verification result in each LLM-call audit record.

The frozen E1 Method 3 matrix lives in
`protocol/e1_m3_cli_trials.json`. It contains all 81 trials and joins each
runtime folder to the scoring workbook through `experiment_number` and
`scoring_condition_id`. Runtime IDs spell out `method3`, for example
`E1-C2-P3-method3-gemma-r1`.

Print the six launch groups and all 81 trial commands without calling a model:

```bash
python3 evaluation/scripts/run_e1_m3_cli.py
```

Build and source the three changed ROS packages once before an actual run:

```bash
colcon build --symlink-install \
  --packages-select llm_interface plan_reviewer user_interface
source install/setup.bash
```

Run one approved group at a time. The runner launches the matching frozen
model parameters and the Husky or BlueBoat system description, waits for the
mission coordinator, then runs 21 Husky or six BlueBoat trials. Completed
trial folders are skipped.

```bash
python3 evaluation/scripts/run_e1_m3_cli.py --execute \
  --model gpt-5.6-sol --platform husky --max-trials 10
```

Each trial folder contains `trial_protocol.json` with the spreadsheet keys,
exact commands, input paths, and SHA-256 hashes. The CLI adds the request,
context, LLM-call, plan, transcript, result evidence, and `cost_summary.json`.
Each invocation also writes a batch cost summary under `_batch_manifests`.

After the trials, validate and normalize the CLI evidence without changing it:

```bash
python3 evaluation/scripts/import_m3_cli_results.py --require-all
```

The normalized artifacts are written to
`evaluation/results/e1_m3_scoring_ready`.

Multimodal mode is opt-in:

```bash
python3 evaluation/scripts/run_evaluation.py --multimodal ...
```

It blocks when an image file or hash is missing. The repository now includes
three immutable synthetic geometry fixtures:
two M3 images and one C1 annotated map. They are controlled evaluation inputs,
not field captures; replace them with frozen real imagery only if the study
needs a real-world imagery claim.

M2 is text-only: BTGenBot-2 requests never include image data. Run M2 without
`--multimodal`; that flag performs a mission-level image preflight before
method dispatch and can block M2 even though its request would contain only
text.

## BTGenBot-2 on Colab

The recommended workflow is batch execution:

1. After freezing the protocol, make an M2 dry run with `--scored` and the
   compiled `--factory-helper` so the exact requests and hashes are recorded.
   A Colab URL is not required for this step.
2. Export them:

   ```bash
   python3 evaluation/scripts/export_btgenbot2_batch.py
   ```

3. Upload the JSONL file to Colab and run `run_batch(...)`.
4. Download the raw output JSONL and import it:

   ```bash
   python3 evaluation/scripts/import_btgenbot2_batch.py \
     evaluation_outputs.jsonl \
     --scored \
     --factory-helper install/bt_executor/lib/bt_executor/bt_factory_check
   ```

The optional tunnel requires `COLAB_EVAL_TOKEN` on both sides. The server has no
wildcard CORS policy and exposes no unauthenticated generation route.

The AIRLab-POLIMI repository contains a complete checkpoint despite its LoRA
name. The notebook loads its model and tokenizer directly; separate gated base
weights and PEFT merging are not needed. Historical `adapter` fields record the
full checkpoint revision. The pinned base revision is provenance metadata.

The notebook returns `raw_text` and a separate `extracted_xml` diagnostic.
Only `raw_text` is used for first-attempt scoring. It also records exact model
revisions, package versions, GPU, dtype, seed, token counts, latency, and finish
reason. Warm-up latency is excluded.

## BTGenBot-2 local chat endpoint

`model_conditions.json` points to the local OpenAI-compatible endpoint and
records the tested `max_tokens`, `temperature`, and `top_p` values. Run M2 with
the default URL, or override it with `--btgenbot-url`:

```bash
python3 evaluation/scripts/run_evaluation.py \
  --experiment E1 --methods M2 --mission S2 --paraphrase S2-P1 \
  --output-dir evaluation/results/btgenbot2_local_pilot \
  --log evaluation/results/btgenbot2_local_pilot/run_log.jsonl \
  --manifest evaluation/results/btgenbot2_local_pilot/manifest.json
```

The runner sends the released BTGenBot-2 system prompt and a compact behavior
summary followed by `Actions: [ActionName (parameters: ...)]`. It uses sampled
generation with a 500-token limit, matching the released inference notebook,
but retains one attempt for scored E1. Complete context is serialized as compact
JSON and the full deployed action library is retained. The runner preserves the
raw completion, finish reason, usage, returned model ID, server fingerprint, and
top-level `metadata` when present. The frozen Jetson endpoint does not expose
revision metadata, so the protocol records the operator's stability assurance
and the missing metadata instead of rejecting the run. Returned conflicting
metadata is still rejected. `--colab-url` remains available for the older
`/generate` service.

Each non-dry-run M2 result directory also contains `response.xml`. Complete
`<root>...</root>` responses are extracted there; malformed or truncated
responses are saved as returned for inspection. `result.json` records whether
the file contains an extracted XML document.

The endpoint reports a 131,072-token context limit. Context-rich missions remain
far outside BTGenBot-2's 1,180-token fine-tuning sequence length even after
compact serialization; record this as a method limitation.

After the server restart, pilot the full M2 matrix with extra time for long
prefills:

```bash
python3 evaluation/scripts/run_evaluation.py \
  --experiment E1 --methods M2 --platform all \
  --timeout-s 300 --max-transport-retries 0 \
  --output-dir evaluation/results/btgenbot2_e1_local_pilot \
  --log evaluation/results/btgenbot2_e1_local_pilot/run_log.jsonl \
  --manifest evaluation/results/btgenbot2_e1_local_pilot/manifest.json
```

The official notebook example passed on the Jetson. Three compact S1 attempts
failed XML syntax because the required deployed action names do not occur in the
released training dataset. Evidence is stored in
`evaluation/results/btgenbot2_prompt_contract_2026-09-26/summary.json`.

## Scoring and claims

```bash
python3 evaluation/scripts/export_blind_review.py \
  --input-dir evaluation/raw_outputs \
  --input-dir evaluation/results/e1_m3_scoring_ready

# Fill blind_review_responses.jsonl, then aggregate it with the hidden key.
python3 evaluation/scripts/score_results.py \
  --input-dir evaluation/raw_outputs \
  --input-dir evaluation/results/e1_m3_scoring_ready \
  --reviews evaluation/results/blind_review_responses.jsonl \
  --review-key evaluation/results/blind_review_key.json
```

Both commands use only artifacts explicitly created with `--scored`. Use
`--include-unscored` only to inspect pilots. The summary reports transport and
protocol errors separately so missing results remain visible.

The scorer keeps first-attempt validity, first-attempt task success, final
automated task success, and human semantic pass separate. Every applicable
semantic element receives 0, 1, or 2; a semantic pass requires 2 for every
element. One reviewer scores every interpretable artifact. A second reviewer
scores all complex and adverse artifacts and a deterministic 20 percent sample
of the rest. Disagreements require adjudicated scores.

For E1, the response template also records timed manual repair. The fixed
budget is 300 seconds. Count one correction per validator or rubric finding,
not per changed line or waypoint. Valid artifacts are `not_needed`; failed
repairs are `attempted_failed`; omitted repairs are `not_attempted`, with null
effort values. A `corrected` response cites the repaired artifact and saved
validation record, records a passing deterministic recheck, and scores every
original semantic element again. M3 validator-triggered payload calls are
automatic refinements.
Transport retries remain a separate provider-reliability metric.

The summary groups E1 by platform, specificity, and complexity; E2 by context
condition; E3 by scale; and E4 by adverse type and expected outcome. Primary
intervals resample mission clusters 10,000 times. Intended pairs use a
mission-cluster interval and exact sign-flip test.

E5 physical-execution records can be scored with:

```bash
python3 evaluation/scripts/score_results.py \
  --reviews evaluation/results/blind_review_responses.jsonl \
  --review-key evaluation/results/blind_review_key.json \
  --execution-file evaluation/results/execution_trials.json
```

The file follows `protocol/execution_scoring.json`. Each trial cites its passed
planning condition. It records the frozen route and measurement denominators,
terminal outcome, duration, interventions, safety incidents, factory loading,
integration checks, and evidence for all eight portability components. Copy
the protocol file's SHA-256 into `execution_scoring_sha256` before trials begin.
Put a planned trial that cannot start in `not_started` with its reason; the
scorer reports it separately from an unrecorded planned trial.

For the M1 scale conditions, each artifact also records the exact node order,
condition-manifest hash, selected distractors, invented nodes, and port errors.
The deterministic scorer does not prescribe a node combination; blinded semantic
review decides whether each valid tree accomplishes the mission. The scorer groups
E3 results by model and scale level.
Control nodes remain fixed and are excluded from the 11/50/100 counts.
The source plugin registers 16 custom BT identifiers, including six BlueBoat
interfaces. E3 deliberately retains the original ten-node Husky subset listed in
`base_node_descriptions`; its four libraries add 2, 14, 40, and 90 protocol-only
alternatives, respectively. E1 advertises the complete registered-node manifest. These options
are presented uniformly in the prompt and are never identified as distractors
to the evaluated model. Added nodes use the same PascalCase convention as the
compiled BT interfaces; examples include `MoveToLocation`,
`AnalyzeSensorSource`, `ResolveNamedLocation`, and `ValidateCollectedData`.

Static XML validation checks raw parsing, registered names, ports, required
inputs, and blackboard data flow. It does not pretend to be the real factory
loader. `factory_load` is `not_run` unless a compiled helper is supplied with
`--factory-helper`; `not_run` never counts as first-attempt validity. Scored M1
and M2 runs require the executable helper. After building and sourcing the ROS
workspace, use:

```bash
--factory-helper install/bt_executor/lib/bt_executor/bt_factory_check
```

Safety evidence is limited to the guards actually tested:

- unsupported capability and wrong-platform refusal;
- missing-area clarification;
- range admission and blocked-region review;
- map/GPS contradiction handling;
- geofence and blocked-region review;
- prompt-injection coordinate rejection.

These checks support a claim of reduced unsafe acceptance under the tested
conditions. They do not prove general robot safety. Runtime collision
avoidance, emergency stops, communication loss, actuator faults, weather, and
unseen hazards require separate ROS/execution evidence.

## Freeze checklist

Before using `--scored`:

1. ✅ The offline BlueBoat contract, capability snapshot, and factory load are
   verified. Physical ROS/Gazebo execution remains separate E5 evidence.
2. Select exactly one OpenRouter provider per general model and verify the
   returned provider in a pilot.
3. Pin the BTGenBot-2 base and adapter commit hashes.
4. ✅ Add immutable synthetic context images and hashes, or explicitly label
   the study structured-text-only if those images are not retained.
5. ✅ Compile and exercise the BT.CPP factory-load helper.
6. Run pilots and exclude their outputs from the scored directory.
7. Regenerate the protocol snapshot:

   ```bash
   python3 evaluation/scripts/freeze_protocol.py \
     --update-dataset-snapshot
   ```

8. Review all routes and rubrics, then set the core missions, context variants,
   E3 protocols, safety cases, scoring rubric, and execution scoring protocol
   to `frozen`. Commit the snapshot and do not change prompts or validators
   after observing scored failures.
