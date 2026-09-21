# Thesis evaluation

This folder is the machine-readable evaluation source of truth. It separates
the direct-generation baselines from the thesis pipeline and preserves every
raw first model output.

## Scope and current status

- The dataset is still a draft pending final freeze and scored runs.
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

Dataset version `0.4.0-draft` assigns P1, P2, and P3 to low, medium, and high
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
  reference model. The two missions and six conditions produce 36 outputs.
- `E3`: method-specific choice-space scaling. M1 defines libraries of 12,
  24, 50, and 100 custom-node identifiers. M3 separately varies the tree
  catalogue between the expected tree plus one incompatible tree and all seven
  current trees. M1 produces 252 outputs; M3 produces 54.
- `E4`: adverse requests and safety/failure quality. Clarification and refusal
  are correct outcomes for predefined cases.
- `E5`: three physical executions of the accepted GPT-5.6-Sol M3 P2 plan for
  each of the nine missions. This is not produced by the offline runner and
  must be reported separately.

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
- `scripts/score_results.py`: per-run tables, mission-cluster intervals, paired
  effects, human ratings, correction effort, and optional E5 summaries.
- `scripts/export_blind_review.py`: removes model and method labels for manual
  semantic review and creates the response template.
- `colab/btgenbot2_server.ipynb`: authenticated BTGenBot-2 endpoint and batch
  workflow.

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
  --paraphrase S1-P1 \
  --variant M1-N100
```

The normal output directory is ignored by Git. Each deterministic condition ID
contains `request.json` and `result.json`. A run manifest records dataset
hashes, repository snapshot, seeds, host details, counts, and costs. Existing
condition IDs are skipped unless `--force` is used.

Use `--repetitions`, `--seed`, `--max-cost-usd`, and `--max-transport-retries`
for controlled pilots. Transport retries preserve one model output; M3
validation/refinement attempts are separate stages. `first_attempt_valid` is
always computed from the original raw output.

Multimodal mode is opt-in:

```bash
python3 evaluation/scripts/run_evaluation.py --multimodal ...
```

It blocks when an image file or hash is missing. The repository now includes
three immutable synthetic geometry fixtures:
two M3 images and one C1 annotated map. They are controlled evaluation inputs,
not field captures; replace them with frozen real imagery only if the study
needs a real-world imagery claim.

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

## Scoring and claims

```bash
python3 evaluation/scripts/export_blind_review.py

# Fill blind_review_responses.jsonl, then aggregate it with the hidden key.
python3 evaluation/scripts/score_results.py \
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
condition-manifest hash, required-node recall, selected distractors, invented
nodes, and port errors. The scorer groups E3 results by model and scale level.
Control nodes remain fixed and are excluded from the 12/24/50/100 counts.
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
