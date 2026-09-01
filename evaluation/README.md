# Thesis evaluation

This folder is the machine-readable evaluation source of truth. It separates
the direct-generation baselines from the thesis pipeline and preserves every
raw first model output.

## Scope and current status

- The dataset is still a draft.
- It contains nine missions with three paraphrases each.
- Five Husky missions are implemented in the current catalogue.
- Four BlueBoat missions specify the intended multi-platform evaluation but are
  blocked until the BlueBoat capability model, nodes, trees, and execution
  evidence exist.
- The designed E1 matrix contains 189 outputs: 27 prompts times six
  general-model method conditions plus BTGenBot-2.
- The currently executable Husky subset contains 105 outputs: 15 prompts times
  seven conditions. BlueBoat cases must not enter supported-mission success
  rates while blocked.
- No user study is planned.

The runner refuses a scored run while the dataset, providers, or BTGenBot-2
revisions are not frozen. Preliminary and dry runs remain available.

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
- `E2`: M3 context ablation with complete, text-only, missing-source,
  contradictory, and irrelevant-distractor variants.
- `E3`: method-specific choice-space scaling. M1 defines libraries of 12,
  24, 50, and 100 custom-node identifiers. M3 separately varies the tree
  catalogue; its expanded synthetic-tree condition remains blocked.
- `E4`: adverse requests and safety/failure quality. Clarification and refusal
  are correct outcomes for predefined cases.
- `E5`: ROS/Gazebo or physical execution evidence. This is not produced by the
  offline runner and must be reported separately.

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
- `scripts/run_evaluation.py`: E1–E4 runner.
- `scripts/score_results.py`: per-run tables, Wilson intervals, and exact paired
  McNemar comparisons.
- `scripts/export_blind_review.py`: removes model and method labels for manual
  semantic review.
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

It blocks when an image file or hash is missing. The current placeholder
metadata therefore cannot be claimed as multimodal evidence.

## BTGenBot-2 on Colab

The recommended workflow is batch execution:

1. Make an M2 dry run so the exact requests are recorded. A Colab URL is not
   required for this step.
2. Export them:

   ```bash
   python3 evaluation/scripts/export_btgenbot2_batch.py
   ```

3. Upload the JSONL file to Colab and run `run_batch(...)`.
4. Download the raw output JSONL and import it:

   ```bash
   python3 evaluation/scripts/import_btgenbot2_batch.py \
     evaluation_outputs.jsonl
   ```

The optional tunnel requires `COLAB_EVAL_TOKEN` on both sides. The server has no
wildcard CORS policy and exposes no unauthenticated generation route.

The notebook returns `raw_text` and a separate `extracted_xml` diagnostic.
Only `raw_text` is used for first-attempt scoring. It also records exact model
revisions, package versions, GPU, dtype, seed, token counts, latency, and finish
reason. Warm-up latency is excluded.

## Scoring and claims

```bash
python3 evaluation/scripts/score_results.py
python3 evaluation/scripts/export_blind_review.py
```

Both commands use only artifacts explicitly created with `--scored`. Use
`--include-unscored` only to inspect pilots. The summary reports transport and
protocol errors separately so missing results remain visible.

For E1–E3, the primary automated metric is task success. It includes interface
validity and the available deterministic semantic checks. For E4, the primary
metric is the correct plan/clarification/refusal outcome; an expected plan must
also pass the deterministic safety review. Manual semantic scores remain
separate.

For the M1 scale conditions, each artifact also records the exact node order,
condition-manifest hash, required-node recall, selected distractors, invented
nodes, and port errors. The scorer groups E3 results by model and scale level.
Control nodes remain fixed and are excluded from the 12/24/50/100 counts.
The latest compiled plugin exposes 10 custom BT nodes because `FindAnything`
now runs during pre-BT context gathering. Consequently, the four M1 libraries
add 2, 14, 40, and 90 protocol-only alternatives, respectively. These options
are presented uniformly in the prompt and are never identified as distractors
to the evaluated model. Added nodes use the same PascalCase convention as the
compiled BT interfaces; examples include `MoveToLocation`,
`AnalyzeSensorSource`, `ResolveNamedLocation`, and `ValidateCollectedData`.

Static XML validation checks raw parsing, registered names, ports, required
inputs, and blackboard data flow. It does not pretend to be the real factory
loader. `factory_load` is `not_run` unless a compiled helper is supplied with
`--factory-helper`. After building and sourcing the ROS workspace, use:

```bash
--factory-helper install/bt_executor/lib/bt_executor/bt_factory_check
```

Safety evidence is limited to the guards actually tested:

- unsupported capability and wrong-platform refusal;
- missing-area clarification;
- range and battery admission;
- map/GPS contradiction handling;
- geofence and blocked-region review;
- prompt-injection coordinate rejection.

These checks support a claim of reduced unsafe acceptance under the tested
conditions. They do not prove general robot safety. Runtime collision
avoidance, emergency stops, communication loss, actuator faults, weather, and
unseen hazards require separate ROS/execution evidence.

## Freeze checklist

Before using `--scored`:

1. Implement and verify BlueBoat, or formally narrow the empirical scope and
   revise the dataset counts.
2. Select exactly one OpenRouter provider per general model and verify the
   returned provider in a pilot.
3. Pin the BTGenBot-2 base and adapter commit hashes.
4. Add immutable context images and hashes, or label the study
   structured-text-only.
5. Compile and exercise the BT.CPP factory-load helper.
6. Run pilots and exclude their outputs from the scored directory.
7. Regenerate the protocol snapshot:

   ```bash
   python3 evaluation/scripts/freeze_protocol.py \
     --update-dataset-snapshot
   ```

8. Review all routes and rubrics, change the relevant freeze fields to
   `frozen`, including both E3 protocol files, commit the snapshot, and do not
   change prompts or validators after observing scored failures.
