# Thesis Evaluation Dataset

This directory contains the machine-readable evaluation dataset for the thesis.
It is the source of truth for the scored mission set. Earlier scenario documents
remain useful as planning history, but they must not override this dataset after
the evaluation is frozen.

## Current status

- Dataset version: `0.1.0-draft`
- Freeze status: not frozen
- Core missions: 9
- Paraphrases per mission: 3
- Complexity split: 3 simple, 3 medium, 3 complex
- Platform split: 5 Husky, 4 BlueBoat
- Primary model/method conditions: 7
- Planned E1 outputs: `9 × 3 × 7 = 189`

The five Husky missions are supported by the current catalogue snapshot. The
four BlueBoat missions define the required evaluation contract but are blocked
from scored runs until a BlueBoat capability model, tree, action adapters, and
validation evidence exist. This status is encoded per mission.

## Files

- `protocol/core_missions.json`: nine supported-domain missions, paraphrases,
  expected tree and payload, complexity score, and semantic scoring elements.
- `fixtures/context/core_contexts.json`: fixed context fixture for every core
  mission. All compared methods receive evidence derived from the same fixture.
- `protocol/safety_cases.json`: clarification, refusal, and adverse-context
  cases. These are reported separately from supported mission success.
- `protocol/complexity_rubric.json`: predeclared complexity dimensions and
  score ranges.
- `protocol/scoring_rubric.json`: common semantic and failure-quality scoring
  rules.
- `protocol/dataset_schema.json`: JSON Schema for the core mission file.
- `scripts/validate_dataset.py`: dependency-free structural consistency check.

## Compared methods

- `M1`: a general-purpose LLM generates complete BT.CPP XML from the mission,
  fixed context, and exact registered node/port catalogue.
- `M2`: BTGenBot-2 generates complete BT.CPP XML from the mission and compatible
  action specification.
- `M3`: the thesis system checks capabilities, selects a frozen tree, generates
  a payload, reviews the plan, and either accepts, clarifies, or refuses.

For each general-purpose model, M1 and M3 must use the same fixed model version.
The three-model design creates six general-model conditions, plus BTGenBot-2.

## Reference answers

Reference payloads are canonical acceptable answers for deterministic tests.
Equivalent routes may also be semantically correct when they satisfy every
mission-specific rubric element. A result must not be marked wrong only because
its valid waypoint sequence differs from the reference sequence.

For M1 and M2, `reference_behavior` describes the required behavior rather than
forcing one exact XML layout. Generated XML is still checked against the real
registered nodes, ports, plugins, and BT.CPP loader.

## Freeze rules

Before scored runs:

1. Implement and verify the BlueBoat contract or remove the blocked missions and
   explicitly narrow the thesis scope.
2. Freeze exact model identifiers and decoding settings.
3. Freeze the tree catalogue and direct-generation action catalogue.
4. Review every reference route against its map fixture.
5. Pilot all validators, then exclude pilot outputs from scored results.
6. Record the final repository commit and file hashes in the dataset metadata.
7. Change `freeze_status` to `frozen` and increment the dataset version.

Do not add a new tree after seeing a scored failure and count the rerun as an
original success.

## Validation

Run from the repository root:

```bash
python3 evaluation/scripts/validate_dataset.py
```

The validator checks counts, IDs, platform balance, complexity totals, context
references, paraphrase uniqueness, expected outcomes, and implementation status.

## Known blockers

- BlueBoat platform description and executable tree do not yet exist.
- The three general-purpose model identifiers are not yet frozen.
- The exact direct-generation control/action catalogue still needs a final port
  snapshot from the compiled plugin set.
- Context image artifacts are represented by metadata placeholders in this
  first version. The final fixtures must include immutable files and hashes.
