# E1 scoring inputs

Use these frozen definitions:

- Missions: `evaluation/protocol/core_missions.json`
- Context: `evaluation/fixtures/context/core_contexts.json`
- Rubric: `evaluation/protocol/scoring_rubric.json`
- Models: `evaluation/protocol/model_conditions.json`
- Runtime: `evaluation/protocol/runtime_contract.json`
- Runtime amendment: `evaluation/protocol/e1_runtime_runner_amendment_20260928.json`
- M3 scoring correction: `evaluation/protocol/e1_m3_scoring_correction_amendment_20260928.json`
- M3 first-output adjudication: `evaluation/protocol/e1_m3_first_output_adjudication_20260928.json`
- M3 GPT direct fallback: `evaluation/protocol/e1_m3_gpt_openai_fallback_amendment_20260928.json`
- BlueBoat amendment: `evaluation/protocol/e1_blueboat_location_amendment_20260928.json`
- BlueBoat map artifacts: `evaluation/fixtures/artifacts/BLUEBOAT_FERINGASEE/`

Location rule:

- Husky missions use the frozen Hollerner See fixtures.
- BlueBoat missions S2 and M2 use Feringasee only.

Use these result folders:

- M1: `evaluation/results/scoring_ready/`, select `request.json.method == "M1"`. Use all 81 records.
- M2: `evaluation/results/scoring_ready/`, select `request.json.method == "M2"`. Use all 27 records. Corrected BlueBoat rows are `25, 32, 39, 88, 95, 102`.
- M3: use all 81 records from three model-specific selections:
  - Gemini: use all 27 Gemini records in `evaluation/results/e1_m3_scoring_ready/`. Corrected rows are `118, 160`.
  - Gemma: use all 27 records in `evaluation/results/e1_m3_gemma_full_scoring_ready/`. Corrected rows are `112, 140, 161, 168`.
  - GPT-5.6-Sol: use all 27 records in `evaluation/results/e1_m3_gpt_scoring_ready/`. Corrected rows are `19, 61, 82, 110, 124, 131, 138, 145, 152, 159, 166`; row `145` uses the direct OpenAI fallback.
    Rows `124, 152, 159, 166` use their saved first payloads under the first-output adjudication. No model rerun was performed.
  - Corrected Gemini BlueBoat rows are `27, 34, 41, 90, 97, 104`.

Do not use:

- Gemma records outside `evaluation/results/e1_m3_gemma_full_scoring_ready/`; older records are superseded.
- Anything under `evaluation/results/unusable/` or `evaluation/pilots/`.
- `evaluation/raw_outputs/` directly for final scoring.

Current usable total: 189 of 189 E1 outputs. No E1 conditions are pending.

Score with `score_results.py --selection evaluation/protocol/e1_scoring_selection.json`. After new runs or reruns are imported, regenerate the selection with `scripts/build_e1_selection.py`; it is frozen once all 189 conditions have exactly one usable record. Retired records are listed in `protocol/e1_superseded_records.json`.

Preliminary automatic tables without human review are in `evaluation/results/e1_automatic_scoring_20260928/`.
