# Reuse audit — revised mission set

The mission descriptions and context fixtures were revised after the first M1/M2 run. The current dataset validates and defines 9 missions, 27 paraphrases, and 189 supported offline E1 conditions across seven Husky and two BlueBoat missions. Three immutable synthetic image fixtures are now available for M3 and C1 multimodal checks.

## Reusable artifacts

- `evaluation/.venv_btgenbot312/`: pinned Python 3.12 inference environment.
- Hugging Face checkpoint cache and metadata for `AIRLab-POLIMI/llama-3.2-1b-it-ft-lora-bt`, revision `8baa45e3b3c4d3788c9884347ef5d9958f8984d4`.
- BTGenBot server/notebook setup and model-loading correction (the repository is a full checkpoint despite its LoRA name).
- Isolated `bt_factory_check` build and its smoke-test logs.
- Runtime/node metadata and validators, with the protocol snapshot regenerated after the BlueBoat and image updates.
- `evaluation/results/reworked_e1_dry_run_2026-09-14/`: 189 fresh E1 request/result envelopes with zero skipped or blocked conditions.
- `evaluation/results/reworked_e1_gemini38_dry_run_2026-09-14/`: 189 fresh E1 request/result envelopes using the current Gemini 3.8 condition.
- `evaluation/results/m2_reworked_e1_2026-09-14/requests.jsonl`: 27 fresh BTGenBot-2 requests tied to the current dataset hashes.
- Raw M1 and M2 outputs as historical pilot evidence and failure examples.

## Not reusable as scored results

The prior M1 E1 manifest records core dataset hash `20aa9789c69d51623a7f989097a4276354dfe55e48bf1b4612351940611a4e0c` and context hash `d71cd249f4077faf8d85fe6faeef0786c046ab7d7ef0e62a90a64a216cfa164e`. The revised files hash to:

- core missions: `232e26b55c72d3c91bc21b84d4ec89955fb3bd1fff4c6f7e1311a047b13d29b5`;
- context fixtures: `889cfba9f4b58deecd6dde61733d5b84f6d29caf0313b5827ed2edc8e0d33050`.

Therefore all prior generated XML, M2 batch outputs, and dry-run requests are tied to the previous wording/context and must not enter revised-dataset success rates. They can remain for provenance, debugging, and qualitative comparison. The revised dataset now has fresh deterministic requests; fresh M1/M3 outputs still require the OpenRouter quota, and the M2 batch still needs to be executed.

The Gemini model change invalidates earlier Gemini 3.5 outputs as scored evidence; those remain historical artifacts.

## Regeneration sequence

1. Freeze/review the revised mission and context files; preserve the current 7-Husky/2-BlueBoat scope.
2. Run fresh M1 and M3 requests for all 27 prompts; do not reuse the old M1 campaign.
3. Run the exported M2 batch with the pinned checkpoint or Colab endpoint; the request batch is ready.
4. Re-run scoring and blind-review export; label all previous artifacts `historical_pilot`.

The revised dataset still has draft protocol/freeze fields and needs fresh scored outputs. The offline BlueBoat contract, factory-load evidence, and image hashes are complete; physical ROS execution remains separate E5 evidence.
