#!/usr/bin/env python3
"""Regenerate the checked-in Colab notebook from the reviewed server source."""

from __future__ import annotations

import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "btgenbot2_server.py"
OUTPUT = HERE / "btgenbot2_server.ipynb"


def code_cell(source: str) -> dict:
    return {
        "cell_type": "code",
        "execution_count": None,
        "metadata": {},
        "outputs": [],
        "source": [line + "\n" for line in source.splitlines()],
    }


def markdown_cell(source: str) -> dict:
    return {
        "cell_type": "markdown",
        "metadata": {},
        "source": [line + "\n" for line in source.splitlines()],
    }


def main() -> int:
    server_source = SOURCE.read_text(encoding="utf-8")
    notebook = {
        "cells": [
            markdown_cell(
                """# BTGenBot-2 thesis evaluation

This notebook runs the frozen BTGenBot-2 baseline. It preserves `raw_text`; `extracted_xml` is diagnostic only.

Before running:

1. Select a GPU runtime.
2. Accept the Hugging Face access terms for both the base and adapter repositories.
3. Set `HF_TOKEN`, `COLAB_EVAL_TOKEN`, and `NGROK_TOKEN` in Colab Secrets.
4. For scored runs, also set the two exact revision hashes shown in `evaluation/protocol/model_conditions.json`.

Prefer batch mode when possible. The authenticated tunnel is provided for small pilot runs."""
            ),
            code_cell(
                """# Pinned software environment. Torch is supplied by the selected Colab runtime.
!pip install -q \\
  transformers==4.56.2 \\
  peft==0.17.1 \\
  accelerate==1.10.1 \\
  huggingface-hub==0.34.4 \\
  fastapi==0.116.1 \\
  uvicorn==0.35.0 \\
  pyngrok==7.3.0 \\
  nest-asyncio==1.6.0"""
            ),
            code_cell(
                """# Load secrets without printing them.
from google.colab import userdata
import os

for name in (
    "HF_TOKEN",
    "COLAB_EVAL_TOKEN",
    "NGROK_TOKEN",
    "BTGENBOT_BASE_REVISION",
    "BTGENBOT_ADAPTER_REVISION",
):
    try:
        value = userdata.get(name)
    except Exception:
        value = None
    if value:
        os.environ[name] = value"""
            ),
            code_cell(server_source),
            markdown_cell(
                """## Recommended: deterministic batch mode

Upload `evaluation_requests.jsonl`, run the next cell, then download `evaluation_outputs.jsonl`. Each request needs `task`, `actions`, `seed`, and optionally `max_new_tokens` and `request_id`."""
            ),
            code_cell(
                """# Uncomment after uploading the frozen request file.
# run_batch("/content/evaluation_requests.jsonl", "/content/evaluation_outputs.jsonl")"""
            ),
            markdown_cell(
                """## Optional: authenticated pilot endpoint

The URL is public, but every request requires the secret bearer token. Do not share either value. Colab sessions are temporary, so this mode is not the preferred scored-run path."""
            ),
            code_cell(
                """import nest_asyncio
import uvicorn
from pyngrok import ngrok

nest_asyncio.apply()
ngrok.set_auth_token(os.environ["NGROK_TOKEN"])
public_url = ngrok.connect(8000).public_url
print("Authenticated endpoint:", public_url)
print("Use the same COLAB_EVAL_TOKEN in the local evaluation runner.")
uvicorn.run(app, host="0.0.0.0", port=8000)"""
            ),
        ],
        "metadata": {
            "accelerator": "GPU",
            "colab": {"provenance": []},
            "kernelspec": {
                "display_name": "Python 3",
                "language": "python",
                "name": "python3",
            },
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 0,
    }
    OUTPUT.write_text(json.dumps(notebook, indent=1) + "\n", encoding="utf-8")
    print(f"Wrote {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
