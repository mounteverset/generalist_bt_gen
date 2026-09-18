"""Authenticated BTGenBot-2 evaluation service and deterministic batch runner.

This file is mirrored into btgenbot2_server.ipynb by make_colab_notebook.py.
"""

from __future__ import annotations

import importlib.metadata
import json
import os
import platform
import secrets
import threading
import time
from pathlib import Path
from typing import Any, Optional

import torch
from fastapi import Depends, FastAPI, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from huggingface_hub import model_info
from pydantic import BaseModel, Field
from transformers import AutoModelForCausalLM, AutoTokenizer, set_seed


BASE_MODEL = "meta-llama/Llama-3.2-1B-Instruct"
ADAPTER_MODEL = "AIRLab-POLIMI/llama-3.2-1b-it-ft-lora-bt"
BASE_REVISION = os.environ.get("BTGENBOT_BASE_REVISION", "").strip()
ADAPTER_REVISION = os.environ.get("BTGENBOT_ADAPTER_REVISION", "").strip()
HF_TOKEN = os.environ.get("HF_TOKEN", "").strip()
EVAL_AUTH_TOKEN = os.environ.get("COLAB_EVAL_TOKEN", "").strip()

if not HF_TOKEN:
    raise RuntimeError(
        "HF_TOKEN is required. Accept the gated model terms before starting Colab."
    )
if not EVAL_AUTH_TOKEN:
    raise RuntimeError(
        "COLAB_EVAL_TOKEN is required. Use the same long random token in the local runner."
    )

RESOLVED_BASE_REVISION = BASE_REVISION or model_info(
    BASE_MODEL, token=HF_TOKEN
).sha
RESOLVED_ADAPTER_REVISION = ADAPTER_REVISION or model_info(
    ADAPTER_MODEL, token=HF_TOKEN
).sha
REVISION_FREEZE_STATUS = (
    "frozen"
    if BASE_REVISION and ADAPTER_REVISION
    else "unfrozen_resolved_at_runtime"
)

tokenizer = AutoTokenizer.from_pretrained(
    ADAPTER_MODEL,
    revision=RESOLVED_ADAPTER_REVISION,
    token=HF_TOKEN,
)
if tokenizer.pad_token is None:
    tokenizer.pad_token = tokenizer.eos_token

# Despite its repository name, the published artifact is a complete checkpoint.
model = AutoModelForCausalLM.from_pretrained(
    ADAPTER_MODEL,
    revision=RESOLVED_ADAPTER_REVISION,
    torch_dtype="auto",
    device_map="auto",
    token=HF_TOKEN,
)
model.eval()
if torch.cuda.is_available():
    torch.cuda.empty_cache()

GENERATION_LOCK = threading.Lock()


def package_versions() -> dict[str, str]:
    names = [
        "transformers",
        "accelerate",
        "torch",
        "fastapi",
        "uvicorn",
        "huggingface-hub",
    ]
    versions: dict[str, str] = {}
    for name in names:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "not-installed"
    return versions


ENVIRONMENT = {
    "checkpoint_format": "full_model",
    "base_model": BASE_MODEL,
    "base_revision": RESOLVED_BASE_REVISION,
    "adapter_model": ADAPTER_MODEL,
    "adapter_revision": RESOLVED_ADAPTER_REVISION,
    "revision_freeze_status": REVISION_FREEZE_STATUS,
    "python": platform.python_version(),
    "packages": package_versions(),
    "torch_dtype": str(next(model.parameters()).dtype),
    "device": str(next(model.parameters()).device),
    "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "cpu",
}


def extract_xml(raw_text: str) -> Optional[str]:
    start = raw_text.find("<root")
    end = raw_text.find("</root>", start + 1)
    if start < 0 or end < 0:
        return None
    return raw_text[start : end + len("</root>")]


def generate_bt(
    task: str,
    actions: str,
    *,
    max_new_tokens: int = 2048,
    seed: int = 42,
) -> dict[str, Any]:
    input_text = f"Task:\n{task}\n\nActions:\n{actions}"
    messages = [
        {
            "role": "system",
            "content": (
                "You are a Behavior Tree generator. Return the requested raw output only."
            ),
        },
        {"role": "user", "content": input_text},
    ]
    prompt = tokenizer.apply_chat_template(
        messages,
        tokenize=False,
        add_generation_prompt=True,
    )
    inputs = tokenizer(prompt, return_tensors="pt").to(model.device)
    set_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    started = time.monotonic()
    with GENERATION_LOCK, torch.inference_mode():
        outputs = model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            do_sample=False,
            pad_token_id=tokenizer.pad_token_id,
        )
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    latency_s = time.monotonic() - started
    generated_ids = outputs[0][inputs.input_ids.shape[1] :]
    raw_text = tokenizer.decode(generated_ids, skip_special_tokens=True).strip()
    hit_eos = bool(
        generated_ids.numel()
        and generated_ids[-1].item() in set(tokenizer.all_special_ids)
    )
    return {
        "raw_text": raw_text,
        "extracted_xml": extract_xml(raw_text),
        "metadata": {
            **ENVIRONMENT,
            "seed": seed,
            "input_tokens": int(inputs.input_ids.shape[1]),
            "output_tokens": int(generated_ids.shape[0]),
            "max_new_tokens": max_new_tokens,
            "latency_s": round(latency_s, 3),
            "finish_reason": "eos" if hit_eos else "length_or_other",
        },
    }


def warm_up() -> None:
    generate_bt(
        "Return a one-action tree.",
        "Wait(seconds: double)",
        max_new_tokens=16,
        seed=0,
    )


def run_batch(input_jsonl: str, output_jsonl: str) -> None:
    """Run frozen requests without depending on a public tunnel."""
    input_path = Path(input_jsonl)
    output_path = Path(output_jsonl)
    with input_path.open("r", encoding="utf-8") as source, output_path.open(
        "w", encoding="utf-8"
    ) as target:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue
            request = json.loads(line)
            response = generate_bt(
                request["task"],
                request["actions"],
                max_new_tokens=int(request.get("max_new_tokens", 2048)),
                seed=int(request.get("seed", 42)),
            )
            response["request_id"] = request.get("request_id", line_number)
            target.write(json.dumps(response, ensure_ascii=False) + "\n")
            target.flush()


security = HTTPBearer(auto_error=False)
app = FastAPI(title="BTGenBot-2 thesis evaluation server")


def authorize(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
) -> None:
    if (
        credentials is None
        or credentials.scheme.lower() != "bearer"
        or not secrets.compare_digest(credentials.credentials, EVAL_AUTH_TOKEN)
    ):
        raise HTTPException(status_code=401, detail="Unauthorized")


class GenerateRequest(BaseModel):
    task: str = Field(min_length=1, max_length=100000)
    actions: str = Field(min_length=1, max_length=100000)
    max_new_tokens: int = Field(default=2048, ge=1, le=4096)
    seed: int = 42


@app.get("/health", dependencies=[Depends(authorize)])
def health() -> dict[str, Any]:
    return {"status": "ok", **ENVIRONMENT}


@app.post("/generate", dependencies=[Depends(authorize)])
def generate(request: GenerateRequest) -> dict[str, Any]:
    try:
        return generate_bt(
            request.task,
            request.actions,
            max_new_tokens=request.max_new_tokens,
            seed=request.seed,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


print(json.dumps(ENVIRONMENT, indent=2))
print("Running one unscored warm-up generation...")
warm_up()
print("Warm-up complete. Its latency is not included in evaluation results.")
