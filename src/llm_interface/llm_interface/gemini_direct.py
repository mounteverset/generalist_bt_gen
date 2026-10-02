from __future__ import annotations

import base64
import json
import os
import re
import time
from pathlib import Path
from typing import Any
from urllib.parse import quote
from urllib.request import Request, urlopen

from langchain_core.messages import AIMessage
from langchain_core.runnables import RunnableLambda


INPUT_USD_PER_MILLION = 0.75
CACHED_INPUT_USD_PER_MILLION = 0.075
OUTPUT_USD_PER_MILLION = 3.75
MAX_INPUT_TOKENS = 1_048_576


def _write_ledger(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def _reserve_cost(max_output_tokens: int) -> tuple[Path, float] | None:
    raw_path = os.environ.get('GENERALIST_BT_COST_LEDGER', '').strip()
    raw_limit = os.environ.get('GENERALIST_BT_COST_LIMIT_USD', '').strip()
    if not raw_path or not raw_limit:
        return None
    path = Path(raw_path).expanduser().resolve()
    value = json.loads(path.read_text(encoding='utf-8')) if path.is_file() else {}
    actual = float(value.get('actual_cost_usd', 0.0) or 0.0)
    reserved = float(value.get('reserved_cost_usd', 0.0) or 0.0)
    limit = float(raw_limit)
    maximum = (
        MAX_INPUT_TOKENS * INPUT_USD_PER_MILLION
        + max_output_tokens * OUTPUT_USD_PER_MILLION
    ) / 1_000_000
    if actual + reserved + maximum > limit:
        raise RuntimeError(
            f'Gemini cost cap would be exceeded: spent/reserved '
            f'${actual + reserved:.6f}, next-call ceiling ${maximum:.6f}, '
            f'limit ${limit:.2f}'
        )
    value.update(
        {
            'limit_usd': limit,
            'actual_cost_usd': actual,
            'reserved_cost_usd': reserved + maximum,
            'updated_unix_s': time.time(),
        }
    )
    _write_ledger(path, value)
    return path, maximum


def _commit_cost(reservation: tuple[Path, float] | None, cost: float) -> None:
    if reservation is None:
        return
    path, maximum = reservation
    value = json.loads(path.read_text(encoding='utf-8'))
    value['actual_cost_usd'] = float(value.get('actual_cost_usd', 0.0)) + cost
    value['reserved_cost_usd'] = max(
        0.0, float(value.get('reserved_cost_usd', 0.0)) - maximum
    )
    value['successful_calls'] = int(value.get('successful_calls', 0)) + 1
    value['updated_unix_s'] = time.time()
    _write_ledger(path, value)


def _release_cost(reservation: tuple[Path, float] | None) -> None:
    if reservation is None:
        return
    path, maximum = reservation
    value = json.loads(path.read_text(encoding='utf-8'))
    value['reserved_cost_usd'] = max(
        0.0, float(value.get('reserved_cost_usd', 0.0)) - maximum
    )
    value['updated_unix_s'] = time.time()
    _write_ledger(path, value)


def _parts(value: Any) -> list[dict[str, Any]]:
    if hasattr(value, 'to_messages'):
        value = value.to_messages()
    elif hasattr(value, 'to_string'):
        value = value.to_string()
    values = value if isinstance(value, list) else [value]
    parts: list[dict[str, Any]] = []
    for item in values:
        content = getattr(item, 'content', item)
        blocks = content if isinstance(content, list) else [content]
        for block in blocks:
            if isinstance(block, str):
                parts.append({'text': block})
                continue
            if not isinstance(block, dict):
                parts.append({'text': str(block)})
                continue
            if block.get('type') == 'text':
                parts.append({'text': str(block.get('text') or '')})
                continue
            image = block.get('image_url')
            url = image.get('url') if isinstance(image, dict) else image
            match = re.fullmatch(r'data:([^;]+);base64,(.+)', str(url or ''), re.S)
            if match:
                base64.b64decode(match.group(2), validate=True)
                parts.append(
                    {
                        'inlineData': {
                            'mimeType': match.group(1),
                            'data': match.group(2),
                        }
                    }
                )
    return parts


def direct_gemini_runnable(
    *,
    model: str,
    max_output_tokens: int,
    seed: int,
    thinking_level: str,
    temperature: float | None,
    timeout_s: float,
):
    api_key = os.environ.get('GEMINI_API_KEY', '')
    if not api_key:
        raise RuntimeError('GEMINI_API_KEY is not set')

    def invoke(value: Any) -> AIMessage:
        reservation = _reserve_cost(max_output_tokens)
        committed = False
        try:
            generation_config: dict[str, Any] = {
                'maxOutputTokens': max_output_tokens,
                'seed': seed,
                'thinkingConfig': {'thinkingLevel': thinking_level.upper()},
            }
            if temperature is not None:
                generation_config['temperature'] = temperature
            body = {
                'contents': [{'role': 'user', 'parts': _parts(value)}],
                'generationConfig': generation_config,
            }
            request = Request(
                'https://generativelanguage.googleapis.com/v1beta/models/'
                + quote(model, safe='')
                + ':generateContent',
                data=json.dumps(body).encode('utf-8'),
                headers={
                    'x-goog-api-key': api_key,
                    'Content-Type': 'application/json',
                },
                method='POST',
            )
            with urlopen(request, timeout=timeout_s) as response:
                raw = json.load(response)
            candidates = raw.get('candidates') or []
            if not candidates:
                raise ValueError(f'Gemini returned no candidate: {raw.get("promptFeedback", {})}')
            candidate = candidates[0]
            response_parts = (candidate.get('content') or {}).get('parts') or []
            content = ''.join(
                str(part.get('text') or '')
                for part in response_parts
                if not part.get('thought')
            )
            usage = raw.get('usageMetadata') or {}
            prompt_tokens = int(usage.get('promptTokenCount') or 0)
            cached_tokens = int(usage.get('cachedContentTokenCount') or 0)
            output_tokens = int(usage.get('candidatesTokenCount') or 0) + int(
                usage.get('thoughtsTokenCount') or 0
            )
            cost = (
                (prompt_tokens - cached_tokens) * INPUT_USD_PER_MILLION
                + cached_tokens * CACHED_INPUT_USD_PER_MILLION
                + output_tokens * OUTPUT_USD_PER_MILLION
            ) / 1_000_000
            _commit_cost(reservation, cost)
            committed = True
            returned_model = str(raw.get('modelVersion') or '')
            return AIMessage(
                content=content,
                response_metadata={
                    'model_name': returned_model,
                    'finish_reason': candidate.get('finishReason'),
                    'provider': 'Google Gemini API',
                    'cost_usd': cost,
                    'seed': seed,
                    'thinking_level': thinking_level.lower(),
                },
                usage_metadata={
                    'input_tokens': prompt_tokens,
                    'output_tokens': output_tokens,
                    'total_tokens': int(usage.get('totalTokenCount') or 0),
                    'input_token_details': {'cache_read': cached_tokens},
                },
            )
        finally:
            if not committed:
                _release_cost(reservation)

    return RunnableLambda(invoke)
