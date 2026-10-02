from __future__ import annotations

import json

import pytest

from llm_interface import gemini_direct


class FakeResponse:
    def __init__(self, value):
        self.value = value

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return json.dumps(self.value).encode()


def test_direct_gemini_records_high_reasoning_seed_and_cost(monkeypatch, tmp_path):
    captured = {}

    def fake_open(request, timeout):
        captured['body'] = json.loads(request.data)
        captured['timeout'] = timeout
        return FakeResponse(
            {
                'modelVersion': 'gemini-3.8-flash-20260901',
                'candidates': [
                    {
                        'finishReason': 'STOP',
                        'content': {'parts': [{'text': '{"ok": true}'}]},
                    }
                ],
                'usageMetadata': {
                    'promptTokenCount': 100,
                    'candidatesTokenCount': 20,
                    'thoughtsTokenCount': 10,
                    'totalTokenCount': 130,
                },
            }
        )

    ledger = tmp_path / 'cost.json'
    monkeypatch.setenv('GEMINI_API_KEY', 'test-key')
    monkeypatch.setenv('GENERALIST_BT_COST_LEDGER', str(ledger))
    monkeypatch.setenv('GENERALIST_BT_COST_LIMIT_USD', '15')
    monkeypatch.setattr(gemini_direct, 'urlopen', fake_open)
    model = gemini_direct.direct_gemini_runnable(
        model='gemini-3.8-flash',
        max_output_tokens=8192,
        seed=42,
        thinking_level='high',
        temperature=0.0,
        timeout_s=300,
    )

    result = model.invoke('prompt')

    assert result.content == '{"ok": true}'
    assert result.response_metadata['provider'] == 'Google Gemini API'
    assert result.response_metadata['cost_usd'] == pytest.approx(0.0001875)
    assert captured['body']['generationConfig']['thinkingConfig'] == {
        'thinkingLevel': 'HIGH'
    }
    assert captured['body']['generationConfig']['seed'] == 42
    assert json.loads(ledger.read_text())['actual_cost_usd'] == pytest.approx(
        0.0001875
    )


def test_direct_gemini_blocks_call_before_cost_cap(monkeypatch, tmp_path):
    ledger = tmp_path / 'cost.json'
    ledger.write_text(
        json.dumps({'actual_cost_usd': 14.5, 'reserved_cost_usd': 0.0})
    )
    monkeypatch.setenv('GEMINI_API_KEY', 'test-key')
    monkeypatch.setenv('GENERALIST_BT_COST_LEDGER', str(ledger))
    monkeypatch.setenv('GENERALIST_BT_COST_LIMIT_USD', '15')
    model = gemini_direct.direct_gemini_runnable(
        model='gemini-3.8-flash',
        max_output_tokens=8192,
        seed=42,
        thinking_level='high',
        temperature=0.0,
        timeout_s=300,
    )

    with pytest.raises(RuntimeError, match='cost cap would be exceeded'):
        model.invoke('prompt')


def test_direct_gemini_releases_cost_reservation_after_failure(monkeypatch, tmp_path):
    ledger = tmp_path / 'cost.json'
    monkeypatch.setenv('GEMINI_API_KEY', 'test-key')
    monkeypatch.setenv('GENERALIST_BT_COST_LEDGER', str(ledger))
    monkeypatch.setenv('GENERALIST_BT_COST_LIMIT_USD', '15')
    monkeypatch.setattr(
        gemini_direct,
        'urlopen',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(TimeoutError()),
    )
    model = gemini_direct.direct_gemini_runnable(
        model='gemini-3.8-flash',
        max_output_tokens=8192,
        seed=42,
        thinking_level='high',
        temperature=0.0,
        timeout_s=300,
    )

    with pytest.raises(TimeoutError):
        model.invoke('prompt')

    assert json.loads(ledger.read_text())['reserved_cost_usd'] == 0.0
