import pytest


pytest.importorskip('langchain_core')

from llm_interface.node import LLMInterfaceNode


def test_reasoning_block_does_not_shadow_openai_json_output():
    node = LLMInterfaceNode.__new__(LLMInterfaceNode)
    content = [
        {'type': 'reasoning', 'summary': [], 'content': []},
        {'type': 'text', 'text': '{"tree_id":"explore_area.xml"}'},
    ]

    assert node._prepare_llm_json_text(content) == '{"tree_id":"explore_area.xml"}'
