import json
import textwrap

import pytest
import rclpy
from rclpy.node import NodeOptions
from rclpy.parameter import Parameter

from llm_interface.llm_service_node import LLMClientResult, LLMServiceNode
from llm_interface.srv import LLMQuery


class DummyClient:
    def __init__(self, payload: dict[str, object], reasoning: str = ""):
        self._payload = payload
        self._reasoning = reasoning
        self.calls = []

    def complete(self, messages, **kwargs):
        self.calls.append({"messages": list(messages), **kwargs})
        return LLMClientResult(json.dumps(self._payload), reasoning=self._reasoning)


@pytest.fixture(name="rcl_context")
def rcl_context_fixture():
    if not rclpy.ok():
        rclpy.init()
    yield
    rclpy.shutdown()


def create_config(tmp_path):
    content = textwrap.dedent(
        """
        defaults:
          model: gpt-test
          temperature: 0.3
          max_tokens: 512
        prompts:
          llm_query:
            system: |
              respond carefully
            user: |
              Task: {prompt}
              Context: {context}
              Metadata: {metadata}
        """
    )
    path = tmp_path / "prompt.yaml"
    path.write_text(content, encoding="utf-8")
    return str(path)


def test_service_success_path(tmp_path, rcl_context):
    config_path = create_config(tmp_path)
    payload = {
        "behavior_tree": {
            "tree_id": "Generated",
            "root": "root_sequence",
            "nodes": [
                {"id": "root_sequence", "type": "Sequence", "children": ["drive"]},
                {"id": "drive", "type": "Action", "plugin": "NavigateGPS"},
            ],
        },
        "reasoning": "Because the robot must navigate",
    }
    client = DummyClient(payload)

    options = NodeOptions(parameter_overrides=[
        Parameter("prompt_config_path", value=config_path),
    ])
    node = LLMServiceNode(client_factory=lambda _: client, node_options=options)

    request = LLMQuery.Request()
    request.prompt = "Navigate to the barn"
    request.context_json = "{}"
    request.metadata_json = "{}"

    result = node._process_request(request)

    assert result["success"] is True
    payload_out = json.loads(result["response_json"])
    assert "behavior_tree_xml" in payload_out["artifacts"]
    assert "Navigate to the barn" in client.calls[0]["messages"][1]["content"]

    node.destroy_node()


def test_service_failure_when_client_missing(tmp_path, rcl_context):
    config_path = create_config(tmp_path)
    options = NodeOptions(parameter_overrides=[
        Parameter("prompt_config_path", value=config_path),
    ])

    def bad_factory(_):
        raise RuntimeError("boom")

    node = LLMServiceNode(client_factory=bad_factory, node_options=options)

    request = LLMQuery.Request()
    result = node._process_request(request)
    assert result["success"] is False
    assert "client" in result["reasoning"]

    node.destroy_node()
