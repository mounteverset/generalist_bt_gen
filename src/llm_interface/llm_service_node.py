"""ROS 2 service node that queries an LLM to produce Behavior Tree updates."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from llm_interface.bt_xml_generator import (
  BehaviorTreeGenerationError,
  generate_bt_xml,
  parse_spec,
)
from llm_interface.prompt_templates import (
  PromptRepository,
  PromptTemplateError,
)
from llm_interface.srv import LLMQuery


@dataclass
class LLMClientResult:
  payload_text: str
  reasoning: Optional[str] = None


class OpenAIChatClient:
  """Wrapper around the OpenAI SDK to simplify testing and error handling."""

  def __init__(self, *, timeout: float = 30.0):
    try:
      from openai import OpenAI
    except ImportError as exc:  # pragma: no cover - covered by failure path in node
      raise RuntimeError(
        "openai package is not installed. Install it or provide a custom client."
      ) from exc

    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
      raise RuntimeError("Environment variable OPENAI_API_KEY must be set for OpenAI client")
    self._client = OpenAI(api_key=api_key, timeout=timeout)

  def complete(
    self,
    messages: Any,
    *,
    model: str,
    temperature: float,
    max_tokens: int,
    response_format: Optional[str] = None,
  ) -> LLMClientResult:
    response_kwargs: Dict[str, Any] = {
      "model": model,
      "temperature": temperature,
      "max_output_tokens": max_tokens,
      "input": list(messages),
    }
    if response_format:
      response_kwargs["response_format"] = {"type": response_format}

    response = self._client.responses.create(**response_kwargs)
    text = _extract_response_text(response)
    reasoning = _extract_reasoning(response)
    return LLMClientResult(payload_text=text, reasoning=reasoning)


def _extract_response_text(response: Any) -> str:
  """Handle both Responses API and legacy chat completions."""
  if hasattr(response, "output") and response.output:
    # New Responses API
    for item in response.output:
      content = getattr(item, "content", None)
      if content:
        for block in content:
          text = getattr(block, "text", None)
          if text:
            return text
  if hasattr(response, "choices") and response.choices:
    choice = response.choices[0]
    if hasattr(choice, "message"):
      return getattr(choice.message, "content", "") or ""
    return getattr(choice, "text", "") or ""
  raise RuntimeError("Unable to extract content from LLM response")


def _extract_reasoning(response: Any) -> Optional[str]:
  metadata = getattr(response, "output_metadata", None)
  if metadata and hasattr(metadata, "usage"):
    return getattr(metadata, "usage", None)
  return None


class LLMServiceNode(Node):
  """Service node that coordinates prompt rendering, LLM invocation, and XML conversion."""

  def __init__(
    self,
    *,
    client_factory: Optional[Callable[[Node], Any]] = None,
    node_options: Optional[rclpy.node.NodeOptions] = None,
  ) -> None:
    super().__init__("llm_service_node", options=node_options)

    self._declare_parameters()
    config_path = self.get_parameter("prompt_config_path").get_parameter_value().string_value
    self._repository = self._load_prompts(config_path)

    # Update defaults from config if not explicitly overridden
    self._set_parameter_if_not_set("model", self._repository.default_model)
    self._set_parameter_if_not_set("temperature", self._repository.default_temperature)
    self._set_parameter_if_not_set("max_tokens", self._repository.default_max_tokens)
    self._set_parameter_if_not_set("response_format", self._repository.default_response_format or "")

    factory = client_factory or (lambda _: OpenAIChatClient(timeout=self.get_parameter("timeout_sec").value))
    try:
      self._client = factory(self)
    except Exception as exc:  # pragma: no cover - tested through failure path
      self.get_logger().error(f"Failed to initialize LLM client: {exc}")
      self._client = None

    service_name = self.get_parameter("service_name").get_parameter_value().string_value
    self._service = self.create_service(LLMQuery, service_name, self._handle_request)
    self.get_logger().info(f"LLM service ready on '{service_name}'")

  def _declare_parameters(self) -> None:
    default_config_path = self._default_config_path()
    self.declare_parameter("prompt_config_path", default_config_path)
    self.declare_parameter("template_name", "llm_query")
    self.declare_parameter("model", "")
    self.declare_parameter("temperature", 0.2)
    self.declare_parameter("max_tokens", 2048)
    self.declare_parameter("response_format", "json_object")
    self.declare_parameter("timeout_sec", 45.0)
    self.declare_parameter("service_name", "llm_query")

  def _load_prompts(self, path: str) -> PromptRepository:
    try:
      return PromptRepository.from_path(path)
    except FileNotFoundError as exc:
      raise RuntimeError(f"Prompt configuration file not found: {path}") from exc
    except PromptTemplateError as exc:
      raise RuntimeError(f"Failed to load prompt templates: {exc}") from exc

  def _set_parameter_if_not_set(self, name: str, value: Any) -> None:
    param = self.get_parameter(name)
    if param.type_ == Parameter.Type.NOT_SET or param.value in ("", None):
      self.set_parameters([Parameter(name=name, value=value)])

  def _handle_request(self, request: LLMQuery.Request, response: LLMQuery.Response) -> LLMQuery.Response:
    result = self._process_request(request)
    response.success = result["success"]
    response.response_json = result["response_json"]
    response.reasoning = result["reasoning"]
    return response

  def _process_request(self, request: LLMQuery.Request) -> Dict[str, Any]:
    if self._client is None:
      return _failure("LLM client is not configured")

    template_name = self.get_parameter("template_name").get_parameter_value().string_value
    try:
      prompt = self._repository.render(
        template_name,
        prompt=request.prompt,
        context=request.context_json or "{}",
        metadata=request.metadata_json or "{}",
      )
    except PromptTemplateError as exc:
      self.get_logger().error(f"Prompt rendering failed: {exc}")
      return _failure(f"Prompt rendering failed: {exc}")

    try:
      result = self._client.complete(
        prompt.to_messages(),
        model=self.get_parameter("model").value,
        temperature=float(self.get_parameter("temperature").value),
        max_tokens=int(self.get_parameter("max_tokens").value),
        response_format=self.get_parameter("response_format").value or None,
      )
    except Exception as exc:
      self.get_logger().error(f"LLM request failed: {exc}")
      return _failure(f"LLM request failed: {exc}")

    try:
      payload = parse_spec(result.payload_text)
    except BehaviorTreeGenerationError as exc:
      self.get_logger().error(str(exc))
      return _failure(str(exc))

    try:
      bt_spec = payload.get("behavior_tree") or payload
      bt_xml = generate_bt_xml(bt_spec)
      payload.setdefault("artifacts", {})["behavior_tree_xml"] = bt_xml
    except BehaviorTreeGenerationError as exc:
      self.get_logger().error(f"Failed to generate BT XML: {exc}")
      return _failure(f"Failed to generate BT XML: {exc}")

    reasoning = payload.get("reasoning") or result.reasoning or "LLM provided response"
    response_json = json.dumps(payload, ensure_ascii=False)
    return {"success": True, "response_json": response_json, "reasoning": reasoning}

  @staticmethod
  def _default_config_path() -> str:
    try:
      share_dir = get_package_share_directory("llm_interface")
      return os.path.join(share_dir, "config", "prompt_config.yaml")
    except Exception:
      # Allow tests to inject their own path
      return ""


def _failure(reason: str) -> Dict[str, Any]:
  return {"success": False, "response_json": "{}", "reasoning": reason}


def main(args: Optional[list[str]] = None) -> None:  # pragma: no cover - exercised via entry point
  rclpy.init(args=args)
  node: Optional[LLMServiceNode] = None
  try:
    node = LLMServiceNode()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if node is not None:
      node.destroy_node()
    rclpy.shutdown()
