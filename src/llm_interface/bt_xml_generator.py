"""Utilities to convert structured JSON data into BehaviorTree.CPP XML."""

from __future__ import annotations

import json
from typing import Any, Dict, Iterable, Mapping
from xml.etree import ElementTree as ET


class BehaviorTreeGenerationError(RuntimeError):
  """Raised when the provided specification is invalid."""


CONTROL_NODES = {
  "Sequence",
  "Fallback",
  "ReactiveSequence",
  "ReactiveFallback",
  "Parallel",
  "SequenceStar",
}

DECORATOR_NODES = {
  "RetryUntilSuccessful",
  "Repeat",
  "Inverter",
  "ForceSuccess",
  "ForceFailure",
  "KeepRunningUntilFailure",
}


def parse_spec(raw_payload: str | Mapping[str, Any]) -> Dict[str, Any]:
  """Parse an LLM payload containing a behavior tree description."""
  if isinstance(raw_payload, Mapping):
    return dict(raw_payload)
  try:
    return json.loads(raw_payload)
  except json.JSONDecodeError as exc:
    raise BehaviorTreeGenerationError(f"LLM response is not valid JSON: {exc}") from exc


def generate_bt_xml(spec: Mapping[str, Any]) -> str:
  """Convert a dictionary specification into BT XML."""
  tree_id = spec.get("tree_id", "LLMGeneratedTree")
  nodes = spec.get("nodes")
  root_id = spec.get("root")

  if not isinstance(nodes, Iterable):
    raise BehaviorTreeGenerationError("Behavior tree specification must contain a 'nodes' list")
  node_map = {node["id"]: node for node in nodes if isinstance(node, Mapping) and "id" in node}
  if not node_map:
    raise BehaviorTreeGenerationError("No valid nodes found in behavior tree specification")
  if not root_id:
    raise BehaviorTreeGenerationError("Behavior tree specification missing 'root' node id")
  if root_id not in node_map:
    raise BehaviorTreeGenerationError(f"Root node '{root_id}' not found in specification")

  bt = ET.Element("BehaviorTree", attrib={"ID": tree_id})
  bt.append(_build_node(node_map, root_id))

  tree = ET.Element("root")
  tree.append(bt)
  xml_bytes = ET.tostring(tree, encoding="utf-8")
  return '<?xml version="1.0"?>\n' + xml_bytes.decode("utf-8")


def _build_node(node_map: Mapping[str, Mapping[str, Any]], node_id: str) -> ET.Element:
  if node_id not in node_map:
    raise BehaviorTreeGenerationError(f"Node '{node_id}' referenced but not defined")
  node = node_map[node_id]
  node_type = node.get("type")
  if not node_type:
    raise BehaviorTreeGenerationError(f"Node '{node_id}' is missing a 'type' field")

  name = node.get("name", node_id)

  if node_type in CONTROL_NODES:
    element = ET.Element(node_type, attrib={"name": name})
    for child_id in node.get("children", []):
      element.append(_build_node(node_map, child_id))
    return element

  if node_type in DECORATOR_NODES:
    element = ET.Element(node_type, attrib={"name": name})
    children = node.get("children", [])
    if len(children) != 1:
      raise BehaviorTreeGenerationError(f"Decorator node '{node_id}' must have exactly one child")
    element.append(_build_node(node_map, children[0]))
    return element

  plugin_id = node.get("plugin") or node_type
  element_type = node.get("category", "Action")
  element = ET.Element(element_type, attrib={"ID": plugin_id, "name": name})

  for kind in ("inputs", "outputs"):
    for key, value in node.get(kind, {}).items():
      child = ET.SubElement(element, kind[:-1], attrib={"key": key})
      child.text = _coerce_value(value)

  return element


def _coerce_value(value: Any) -> str:
  if isinstance(value, str):
    return value
  if isinstance(value, (int, float)):
    return str(value)
  return json.dumps(value, ensure_ascii=False)
