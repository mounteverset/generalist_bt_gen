"""Helpers for loading and rendering prompt templates for the LLM service."""

from __future__ import annotations

import dataclasses
import textwrap
from pathlib import Path
from string import Formatter
from typing import Any, Dict, Iterable, Mapping, Optional

import yaml


class PromptTemplateError(RuntimeError):
  """Base exception for prompt template issues."""


class TemplateNotFoundError(PromptTemplateError):
  """Raised when the requested template does not exist."""


class TemplateRenderError(PromptTemplateError):
  """Raised when a template cannot be rendered with the provided context."""


@dataclasses.dataclass(frozen=True)
class PromptParts:
  """Rendered prompt fragments ready for submission to the LLM."""

  system: str
  user: str
  metadata: Dict[str, Any]

  def to_messages(self) -> Iterable[Dict[str, str]]:
    """Convert prompt parts into the message structure expected by chat APIs."""
    messages = []
    if self.system.strip():
      messages.append({"role": "system", "content": self.system})
    messages.append({"role": "user", "content": self.user})
    return messages


@dataclasses.dataclass(frozen=True)
class PromptTemplate:
  """Represents a single prompt template consisting of system and user sections."""

  name: str
  system: str
  user: str
  metadata: Dict[str, Any]

  def render(self, **variables: Any) -> PromptParts:
    """Render the template using the provided variables."""
    try:
      system = _safe_format(self.system, **variables)
      user = _safe_format(self.user, **variables)
    except KeyError as exc:  # pragma: no cover - should be wrapped by TemplateFormatter
      raise TemplateRenderError(f"Missing variable '{exc.args[0]}' for template '{self.name}'") from exc

    metadata = {**self.metadata}
    metadata.setdefault("variables", variables)
    return PromptParts(system=system, user=user, metadata=metadata)


class PromptRepository:
  """Loads and serves prompt templates defined in a YAML configuration file."""

  def __init__(self, templates: Mapping[str, PromptTemplate], defaults: Mapping[str, Any]):
    self._templates = dict(templates)
    self._defaults = dict(defaults)

  @classmethod
  def from_path(cls, path: str | Path) -> "PromptRepository":
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    raw_templates = data.get("prompts", {})
    templates = {}
    for name, definition in raw_templates.items():
      system = textwrap.dedent(definition.get("system", "")).strip()
      user = textwrap.dedent(definition.get("user", "")).strip()
      metadata = definition.get("metadata", {})
      templates[name] = PromptTemplate(
        name=name,
        system=system,
        user=user,
        metadata=dict(metadata),
      )
    defaults = data.get("defaults", {})
    return cls(templates=templates, defaults=defaults)

  def available_templates(self) -> Iterable[str]:
    return self._templates.keys()

  def get_template(self, name: str) -> PromptTemplate:
    try:
      return self._templates[name]
    except KeyError as exc:
      raise TemplateNotFoundError(f"Template '{name}' not found. Available: {sorted(self._templates)}") from exc

  def render(self, name: str, **variables: Any) -> PromptParts:
    template = self.get_template(name)
    merged_variables = {**self._defaults.get("variables", {}), **variables}
    return template.render(**merged_variables)

  @property
  def default_model(self) -> str:
    return str(self._defaults.get("model", "gpt-4o-mini"))

  @property
  def default_temperature(self) -> float:
    return float(self._defaults.get("temperature", 0.2))

  @property
  def default_max_tokens(self) -> int:
    return int(self._defaults.get("max_tokens", 1024))

  @property
  def default_response_format(self) -> Optional[str]:
    value = self._defaults.get("response_format")
    return str(value) if value else None


def _safe_format(template: str, **variables: Any) -> str:
  formatter = _MissingAwareFormatter()
  try:
    return formatter.vformat(template, args=(), kwargs=variables)
  except KeyError as exc:
    raise TemplateRenderError(f"Missing variable '{exc.args[0]}' for template substitution") from exc


class _MissingAwareFormatter(Formatter):
  """String formatter that reports missing keys as TemplateRenderError."""

  def get_value(self, key: Any, args: Any, kwargs: Mapping[str, Any]) -> Any:  # pragma: no cover - wrapper ensures conversion
    if isinstance(key, str):
      if key not in kwargs:
        raise KeyError(key)
      return kwargs[key]
    return super().get_value(key, args, kwargs)
