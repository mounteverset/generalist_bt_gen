from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Awaitable, Callable, Dict


_SESSION_ID_RE = re.compile(r'^[a-zA-Z0-9_.:-]+$')


@dataclass
class ParsedCommand:
    name: str
    payload: str


def parse_command(text: str) -> ParsedCommand:
    stripped = text.strip()
    if not stripped.startswith('/'):
        raise ValueError('Command must start with /')
    command, *rest = stripped[1:].split(' ', 1)
    payload = rest[0] if rest else ''
    return ParsedCommand(name=command.lower(), payload=payload)


def parse_session_and_note(payload: str) -> tuple[str, str]:
    content = (payload or '').strip()
    if not content:
        return '', ''
    first, *rest = content.split(' ', 1)
    if _looks_like_session_id(first):
        return first.strip(), (rest[0].strip() if rest else '')
    return '', content


def _looks_like_session_id(token: str) -> bool:
    value = (token or '').strip()
    if not value or not _SESSION_ID_RE.match(value):
        return False
    if value in {'unknown', 'pending'}:
        return True
    if value.startswith(('chatui-', 'webui-')):
        return True
    return '-' in value and any(ch.isdigit() for ch in value)


class CommandRouter:
    """Shared slash-command dispatcher."""

    def __init__(self, handlers: Dict[str, Callable[[str], Awaitable[None]]]) -> None:
        self._handlers = handlers

    async def dispatch(self, text: str) -> bool:
        parsed = parse_command(text)
        handler = self._handlers.get(parsed.name)
        if handler is None:
            return False
        await handler(parsed.payload)
        return True
