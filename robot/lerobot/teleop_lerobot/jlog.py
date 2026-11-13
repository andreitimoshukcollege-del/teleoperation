"""A strict reader of the session's ``.jlog`` files (trainer/contract/SESSION_FORMAT.md, "Lines").

The trainer has its own reader, but nothing outside ``trainer/`` may import from it (trainer/CLAUDE.md, the isolation
rule): the format spec is the shared surface, so this follows the spec. Like the trainer's, it has no best-effort mode.
A file that is cut off, edited, or names the wrong writer is refused, because a take that fails here must never become
an episode.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional


class JlogError(ValueError):
    """A file that is not a valid, complete ``.jlog``."""


@dataclass
class Jlog:
    path: str
    writer: str
    ticks_per_second: int
    clock: str
    session_id: str
    records: Dict[str, List[List[str]]] = field(default_factory=dict)
    dropped: int = 0

    def tag(self, name: str) -> List[List[str]]:
        return self.records.get(name, [])


def read_jlog(path: str, writer: Optional[str] = None) -> Jlog:
    with open(path, "rb") as f:
        data = f.read()
    if not data.endswith(b"\n"):
        raise JlogError(f"{path}: does not end with a newline (cut off?)")
    last = data.rfind(b"\n", 0, len(data) - 1) + 1
    body, trailer = data[:last], data[last:-1].decode("ascii").split("|")
    if len(trailer) != 4 or trailer[0] != "END":
        raise JlogError(f"{path}: no END trailer (the writer did not finish)")
    if hashlib.sha256(body).hexdigest() != trailer[3]:
        raise JlogError(f"{path}: the trailer's sha256 does not match")
    lines = body.decode("ascii").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    if not lines:
        raise JlogError(f"{path}: no header")
    head = lines[0].split("|")
    if len(head) != 6 or head[0] != "JSES" or head[1] != "1":
        raise JlogError(f"{path}: line 1 is not a version-1 JSES header: {lines[0]!r}")
    if writer is not None and head[2] != writer:
        raise JlogError(f"{path}: writer is {head[2]!r}, expected {writer!r}")
    if len(lines) - 1 != int(trailer[1]):
        raise JlogError(f"{path}: the trailer says {trailer[1]} records, the file has {len(lines) - 1}")
    log = Jlog(path, head[2], int(head[3]), head[4], head[5], dropped=int(trailer[2]))
    for line in lines[1:]:
        parts = line.split("|")
        log.records.setdefault(parts[0], []).append(parts[1:])
    return log


def items(text: str) -> List[List[str]]:
    """A list field, ``a:b;c:d``, as its items' parts. An empty field is an empty list."""
    return [] if text == "" else [item.split(":") for item in text.split(";")]
