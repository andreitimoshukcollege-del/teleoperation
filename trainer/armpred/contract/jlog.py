"""The ``.jlog`` line format shared by every session file (contract/SESSION_FORMAT.md, "Lines").

One record per line, ``|``-separated, a ``JSES`` header first and an ``END`` trailer last. The
trailer carries the record count, the writer's drop count and a SHA-256 over every byte before it, so
a cut-off or edited file is caught rather than half-read. SHA-256 rather than a hand-rolled hash
because the robot's driver writes in Python on a Jetson Nano: ``hashlib`` hashes in C, outside the
GIL, so the log cannot steal time from the driver thread whose timing it records.
"""

from __future__ import annotations

import hashlib
import io
import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

FORMAT_TAG = "JSES"
TRAILER_TAG = "END"
SUPPORTED_VERSIONS = (1,)

class JlogError(ValueError):
    """A file that is not a valid, complete ``.jlog``."""


@dataclass
class JlogHeader:
    version: int
    writer: str
    ticks_per_second: int
    clock: str
    session_id: str


@dataclass
class Jlog:
    """A parsed file: its header, its records grouped by tag (each a list of field lists, tag
    removed), and what the trailer said."""

    path: str
    header: JlogHeader
    records: Dict[str, List[List[str]]] = field(default_factory=dict)
    record_count: int = 0
    dropped: int = 0

    def tag(self, name: str) -> List[List[str]]:
        return self.records.get(name, [])


def read_jlog(path: str, expected_writer: Optional[str] = None) -> Jlog:
    """Reads and validates one ``.jlog``. Raises ``JlogError`` on a bad header, a missing or
    inconsistent trailer, or a checksum mismatch: a session that fails here must never be trained
    on, so there is deliberately no "best effort" mode."""
    with open(path, "rb") as f:
        data = f.read()
    if not data.endswith(b"\n"):
        raise JlogError(f"{path}: does not end with a newline (cut off?)")
    last_start = data.rfind(b"\n", 0, len(data) - 1) + 1
    body, trailer_line = data[:last_start], data[last_start:-1]
    trailer = trailer_line.decode("ascii").split("|")
    if len(trailer) != 4 or trailer[0] != TRAILER_TAG:
        raise JlogError(f"{path}: no END trailer (the writer did not finish)")
    try:
        record_count, dropped, checksum = int(trailer[1]), int(trailer[2]), trailer[3]
    except ValueError as e:
        raise JlogError(f"{path}: malformed trailer {trailer_line!r}") from e
    actual = hashlib.sha256(body).hexdigest()
    if actual != checksum:
        raise JlogError(f"{path}: checksum {actual} != trailer {checksum}")

    lines = body.decode("ascii").split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    if not lines:
        raise JlogError(f"{path}: no header")
    header_fields = lines[0].split("|")
    if len(header_fields) != 6 or header_fields[0] != FORMAT_TAG:
        raise JlogError(f"{path}: line 1 is not a JSES header: {lines[0]!r}")
    version = int(header_fields[1])
    if version not in SUPPORTED_VERSIONS:
        raise JlogError(f"{path}: unsupported session format version {version}")
    header = JlogHeader(
        version=version,
        writer=header_fields[2],
        ticks_per_second=int(header_fields[3]),
        clock=header_fields[4],
        session_id=header_fields[5],
    )
    if expected_writer is not None and header.writer != expected_writer:
        raise JlogError(f"{path}: writer is {header.writer!r}, expected {expected_writer!r}")
    if len(lines) - 1 != record_count:
        raise JlogError(f"{path}: trailer says {record_count} records, file has {len(lines) - 1}")

    records: Dict[str, List[List[str]]] = {}
    for line in lines[1:]:
        parts = line.split("|")
        records.setdefault(parts[0], []).append(parts[1:])
    return Jlog(path=path, header=header, records=records, record_count=record_count, dropped=dropped)


class JlogWriter:
    """Writes a ``.jlog`` with its header, keeping the checksum as it goes. ``close()`` writes the
    trailer; a writer that is never closed leaves a file the reader rejects, which is the point."""

    def __init__(self, path: str, writer: str, ticks_per_second: int, clock: str, session_id: str):
        for value in (writer, clock, session_id):
            if "|" in value or "\n" in value:
                raise ValueError(f"header field {value!r} contains a separator")
        self._file = io.open(path, "wb")
        self._hash = hashlib.sha256()
        self._records = 0
        self.dropped = 0
        self._emit(f"{FORMAT_TAG}|1|{writer}|{ticks_per_second}|{clock}|{session_id}")

    def _emit(self, line: str) -> None:
        data = (line + "\n").encode("ascii")
        self._hash.update(data)
        self._file.write(data)

    def record(self, tag: str, *fields: object) -> None:
        self._emit("|".join([tag, *(format_field(f) for f in fields)]))
        self._records += 1

    def close(self) -> None:
        if self._file.closed:
            return
        self._file.write(f"{TRAILER_TAG}|{self._records}|{self.dropped}|{self._hash.hexdigest()}\n".encode("ascii"))
        self._file.close()

    def __enter__(self) -> "JlogWriter":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def format_field(value: object) -> str:
    """One field as the format writes it: round-trip floats, decimal integers, empty for None."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return repr(value)
    text = str(value)
    if "|" in text or "\n" in text:
        raise ValueError(f"field {text!r} contains a separator")
    return text


def join_items(items: Iterable[Iterable[object]]) -> str:
    """A list field: ``a:b;c:d``."""
    return ";".join(":".join(format_field(p) for p in item) for item in items)


def split_items(text: str) -> List[List[str]]:
    """The inverse of ``join_items``; an empty field is an empty list."""
    if text == "":
        return []
    return [item.split(":") for item in text.split(";")]


def session_files(directory: str) -> List[str]:
    return sorted(name for name in os.listdir(directory) if name.endswith(".jlog"))
