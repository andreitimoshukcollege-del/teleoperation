"""Reading files out of an ISO 9660 image (G.1050's electronic attachment is one), read-only and with the standard
library: the GPU machine has no ISO tool, and mounting needs root.

Joliet names (UCS-2) are used when the image has them, else the primary volume's (``;1`` version suffixes dropped).
Nothing is written by name: :func:`walk` yields paths for the caller to match, and :func:`read` returns bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import BinaryIO, Iterator, List, Optional, Tuple

SECTOR = 2048
MAX_DEPTH = 16


@dataclass(frozen=True)
class Entry:
    path: str  # "/" separated, no leading slash
    extent: int
    size: int
    is_dir: bool


def _descriptors(f: BinaryIO) -> Tuple[Optional[bytes], Optional[bytes]]:
    primary = joliet = None
    for lba in range(16, 64):
        f.seek(lba * SECTOR)
        d = f.read(SECTOR)
        if len(d) < SECTOR or d[1:6] != b"CD001":
            break
        kind = d[0]
        if kind == 1 and primary is None:
            primary = d
        elif kind == 2 and d[88:91] in (b"%/@", b"%/C", b"%/E"):
            joliet = d
        elif kind == 255:
            break
    return primary, joliet


def _records(f: BinaryIO, extent: int, size: int) -> Iterator[Tuple[int, int, int, bytes]]:
    f.seek(extent * SECTOR)
    data = f.read(size)
    at = 0
    while at < len(data):
        n = data[at]
        if n == 0:  # records do not cross sectors: skip to the next one
            at = (at // SECTOR + 1) * SECTOR
            continue
        rec = data[at:at + n]
        if len(rec) < 34:
            break
        ext, = struct.unpack_from("<I", rec, 2)
        length, = struct.unpack_from("<I", rec, 10)
        flags = rec[25]
        name = rec[33:33 + rec[32]]
        yield ext, length, flags, name
        at += n


def walk(f: BinaryIO) -> List[Entry]:
    primary, joliet = _descriptors(f)
    vd = joliet or primary
    if vd is None:
        raise ValueError("not an ISO 9660 image")
    root = vd[156:156 + 34]
    ext, = struct.unpack_from("<I", root, 2)
    size, = struct.unpack_from("<I", root, 10)
    out: List[Entry] = []
    seen = set()

    def visit(extent: int, length: int, prefix: str, depth: int) -> None:
        if depth > MAX_DEPTH or extent in seen:
            return
        seen.add(extent)
        for e, n, flags, raw in _records(f, extent, length):
            if raw in (b"\x00", b"\x01"):
                continue  # . and ..
            name = raw.decode("utf-16-be", "replace") if joliet else raw.decode("ascii", "replace")
            name = name.split(";")[0].rstrip(".")
            if not name or "/" in name or name in (".", ".."):
                continue
            path = f"{prefix}{name}"
            is_dir = bool(flags & 2)
            out.append(Entry(path, e, n, is_dir))
            if is_dir:
                visit(e, n, path + "/", depth + 1)

    visit(ext, size, "", 0)
    return out


def read(f: BinaryIO, entry: Entry, max_bytes: int) -> bytes:
    if entry.is_dir:
        raise ValueError(f"{entry.path} is a directory")
    if entry.size > max_bytes:
        raise ValueError(f"{entry.path}: {entry.size} bytes is more than {max_bytes}")
    f.seek(entry.extent * SECTOR)
    return f.read(entry.size)
