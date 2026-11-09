"""Delay traces with loss (docs/adr/0035 amendment 3): ``.trace`` version 2.

Version 1 (docs/adr/0004, ``Teleop.Eval/Sweep/TraceFile.cs``) is ``TRACE|1|<tps>`` and then one delay in ticks per
line, one line per datagram, consumed in order. It cannot say a datagram was lost, and it does not say how often the
datagrams were sent.

Version 2 adds both:

    TRACE|2|<ticksPerSecond>|<sendIntervalTicks>
    <delay ticks>   a datagram that arrived, this long after it was sent
    L               a datagram that was lost

Datagram i is sent at ``i * sendIntervalTicks``. Only the trainer reads version 2; Core, ``Teleop.Eval``, the Bridge and
``analysis/`` read version 1 and refuse a version-2 file by its header. :func:`read` reads both (a version-1 file has
no loss and no interval: ``send_interval_ticks`` is 0).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

LOST = -1


@dataclass
class Trace:
    ticks_per_second: int
    send_interval_ticks: int  # 0: unknown (a version-1 file)
    delays: np.ndarray  # int64 ticks; LOST where the datagram was lost

    @property
    def lost(self) -> np.ndarray:
        return self.delays == LOST

    @property
    def interval_ms(self) -> float:
        return self.send_interval_ticks * 1000.0 / self.ticks_per_second

    @property
    def seconds(self) -> float:
        return len(self.delays) * self.send_interval_ticks / self.ticks_per_second

    def delays_ms(self) -> np.ndarray:
        """Delays in ms, NaN where lost."""
        out = self.delays.astype(np.float64) * 1000.0 / self.ticks_per_second
        out[self.lost] = np.nan
        return out


def from_ms(delays_ms: Sequence[Optional[float]], interval_ms: float, tps: int = 10_000_000) -> Trace:
    """A trace from delays in ms (None or NaN: lost), sent every ``interval_ms``."""
    d = np.empty(len(delays_ms), dtype=np.int64)
    for i, v in enumerate(delays_ms):
        if v is None or v != v:
            d[i] = LOST
        else:
            if v < 0:
                raise ValueError(f"datagram {i}: a negative delay ({v} ms)")
            d[i] = int(round(v * tps / 1000.0))
    return Trace(tps, int(round(interval_ms * tps / 1000.0)), d)


def write(path: str, trace: Trace) -> None:
    if trace.send_interval_ticks <= 0:
        raise ValueError("a version-2 trace needs its send interval")
    if np.any((trace.delays < 0) & (trace.delays != LOST)):
        raise ValueError("delays must not be negative")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="ascii", newline="\n") as f:
        f.write(f"TRACE|2|{trace.ticks_per_second}|{trace.send_interval_ticks}\n")
        f.write("\n".join("L" if v == LOST else str(int(v)) for v in trace.delays))
        f.write("\n")
    os.replace(tmp, path)


def read(path: str) -> Trace:
    with open(path, "r", encoding="ascii") as f:
        lines = f.read().splitlines()
    if not lines:
        raise ValueError(f"{path}: empty")
    head = lines[0].split("|")
    if head[0] != "TRACE" or len(head) not in (3, 4):
        raise ValueError(f"{path}: not a trace header: {lines[0]!r}")
    if head[1] == "1" and len(head) == 3:
        interval = 0
    elif head[1] == "2" and len(head) == 4:
        interval = int(head[3])
    else:
        raise ValueError(f"{path}: unsupported trace version {head[1]!r}")
    tps = int(head[2])
    body: List[int] = []
    for n, line in enumerate(lines[1:], start=2):
        if not line:
            continue
        if line == "L":
            if interval == 0:
                raise ValueError(f"{path}:{n}: a loss marker in a version-1 trace")
            body.append(LOST)
            continue
        v = int(line)
        if v < 0:
            raise ValueError(f"{path}:{n}: a negative delay")
        body.append(v)
    return Trace(tps, interval, np.asarray(body, dtype=np.int64))
