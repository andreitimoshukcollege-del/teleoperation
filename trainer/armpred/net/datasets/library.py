"""The network datasets' traces as the trainer replays them (docs/adr/0035 N5): by time, with their losses.

A schedule segment that names a trace (``Segment.trace``, ``<dataset>/<id>`` under ``Segment.trace_root``) sends each
datagram through the trace's sample at its time in the segment, ``trace_offset_s`` into the trace: delayed by it, or
lost where it is ``L``. This is what Core's ``TimedTraceImpairment`` does on RobotHost's uplink (N4), so a session the
arm lived through on a trace and a view synthesised on it see the same network. Traces are read once per process and
kept (they are a few MB of int64 at most).

``window(...)`` gives the trace's own quantiles, loss and floor around a moment: the truth a perfect forecaster of the
link would know (``forecast.LinkTruth``), as the realized delays give it for any other recorded network.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from . import trace as T


@dataclass(frozen=True)
class Loaded:
    delays_ms: np.ndarray  # float64, NaN where lost
    interval_s: float

    @property
    def seconds(self) -> float:
        return len(self.delays_ms) * self.interval_s


_CACHE: Dict[str, Loaded] = {}


def path_of(root: str, ref: str) -> str:
    dataset, _, tid = ref.partition("/")
    if not dataset or not tid or "/" in tid or dataset.startswith(".") or tid.startswith("."):
        raise ValueError(f"not a <dataset>/<id> trace reference: {ref!r}")
    return os.path.join(root, dataset, "traces", tid + ".trace")


def load(root: str, ref: str) -> Loaded:
    p = path_of(root, ref)
    hit = _CACHE.get(p)
    if hit is None:
        tr = T.read(p)
        if tr.send_interval_ticks <= 0:
            raise ValueError(f"{p}: a version-2 trace is needed (its send interval and losses)")
        hit = Loaded(tr.delays_ms(), tr.interval_ms / 1000.0)
        _CACHE[p] = hit
    return hit


def index_at(t: Loaded, offset_s: float, elapsed_s: float) -> int:
    """The sample a datagram sent ``elapsed_s`` into the segment uses (wrapping at the trace's end)."""
    # In whole nanoseconds, as Core's TimedTraceImpairment works in ticks: 10.02 s - 10.0 s is 0.0199999... in floats.
    at_ns = int(round((offset_s + max(0.0, elapsed_s)) * 1e9))
    return (at_ns // int(round(t.interval_s * 1e9))) % len(t.delays_ms)


def sample(root: str, ref: str, offset_s: float, elapsed_s: float) -> Optional[float]:
    """The delay (ms) of a datagram sent ``elapsed_s`` into the segment; None when the trace loses it."""
    t = load(root, ref)
    d = t.delays_ms[index_at(t, offset_s, elapsed_s)]
    return None if d != d else float(d)


def window(root: str, ref: str, offset_s: float, elapsed_s: float, quantiles, half_s: float = 0.25,
           floor_s: float = 5.0) -> Tuple[np.ndarray, float, float]:
    """(quantiles ms, P(loss), floor ms) of the trace within ``half_s`` of the moment; the floor over ``floor_s``."""
    t = load(root, ref)
    n = len(t.delays_ms)
    centre = index_at(t, offset_s, elapsed_s)
    half = max(1, int(round(half_s / t.interval_s)))
    far = max(half, int(round(floor_s / t.interval_s)))
    near = t.delays_ms[(centre + np.arange(-half, half + 1)) % n]
    wide = t.delays_ms[(centre + np.arange(-far, far + 1)) % n]
    ok = near[~np.isnan(near)]
    loss = float(np.isnan(near).mean())
    if ok.size == 0:
        return np.zeros(len(quantiles)), loss, 0.0
    wok = wide[~np.isnan(wide)]
    return np.percentile(ok, [100 * q for q in quantiles]), loss, float(wok.min()) if wok.size else 0.0
