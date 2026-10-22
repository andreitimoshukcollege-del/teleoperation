"""Copy of ``Transport/NetworkProfileCatalog.cs`` (Core) and of ``Teleop.Eval/Sweep/NetworkProfileCatalog.cs``.

Names resolve exactly as in the repo, ``MsToTicks`` truncates exactly as C#'s ``(long)`` does, and
``create_impairments`` emits only the non-zero axes in Core's order (delay, jitter, loss, reorder;
a trace goes first), so the substreams and draws line up with Core's.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import List, Optional

from .impairments import FixedDelay, GilbertElliottLoss, Impairment, Reorder, TraceDelay, UniformJitter


@dataclass(frozen=True)
class NetworkProfile:
    base_delay_ticks: int
    jitter_ticks: int
    loss_after_delivered: float
    loss_after_lost: float
    reorder_probability: float = 0.0
    reorder_delay_ticks: int = 0


def ms_to_ticks(ms: float, ticks_per_second: int) -> int:
    # C#: (long)(ms / 1000.0 * ticksPerSecond) -- double arithmetic, truncation toward zero.
    return int(ms / 1000.0 * ticks_per_second)


NAMED_PROFILES = ("lan", "50ms-5j", "150ms-20j-0.5loss", "300ms-60j-2loss-bursty")
RESERVED_PENDING_REAL_CAPTURE = ("cellular-congested", "leo-satellite", "long-haul")

_NUMBER = r"(\d+(?:\.\d+)?)"
_JITTER = re.compile(rf"^jitter-{_NUMBER}ms$")
_DELAY = re.compile(rf"^delay-{_NUMBER}ms$")
_LOSS = re.compile(rf"^loss-{_NUMBER}pct$")


def resolve_parametric(name: str, tps: int) -> Optional[NetworkProfile]:
    if name == "lan":
        return NetworkProfile(ms_to_ticks(2, tps), ms_to_ticks(1, tps), 0.0, 0.0)
    if name == "50ms-5j":
        return NetworkProfile(ms_to_ticks(50, tps), ms_to_ticks(5, tps), 0.0, 0.0)
    if name == "150ms-20j-0.5loss":
        return NetworkProfile(ms_to_ticks(150, tps), ms_to_ticks(20, tps), 0.005, 0.005)
    if name == "300ms-60j-2loss-bursty":
        return NetworkProfile(ms_to_ticks(300, tps), ms_to_ticks(60, tps), 0.00612, 0.7)
    m = _JITTER.match(name)
    if m:
        return NetworkProfile(ms_to_ticks(50, tps), ms_to_ticks(float(m.group(1)), tps), 0.0, 0.0)
    m = _DELAY.match(name)
    if m:
        return NetworkProfile(ms_to_ticks(float(m.group(1)), tps), ms_to_ticks(5, tps), 0.0, 0.0)
    m = _LOSS.match(name)
    if m:
        p = float(m.group(1)) / 100.0
        return NetworkProfile(ms_to_ticks(100, tps), ms_to_ticks(10, tps), p, p)
    return _resolve_combined(name, tps)


def _resolve_combined(name: str, tps: int) -> Optional[NetworkProfile]:
    prefix = "combo__"
    if not name.startswith(prefix):
        return None
    delay = jitter = loss = None
    for segment in name[len(prefix):].split("__"):
        m = _DELAY.match(segment)
        if m:
            if delay is not None:
                return None
            delay = float(m.group(1))
            continue
        m = _JITTER.match(segment)
        if m:
            if jitter is not None:
                return None
            jitter = float(m.group(1))
            continue
        m = _LOSS.match(segment)
        if m:
            if loss is not None:
                return None
            loss = float(m.group(1))
            continue
        return None
    if delay is None and jitter is None and loss is None:
        return None
    p = (loss or 0.0) / 100.0
    return NetworkProfile(ms_to_ticks(delay or 0.0, tps), ms_to_ticks(jitter or 0.0, tps), p, p)


def create_impairments(profile: NetworkProfile) -> List[Impairment]:
    out: List[Impairment] = []
    if profile.base_delay_ticks != 0:
        out.append(FixedDelay(profile.base_delay_ticks))
    if profile.jitter_ticks != 0:
        out.append(UniformJitter(profile.jitter_ticks))
    if profile.loss_after_delivered != 0.0 or profile.loss_after_lost != 0.0:
        out.append(GilbertElliottLoss(profile.loss_after_delivered, profile.loss_after_lost))
    if profile.reorder_probability != 0.0:
        out.append(Reorder(profile.reorder_probability, profile.reorder_delay_ticks))
    return out


def read_trace(path: str):
    """Teleop.Eval Sweep/TraceFile.cs: ``TRACE|1|<tps>`` then one delay (ticks) per line."""
    with open(path, "r", encoding="ascii") as f:
        lines = f.read().splitlines()
    header = lines[0].split("|")
    if len(header) != 3 or header[0] != "TRACE" or header[1] != "1":
        raise ValueError(f"{path}: not a version-1 trace file")
    return int(header[2]), [int(x) for x in lines[1:]]


def rescale_trace(samples: List[int], from_tps: int, to_tps: int) -> List[int]:
    """Bridge's DelayTraceLoader rescales between tick rates; a trace replayed at another rate
    unscaled would inflate every delay by the ratio (unity/Teleop.Bridge/CLAUDE.md)."""
    if from_tps == to_tps:
        return list(samples)
    return [int(round(s * to_tps / from_tps)) for s in samples]


def resolve_impairments(name: str, tps: int, traces_dir: Optional[str] = None) -> List[Impairment]:
    """Eval's TryResolve, then CreateImpairments. ``synthetic-burst`` reads its trace from
    ``traces_dir`` (the packed trainer carries a copy under ``contract/traces``) and is rescaled to
    ``tps`` -- Eval would refuse a mismatched rate instead; the trainer runs views at the operator's
    rate, so it rescales the way the headset's loader does."""
    if name == "synthetic-burst":
        if traces_dir is None:
            raise ValueError("synthetic-burst needs traces_dir")
        trace_tps, samples = read_trace(os.path.join(traces_dir, "synthetic-burst.trace"))
        return [TraceDelay(rescale_trace(samples, trace_tps, tps))]
    if name in RESERVED_PENDING_REAL_CAPTURE:
        raise ValueError(f"'{name}' is reserved for a real network capture, not yet available")
    profile = resolve_parametric(name, tps)
    if profile is None:
        raise ValueError(f"unknown network profile '{name}'")
    return create_impairments(profile)
