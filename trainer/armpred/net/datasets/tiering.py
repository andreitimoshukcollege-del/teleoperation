"""Which tier a trace is in, by the standards (docs/adr/0035 amendment 2): G.1050's service test profiles, measured the
way Y.1541 measures.

- **Per minute** (Y.1541 Table 1's evaluation interval; a trace shorter than that is one window, and a last partial
  window shorter than 10 s joins the one before): the mean one-way delay of the datagrams that arrived, the delay
  variation (Y.1540's 2-point IPDV: the 1 - 10^-3 quantile of the delay minus its minimum), and the random loss.
- **Over the whole trace**: sequential (burst) loss, and reordering. A run of consecutive lost datagrams that lasts at
  least 40 ms (G.1050 Table 6's shortest burst) is sequential loss; a shorter run is random loss. Datagram i+1 is
  reordered when it arrives before datagram i.
- **G.1050 gives jitter as "peak-to-peak"** without saying over which packets; the 2-point IPDV above is used for it, so
  one late datagram in thousands does not decide the tier.

A trace's profile is the least severe one whose every bound holds in every minute (Y.1541: "Any minute observed should
meet these objectives"); past C, it is ``beyond-C``, still tier ``very-bad``.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

import numpy as np

from . import standards as S
from .trace import Trace

MIN_LAST_WINDOW_S = 10.0


@dataclass
class Window:
    start_s: float
    seconds: float
    sent: int
    mean_ms: Optional[float]
    min_ms: Optional[float]
    ipdv_ms: Optional[float]
    loss_ratio: float
    random_loss_ratio: float


@dataclass
class Assessment:
    seconds: float
    sent: int
    loss_ratio: float
    random_loss_ratio: float
    burst_count: int
    longest_burst_ms: float
    burst_rate_per_s: float
    reorder_ratio: float
    delay_ms: Dict[str, Optional[float]]
    worst_minute: Dict[str, Optional[float]]
    y1541_class: int
    g1050_profile: str  # "A well-managed" .. "C unmanaged", or "beyond-C"
    tier: str
    windows: List[Window] = field(default_factory=list)

    def to_json(self, with_windows: bool = False) -> Dict:
        d = asdict(self)
        if not with_windows:
            d.pop("windows")
        return d


def quantile(values: np.ndarray, q: float) -> float:
    """Nearest rank, as armpred.net.linktrace._pct: the sorted value at round(q * (n - 1))."""
    s = np.sort(values)
    return float(s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))])


def _runs(lost: np.ndarray) -> List[int]:
    """Lengths of the runs of consecutive lost datagrams."""
    runs, n = [], 0
    for x in lost:
        if x:
            n += 1
        elif n:
            runs.append(n)
            n = 0
    if n:
        runs.append(n)
    return runs


def _random_mask(lost: np.ndarray, burst_min: int) -> np.ndarray:
    """True where a lost datagram belongs to a run shorter than ``burst_min`` (random loss)."""
    out = np.zeros(len(lost), dtype=bool)
    i = 0
    while i < len(lost):
        if lost[i]:
            j = i
            while j < len(lost) and lost[j]:
                j += 1
            if j - i < burst_min:
                out[i:j] = True
            i = j
        else:
            i += 1
    return out


def windows_of(trace: Trace, burst_min: int) -> List[Window]:
    per = max(1, int(round(S.EVALUATION_INTERVAL_S * 1000.0 / trace.interval_ms)))
    tail = int(round(MIN_LAST_WINDOW_S * 1000.0 / trace.interval_ms))
    n = len(trace.delays)
    bounds = list(range(0, n, per)) + [n]
    if len(bounds) > 2 and bounds[-1] - bounds[-2] < tail:
        del bounds[-2]
    ms = trace.delays_ms()
    lost = trace.lost
    rnd = _random_mask(lost, burst_min)
    out = []
    for a, b in zip(bounds, bounds[1:]):
        d = ms[a:b][~lost[a:b]]
        sent = b - a
        out.append(Window(
            start_s=round(a * trace.interval_ms / 1000.0, 3), seconds=round(sent * trace.interval_ms / 1000.0, 3), sent=sent,
            mean_ms=round(float(d.mean()), 3) if len(d) else None,
            min_ms=round(float(d.min()), 3) if len(d) else None,
            ipdv_ms=round(quantile(d, S.IPDV_QUANTILE) - float(d.min()), 3) if len(d) else None,
            loss_ratio=float(lost[a:b].sum()) / sent, random_loss_ratio=float(rnd[a:b].sum()) / sent))
    return out


def _meets_class(c: S.QosClass, w: Window) -> bool:
    if w.mean_ms is None:
        return c.loss_ratio is None
    if c.mean_delay_ms is not None and w.mean_ms > c.mean_delay_ms:
        return False
    if c.ipdv_ms is not None and w.ipdv_ms > c.ipdv_ms:
        return False
    return c.loss_ratio is None or w.loss_ratio <= c.loss_ratio


def _meets_profile(p: S.ServiceProfile, ws: List[Window], longest_burst_ms: float, bursts: int, rate: float, reorder: float) -> bool:
    for w in ws:
        if w.mean_ms is None:
            return False
        if w.mean_ms > p.latency_ms or w.ipdv_ms > p.jitter_pp_ms or w.random_loss_ratio > p.random_loss_ratio:
            return False
    if p.burst_loss_ms is None:
        if bursts:
            return False
    elif bursts and (longest_burst_ms > p.burst_loss_ms or rate >= p.burst_rate_per_s):
        return False
    return reorder <= p.reorder_ratio


def assess(trace: Trace) -> Assessment:
    if trace.send_interval_ticks <= 0:
        raise ValueError("assessing a trace needs its send interval (a version-2 trace)")
    if len(trace.delays) == 0:
        raise ValueError("an empty trace")
    burst_min = max(2, int(np.ceil(S.G1050_SHORTEST_BURST_MS / trace.interval_ms - 1e-9)))
    lost = trace.lost
    runs = _runs(lost)
    bursts = [r for r in runs if r >= burst_min]
    seconds = trace.seconds
    ms = trace.delays_ms()
    arrived = ~lost
    idx = np.nonzero(arrived)[0]
    arrival = idx * trace.interval_ms + ms[idx]
    reordered = int(np.sum(np.diff(arrival) < -1e-6)) if len(arrival) > 1 else 0  # equal arrivals are in order
    ws = windows_of(trace, burst_min)
    d = ms[arrived]
    longest = max(bursts) * trace.interval_ms if bursts else 0.0
    rate = len(bursts) / seconds if seconds > 0 else 0.0
    reorder = reordered / max(1, len(d))
    y = next(c.number for c in S.Y1541_CLASSES if all(_meets_class(c, w) for w in ws))
    profile = next((p for p in S.G1050_PROFILES if _meets_profile(p, ws, longest, len(bursts), rate, reorder)), None)
    measured = [w for w in ws if w.mean_ms is not None]
    return Assessment(
        seconds=round(seconds, 3), sent=len(trace.delays),
        loss_ratio=round(float(lost.mean()), 6),
        random_loss_ratio=round(float(_random_mask(lost, burst_min).mean()), 6),
        burst_count=len(bursts), longest_burst_ms=round(longest, 3), burst_rate_per_s=round(rate, 6),
        reorder_ratio=round(reorder, 6),
        delay_ms={k: (round(quantile(d, q), 3) if len(d) else None) for k, q in (("min", 0.0), ("p50", .5), ("p95", .95), ("p99", .99), ("max", 1.0))},
        worst_minute={
            "mean_ms": max((w.mean_ms for w in measured), default=None),
            "ipdv_ms": max((w.ipdv_ms for w in measured), default=None),
            "loss_ratio": max(w.loss_ratio for w in ws),
        },
        y1541_class=y,
        g1050_profile=profile.name if profile else "beyond-C",
        tier=S.TIER_OF_PROFILE[profile.letter] if profile else "very-bad",
        windows=ws,
    )
