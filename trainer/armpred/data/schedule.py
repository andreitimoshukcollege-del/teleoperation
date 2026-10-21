"""Network schedules: what the downlink and the uplink do, and when it changes.

Two kinds of segment, and the difference matters for what a number may be called:
- **named** segments use a profile from Core's catalog through the exact copies in ``armpred.net``,
  so a view built only from them is what ``Teleop.Eval`` would build; official scoring uses only
  these;
- **trainer-only** segments (random parametric profiles, delay ramps, sinusoidal delay) widen the
  training distribution. They are never used for a reported number.

The uplink is only ever a slow shift of the command stream (|slope| <= ``MAX_UPLINK_SLOPE``),
because a faster change, or uplink jitter and loss, would change which target the relay's gate
writes, and the recorded arm would no longer be the arm that would have moved.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..net.impairments import EmulatedLink, Impairment
from ..net.profiles import NAMED_PROFILES, NetworkProfile, create_impairments, ms_to_ticks, resolve_impairments

MAX_UPLINK_SLOPE = 0.05  # seconds of delay change per second


@dataclass(frozen=True)
class Segment:
    """One downlink condition from ``start_s`` (seconds from the session start) on."""

    start_s: float
    name: str  # a named profile, or a description of a trainer-only one
    named: bool
    profile: Optional[NetworkProfile] = None  # trainer-only parametric
    ramp_to_ms: Optional[float] = None  # trainer-only: base delay ramps from the profile's to this
    sine_amplitude_ms: float = 0.0  # trainer-only: delay + A sin(2 pi f t)
    sine_hz: float = 0.0

    def impairments(self, tps: int, traces_dir: Optional[str]) -> List[Impairment]:
        if self.named:
            return resolve_impairments(self.name, tps, traces_dir)
        return create_impairments(self.profile)

    def extra_delay_ticks(self, t_s: float, end_s: float, tps: int) -> int:
        """Delay a trainer-only segment adds on top of its impairments, at time ``t_s``."""
        extra_ms = 0.0
        if self.ramp_to_ms is not None and self.profile is not None:
            u = 0.0 if end_s <= self.start_s else min(1.0, max(0.0, (t_s - self.start_s) / (end_s - self.start_s)))
            base_ms = self.profile.base_delay_ticks / tps * 1000.0
            extra_ms += (self.ramp_to_ms - base_ms) * u
        if self.sine_amplitude_ms:
            extra_ms += self.sine_amplitude_ms * (1.0 + math.sin(2 * math.pi * self.sine_hz * (t_s - self.start_s)))
        return ms_to_ticks(extra_ms, tps)


@dataclass(frozen=True)
class UplinkShift:
    """Extra uplink delay over time: piecewise-linear knots (seconds, ms), slopes bounded."""

    knots: Tuple[Tuple[float, float], ...] = ((0.0, 0.0),)

    def __post_init__(self) -> None:
        for (t0, d0), (t1, d1) in zip(self.knots, self.knots[1:]):
            if t1 <= t0:
                raise ValueError("uplink knots must increase in time")
            if abs(d1 - d0) / 1000.0 / (t1 - t0) > MAX_UPLINK_SLOPE + 1e-12:
                raise ValueError(f"uplink shift changes faster than {MAX_UPLINK_SLOPE} s/s between {t0} and {t1}")
        if any(d < 0 for _, d in self.knots):
            raise ValueError("an uplink shift cannot be negative")

    def ms_at(self, t_s: np.ndarray) -> np.ndarray:
        ts = np.array([k[0] for k in self.knots])
        ds = np.array([k[1] for k in self.knots])
        return np.interp(np.asarray(t_s, dtype=np.float64), ts, ds)


@dataclass(frozen=True)
class Schedule:
    segments: Tuple[Segment, ...]
    uplink: UplinkShift = UplinkShift()
    added_delay_ms: float = 0.0  # AddedDelayMs-style hold-back at the headset

    @property
    def only_named(self) -> bool:
        return all(s.named for s in self.segments) and self.added_delay_ms == 0.0 and self.uplink.knots == ((0.0, 0.0),)

    def describe(self) -> str:
        parts = [f"{s.name}@{s.start_s:g}" for s in self.segments]
        if self.uplink.knots != ((0.0, 0.0),):
            parts.append("uplink:" + ",".join(f"{t:g}s={d:g}ms" for t, d in self.uplink.knots))
        if self.added_delay_ms:
            parts.append(f"added:{self.added_delay_ms:g}ms")
        return ";".join(parts)

    def segment_index(self, t_s: np.ndarray) -> np.ndarray:
        starts = np.array([s.start_s for s in self.segments])
        return np.clip(np.searchsorted(starts, np.asarray(t_s), side="right") - 1, 0, len(self.segments) - 1)


def parse(spec: str) -> Schedule:
    """``lan@0;150ms-20j-0.5loss@60`` -- named profiles only (the grammar official scoring uses)."""
    segments = []
    for part in spec.split(";"):
        name, _, start = part.partition("@")
        segments.append(Segment(start_s=float(start or 0.0), name=name, named=True))
    segments.sort(key=lambda s: s.start_s)
    if segments[0].start_s != 0.0:
        raise ValueError("a schedule's first segment must start at 0")
    return Schedule(segments=tuple(segments))


@dataclass(frozen=True)
class RandomScheduleConfig:
    """ADR 0021's training mix. Shares are of segments, not of time."""

    mean_dwell_s: float = 6.0
    min_dwell_s: float = 0.5
    max_dwell_s: float = 30.0
    share_parametric: float = 0.4
    share_added_delay: float = 0.2
    share_named: float = 0.2  # the rest: synthetic-burst-style traces
    named: Sequence[str] = ("lan", "50ms-5j", "150ms-20j-0.5loss")  # training profiles only
    max_delay_ms: float = 600.0
    ramp_share: float = 0.5
    uplink_lan_share: float = 0.5


def random_schedule(duration_s: float, rng: np.random.Generator, config: RandomScheduleConfig = RandomScheduleConfig()) -> Schedule:
    tps = 1_000_000_000
    segments: List[Segment] = []
    t = 0.0
    added_ms = 0.0
    while t < duration_s:
        r = rng.random()
        if r < config.share_named:
            segments.append(Segment(t, str(rng.choice(list(config.named))), True))
        elif r < config.share_named + config.share_parametric + config.share_added_delay:
            # 30% near-LAN, 70% log-uniform from 20 ms to max_delay_ms (the methodology review's mix).
            delay = rng.uniform(0.0, 20.0) if rng.random() < 0.3 else math.exp(rng.uniform(math.log(20.0), math.log(config.max_delay_ms)))
            jitter = rng.uniform(0.0, 0.3 * delay + 5.0)
            if rng.random() < 0.5:
                p_good, p_bad = rng.uniform(0.0, 0.03), rng.uniform(0.0, 0.7)
            else:
                p_good = p_bad = rng.uniform(0.0, 0.05)
            reorder = rng.uniform(0.0, 0.02) if rng.random() < 0.3 else 0.0
            profile = NetworkProfile(ms_to_ticks(delay, tps), ms_to_ticks(jitter, tps), p_good, p_bad,
                                     reorder, ms_to_ticks(rng.uniform(5.0, 40.0), tps))
            ramp = None
            if rng.random() < config.ramp_share:
                ramp = math.exp(rng.uniform(math.log(5.0), math.log(config.max_delay_ms)))
            sine_a = rng.uniform(0.0, 60.0) if rng.random() < 0.1 else 0.0
            segments.append(Segment(t, f"param:d{delay:.0f}j{jitter:.0f}l{p_good:.3f}/{p_bad:.2f}r{reorder:.3f}", False,
                                    profile=profile, ramp_to_ms=ramp, sine_amplitude_ms=sine_a,
                                    sine_hz=rng.uniform(0.05, 0.5)))
            if r >= config.share_named + config.share_parametric:
                added_ms = float(rng.choice([0.0, 50.0, 100.0, 200.0, 300.0, 400.0]))
        else:
            # Bursty congestion, trainer-only: a low baseline with long delay bursts riding on it.
            # Not the synthetic-burst trace, which ADR 0021 holds out of training entirely.
            base = rng.uniform(5.0, 60.0)
            profile = NetworkProfile(ms_to_ticks(base, tps), ms_to_ticks(rng.uniform(1.0, 10.0), tps), 0.0, 0.0)
            segments.append(Segment(t, f"burst:b{base:.0f}", False, profile=profile,
                                    sine_amplitude_ms=rng.uniform(40.0, 250.0), sine_hz=rng.uniform(0.3, 2.0)))
        dwell = min(config.max_dwell_s, max(config.min_dwell_s, rng.exponential(config.mean_dwell_s)))
        t += dwell
    # The uplink: LAN half the time, otherwise a slow random walk around a fraction of the downlink.
    knots = [(0.0, 0.0)]
    if rng.random() >= config.uplink_lan_share:
        level = 0.0
        tk = 0.0
        while tk < duration_s:
            step = rng.uniform(5.0, 60.0)
            target = max(0.0, math.exp(rng.uniform(math.log(5.0), math.log(config.max_delay_ms))) * rng.uniform(0.5, 1.5))
            change = target - level
            limit = MAX_UPLINK_SLOPE * 1000.0 * step * 0.95
            change = max(-limit, min(limit, change))
            level = max(0.0, level + change)
            tk += step
            knots.append((tk, level))
    return Schedule(segments=tuple(segments), uplink=UplinkShift(tuple(knots)), added_delay_ms=added_ms)


def build_downlink(schedule: Schedule, send_ticks: np.ndarray, send_s: np.ndarray, seed: int, tps: int,
                   traces_dir: Optional[str]):
    """Each frame goes through the link of the segment active when it was sent; each segment has its
    own link, seeded from (seed, segment index), as one EmulatedTransport per segment would."""
    n = len(send_ticks)
    delivered = np.zeros(n, dtype=bool)
    arrival = np.zeros(n, dtype=np.int64)
    seg = schedule.segment_index(send_s)
    ends = [s.start_s for s in schedule.segments[1:]] + [float("inf")]
    links = {}
    for i in range(n):
        k = int(seg[i])
        link = links.get(k)
        if link is None:
            link = EmulatedLink(schedule.segments[k].impairments(tps, traces_dir), (seed * 1_000_003 + k) & ((1 << 64) - 1))
            links[k] = link
        ok, a = link.transmit_one(int(send_ticks[i]))
        if ok and not schedule.segments[k].named:
            a += schedule.segments[k].extra_delay_ticks(float(send_s[i]), ends[k], tps)
        delivered[i] = ok
        arrival[i] = a
    return delivered, arrival, seg
