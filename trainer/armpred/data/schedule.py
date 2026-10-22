"""Network schedules: what a link does, and when it changes.

Two kinds of segment, and the difference matters for what a number may be called:
- **named** segments use a profile from Core's catalog through the exact copies in ``armpred.net``,
  so a view built only from them is what ``Teleop.Eval`` would build; official scoring uses only
  these;
- **trainer-only** segments (random parametric profiles, delay ramps, sinusoidal delay, a filling
  queue, an outage) widen the training distribution. They are never used for a reported number.

A schedule describes either direction. **Offline**, for a recorded session, only the downlink is
synthesised in full; the uplink is only ever a slow shift of the command stream (|slope| <=
``MAX_UPLINK_SLOPE``), because a faster change, or uplink jitter and loss, would change which target
the relay's gate writes, and the recorded arm would no longer be the arm that would have moved. A
fast-changing uplink exists only where the arm physically lived through it: in the simulator
(``SimParams.uplink_schedule``) and in sessions recorded with the robot-side impairment on
(ADR 0021 decision 13).
"""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import asdict, dataclass, field, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ..net.impairments import EmulatedLink, Impairment
from ..net.profiles import NAMED_PROFILES, NetworkProfile, create_impairments, ms_to_ticks, resolve_impairments, resolve_parametric

MAX_UPLINK_SLOPE = 0.05  # seconds of delay change per second

# A filling queue drops arrivals near its top (tail drop); an outage loses some of what it holds.
QUEUE_TAIL_FRACTION = 0.9
QUEUE_TAIL_DROP_P = 0.3
OUTAGE_DROP_P = 0.2


@dataclass(frozen=True)
class Segment:
    """One link condition from ``start_s`` (seconds from the session start) on."""

    start_s: float
    name: str  # a named profile, or a description of a trainer-only one
    named: bool
    profile: Optional[NetworkProfile] = None  # trainer-only parametric
    ramp_to_ms: Optional[float] = None  # trainer-only: base delay ramps from the profile's to this
    sine_amplitude_ms: float = 0.0  # trainer-only: delay + A sin(2 pi f t)
    sine_hz: float = 0.0
    # Trainer-only, a filling queue (bufferbloat): the extra delay rises at ``queue_fill_ms_per_s``
    # to ``queue_peak_ms``, the queue drains at once, and it fills again; near the top, tail drop.
    queue_peak_ms: float = 0.0
    queue_fill_ms_per_s: float = 0.0
    # Trainer-only, an outage: nothing is delivered in [outage_at_s, outage_at_s + outage_s) (seconds
    # from the session start); what was sent in it arrives when it ends, some of it lost.
    outage_at_s: Optional[float] = None
    outage_s: float = 0.0

    def impairments(self, tps: int, traces_dir: Optional[str]) -> List[Impairment]:
        if self.named:
            return resolve_impairments(self.name, tps, traces_dir)
        return create_impairments(self.profile)

    def base_profile(self, tps: int) -> Optional[NetworkProfile]:
        """The profile behind the segment's impairments; None for a recorded trace."""
        if self.named:
            return resolve_parametric(self.name, tps)
        return self.profile

    def extra_delay_ticks(self, t_s: float, end_s: float, tps: int) -> int:
        """Delay a trainer-only segment adds on top of its impairments, at time ``t_s``."""
        return ms_to_ticks(self.extra_delay_ms(t_s, end_s, tps), tps)

    def extra_delay_ms(self, t_s: float, end_s: float, tps: int) -> float:
        extra_ms = 0.0
        if self.ramp_to_ms is not None and self.profile is not None:
            u = 0.0 if end_s <= self.start_s else min(1.0, max(0.0, (t_s - self.start_s) / (end_s - self.start_s)))
            base_ms = self.profile.base_delay_ticks / tps * 1000.0
            extra_ms += (self.ramp_to_ms - base_ms) * u
        if self.sine_amplitude_ms:
            extra_ms += self.sine_amplitude_ms * (1.0 + math.sin(2 * math.pi * self.sine_hz * (t_s - self.start_s)))
        if self.queue_peak_ms > 0.0 and self.queue_fill_ms_per_s > 0.0:
            period_s = self.queue_peak_ms / self.queue_fill_ms_per_s
            extra_ms += self.queue_fill_ms_per_s * ((t_s - self.start_s) % period_s)
        if self.in_outage(t_s):
            extra_ms += (self.outage_at_s + self.outage_s - t_s) * 1000.0
        return extra_ms

    def in_outage(self, t_s: float) -> bool:
        return self.outage_at_s is not None and self.outage_at_s <= t_s < self.outage_at_s + self.outage_s

    def extra_drop_probability(self, t_s: float) -> float:
        """The chance a trainer-only segment drops a datagram sent at ``t_s`` beyond its profile's loss."""
        p = 0.0
        if self.queue_peak_ms > 0.0 and self.queue_fill_ms_per_s > 0.0:
            period_s = self.queue_peak_ms / self.queue_fill_ms_per_s
            if (t_s - self.start_s) % period_s >= QUEUE_TAIL_FRACTION * period_s:
                p = QUEUE_TAIL_DROP_P
        if self.in_outage(t_s):
            p = max(p, OUTAGE_DROP_P)
        return p

    def to_json(self) -> dict:
        d = asdict(self)
        if self.profile is not None:
            d["profile"] = asdict(self.profile)
        return d

    @staticmethod
    def from_json(d: dict) -> "Segment":
        d = dict(d)
        if d.get("profile") is not None:
            d["profile"] = NetworkProfile(**d["profile"])
        return Segment(**d)


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

    def segment_end_s(self, k: int) -> float:
        return self.segments[k + 1].start_s if k + 1 < len(self.segments) else float("inf")

    def to_json(self) -> str:
        return json.dumps({"segments": [s.to_json() for s in self.segments], "uplink": [list(k) for k in self.uplink.knots],
                           "added_delay_ms": self.added_delay_ms}, sort_keys=True, separators=(",", ":"))

    @staticmethod
    def from_json(text: str) -> "Schedule":
        d = json.loads(text)
        return Schedule(segments=tuple(Segment.from_json(s) for s in d["segments"]),
                        uplink=UplinkShift(tuple(tuple(k) for k in d.get("uplink", [[0.0, 0.0]]))),
                        added_delay_ms=float(d.get("added_delay_ms", 0.0)))


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
    share_named: float = 0.2
    share_queue: float = 0.08  # ADR 0021 decision 13: a filling queue (buffering)
    share_outage: float = 0.04  # ADR 0021 decision 13: an outage, then a burst
    # The rest: bursty congestion, a sine riding on a low baseline.
    named: Sequence[str] = ("lan", "50ms-5j", "150ms-20j-0.5loss")  # training profiles only
    max_delay_ms: float = 600.0
    ramp_share: float = 0.5
    uplink_lan_share: float = 0.5


def _log_uniform(rng: np.random.Generator, lo: float, hi: float) -> float:
    return math.exp(rng.uniform(math.log(lo), math.log(hi)))


def _random_segments(duration_s: float, rng: np.random.Generator, config: RandomScheduleConfig,
                     allow_added_delay: bool) -> Tuple[List[Segment], float]:
    """The segments of a random training schedule, and the hold-back one of them asked for."""
    tps = 1_000_000_000
    segments: List[Segment] = []
    t = 0.0
    added_ms = 0.0
    c_named = config.share_named
    c_param = c_named + config.share_parametric
    c_added = c_param + config.share_added_delay
    c_queue = c_added + config.share_queue
    c_outage = c_queue + config.share_outage
    while t < duration_s:
        dwell = min(config.max_dwell_s, max(config.min_dwell_s, rng.exponential(config.mean_dwell_s)))
        r = rng.random()
        if r < c_named:
            segments.append(Segment(t, str(rng.choice(list(config.named))), True))
        elif r < c_added:
            # 30% near-LAN, 70% log-uniform from 20 ms to max_delay_ms (the methodology review's mix).
            delay = rng.uniform(0.0, 20.0) if rng.random() < 0.3 else _log_uniform(rng, 20.0, config.max_delay_ms)
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
                ramp = _log_uniform(rng, 5.0, config.max_delay_ms)
            sine_a = rng.uniform(0.0, 60.0) if rng.random() < 0.1 else 0.0
            segments.append(Segment(t, f"param:d{delay:.0f}j{jitter:.0f}l{p_good:.3f}/{p_bad:.2f}r{reorder:.3f}", False,
                                    profile=profile, ramp_to_ms=ramp, sine_amplitude_ms=sine_a,
                                    sine_hz=rng.uniform(0.05, 0.5)))
            if r >= c_param and allow_added_delay:
                added_ms = float(rng.choice([0.0, 50.0, 100.0, 200.0, 300.0, 400.0]))
        elif r < c_queue:
            # Buffering: a small flow behind a queue that fills, drains, and fills again.
            base = rng.uniform(2.0, 60.0)
            profile = NetworkProfile(ms_to_ticks(base, tps), ms_to_ticks(rng.uniform(0.5, 0.2 * base + 2.0), tps), 0.0, 0.0)
            peak = _log_uniform(rng, 80.0, config.max_delay_ms)
            fill = rng.uniform(50.0, 600.0)
            segments.append(Segment(t, f"queue:b{base:.0f}p{peak:.0f}f{fill:.0f}", False, profile=profile,
                                    queue_peak_ms=peak, queue_fill_ms_per_s=fill))
        elif r < c_outage:
            # An outage (a Wi-Fi roam, say): nothing for 0.2-2 s, then everything held arrives at once.
            base = rng.uniform(2.0, 80.0)
            profile = NetworkProfile(ms_to_ticks(base, tps), ms_to_ticks(rng.uniform(0.5, 10.0), tps), 0.0, 0.0)
            length = rng.uniform(0.2, 2.0)
            dwell = max(dwell, length + 1.0)
            at = t + rng.uniform(0.2, dwell - length)
            segments.append(Segment(t, f"outage:b{base:.0f}o{length * 1000:.0f}", False, profile=profile,
                                    outage_at_s=at, outage_s=length))
        else:
            # Bursty congestion, trainer-only: a low baseline with long delay bursts riding on it.
            # Not the synthetic-burst trace, which ADR 0021 holds out of training entirely.
            base = rng.uniform(5.0, 60.0)
            profile = NetworkProfile(ms_to_ticks(base, tps), ms_to_ticks(rng.uniform(1.0, 10.0), tps), 0.0, 0.0)
            segments.append(Segment(t, f"burst:b{base:.0f}", False, profile=profile,
                                    sine_amplitude_ms=rng.uniform(40.0, 250.0), sine_hz=rng.uniform(0.3, 2.0)))
        t += dwell
    return segments, added_ms


def random_schedule(duration_s: float, rng: np.random.Generator, config: RandomScheduleConfig = RandomScheduleConfig()) -> Schedule:
    """A training schedule for offline synthesis: a random downlink, a hold-back, and a slow uplink shift."""
    segments, added_ms = _random_segments(duration_s, rng, config, allow_added_delay=True)
    # The uplink: LAN half the time, otherwise a slow random walk around a fraction of the downlink.
    knots = [(0.0, 0.0)]
    if rng.random() >= config.uplink_lan_share:
        level = 0.0
        tk = 0.0
        while tk < duration_s:
            step = rng.uniform(5.0, 60.0)
            target = max(0.0, _log_uniform(rng, 5.0, config.max_delay_ms) * rng.uniform(0.5, 1.5))
            change = target - level
            limit = MAX_UPLINK_SLOPE * 1000.0 * step * 0.95
            change = max(-limit, min(limit, change))
            level = max(0.0, level + change)
            tk += step
            knots.append((tk, level))
    return Schedule(segments=tuple(segments), uplink=UplinkShift(tuple(knots)), added_delay_ms=added_ms)


def random_uplink_schedule(duration_s: float, rng: np.random.Generator,
                           config: RandomScheduleConfig = RandomScheduleConfig()) -> Schedule:
    """A physical uplink for the simulator (ADR 0021 decision 13): the same mix, no hold-back."""
    segments, _ = _random_segments(duration_s, rng, config, allow_added_delay=False)
    return Schedule(segments=tuple(segments))


_MASK64 = (1 << 64) - 1


class ScheduledLink:
    """One direction through a schedule, a datagram at a time in send order. Each segment has its
    own link, seeded from (seed, segment index), as one EmulatedTransport per segment would; a
    trainer-only segment then adds its extra delay and its extra drops."""

    def __init__(self, schedule: Schedule, seed: int, tps: int, traces_dir: Optional[str]):
        self.schedule = schedule
        self.seed = int(seed)
        self.tps = int(tps)
        self.traces_dir = traces_dir
        self._starts = [seg.start_s for seg in schedule.segments]
        self._links = {}
        self._drops = {}

    def transmit_one(self, send_ticks: int, send_s: float) -> Tuple[bool, int, int]:
        """(delivered, arrival, segment index)."""
        k = min(max(bisect.bisect_right(self._starts, send_s) - 1, 0), len(self._starts) - 1)
        segment = self.schedule.segments[k]
        link = self._links.get(k)
        if link is None:
            link = EmulatedLink(segment.impairments(self.tps, self.traces_dir), (self.seed * 1_000_003 + k) & _MASK64)
            self._links[k] = link
        ok, a = link.transmit_one(int(send_ticks))
        if ok and not segment.named:
            a += segment.extra_delay_ticks(float(send_s), self.schedule.segment_end_s(k), self.tps)
            p = segment.extra_drop_probability(float(send_s))
            if p > 0.0:
                drops = self._drops.get(k)
                if drops is None:
                    drops = np.random.default_rng(np.random.PCG64(((self.seed * 1_000_003 + k) ^ 0x9E3779B97F4A7C15) & _MASK64))
                    self._drops[k] = drops
                if drops.random() < p:
                    ok = False
        return ok, a, k


def build_link(schedule: Schedule, send_ticks: np.ndarray, send_s: np.ndarray, seed: int, tps: int,
               traces_dir: Optional[str]):
    """Every datagram through the link of the segment active when it was sent (``ScheduledLink``)."""
    n = len(send_ticks)
    delivered = np.zeros(n, dtype=bool)
    arrival = np.zeros(n, dtype=np.int64)
    seg = np.zeros(n, dtype=np.int64)
    link = ScheduledLink(schedule, seed, tps, traces_dir)
    for i in range(n):
        ok, a, k = link.transmit_one(int(send_ticks[i]), float(send_s[i]))
        delivered[i] = ok
        arrival[i] = a
        seg[i] = k
    return delivered, arrival, seg


# The downlink is a link like any other; the name stays for the views.
build_downlink = build_link
