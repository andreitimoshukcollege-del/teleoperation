"""Network forecast v1 (contract/FORECAST_FORMAT.md), and forecasters of known quality.

A separate network model, built by someone else later, will tell the arm model what the network is
doing and is about to do (ADR 0021 decisions 11 and 12). Until it exists, the arm model trains and is
evaluated on forecasts made here from each view's true conditions, at fixed quality levels, so the
benefit can be measured against forecast quality before anyone builds the real thing.

- **The truth** (``LinkTruth``) is one direction's conditions as a function of send time: the
  distribution of the one-way delay (as quantiles), the loss probability, the floor and the outages,
  from the schedule that made the view. It is never a model input.
- **A forecast** is issued every ``ISSUE_INTERVAL_S``. For each direction and each horizon h of
  ``HORIZONS_MS`` (send time t + h; negative h is what was already sent) it gives the delay
  quantiles ``QUANTILES`` and P(loss); per direction, the floor and P(an outage of at least
  ``OUTAGE_MIN_S`` within ``OUTAGE_WINDOW_S``); and validity flags.
- **The levels** ``oracle``/``good``/``fair``/``poor`` degrade the truth the way ADR 0021 decision 12
  fixes; ``persistence`` and ``rtt-half`` use only what the headset observes today; ``absent`` is no
  forecast at all.

Everything is seeded and deterministic for a given view, level and seed.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

HORIZONS_MS: Tuple[int, ...] = (-1000, -500, -250, -100, 0, 100, 250, 500, 1000)
QUANTILES: Tuple[float, ...] = (0.10, 0.50, 0.90, 0.99)
H0 = HORIZONS_MS.index(0)
UP, DOWN = 0, 1
ISSUE_INTERVAL_S = 0.1
OUTAGE_MIN_S = 0.25
OUTAGE_WINDOW_S = 1.0
FALSE_ALARM_S = (0.5, 3.0)


@dataclass(frozen=True)
class Level:
    """A forecaster's quality (ADR 0021 decision 12's table)."""

    name: str
    latency_s: float  # how long after a change the forecaster knows it
    announce_p: float  # the share of changes it announces in advance
    lead_s: Tuple[float, float]  # how far in advance, uniform
    sigma: float  # lognormal error on the delay above the floor (and on P(loss)), one draw per change
    false_alarms_per_s: float
    spread: Tuple[float, float]  # the quantiles' spread around p50 is scaled by U(lo, hi), once per view


DEGRADED: Dict[str, Level] = {
    "oracle": Level("oracle", 0.0, 1.0, (60.0, 60.0), 0.0, 0.0, (1.0, 1.0)),
    "good": Level("good", 0.1, 0.7, (0.1, 1.0), 0.15, 1.0 / 300.0, (0.8, 1.25)),
    "fair": Level("fair", 0.3, 0.4, (0.1, 1.0), 0.30, 1.0 / 60.0, (0.6, 1.6)),
    "poor": Level("poor", 1.0, 0.1, (0.1, 1.0), 0.50, 1.0 / 30.0, (0.5, 2.0)),
}
CAUSAL = ("persistence", "rtt-half")
LEVELS: Tuple[str, ...] = tuple(DEGRADED) + CAUSAL + ("absent",)
# ADR 0021 decision 12: each training view draws its level from this mixture.
TRAINING_MIXTURE: Dict[str, float] = {"oracle": 0.10, "good": 0.20, "fair": 0.20, "poor": 0.15,
                                      "persistence": 0.15, "rtt-half": 0.10, "absent": 0.10}


@dataclass
class ForecastStream:
    """Every forecast a view's headset would have received, in issue order."""

    level: str
    t: np.ndarray  # int64 [n], operator ticks
    q_ms: np.ndarray  # float32 [n, 2, len(HORIZONS_MS), len(QUANTILES)]
    loss: np.ndarray  # float32 [n, 2, len(HORIZONS_MS)]
    floor_ms: np.ndarray  # float32 [n, 2]
    outage: np.ndarray  # float32 [n, 2]
    valid_q: np.ndarray  # bool [n, 2]
    valid_loss: np.ndarray  # bool [n, 2]
    valid_outage: np.ndarray  # bool [n, 2]

    def __len__(self) -> int:
        return int(self.t.shape[0])

    def index_at(self, t: int) -> int:
        """The newest forecast issued at or before ``t``; -1 if none."""
        return int(np.searchsorted(self.t, t, side="right")) - 1

    @staticmethod
    def empty(level: str, t: np.ndarray) -> "ForecastStream":
        n = len(t)
        z = lambda *shape: np.zeros((n,) + shape, dtype=np.float32)  # noqa: E731
        f = lambda: np.zeros((n, 2), dtype=bool)  # noqa: E731
        return ForecastStream(level, np.asarray(t, dtype=np.int64), z(2, len(HORIZONS_MS), len(QUANTILES)),
                              z(2, len(HORIZONS_MS)), z(2), z(2), f(), f(), f())


# -- the truth --------------------------------------------------------------------------------------

def profile_quantiles_ms(base_ms: float, jitter_ms: float, reorder_p: float, reorder_ms: float) -> np.ndarray:
    """Quantiles of max(0, base + U(-J, J) + R * Bernoulli(p)): Core's fixed delay, uniform jitter and
    reordering. The jitter is continuous here; Core's is whole ticks, which changes nothing at ms scale."""
    out = np.zeros(len(QUANTILES))
    for i, p in enumerate(QUANTILES):
        if jitter_ms <= 0.0:
            x = base_ms + (reorder_ms if p > 1.0 - reorder_p else 0.0)
        else:
            lo, hi = base_ms - jitter_ms, base_ms + jitter_ms + reorder_ms
            width = 2.0 * jitter_ms
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                cdf = (1.0 - reorder_p) * min(1.0, max(0.0, (mid - (base_ms - jitter_ms)) / width)) + \
                    reorder_p * min(1.0, max(0.0, (mid - (base_ms - jitter_ms + reorder_ms)) / width))
                if cdf < p:
                    lo = mid
                else:
                    hi = mid
            x = 0.5 * (lo + hi)
        out[i] = max(0.0, x)
    return out


def stationary_loss(after_delivered: float, after_lost: float) -> float:
    """A Gilbert-Elliott chain's long-run loss: P(lost) = a / (1 - b + a)."""
    a, b = float(after_delivered), float(after_lost)
    if a <= 0.0:
        return 0.0
    return a / (1.0 - b + a)


@dataclass
class _SegmentTruth:
    start: int  # operator ticks
    end: int
    q_base_ms: np.ndarray  # [4], the profile's quantiles before trainer-only extras
    loss_base: float
    floor_ms: float
    segment: object  # schedule.Segment
    outage: Optional[Tuple[int, int]]  # operator ticks


class LinkTruth:
    """One direction's true conditions by send time, from the schedule that made a view.

    ``to_sched_s`` maps operator ticks to the schedule's seconds and ``to_op`` maps back; ``offset_ms``
    is a delay every datagram also carries (the simulator's LAN under its uplink, the offline uplink
    shift). A segment with no parametric profile (a recorded trace) takes its quantiles from what the
    datagrams sent near the time actually did (``realized``: send ticks, delay ms, delivered)."""

    def __init__(self, schedule, tps: int, to_sched_s: Callable[[float], float], to_op: Callable[[float], int],
                 offset_ms: Callable[[float], float] = lambda t: 0.0,
                 realized: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None):
        self.schedule = schedule
        self.tps = int(tps)
        self.to_sched_s = to_sched_s
        self.offset_ms = offset_ms
        self.segments: List[_SegmentTruth] = []
        for k, seg in enumerate(schedule.segments):
            start = to_op(seg.start_s)
            end_s = schedule.segment_end_s(k)
            end = to_op(end_s) if math.isfinite(end_s) else np.iinfo(np.int64).max
            prof = seg.base_profile(self.tps)
            if prof is not None:
                base, jit = prof.base_delay_ticks / self.tps * 1e3, prof.jitter_ticks / self.tps * 1e3
                q = profile_quantiles_ms(base, jit, prof.reorder_probability, prof.reorder_delay_ticks / self.tps * 1e3)
                loss = stationary_loss(prof.loss_after_delivered, prof.loss_after_lost)
                floor = max(0.0, base - jit)
            else:
                q, loss, floor = None, 0.0, 0.0
            outage = None
            if getattr(seg, "outage_at_s", None) is not None and seg.outage_s >= OUTAGE_MIN_S:
                outage = (to_op(seg.outage_at_s), to_op(seg.outage_at_s + seg.outage_s))
            self.segments.append(_SegmentTruth(start, end, q, loss, floor, seg, outage))
        self._starts = [s.start for s in self.segments]
        self._realized = None
        if realized is not None:
            send, delay, delivered = realized
            order = np.argsort(send, kind="stable")
            self._realized = (np.asarray(send)[order], np.asarray(delay, dtype=np.float64)[order], np.asarray(delivered)[order])

    def segment_at(self, t_op: int) -> int:
        return min(max(bisect.bisect_right(self._starts, t_op) - 1, 0), len(self._starts) - 1)

    def changes(self) -> List[int]:
        """The operator tick at which each segment after the first begins."""
        return self._starts[1:]

    def outages(self) -> List[Tuple[int, int, int]]:
        return [(s.outage[0], s.outage[1], k) for k, s in enumerate(self.segments) if s.outage is not None]

    def value(self, k: int, t_op: int, with_outage: bool = True) -> Tuple[np.ndarray, float, float]:
        """Segment ``k``'s (quantiles ms [4], P(loss), floor ms) for a datagram sent at ``t_op``."""
        s = self.segments[k]
        seg = s.segment
        x = self.to_sched_s(t_op)
        offset = self.offset_ms(t_op)
        if s.q_base_ms is None:
            q, loss, floor = self._realized_value(t_op)
        else:
            q, loss, floor = s.q_base_ms.copy(), s.loss_base, s.floor_ms
        if not seg.named:
            extra = seg.extra_delay_ms(x, self.schedule.segment_end_s(k), self.tps)
            if not with_outage and seg.in_outage(x):
                extra -= (seg.outage_at_s + seg.outage_s - x) * 1000.0
            q = np.maximum(0.0, q + extra)
            p_extra = seg.extra_drop_probability(x)
            if not with_outage and seg.in_outage(x):
                p_extra = 0.0
            loss = 1.0 - (1.0 - loss) * (1.0 - p_extra)
        return q + offset, float(loss), floor + offset

    def _realized_value(self, t_op: int) -> Tuple[np.ndarray, float, float]:
        if self._realized is None:
            return np.zeros(len(QUANTILES)), 0.0, 0.0
        send, delay, delivered = self._realized
        half = int(0.25 * self.tps)
        a, b = np.searchsorted(send, t_op - half), np.searchsorted(send, t_op + half)
        ok = delivered[a:b]
        d = delay[a:b][ok]
        loss = float(1.0 - ok.mean()) if b > a else 0.0
        if d.size == 0:
            return np.zeros(len(QUANTILES)), loss, 0.0
        far = int(5 * self.tps)
        a2, b2 = np.searchsorted(send, t_op - far), np.searchsorted(send, t_op + far)
        dd = delay[a2:b2][delivered[a2:b2]]
        return np.percentile(d, [100 * q for q in QUANTILES]), loss, float(dd.min()) if dd.size else 0.0

    def outage_ahead(self, t_op: int) -> bool:
        """Is there an outage of at least OUTAGE_MIN_S overlapping [t, t + OUTAGE_WINDOW_S)?"""
        w = int(OUTAGE_WINDOW_S * self.tps)
        return any(a < t_op + w and b > t_op for a, b, _ in self.outages())


# -- what the headset observes (for the causal levels) ---------------------------------------------

@dataclass
class Observations:
    """What the headset could have measured, from a view: each delivered frame's arrival and its
    send time as the headset estimated it (ClockSync's offset at the time), and each keepalive round
    trip with the downlink delay of the frame that echoed it."""

    tps: int
    frame_arrival: np.ndarray  # int64, delivered frames only, in frame (sequence) order
    frame_send_est: np.ndarray  # int64, -1 before sync
    frame_index: np.ndarray  # int64, the frame's position in the session (its sequence)
    rtt_t: np.ndarray  # int64, when each round trip completed
    rtt_ticks: np.ndarray  # int64, every round trip offered, accepted or not
    rtt_down_ticks: np.ndarray  # int64, the observed downlink delay of the echoing frame (-1 unknown)
    sync_t: np.ndarray  # int64, ClockSync's state after each round trip
    sync_uncertainty: np.ndarray  # int64, RTT/2
    sync_min_rtt: np.ndarray  # int64
    sync_synced: np.ndarray  # bool


# -- the forecasters --------------------------------------------------------------------------------

def issue_times(start: int, end: int, tps: int) -> np.ndarray:
    step = int(round(ISSUE_INTERVAL_S * tps))
    return np.arange(int(start), int(end) + 1, step, dtype=np.int64)


def make_forecasts(level: str, t_issue: np.ndarray, truths: Sequence[LinkTruth], obs: Optional[Observations],
                   seed: int) -> ForecastStream:
    """The forecast stream a view's headset would have had from a forecaster of ``level``."""
    if level == "absent":
        return ForecastStream.empty(level, t_issue)
    if level in DEGRADED:
        return _degraded(DEGRADED[level], t_issue, truths, seed)
    if level == "persistence":
        return _persistence(t_issue, obs)
    if level == "rtt-half":
        return _rtt_half(t_issue, obs)
    raise ValueError(f"unknown forecast level {level!r}; one of {LEVELS}")


@dataclass
class _Plan:
    """A degraded forecaster's draws for one direction: which changes it announces and how early, its
    error on each segment, and its false alarms."""

    announced: np.ndarray  # bool per change (segment k >= 1)
    lead: np.ndarray  # ticks per change
    z_delay: np.ndarray  # per segment
    z_loss: np.ndarray
    outage_announced: Dict[int, Tuple[bool, int]]  # segment -> (announced, lead ticks)
    alarms: List[Tuple[int, int, int, float]]  # (start, end, lead ticks, delta ms)
    spread: float
    sigma: float


def _plan(level: Level, truth: LinkTruth, rng: np.random.Generator, span: Tuple[int, int]) -> _Plan:
    tps = truth.tps
    n_changes = len(truth.changes())
    announced = rng.random(n_changes) < level.announce_p
    lead = (rng.uniform(level.lead_s[0], level.lead_s[1], n_changes) * tps).astype(np.int64)
    z_delay = rng.normal(0.0, 1.0, len(truth.segments))
    z_loss = rng.normal(0.0, 1.0, len(truth.segments))
    outage_announced = {}
    for _, _, k in truth.outages():
        outage_announced[k] = (bool(rng.random() < level.announce_p),
                               int(rng.uniform(level.lead_s[0], level.lead_s[1]) * tps))
    alarms = []
    if level.false_alarms_per_s > 0.0:
        t = span[0]
        while True:
            t += int(rng.exponential(1.0 / level.false_alarms_per_s) * tps)
            if t >= span[1]:
                break
            length = int(rng.uniform(*FALSE_ALARM_S) * tps)
            delta = math.exp(rng.uniform(math.log(20.0), math.log(300.0))) * (1.0 if rng.random() < 0.7 else -1.0)
            alarms.append((t, t + length, int(rng.uniform(level.lead_s[0], level.lead_s[1]) * tps), delta))
    spread = float(rng.uniform(*level.spread))
    return _Plan(announced, lead, z_delay, z_loss, outage_announced, alarms, spread, level.sigma)


def _degraded(level: Level, t_issue: np.ndarray, truths: Sequence[LinkTruth], seed: int) -> ForecastStream:
    out = ForecastStream.empty(level.name, t_issue)
    if len(t_issue) == 0:
        return out
    span = (int(t_issue[0]) - int(2 * truths[0].tps), int(t_issue[-1]) + int(2 * truths[0].tps))
    for d, truth in enumerate(truths):
        rng = np.random.default_rng(np.random.PCG64([int(seed) & 0xFFFFFFFF, d, 0xF0CA57]))
        plan = _plan(level, truth, rng, span)
        tps = truth.tps
        latency = int(level.latency_s * tps)
        changes = truth.changes()
        out.valid_q[:, d] = True
        out.valid_loss[:, d] = True
        out.valid_outage[:, d] = True
        for i, t in enumerate(t_issue):
            t = int(t)
            known = t - latency
            k_known = truth.segment_at(known)
            for hi, h_ms in enumerate(HORIZONS_MS):
                s = t + int(h_ms / 1000.0 * tps)
                q, loss = _belief(truth, plan, changes, t, known, k_known, s)
                for a_start, a_end, a_lead, delta in plan.alarms:
                    if a_start - a_lead <= t < a_start + latency and a_start <= s < a_end:
                        q = np.maximum(0.0, q + delta)
                out.q_ms[i, d, hi] = q
                out.loss[i, d, hi] = loss
            _, _, floor = truth.value(k_known, known, with_outage=False)
            out.floor_ms[i, d] = floor
            out.outage[i, d] = 1.0 if _outage_seen(truth, plan, t, known) else 0.0
    return out


def _belief(truth: LinkTruth, plan: _Plan, changes: List[int], t: int, known: int, k_known: int,
            s: int) -> Tuple[np.ndarray, float]:
    """What a degraded forecaster believes about a datagram sent at ``s``, issued at ``t``."""
    if s <= known:
        k, t_eval, outage_ok = truth.segment_at(s), s, True
    else:
        k, t_eval = k_known, known
        for j in range(k_known + 1, truth.segment_at(s) + 1):
            if plan.announced[j - 1] and t >= changes[j - 1] - plan.lead[j - 1]:
                k, t_eval = j, s
        outage_ok = t_eval <= known
        if not outage_ok and k in plan.outage_announced:
            announced, lead = plan.outage_announced[k]
            o_start = truth.segments[k].outage[0]
            outage_ok = announced and t >= o_start - lead
    q, loss, floor = truth.value(k, t_eval, with_outage=outage_ok)
    q = floor + (q - floor) * math.exp(plan.sigma * plan.z_delay[k])
    loss = min(1.0, loss * math.exp(plan.sigma * plan.z_loss[k]))
    p50 = q[QUANTILES.index(0.50)]
    q = np.maximum(0.0, p50 + plan.spread * (q - p50))
    return q, loss


def _outage_seen(truth: LinkTruth, plan: _Plan, t: int, known: int) -> bool:
    w = int(OUTAGE_WINDOW_S * truth.tps)
    for a, b, k in truth.outages():
        if not (a < t + w and b > t):
            continue
        announced, lead = plan.outage_announced.get(k, (False, 0))
        if a <= known or (announced and t >= a - lead):
            return True
    return False


def _persistence(t_issue: np.ndarray, obs: Optional[Observations]) -> ForecastStream:
    """Today's observations, held: no look-ahead. Downlink delays from the frames that arrived in the
    last 2 s (send time as the headset estimated it); loss from gaps in their sequence; the uplink
    from keepalive round trips minus the downlink delay of the frame that echoed each."""
    out = ForecastStream.empty("persistence", t_issue)
    if obs is None or len(t_issue) == 0:
        return out
    tps = obs.tps
    pct = [100 * q for q in QUANTILES]
    ok = obs.frame_send_est >= 0
    arr, send, idx = obs.frame_arrival[ok], obs.frame_send_est[ok], obs.frame_index[ok]
    order = np.argsort(arr, kind="stable")
    arr, send, idx = arr[order], send[order], idx[order]
    delay_ms = (arr - send) / tps * 1e3
    rtt_ok = obs.rtt_down_ticks >= 0
    up_t = obs.rtt_t[rtt_ok]
    up_ms = np.maximum(0.0, (obs.rtt_ticks[rtt_ok] - obs.rtt_down_ticks[rtt_ok]) / tps * 1e3)
    two, ten = int(2 * tps), int(10 * tps)
    for i, t in enumerate(t_issue):
        t = int(t)
        a, b = np.searchsorted(arr, t - two, side="left"), np.searchsorted(arr, t, side="right")
        if b - a >= 5:
            recent = delay_ms[a:b]
            q_now = np.percentile(recent, pct)
            seqs = idx[a:b]
            expected = int(seqs.max() - seqs.min() + 1)
            loss = 1.0 - (b - a) / expected if expected > 0 else 0.0
            a10 = np.searchsorted(arr, t - ten, side="left")
            floor = float(delay_ms[a10:b].min())
            out.valid_q[i, DOWN] = out.valid_loss[i, DOWN] = out.valid_outage[i, DOWN] = True
            out.floor_ms[i, DOWN] = floor
            silent = (t - int(arr[b - 1])) / tps
            out.outage[i, DOWN] = 1.0 if silent >= OUTAGE_MIN_S else 0.0
            for hi, h_ms in enumerate(HORIZONS_MS):
                q = q_now
                if h_ms < 0:
                    # Frames sent then: what those that arrived took; if none has arrived, they are
                    # still in flight, so the delay is at least how long ago they were sent.
                    s = t + int(h_ms / 1000.0 * tps)
                    near = (send[a:b] >= s - int(0.1 * tps)) & (send[a:b] < s + int(0.1 * tps))
                    if near.any():
                        q = np.percentile(recent[near], pct)
                    else:
                        q = np.maximum(q_now, -h_ms)
                out.q_ms[i, DOWN, hi] = q
                out.loss[i, DOWN, hi] = loss
        ua, ub = np.searchsorted(up_t, t - ten, side="left"), np.searchsorted(up_t, t, side="right")
        if ub - ua >= 2:
            last = up_ms[max(ua, ub - 5):ub]
            q_up = np.percentile(last, pct)
            out.valid_q[i, UP] = True
            out.floor_ms[i, UP] = float(up_ms[ua:ub].min())
            out.q_ms[i, UP, :] = q_up
    return out


def _rtt_half(t_issue: np.ndarray, obs: Optional[Observations]) -> ForecastStream:
    """Today's estimate, as a forecast: ClockSync's smoothed RTT/2 for both directions, every horizon,
    no spread, no loss and no outage information."""
    out = ForecastStream.empty("rtt-half", t_issue)
    if obs is None or len(t_issue) == 0:
        return out
    for i, t in enumerate(t_issue):
        k = int(np.searchsorted(obs.sync_t, int(t), side="right")) - 1
        if k < 0 or not obs.sync_synced[k]:
            continue
        half = obs.sync_uncertainty[k] / obs.tps * 1e3
        for d in (UP, DOWN):
            out.valid_q[i, d] = True
            out.q_ms[i, d, :, :] = half
            out.floor_ms[i, d] = obs.sync_min_rtt[k] / 2.0 / obs.tps * 1e3
    return out


def effective_times(cmd_send: np.ndarray, cmd_eff: np.ndarray, forecasts: ForecastStream, tps: int) -> np.ndarray:
    """ADR 0021 decision 11: with a forecast, a command takes effect at send + the forecast's p50
    uplink delay at h = 0 (the newest forecast issued by then); without one, as today (``cmd_eff``).
    Never before the send, and never earlier than the command before it, which the predictor would
    reject (a FIFO path cannot deliver a later datagram first)."""
    out = np.array(cmd_eff, dtype=np.int64)
    if len(forecasts) == 0:
        return out
    k = np.searchsorted(forecasts.t, cmd_send, side="right") - 1
    p50 = QUANTILES.index(0.50)
    for i in range(len(out)):
        j = int(k[i])
        if j >= 0 and forecasts.valid_q[j, UP]:
            out[i] = int(cmd_send[i]) + int(round(float(forecasts.q_ms[j, UP, H0, p50]) / 1e3 * tps))
        out[i] = max(out[i], int(cmd_send[i]))
        if i > 0:
            out[i] = max(out[i], out[i - 1])
    return out


def draw_level(rng: np.random.Generator, mixture: Dict[str, float] = TRAINING_MIXTURE) -> str:
    names = sorted(mixture)
    p = np.array([mixture[n] for n in names], dtype=np.float64)
    return str(names[int(rng.choice(len(names), p=p / p.sum()))])
