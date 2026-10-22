"""Training samples from a view: a window at a query time, labelled at a driver read instant.

Labels exist only where the truth is exact: at a driver read of the joint being labelled (ADR 0021:
"errors are scored only at logged read instants"). So a query is (t_now, T, joint): the window is
built from every call made up to ``t_now``, for the target ``T`` = a read instant of ``joint``,
with ``T - t_now`` up to ``delta_max_ms`` (the display looks slightly ahead).

Three kinds of query, mixed by ``SampleConfig``:
- **base**: read instants drawn uniformly over the view;
- **event**: read instants within 3 s after an event (a network switch, a slider change, a hand on or
  off, a payload, an uplink change), so adaptation is not drowned by steady state;
- **pairs**: the same read instant asked just before and just after a fresh reading arrives, for the
  consistency term that trades error against the jump the operator would see.

A view's samples are built at one network forecast level (``armpred.net.forecast``): the forecasts
reach the window as calls of their own, and with a forecast the commands' effective times are the
forecast's (ADR 0021 decisions 11 and 12). ``absent`` is the headset as it is today.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..baselines.types import JointPredictorConfig, JointSample, headset_joint_config
from ..features.layout import ARM_MOTORS, FeatureState, arm_features, JOINT_FEATURES
from ..net import forecast as FC
from .views import View, view_observations, view_truths

EVENT_KINDS = ("network", "uplink", "slider", "hand-on", "hand-off", "payload", "mark")
CALL_COMMAND, CALL_STATE, CALL_FORECAST = 0, 1, 2


@dataclass(frozen=True)
class SampleConfig:
    queries_per_s: float = 20.0
    event_share: float = 0.25
    event_window_s: float = 3.0
    pair_share: float = 0.2
    delta_max_ms: float = 60.0
    warmup_s: float = 2.0


@dataclass
class Samples:
    joints: np.ndarray  # float32 [N, JOINT_FEATURES] the labelled joint's row
    arm: np.ndarray  # float32 [N, ARM_FEATURES(+1)]
    joint: np.ndarray  # int8 [N] index into ARM_MOTORS
    truth: np.ndarray  # float64 [N] rad
    prior: np.ndarray  # float64 [N] the prior's prediction for that joint
    newest: np.ndarray  # float64 [N] that joint's newest fresh reading (hold-last)
    valid: np.ndarray  # bool [N]: False -> the predictor would fall back
    pair: np.ndarray  # int32 [N]: pair id, -1 if unpaired; a pair is (before, after)
    since_event_s: np.ndarray  # float32 [N]: time since the newest event before T (inf if none)
    staleness_s: np.ndarray  # float32 [N]
    kind: np.ndarray  # int8 [N]: 0 base, 1 event, 2 pair
    level: np.ndarray  # int8 [N]: the forecast level, an index into forecast.LEVELS

    def __len__(self) -> int:
        return int(self.truth.shape[0])

    @staticmethod
    def concat(parts: List["Samples"]) -> "Samples":
        fields = Samples.__dataclass_fields__
        return Samples(**{name: np.concatenate([getattr(p, name) for p in parts]) for name in fields})


def call_stream(view: View, forecasts: Optional[FC.ForecastStream] = None) -> List[Tuple[int, int, int]]:
    """Every predictor call in order: (time, CALL_*, index). A command and a state at the same tick:
    the command first (VIEW_FORMAT.md); a forecast issued at that tick comes after both."""
    calls = [(int(t), CALL_COMMAND, i) for i, t in enumerate(view.cmd_t)] + \
            [(int(t), CALL_STATE, i) for i, t in enumerate(view.st_t)]
    if forecasts is not None and forecasts.level != "absent":
        calls += [(int(t), CALL_FORECAST, i) for i, t in enumerate(forecasts.t)]
    calls.sort()
    return calls


def view_forecasts(view: View, level: str, seed: int) -> Tuple[Optional[FC.ForecastStream], np.ndarray]:
    """The forecast stream at ``level`` for a view, and the effective times the host would then pass.
    ``absent`` is None and the view's own effective times (RTT/2, as the headset does today)."""
    if level == "absent":
        return None, view.cmd_eff
    first = min(int(view.cmd_t[0]) if len(view.cmd_t) else 0, int(view.st_t[0]) if len(view.st_t) else 0)
    last = max(int(view.cmd_t[-1]) if len(view.cmd_t) else 0, int(view.st_t[-1]) if len(view.st_t) else 0)
    truths = view_truths(view) if level in FC.DEGRADED else None
    stream = FC.make_forecasts(level, FC.issue_times(first, last, view.tps), truths, view_observations(view), seed)
    return stream, FC.effective_times(view.cmd_t, view.cmd_eff, stream, view.tps)


def command_samples(view: View, i: int) -> List[JointSample]:
    n = int(view.cmd_count[i])
    return [JointSample(int(view.cmd_motor[i, k]), float(view.cmd_value[i, k]), False) for k in range(n)]


def state_samples(view: View, i: int) -> List[JointSample]:
    n = int(view.st_count[i])
    return [JointSample(int(view.st_motor[i, k]), float(view.st_value[i, k]), bool(view.st_flags[i, k] & 1)) for k in range(n)]


def _queries(view: View, rng: np.random.Generator, cfg: SampleConfig) -> List[Tuple[int, int, int, float, int, int]]:
    """(t_now, T, joint index, truth, pair id, kind), sorted by t_now."""
    tps = view.tps
    arm = np.isin(view.tr_motor, ARM_MOTORS)
    first_state = int(view.st_t[0]) if len(view.st_t) else None
    if first_state is None:
        return []
    usable = arm & (view.tr_t >= first_state + int(cfg.warmup_s * tps))
    idx = np.nonzero(usable)[0]
    if idx.size == 0:
        return []
    duration_s = (int(view.tr_t[idx[-1]]) - int(view.tr_t[idx[0]])) / tps
    n_total = int(cfg.queries_per_s * duration_s)
    n_event = int(n_total * cfg.event_share)
    n_pair = int(n_total * cfg.pair_share) // 2
    n_base = max(0, n_total - n_event - 2 * n_pair)
    motor_index = {m: k for k, m in enumerate(ARM_MOTORS)}
    delta = lambda: int(rng.uniform(0.0, cfg.delta_max_ms) / 1000.0 * tps)  # noqa: E731
    out = []
    for r in rng.choice(idx, size=min(n_base, idx.size), replace=False):
        t = int(view.tr_t[r])
        out.append((t - delta(), t, motor_index[int(view.tr_motor[r])], float(view.tr_value[r]), -1, 0))
    event_t = np.array([t for t, k in zip(view.events_t, view.events_kind) if k in EVENT_KINDS], dtype=np.int64)
    if event_t.size and n_event:
        near = np.zeros(idx.size, dtype=bool)
        tr = view.tr_t[idx]
        for e in event_t:
            near |= (tr >= e) & (tr < e + int(cfg.event_window_s * tps))
        cand = idx[near]
        if cand.size:
            for r in rng.choice(cand, size=min(n_event, cand.size), replace=False):
                t = int(view.tr_t[r])
                out.append((t - delta(), t, motor_index[int(view.tr_motor[r])], float(view.tr_value[r]), -1, 1))
    # Pairs: a read instant within 100 ms after a fresh reading's arrival, asked just before and after it.
    fresh = _fresh_state_times(view)
    if fresh.size and n_pair:
        tr = view.tr_t[idx]
        for p in range(n_pair):
            a = int(rng.choice(fresh))
            cand = idx[(tr >= a) & (tr < a + int(0.1 * tps))]
            if cand.size == 0:
                continue
            r = int(rng.choice(cand))
            t = int(view.tr_t[r])
            j = motor_index[int(view.tr_motor[r])]
            out.append((a - 1, t, j, float(view.tr_value[r]), p, 2))
            out.append((a, t, j, float(view.tr_value[r]), p, 2))
    out.sort(key=lambda q: (q[0], q[4], q[1]))
    return out


def _fresh_state_times(view: View) -> np.ndarray:
    """The ticks at which a frame with a new sensed stamp reached the predictor."""
    times, newest = [], None
    for t, s in zip(view.st_t, view.st_sensed):
        if newest is None or s > newest:
            newest = int(s)
            times.append(int(t))
    return np.array(times, dtype=np.int64)


def view_samples(view: View, rng: np.random.Generator, cfg: SampleConfig = SampleConfig(),
                 prior_config: Optional[JointPredictorConfig] = None, aware: bool = False,
                 level: str = "absent", forecast_seed: int = 0) -> Samples:
    prior_config = prior_config or headset_joint_config(view.tps)
    queries = _queries(view, rng, cfg)
    n = len(queries)
    out = Samples(
        joints=np.zeros((n, JOINT_FEATURES), dtype=np.float32),
        arm=np.zeros((n, arm_features(aware)), dtype=np.float32),
        joint=np.zeros(n, dtype=np.int8),
        truth=np.zeros(n), prior=np.zeros(n), newest=np.zeros(n),
        valid=np.zeros(n, dtype=bool), pair=np.full(n, -1, dtype=np.int32),
        since_event_s=np.full(n, np.inf, dtype=np.float32), staleness_s=np.zeros(n, dtype=np.float32),
        kind=np.zeros(n, dtype=np.int8), level=np.full(n, FC.LEVELS.index(level), dtype=np.int8),
    )
    if n == 0:
        return out
    forecasts, eff = view_forecasts(view, level, forecast_seed)
    state = FeatureState(prior_config, aware=aware)
    calls = call_stream(view, forecasts)
    ci = 0
    event_t = np.array([t for t, k in zip(view.events_t, view.events_kind) if k in EVENT_KINDS], dtype=np.int64)
    for qi, (t_now, target, j, truth, pair, kind) in enumerate(queries):
        while ci < len(calls) and calls[ci][0] <= t_now:
            _, what, i = calls[ci]
            if what == CALL_COMMAND:
                state.observe_command(int(eff[i]), command_samples(view, i), float(view.cmd_speed[i]))
            elif what == CALL_STATE:
                state.observe_state(int(view.st_sensed[i]), state_samples(view, i))
            else:
                state.observe_forecast(forecasts, i)
            ci += 1
        w = state.window(target)
        out.joints[qi] = w.joints[j]
        out.arm[qi] = w.arm
        out.joint[qi] = j
        out.truth[qi] = truth
        out.prior[qi] = w.prior[j]
        out.newest[qi] = w.newest[j]
        out.valid[qi] = w.valid
        out.pair[qi] = pair
        out.staleness_s[qi] = w.staleness_s
        out.kind[qi] = kind
        before = event_t[event_t <= target]
        if before.size:
            out.since_event_s[qi] = (target - int(before[-1])) / view.tps
    return out
