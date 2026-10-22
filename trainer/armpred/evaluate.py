"""The trainer's own evaluation, and the trainer gate (ADR 0021 decision 9).

Every predictor receives the same calls a view records, and is asked:
- at **truth read instants** (exact labels): ``joint_prediction_error_deg``;
- just before and just after each **fresh reading** reaches it: ``joint_correction_deg``;
- at every **90 Hz display tick** over a stretch of each view: ``joint_display_jitter_deg``;
- around **events**, aligned on t_info (the first moment data could show the change):
  ``joint_event_error_deg`` and ``joint_recovery_time_ms``;
- over the display ticks' **motion segments**: ``joint_prediction_lead_ms``, how far ahead of (or
  behind) the arm the prediction runs.

A network forecast (ADR 0021 decisions 11 and 12) reaches only the predictors that take one
(``uses_forecast``), with the effective times the host would then pass; everything else sees the view
as the headset does today. The forecast-quality sweep runs the forecast-taking predictors once per
level, named ``<predictor>@<level>``.

This decides whether the repo's C# consumer gets built. It is not a result: a citable number comes
only from ``Teleop.Eval`` scoring the exported model on views (decision 8).
"""

from __future__ import annotations

import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .baselines.adaptive import RelayReplayAdaptive
from .baselines.hold_last import HoldLastJointPredictor
from .baselines.relay_replay import RelayReplayPredictor
from .baselines.relay_replay_servo import RelayReplayServoPredictor
from .baselines.types import JointPredictorConfig, JointSample, headset_joint_config
from .data.dataset import CALL_COMMAND, CALL_FORECAST, CALL_STATE, EVENT_KINDS, call_stream, command_samples, state_samples
from .data.views import View, view_truths
from .features import layout as L
from .net import forecast as FC

DEG = 180.0 / math.pi
ARM = L.ARM_MOTORS
EVENT_WINDOWS_MS = ((-2000, 0), (0, 100), (100, 250), (250, 500), (500, 1000), (1000, 2000))
UPLINK_CHANGE = "uplink-change"  # an uplink event that moved the one-way delay by at least:
UPLINK_CHANGE_MS = 50.0  # ADR 0021 decision 14
# joint_prediction_lead_ms (docs/metrics.md section 4)
LEAD_SPEED_RAD_S = 0.3
LEAD_MIN_SEGMENT_S = 0.5
LEAD_MAX_MS = 300
LEAD_STEP_MS = 5


class Predictor:
    name = "?"
    uses_forecast = False

    def reset(self) -> None: ...
    def observe_command(self, eff: int, targets: Sequence[JointSample], speed: float) -> None: ...
    def observe_state(self, sensed: int, joints: Sequence[JointSample]) -> None: ...
    def observe_forecast(self, stream: FC.ForecastStream, index: int) -> None: ...

    def predict(self, target: int) -> Tuple[Optional[np.ndarray], bool]:
        """Arm joints in ARM order, and whether this was a fallback."""
        raise NotImplementedError


class Baseline(Predictor):
    """hold-last or relay-replay, speed-aware (as the headset runs it) or blind (the host passes 0)."""

    def __init__(self, name: str, impl, blind: bool, uses_forecast: bool = False):
        self.name, self.impl, self.blind = name, impl, blind
        # A forecast-fed baseline sees only the forecast's effective times; it has no window to put
        # the forecast itself in.
        self.uses_forecast = uses_forecast

    def reset(self) -> None:
        self.impl.reset()

    def observe_command(self, eff, targets, speed):
        self.impl.observe_command(eff, targets, 0.0 if self.blind else speed)

    def observe_state(self, sensed, joints):
        self.impl.observe_state(sensed, joints)

    def predict(self, target):
        ok, out = self.impl.try_predict(target)
        if not ok:
            return None, False
        by = {s.motor_id: s.value for s in out}
        if any(m not in by for m in ARM):
            return None, False
        return np.array([by[m] for m in ARM], dtype=np.float64), False


class ModelPredictor(Predictor):
    """seq-model as the C# predictor will run it: the window, the exported graph, the fallback."""

    uses_forecast = True

    def __init__(self, onnx_path: str, prior_config: JointPredictorConfig, aware: bool, name: str = "seq-model"):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
        self.session = ort.InferenceSession(onnx_path, options, providers=["CPUExecutionProvider"])
        self.state = L.FeatureState(prior_config, aware=aware)
        self.name = name
        self.fallbacks = 0
        self.calls = 0
        self.inference_s: List[float] = []

    def reset(self):
        self.state.reset()

    def observe_command(self, eff, targets, speed):
        self.state.observe_command(eff, targets, speed)

    def observe_state(self, sensed, joints):
        self.state.observe_state(sensed, joints)

    def observe_forecast(self, stream, index):
        self.state.observe_forecast(stream, index)

    def predict(self, target):
        self.calls += 1
        w = self.state.window(target)
        if not w.valid:
            self.fallbacks += 1
            if not np.any(w.prior):
                return None, True
            return w.prior.copy(), True
        x = np.concatenate([w.joints.reshape(-1), w.arm]).astype(np.float32).reshape(1, -1)
        t0 = time.perf_counter()
        y = self.session.run(["y"], {"x": x})[0].reshape(-1)
        self.inference_s.append(time.perf_counter() - t0)
        if not np.all(np.isfinite(y)):
            self.fallbacks += 1
            return w.prior.copy(), True
        return w.prior + y[0::2].astype(np.float64), False


@dataclass
class ViewResult:
    view: str
    schedule: str
    conditions: dict
    tps: int = 1_000_000_000
    # per predictor: arrays of (time, joint index, error deg)
    errors: Dict[str, np.ndarray] = field(default_factory=dict)
    corrections: Dict[str, np.ndarray] = field(default_factory=dict)
    jitter: Dict[str, np.ndarray] = field(default_factory=dict)
    fallbacks: Dict[str, Tuple[int, int]] = field(default_factory=dict)
    inference_us: Dict[str, np.ndarray] = field(default_factory=dict)
    events: List[Tuple[int, str, int]] = field(default_factory=list)  # (t_event, kind, t_info)
    lead_ms: Dict[str, np.ndarray] = field(default_factory=dict)  # per motion segment

    def merge(self, other: "ViewResult") -> None:
        """Adds another run's predictors over the same view (the forecast sweep)."""
        for name in ("errors", "corrections", "jitter", "fallbacks", "inference_us", "lead_ms"):
            getattr(self, name).update(getattr(other, name))


def _uplink_change_ms(view: View, t_event: int) -> float:
    """How far the uplink's one-way delay (p50) moved across an event: 0.5 s after against 0.5 s before."""
    try:
        up, _ = view_truths(view)
    except (ValueError, KeyError):
        return 0.0
    half = int(0.5 * view.tps)
    p50 = FC.QUANTILES.index(0.50)
    after = up.value(up.segment_at(t_event + half), t_event + half)[0][p50]
    before = up.value(up.segment_at(t_event - half), t_event - half)[0][p50]
    return float(abs(after - before))


def _t_info(view: View) -> List[Tuple[int, str, int]]:
    """t_info per event: a downlink switch shows up at once; anything the robot does (load, hand,
    slider, uplink) shows up in the first frame sensed after it, when that frame reaches the headset.
    An uplink event that moved the delay by at least ``UPLINK_CHANGE_MS`` is also an ``uplink-change``."""
    out = []
    fresh_t, fresh_s, newest = [], [], None
    for t, s in zip(view.st_t, view.st_sensed):
        if newest is None or s > newest:
            newest = int(s)
            fresh_t.append(int(t))
            fresh_s.append(int(s))
    fresh_t, fresh_s = np.array(fresh_t), np.array(fresh_s)
    for t, kind in zip(view.events_t, view.events_kind):
        if kind not in EVENT_KINDS:
            continue
        if kind == "network":
            out.append((int(t), kind, int(t)))
            continue
        k = np.searchsorted(fresh_s, int(t), side="left") if fresh_s.size else 0
        if k < fresh_s.size:
            out.append((int(t), kind, int(fresh_t[k])))
            if kind == "uplink" and _uplink_change_ms(view, int(t)) >= UPLINK_CHANGE_MS:
                out.append((int(t), UPLINK_CHANGE, int(fresh_t[k])))
    return out


def lead_times_ms(view: View, ticks: np.ndarray, predictions: np.ndarray) -> np.ndarray:
    """``joint_prediction_lead_ms`` for one predictor: per motion segment of each arm joint (the true
    speed above ``LEAD_SPEED_RAD_S`` for at least ``LEAD_MIN_SEGMENT_S``, on the display ticks), the
    shift tau in [-LEAD_MAX_MS, LEAD_MAX_MS] that minimises the median |prediction(t) - truth(t + tau)|.
    Positive: the prediction shows where the arm will be (it runs ahead). The truth between reads is
    linear between them."""
    tps = view.tps
    out = []
    shifts = np.arange(-LEAD_MAX_MS, LEAD_MAX_MS + 1, LEAD_STEP_MS)
    for j, m in enumerate(ARM):
        sel = view.tr_motor == m
        tt, tv = view.tr_t[sel].astype(np.float64), view.tr_value[sel]
        if tt.size < 4:
            continue
        truth = np.interp(ticks.astype(np.float64), tt, tv, left=np.nan, right=np.nan)
        speed = np.abs(np.gradient(truth, ticks.astype(np.float64) / tps)) if ticks.size > 2 else np.zeros(ticks.size)
        moving = np.isfinite(speed) & (speed > LEAD_SPEED_RAD_S) & np.isfinite(predictions[:, j])
        # Contiguous runs of motion, long enough to align.
        edges = np.flatnonzero(np.diff(np.concatenate([[0], moving.astype(np.int8), [0]])))
        for a, b in zip(edges[0::2], edges[1::2]):
            if (ticks[b - 1] - ticks[a]) / tps < LEAD_MIN_SEGMENT_S:
                continue
            seg_t = ticks[a:b].astype(np.float64)
            pred = predictions[a:b, j]
            best, best_err = None, np.inf
            for tau in shifts:
                shifted = np.interp(seg_t + tau / 1000.0 * tps, tt, tv, left=np.nan, right=np.nan)
                ok = np.isfinite(shifted)
                if ok.sum() < 0.8 * ok.size:
                    continue
                err = float(np.median(np.abs(pred[ok] - shifted[ok])))
                if err < best_err:
                    best, best_err = float(tau), err
            if best is not None:
                out.append(best)
    return np.array(out, dtype=np.float64)


def evaluate_view(view: View, predictors: List[Predictor], reads_per_s: float = 20.0, jitter_s: float = 20.0,
                  warmup_s: float = 2.0, seed: int = 0, forecasts: Optional[FC.ForecastStream] = None,
                  eff_forecast: Optional[np.ndarray] = None) -> ViewResult:
    rng = np.random.default_rng(seed)
    tps = view.tps
    for p in predictors:
        p.reset()
    calls = call_stream(view, forecasts)
    takers = [p for p in predictors if p.uses_forecast]
    eff_of = {p.name: (eff_forecast if (p.uses_forecast and eff_forecast is not None) else view.cmd_eff) for p in predictors}
    first_state = int(view.st_t[0]) if len(view.st_t) else 0
    arm_reads = np.nonzero(np.isin(view.tr_motor, ARM) & (view.tr_t >= first_state + int(warmup_s * tps)))[0]
    duration = (int(view.tr_t[arm_reads[-1]]) - int(view.tr_t[arm_reads[0]])) / tps if arm_reads.size else 0.0
    keep = min(arm_reads.size, int(reads_per_s * duration))
    chosen = np.sort(rng.choice(arm_reads, size=keep, replace=False)) if keep else np.zeros(0, dtype=np.int64)
    # Fresh-state calls: where the correction is measured.
    fresh_calls = set()
    newest = None
    for idx, (t, what, i) in enumerate(calls):
        if what == CALL_STATE:
            s = int(view.st_sensed[i])
            if newest is None or s > newest:
                newest = s
                fresh_calls.add(idx)
    jitter_from = first_state + int(warmup_s * tps)
    jitter_ticks = jitter_from + np.round(np.arange(int(jitter_s * 90)) * tps / 90.0).astype(np.int64)
    queries = [(int(view.tr_t[r]), 0, r) for r in chosen] + [(int(t), 1, k) for k, t in enumerate(jitter_ticks)]
    queries.sort()
    res = ViewResult(view=view.meta["session"], schedule=view.meta["schedule"], conditions=view.meta.get("conditions", {}),
                     tps=tps)
    err = {p.name: [] for p in predictors}
    corr = {p.name: [] for p in predictors}
    disp = {p.name: [] for p in predictors}
    motor_index = {m: k for k, m in enumerate(ARM)}
    ci = 0
    qi = 0
    n_calls = len(calls)
    while qi < len(queries) or ci < n_calls:
        next_call_t = calls[ci][0] if ci < n_calls else None
        next_q_t = queries[qi][0] if qi < len(queries) else None
        if next_call_t is not None and (next_q_t is None or next_call_t <= next_q_t):
            t, what, i = calls[ci]
            if what == CALL_COMMAND:
                targets = command_samples(view, i)
                for p in predictors:
                    p.observe_command(int(eff_of[p.name][i]), targets, float(view.cmd_speed[i]))
            elif what == CALL_FORECAST:
                for p in takers:
                    p.observe_forecast(forecasts, i)
            else:
                joints = state_samples(view, i)
                fresh = ci in fresh_calls
                before = [p.predict(t)[0] for p in predictors] if fresh else None
                for p in predictors:
                    p.observe_state(int(view.st_sensed[i]), joints)
                if fresh:
                    for p, b in zip(predictors, before):
                        a, _ = p.predict(t)
                        if a is not None and b is not None:
                            corr[p.name].append(np.abs(a - b) * DEG)
            ci += 1
            continue
        t, kind, k = queries[qi]
        qi += 1
        if kind == 0:
            j = motor_index[int(view.tr_motor[k])]
            truth = float(view.tr_value[k])
            for p in predictors:
                pred, _ = p.predict(t)
                if pred is not None:
                    err[p.name].append((t, j, abs(pred[j] - truth) * DEG))
        else:
            for p in predictors:
                pred, _ = p.predict(t)
                disp[p.name].append(pred if pred is not None else np.full(len(ARM), np.nan))
    for p in predictors:
        res.errors[p.name] = np.array(err[p.name], dtype=np.float64).reshape(-1, 3)
        res.corrections[p.name] = np.concatenate(corr[p.name]) if corr[p.name] else np.zeros(0)
        d = np.array(disp[p.name], dtype=np.float64)
        res.jitter[p.name] = np.abs(d[2:] - 2 * d[1:-1] + d[:-2]).reshape(-1) * DEG if len(d) > 2 else np.zeros(0)
        res.lead_ms[p.name] = lead_times_ms(view, jitter_ticks, d.reshape(-1, len(ARM))) if len(d) else np.zeros(0)
        if isinstance(p, ModelPredictor):
            res.fallbacks[p.name] = (p.fallbacks, p.calls)
            res.inference_us[p.name] = np.array(p.inference_s) * 1e6
    res.events = _t_info(view)
    return res


def percentiles(e: np.ndarray) -> Dict[str, float]:
    e = np.asarray(e, dtype=np.float64)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return {"p50": float("nan"), "p95": float("nan"), "p99": float("nan"), "max": float("nan"), "n": 0}
    return {"p50": float(np.percentile(e, 50)), "p95": float(np.percentile(e, 95)), "p99": float(np.percentile(e, 99)),
            "max": float(e.max()), "n": int(e.size)}


def event_windows(results: Sequence[ViewResult], name: str, kinds: Sequence[str]) -> Dict[str, Dict[str, float]]:
    """``joint_event_error_deg``: errors binned by target time relative to t_info, with the
    information-limited stretch [t_event, t_info) on its own."""
    bins: Dict[str, List[float]] = {}
    for r in results:
        e = r.errors.get(name)
        if e is None or e.size == 0:
            continue
        tps = float(r.tps)
        for t_event, kind, t_info in r.events:
            if kind not in kinds:
                continue
            rel_info_ms = (e[:, 0] - t_info) / tps * 1000.0
            rel_event_ms = (e[:, 0] - t_event) / tps * 1000.0
            limited = (rel_event_ms >= 0) & (rel_info_ms < 0)
            bins.setdefault("information-limited", []).extend(e[limited, 2].tolist())
            for lo, hi in EVENT_WINDOWS_MS:
                if lo < 0:
                    sel = (rel_event_ms >= lo) & (rel_event_ms < hi)
                else:
                    sel = (rel_info_ms >= lo) & (rel_info_ms < hi)
                bins.setdefault(f"{lo}..{hi} ms", []).extend(e[sel, 2].tolist())
    return {k: percentiles(np.array(v)) for k, v in bins.items()}


def recovery_times_ms(results: Sequence[ViewResult], name: str, kinds: Sequence[str], threshold_deg: float = 1.5,
                      hold_ms: float = 250.0, cap_ms: float = 3000.0) -> np.ndarray:
    """``joint_recovery_time_ms``: from t_info until the 100 ms trailing median error stays below the
    threshold for ``hold_ms``; ``cap_ms`` when it never does within the cap."""
    out = []
    for r in results:
        e = r.errors.get(name)
        if e is None or e.size == 0:
            continue
        for _, kind, t_info in r.events:
            if kind not in kinds:
                continue
            rel = (e[:, 0] - t_info) / r.tps * 1000.0
            sel = (rel >= 0) & (rel <= cap_ms + 100.0)
            if not np.any(sel):
                continue
            t = rel[sel]
            v = e[sel, 2]
            order = np.argsort(t)
            t, v = t[order], v[order]
            below_since = None
            found = cap_ms
            for k in range(len(t)):
                window = v[(t > t[k] - 100.0) & (t <= t[k])]
                if np.median(window) < threshold_deg:
                    if below_since is None:
                        below_since = t[k]
                    if t[k] - below_since >= hold_ms:
                        found = below_since
                        break
                else:
                    below_since = None
            out.append(min(found, cap_ms))
    return np.array(out)


def condition_key(conditions: dict) -> str:
    """A session's cell: slider and load, from session.json (never from the model's inputs)."""
    slider = conditions.get("speedSliderRadPerSec", "?")
    load = conditions.get("load", {})
    kind = load.get("kind", "?")
    grams = load.get("payloadGrams", 0) or 0
    uplink = conditions.get("network", {}).get("uplinkProfile", "lan")
    label = f"slider={slider} load={kind}" + (f"{grams:g}g" if kind == "payload" else "")
    return label + (f" uplink={uplink}" if uplink not in ("lan", "none") else "")


def _names(results: Sequence[ViewResult]) -> List[str]:
    """Every predictor any view has, in first-seen order (the sweep's run only on some schedules)."""
    out: List[str] = []
    for r in results:
        for n in r.errors:
            if n not in out:
                out.append(n)
    return out


def _cat(arrays: List[np.ndarray]) -> np.ndarray:
    return np.concatenate(arrays) if arrays else np.zeros(0)


def window_errors(results: Sequence[ViewResult], name: str, kind: str, after_info_ms: float = 1000.0) -> Dict[str, np.ndarray]:
    """Per view: the errors of ``name`` with target time in [t_event, t_info + after_info_ms) after each
    event of ``kind`` -- the information-limited stretch and what follows it (ADR 0021 decision 14)."""
    out: Dict[str, List[np.ndarray]] = {}
    for r in results:
        e = r.errors.get(name)
        if e is None or e.size == 0:
            continue
        for t_event, k, t_info in r.events:
            if k != kind:
                continue
            sel = (e[:, 0] >= t_event) & (e[:, 0] < t_info + after_info_ms / 1000.0 * r.tps)
            out.setdefault(r.view, []).append(e[sel, 2])
    return {v: _cat(a) for v, a in out.items()}


def is_live_impaired(conditions: dict) -> bool:
    return conditions.get("network", {}).get("uplinkProfile", "lan") not in ("lan", "none")


def summarize(results: Sequence[ViewResult]) -> Dict[str, object]:
    names = _names(results)

    def stats(rs: Sequence[ViewResult], n: str) -> Dict[str, Dict[str, float]]:
        rs = [r for r in rs if n in r.errors]
        return {
            "joint_prediction_error_deg": percentiles(_cat([r.errors[n][:, 2] for r in rs])),
            "joint_correction_deg": percentiles(_cat([r.corrections[n] for r in rs])),
            "joint_display_jitter_deg": percentiles(_cat([r.jitter[n] for r in rs])),
        }

    by_schedule: Dict[str, Dict[str, object]] = {}
    for sched in sorted({r.schedule for r in results}):
        rs = [r for r in results if r.schedule == sched]
        by_schedule[sched] = {n: stats(rs, n) for n in names if any(n in r.errors for r in rs)}
    overall = {}
    for n in names:
        o = stats(results, n)
        o["joint_event_error_deg"] = {k: event_windows(results, n, [k]) for k in ("network", "hand-on", "slider", "payload", UPLINK_CHANGE)}
        o["joint_recovery_time_ms"] = {k: percentiles(recovery_times_ms(results, n, [k])) for k in ("network", "hand-on", "slider", UPLINK_CHANGE)}
        o["joint_prediction_lead_ms"] = percentiles_signed(_cat([r.lead_ms.get(n, np.zeros(0)) for r in results]))
        o["uplink_change_window_deg"] = percentiles(_cat(list(window_errors(results, n, UPLINK_CHANGE).values())))
        overall[n] = o
    fallback = {}
    inference = {}
    for r in results:
        for n, (f, c) in r.fallbacks.items():
            a, b = fallback.get(n, (0, 0))
            fallback[n] = (a + f, b + c)
        for n, us in r.inference_us.items():
            inference.setdefault(n, []).append(us)
    by_condition: Dict[str, Dict[str, object]] = {}
    for cond in sorted({condition_key(r.conditions) for r in results}):
        rs = [r for r in results if condition_key(r.conditions) == cond]
        by_condition[cond] = {n: {"joint_prediction_error_deg": percentiles(_cat([r.errors[n][:, 2] for r in rs if n in r.errors]))}
                              for n in names if any(n in r.errors for r in rs)}
    live = [r for r in results if is_live_impaired(r.conditions)]
    return {
        "views": len(results),
        "by_schedule": by_schedule,
        "by_condition": by_condition,
        "live_impaired": {n: stats(live, n) for n in names if any(n in r.errors for r in live)},
        "overall": overall,
        "joint_predictor_fallback": {n: {"fallbacks": f, "calls": c, "rate": f / c if c else float("nan")} for n, (f, c) in fallback.items()},
        "joint_inference_us": {n: percentiles(np.concatenate(v)) for n, v in inference.items()},
    }


def percentiles_signed(x: np.ndarray) -> Dict[str, float]:
    """A signed quantity's spread (the lead): p5, p50, p95 and the share of segments at the search's edge."""
    x = np.asarray(x, dtype=np.float64)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"p5": float("nan"), "p50": float("nan"), "p95": float("nan"), "edge": float("nan"), "n": 0}
    return {"p5": float(np.percentile(x, 5)), "p50": float(np.percentile(x, 50)), "p95": float(np.percentile(x, 95)),
            "edge": float(np.mean(np.abs(x) >= LEAD_MAX_MS)), "n": int(x.size)}


def bootstrap_window_p95_difference(results: Sequence[ViewResult], a: str, b: str, kind: str = UPLINK_CHANGE,
                                    n: int = 2000, seed: int = 0) -> Dict[str, float]:
    """The session bootstrap of decision 9, on the errors inside ``window_errors`` windows."""
    wa, wb = window_errors(results, a, kind), window_errors(results, b, kind)
    sessions = sorted(set(wa) & set(wb))
    if not sessions:
        return {"diff": float("nan"), "lo": float("nan"), "hi": float("nan"), "sessions": 0}
    rng = np.random.default_rng(seed)

    def diff(chosen) -> float:
        ea = _cat([wa[s] for s in chosen])
        eb = _cat([wb[s] for s in chosen])
        if ea.size == 0 or eb.size == 0:
            return float("nan")
        return float(np.percentile(ea, 95) - np.percentile(eb, 95))

    point = diff(sessions)
    draws = np.array([diff(rng.choice(sessions, size=len(sessions), replace=True)) for _ in range(n)])
    draws = draws[np.isfinite(draws)]
    return {"diff": point, "lo": float(np.percentile(draws, 2.5)) if draws.size else float("nan"),
            "hi": float(np.percentile(draws, 97.5)) if draws.size else float("nan"), "sessions": len(sessions)}


def bootstrap_p95_difference(results: Sequence[ViewResult], a: str, b: str, schedule: Optional[str] = None,
                             n: int = 2000, seed: int = 0) -> Dict[str, float]:
    """Paired session bootstrap of p95(a) - p95(b): sessions are the unit, not samples (methodology
    review: queries are autocorrelated and sessions dominate the variance)."""
    rs = [r for r in results if (schedule is None or r.schedule == schedule) and a in r.errors and b in r.errors]
    sessions = sorted({r.view for r in rs})
    if not sessions:
        return {"diff": float("nan"), "lo": float("nan"), "hi": float("nan")}
    by_session = {s: ([r.errors[a][:, 2] for r in rs if r.view == s], [r.errors[b][:, 2] for r in rs if r.view == s]) for s in sessions}
    rng = np.random.default_rng(seed)

    def diff(chosen) -> float:
        ea = np.concatenate([x for s in chosen for x in by_session[s][0]])
        eb = np.concatenate([x for s in chosen for x in by_session[s][1]])
        return float(np.percentile(ea, 95) - np.percentile(eb, 95))

    point = diff(sessions)
    draws = [diff(rng.choice(sessions, size=len(sessions), replace=True)) for _ in range(n)]
    return {"diff": point, "lo": float(np.percentile(draws, 2.5)), "hi": float(np.percentile(draws, 97.5)), "sessions": len(sessions)}


def default_predictors(onnx_path: Optional[str], tps: int, prior_config: Optional[JointPredictorConfig] = None,
                       aware: bool = False, sensed_ages_ms: Optional[Sequence[float]] = None,
                       servo: Optional[Dict[str, float]] = None) -> List[Predictor]:
    """The ladder (ADR 0021): hold-last, relay-replay as the headset runs it, relay-replay blind,
    relay-replay-adaptive, relay-replay-servo (decision 15, its servo parameters the C# defaults
    until sysid fits them), and the model; with fitted reading ages (``armpred sysid``), also
    relay-replay-fitted (speed-aware, as the headset runs it, with the fitted ages), and the blind,
    adaptive and model rows use those ages too."""
    if prior_config is None and sensed_ages_ms:
        prior_config = headset_joint_config(tps, sensed_ages_ms=tuple(sensed_ages_ms))
    config = prior_config or headset_joint_config(tps)
    out: List[Predictor] = [
        Baseline("hold-last", HoldLastJointPredictor(config), blind=True),
        Baseline("relay-replay", RelayReplayPredictor(headset_joint_config(tps)), blind=False),
        Baseline("relay-replay-blind", RelayReplayPredictor(config), blind=True),
        Baseline("relay-replay-adaptive", RelayReplayAdaptive(config), blind=True),
        Baseline("relay-replay-servo", RelayReplayServoPredictor(headset_joint_config(tps)), blind=False),
    ]
    if sensed_ages_ms:
        out.append(Baseline("relay-replay-fitted", RelayReplayPredictor(config), blind=False))
    if servo:
        out.append(Baseline("relay-replay-servo-fitted", RelayReplayServoPredictor(servo_config(config, servo)), blind=False))
    if onnx_path:
        out.append(ModelPredictor(onnx_path, config, aware))
    return out


def servo_config(config: JointPredictorConfig, servo: Dict[str, float]) -> JointPredictorConfig:
    """A config with ``armpred sysid``'s fitted relay-replay-servo parameters (prior.json's
    ``relayReplayServo``); what it did not fit keeps the C# default."""
    import dataclasses

    names = {"servoLagSeconds": "servo_lag_seconds", "servoMaxSpeedRadPerSec": "servo_max_speed_rad_per_sec",
             "servoPipelineDelaySeconds": "servo_pipeline_delay_seconds"}
    return dataclasses.replace(config, **{names[k]: float(v) for k, v in servo.items() if k in names})


# The forecast-quality sweep (ADR 0021 decisions 12 and 14): every level but absent, which is the
# base run itself.
SWEEP_LEVELS: Tuple[str, ...] = tuple(level for level in FC.LEVELS if level != "absent")


def forecast_predictors(onnx_path: Optional[str], tps: int, level: str, prior_config: Optional[JointPredictorConfig] = None,
                        aware: bool = False) -> List[Predictor]:
    """The predictors a forecast reaches, at one level: relay-replay and relay-replay-servo as the
    headset runs them but fed the forecast's effective times (decision 14's F3 baselines), and the model."""
    config = prior_config or headset_joint_config(tps)
    out: List[Predictor] = [Baseline(f"relay-replay-fc@{level}", RelayReplayPredictor(headset_joint_config(tps)),
                                     blind=False, uses_forecast=True),
                            Baseline(f"relay-replay-servo-fc@{level}", RelayReplayServoPredictor(headset_joint_config(tps)),
                                     blind=False, uses_forecast=True)]
    if onnx_path:
        out.append(ModelPredictor(onnx_path, config, aware, name=f"seq-model@{level}"))
    return out


def evaluate_with_sweep(view: View, onnx_path: Optional[str], aware: bool = False,
                        sensed_ages_ms: Optional[Sequence[float]] = None, levels: Sequence[str] = SWEEP_LEVELS,
                        seed: int = 0, jitter_s: float = 20.0, servo: Optional[Dict[str, float]] = None) -> ViewResult:
    """The base run (no forecast: the headset today, and every baseline), then one run per forecast
    level for the predictors a forecast reaches, merged into one result."""
    from .data.dataset import view_forecasts

    base = evaluate_view(view, default_predictors(onnx_path, view.tps, aware=aware, sensed_ages_ms=sensed_ages_ms,
                                                  servo=servo), seed=seed, jitter_s=jitter_s)
    config = headset_joint_config(view.tps, sensed_ages_ms=tuple(sensed_ages_ms)) if sensed_ages_ms else None
    for level in levels:
        forecasts, eff = view_forecasts(view, level, seed ^ 0x5F0CA57)
        run = evaluate_view(view, forecast_predictors(onnx_path, view.tps, level, prior_config=config, aware=aware),
                            seed=seed, jitter_s=jitter_s, forecasts=forecasts, eff_forecast=eff)
        base.merge(run)
    return base
