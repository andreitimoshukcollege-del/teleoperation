"""The trainer's own evaluation, and the trainer gate (ADR 0021 decision 9).

Every predictor receives the same calls a view records, and is asked:
- at **truth read instants** (exact labels): ``joint_prediction_error_deg``;
- just before and just after each **fresh reading** reaches it: ``joint_correction_deg``;
- at every **90 Hz display tick** over a stretch of each view: ``joint_display_jitter_deg``;
- around **events**, aligned on t_info (the first moment data could show the change):
  ``joint_event_error_deg`` and ``joint_recovery_time_ms``.

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
from .baselines.types import JointPredictorConfig, JointSample, headset_joint_config
from .data.dataset import EVENT_KINDS, call_stream, command_samples, state_samples
from .data.views import View
from .features import layout as L

DEG = 180.0 / math.pi
ARM = L.ARM_MOTORS
EVENT_WINDOWS_MS = ((-2000, 0), (0, 100), (100, 250), (250, 500), (500, 1000), (1000, 2000))


class Predictor:
    name = "?"

    def reset(self) -> None: ...
    def observe_command(self, eff: int, targets: Sequence[JointSample], speed: float) -> None: ...
    def observe_state(self, sensed: int, joints: Sequence[JointSample]) -> None: ...

    def predict(self, target: int) -> Tuple[Optional[np.ndarray], bool]:
        """Arm joints in ARM order, and whether this was a fallback."""
        raise NotImplementedError


class Baseline(Predictor):
    """hold-last or relay-replay, speed-aware (as the headset runs it) or blind (the host passes 0)."""

    def __init__(self, name: str, impl, blind: bool):
        self.name, self.impl, self.blind = name, impl, blind

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


def _t_info(view: View) -> List[Tuple[int, str, int]]:
    """t_info per event: a downlink switch shows up at once; anything the robot does (load, hand,
    slider, uplink) shows up in the first frame sensed after it, when that frame reaches the headset."""
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
    return out


def evaluate_view(view: View, predictors: List[Predictor], reads_per_s: float = 20.0, jitter_s: float = 20.0,
                  warmup_s: float = 2.0, seed: int = 0) -> ViewResult:
    rng = np.random.default_rng(seed)
    tps = view.tps
    for p in predictors:
        p.reset()
    calls = call_stream(view)
    first_state = int(view.st_t[0]) if len(view.st_t) else 0
    arm_reads = np.nonzero(np.isin(view.tr_motor, ARM) & (view.tr_t >= first_state + int(warmup_s * tps)))[0]
    duration = (int(view.tr_t[arm_reads[-1]]) - int(view.tr_t[arm_reads[0]])) / tps if arm_reads.size else 0.0
    keep = min(arm_reads.size, int(reads_per_s * duration))
    chosen = np.sort(rng.choice(arm_reads, size=keep, replace=False)) if keep else np.zeros(0, dtype=np.int64)
    # Fresh-state calls: where the correction is measured.
    fresh_calls = set()
    newest = None
    for idx, (t, what, i) in enumerate(calls):
        if what == 1:
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
            if what == 0:
                targets = command_samples(view, i)
                for p in predictors:
                    p.observe_command(int(view.cmd_eff[i]), targets, float(view.cmd_speed[i]))
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


def summarize(results: Sequence[ViewResult]) -> Dict[str, object]:
    names = list(results[0].errors) if results else []
    by_schedule: Dict[str, Dict[str, object]] = {}
    for sched in sorted({r.schedule for r in results}):
        rs = [r for r in results if r.schedule == sched]
        by_schedule[sched] = {n: {
            "joint_prediction_error_deg": percentiles(np.concatenate([r.errors[n][:, 2] for r in rs])),
            "joint_correction_deg": percentiles(np.concatenate([r.corrections[n] for r in rs])),
            "joint_display_jitter_deg": percentiles(np.concatenate([r.jitter[n] for r in rs])),
        } for n in names}
    overall = {n: {
        "joint_prediction_error_deg": percentiles(np.concatenate([r.errors[n][:, 2] for r in results])),
        "joint_correction_deg": percentiles(np.concatenate([r.corrections[n] for r in results])),
        "joint_display_jitter_deg": percentiles(np.concatenate([r.jitter[n] for r in results])),
        "joint_event_error_deg": {k: event_windows(results, n, [k]) for k in ("network", "hand-on", "slider", "payload")},
        "joint_recovery_time_ms": {k: percentiles(recovery_times_ms(results, n, [k])) for k in ("network", "hand-on", "slider")},
    } for n in names}
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
        by_condition[cond] = {n: {
            "joint_prediction_error_deg": percentiles(np.concatenate([r.errors[n][:, 2] for r in rs])),
        } for n in names}
    return {
        "views": len(results),
        "by_schedule": by_schedule,
        "by_condition": by_condition,
        "overall": overall,
        "joint_predictor_fallback": {n: {"fallbacks": f, "calls": c, "rate": f / c if c else float("nan")} for n, (f, c) in fallback.items()},
        "joint_inference_us": {n: percentiles(np.concatenate(v)) for n, v in inference.items()},
    }


def bootstrap_p95_difference(results: Sequence[ViewResult], a: str, b: str, schedule: Optional[str] = None,
                             n: int = 2000, seed: int = 0) -> Dict[str, float]:
    """Paired session bootstrap of p95(a) - p95(b): sessions are the unit, not samples (methodology
    review: queries are autocorrelated and sessions dominate the variance)."""
    rs = [r for r in results if schedule is None or r.schedule == schedule]
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
                       aware: bool = False, sensed_ages_ms: Optional[Sequence[float]] = None) -> List[Predictor]:
    """The ladder (ADR 0021): hold-last, relay-replay as the headset runs it, relay-replay blind,
    relay-replay-adaptive, and the model; with fitted reading ages (``armpred sysid``), also
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
    ]
    if sensed_ages_ms:
        out.append(Baseline("relay-replay-fitted", RelayReplayPredictor(config), blind=False))
    if onnx_path:
        out.append(ModelPredictor(onnx_path, config, aware))
    return out
