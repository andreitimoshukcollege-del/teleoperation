"""The FR3's trainer-side evaluation (docs/adr/0028 decision 5): every predictor on the same views, at the
display's instants, against the truth.

For each test view (one session under one named schedule) the predictors get the headset's calls in order:
``hold-last``, ``franka-replay`` (the prior) and the learned model on top of it (layout v4). At every
``EVAL_HZ`` tick after the warm-up, each is asked for that tick:
- ``joint_prediction_error_deg``: |prediction - truth| for every arm joint, the truth interpolated from ``QS``
  at 250 Hz;
- ``joint_correction_deg``: at every fresh reading, the jump in the prediction for its arrival instant across
  the observation (docs/metrics.md §4).

The hands are reported apart and are never in the gate. Percentiles, never means.

**Every number here is simulated.** The arm's tracking is the simulator's invention and ``franka-replay`` is
exact up to it, so this decides only whether a model is worth fine-tuning on real sessions. It is never a
gate verdict: the FR3 gate is scored on real sessions only (ADR 0028 decision 5).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .baselines.franka_replay import FrankaReplayPredictor, franka_joint_config, shaper_for_profile
from .baselines.hold_last import HoldLastJointPredictor
from .baselines.types import JointSample
from .data.dataset import CALL_COMMAND, CALL_STATE, call_stream, command_samples, state_samples
from .data.views import View
from .features import layout_v4 as L4

DEG = 180.0 / math.pi
EVAL_HZ = 30.0
WARMUP_S = 2.0


class LearnedV4:
    """The learned predictor as the headset will run it: layout v4's window, the model's correction on every row,
    and a fall back to the prior when the window is not valid."""

    name = "learned"

    def __init__(self, model, tps: int, profile=None):
        import torch

        self.torch = torch
        self.model = model.eval()
        self.profile = profile or L4.default_profile()
        self.state = L4.FeatureState(L4.default_prior_config(tps, self.profile), profile=self.profile)
        self.motors = [m for m, _, _ in L4._rows(self.profile)]
        self.fallbacks = 0
        self.calls = 0
        self.seconds = 0.0

    def reset(self) -> None:
        self.state.reset()

    def observe_command(self, t: int, targets, speed: float = 0.0) -> None:
        self.state.observe_command(t, targets, speed)

    def observe_state(self, t: int, joints) -> None:
        self.state.observe_state(t, joints)

    def try_predict(self, t: int):
        t0 = time.perf_counter()
        w = self.state.window(t)
        self.calls += 1
        ok, prior = self.state.prior.try_predict(t)
        if not ok:
            return False, []
        if not w.valid:
            self.fallbacks += 1
            return True, prior
        with self.torch.no_grad():
            arm = self.torch.from_numpy(w.arms[w.arm_of_row])
            y = self.model(self.torch.from_numpy(w.joints), arm).numpy()
        corrected = {m: L4.to_wire(m, w.prior[r] + float(y[r, 0]), self.profile) for r, m in enumerate(self.motors)}
        self.seconds += time.perf_counter() - t0
        return True, [JointSample(s.motor_id, corrected.get(s.motor_id, s.value), s.sensed) for s in prior]


@dataclass
class ViewResult:
    name: str
    schedule: str
    errors: Dict[str, np.ndarray] = field(default_factory=dict)
    hand_errors: Dict[str, np.ndarray] = field(default_factory=dict)
    corrections: Dict[str, np.ndarray] = field(default_factory=dict)
    inference_us: Optional[float] = None
    fallback_share: Optional[float] = None


def _truth_at(view: View, motors: Sequence[int]):
    """Per motor, (times, values) of the truth, for interpolation."""
    out = {}
    for m in motors:
        sel = view.tr_motor == m
        out[m] = (view.tr_t[sel].astype(np.float64), view.tr_value[sel])
    return out


def evaluate_view(view: View, predictors: Dict[str, object], profile=None) -> ViewResult:
    profile = profile or L4.default_profile()
    hands = set(profile.hand_motors)
    motors = profile.motors
    truth = _truth_at(view, motors)
    tps = view.tps
    calls = call_stream(view, None)
    if not len(view.st_t):
        return ViewResult(view.meta.get("session", "?"), view.meta.get("schedule", "?"))
    start = int(view.st_t[0]) + int(WARMUP_S * tps)
    end = int(min(view.tr_t.max(), view.cmd_t.max()))
    ticks = np.arange(start, end, int(tps / EVAL_HZ), dtype=np.int64)
    err = {n: [] for n in predictors}
    hand_err = {n: [] for n in predictors}
    corr = {n: [] for n in predictors}
    newest_sensed = None
    ci = 0
    for t in ticks:
        while ci < len(calls) and calls[ci][0] <= t:
            ct, what, i = calls[ci]
            if what == CALL_COMMAND:
                targets = command_samples(view, i)
                for p in predictors.values():
                    p.observe_command(int(view.cmd_eff[i]), targets, 0.0)
            elif what == CALL_STATE:
                joints = state_samples(view, i)
                sensed = int(view.st_sensed[i])
                fresh = newest_sensed is None or sensed > newest_sensed
                if fresh:
                    newest_sensed = sensed
                for name, p in predictors.items():
                    before = p.try_predict(ct) if fresh and ct >= start else (False, [])
                    p.observe_state(sensed, joints)
                    if before[0]:
                        ok, after = p.try_predict(ct)
                        if ok:
                            b = {s.motor_id: s.value for s in before[1]}
                            corr[name] += [abs(s.value - b[s.motor_id]) * DEG for s in after if s.motor_id in b and s.motor_id not in hands]
            ci += 1
        for name, p in predictors.items():
            ok, out = p.try_predict(int(t))
            if not ok:
                continue
            for s in out:
                if s.motor_id not in truth:
                    continue
                tt, tv = truth[s.motor_id]
                value = float(np.interp(float(t), tt, tv))
                (hand_err if s.motor_id in hands else err)[name].append(abs(s.value - value) * (1.0 if s.motor_id in hands else DEG))
    res = ViewResult(view.meta.get("session", "?"), view.meta.get("schedule", "?"))
    for n in predictors:
        res.errors[n] = np.array(err[n])
        res.hand_errors[n] = np.array(hand_err[n])
        res.corrections[n] = np.array(corr[n])
    learned = predictors.get("learned")
    if isinstance(learned, LearnedV4) and learned.calls:
        res.inference_us = 1e6 * learned.seconds / max(1, learned.calls - learned.fallbacks)
        res.fallback_share = learned.fallbacks / learned.calls
    return res


def baselines(tps: int, profile=None) -> Dict[str, object]:
    profile = profile or L4.default_profile()
    config = franka_joint_config(tps, max_joints=len(profile.motors), instant=tuple(profile.hand_motors))
    return {"hold-last": HoldLastJointPredictor(config), "franka-replay": FrankaReplayPredictor(config, shaper_for_profile(profile))}


def percentiles(e: np.ndarray) -> Dict[str, float]:
    e = e[np.isfinite(e)] if e.size else e
    if e.size == 0:
        return {"p50": float("nan"), "p95": float("nan"), "p99": float("nan"), "max": float("nan"), "n": 0}
    return {"p50": float(np.percentile(e, 50)), "p95": float(np.percentile(e, 95)), "p99": float(np.percentile(e, 99)),
            "max": float(e.max()), "n": int(e.size)}


def summarize(results: List[ViewResult]) -> Dict[str, object]:
    """Per schedule and overall, per predictor: error and correction percentiles (degrees), hands apart (fraction)."""
    names = sorted({n for r in results for n in r.errors})
    by_schedule: Dict[str, Dict[str, object]] = {}
    for sched in sorted({r.schedule for r in results}):
        rs = [r for r in results if r.schedule == sched]
        by_schedule[sched] = {n: {"joint_prediction_error_deg": percentiles(np.concatenate([r.errors[n] for r in rs])),
                                  "joint_correction_deg": percentiles(np.concatenate([r.corrections[n] for r in rs])),
                                  "hand_error_fraction": percentiles(np.concatenate([r.hand_errors[n] for r in rs]))} for n in names}
    overall = {n: {"joint_prediction_error_deg": percentiles(np.concatenate([r.errors[n] for r in results])),
                   "joint_correction_deg": percentiles(np.concatenate([r.corrections[n] for r in results]))} for n in names}
    inference = [r.inference_us for r in results if r.inference_us is not None]
    fallback = [r.fallback_share for r in results if r.fallback_share is not None]
    return {"simulated": True, "views": len(results), "bySchedule": by_schedule, "overall": overall,
            "learnedInferenceUsMean": float(np.mean(inference)) if inference else None,
            "learnedFallbackShare": float(np.mean(fallback)) if fallback else None}


def sim_gate_rows(summary: Dict[str, object]) -> List[str]:
    """ADR 0028 decision 5's F1-F3 as they would read on these simulated views: a diagnostic, never a verdict."""
    rows = []
    s = summary["bySchedule"]

    def best(sched, metric, q):
        cands = [(s[sched][n][metric][q], n) for n in ("franka-replay", "hold-last") if n in s.get(sched, {})]
        return min(cands) if cands else (float("nan"), "-")

    def learned(sched, metric, q):
        return s.get(sched, {}).get("learned", {}).get(metric, {}).get(q, float("nan"))

    hard = "150ms-20j-0.5loss@0"
    if hard in s:
        b, who = best(hard, "joint_prediction_error_deg", "p95")
        l = learned(hard, "joint_prediction_error_deg", "p95")
        ok = l <= b * 0.85 and b - l >= 0.05
        rows.append(f"F1 (sim) {hard}: learned p95 {l:.3f} vs best {who} {b:.3f} -> {'would pass' if ok else 'would fail'}")
    for sched in ("lan@0", "50ms-5j@0"):
        if sched in s:
            b95, _ = best(sched, "joint_prediction_error_deg", "p95")
            b99, _ = best(sched, "joint_prediction_error_deg", "p99")
            l95, l99 = learned(sched, "joint_prediction_error_deg", "p95"), learned(sched, "joint_prediction_error_deg", "p99")
            ok = l95 <= b95 + 0.02 and l99 <= b99 + 0.05
            rows.append(f"F2 (sim) {sched}: learned p95/p99 {l95:.3f}/{l99:.3f} vs best {b95:.3f}/{b99:.3f} -> {'would pass' if ok else 'would fail'}")
    o = summary["overall"]
    if "learned" in o and "franka-replay" in o:
        lc, pc = o["learned"]["joint_correction_deg"], o["franka-replay"]["joint_correction_deg"]
        ok = lc["p95"] <= pc["p95"] + 0.02 and lc["max"] <= 1.5 * pc["max"]
        rows.append(f"F3 (sim) correction p95/max: learned {lc['p95']:.3f}/{lc['max']:.3f} vs franka-replay {pc['p95']:.3f}/{pc['max']:.3f} -> "
                    f"{'would pass' if ok else 'would fail'}")
    return rows
