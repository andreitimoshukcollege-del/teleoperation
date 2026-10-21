"""relay-replay-adaptive (dev baseline, ADR 0021): the speed-blind prior, corrected online by two
classical estimates per joint, with nothing learned offline.

The learned model must beat this, not just the fixed ``relay-replay``: speed, load and lag are each
one or two hidden numbers per joint, and a classical estimator can chase them too. Per arm joint:

- **lag** tau: the arm runs ``tau`` behind the prior (a slower servo, a load, an uplink later than
  RTT/2 says), so the prediction is ``prior(T - tau)``. It may be negative: on an asymmetric link
  the headset's RTT/2 overstates the uplink, commands land earlier than the prior assumes, and the
  arm runs *ahead* of it;
- **droop** b: a steady offset (gravity under a payload, a hand pushing), added on top.

On every fresh reading, before it is seen, the error ``r = v - (prior(t - tau) + b)`` updates one of
them. While the joint moves, tau takes a normalised-LMS step: ``d r / d tau = slope``, so the lag
that would zero the error is ``tau - r / slope``, and tau moves ``LAG_RATE`` of the way there (at
most ``LAG_STEP_MAX_S`` per reading). While it is nearly still, the error is the droop's to explain,
and b moves ``DROOP_RATE`` of the way. Both are clipped to physical ranges. It sees exactly what the
headset sees.

When the Core version is built (ADR 0021 decision 8) it must reproduce this file's arithmetic, the
way ``relay_replay.py`` reproduces Core's.
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

from .relay_replay import RelayReplayPredictor
from .types import JointPredictorConfig, JointSample, PredictorDiagnostics

ARM = (1, 2, 3, 4, 5)
SLOPE_DT_S = 0.01
LAG_MIN_S = -0.2
LAG_MAX_S = 0.3
LAG_STEP_MAX_S = 0.02
DROOP_MAX_RAD = 0.15
LAG_RATE = 0.1
DROOP_RATE = 0.05
MOVING_RAD_PER_S = 0.2
STILL_RAD_PER_S = 0.05


class RelayReplayAdaptive:
    def __init__(self, config: JointPredictorConfig):
        self.config = config
        self.prior = RelayReplayPredictor(config)
        self.tps = config.ticks_per_second
        self.reset()

    def reset(self) -> None:
        self.prior.reset()
        self.lag_s: Dict[int, float] = {m: 0.0 for m in ARM}
        self.droop: Dict[int, float] = {m: 0.0 for m in ARM}
        self.last_fresh = None
        self.updates = 0

    @property
    def diagnostics(self) -> PredictorDiagnostics:
        d = self.prior.diagnostics
        d.extra = {"lag_s": dict(self.lag_s), "droop": dict(self.droop), "updates": self.updates}
        return d

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed_rad_per_sec: float) -> None:
        self.prior.observe_command(effective_ticks, targets, 0.0)  # speed-blind, like the learned model

    def _at(self, ticks: int) -> Dict[int, float]:
        ok, out = self.prior.try_predict(ticks)
        return {s.motor_id: s.value for s in out} if ok else {}

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        fresh = self.last_fresh is None or sensed_ticks > self.last_fresh
        if fresh and self.prior.has_state:
            for j in joints:
                m = j.motor_id
                if m not in self.lag_s or not j.sensed:
                    continue
                t = sensed_ticks - int(self.lag_s[m] * self.tps)
                now = self._at(t).get(m)
                before = self._at(t - int(SLOPE_DT_S * self.tps)).get(m)
                if now is None or before is None:
                    continue
                slope = (now - before) / SLOPE_DT_S
                r = j.value - (now + self.droop[m])
                if abs(slope) >= MOVING_RAD_PER_S:
                    # The arm behind a rising prior gives r < 0 with slope > 0: the lag grows.
                    step = max(-LAG_STEP_MAX_S, min(LAG_STEP_MAX_S, -LAG_RATE * r / slope))
                    self.lag_s[m] = min(LAG_MAX_S, max(LAG_MIN_S, self.lag_s[m] + step))
                elif abs(slope) < STILL_RAD_PER_S:
                    self.droop[m] = min(DROOP_MAX_RAD, max(-DROOP_MAX_RAD, self.droop[m] + DROOP_RATE * r))
            self.updates += 1
        if fresh:
            self.last_fresh = sensed_ticks
        self.prior.observe_state(sensed_ticks, joints)

    def try_predict(self, target_ticks: int) -> Tuple[bool, List[JointSample]]:
        ok, out = self.prior.try_predict(target_ticks)
        if not ok:
            return False, []
        adjusted = []
        for s in out:
            if s.motor_id in self.lag_s:
                shifted = self._at(target_ticks - int(self.lag_s[s.motor_id] * self.tps)).get(s.motor_id, s.value)
                adjusted.append(JointSample(s.motor_id, shifted + self.droop[s.motor_id], s.sensed))
            else:
                adjusted.append(s)
        return True, adjusted
