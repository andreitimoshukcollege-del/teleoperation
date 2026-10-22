"""Copy of ``Prediction/HoldLastJointPredictor.cs`` (hold-last): the last measured joints, unchanged.

The baseline row every joint-space table must carry (docs/metrics.md §8).
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

from .types import JointPredictorConfig, JointSample, PredictorDiagnostics, f32


class HoldLastJointPredictor:
    def __init__(self, config: JointPredictorConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._joints: List[JointSample] = []
        self._has_state = False
        self._state_ticks = 0
        self._accepted = 0
        self._rejected = 0
        self._last_horizon = 0

    @property
    def diagnostics(self) -> PredictorDiagnostics:
        return PredictorDiagnostics(self._last_horizon, self._state_ticks if self._has_state else 0,
                                    self._accepted, self._rejected)

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed_rad_per_sec: float) -> None:
        pass

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        if self._has_state and sensed_ticks < self._state_ticks:
            self._rejected += 1
            return
        n = min(len(joints), self.config.max_joints)
        self._joints = [JointSample(j.motor_id, f32(j.value), j.sensed) for j in joints[:n]]
        self._state_ticks = sensed_ticks
        self._has_state = True
        self._accepted += 1

    def try_predict(self, target_ticks: int) -> Tuple[bool, List[JointSample]]:
        if not self._has_state:
            return False, []
        self._last_horizon = max(0, target_ticks - self._state_ticks)
        return True, list(self._joints)
