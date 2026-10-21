"""Layout v1: what the model sees for one prediction, built from nothing but the predictor's calls.

``FeatureState`` takes the same calls an ``ICommandAwarePredictor`` takes (``observe_command``,
``observe_state``) and, for a target time, returns one row per arm joint plus an arm-wide row.

Per arm joint (``JOINT_FEATURES`` = 177), every time relative to the target T and divided by
``TIME_SCALE_S``, every angle relative to the joint's newest fresh reading ``v`` and squashed by
``asinh(x / ANGLE_SCALE_RAD)``:

| Part | Count | Content |
|---|---|---|
| prior | 2 | the speed-blind relay-replay prediction at T; its slope over the last 40 ms |
| commands | 24 x 3 | newest first: time, target, mask |
| command bins | 8 x 2 | 100 ms bins from T-0.5 s back to T-1.3 s: mean target, mask |
| readings | 16 x 3 | the newest fresh readings: time, value, mask |
| innovations | 8 x 4 | the prior's error on each fresh reading before it arrived: time, error, horizon, mask |
| identity | 5 | which joint (one-hot, motors 1-5) |
| angle | 2 | sin, cos of ``v`` |

Arm-wide (``ARM_FEATURES`` = 13): sin/cos of the newest proximal, distal and wrist-pitch readings
(gravity), staleness, readings in the last second, the longest gap between readings in the last two
seconds, the newest command's effective time, the gripper command and its change over 0.5 s, and how
full the command window is. The speed-aware ablation (``aware=True``) adds the newest command's
requested speed (layout ``v1-aware``).

**Never in the window:** the requested speed (except ``v1-aware``), any network profile or delay,
the hold-back, the payload, or anything else not passed through the predictor's calls.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..baselines.relay_replay import RelayReplayPredictor
from ..baselines.types import JointPredictorConfig, JointSample

LAYOUT_ID = "v1"
ARM_MOTORS: Tuple[int, ...] = (1, 2, 3, 4, 5)
GRAVITY_MOTORS: Tuple[int, ...] = (2, 3, 4)
GRIPPER_MOTOR = 10

N_COMMANDS = 24
N_BINS = 8
BIN_S = 0.1
BINS_FROM_S = 0.5
N_READINGS = 16
N_INNOVATIONS = 8
TIME_SCALE_S = 0.5
ANGLE_SCALE_RAD = 0.2
INNOVATION_SCALE_RAD = 0.05
SLOPE_WINDOW_S = 0.04
SLOPE_SCALE_RAD_S = 2.0
SPEED_SCALE_RAD_S = 6.0
COMMAND_RING = 64  # JointPredictorConfig.CommandHistoryCapacity on the headset
MAX_CORRECTION_RAD = math.radians(15.0)
FALLBACK_STALENESS_S = 1.0

JOINT_FEATURES = 2 + N_COMMANDS * 3 + N_BINS * 2 + N_READINGS * 3 + N_INNOVATIONS * 4 + len(ARM_MOTORS) + 2
ARM_FEATURES = 13


def arm_features(aware: bool) -> int:
    return ARM_FEATURES + (1 if aware else 0)


def _sq(x: float, scale: float) -> float:
    return math.asinh(x / scale)


@dataclass
class Window:
    """One prediction's inputs and what the model is corrected against."""

    joints: np.ndarray  # float32 [5, JOINT_FEATURES]
    arm: np.ndarray  # float32 [ARM_FEATURES (+1)]
    prior: np.ndarray  # float64 [5] the prior's prediction at T
    newest: np.ndarray  # float64 [5] each joint's newest fresh reading
    valid: bool  # False: the predictor falls back to the prior
    staleness_s: float
    reason: str = ""


class FeatureState:
    """Everything the window needs, updated call by call, allocation-light and deterministic."""

    def __init__(self, prior_config: JointPredictorConfig, aware: bool = False):
        self.config = prior_config
        self.tps = prior_config.ticks_per_second
        self.aware = aware
        self.prior = RelayReplayPredictor(prior_config)
        self.reset()

    def reset(self) -> None:
        self.prior.reset()
        self.commands: Deque[Tuple[int, Dict[int, float], float]] = deque(maxlen=COMMAND_RING)
        self.readings: Dict[int, Deque[Tuple[int, float]]] = {m: deque(maxlen=N_READINGS) for m in ARM_MOTORS}
        self.innovations: Dict[int, Deque[Tuple[int, float, int]]] = {m: deque(maxlen=N_INNOVATIONS) for m in ARM_MOTORS}
        self.fresh_times: Deque[int] = deque(maxlen=64)
        self.last_fresh = None
        self.newest: Dict[int, float] = {}
        self.last_command_eff = None

    # -- the predictor's calls -----------------------------------------------------------------

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed: float) -> None:
        self.prior.observe_command(effective_ticks, targets, speed if self.aware else 0.0)
        if self.last_command_eff is not None and effective_ticks < self.last_command_eff:
            return  # the prior rejected it; so does the window
        self.last_command_eff = effective_ticks
        self.commands.append((int(effective_ticks), {t.motor_id: float(t.value) for t in targets}, float(speed)))

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        fresh = self.last_fresh is None or sensed_ticks > self.last_fresh
        if fresh:
            # The prior's error on this reading, before it sees it.
            ok, predicted = self.prior.try_predict(sensed_ticks)
            prior_state = self.prior.state_ticks if self.prior.has_state else None
            if ok:
                by_motor = {s.motor_id: s.value for s in predicted}
                for j in joints:
                    if j.motor_id in self.innovations and j.sensed and j.motor_id in by_motor:
                        horizon = sensed_ticks - prior_state if prior_state is not None else 0
                        self.innovations[j.motor_id].append((int(sensed_ticks), float(j.value) - by_motor[j.motor_id], int(horizon)))
            for j in joints:
                if j.motor_id in self.readings and j.sensed:
                    self.readings[j.motor_id].append((int(sensed_ticks), float(j.value)))
                    self.newest[j.motor_id] = float(j.value)
            self.last_fresh = int(sensed_ticks)
            self.fresh_times.append(int(sensed_ticks))
        self.prior.observe_state(sensed_ticks, joints)

    # -- the window ----------------------------------------------------------------------------

    def window(self, target_ticks: int) -> Window:
        tps = float(self.tps)
        n_arm = arm_features(self.aware)
        joints = np.zeros((len(ARM_MOTORS), JOINT_FEATURES), dtype=np.float32)
        arm = np.zeros(n_arm, dtype=np.float32)
        prior = np.zeros(len(ARM_MOTORS))
        newest = np.zeros(len(ARM_MOTORS))
        ok, predicted = self.prior.try_predict(target_ticks)
        if not ok or self.last_fresh is None or any(m not in self.newest for m in ARM_MOTORS):
            return Window(joints, arm, prior, newest, False, float("inf"), "no fresh reading yet")
        p_now = {s.motor_id: s.value for s in predicted}
        _, before = self.prior.try_predict(target_ticks - int(SLOPE_WINDOW_S * tps))
        p_before = {s.motor_id: s.value for s in before}
        staleness = (target_ticks - self.last_fresh) / tps
        cmds = list(self.commands)
        for row, m in enumerate(ARM_MOTORS):
            v = self.newest[m]
            prior[row] = p_now.get(m, v)
            newest[row] = v
            f = joints[row]
            k = 0
            f[k] = _sq(prior[row] - v, ANGLE_SCALE_RAD)
            f[k + 1] = (prior[row] - p_before.get(m, prior[row])) / SLOPE_WINDOW_S / SLOPE_SCALE_RAD_S
            k += 2
            recent = [c for c in reversed(cmds) if m in c[1]][:N_COMMANDS]
            for i in range(N_COMMANDS):
                if i < len(recent):
                    t_eff, values, _ = recent[i]
                    f[k] = (t_eff - target_ticks) / tps / TIME_SCALE_S
                    f[k + 1] = _sq(values[m] - v, ANGLE_SCALE_RAD)
                    f[k + 2] = 1.0
                k += 3
            for b in range(N_BINS):
                hi = target_ticks - (BINS_FROM_S + BIN_S * b) * tps
                lo = target_ticks - (BINS_FROM_S + BIN_S * (b + 1)) * tps
                inside = [c[1][m] for c in cmds if m in c[1] and lo <= c[0] < hi]
                if inside:
                    f[k] = _sq(sum(inside) / len(inside) - v, ANGLE_SCALE_RAD)
                    f[k + 1] = 1.0
                k += 2
            reads = list(self.readings[m])[::-1]
            for i in range(N_READINGS):
                if i < len(reads):
                    t_r, value = reads[i]
                    f[k] = (t_r - target_ticks) / tps / TIME_SCALE_S
                    f[k + 1] = _sq(value - v, ANGLE_SCALE_RAD)
                    f[k + 2] = 1.0
                k += 3
            innovations = list(self.innovations[m])[::-1]
            for i in range(N_INNOVATIONS):
                if i < len(innovations):
                    t_i, err, horizon = innovations[i]
                    f[k] = (t_i - target_ticks) / tps / TIME_SCALE_S
                    f[k + 1] = _sq(err, INNOVATION_SCALE_RAD)
                    f[k + 2] = horizon / tps / TIME_SCALE_S
                    f[k + 3] = 1.0
                k += 4
            f[k + row] = 1.0
            k += len(ARM_MOTORS)
            f[k] = math.sin(v)
            f[k + 1] = math.cos(v)
        k = 0
        for m in GRAVITY_MOTORS:
            arm[k], arm[k + 1] = math.sin(self.newest[m]), math.cos(self.newest[m])
            k += 2
        arm[k] = staleness / TIME_SCALE_S
        window_start = target_ticks - 1.0 * tps
        fresh = [t for t in self.fresh_times if t <= target_ticks]
        arm[k + 1] = sum(1 for t in fresh if t >= window_start) / 20.0
        recent_fresh = [t for t in fresh if t >= target_ticks - 2.0 * tps]
        gaps = np.diff(recent_fresh) if len(recent_fresh) > 1 else np.zeros(1)
        arm[k + 2] = float(gaps.max()) / tps / TIME_SCALE_S
        if cmds:
            arm[k + 3] = (cmds[-1][0] - target_ticks) / tps / TIME_SCALE_S
            gripper = [c[1][GRIPPER_MOTOR] for c in cmds if GRIPPER_MOTOR in c[1]]
            if gripper:
                arm[k + 4] = gripper[-1]
                old = [c[1][GRIPPER_MOTOR] for c in cmds if GRIPPER_MOTOR in c[1] and c[0] <= target_ticks - 0.5 * tps]
                arm[k + 5] = gripper[-1] - (old[-1] if old else gripper[0])
            arm[k + 6] = min(len(cmds), N_COMMANDS) / N_COMMANDS
            if self.aware:
                arm[k + 7] = cmds[-1][2] / SPEED_SCALE_RAD_S
        valid = staleness <= FALLBACK_STALENESS_S
        return Window(joints, arm, prior, newest, valid, staleness, "" if valid else "stale")


def correction(raw: np.ndarray) -> np.ndarray:
    """The model's first output per joint, as a bounded correction to the prior (rad)."""
    return MAX_CORRECTION_RAD * np.tanh(raw)
