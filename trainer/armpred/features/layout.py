"""Layout v3: what the model sees for one prediction, built from nothing but the predictor's calls.

``FeatureState`` takes the same calls an ``ICommandAwarePredictor`` takes (``observe_command``,
``observe_state``), plus the network forecasts the host passes on (``observe_forecast``, ADR 0021
decision 11), and for a target time returns one row per predicted joint plus an arm-wide row.

**v3 (2026-10-25) predicts the gripper** (motor 10) as a sixth joint, after the five arm joints
(``PREDICTED_MOTORS``). Its value on the wire is a 0..1 fraction (open..closed); inside the window,
and in the model's output, it is in *servo radians of travel*: fraction / ``GRIPPER_FRACTION_PER_RAD``
(RobotHost's pulses per radian over its open..closed span, the headset's GripperFractionPerRadian).
So every radian scale below means the same physical thing for every row, and a gripper error reads
in servo degrees. ``to_model`` and ``to_wire`` convert. A row whose joint has no sensed reading yet
(the gripper in a recording from before it was read) is referenced to the prior instead, and says so
(its last feature); only the arm joints need a reading for the window to be valid.

Per joint (``JOINT_FEATURES`` = 251), every time relative to the target T and divided by
``TIME_SCALE_S``, every angle relative to the joint's newest fresh reading ``v`` and squashed by
``asinh(x / ANGLE_SCALE_RAD)``:

| Part | Count | Content |
|---|---|---|
| prior | 2 | the speed-blind relay-replay prediction at T; its slope over the last 40 ms |
| commanded path | 80 x 2 | newest first, every 20 ms of effective time from T + 0.2 s back to T - 1.38 s: the target in force (the newest command whose effective time is not later), mask |
| readings | 16 x 3 | the newest fresh readings: time, value, mask |
| innovations | 8 x 4 | the prior's error on each fresh reading before it arrived: time, error, horizon, mask |
| identity | 6 | which joint (one-hot, motors 1-5, 10) |
| angle | 2 | sin, cos of ``v`` |
| reference | 1 | 1 when ``v`` is a reading, 0 when no reading has arrived and ``v`` is the prior |

The commanded path replaces layout v1's 24 newest commands and its 100 ms bins (ADR 0021 decision
10): it is anchored on T, not on the newest command, so it covers every command executing between
the last reading and T at any delay up to the 1 s staleness fallback. It runs over *effective* time,
the only time the predictor is told; with a forecast that is the send plus the forecast's uplink
delay, so the path is laid out as the robot would receive it.

Arm-wide (``ARM_FEATURES`` = 114): the 13 of v1 -- sin/cos of the newest proximal, distal and
wrist-pitch readings (gravity), staleness, readings in the last second, the longest gap between
readings in the last two seconds, the newest command's effective time, the gripper command and its
change over 0.5 s, how full the command window is -- then the **network block** (101): per direction
(uplink, downlink) and horizon (``forecast.HORIZONS_MS``), the delay quantiles p10/p50/p90/p99 as
``asinh(ms / 100)`` and P(loss); per direction the floor (``asinh(ms / 100)``) and P(outage); six
validity flags (quantiles, loss, outage, per direction); and the forecast's age / 0.5 s. With no
forecast the block is zeros with every flag 0. The speed-aware ablation (``aware=True``) appends the
newest command's requested speed (layout ``v3-aware``).

**Never in the window:** the requested speed (except ``v3-aware``), the network profile, schedule or
true delays (a forecast is an estimate, at a stated quality, ADR 0021 decision 11), the hold-back,
the payload, or anything else not passed through the predictor's calls.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..baselines.relay_replay import RelayReplayPredictor
from ..baselines.types import JointPredictorConfig, JointSample
from ..net import forecast as FC

LAYOUT_ID = "v3"
ARM_MOTORS: Tuple[int, ...] = (1, 2, 3, 4, 5)
GRAVITY_MOTORS: Tuple[int, ...] = (2, 3, 4)
GRIPPER_MOTOR = 10
# The joints the model predicts, in row order: the arm, then the gripper. The trainer gate (ADR 0021
# decision 9) is scored on ARM_MOTORS only; the gripper is reported on its own.
PREDICTED_MOTORS: Tuple[int, ...] = ARM_MOTORS + (GRIPPER_MOTOR,)
# The gripper's 0..1 fraction per radian of servo travel: RobotHost's PulsePerRadian (1000 per 240
# degrees) over its open..closed span (645 - 116 = 529 pulses, measured 2026-10-25). The headset's
# JetRoverArmConfig.GripperFractionPerRadian and the trainer's headset_joint_config use the same.
GRIPPER_FRACTION_PER_RAD = 0.4513

N_PATH = 80
PATH_STEP_S = 0.02
PATH_AHEAD_S = 0.2  # the newest point, past T: a command may land sooner than its estimate says
N_READINGS = 16
N_INNOVATIONS = 8
TIME_SCALE_S = 0.5
ANGLE_SCALE_RAD = 0.2
INNOVATION_SCALE_RAD = 0.05
DELAY_SCALE_MS = 100.0
SLOPE_WINDOW_S = 0.04
SLOPE_SCALE_RAD_S = 2.0
SPEED_SCALE_RAD_S = 6.0
COMMAND_RING = 96  # about 2 s at 48 Hz: the path's span and some
WINDOW_FULL = 64
MAX_CORRECTION_RAD = math.radians(15.0)
# The gripper's row may correct further: a gripper command is one jump across its travel (about 127 servo
# degrees), and the speed-blind prior gets its timing badly wrong (2026-10-25 smoke run: the blind prior
# 13.4 servo degrees p95, hold-last 9.1), which a 15-degree cap could not undo.
GRIPPER_MAX_CORRECTION_RAD = math.radians(90.0)
FALLBACK_STALENESS_S = 1.0

JOINT_FEATURES = 2 + N_PATH * 2 + N_READINGS * 3 + N_INNOVATIONS * 4 + len(PREDICTED_MOTORS) + 2 + 1
N_HORIZONS = len(FC.HORIZONS_MS)
NETWORK_FEATURES = 2 * N_HORIZONS * (len(FC.QUANTILES) + 1) + 2 + 2 + 6 + 1
BASE_ARM_FEATURES = 13
ARM_FEATURES = BASE_ARM_FEATURES + NETWORK_FEATURES


def layout_id(aware: bool) -> str:
    return LAYOUT_ID + ("-aware" if aware else "")


def arm_features(aware: bool) -> int:
    return ARM_FEATURES + (1 if aware else 0)


def _sq(x: float, scale: float) -> float:
    return math.asinh(x / scale)


def to_model(motor: int, value: float) -> float:
    """A joint value as the wire carries it (rad; the gripper's 0..1 fraction) in the model's units (rad;
    the gripper's servo radians)."""
    return value / GRIPPER_FRACTION_PER_RAD if motor == GRIPPER_MOTOR else value


def to_wire(motor: int, value: float) -> float:
    """The inverse of ``to_model``."""
    return value * GRIPPER_FRACTION_PER_RAD if motor == GRIPPER_MOTOR else value


@dataclass
class Window:
    """One prediction's inputs and what the model is corrected against."""

    joints: np.ndarray  # float32 [6, JOINT_FEATURES], rows in PREDICTED_MOTORS order
    arm: np.ndarray  # float32 [ARM_FEATURES (+1)]
    prior: np.ndarray  # float64 [6] the prior's prediction at T, in model units (to_model)
    newest: np.ndarray  # float64 [6] each joint's newest fresh reading (the prior where it has none), model units
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
        # Kept in model units (to_model), as the window uses them.
        self.readings: Dict[int, Deque[Tuple[int, float]]] = {m: deque(maxlen=N_READINGS) for m in PREDICTED_MOTORS}
        self.innovations: Dict[int, Deque[Tuple[int, float, int]]] = {m: deque(maxlen=N_INNOVATIONS) for m in PREDICTED_MOTORS}
        self.fresh_times: Deque[int] = deque(maxlen=64)
        self.last_fresh = None
        self.newest: Dict[int, float] = {}
        self.last_command_eff = None
        self.forecast: Optional[Tuple[int, np.ndarray]] = None  # (issued at, the encoded block without its age)

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
                        error = to_model(j.motor_id, float(j.value)) - to_model(j.motor_id, by_motor[j.motor_id])
                        self.innovations[j.motor_id].append((int(sensed_ticks), error, int(horizon)))
            for j in joints:
                if j.motor_id in self.readings and j.sensed:
                    value = to_model(j.motor_id, float(j.value))
                    self.readings[j.motor_id].append((int(sensed_ticks), value))
                    self.newest[j.motor_id] = value
            self.last_fresh = int(sensed_ticks)
            self.fresh_times.append(int(sensed_ticks))
        self.prior.observe_state(sensed_ticks, joints)

    def observe_forecast(self, stream: "FC.ForecastStream", index: int) -> None:
        """The host passes on the network model's newest forecast (ADR 0021 decision 11)."""
        self.forecast = (int(stream.t[index]), encode_forecast(stream, index))

    def clear_forecast(self) -> None:
        self.forecast = None

    # -- the window ----------------------------------------------------------------------------

    def window(self, target_ticks: int) -> Window:
        tps = float(self.tps)
        n_arm = arm_features(self.aware)
        joints = np.zeros((len(PREDICTED_MOTORS), JOINT_FEATURES), dtype=np.float32)
        arm = np.zeros(n_arm, dtype=np.float32)
        prior = np.zeros(len(PREDICTED_MOTORS))
        newest = np.zeros(len(PREDICTED_MOTORS))
        ok, predicted = self.prior.try_predict(target_ticks)
        if not ok or self.last_fresh is None or any(m not in self.newest for m in ARM_MOTORS):
            return Window(joints, arm, prior, newest, False, float("inf"), "no fresh reading yet")
        p_now = {s.motor_id: to_model(s.motor_id, s.value) for s in predicted}
        _, before = self.prior.try_predict(target_ticks - int(SLOPE_WINDOW_S * tps))
        p_before = {s.motor_id: to_model(s.motor_id, s.value) for s in before}
        staleness = (target_ticks - self.last_fresh) / tps
        cmds = list(self.commands)
        effs = np.array([c[0] for c in cmds], dtype=np.int64)
        # The commanded path's grid, newest first: T + PATH_AHEAD_S, then back every PATH_STEP_S.
        grid = target_ticks + int(round(PATH_AHEAD_S * tps)) - np.round(np.arange(N_PATH) * PATH_STEP_S * tps).astype(np.int64)
        in_force = np.searchsorted(effs, grid, side="right") - 1 if len(effs) else np.full(N_PATH, -1)
        for row, m in enumerate(PREDICTED_MOTORS):
            sensed = m in self.newest
            # A joint with no reading yet (the gripper, in a recording made before it was read) is
            # referenced to the prior, and its reference feature says so.
            v = self.newest[m] if sensed else p_now.get(m, 0.0)
            prior[row] = p_now.get(m, v)
            newest[row] = v
            f = joints[row]
            k = 0
            f[k] = _sq(prior[row] - v, ANGLE_SCALE_RAD)
            f[k + 1] = (prior[row] - p_before.get(m, prior[row])) / SLOPE_WINDOW_S / SLOPE_SCALE_RAD_S
            k += 2
            # The target in force for this motor: the newest command at or before each grid point
            # that carried one (every command carries every joint on the JetRover).
            last_index, last_value = -2, 0.0
            for i in range(N_PATH):
                c = int(in_force[i])
                while c >= 0 and m not in cmds[c][1]:
                    c -= 1
                if c >= 0:
                    if c != last_index:
                        last_index, last_value = c, _sq(to_model(m, cmds[c][1][m]) - v, ANGLE_SCALE_RAD)
                    f[k] = last_value
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
            k += len(PREDICTED_MOTORS)
            f[k] = math.sin(v)
            f[k + 1] = math.cos(v)
            f[k + 2] = 1.0 if sensed else 0.0
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
            arm[k + 6] = min(len(cmds), WINDOW_FULL) / WINDOW_FULL
        k = BASE_ARM_FEATURES
        if self.forecast is not None:
            issued, block = self.forecast
            arm[k:k + NETWORK_FEATURES - 1] = block
            arm[k + NETWORK_FEATURES - 1] = (target_ticks - issued) / tps / TIME_SCALE_S
        if self.aware and cmds:
            arm[BASE_ARM_FEATURES + NETWORK_FEATURES] = cmds[-1][2] / SPEED_SCALE_RAD_S
        valid = staleness <= FALLBACK_STALENESS_S
        return Window(joints, arm, prior, newest, valid, staleness, "" if valid else "stale")


def encode_forecast(stream: "FC.ForecastStream", index: int) -> np.ndarray:
    """One forecast as the network block's first ``NETWORK_FEATURES - 1`` floats (its age is added at
    the window): per direction, per horizon, asinh(quantile ms / 100) and P(loss); the floors; the
    outage probabilities; the six validity flags. An invalid field is zero, and its flag says so."""
    out = np.zeros(NETWORK_FEATURES - 1, dtype=np.float32)
    k = 0
    for d in (FC.UP, FC.DOWN):
        vq, vl = bool(stream.valid_q[index, d]), bool(stream.valid_loss[index, d])
        for h in range(N_HORIZONS):
            if vq:
                out[k:k + len(FC.QUANTILES)] = np.arcsinh(stream.q_ms[index, d, h] / DELAY_SCALE_MS)
            k += len(FC.QUANTILES)
            if vl:
                out[k] = stream.loss[index, d, h]
            k += 1
    for d in (FC.UP, FC.DOWN):
        if stream.valid_q[index, d]:
            out[k] = math.asinh(float(stream.floor_ms[index, d]) / DELAY_SCALE_MS)
        k += 1
    for d in (FC.UP, FC.DOWN):
        if stream.valid_outage[index, d]:
            out[k] = stream.outage[index, d]
        k += 1
    for d in (FC.UP, FC.DOWN):
        out[k] = float(stream.valid_q[index, d])
        out[k + 1] = float(stream.valid_loss[index, d])
        out[k + 2] = float(stream.valid_outage[index, d])
        k += 3
    assert k == NETWORK_FEATURES - 1
    return out


def correction(raw: np.ndarray, motor: int = 1) -> np.ndarray:
    """The model's first output for one joint, as a bounded correction to the prior (rad; the gripper's servo rad)."""
    return (GRIPPER_MAX_CORRECTION_RAD if motor == GRIPPER_MOTOR else MAX_CORRECTION_RAD) * np.tanh(raw)
