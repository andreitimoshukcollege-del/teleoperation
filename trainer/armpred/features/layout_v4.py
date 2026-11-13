"""Layout v4: the FR3 Duo's window (docs/adr/0028 decision 1), from nothing but the predictor's calls.

Layout v3 (``layout.py``) is the JetRover's and does not change. v4 keeps its row structure and runs **per arm,
with weights shared across arms and joints**:

- **One row per motor**, in the profile's order: each arm's seven joints, then its hand (16 on the FR3 Duo). A
  hand's value is its 0..1 opening fraction times ``HAND_MODEL_PER_FRACTION`` in model units.
- **Each row (``JOINT_FEATURES`` = 253)**, relative to the joint's newest fresh reading ``v`` and squashed by
  ``asinh(x / scale)``:

  | Part | Count | Content |
  |---|---|---|
  | prior | 2 | the ``franka-replay`` prediction at T; its slope over the last 40 ms |
  | commanded path | 80 x 2 | newest first, every 20 ms of effective time from T + 0.2 s back: the target in force, mask |
  | readings | 16 x 3 | the newest fresh readings: time, value, mask |
  | innovations | 8 x 4 | the prior's error on each fresh reading before it arrived: time, error, horizon, mask |
  | identity | 8 | which joint of its arm (one-hot, joints 1-7, then the hand) |
  | angle | 2 | sin, cos of ``v`` |
  | reference | 1 | 1 when ``v`` is a reading, 0 when it is the prior |

  The scales are the FR3's: errors are milliradians, not tenths (``ANGLE_SCALE_RAD``, ``INNOVATION_SCALE_RAD``).
- **One arm block per arm (``ARM_FEATURES`` = 125):**
  - sin/cos of the arm's seven newest readings;
  - gravity's direction in the arm's mounted base frame (fixed by the mount, so the two arms differ);
  - staleness;
  - fresh readings in the last second / 50 (the FR3's stream is 50 Hz of fresh readings);
  - the longest gap between fresh readings in 2 s;
  - the newest command's effective time;
  - the arm's hand command and its change over 0.5 s;
  - how full the command window is;
  - then v3's network block, unchanged.

  Row r is paired with its own arm's block.
- **The caps** (pre-registered, docs/adr/0028 decision 1): an arm joint's correction is at most 3 degrees, a
  hand's 0.5 in model units (half its travel).

Never in the window: the requested speed, the network profile or true delays, the payload, anything in a
simulated session's latents.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..baselines.franka_replay import FrankaReplayPredictor, TrajectoryShaperConfig, franka_joint_config, shaper_for_profile
from ..baselines.types import JointPredictorConfig, JointSample
from ..contract.chain_profile import SerialChainProfile, load_chain_profile
from ..net import forecast as FC
from . import layout as V3

LAYOUT_ID = "v4"
HAND_MODEL_PER_FRACTION = 1.0

N_PATH, PATH_STEP_S, PATH_AHEAD_S = V3.N_PATH, V3.PATH_STEP_S, V3.PATH_AHEAD_S
N_READINGS, N_INNOVATIONS = V3.N_READINGS, V3.N_INNOVATIONS
TIME_SCALE_S = V3.TIME_SCALE_S
ANGLE_SCALE_RAD = 0.05
INNOVATION_SCALE_RAD = 0.005
SLOPE_WINDOW_S, SLOPE_SCALE_RAD_S = V3.SLOPE_WINDOW_S, V3.SLOPE_SCALE_RAD_S
COMMAND_RING, WINDOW_FULL = V3.COMMAND_RING, V3.WINDOW_FULL
FRESH_RATE_HZ = 50.0
MAX_CORRECTION_RAD = math.radians(3.0)
HAND_MAX_CORRECTION = 0.5
FALLBACK_STALENESS_S = V3.FALLBACK_STALENESS_S
ARM_JOINTS = 7
IDENTITY = ARM_JOINTS + 1

JOINT_FEATURES = 2 + N_PATH * 2 + N_READINGS * 3 + N_INNOVATIONS * 4 + IDENTITY + 2 + 1
# The hand's identity entry: the last of the identity block. The head's cap reads it, as v3's reads the gripper's.
HAND_FLAG_INDEX = 2 + N_PATH * 2 + N_READINGS * 3 + N_INNOVATIONS * 4 + IDENTITY - 1
NETWORK_FEATURES = V3.NETWORK_FEATURES
BASE_ARM_FEATURES = 2 * ARM_JOINTS + 3 + 7
ARM_FEATURES = BASE_ARM_FEATURES + NETWORK_FEATURES

_DEFAULT_PROFILE: Optional[SerialChainProfile] = None


def default_profile() -> SerialChainProfile:
    """The FR3 Duo, from the trainer's copy of its profile (``contract/profiles/fr3-duo.json``)."""
    global _DEFAULT_PROFILE
    if _DEFAULT_PROFILE is None:
        from ..sim.fr3 import FR3_PROFILE

        _DEFAULT_PROFILE = load_chain_profile(FR3_PROFILE)
    return _DEFAULT_PROFILE


def layout_id(aware: bool = False) -> str:
    if aware:
        raise ValueError("layout v4 has no speed-aware variant: an FR3 command carries no speed")
    return LAYOUT_ID


def arm_features(aware: bool = False) -> int:
    layout_id(aware)
    return ARM_FEATURES


def predicted_motors(profile: Optional[SerialChainProfile] = None) -> Tuple[int, ...]:
    return tuple((profile or default_profile()).motors)


PREDICTED_MOTORS: Tuple[int, ...] = ()  # filled on first use: predicted_motors() (import order: the profile loads lazily)


def _rows(profile: SerialChainProfile):
    """Per row: motor id, arm index, identity index (0-6 a joint, 7 the hand)."""
    out = []
    for a, arm in enumerate(profile.arms):
        for i, j in enumerate(arm.joints):
            out.append((j.motor_id, a, i))
        if arm.gripper is not None:
            out.append((arm.gripper.motor_id, a, ARM_JOINTS))
    return out


def to_model(motor: int, value: float, profile: Optional[SerialChainProfile] = None) -> float:
    """A wire value (rad; a hand's 0..1 fraction) in model units."""
    return value * HAND_MODEL_PER_FRACTION if motor in (profile or default_profile()).hand_motors else value


def to_wire(motor: int, value: float, profile: Optional[SerialChainProfile] = None) -> float:
    return value / HAND_MODEL_PER_FRACTION if motor in (profile or default_profile()).hand_motors else value


def is_arm_row(profile: Optional[SerialChainProfile] = None) -> np.ndarray:
    """Which rows are arm joints (scored by the gate) rather than hands."""
    return np.array([ident < ARM_JOINTS for _, _, ident in _rows(profile or default_profile())])


def default_prior_config(tps: int, profile: Optional[SerialChainProfile] = None) -> JointPredictorConfig:
    p = profile or default_profile()
    return franka_joint_config(tps, max_joints=len(p.motors), instant=tuple(p.hand_motors))


def correction(raw: np.ndarray, hand: bool = False) -> np.ndarray:
    return (HAND_MAX_CORRECTION if hand else MAX_CORRECTION_RAD) * np.tanh(raw)


def _sq(x: float, scale: float) -> float:
    return math.asinh(x / scale)


@dataclass
class Window:
    joints: np.ndarray  # float32 [rows, JOINT_FEATURES], profile order
    arms: np.ndarray  # float32 [arms, ARM_FEATURES]
    arm_of_row: np.ndarray  # int [rows]
    prior: np.ndarray  # float64 [rows], model units
    newest: np.ndarray  # float64 [rows], model units
    valid: bool
    staleness_s: float
    reason: str = ""

    @property
    def arm(self) -> np.ndarray:
        """The first arm's block (for code that reads one arm block; use ``arm_row``)."""
        return self.arms[0]

    def arm_row(self, row: int) -> np.ndarray:
        return self.arms[self.arm_of_row[row]]


class FeatureState:
    """Layout v4's window, updated call by call, deterministic."""

    def __init__(self, prior_config: JointPredictorConfig, aware: bool = False, profile: Optional[SerialChainProfile] = None):
        layout_id(aware)
        self.profile = profile or default_profile()
        self.config = prior_config
        self.tps = prior_config.ticks_per_second
        self.shaper: TrajectoryShaperConfig = shaper_for_profile(self.profile)
        self.rows = _rows(self.profile)
        self.motors = [m for m, _, _ in self.rows]
        self.hands = set(self.profile.hand_motors)
        self.arm_count = len(self.profile.arms)
        self.arm_motors = [[m for m, a, i in self.rows if a == k and i < ARM_JOINTS] for k in range(self.arm_count)]
        self.hand_of_arm = [next((m for m, a, i in self.rows if a == k and i == ARM_JOINTS), None) for k in range(self.arm_count)]
        self.gravity = [arm.gravity_in_base() for arm in self.profile.arms]
        self.prior = FrankaReplayPredictor(prior_config, self.shaper)
        self.reset()

    def _m(self, motor: int, value: float) -> float:
        return value * HAND_MODEL_PER_FRACTION if motor in self.hands else value

    def reset(self) -> None:
        self.prior.reset()
        self.commands: Deque[Tuple[int, Dict[int, float]]] = deque(maxlen=COMMAND_RING)
        self.readings: Dict[int, Deque[Tuple[int, float]]] = {m: deque(maxlen=N_READINGS) for m in self.motors}
        self.innovations: Dict[int, Deque[Tuple[int, float, int]]] = {m: deque(maxlen=N_INNOVATIONS) for m in self.motors}
        self.fresh_times: Deque[int] = deque(maxlen=128)
        self.last_fresh: Optional[int] = None
        self.newest: Dict[int, float] = {}
        self.last_command_eff: Optional[int] = None
        self.forecast: Optional[Tuple[int, np.ndarray]] = None

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed: float = 0.0) -> None:
        self.prior.observe_command(effective_ticks, targets, 0.0)
        if self.last_command_eff is not None and effective_ticks < self.last_command_eff:
            return
        self.last_command_eff = effective_ticks
        self.commands.append((int(effective_ticks), {t.motor_id: float(t.value) for t in targets}))

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        fresh = self.last_fresh is None or sensed_ticks > self.last_fresh
        if fresh:
            ok, predicted = self.prior.try_predict(sensed_ticks)
            prior_state = self.prior.state_ticks if self.prior.has_state else None
            if ok:
                by_motor = {s.motor_id: s.value for s in predicted}
                for j in joints:
                    if j.motor_id in self.innovations and j.sensed and j.motor_id in by_motor:
                        horizon = sensed_ticks - prior_state if prior_state is not None else 0
                        error = self._m(j.motor_id, float(j.value)) - self._m(j.motor_id, by_motor[j.motor_id])
                        self.innovations[j.motor_id].append((int(sensed_ticks), error, int(horizon)))
            for j in joints:
                if j.motor_id in self.readings and j.sensed:
                    value = self._m(j.motor_id, float(j.value))
                    self.readings[j.motor_id].append((int(sensed_ticks), value))
                    self.newest[j.motor_id] = value
            self.last_fresh = int(sensed_ticks)
            self.fresh_times.append(int(sensed_ticks))
        self.prior.observe_state(sensed_ticks, joints)

    def observe_forecast(self, stream: "FC.ForecastStream", index: int) -> None:
        self.forecast = (int(stream.t[index]), V3.encode_forecast(stream, index))

    def window(self, target_ticks: int) -> Window:
        tps = float(self.tps)
        rows = len(self.rows)
        joints = np.zeros((rows, JOINT_FEATURES), dtype=np.float32)
        arms = np.zeros((self.arm_count, ARM_FEATURES), dtype=np.float32)
        arm_of_row = np.array([a for _, a, _ in self.rows], dtype=np.int64)
        prior = np.zeros(rows)
        newest = np.zeros(rows)
        ok, predicted = self.prior.try_predict(target_ticks)
        arm_joint_motors = [m for k in range(self.arm_count) for m in self.arm_motors[k]]
        if not ok or self.last_fresh is None or any(m not in self.newest for m in arm_joint_motors):
            return Window(joints, arms, arm_of_row, prior, newest, False, float("inf"), "no fresh reading yet")
        p_now = {s.motor_id: self._m(s.motor_id, s.value) for s in predicted}
        _, before = self.prior.try_predict(target_ticks - int(SLOPE_WINDOW_S * tps))
        p_before = {s.motor_id: self._m(s.motor_id, s.value) for s in before}
        staleness = (target_ticks - self.last_fresh) / tps
        cmds = list(self.commands)
        effs = np.array([c[0] for c in cmds], dtype=np.int64)
        grid = target_ticks + int(round(PATH_AHEAD_S * tps)) - np.round(np.arange(N_PATH) * PATH_STEP_S * tps).astype(np.int64)
        in_force = np.searchsorted(effs, grid, side="right") - 1 if len(effs) else np.full(N_PATH, -1)
        for row, (m, a, ident) in enumerate(self.rows):
            sensed = m in self.newest
            v = self.newest[m] if sensed else p_now.get(m, 0.0)
            prior[row] = p_now.get(m, v)
            newest[row] = v
            f = joints[row]
            k = 0
            f[k] = _sq(prior[row] - v, ANGLE_SCALE_RAD)
            f[k + 1] = (prior[row] - p_before.get(m, prior[row])) / SLOPE_WINDOW_S / SLOPE_SCALE_RAD_S
            k += 2
            last_index, last_value = -2, 0.0
            for i in range(N_PATH):
                c = int(in_force[i])
                while c >= 0 and m not in cmds[c][1]:
                    c -= 1
                if c >= 0:
                    if c != last_index:
                        last_index, last_value = c, _sq(self._m(m, cmds[c][1][m]) - v, ANGLE_SCALE_RAD)
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
            f[k + ident] = 1.0
            k += IDENTITY
            f[k] = math.sin(v)
            f[k + 1] = math.cos(v)
            f[k + 2] = 1.0 if sensed else 0.0
        window_start = target_ticks - 1.0 * tps
        fresh = [t for t in self.fresh_times if t <= target_ticks]
        recent = [t for t in fresh if t >= target_ticks - 2.0 * tps]
        gaps = np.diff(recent) if len(recent) > 1 else np.zeros(1)
        for a in range(self.arm_count):
            block = arms[a]
            k = 0
            for m in self.arm_motors[a]:
                block[k], block[k + 1] = math.sin(self.newest[m]), math.cos(self.newest[m])
                k += 2
            k = 2 * ARM_JOINTS
            block[k:k + 3] = self.gravity[a]
            k += 3
            block[k] = staleness / TIME_SCALE_S
            block[k + 1] = sum(1 for t in fresh if t >= window_start) / FRESH_RATE_HZ
            block[k + 2] = float(gaps.max()) / tps / TIME_SCALE_S
            if cmds:
                block[k + 3] = (cmds[-1][0] - target_ticks) / tps / TIME_SCALE_S
                hand = self.hand_of_arm[a]
                if hand is not None:
                    values = [c[1][hand] for c in cmds if hand in c[1]]
                    if values:
                        block[k + 4] = values[-1]
                        old = [c[1][hand] for c in cmds if hand in c[1] and c[0] <= target_ticks - 0.5 * tps]
                        block[k + 5] = values[-1] - (old[-1] if old else values[0])
                block[k + 6] = min(len(cmds), WINDOW_FULL) / WINDOW_FULL
            k = BASE_ARM_FEATURES
            if self.forecast is not None:
                issued, encoded = self.forecast
                block[k:k + NETWORK_FEATURES - 1] = encoded
                block[k + NETWORK_FEATURES - 1] = (target_ticks - issued) / tps / TIME_SCALE_S
        valid = staleness <= FALLBACK_STALENESS_S
        return Window(joints, arms, arm_of_row, prior, newest, valid, staleness, "" if valid else "stale")
