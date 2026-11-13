"""Copies of ``Prediction/JointTrajectoryShaper.cs``, ``Types/TrajectoryShaperConfig.cs`` and
``Prediction/FrankaReplayPredictor.cs`` (docs/adr/0027), pinned to Core by
``contract/golden/joint-shaper-v1.json`` and ``franka-replay-v1.json``.

The shaper works in double precision, as Core's does, operation for operation in the same order; only
a ``JointSample`` value is a C# ``float`` (``f32``). ``SerialChainShaping.ForProfile`` (Teleop.RobotArm)
has its copy in ``shaper_for_profile``, so the trainer's FR3 simulator shapes as the robot's backend does.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from .types import JointPredictorConfig, JointSample, PredictorDiagnostics, f32

DEFAULT_STEP_SECONDS = 0.001
DEFAULT_VELOCITY_SCALE = 0.9
DEFAULT_ACCELERATION_SCALE = 0.9
DEFAULT_JERK_SCALE = 0.9
DEFAULT_BRAKING_SCALE = 0.5
DEFAULT_SETTLE_SECONDS = 0.02
DEFAULT_POSITION_MARGIN = 0.02
DEFAULT_FINGER_DEADBAND = 0.02  # SerialChainShaping.DefaultFingerDeadband


@dataclass(frozen=True)
class ShapedJointLimits:
    motor_id: int
    is_finger: bool
    lower: float
    upper: float
    soft_lower: float
    soft_upper: float
    velocity: float
    acceleration: float
    jerk: float
    position_based_deceleration: float = 0.0
    position_based_velocity_offset: float = 0.0
    finger_speed: float = 0.0
    finger_deadband: float = 0.0

    @staticmethod
    def joint(motor_id: int, lower: float, upper: float, soft_lower: float, soft_upper: float, velocity: float,
              acceleration: float, jerk: float, pbv_deceleration: float = 0.0, pbv_offset: float = 0.0) -> "ShapedJointLimits":
        if not lower < upper:
            raise ValueError(f"motor {motor_id}: the lower limit must be below the upper")
        if not (lower <= soft_lower < soft_upper <= upper):
            raise ValueError(f"motor {motor_id}: the soft limits must be inside the hard limits")
        if not (velocity > 0 and acceleration > 0 and jerk > 0):
            raise ValueError(f"motor {motor_id}: velocity, acceleration and jerk must be positive")
        return ShapedJointLimits(motor_id, False, lower, upper, soft_lower, soft_upper, velocity, acceleration, jerk,
                                 pbv_deceleration, pbv_offset, 0.0, 0.0)

    @staticmethod
    def finger(motor_id: int, speed: float, deadband: float) -> "ShapedJointLimits":
        if not (speed > 0 and math.isfinite(speed)):
            raise ValueError(f"motor {motor_id}: the finger speed must be positive and finite")
        if not (0 <= deadband < 1):
            raise ValueError(f"motor {motor_id}: the finger deadband must be in [0, 1)")
        return ShapedJointLimits(motor_id, True, 0.0, 1.0, 0.0, 1.0, speed, 0.0, 0.0, 0.0, 0.0, speed, deadband)


@dataclass(frozen=True)
class TrajectoryShaperConfig:
    joints: Tuple[ShapedJointLimits, ...]
    step_seconds: float = DEFAULT_STEP_SECONDS
    velocity_scale: float = DEFAULT_VELOCITY_SCALE
    acceleration_scale: float = DEFAULT_ACCELERATION_SCALE
    jerk_scale: float = DEFAULT_JERK_SCALE
    braking_scale: float = DEFAULT_BRAKING_SCALE
    settle_seconds: float = DEFAULT_SETTLE_SECONDS
    position_margin: float = DEFAULT_POSITION_MARGIN

    def __post_init__(self) -> None:
        ids = [j.motor_id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError("a motor is listed twice")
        if not (self.settle_seconds > self.step_seconds):
            raise ValueError("settle_seconds must be longer than one step")

    def index_of(self, motor_id: int) -> int:
        for i, j in enumerate(self.joints):
            if j.motor_id == motor_id:
                return i
        return -1

    @staticmethod
    def from_golden(d: dict) -> "TrajectoryShaperConfig":
        """The ``shaper`` object of a golden file (TrainerGoldenBuilder.WriteShaperConfig)."""
        joints = tuple(ShapedJointLimits(
            m["motorId"], bool(m["isFinger"]), m["lower"], m["upper"], m["softLower"], m["softUpper"], m["velocity"],
            m["acceleration"], m["jerk"], m["positionBasedDeceleration"], m["positionBasedVelocityOffset"],
            m["fingerSpeed"], m["fingerDeadband"]) for m in d["motors"])
        return TrajectoryShaperConfig(joints, d["stepSeconds"], d["velocityScale"], d["accelerationScale"], d["jerkScale"],
                                      d["brakingScale"], d["settleSeconds"], d["positionMargin"])


class ShapedState:
    """``ShapedJointState``: position, velocity, acceleration, and a finger's move."""

    __slots__ = ("q", "v", "a", "moving", "to", "last_to")

    def __init__(self, q: float = 0.0, v: float = 0.0, a: float = 0.0, moving: bool = False, to: float = 0.0, last_to: float = 0.0):
        self.q, self.v, self.a, self.moving, self.to, self.last_to = q, v, a, moving, to, last_to

    @staticmethod
    def at_rest(q: float) -> "ShapedState":
        return ShapedState(q, 0.0, 0.0, False, q, q)

    def copy(self) -> "ShapedState":
        return ShapedState(self.q, self.v, self.a, self.moving, self.to, self.last_to)


def step(s: ShapedState, target: float, l: ShapedJointLimits, c: TrajectoryShaperConfig) -> None:
    """``JointTrajectoryShaper.Step``: one step of ``s`` toward ``target``, in place."""
    if l.is_finger:
        _step_finger(s, target, l, c.step_seconds)
    else:
        _step_joint(s, target, l, c)


def _step_joint(s: ShapedState, target: float, l: ShapedJointLimits, c: TrajectoryShaperConfig) -> None:
    dt = c.step_seconds
    lo, hi = l.soft_lower + c.position_margin, l.soft_upper - c.position_margin
    if lo > hi:
        lo = hi = 0.5 * (l.soft_lower + l.soft_upper)
    goal = lo if target < lo else hi if target > hi else target

    v_max = l.velocity * c.velocity_scale
    a_max = l.acceleration * c.acceleration_scale
    j_step = l.jerk * c.jerk_scale * dt

    v_up, v_down = v_max, v_max
    if l.position_based_deceleration > 0:
        up = c.velocity_scale * (l.position_based_velocity_offset + math.sqrt(2.0 * l.position_based_deceleration * max(0.0, l.upper - s.q)))
        down = c.velocity_scale * (l.position_based_velocity_offset + math.sqrt(2.0 * l.position_based_deceleration * max(0.0, s.q - l.lower)))
        if up < v_up:
            v_up = up
        if down < v_down:
            v_down = down

    braking_acceleration = a_max * c.braking_scale
    if l.position_based_deceleration > 0:
        position_based = c.velocity_scale * c.velocity_scale * l.position_based_deceleration
        if position_based < braking_acceleration:
            braking_acceleration = position_based

    e = goal - s.q
    distance = abs(e)
    braking = math.sqrt(2.0 * braking_acceleration * distance)
    settling = distance / c.settle_seconds
    v_ref = (-1.0 if e < 0 else 1.0 if e > 0 else 0.0) * (braking if braking < settling else settling)
    if v_ref > v_up:
        v_ref = v_up
    if v_ref < -v_down:
        v_ref = -v_down

    a = (v_ref - s.v) / dt
    if a > a_max:
        a = a_max
    if a < -a_max:
        a = -a_max
    if a > s.a + j_step:
        a = s.a + j_step
    if a < s.a - j_step:
        a = s.a - j_step

    headroom_gain = l.jerk * c.jerk_scale / a_max
    safe_max, safe_min = headroom_gain * (v_up - s.v), headroom_gain * (-v_down - s.v)
    if safe_max > a_max:
        safe_max = a_max
    if safe_min < -a_max:
        safe_min = -a_max
    if a > safe_max:
        a = safe_max
    if a < safe_min:
        a = safe_min

    v = s.v + a * dt
    if v > v_up:
        v = v_up
    if v < -v_down:
        v = -v_down

    s.a = (v - s.v) / dt
    s.v = v
    s.q += v * dt


def _step_finger(s: ShapedState, target: float, l: ShapedJointLimits, dt: float) -> None:
    goal = 0.0 if target < 0 else 1.0 if target > 1 else target
    if not s.moving and abs(goal - s.last_to) > l.finger_deadband:
        s.moving = True
        s.to = goal
        s.last_to = goal
    if not s.moving:
        s.v = 0.0
        s.a = 0.0
        return
    step_size = l.finger_speed * dt
    remaining = s.to - s.q
    if abs(remaining) <= step_size:
        s.q = s.to
        s.v = 0.0
        s.moving = False
    else:
        s.v = l.finger_speed if remaining > 0 else -l.finger_speed
        s.q += step_size if remaining > 0 else -step_size
    s.a = 0.0


def shaper_for_profile(profile: "SerialChainProfile", finger_deadband: float = DEFAULT_FINGER_DEADBAND) -> TrajectoryShaperConfig:
    """``SerialChainShaping.ForProfile``: every arm joint and gripper, in profile order, at the default shares."""
    limits: List[ShapedJointLimits] = []
    for arm in profile.arms:
        for j in arm.joints:
            limits.append(ShapedJointLimits.joint(j.motor_id, j.lower, j.upper, j.soft_lower, j.soft_upper, j.velocity,
                                                  j.acceleration, j.jerk, j.pbv_deceleration, j.pbv_offset))
        if arm.gripper is not None:
            limits.append(ShapedJointLimits.finger(arm.gripper.motor_id, arm.gripper.speed / arm.gripper.finger_travel, finger_deadband))
    return TrajectoryShaperConfig(tuple(limits))


CHECKPOINT_EVERY = 10


class _Slot:
    __slots__ = ("motor", "limit", "commands", "has_base", "base", "anchor", "anchor_ticks", "has_anchor", "checkpoints")

    def __init__(self, motor: int, limit: int, capacity: int):
        self.motor = motor
        self.limit = limit
        self.commands: Deque[Tuple[int, float]] = deque()
        self.has_base = False
        self.base = 0.0
        self.anchor = ShapedState()
        self.anchor_ticks = 0
        self.has_anchor = False
        # The replay from the anchor, every CHECKPOINT_EVERY whole steps: (steps done, state, cursor, target). A
        # speed-up only (not in Core): resuming from one runs exactly the steps a replay from the anchor would.
        self.checkpoints: List[Tuple[int, "ShapedState", int, float]] = []


class FrankaReplayPredictor:
    """``FrankaReplayPredictor``: the commands replayed through the shaper from a re-anchored state."""

    def __init__(self, config: JointPredictorConfig, shaper: TrajectoryShaperConfig):
        self.config = config
        self.shaper = shaper
        self.max_joints = config.max_joints
        self.capacity = config.command_history_capacity
        self.step_ticks = max(1, int(round(shaper.step_seconds * config.ticks_per_second)))
        self.reset()

    def reset(self) -> None:
        self.slots: List[_Slot] = []
        self.ids: List[int] = []
        self.slot_of: List[int] = []
        self.measured: List[float] = []
        self.sensed: List[bool] = []
        self.has_state = False
        self.state_ticks = 0
        self.last_command_ticks: Optional[int] = None
        self.accepted = 0
        self.rejected = 0
        self.last_horizon = 0

    @property
    def diagnostics(self) -> PredictorDiagnostics:
        return PredictorDiagnostics(self.last_horizon, self.state_ticks if self.has_state else 0, self.accepted, self.rejected)

    def _slot(self, motor: int) -> int:
        for k, s in enumerate(self.slots):
            if s.motor == motor:
                return k
        if len(self.slots) == self.max_joints:
            return -1
        self.slots.append(_Slot(motor, self.shaper.index_of(motor), self.capacity))
        return len(self.slots) - 1

    def observe_command(self, effective_ticks: int, targets: Sequence[JointSample], speed: float = 0.0) -> None:
        if self.last_command_ticks is not None and effective_ticks < self.last_command_ticks:
            self.rejected += 1
            return
        self.last_command_ticks = effective_ticks
        for t in targets:
            k = self._slot(t.motor_id)
            if k < 0 or self.slots[k].limit < 0:
                continue
            s = self.slots[k]
            if len(s.commands) == self.capacity:
                evicted = s.commands.popleft()
                s.has_base = True
                s.base = evicted[1]
                # The pushed-out target now holds before the new oldest: replays from an older anchor change.
                if s.anchor_ticks < evicted[0]:
                    s.checkpoints.clear()
                # The cursor counts from the oldest kept, which moved up by one.
                s.checkpoints = [(n, st, c - 1, tg) for n, st, c, tg in s.checkpoints if c >= 1]
            s.commands.append((int(effective_ticks), float(t.value)))
            # A step at or after this command's time steers toward it now: checkpoints past it are stale.
            if s.checkpoints and s.has_anchor:
                cut = int(effective_ticks) - s.anchor_ticks
                s.checkpoints = [cp for cp in s.checkpoints if cp[0] * self.step_ticks <= cut]
        self.accepted += 1

    def observe_state(self, sensed_ticks: int, joints: Sequence[JointSample]) -> None:
        if self.has_state and sensed_ticks < self.state_ticks:
            self.rejected += 1
            return
        n = min(len(joints), self.max_joints)
        ids, slot_of, measured, sensed = [], [], [], []
        for joint in joints[:n]:
            k = self._slot(joint.motor_id)
            if k < 0:
                continue
            s = self.slots[k]
            if s.limit >= 0:
                limits = self.shaper.joints[s.limit]
                if not s.has_anchor:
                    state = ShapedState.at_rest(float(joint.value))
                    if not s.has_base:
                        s.has_base = True
                        s.base = float(joint.value)
                elif sensed_ticks - s.anchor_ticks > self.config.max_horizon_ticks:
                    state = ShapedState.at_rest(float(joint.value) if joint.sensed else s.anchor.q)
                    state.last_to = self._target_at(s, sensed_ticks)[0]
                else:
                    state = s.anchor.copy()
                    self._advance(s, limits, state, s.anchor_ticks, sensed_ticks)
                if joint.sensed:
                    state.q = float(joint.value)
                s.anchor = state
                s.anchor_ticks = int(sensed_ticks)
                s.has_anchor = True
                s.checkpoints = []
            ids.append(joint.motor_id)
            slot_of.append(k)
            measured.append(float(joint.value))
            sensed.append(bool(joint.sensed))
        self.ids, self.slot_of, self.measured, self.sensed = ids, slot_of, measured, sensed
        self.state_ticks = int(sensed_ticks)
        self.has_state = True
        self.accepted += 1

    def try_predict(self, target_ticks: int) -> Tuple[bool, List[JointSample]]:
        if not self.has_state:
            return False, []
        end = target_ticks
        if end < self.state_ticks:
            end = self.state_ticks
        if end > self.state_ticks + self.config.max_horizon_ticks:
            end = self.state_ticks + self.config.max_horizon_ticks
        self.last_horizon = end - self.state_ticks
        out = []
        for j, k in enumerate(self.slot_of):
            s = self.slots[k]
            value = self.measured[j]
            if s.limit >= 0 and s.has_anchor:
                state = s.anchor.copy()
                self._advance(s, self.shaper.joints[s.limit], state, s.anchor_ticks, end)
                value = f32(state.q)
            out.append(JointSample(self.ids[j], value, self.sensed[j]))
        return True, out

    def _advance(self, s: _Slot, limits: ShapedJointLimits, state: ShapedState, start: int, end: int) -> None:
        if end <= start:
            return
        span = end - start
        whole = span // self.step_ticks
        remainder = span - whole * self.step_ticks
        commands = s.commands
        size = len(commands)
        first = 0
        use_checkpoints = start == s.anchor_ticks and state is not s.anchor
        if use_checkpoints:
            best = None
            for cp in s.checkpoints:
                if cp[0] <= whole and (best is None or cp[0] > best[0]):
                    best = cp
            if best is not None:
                first, saved, cursor, target = best
                state.q, state.v, state.a, state.moving, state.to, state.last_to = saved.q, saved.v, saved.a, saved.moving, saved.to, saved.last_to
        if first == 0:
            target, cursor = self._target_at(s, start)
        for k in range(first, whole):
            t = start + k * self.step_ticks
            while cursor < size and commands[cursor][0] <= t:
                target = commands[cursor][1]
                cursor += 1
            step(state, target, limits, self.shaper)
            if use_checkpoints and (k + 1) % CHECKPOINT_EVERY == 0 and (not s.checkpoints or s.checkpoints[-1][0] < k + 1):
                s.checkpoints.append((k + 1, state.copy(), cursor, target))
        if remainder > 0:
            t = start + whole * self.step_ticks
            while cursor < size and commands[cursor][0] <= t:
                target = commands[cursor][1]
                cursor += 1
            nxt = state.copy()
            step(nxt, target, limits, self.shaper)
            u = remainder / self.step_ticks
            state.q += (nxt.q - state.q) * u
            state.v += (nxt.v - state.v) * u
            state.a += (nxt.a - state.a) * u

    def _target_at(self, s: _Slot, t: int) -> Tuple[float, int]:
        target = s.base if s.has_base else s.anchor.q
        cursor = 0
        for when, value in s.commands:
            if when > t:
                break
            target = value
            cursor += 1
        return target, cursor


def franka_joint_config(ticks_per_second: int, max_joints: int = 16, max_horizon_ms: float = 1000.0,
                        capacity: int = 96, instant: Tuple[int, ...] = (8, 18)) -> JointPredictorConfig:
    """The headset's config for an FR3 Duo: 16 motors, the hands instant for predictors without a finger model."""
    return JointPredictorConfig(
        max_joints=max_joints, command_history_capacity=capacity, max_joint_speed_rad_per_sec=2.0,
        instant_motor_ids=instant, max_horizon_ticks=int(max_horizon_ms / 1000.0 * ticks_per_second),
        ticks_per_second=ticks_per_second)
