"""A copy of Core's ``Prediction/JointTrajectoryShaper.cs`` and ``Types/TrajectoryShaperConfig.cs`` (docs/adr/0027),
for the Isaac bridge (docs/adr/0031 decision 2): the trainer's copy (``trainer/armpred/baselines/franka_replay.py``),
taken as text because nothing in the repo imports the trainer.

Pinned to Core by ``golden/joint-shaper-v1.json`` (the trainer's golden, byte for byte; an Eval test holds the two
equal). Double precision, operation for operation in Core's order: a mismatch is a bug in whichever moved.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

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


def shaper_for_profile(profile, finger_deadband: float = DEFAULT_FINGER_DEADBAND) -> TrajectoryShaperConfig:
    """``SerialChainShaping.ForProfile``: every arm joint and gripper, in profile order, at the default shares."""
    limits: List[ShapedJointLimits] = []
    for arm in profile.arms:
        for j in arm.joints:
            limits.append(ShapedJointLimits.joint(j.motor_id, j.lower, j.upper, j.soft_lower, j.soft_upper, j.velocity,
                                                  j.acceleration, j.jerk, j.pbv_deceleration, j.pbv_offset))
        if arm.gripper is not None:
            limits.append(ShapedJointLimits.finger(arm.gripper.motor_id, arm.gripper.speed / arm.gripper.finger_travel, finger_deadband))
    return TrajectoryShaperConfig(tuple(limits))
