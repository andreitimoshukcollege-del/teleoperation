"""Copies of ``Types/JointSample.cs`` and ``Types/JointPredictorConfig.cs``.

C# ``float`` values are held as Python floats that are always float32-representable: every C#
single-precision operation is computed in double and rounded with ``f32``. That is exact for +, -, *
and /, because double carries more than 2 * 24 + 2 bits (double rounding is then innocuous).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence, Tuple

import numpy as np

LONG_MIN = -(2**63)
LONG_MAX = 2**63 - 1


def f32(x: float) -> float:
    """Rounds a double to the nearest float32, as C# stores a ``float``."""
    return float(np.float32(x))


@dataclass(frozen=True)
class JointSample:
    motor_id: int
    value: float  # float32-representable
    sensed: bool = False


# JointPredictorConfig.JetRoverPulsesPerRadian: 1000 pulses span 240 degrees (double).
JETROVER_PULSES_PER_RADIAN = 1000.0 / (240.0 * math.pi / 180.0)
DEFAULT_RELAY_MIN_MOVE_SECONDS = f32(0.05)
DEFAULT_RELAY_DEADBAND_RAD = f32(5.0 / JETROVER_PULSES_PER_RADIAN)
DEFAULT_RELAY_MOVE_SPEED_RAD_PER_SEC = f32(2200.0 / JETROVER_PULSES_PER_RADIAN)
# relay-replay-servo's servo (JointPredictorConfig.DefaultServo*): guesses until sysid fits them.
DEFAULT_SERVO_MAX_SPEED_RAD_PER_SEC = f32(5.0)
DEFAULT_SERVO_LAG_SECONDS = f32(0.02)
DEFAULT_SERVO_PIPELINE_DELAY_SECONDS = f32(0.02)
DEFAULT_SERVO_FIRST_MOVE_FROM_MEASURED = True
DEFAULT_SERVO_STEP_SECONDS = f32(0.002)


@dataclass(frozen=True)
class JointPredictorConfig:
    max_joints: int
    command_history_capacity: int
    max_joint_speed_rad_per_sec: float
    instant_motor_ids: Tuple[int, ...]
    max_horizon_ticks: int
    ticks_per_second: int
    scaled_motor_ids: Tuple[int, ...] = ()
    speed_scales: Tuple[float, ...] = ()
    relay_min_move_seconds: float = DEFAULT_RELAY_MIN_MOVE_SECONDS
    relay_deadband_rad: float = DEFAULT_RELAY_DEADBAND_RAD
    relay_move_speed_rad_per_sec: float = DEFAULT_RELAY_MOVE_SPEED_RAD_PER_SEC
    sensed_age_motor_ids: Tuple[int, ...] = ()
    sensed_age_seconds: Tuple[float, ...] = ()
    servo_max_speed_rad_per_sec: float = DEFAULT_SERVO_MAX_SPEED_RAD_PER_SEC  # math.inf: no cap
    servo_lag_seconds: float = DEFAULT_SERVO_LAG_SECONDS
    servo_pipeline_delay_seconds: float = DEFAULT_SERVO_PIPELINE_DELAY_SECONDS
    servo_first_move_from_measured: bool = DEFAULT_SERVO_FIRST_MOVE_FROM_MEASURED
    servo_step_seconds: float = DEFAULT_SERVO_STEP_SECONDS

    def __post_init__(self) -> None:
        if len(self.sensed_age_motor_ids) != len(self.sensed_age_seconds):
            raise ValueError("sensed_age_motor_ids and sensed_age_seconds must be the same length")
        if any(not (a >= 0.0) for a in self.sensed_age_seconds):
            raise ValueError("every sensed age must be zero or positive")
        if not (self.relay_min_move_seconds >= 0.0):
            raise ValueError("relay_min_move_seconds")
        if not (self.relay_deadband_rad >= 0.0):
            raise ValueError("relay_deadband_rad")
        if not (self.relay_move_speed_rad_per_sec > 0.0):
            raise ValueError("relay_move_speed_rad_per_sec")
        if len(self.scaled_motor_ids) != len(self.speed_scales):
            raise ValueError("scaled_motor_ids and speed_scales must be the same length")
        if any(not (s > 0.0) for s in self.speed_scales):
            raise ValueError("every speed scale must be positive")
        if self.max_joints <= 0 or self.command_history_capacity <= 0:
            raise ValueError("max_joints and command_history_capacity must be positive")
        if not (self.max_joint_speed_rad_per_sec > 0.0):
            raise ValueError("max_joint_speed_rad_per_sec")
        if self.max_horizon_ticks < 0 or self.ticks_per_second <= 0:
            raise ValueError("max_horizon_ticks / ticks_per_second")
        if not (self.servo_max_speed_rad_per_sec > 0.0):
            raise ValueError("servo_max_speed_rad_per_sec")
        for name in ("servo_lag_seconds", "servo_pipeline_delay_seconds"):
            if not (getattr(self, name) >= 0.0) or math.isinf(getattr(self, name)):
                raise ValueError(name)
        if not (self.servo_step_seconds > 0.0) or math.isinf(self.servo_step_seconds):
            raise ValueError("servo_step_seconds")
        # Every float field is a C# float: store it rounded, so arithmetic matches.
        object.__setattr__(self, "max_joint_speed_rad_per_sec", f32(self.max_joint_speed_rad_per_sec))
        object.__setattr__(self, "speed_scales", tuple(f32(s) for s in self.speed_scales))
        object.__setattr__(self, "relay_min_move_seconds", f32(self.relay_min_move_seconds))
        object.__setattr__(self, "relay_deadband_rad", f32(self.relay_deadband_rad))
        object.__setattr__(self, "relay_move_speed_rad_per_sec", f32(self.relay_move_speed_rad_per_sec))
        object.__setattr__(self, "sensed_age_seconds", tuple(f32(a) for a in self.sensed_age_seconds))
        object.__setattr__(self, "servo_max_speed_rad_per_sec", f32(self.servo_max_speed_rad_per_sec))
        object.__setattr__(self, "servo_lag_seconds", f32(self.servo_lag_seconds))
        object.__setattr__(self, "servo_pipeline_delay_seconds", f32(self.servo_pipeline_delay_seconds))
        object.__setattr__(self, "servo_step_seconds", f32(self.servo_step_seconds))


def headset_joint_config(ticks_per_second: int, max_horizon_ms: float = 500.0,
                         sensed_ages_ms: Sequence[float] = (45.0, 35.0, 25.0, 15.0, 5.0)) -> JointPredictorConfig:
    """The predictor config the headset builds (Bridge JetRoverJointStateLink + JetRoverArmConfig):
    13 joints, a 64-command ring, the gripper (motor 10) scaled at 0.358 per radian, and the
    per-motor reading ages for motors 1-5."""
    return JointPredictorConfig(
        max_joints=13,
        command_history_capacity=64,
        max_joint_speed_rad_per_sec=6.0,
        instant_motor_ids=(),
        max_horizon_ticks=int(max_horizon_ms / 1000.0 * ticks_per_second),
        ticks_per_second=ticks_per_second,
        scaled_motor_ids=(10,),
        speed_scales=(0.358,),
        sensed_age_motor_ids=(1, 2, 3, 4, 5),
        sensed_age_seconds=tuple(f32(ms / 1000.0) for ms in sensed_ages_ms),
    )


@dataclass
class PredictorDiagnostics:
    horizon_ticks: int = 0
    last_observation_ticks: int = 0
    accepted: int = 0
    rejected: int = 0
    extra: dict = field(default_factory=dict)
