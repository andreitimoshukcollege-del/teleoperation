"""The robot profile and pulse space a session was recorded with.

The profile JSON is a copy of the repo's ``core/RobotProfiles/jetrover.json``, carried inside every
data bundle (``just pack-dataset``), so this package never reads the repo. The pulse space is
RobotHost's ``GenericArmPlantConfig.Default``: those numbers are not in the profile JSON, so they
are restated here, once, with where they come from, and a bundle's ``index.json`` may override them.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class Joint:
    motor_id: int
    role: str
    zero_offset_radians: float = 0.0
    min_angle_radians: Optional[float] = None
    max_angle_radians: Optional[float] = None


@dataclass(frozen=True)
class PulseSpace:
    """GenericArmPlantConfig.Default (core/Teleop.RobotHost/Plant/GenericArmPlantConfig.cs)."""

    zero_pulse: int = 500
    # C#: 1000f / (240f * MathF.PI / 180f), single precision.
    pulses_per_radian: float = float(np.float32(1000.0) / (np.float32(240.0) * np.float32(math.pi) / np.float32(180.0)))
    min_pulse: int = 0
    max_pulse: int = 1000
    default_pulses_per_second: float = 2200.0
    # Measured on the robot on 2026-10-25 (stops 106 and 655, 10 inside each; docs/adr/0018 §8).
    gripper_open_pulse: float = 116.0
    gripper_closed_pulse: float = 645.0


@dataclass(frozen=True)
class RobotProfile:
    name: str
    base_height: float
    proximal_link_length: float
    distal_link_length: float
    joints: List[Joint]
    pulse: PulseSpace = field(default_factory=PulseSpace)

    def joint(self, motor_id: int) -> Joint:
        for j in self.joints:
            if j.motor_id == motor_id:
                return j
        raise KeyError(f"profile {self.name!r} has no motor {motor_id}")

    def motor_for_role(self, role: str) -> int:
        for j in self.joints:
            if j.role == role:
                return j.motor_id
        raise KeyError(f"profile {self.name!r} has no {role} joint")

    @property
    def arm_motor_ids(self) -> List[int]:
        """The motors whose value is an angle (everything but the gripper's open/close)."""
        return [j.motor_id for j in self.joints if j.role != "GripperMain"]

    @property
    def gripper_motor_id(self) -> Optional[int]:
        for j in self.joints:
            if j.role == "GripperMain":
                return j.motor_id
        return None

    def pulse_to_angle(self, motor_id: int, pulse: np.ndarray) -> np.ndarray:
        """The truth conversion (contract/SESSION_FORMAT.md, "The truth"), in float64."""
        j = self.joint(motor_id)
        return (np.asarray(pulse, dtype=np.float64) - self.pulse.zero_pulse) / self.pulse.pulses_per_radian - j.zero_offset_radians

    def pulse_to_gripper_fraction(self, pulse: np.ndarray) -> np.ndarray:
        """A gripper pulse as RobotHost reports it (GenericArmPlant.ReadJointState): open..closed as 0..1, clamped."""
        span = self.pulse.gripper_closed_pulse - self.pulse.gripper_open_pulse
        fraction = (np.asarray(pulse, dtype=np.float64) - self.pulse.gripper_open_pulse) / span
        return np.clip(fraction, 0.0, 1.0)

    def pulse_to_value(self, motor_id: int, pulse: np.ndarray) -> np.ndarray:
        """The truth conversion for any motor: an arm joint's angle (rad), or the gripper's 0..1 fraction
        (contract/SESSION_FORMAT.md, "The truth")."""
        if motor_id == self.gripper_motor_id:
            return self.pulse_to_gripper_fraction(pulse)
        return self.pulse_to_angle(motor_id, pulse)

    def angle_to_pulse(self, motor_id: int, angle: np.ndarray) -> np.ndarray:
        """RobotHost's command conversion: zeroPulse + (angle + zeroOffset) * pulsesPerRadian, before its clamp."""
        j = self.joint(motor_id)
        return self.pulse.zero_pulse + (np.asarray(angle, dtype=np.float64) + j.zero_offset_radians) * self.pulse.pulses_per_radian


def load_profile(path: str, pulse_overrides: Optional[Dict[str, float]] = None) -> RobotProfile:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return profile_from_dict(raw, pulse_overrides)


def profile_from_dict(raw: dict, pulse_overrides: Optional[Dict[str, float]] = None) -> RobotProfile:
    joints = [
        Joint(
            motor_id=int(j["motorId"]),
            role=str(j["role"]),
            zero_offset_radians=float(j.get("zeroOffsetRadians", 0.0)),
            min_angle_radians=float(j["minAngleRadians"]) if "minAngleRadians" in j else None,
            max_angle_radians=float(j["maxAngleRadians"]) if "maxAngleRadians" in j else None,
        )
        for j in raw["joints"]
    ]
    pulse = PulseSpace()
    if pulse_overrides:
        pulse = PulseSpace(**{**pulse.__dict__, **pulse_overrides})
    return RobotProfile(
        name=str(raw.get("name", "")),
        base_height=float(raw.get("baseHeight", 0.0)),
        proximal_link_length=float(raw["proximalLinkLength"]),
        distal_link_length=float(raw["distalLinkLength"]),
        joints=joints,
        pulse=pulse,
    )


# The JetRover as core/RobotProfiles/jetrover.json describes it, for the simulator when no bundle
# supplies a profile. A bundle's own copy always wins.
JETROVER = profile_from_dict(
    {
        "name": "jetrover",
        "baseHeight": 0.035,
        "proximalLinkLength": 0.13,
        "distalLinkLength": 0.13,
        "joints": [
            {"motorId": 1, "role": "BaseYaw"},
            {"motorId": 2, "role": "Proximal", "zeroOffsetRadians": 0.12217305},
            {"motorId": 3, "role": "Distal"},
            {"motorId": 4, "role": "Wrist", "wristIndex": 0},
            {"motorId": 10, "role": "GripperMain"},
            {"motorId": 5, "role": "GripperRotate"},
        ],
    }
)
