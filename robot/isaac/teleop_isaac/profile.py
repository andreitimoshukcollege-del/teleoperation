"""A serial-chain robot profile (docs/adr/0026), as the bridge needs it: ``core/RobotProfiles/fr3-duo.json``'s arms,
joints (motor ids, names, limits) and grippers. Only what the shaper and the Isaac joint mapping use; the kinematics
stay Core's.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class Joint:
    motor_id: int
    name: str
    lower: float
    upper: float
    soft_lower: float
    soft_upper: float
    velocity: float
    acceleration: float
    jerk: float
    pbv_deceleration: float
    pbv_offset: float


@dataclass(frozen=True)
class Gripper:
    motor_id: int
    name: str  # the first finger's joint; with second_finger_mimics, the other follows it
    finger_travel: float  # metres from closed to open, per finger
    speed: float  # metres per second, per finger
    second_finger_mimics: bool


@dataclass(frozen=True)
class Arm:
    name: str
    joints: Tuple[Joint, ...]
    gripper: Optional[Gripper]


@dataclass(frozen=True)
class Profile:
    name: str
    arms: Tuple[Arm, ...]

    def motor_ids(self) -> List[int]:
        """Every motor in profile order (each arm's joints, then its hand): the shaper's order, and RobotHost's."""
        ids: List[int] = []
        for arm in self.arms:
            ids.extend(j.motor_id for j in arm.joints)
            if arm.gripper is not None:
                ids.append(arm.gripper.motor_id)
        return ids


def load(path: str) -> Profile:
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    if d.get("kind") != "serial-chain":
        raise ValueError(f"{path} is not a serial-chain profile")
    arms = []
    for a in d["arms"]:
        joints = tuple(Joint(j["motorId"], j["name"], j["lower"], j["upper"], j["softLower"], j["softUpper"], j["velocity"],
                             j["acceleration"], j["jerk"], j.get("positionBasedDeceleration", 0.0),
                             j.get("positionBasedVelocityOffset", 0.0)) for j in a["joints"])
        g = a.get("gripper") if a.get("hasGripper") else None
        gripper = Gripper(g["motorId"], g["name"], g["fingerTravel"], g["speed"], bool(g.get("secondFingerMimics", False))) if g else None
        arms.append(Arm(a["name"], joints, gripper))
    return Profile(d["name"], tuple(arms))
