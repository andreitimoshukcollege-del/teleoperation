"""A serial-chain robot profile (docs/adr/0026): the FR3 Duo's ``core/RobotProfiles/fr3-duo.json``, carried in
every v2 session as ``robot-profile.json``, so this package never reads the repo.

It is a copy of ``Teleop.RobotArm``'s ``SerialChainProfileDocument.ToProfile`` (the frames composed from the
URDF's own ``xyz``/``rpy``), with forward kinematics in numpy for the simulator's payload model and the model's
gravity features. Nothing here is pinned bit for bit: it decides no prediction, only what the simulator does and
which direction gravity points in each arm's base.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

KIND = "serial-chain"


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _transform(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = _rpy(*rpy)
    t[:3, 3] = xyz
    return t


def _compose(origins) -> np.ndarray:
    t = np.eye(4)
    for o in origins:
        t = t @ _transform(o["xyz"], o["rpy"])
    return t


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    r = np.eye(3) + s * k + (1 - c) * (k @ k)
    t = np.eye(4)
    t[:3, :3] = r
    return t


@dataclass(frozen=True)
class ChainJoint:
    motor_id: int
    name: str
    type: str
    origin: np.ndarray  # 4x4, parent link -> joint frame at zero
    axis: np.ndarray  # unit, in the joint frame
    lower: float
    upper: float
    soft_lower: float
    soft_upper: float
    velocity: float
    acceleration: float
    jerk: float
    effort: float
    pbv_deceleration: float
    pbv_offset: float


@dataclass(frozen=True)
class ChainGripper:
    motor_id: int
    name: str
    finger_travel: float
    speed: float
    second_finger_mimics: bool


@dataclass(frozen=True)
class ChainArm:
    name: str
    mount: np.ndarray  # 4x4, robot root -> arm base
    joints: Tuple[ChainJoint, ...]
    tool: np.ndarray  # 4x4, last joint -> TCP
    gripper: Optional[ChainGripper]

    def frames(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Each joint's origin and axis in the root frame (after the joints before it), and the TCP position."""
        t = self.mount.copy()
        origins, axes = np.zeros((len(self.joints), 3)), np.zeros((len(self.joints), 3))
        for i, j in enumerate(self.joints):
            t = t @ j.origin
            origins[i] = t[:3, 3]
            axes[i] = t[:3, :3] @ j.axis
            if j.type == "revolute":
                t = t @ _axis_angle(j.axis, float(q[i]))
            else:
                move = np.eye(4)
                move[:3, 3] = j.axis * float(q[i])
                t = t @ move
        tcp = (t @ self.tool)[:3, 3]
        return origins, axes, tcp

    def gravity_in_base(self) -> np.ndarray:
        """The unit gravity direction (root -z) in this arm's base frame: fixed by the mount, so the two arms differ."""
        return self.mount[:3, :3].T @ np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class SerialChainProfile:
    name: str
    arms: Tuple[ChainArm, ...]
    source_sha256: str = ""

    @property
    def motors(self) -> List[int]:
        """Every motor id, in profile order: each arm's joints, then its gripper."""
        out: List[int] = []
        for arm in self.arms:
            out += [j.motor_id for j in arm.joints]
            if arm.gripper is not None:
                out.append(arm.gripper.motor_id)
        return out

    @property
    def hand_motors(self) -> List[int]:
        return [a.gripper.motor_id for a in self.arms if a.gripper is not None]

    def arm_of(self, motor: int) -> int:
        for k, arm in enumerate(self.arms):
            if motor in [j.motor_id for j in arm.joints] or (arm.gripper is not None and arm.gripper.motor_id == motor):
                return k
        return -1


def profile_from_dict(raw: dict) -> SerialChainProfile:
    if raw.get("kind") != KIND:
        raise ValueError(f"profile {raw.get('name')!r} has kind {raw.get('kind')!r}, not {KIND!r}")
    arms = []
    for a in raw["arms"]:
        joints = tuple(ChainJoint(
            motor_id=int(j["motorId"]), name=j["name"], type=j["type"], origin=_compose(j["origin"]),
            axis=np.asarray(j["axis"], dtype=np.float64), lower=float(j["lower"]), upper=float(j["upper"]),
            soft_lower=float(j["softLower"]), soft_upper=float(j["softUpper"]), velocity=float(j["velocity"]),
            acceleration=float(j["acceleration"]), jerk=float(j["jerk"]), effort=float(j["effort"]),
            pbv_deceleration=float(j["positionBasedDeceleration"]), pbv_offset=float(j["positionBasedVelocityOffset"]),
        ) for j in a["joints"])
        g = a.get("gripper") or {}
        gripper = ChainGripper(int(g["motorId"]), g["name"], float(g["fingerTravel"]), float(g["speed"]),
                               bool(g["secondFingerMimics"])) if a.get("hasGripper") else None
        arms.append(ChainArm(a["name"], _compose(a["mount"]), joints, _compose(a["tool"]), gripper))
    return SerialChainProfile(str(raw["name"]), tuple(arms), str(raw.get("source", {}).get("sha256", "")))


def load_chain_profile(path: str) -> SerialChainProfile:
    with open(path, "r", encoding="utf-8") as f:
        return profile_from_dict(json.load(f))


def is_chain_profile(path: str) -> bool:
    if not os.path.exists(path):
        return False
    with open(path, "r", encoding="utf-8") as f:
        try:
            return json.load(f).get("kind") == KIND
        except (ValueError, AttributeError):
            return False


# Franka's FR3 "ready" pose for the left arm; the right arm mirrors it (joints 1, 3, 5, 7 negated), as
# RobotHost's SerialChainHost.StartPose has it.
READY_POSE = (0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4)


def start_pose(profile: SerialChainProfile) -> List[float]:
    """Every motor's start, in profile order: the ready pose per arm (mirrored for every second arm), hands open."""
    out: List[float] = []
    for a, arm in enumerate(profile.arms):
        for i, j in enumerate(arm.joints):
            q = READY_POSE[i] if i < len(READY_POSE) else 0.0
            if a % 2 == 1 and i % 2 == 0:
                q = -q
            out.append(float(min(max(q, j.soft_lower), j.soft_upper)) + 0.0)
        if arm.gripper is not None:
            out.append(1.0)
    return out
