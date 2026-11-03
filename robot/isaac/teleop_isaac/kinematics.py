"""Forward kinematics from the URDF (docs/adr/0032 decision 2): what the renderer poses the visual robot with.

Plain Python and the standard library. Every link's pose is its parent's, times the joint's origin (URDF xyz, then fixed-axis
roll/pitch/yaw), times the joint's motion about or along its axis. The tool poses it computes are held to Core's
(``golden/fr3-duo-fk-v1.json``, written by ``Fr3FkGoldenTests``), so the rendered robot is the one the IK and the predictors
reason about. Quaternions are w, x, y, z.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence, Tuple

Vec = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def q_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def q_axis(axis: Sequence[float], angle: float) -> Quat:
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    s = math.sin(angle / 2) / n
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def q_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """URDF's fixed-axis roll, pitch, yaw: yaw about z, after pitch about y, after roll about x."""
    return q_mul(q_axis((0, 0, 1), yaw), q_mul(q_axis((0, 1, 0), pitch), q_axis((1, 0, 0), roll)))


def q_rotate(q: Quat, v: Sequence[float]) -> Vec:
    w, x, y, z = q
    vx, vy, vz = v
    # v + 2w(u x v) + 2u x (u x v), with u = (x, y, z)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    return (vx + 2 * (w * cx + y * cz - z * cy), vy + 2 * (w * cy + z * cx - x * cz), vz + 2 * (w * cz + x * cy - y * cx))


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    xyz: Vec
    rpy: Vec
    axis: Vec


def read_urdf(path: str) -> Dict[str, Joint]:
    """Every joint of the URDF, by name."""
    joints: Dict[str, Joint] = {}
    for j in ET.parse(path).getroot().findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        xyz = tuple(float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split())
        rpy = tuple(float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split())
        axis = tuple(float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split())
        joints[j.get("name")] = Joint(j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"),
                                      xyz, rpy, axis)
    return joints


def chain(joints: Mapping[str, Joint], link: str) -> List[Joint]:
    """The joints from the root down to ``link``."""
    by_child = {j.child: j for j in joints.values()}
    out: List[Joint] = []
    while link in by_child:
        j = by_child[link]
        out.append(j)
        link = j.parent
    return list(reversed(out))


def link_pose(joints: Mapping[str, Joint], values: Mapping[str, float], link: str) -> Tuple[Vec, Quat]:
    """``link``'s pose in the root frame for joint ``values`` (name -> rad, or m for a prismatic joint; absent ones are 0)."""
    p: Vec = (0.0, 0.0, 0.0)
    q: Quat = (1.0, 0.0, 0.0, 0.0)
    for j in chain(joints, link):
        d = q_rotate(q, j.xyz)
        p = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
        q = q_mul(q, q_rpy(*j.rpy))
        v = values.get(j.name, 0.0)
        if j.type in ("revolute", "continuous"):
            q = q_mul(q, q_axis(j.axis, v))
        elif j.type == "prismatic":
            d = q_rotate(q, (j.axis[0] * v, j.axis[1] * v, j.axis[2] * v))
            p = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
    return p, q


def local_motion(joint: Joint, rest_translation: Vec, rest_orientation: Quat, value: float) -> Tuple[Vec, Quat]:
    """A moving link's pose relative to its parent at ``value``, from its authored rest pose (the joint's origin)."""
    if joint.type in ("revolute", "continuous"):
        return rest_translation, q_mul(rest_orientation, q_axis(joint.axis, value))
    if joint.type == "prismatic":
        d = q_rotate(rest_orientation, (joint.axis[0] * value, joint.axis[1] * value, joint.axis[2] * value))
        return (rest_translation[0] + d[0], rest_translation[1] + d[1], rest_translation[2] + d[2]), rest_orientation
    return rest_translation, rest_orientation
