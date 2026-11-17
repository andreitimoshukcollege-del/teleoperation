"""The Isaac renderer (docs/adr/0032 decisions 2 and 3): the scene's cameras, rendered beside the physics bridge, as live
feeds on ADR 0014's camera wire.

Run on the GPU machine with Isaac's Python, after the bridge (``just isaac-host`` does both):

    ~/isaacsim/venv/bin/python -m teleop_isaac.renderer --scene ../../core/Scenes/fr3-duo-bench.json --tap /tmp/teleop-isaac-tap.sock

A second, headless Kit process. It opens the bridge's converted robot USD with the importer's ``Physics`` variant set to
``none``, so it is visual only and nothing steps, and builds the scene file's surfaces and objects as visuals. The bridge
sends it a copy of every state and world message (``--tap``); each frame it poses the robot from the newest state (each
moving link's rest pose times its joint's motion, ``kinematics.local_motion``) and the objects from the newest world
message, renders, and hands each camera's colour image to an encoder thread, which sends it as JPEG to whoever
subscribed to that camera's port.

**Frame association.** An image read after posing frame k shows frame k - lag (the spike measured lag 2, stable in every
sample). At start-up the renderer measures the lag with a test cube moved to a new place every frame, and stamps each
frame's ``captureTicks`` with the sensed time of the state it shows: CLOCK_MONOTONIC nanoseconds, RobotHost's clock.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import os
import signal
import socket
import sys
import threading
import time
import traceback
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from . import backdrops as backdrops_module
from . import clutter as clutter_module
from . import environments as environments_module
from . import materials as materials_module
from . import camfeed, camwire, kinematics, wire
from . import profile as profile_module
from . import scene as scene_module

APERTURE_MM = 20.955  # horizontal aperture; the focal length follows from the field of view
# A mount link's frame looks along +x with z up; a USD camera looks along its -z with +y up.
MOUNT_TO_CAMERA = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))  # columns: the camera's x, y, z in the mount frame
LAG_CUBE = 0.06


# --- what needs no Isaac (tests/test_renderer.py) ---------------------------------------------------------------------

class MotorMap:
    """From a state's per-motor values to the URDF's joint values: an arm joint's radians as they are, a hand's 0..1
    opening as each finger's travel (both fingers, the second mimicking the first)."""

    def __init__(self, p: profile_module.Profile):
        self.joints: Dict[int, Tuple[str, ...]] = {}
        self.scale: Dict[int, float] = {}
        for arm in p.arms:
            for j in arm.joints:
                self.joints[j.motor_id] = (j.name,)
                self.scale[j.motor_id] = 1.0
            if arm.gripper is not None:
                g = arm.gripper
                second = g.name[:-1] + "2" if g.name.endswith("1") else None
                self.joints[g.motor_id] = tuple(n for n in (g.name, second) if n)
                self.scale[g.motor_id] = g.finger_travel

    def values(self, motors: Sequence[Tuple[int, float]]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for m, v in motors:
            for name in self.joints.get(m, ()):
                out[name] = v * self.scale[m]
        return out


class SimMotorMap:
    """:class:`MotorMap` for a robot described by a sim sidecar (the JetRover's, ``servo.SimModel``): a joint's radians as
    they are; the gripper's 0..1 closing as each finger's travel (``SimModel.finger_value``: open past the CAD's pose at 0)."""

    def __init__(self, model):
        self.model = model
        self.joints: Dict[int, Tuple[str, ...]] = {}
        for mid, m in model.motors.items():
            self.joints[mid] = m.fingers if m.is_gripper else (m.joint,)

    def values(self, motors: Sequence[Tuple[int, float]]) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for m, v in motors:
            gripper = m in self.model.motors and self.model.motors[m].is_gripper
            for name in self.joints.get(m, ()):
                out[name] = self.model.finger_value(m, v) if gripper else v
        return out


def estimate_lag(samples: Sequence[int]) -> Tuple[Optional[int], float]:
    """(the most common lag, the share of samples that agree), or (None, 0) without samples."""
    if not samples:
        return None, 0.0
    mode = max(set(samples), key=list(samples).count)
    return mode, list(samples).count(mode) / len(samples)


def intrinsics(width: int, height: int, hfov_deg: float) -> Dict[str, float]:
    """A pinhole with square pixels, its principal point at the centre and pixel centres at +0.5."""
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return {"fx": fx, "fy": fx, "cx": width / 2.0, "cy": height / 2.0}


def look_at(position: Sequence[float], target: Sequence[float], up=(0.0, 0.0, 1.0)) -> Tuple[float, float, float, float]:
    """The orientation (w, x, y, z) of a USD camera at ``position`` looking at ``target``."""
    f = [t - p for t, p in zip(target, position)]
    n = math.sqrt(sum(c * c for c in f))
    z = [-c / n for c in f]
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(c * c for c in x))
    x = [c / n for c in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return matrix_to_quat(((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2])))


def matrix_to_quat(m) -> Tuple[float, float, float, float]:
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def mount_camera_orientation() -> Tuple[float, float, float, float]:
    return matrix_to_quat(MOUNT_TO_CAMERA)


def mounted_camera_local(c: Mapping) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """A camera on a link (``mount``): its position and USD orientation (w, x, y, z) in that link's frame. With
    ``mountLookAt`` (and ``mountPosition``, ``mountUp``, all in the link's frame) it looks from its position at that point,
    as a wrist camera does (docs/adr/0032 amendment 3); without, it sits at the link's origin looking along the link's +x
    with +z up, as the URDF's ``head_camera_mounting_point`` means."""
    if "mountLookAt" in c:
        position = tuple(float(v) for v in c.get("mountPosition", (0.0, 0.0, 0.0)))
        return position, look_at(position, c["mountLookAt"], tuple(c.get("mountUp", (0.0, 0.0, 1.0))))
    return (0.0, 0.0, 0.0), mount_camera_orientation()


def q_conj(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def compose(parent: Tuple[Sequence[float], Sequence[float]], local: Tuple[Sequence[float], Sequence[float]]):
    """``local`` (a pose in ``parent``'s frame) in the frame ``parent`` is given in."""
    d = kinematics.q_rotate(parent[1], local[0])
    return tuple(a + b for a, b in zip(parent[0], d)), kinematics.q_mul(parent[1], local[1])


def relative_pose(parent: Tuple[Sequence[float], Sequence[float]], child: Tuple[Sequence[float], Sequence[float]]):
    """``child``'s pose (position, w-x-y-z quaternion) in ``parent``'s frame, both given in the same frame."""
    pi = q_conj(parent[1])
    d = [c - p for c, p in zip(child[0], parent[0])]
    return kinematics.q_rotate(pi, d), kinematics.q_mul(pi, child[1])


def screen_corners(center: Sequence[float], size: Sequence[float], normal: Sequence[float]):
    """A vertical backdrop screen's four corners: centred at ``center``, ``size`` = (width, height), facing the horizontal
    ``normal`` (x, y). Corner order goes with texture coordinates (0,0), (1,0), (1,1), (0,1), so the image reads left to
    right, unmirrored, from the side the normal faces."""
    (cx, cy, cz), (w, h) = center, size
    n = math.hypot(normal[0], normal[1])
    ax, ay = -normal[1] / n, normal[0] / n  # left to right, seen from the side the normal faces
    b, t = cz - h / 2, cz + h / 2
    return [(cx - ax * w / 2, cy - ay * w / 2, b), (cx + ax * w / 2, cy + ay * w / 2, b),
            (cx + ax * w / 2, cy + ay * w / 2, t), (cx - ax * w / 2, cy - ay * w / 2, t)]


def backdrop_quads(scene) -> List[Tuple[str, str, List[Tuple[float, float, float]]]]:
    """The scene file's backdrop surfaces (docs/adr/0032 decision 4, amendment 3): (id, mask class, corners) for every mat
    lying on a surface (``mats``: on a surface's top, raised by ``lift``, within ``region`` [x0, x1, y0, y1] when given) and
    every screen (``screens``). Each gets its own random image at every backdrop change. A mat on the work table is the
    ``mat`` mask class, one on another surface that surface's class; a screen is ``screen``."""
    spec = scene.extra.get("backdrops") or {}
    out, ids = [], set()
    surfaces = {x.id: x for x in scene.surfaces}
    for m in spec.get("mats") or []:
        s = surfaces.get(m.get("on", ""))
        if s is None:
            raise ValueError(f"backdrop mat {m.get('id')!r} lies on {m.get('on')!r}, which is not a surface of the scene")
        x0, x1 = s.center[0] - s.size[0] / 2, s.center[0] + s.size[0] / 2
        y0, y1 = s.center[1] - s.size[1] / 2, s.center[1] + s.size[1] / 2
        if "region" in m:
            r = m["region"]
            x0, x1, y0, y1 = max(x0, r[0]), min(x1, r[1]), max(y0, r[2]), min(y1, r[3])
        top = s.top + float(m.get("lift", 0.0005))
        out.append((m["id"], "mat" if s.id == "table" else s.id, [(x0, y0, top), (x1, y0, top), (x1, y1, top), (x0, y1, top)]))
    for sc in spec.get("screens") or []:
        out.append((sc["id"], "screen", screen_corners(sc["center"], sc["size"], sc.get("normal", (-1.0, 0.0)))))
    for qid, _, _ in out:
        if qid in ids:
            raise ValueError(f"two backdrop surfaces are called {qid!r}")
        ids.add(qid)
    return out


def environment_spec(scene) -> Optional[Dict]:
    """The scene file's room around the table (docs/adr/0037 decision 2), checked, or None when it has none: an HDRI on the
    dome light ``dome``, its brightness set so the room's mean radiance is ``meanRadiance`` whatever the photo's exposure,
    turned about the vertical by a seeded draw from ``yawDeg``, after ``rotateXYZDeg``. ``hideSurfaces`` are surfaces the
    cameras do not see (the room's own floor shows instead), though they still collide in the bridge."""
    spec = scene.extra.get("environment")
    if spec is None:
        return None
    if spec.get("kind") != "hdri":
        raise ValueError(f"environment kind {spec.get('kind')!r}: only 'hdri'")
    domes = [light["id"] for light in scene.extra.get("lights") or [] if light.get("kind") == "dome"]
    dome = spec.get("dome", "dome")
    if dome not in domes:
        raise ValueError(f"environment dome {dome!r} is not a dome light of the scene ({domes})")
    radiance = float(spec.get("meanRadiance", 0))
    if not radiance > 0:
        raise ValueError("environment meanRadiance must be positive")
    yaw = [float(v) for v in spec.get("yawDeg", [0.0, 360.0])]
    if len(yaw) != 2 or yaw[0] > yaw[1]:
        raise ValueError("environment yawDeg is [low, high]")
    rot = [float(v) for v in spec.get("rotateXYZDeg", [0.0, 0.0, 0.0])]
    if len(rot) != 3:
        raise ValueError("environment rotateXYZDeg has three angles")
    surfaces = {x.id for x in scene.surfaces}
    hide = list(spec.get("hideSurfaces") or [])
    for sid in hide:
        if sid not in surfaces:
            raise ValueError(f"environment hides {sid!r}, which is not a surface of the scene")
    if spec.get("change", "per-session") != "per-session":
        raise ValueError("environment change: only 'per-session'")
    target = spec.get("targetBrightness")
    if target is not None and not 0 < float(target) < 255:
        raise ValueError("environment targetBrightness is between 0 and 255")
    return {"dome": dome, "manifest": str(spec.get("manifest", "")), "meanRadiance": radiance, "yawDeg": yaw,
            "rotateXYZDeg": rot, "hideSurfaces": hide, "targetBrightness": None if target is None else float(target)}


METER_STEPS = 3          # at most this many exposure corrections after a room change
METER_TOLERANCE = 6.0    # close enough to the target brightness (0-255)
METER_RANGE = (0.25, 4.0)  # the metered intensity stays within this factor of the luminance-normalised one
METER_GAMMA = 2.2        # an image's brightness goes roughly with radiance to the 1/2.2: the correction undoes that


CAMERA_GAIN_RANGE = (0.25, 4.0)  # a camera's own exposure gain, in linear light


def camera_gain(brightness: float, target: float) -> float:
    """A camera's own exposure gain (linear light) that brings its mean brightness from ``brightness`` toward ``target``,
    within :data:`CAMERA_GAIN_RANGE`: its auto-exposure, locked for the session."""
    g = (target / max(brightness, 1.0)) ** METER_GAMMA
    return min(CAMERA_GAIN_RANGE[1], max(CAMERA_GAIN_RANGE[0], g))


def exposure_lut(gain: float):
    """The 256-entry table that applies ``gain`` to an 8-bit sRGB image in linear light (clipping at white), or None for a
    gain of 1."""
    import numpy as np

    if abs(gain - 1.0) < 1e-6:
        return None
    x = np.arange(256, dtype=np.float64) / 255.0
    lin = np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4) * gain
    lin = np.clip(lin, 0.0, 1.0)
    out = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * lin ** (1 / 2.4) - 0.055)
    return np.clip(np.round(out * 255.0), 0, 255).astype(np.uint8)


def metered_intensity(intensity: float, base: float, brightness: float, target: float) -> float:
    """The dome intensity that should bring the metering camera's mean brightness from ``brightness`` to ``target``, as a
    camera's auto-exposure would, kept within :data:`METER_RANGE` of ``base`` (the luminance-normalised intensity)."""
    factor = (target / max(brightness, 1.0)) ** METER_GAMMA
    lo, hi = base * METER_RANGE[0], base * METER_RANGE[1]
    return min(hi, max(lo, intensity * factor))


LOOK_ID = re.compile(r"^[a-z0-9][a-z0-9-]{0,39}$")  # it goes into LK records: no ':', ';' or '|'
LOOK_FIELDS = ("texture", "desaturation", "tint", "roughness", "roughnessTexture", "weight")


def _unit(v, what: str) -> float:
    if not (isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and 0 <= v <= 1):
        raise ValueError(f"{what} is a number from 0 to 1")
    return float(v)


def _look(lid, d) -> Dict:
    if not isinstance(lid, str) or not LOOK_ID.match(lid):
        raise ValueError(f"materials looks: {lid!r}: lowercase letters, digits and '-'")
    if lid in materials_module.TEXTURES:
        raise ValueError(f"materials looks: {lid!r} is a texture's id; a look needs a name of its own")
    if not isinstance(d, dict):
        raise ValueError(f"materials looks: {lid!r} is not an object")
    unknown = sorted(set(d) - set(LOOK_FIELDS))
    if unknown:
        raise ValueError(f"materials looks: {lid!r} has no field {', '.join(unknown)}")
    tid = d.get("texture")
    if tid not in materials_module.TEXTURES:
        raise ValueError(f"materials looks: {lid!r} wears {tid!r}, which is not one of the textures")
    weight = d.get("weight", 1.0)
    if not (isinstance(weight, (int, float)) and not isinstance(weight, bool) and math.isfinite(weight) and weight > 0):
        raise ValueError(f"materials looks: {lid!r} weight must be a positive number")
    tint = d.get("tint", [1.0, 1.0, 1.0])
    if not (isinstance(tint, list) and len(tint) == 3):
        raise ValueError(f"materials looks: {lid!r} tint is [r, g, b], each 0 to 1")
    rough = d.get("roughness")
    rough = None if rough is None else _unit(rough, f"materials looks: {lid!r} roughness")
    return {"texture": tid, "desaturation": _unit(d.get("desaturation", 0.0), f"materials looks: {lid!r} desaturation"),
            "tint": [_unit(v, f"materials looks: {lid!r} tint") for v in tint], "roughness": rough,
            # How much of the texture's own roughness shows: all of it unless the look sets a roughness, then none unless
            # it says how much (OmniPBR's reflectionroughness_texture_influence).
            "roughnessTexture": _unit(d.get("roughnessTexture", 1.0 if rough is None else 0.0),
                                      f"materials looks: {lid!r} roughnessTexture"),
            "weight": float(weight)}


def materials_spec(scene) -> Optional[Dict]:
    """The scene file's materials (docs/adr/0037 amendment 5 and "Surfaces and flat clutter"), checked, or None:
    - which texture classes each simple shape may wear (``box``, ``cylinder``);
    - ``looks``: each a ``texture`` with its ``desaturation``, ``tint``, an optional ``roughness`` (OmniPBR's constant, with
      ``roughnessTexture`` the share of the texture's own) and a ``weight`` in the draw;
    - what each surface wears (``surfaces``): a list of looks, drawn per change by their weights, or one texture id (the
      first form, still read: it wears the section's ``surfaceDesaturation`` and ``surfaceTint``, as a look named after it);
    - ``objects``: looks for simple shapes by name, as for a surface, instead of their class's textures;
    - ``tileScale``, a texture's repeats per its real size (the renderer's projection, measured on the twin).
    ``surfaces`` and ``objects`` come back as lists of look ids, and ``looks`` holds every look they name."""
    spec = scene.extra.get("materials")
    if spec is None:
        return None
    classes = {"box": list(spec.get("box") or []), "cylinder": list(spec.get("cylinder") or [])}
    for shape, cs in classes.items():
        if not all(c in ("wood", "metal") for c in cs):
            raise ValueError(f"materials {shape}: classes are wood and metal, not {cs}")
    tile = float(spec.get("tileScale", 1.0))
    if not tile > 0:
        raise ValueError("materials tileScale must be positive")
    desat = float(spec.get("surfaceDesaturation", 0.0))
    if not 0 <= desat <= 1:
        raise ValueError("materials surfaceDesaturation is 0 to 1")
    tint = [float(v) for v in spec.get("surfaceTint", [1.0, 1.0, 1.0])]
    if len(tint) != 3 or not all(0 <= v <= 1 for v in tint):
        raise ValueError("materials surfaceTint is [r, g, b], each 0 to 1")
    declared = spec.get("looks") or {}
    if not isinstance(declared, dict):
        raise ValueError("materials looks: an object of looks by name")
    looks = {lid: _look(lid, d) for lid, d in declared.items()}

    def choice(where: str, value) -> List[str]:
        if isinstance(value, str):
            if value not in materials_module.TEXTURES:
                raise ValueError(f"materials {where}: {value!r} is not one of the textures")
            looks.setdefault(value, {"texture": value, "desaturation": desat, "tint": tint, "roughness": None,
                                     "roughnessTexture": 1.0, "weight": 1.0})
            return [value]
        if not isinstance(value, list) or not value:
            raise ValueError(f"materials {where}: an empty list; it names a texture or at least one look")
        for lid in value:
            if not isinstance(lid, str) or lid not in declared:
                raise ValueError(f"materials {where}: {lid!r} is not one of the looks")
        if len(set(value)) != len(value):
            raise ValueError(f"materials {where}: a look named twice")
        return list(value)

    known = {x.id for x in scene.surfaces}
    surfaces = {}
    for sid, value in dict(spec.get("surfaces") or {}).items():
        if sid not in known:
            raise ValueError(f"materials surfaces: {sid!r} is not a surface of the scene")
        surfaces[sid] = choice(f"surfaces {sid}", value)
    shapes = {o.name: o.shape for o in scene.objects}
    objects = {}
    for name, value in dict(spec.get("objects") or {}).items():
        if name not in shapes:
            raise ValueError(f"materials objects: {name!r} is not an object of the scene")
        if shapes[name] not in ("box", "cylinder", "sphere"):
            raise ValueError(f"materials objects: {name!r} is a {shapes[name]}, which keeps its own look")
        if name in known:
            raise ValueError(f"materials objects: {name!r} is also a surface's id")
        objects[name] = choice(f"objects {name}", value)
    return {"manifest": str(spec.get("manifest", "")), "classes": classes, "surfaces": surfaces, "objects": objects,
            "looks": looks, "tileScale": tile, "surfaceDesaturation": desat, "surfaceTint": tint}


# Each OmniPBR map input, the stem of the manifest file that feeds it, and whether it is data (not colour).
MAP_INPUTS = (("diffuse_texture", "diffuse", False), ("reflectionroughness_texture", "roughness", True),
              ("normalmap_texture", "normal", True))


def texture_maps(tex: Mapping) -> List[Tuple[str, str, bool]]:
    """What texture ``tex`` (its manifest entry) feeds each OmniPBR map input: (input, file name or "" when it has none,
    raw). A file is found by its stem among the entry's ``files``, so ``diffuse.jpg`` (Poly Haven's) and ``diffuse.png``
    (one made here) both serve."""
    names = sorted(tex.get("files") or {})
    out = []
    for inp, stem, raw in MAP_INPUTS:
        hit = [n for n in names if os.path.splitext(n)[0] == stem]
        out.append((inp, hit[0] if hit else "", raw))
    return out


def settled(trace: Sequence[float], at_least: int, still: int = 3, tolerance: float = 0.25) -> bool:
    """Whether a room change has shown: at least ``at_least`` frames rendered, and the camera's mean brightness (0-255) has
    moved less than ``tolerance`` over each of the last ``still`` frames."""
    if len(trace) < max(at_least, still + 1):
        return False
    tail = trace[-(still + 1):]
    return all(abs(b - a) < tolerance for a, b in zip(tail, tail[1:]))


# The masks' stable ids (docs/adr/0032 decision 5): the same in every session, recorded in each as ``ID`` records. A
# scene object ``n`` is ``OBJECT_BASE + n``. 0 is anything unlabelled, and the background. ``clutter`` is every piece of
# the flat clutter the renderer alone draws on the table (docs/adr/0037, "Surfaces and flat clutter").
MASK_IDS = {"left_arm": 1, "left_hand": 2, "right_arm": 3, "right_hand": 4, "head": 5, "mount": 6, "arm": 7, "hand": 8,
            "table": 10, "bench": 11, "floor": 12, "mat": 13, "screen": 14, "clutter": 15}

# The JetRover's links (docs/adr/0034, robot/jetrover/urdf/jetrover.urdf): one arm, so its parts have no side.
JETROVER_ARM_LINKS = ("turntable", "proximal_link", "distal_link", "wrist_link")
JETROVER_HAND_LINKS = ("gripper_link", "left_finger", "right_finger")
OBJECT_BASE = 100


def mask_class(prim_name: str) -> Optional[str]:
    """The mask class of a robot prim: a link's ``<link>_visual`` by its side and part, the head's own meshes; None for the
    rest (the labels go on prims that never nest, or they come out joined)."""
    if prim_name.startswith("franka_head"):
        return "head"
    # The JetRover's boxes (robot/jetrover/urdf/jetrover.urdf): Isaac's importer names each after its collision body,
    # "turntable.0" as "tn__turntable0_<tag>", so the link's part is the name's letters.
    jet = re.match(r"tn__([A-Za-z]+)\d+_", prim_name)
    if jet:
        return {"turntable": "arm", "proximal": "arm", "distal": "arm", "wrist": "arm", "gripper": "hand", "Body": "mount"}.get(jet.group(1))
    if not prim_name.endswith("_visual"):
        return None
    link = prim_name[:-len("_visual")]
    # The JetRover's CAD meshes (docs/adr/0034 amendment 6): one visual per link and colour, "<link>_<k>_visual".
    jet_link = re.sub(r"_\d+$", "", link)
    if jet_link in JETROVER_ARM_LINKS or jet_link in JETROVER_HAND_LINKS:
        link = jet_link
    if link in JETROVER_ARM_LINKS:
        return "arm"
    if link in JETROVER_HAND_LINKS:
        return "hand"
    side = "left" if link.startswith("left_") else "right" if link.startswith("right_") else None
    if side is None:
        return "head" if link.startswith("head") else "mount"
    part = "hand" if ("hand" in link or "finger" in link) else "arm"
    return f"{side}_{part}"


def mask_table(object_ids: Sequence[int]) -> Dict[str, int]:
    table = dict(MASK_IDS)
    for i in object_ids:
        table[f"object_{i}"] = OBJECT_BASE + i
    return table


def stable_mask(seg, id_to_labels: Mapping, table: Mapping[str, int]):
    """A frame's segmentation (the annotator's ids, which change between frames) as stable ids, uint16."""
    import numpy as np

    lut_keys, lut_vals = [], []
    for k, v in (id_to_labels or {}).items():
        label = v.get("class") if isinstance(v, dict) else v
        if label in table:
            lut_keys.append(int(k))
            lut_vals.append(table[label])
    out = np.zeros(seg.shape, np.uint16)
    for k, v in zip(lut_keys, lut_vals):
        out[seg == k] = v
    return out


def depth_mm(depth):
    """Distance to the image plane in metres as uint16 millimetres; nothing hit (inf, nan) and beyond 65.535 m are 0."""
    import numpy as np

    d = np.asarray(depth, np.float64)
    mm = np.round(d * 1000.0)
    ok = np.isfinite(mm) & (mm > 0) & (mm <= 65535)
    return np.where(ok, mm, 0).astype(np.uint16)


class Stamps:
    """The sensed time of the state posed at each frame, so an image read at frame k is stamped with frame k - lag's."""

    def __init__(self, keep: int = 16):
        self._posed: deque = deque(maxlen=keep)

    def posed(self, sensed: int) -> None:
        self._posed.append(sensed)

    def shown(self, lag: int) -> int:
        if lag < 0 or lag >= len(self._posed):
            return camwire.UNSET
        return self._posed[-1 - lag]


# --- the renderer ----------------------------------------------------------------------------------------------------

def parse_args(argv=None) -> argparse.Namespace:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))
    a = argparse.ArgumentParser(description="the scene's cameras as live feeds (docs/adr/0032)")
    a.add_argument("--scene", required=True)
    a.add_argument("--profile", default=os.path.join(repo, "core", "RobotProfiles", "fr3-duo.json"))
    a.add_argument("--urdf", default=os.path.join(repo, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"))
    a.add_argument("--sim", default="", help="a sim sidecar (robot/jetrover/urdf/jetrover.sim.json, docs/adr/0034): its URDF and "
                                               "motors instead of --profile and --urdf")
    a.add_argument("--work", default=os.path.expanduser("~/isaacsim/work"), help="where the bridge keeps the URDF's USD")
    a.add_argument("--tap", default="/tmp/teleop-isaac-tap.sock", help="the Unix datagram socket the bridge copies its messages to")
    a.add_argument("--lag", type=int, default=-1, help="the render lag in frames; -1: measure it at start-up")
    a.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: until stopped)")
    a.add_argument("--bind", default="0.0.0.0", help="the address the feeds listen on")
    a.add_argument("--record-control", default="", help="RobotHost's session control file; while it names a session, frames "
                                                          "are recorded into it (docs/adr/0032 decision 6). Empty: never")
    a.add_argument("--record-every", type=int, default=1, help="record every this many rendered frames")
    a.add_argument("--backdrops", default="", help="the backdrop images' folder (with manifest.json); default: the scene file's")
    a.add_argument("--backdrop-seed", type=int, default=-1, help="the seed of the backdrop and room draws; -1: from the clock (it is recorded)")
    a.add_argument("--environments", default="", help="the HDRI rooms' folder (with manifest.json, docs/adr/0037); default: the scene file's")
    a.add_argument("--environment-settle-max", type=int, default=90,
                   help="after a room change, render at most this many frames, unrecorded, waiting for the new room to show")
    a.add_argument("--min-free-gb", type=float, default=20.0, help="stop recording below this much free disk")
    # How the frames are rendered (docs/adr/0036 amendment 2; measured 2026-11-15 on the 5090, three 640x360 cameras recording
    # rgb, depth and masks). Isaac's default, DLSS Performance, renders each camera at half its width and height and rebuilds
    # it from earlier frames: soft when still, smeared when the camera moves (a new pose takes ~20 frames to settle), and the
    # wrist camera always moves. DLAA (4) renders at the camera's own resolution and settles in one frame: moving wrist frames
    # 38 dB from their pose's still image and 1.4-1.7x the still detail, 30 fps with one render a pose. Isaac 6.2's Real-Time
    # 2.0 takes only 3 or 4 (anything else turns back into 3: the "0, off" asked for since 2026-11-14 was DLSS all along), and
    # opening a stage puts its settings back: run() sets them once the stage has loaded and stops if Isaac did not keep one.
    # Two path-traced samples a pixel cost nothing at 30 fps; four cost 2 fps.
    a.add_argument("--rtx-renderer", default="RealTimePathTracing", help="Isaac's renderer (RealTimePathTracing or PathTracing)")
    a.add_argument("--anti-aliasing", type=int, default=4, choices=(3, 4),
                   help="Real-Time 2.0's anti-aliasing: 4 DLAA (the camera's own resolution), 3 DLSS (upscaled from earlier frames; Isaac's default)")
    a.add_argument("--dlss-mode", type=int, default=0, choices=(0, 1, 2),
                   help="with --anti-aliasing 3: 0 performance (Isaac's default), 1 balanced, 2 quality")
    a.add_argument("--samples-per-pixel", type=int, default=2, help="Real-Time 2.0's path-traced samples a pixel, each render")
    a.add_argument("--subframes", type=int, default=1, help="render each pose this many times before reading it (Replicator's rt_subframes)")
    a.add_argument("--motion-blur", action="store_true", help="leave Isaac's motion blur setting as it is (off otherwise)")
    args = a.parse_args(argv)
    if args.subframes < 1:
        a.error("--subframes must be at least 1")
    if args.samples_per_pixel < 1:
        a.error("--samples-per-pixel must be at least 1")
    return args


def render_settings(args) -> Dict[str, object]:
    """The carb settings the frames are rendered with (see parse_args). run() sets them once the stage has loaded: opening a
    stage puts Isaac's back."""
    out: Dict[str, object] = {"/rtx/rendermode": args.rtx_renderer}
    if args.rtx_renderer == "RealTimePathTracing":
        out.update({"/rtx/post/aa/op": args.anti_aliasing, "/rtx/post/dlss/execMode": args.dlss_mode,
                    "/rtx/rtpt/spp": args.samples_per_pixel})
    if not args.motion_blur:
        out["/rtx/post/motionblur/enabled"] = False
    return out


def main(argv=None) -> int:
    args = parse_args(argv)
    sys.argv = sys.argv[:1]
    if args.sim:
        from . import servo

        sim_model = servo.SimModel.load(args.sim)
        args.urdf = os.path.join(os.path.dirname(os.path.abspath(args.sim)), sim_model.urdf)
        p = None
    else:
        sim_model = None
        p = profile_module.load(args.profile)
    scene = scene_module.load(args.scene)
    cameras = scene.extra.get("cameras") or []
    if not cameras:
        raise SystemExit(f"error: {args.scene} has no cameras")
    fps = max(c.get("fps", 30) for c in cameras)
    joints = kinematics.read_urdf(args.urdf)
    motors = SimMotorMap(sim_model) if sim_model is not None else MotorMap(p)

    if os.path.exists(args.tap):
        os.unlink(args.tap)
    tap = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    tap.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    tap.bind(args.tap)
    tap.setblocking(False)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": args.rtx_renderer, "anti_aliasing": args.anti_aliasing})
    try:
        return run(app, args, p, scene, cameras, fps, joints, motors, tap)
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        tap.close()
        if os.path.exists(args.tap):
            os.unlink(args.tap)
        app.close()


def run(app, args, p, scene, cameras, fps, joints, motors, tap) -> int:
    import io

    import numpy as np
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from importlib.metadata import version
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
    from PIL import Image
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdSemantics

    def render_frame() -> None:
        """One frame: the pose rendered ``--subframes`` times, so the renderer's history settles on it before it is read."""
        for _ in range(args.subframes):
            RenderingManager.render()

    print(f"[render] renderer {args.rtx_renderer}, anti-aliasing {args.anti_aliasing}, {args.samples_per_pixel} sample(s) a pixel, "
          f"{args.subframes} subframe(s) a frame, motion blur {'as Isaac has it' if args.motion_blur else 'off'}", flush=True)

    if args.sim:
        from .jetrover_bridge import convert_urdf
    else:
        from .bridge import convert_urdf  # each bridge's own folder prefix, so the renderer opens the USD its bridge does
    from .world import IsaacWorld, prim_name

    usd = convert_urdf(args.urdf, args.work)
    stage_utils.open_stage(usd)
    while stage_utils.is_stage_loading():
        app.update()
    # Opening the stage put Isaac's render settings back (DLSS Performance, whatever SimulationApp was given; measured
    # 2026-11-15): ours now, before the world, the cameras and the lag measurement, and checked again before streaming.
    import carb

    carb_settings = carb.settings.get_settings()
    wanted_render = render_settings(args)
    for key, value in wanted_render.items():
        carb_settings.set(key, value)

    def check_render_settings(when: str) -> None:
        kept = {key: carb_settings.get(key) for key in wanted_render}
        if kept != wanted_render:
            raise SystemExit(f"error: Isaac did not keep the render settings {when}: asked {wanted_render}, has {kept}")

    check_render_settings("after the stage loaded")
    stage = stage_utils.get_current_stage()
    robot = stage.GetDefaultPrim()
    robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    for _ in range(3):
        app.update()
    stage.SetEditTarget(stage.GetSessionLayer())  # what this process authors never touches the bridge's USD

    world = IsaacWorld(scene, str(robot.GetPath()))
    world.build(stage, physics=False)
    object_ops, object_shown = {}, {}
    for o, path in zip(scene.objects, world.object_paths):
        ops = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()
        object_ops[o.id] = (ops[0], ops[1])
        object_shown[o.id] = [UsdGeom.Imageable(stage.GetPrimAtPath(path)), True]
    domes = {}
    for light in scene.extra.get("lights") or []:
        if light.get("kind") == "dome":
            domes[light["id"]] = UsdLux.DomeLight.Define(stage, f"/World/Lights/{light['id']}")
            domes[light["id"]].CreateIntensityAttr(float(light.get("intensity", 800)))
        elif light.get("kind") == "distant":
            d = UsdLux.DistantLight.Define(stage, f"/World/Lights/{light['id']}")
            d.CreateIntensityAttr(float(light.get("intensity", 2500)))
            UsdGeom.Xformable(d).AddRotateXYZOp().Set(Gf.Vec3f(*light.get("rotateXYZDeg", [0, 0, 0])))

    # --- the backdrop: a mat on the table top and a screen beyond the table (docs/adr/0032 decision 4) -------------
    from pxr import Sdf, UsdShade

    def textured_quad(path, corners):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*c) for c in corners])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateDoubleSidedAttr(True)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        mat = UsdShade.Material.Define(stage, path + "Material")
        surf = UsdShade.Shader.Define(stage, path + "Material/Surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        tex = UsdShade.Shader.Define(stage, path + "Material/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        file_in = tex.CreateInput("file", Sdf.ValueTypeNames.Asset)
        reader = UsdShade.Shader.Define(stage, path + "Material/St")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        return mesh, file_in

    spec = scene.extra.get("backdrops") or {}
    root_dir = args.backdrops or os.path.dirname(os.path.expanduser(spec.get("manifest", ""))) or backdrops_module.DEFAULT_ROOT
    images = []
    if os.path.exists(os.path.join(root_dir, "manifest.json")):
        images = backdrops_module.load_manifest(root_dir)["images"]
    backdrop_seed = args.backdrop_seed if args.backdrop_seed >= 0 else time.time_ns() % (2 ** 31)
    quads = backdrop_quads(scene)  # (id, mask class, corners): the mats and the screens, each its own image
    quad_meshes, quad_files = {}, {}
    for qid, _, corners in quads:
        quad_meshes[qid], quad_files[qid] = textured_quad(f"/World/Backdrop/{prim_name('q', qid)}", corners)

    # --- the room: an HDRI on the dome light, background and light from one photo (docs/adr/0037 decision 2) --------
    env = environment_spec(scene)
    hdris, env_root, env_dome, env_rotate = [], "", None, None
    if env is not None:
        env_root = args.environments or os.path.dirname(os.path.expanduser(env["manifest"])) or environments_module.DEFAULT_ROOT
        if os.path.exists(os.path.join(env_root, "manifest.json")):
            hdris = environments_module.load_manifest(env_root)["hdris"]
        env_dome = domes[env["dome"]]
        env_dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
        env_rotate = UsdGeom.Xformable(env_dome).AddRotateXYZOp()
        env_rotate.Set(Gf.Vec3f(*env["rotateXYZDeg"]))
        for sid in env["hideSurfaces"]:
            UsdGeom.Imageable(stage.GetPrimAtPath(world.surface_paths[sid])).MakeInvisible()
        if not hdris:
            print(f"[render] warning: no HDRI rooms in {env_root} (just isaac-environments-fetch): a plain dome light", flush=True)

    # --- real materials on the simple shapes and the surfaces (docs/adr/0037 amendment 5, "Surfaces and flat clutter") ---
    mats = materials_spec(scene)
    textures, tex_root, tex_by_id, mat_shaders = [], "", {}, {}  # mat_shaders: object id -> its OmniPBR shader (class textures)
    look_shaders, look_choices = {}, {}  # surface id or object name -> its OmniPBR shader; -> the fetched looks it draws from

    def omnipbr(path: str, project: bool = True):
        mtl = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
        sh.SetSourceAsset("OmniPBR.mdl", "mdl")
        sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(project)  # the shapes have no UVs: projected; clutter has its own
        sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(False)  # object space: it stays on as the object moves
        sh.CreateInput("reflectionroughness_texture_influence", Sdf.ValueTypeNames.Float).Set(1.0)
        mtl.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        mtl.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        mtl.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        return mtl, sh

    def wear(sh, tex: Dict, scale_m: float, projected: bool = True) -> None:
        """Texture ``tex`` on shader ``sh``, each map its manifest entry has (``texture_maps``), tiled at its real size: on an
        object ``scale_m`` across in its projection's units, or (not ``projected``) on a mesh whose texture coordinates are
        metres."""
        d = os.path.join(tex_root, tex["id"])
        for inp, name, raw in texture_maps(tex):
            a = sh.CreateInput(inp, Sdf.ValueTypeNames.Asset)
            a.Set(Sdf.AssetPath(os.path.join(d, name) if name else ""))
            if raw:
                a.GetAttr().SetColorSpace("raw")
        k = (mats["tileScale"] * scale_m if projected else 1.0) / float(tex["sizeM"])
        sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(k, k))

    def wear_look(sh, look: Dict, scale_m: float) -> None:
        """Look ``look`` on shader ``sh``: its texture desaturated and tinted, and its roughness. Every input is set, so
        nothing of the look before stays."""
        wear(sh, tex_by_id[look["texture"]], scale_m)
        sh.CreateInput("albedo_desaturation", Sdf.ValueTypeNames.Float).Set(look["desaturation"])
        sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*look["tint"]))
        sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(
            0.5 if look["roughness"] is None else look["roughness"])
        sh.CreateInput("reflectionroughness_texture_influence", Sdf.ValueTypeNames.Float).Set(look["roughnessTexture"])

    def object_extent(index: int) -> float:
        """How far across the object's projection runs, in its units: a box is a unit cube scaled by its prim, a cylinder
        is built at its size."""
        o = scene.objects[index]
        return 1.0 if o.shape == "box" else max(o.size) * world.model.scales[index]

    surface_extent = {s.id: max(s.size[0], s.size[1]) / max(s.size) for s in scene.surfaces}  # a unit cube scaled: its long side
    object_index = {o.name: i for i, o in enumerate(scene.objects)}

    def look_extent(name: str) -> float:
        return surface_extent[name] if name in surface_extent else object_extent(object_index[name])

    if mats is not None:
        tex_root = os.path.dirname(os.path.expanduser(mats["manifest"])) or materials_module.DEFAULT_ROOT
        if os.path.exists(os.path.join(tex_root, "manifest.json")):
            textures = materials_module.load_manifest(tex_root)["textures"]
        tex_by_id = {t["id"]: t for t in textures}
        if not textures:
            print(f"[render] warning: no textures in {tex_root} (just isaac-materials-fetch): the shapes keep their colours", flush=True)
        else:
            unfetched = sorted({lk["texture"] for lk in mats["looks"].values() if lk["texture"] not in tex_by_id})
            if unfetched:
                print(f"[render] warning: textures not fetched (just isaac-materials-fetch): {', '.join(unfetched)}; the looks that "
                      f"wear them are left out of the draw", flush=True)

            def fetched(ids: Sequence[str]) -> List[str]:
                return [lid for lid in ids if mats["looks"][lid]["texture"] in tex_by_id]

            for o, path in zip(scene.objects, world.object_paths):
                if o.name in mats["objects"]:
                    if fetched(mats["objects"][o.name]):
                        mtl, sh = omnipbr(f"/World/Looks/{prim_name('m', str(o.id))}")
                        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(f"{path}/Shape")).Bind(mtl, UsdShade.Tokens.strongerThanDescendants)
                        look_shaders[o.name], look_choices[o.name] = sh, fetched(mats["objects"][o.name])
                elif o.shape in mats["classes"] and mats["classes"][o.shape]:
                    mtl, sh = omnipbr(f"/World/Looks/{prim_name('m', str(o.id))}")
                    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(f"{path}/Shape")).Bind(mtl, UsdShade.Tokens.strongerThanDescendants)
                    mat_shaders[o.id] = sh
            for sid, ids in mats["surfaces"].items():
                if sid in world.surface_paths and fetched(ids):
                    mtl, sh = omnipbr(f"/World/Looks/{prim_name('s', sid)}")
                    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(world.surface_paths[sid])).Bind(mtl, UsdShade.Tokens.strongerThanDescendants)
                    look_shaders[sid], look_choices[sid] = sh, fetched(ids)

    # --- flat clutter on the table, the renderer's alone (docs/adr/0037, "Surfaces and flat clutter") ------------------
    clutter_all = clutter_module.spec(scene)
    clut = clutter_module.usable(clutter_all, tex_by_id)  # without the kinds whose texture is not fetched
    if clutter_all is not None:
        left_out = sorted({k.kind for k in clutter_all.kinds} - {k.kind for k in (clut.kinds if clut else ())})
        if left_out:
            print(f"[render] warning: clutter kinds left out, their texture not fetched (just isaac-materials-fetch): {', '.join(left_out)}",
                  flush=True)
    clutter_now: List = []  # the pieces on the table this change

    # --- mask labels, on prims that never nest (docs/adr/0032 decision 5) ------------------------------------------
    table = mask_table([o.id for o in scene.objects])

    def label(prim, name):
        UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr().Set([name])

    # The JetRover's collision boxes, should the importer draw them: the CAD's meshes are its look now (docs/adr/0034
    # amendment 6). Its collisions are named after their bodies ("turntable.0" as "tn__turntable0_<tag>").
    hidden = 0
    if args.sim:
        for prim in Usd.PrimRange(robot):
            if re.match(r"tn__[A-Za-z]+\d+_", prim.GetName()) and prim.IsA(UsdGeom.Imageable) and not prim.IsInstanceProxy():
                UsdGeom.Imageable(prim).MakeInvisible()
                hidden += 1
        print(f"[render] JetRover: {hidden} collision prims hidden, its CAD meshes drawn", flush=True)

    labelled = 0
    for prim in Usd.PrimRange(robot):
        cls = mask_class(prim.GetName())
        if cls is not None and not prim.IsInstanceProxy():
            label(prim, cls)
            labelled += 1
    for sid, path in world.surface_paths.items():
        if sid in MASK_IDS:
            label(stage.GetPrimAtPath(path), sid)
    for o, path in zip(scene.objects, world.object_paths):
        label(stage.GetPrimAtPath(path), f"object_{o.id}")
    for qid, cls, _ in quads:
        label(quad_meshes[qid].GetPrim(), cls)

    # The lag cube, labelled so the lag camera's segmentation finds it; hidden once the lag is measured.
    cube = UsdGeom.Cube.Define(stage, "/World/LagCube")
    cube.CreateSizeAttr(1.0)
    cube_xf = UsdGeom.Xformable(cube)
    cube_t = cube_xf.AddTranslateOp()
    cube_xf.AddScaleOp().Set(Gf.Vec3f(LAG_CUBE, LAG_CUBE, LAG_CUBE))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.2, 0.1)])
    UsdSemantics.LabelsAPI.Apply(cube.GetPrim(), "class").CreateLabelsAttr().Set(["lag_cube"])

    links = {x.GetName(): x for x in Usd.PrimRange(robot) if x.GetTypeName() == "Xform"}
    moving = [j for j in joints.values() if j.type in ("revolute", "prismatic", "continuous") and j.child in links]
    view = XformPrim([str(links[j.child].GetPath()) for j in moving])
    rest_t, rest_q = (np.asarray(x.numpy() if hasattr(x, "numpy") else x, float) for x in view.get_local_poses())

    # The render lag is measured with the first camera that stands still (the wrist cameras move with the arms).
    lag_i = next((i for i, c in enumerate(cameras) if "mount" not in c or c["mount"] in ("head_camera_mounting_point",)), None)
    if lag_i is None:
        raise SystemExit("error: the scene has no camera that stands still, to measure the render lag with")
    sensors, feeds, specs, riding = [], [], [], []  # riding: (camera, mount link, its pose in that link's frame)
    for i, c in enumerate(cameras):
        w, h, hfov = int(c["width"]), int(c["height"]), float(c["hfovDeg"])
        focal = APERTURE_MM / (2 * math.tan(math.radians(hfov) / 2))
        attrs = {"focalLength": focal, "horizontalAperture": APERTURE_MM, "verticalAperture": APERTURE_MM * h / w,
                 "clippingRange": Gf.Vec2f(float(c.get("near", 0.05)), float(c.get("far", 10.0)))}
        if "mount" in c:
            if c["mount"] not in links:
                raise SystemExit(f"error: camera {c['id']}'s mount {c['mount']!r} is not a link of the robot")
            # Not a child of its link: Isaac applies a camera's transform one frame sooner than the posed links (measured
            # 2026-11-03: a wrist camera parented to its hand showed the hand's pose of the next frame's state, in every
            # moving frame). So the renderer poses it itself, each frame from the state the links had one frame earlier,
            # and image and camera show the same state (docs/adr/0032 amendment 3).
            local_t, local_q = mounted_camera_local(c)
            world_p, world_q = compose(kinematics.link_pose(joints, {}, c["mount"]), (local_t, local_q))
            cam = RtxCamera(f"/World/Cameras/{prim_name('Cam', c['id'])}", positions=[list(world_p)],
                            orientations=[list(world_q)], attributes=attrs)
            riding.append((cam, c["mount"], (local_t, local_q)))
        else:
            cam = RtxCamera(f"/World/Cameras/{prim_name('Cam', c['id'])}", positions=[list(c["position"])],
                            orientations=[list(look_at(c["position"], c["lookAt"]))], attributes=attrs)
        records = list(c.get("record") or [])
        annotators = ["rgb"]
        if i == lag_i or "mask" in records:
            annotators.append("semantic_segmentation")
        if "depth" in records:
            annotators.append("distance_to_image_plane")
        sensors.append((cam, CameraSensor(cam, resolution=(h, w), annotators=annotators)))
        feeds.append(camfeed.CameraFeed(c["id"], int(c["feedPort"]), bind=args.bind))
        specs.append({"id": c["id"], "width": w, "height": h, "quality": int(c.get("feedQuality", 80)), "record": records,
                      "recordQuality": int(c.get("recordQuality", 92)), "near": float(c.get("near", 0.05)),
                      "far": float(c.get("far", 10.0)), "mount": c.get("mount", ""), **intrinsics(w, h, hfov)})
    app_utils.play(commit=True)  # sensors deliver only while the timeline plays; with Physics at "none" nothing steps

    def read(sensor, name):
        data, info = sensor.get_data(name)
        if data is None:
            return None, info
        a = data.numpy() if hasattr(data, "numpy") else np.asarray(data)
        return (a[..., 0] if a.ndim == 3 and a.shape[-1] == 1 else a), info

    for k in range(600):
        RenderingManager.render()
        if read(sensors[lag_i][1], "rgb")[0] is not None and k > 5:
            break

    # --- the render lag --------------------------------------------------------------------------------------------
    head_cam, head_sensor = sensors[lag_i]
    hp, hq = (np.asarray(x.numpy() if hasattr(x, "numpy") else x, float).reshape(-1) for x in head_cam.get_world_poses())
    hrot = np.array([[1 - 2 * (hq[2] ** 2 + hq[3] ** 2), 2 * (hq[1] * hq[2] - hq[0] * hq[3]), 2 * (hq[1] * hq[3] + hq[0] * hq[2])],
                     [2 * (hq[1] * hq[2] + hq[0] * hq[3]), 1 - 2 * (hq[1] ** 2 + hq[3] ** 2), 2 * (hq[2] * hq[3] - hq[0] * hq[1])],
                     [2 * (hq[1] * hq[3] - hq[0] * hq[2]), 2 * (hq[2] * hq[3] + hq[0] * hq[1]), 1 - 2 * (hq[1] ** 2 + hq[2] ** 2)]])
    s0 = specs[lag_i]

    def project(x):
        xc = hrot.T @ (np.asarray(x, float) - hp[:3])
        if xc[2] >= -1e-6:
            return None
        return s0["cx"] + s0["fx"] * xc[0] / -xc[2], s0["cy"] - s0["fy"] * xc[1] / -xc[2]

    lag, agree, samples = args.lag, 1.0, []
    if args.lag < 0:
        # On the table, in that camera's view, with the scene's objects hidden so none hides it.
        objects_root = UsdGeom.Imageable(stage.GetPrimAtPath(f"{IsaacWorld.ROOT}/Objects"))
        objects_root.MakeInvisible()
        history = []
        top = scene.table.top
        for k in range(90):
            pos = (0.35 + 0.4 * ((k * 7) % 11) / 10, -0.3 + 0.6 * ((k * 3) % 7) / 6, top + LAG_CUBE / 2 + 0.001)
            cube_t.Set(Gf.Vec3d(*pos))
            history.append(pos)
            render_frame()  # as the frames are rendered: the lag is in frames of subframes
            seg, info = read(head_sensor, "semantic_segmentation")
            ids = {v.get("class"): int(i) for i, v in ((info or {}).get("idToLabels") or {}).items()}
            if seg is None or "lag_cube" not in ids:
                continue
            vv, uu = np.nonzero(seg == ids["lag_cube"])
            if len(uu) < 30:
                continue
            centre = (uu.mean() + 0.5, vv.mean() + 0.5)
            best = None
            for j in range(min(4, k + 1)):
                pr = project(history[k - j])
                if pr is not None:
                    e = math.hypot(pr[0] - centre[0], pr[1] - centre[1])
                    if best is None or e < best[0]:
                        best = (e, j)
            if best is not None and best[0] < 8:
                samples.append(best[1])
        objects_root.MakeVisible()
        lag, agree = estimate_lag(samples)
        if lag is None:
            print("[render] warning: the render lag could not be measured (the cube was never seen); using 2", flush=True)
            lag = 2
    UsdGeom.Imageable(cube).MakeInvisible()
    cube_t.Set(Gf.Vec3d(0.0, 0.0, -5.0))
    print(f"[render] Isaac Sim {version('isaacsim')}; render lag {lag} frame(s) ({agree:.1%} of {len(samples)} samples); "
          f"{len(moving)} moving links", flush=True)

    # --- the cameras' poses, for the recording ----------------------------------------------------------------------
    # A camera that stands still: its pose in the robot's base frame. A camera on a link: its pose in that link's URDF
    # frame, as Isaac rendered it (the robot is still at its rest pose, every joint 0), so a reader poses it at every
    # frame from the frame's state by forward kinematics (docs/adr/0032 amendment 3).
    camera_poses = []
    for (cam, _), c in zip(sensors, specs):
        cp, cq = (np.asarray(x.numpy() if hasattr(x, "numpy") else x, float).reshape(-1) for x in cam.get_world_poses())
        at = (tuple(float(v) for v in cp[:3]), tuple(float(v) for v in cq[:4]))
        if c["mount"]:
            local_p, local_q = relative_pose(kinematics.link_pose(joints, {}, c["mount"]), at)
            camera_poses.append((*local_p, *local_q))
        else:
            camera_poses.append((*at[0], *at[1]))

    # --- backdrops: one seeded pair at start-up and after every recorded session (docs/adr/0032 decision 4) ------------
    backdrop = {"change": -1, "images": {}, "settle_until": -1}  # images: quad id -> manifest entry
    used_this_session = []
    room = {"hdri": None, "yaw": 0.0, "intensity": 0.0, "base": 0.0, "settle": 0, "trace": [], "brightness": 0.0, "gains": {}}
    worn = {}  # object id -> texture it wears this change (docs/adr/0037 amendment 5)
    looks_worn = {}  # surface id or object name -> the look it wears this change ("Surfaces and flat clutter")

    def change_materials() -> None:
        worn.clear()
        looks_worn.clear()
        for index, o in enumerate(scene.objects):
            sh = mat_shaders.get(o.id)
            if sh is None:
                continue
            tex = materials_module.pick(textures, mats["classes"][o.shape], backdrop_seed, backdrop["change"], o.id)
            if tex is not None:
                wear(sh, tex, object_extent(index))
                worn[o.id] = tex
        for name, ids in look_choices.items():  # each surface and looked object its own draw, by the looks' weights
            draw = materials_module.surface_look if name in surface_extent else materials_module.object_look
            lid = draw(mats["looks"], ids, backdrop_seed, backdrop["change"], name)
            wear_look(look_shaders[name], mats["looks"][lid], look_extent(name))
            looks_worn[name] = lid

    def lk_items():
        return [[name, lid] for name, lid in sorted(looks_worn.items())]

    clutter_root = "/World/Clutter"

    def rebuild_clutter() -> None:
        """This change's flat clutter (clutter.draw), built afresh under /World/Clutter: a mesh per piece, a sheet wearing its
        texture by its own texture coordinates and tinted, a cable or bag a plain plastic, each labelled ``clutter``."""
        clutter_now.clear()
        if stage.GetPrimAtPath(clutter_root):
            stage.RemovePrim(clutter_root)
        if clut is None:
            return
        UsdGeom.Xform.Define(stage, clutter_root)
        for n, piece in enumerate(clutter_module.draw(clut, backdrop_seed, backdrop["change"])):
            geo = clutter_module.mesh(piece, scene.table.top)
            m = UsdGeom.Mesh.Define(stage, f"{clutter_root}/{prim_name('c', f'{n:02d}_{piece.kind}')}")
            m.CreatePointsAttr([Gf.Vec3f(*v) for v in geo.points])
            m.CreateFaceVertexCountsAttr(geo.counts)
            m.CreateFaceVertexIndicesAttr(geo.indices)
            m.CreateNormalsAttr([Gf.Vec3f(*v) for v in geo.normals])
            m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
            m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            m.CreateDoubleSidedAttr(True)
            mtl, sh = omnipbr(f"{clutter_root}/Looks/{prim_name('m', str(n))}", project=False)
            if piece.texture:
                st = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
                st.Set([Gf.Vec2f(*v) for v in geo.st])
                wear(sh, tex_by_id[piece.texture], 1.0, projected=False)
                sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*piece.colour))
            else:
                sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*piece.colour))
                sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.5 if piece.roughness is None else piece.roughness)
                sh.CreateInput("reflectionroughness_texture_influence", Sdf.ValueTypeNames.Float).Set(0.0)
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mtl)
            label(m.GetPrim(), "clutter")
            clutter_now.append(piece)
    camera_luts = [None] * len(specs)  # each camera's own exposure, locked per room (docs/adr/0037 decision 2)
    rooms_this_session = []

    def change_room() -> bool:
        """The next room on the dome (docs/adr/0037 decision 2); True if the dome changed and must settle."""
        if env is None or not hdris:
            return False
        hdri, yaw = environments_module.pick(hdris, backdrop_seed, backdrop["change"], env["yawDeg"])
        intensity = environments_module.dome_intensity(env["meanRadiance"], hdri)
        env_dome.GetTextureFileAttr().Set(os.path.join(env_root, hdri["file"]))
        env_dome.GetIntensityAttr().Set(intensity)
        rx, ry, rz = env["rotateXYZDeg"]
        env_rotate.Set(Gf.Vec3f(rx, ry, rz + yaw))
        room.update(hdri=hdri, yaw=yaw, intensity=round(intensity, 3), base=intensity, settle=0, trace=[], brightness=0.0)
        rooms_this_session.append(hdri)
        return True

    def en_fields():
        h = room["hdri"] or {}
        return (time.monotonic_ns(), backdrop["change"], backdrop_seed, h.get("id", ""), h.get("sha256", ""), room["yaw"],
                room["intensity"], room["settle"], room["brightness"],
                [[c["id"], room["gains"].get(c["id"], 1.0)] for c in specs])

    def change_backdrop(frame_now: int) -> None:
        backdrop["change"] += 1
        change_materials()
        session_record("MT", backdrop["change"], [[oid, t["id"]] for oid, t in sorted(worn.items())])
        if looks_worn:
            session_record("LK", backdrop["change"], lk_items())
        rebuild_clutter()  # before the room settles: the cameras meter the table as it will be recorded
        if clutter_all is not None:
            session_record("CL", backdrop["change"], clutter_module.cl_items(clutter_now))
        if change_room():
            settle_room()
            session_record("EN", *en_fields())
        if not quads:
            backdrop["images"] = {}
            backdrop["settle_until"] = frame_now + lag + 1
            return
        if len(images) < len(quads):
            backdrop["images"] = {}  # too few images fetched: plain surfaces, and no screens
            for mesh in quad_meshes.values():
                UsdGeom.Imageable(mesh).MakeInvisible()
            return
        picked = backdrops_module.pick(images, backdrop_seed, backdrop["change"], len(quads))
        backdrop["images"] = {qid: img for (qid, _, _), img in zip(quads, picked)}
        for qid, img in backdrop["images"].items():
            quad_files[qid].Set(os.path.join(root_dir, img["file"]))
        backdrop["settle_until"] = frame_now + lag + 1  # a new texture shows within the render lag (the spike: 2 frames)
        session_record("BK", *bk_fields())
        used_this_session.extend(backdrop["images"].values())

    def bk_fields():
        return (time.monotonic_ns(), backdrop["change"], backdrop_seed,
                [[qid, img["sha256"], img["imageId"]] for qid, img in backdrop["images"].items()])

    # --- recording --------------------------------------------------------------------------------------------------
    session = None
    want_backdrop_change = []
    world_snapshot = {"sensed": 0, "revision": -1, "objects": []}
    # Each object's size factor (docs/adr/0034 amendment 6), from the bridge's SIZES: applied to the renderer's copy and recorded.
    world_sizes = {"revision": -1, "sizes": [[o.id, 1.0] for o in scene.objects]}
    streams = [f"{c['id']}.{st}" for c in specs for st in c["record"]]
    scene_sha = hashlib.sha256(open(args.scene, "rb").read()).hexdigest()

    def session_record(tag, *fields):
        if session is not None:
            session.record(tag, *fields)

    def preamble():
        used_this_session.clear()
        used_this_session.extend(backdrop["images"].values())
        rooms_this_session.clear()
        if room["hdri"] is not None:
            rooms_this_session.append(room["hdri"])
        out = [("IV", version("isaacsim"), fps, args.record_every, lag, round(agree, 4), len(samples)),
               ("RX", args.rtx_renderer, args.anti_aliasing, args.subframes, 1 if args.motion_blur else 0, args.samples_per_pixel),
               ("SC", scene.name, scene_sha)]
        for c, pose in zip(specs, camera_poses):
            out.append(("CM", c["id"], c["width"], c["height"], round(c["fx"], 6), round(c["fy"], 6), c["cx"], c["cy"], c["near"],
                        c["far"], *[round(v, 7) for v in pose], c["mount"], [[st] for st in c["record"]],
                        "mount" if c["mount"] else "base"))
        out.append(("ID", [[v, k] for k, v in sorted(table.items(), key=lambda kv: kv[1])]))
        out.append(("BK", *bk_fields()))
        if env is not None:
            out.append(("EN", *en_fields()))
        if worn:
            out.append(("MT", backdrop["change"], [[oid, t["id"]] for oid, t in sorted(worn.items())]))
        if looks_worn:
            out.append(("LK", backdrop["change"], lk_items()))
        if clutter_all is not None:
            out.append(("CL", backdrop["change"], clutter_module.cl_items(clutter_now)))
        if world_snapshot["objects"]:
            out.append(("OL", world_snapshot["sensed"], world_snapshot["revision"], world_snapshot["objects"]))
        out.append(("SZ", max(0, world_sizes["revision"]), world_sizes["sizes"]))
        return out

    def final_files(directory):
        seen, items = set(), []
        for e in used_this_session:
            if e["sha256"] not in seen:
                seen.add(e["sha256"])
                items.append({k: e[k] for k in ("sha256", "imageId", "license", "author", "authorProfileUrl", "title",
                                                 "landingUrl", "source", "url", "width", "height")})
        doc = {"format": "teleop-backdrops-used", "version": 1, "license": backdrops_module.CC_BY_2, "images": items}
        out = {"backdrops.json": (json.dumps(doc, indent=1, sort_keys=True) + "\n").encode("utf-8")}
        if env is not None:
            seen, rooms = set(), []
            for e in rooms_this_session:
                if e["sha256"] not in seen:
                    seen.add(e["sha256"])
                    rooms.append({k: e[k] for k in ("id", "sha256", "name", "authors", "license", "source", "url", "landingUrl",
                                                     "width", "height", "meanLuminance")})
            # Every texture the shapes, the surfaces' looks and the clutter wore; one made here says so in its source.
            used = {t["id"]: t for t in worn.values()}
            used.update({mats["looks"][lid]["texture"]: tex_by_id[mats["looks"][lid]["texture"]] for lid in looks_worn.values()})
            used.update({c.texture: tex_by_id[c.texture] for c in clutter_now if c.texture})
            credits = [{"id": t["id"], "class": t.get("class", ""), "name": t.get("name", ""), "authors": t.get("authors", []),
                        "license": t.get("license", ""), "landingUrl": t.get("landingUrl", ""), "sizeM": t.get("sizeM"),
                        "source": t.get("source", "Poly Haven (polyhaven.com)")} for t in sorted(used.values(), key=lambda t: t["id"])]
            doc = {"format": "teleop-environments-used", "version": 1, "license": environments_module.CC0,
                   "meanRadiance": env["meanRadiance"], "hdris": rooms, "textures": credits}
            out["environment.json"] = (json.dumps(doc, indent=1, sort_keys=True) + "\n").encode("utf-8")
        return out

    if args.record_control:
        if agree < 0.995 or not samples and args.lag < 0:
            print(f"[render] warning: not recording: the render lag agreed in {agree:.1%} of {len(samples)} samples (needs 99.5%)", flush=True)
        else:
            from .isaac_session import IsaacSession
            session = IsaacSession(args.record_control, streams, preamble=preamble, final_files=final_files,
                                   on_closed=lambda d: want_backdrop_change.append(d), min_free_bytes=int(args.min_free_gb * 1024 ** 3),
                                   poll_seconds=0.1)  # frames after RobotHost closes a session have no QS: keep that tail short
            print(f"[render] recording {', '.join(streams)} every {args.record_every} frame(s) into the sessions {args.record_control} names", flush=True)
    recorder = ThreadPoolExecutor(max_workers=1, thread_name_prefix="record")
    record_pending = [0]
    record_lock = threading.Lock()
    record_skipped = [0]

    def encode_and_record(frame_no, sensed, flags, arrays):
        try:
            import cv2

            blobs = {}
            for name, (kind, a, quality) in arrays.items():
                if kind == "rgb":
                    buf = io.BytesIO()
                    Image.fromarray(a).save(buf, "JPEG", quality=quality)
                    blobs[name] = buf.getvalue()
                else:
                    ok, enc = cv2.imencode(".png", a)
                    if ok:
                        blobs[name] = enc.tobytes()
            if session is not None:
                session.frame(frame_no, sensed, flags, blobs)
        finally:
            with record_lock:
                record_pending[0] -= 1

    # --- streaming -------------------------------------------------------------------------------------------------
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    encoders = [ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"jpeg-{s['id']}") for s in specs]
    pending = [0] * len(specs)
    skipped = [0] * len(specs)
    pending_lock = threading.Lock()

    def encode_and_send(i: int, rgb, capture: int) -> None:
        try:
            buf = io.BytesIO()
            Image.fromarray(rgb).save(buf, "JPEG", quality=specs[i]["quality"])
            feeds[i].offer(buf.getvalue(), capture, specs[i]["width"], specs[i]["height"])
        finally:
            with pending_lock:
                pending[i] -= 1

    stamps = Stamps()
    frame_values: deque = deque(maxlen=16)  # the joint values posed at each frame, for the VQ records
    state_sensed, state_values = camwire.UNSET, {}

    def settle_room() -> None:
        """Render the pose again, unrecorded, until the new room shows: the dome's texture loads in the background, so the
        frames right after a change can still show the old room or none (measured on the 5090, docs/adr/0037). Every
        render is stamped as posed, so the lag's frame association holds. Between sessions, so nothing recorded waits."""
        trace = room["trace"]

        def until_settled():
            start = len(trace)
            for _ in range(max(0, args.environment_settle_max)):
                stamps.posed(state_sensed)
                frame_values.append(state_values)
                render_frame()
                rgb, _ = read(sensors[lag_i][1], "rgb")
                if rgb is not None:
                    trace.append(round(float(np.asarray(rgb[::8, ::8, :3], np.float32).mean()), 2))
                if settled(trace[start:], at_least=lag + 3):
                    return True
            return False

        ok = until_settled()
        steps = []
        target = env["targetBrightness"]
        # The camera's auto-exposure (docs/adr/0037 decision 2): meter the still camera's image, set the dome toward the
        # target, settle again; a few steps at most, and within METER_RANGE of the luminance-normalised intensity.
        while ok and target is not None and trace and abs(trace[-1] - target) > METER_TOLERANCE and len(steps) < METER_STEPS:
            new = metered_intensity(room["intensity"], room["base"], trace[-1], target)
            if abs(new - room["intensity"]) < 1e-3 * room["intensity"]:
                break  # at the edge of the range
            steps.append(round(new))
            room["intensity"] = round(new, 3)
            env_dome.GetIntensityAttr().Set(new)
            ok = until_settled()
        room["settle"] = len(trace)
        room["brightness"] = trace[-1] if trace else 0.0
        # Every other camera meters its own image, as its own auto-exposure would, and keeps that gain for the session (the
        # arm is at rest between takes, so a wrist camera meters its rest view). The still camera the dome was set by keeps 1.
        room["gains"] = {}
        for i, (_, sensor) in enumerate(sensors):
            camera_luts[i] = None
            if i == lag_i or target is None:
                continue
            rgb, _ = read(sensor, "rgb")
            if rgb is None:
                continue
            g = round(camera_gain(float(np.asarray(rgb[::8, ::8, :3], np.float32).mean()), target), 3)
            room["gains"][specs[i]["id"]] = g
            camera_luts[i] = exposure_lut(g)
        h = room["hdri"] or {}
        print(f"[render] room {h.get('id')} (yaw {room['yaw']:.0f} deg, dome {room['base']:.0f} -> {room['intensity']:.0f} in "
              f"{len(steps)} metering step(s)): {'settled' if ok else 'NOT settled'} after {len(trace)} frames at brightness "
              f"{room['brightness']}; camera gains {room['gains']}", flush=True)

    change_backdrop(0)
    posed_values = {}  # the joint values the links were posed with at the previous frame
    states_in = worlds_in = 0
    frame_s = 1.0 / fps
    start = time.perf_counter()
    stop_at = start + args.seconds if args.seconds > 0 else None
    report_at = start + 5.0
    frames = window_frames = recorded = 0
    for c, f in zip(specs, feeds):
        print(f"[render] camera {c['id']}: {c['width']}x{c['height']}, fx {c['fx']:.1f}, feed on UDP :{f.port}, records {c['record']}", flush=True)
    print(f"[render] backdrops: {len(images) if quads else 0} images from {root_dir}, seed {backdrop_seed}; {labelled} robot prims labelled for masks", flush=True)
    if env is not None:
        print(f"[render] rooms: {len(hdris)} HDRIs from {env_root}, mean radiance {env['meanRadiance']}, yaw {env['yawDeg']}, "
              f"hidden {env['hideSurfaces']}", flush=True)
    check_render_settings("at the start of the stream")
    print(f"[render] streaming at {fps} fps", flush=True)
    while not stop and (stop_at is None or time.perf_counter() < stop_at):
        while True:
            try:
                d = tap.recv(wire.MAX_BYTES)
            except (BlockingIOError, InterruptedError):
                break
            k = wire.kind_of(d)
            if k == wire.STATE:
                st = wire.decode_state(d)
                if st is not None and st[0] > state_sensed:
                    state_sensed, state_values = st[0], motors.values([(m, q) for m, q, _, _ in st[3]])
                    states_in += 1
            elif k == wire.WORLD:
                w = wire.decode_world(d)
                if w is not None:
                    worlds_in += 1
                    for o in w[5]:
                        if o.id in object_ops:
                            t_op, q_op = object_ops[o.id]
                            t_op.Set(Gf.Vec3d(*o.pose[:3]))
                            q_op.Set(Gf.Quatd(*o.pose[3:]))
                            # A parked object (docs/adr/0037 decision 3) is out of sight on the shelf already; hidden, too.
                            shown = not (o.flags & wire.FLAG_PARKED)
                            if object_shown[o.id][1] != shown:
                                (object_shown[o.id][0].MakeVisible if shown else object_shown[o.id][0].MakeInvisible)()
                                object_shown[o.id][1] = shown
                    if w[2] != world_snapshot["revision"] and w[3] == w[4] - 1:
                        world_snapshot.update(sensed=w[0], revision=w[2],
                                              objects=[[o.id, *[round(v, 6) for v in o.pose]] for o in w[5]])
                        session_record("OL", w[0], w[2], world_snapshot["objects"])
            elif k == wire.SIZES:
                z = wire.decode_sizes(d)
                if z is not None and z[0] != world_sizes["revision"]:
                    sizes = {i: f for i, f in z[1]}
                    for index, o in enumerate(scene.objects):
                        f = sizes.get(o.id)
                        if f is not None and abs(f - world.model.scales[index]) > 1e-6:
                            world.set_scale(index, f)
                            if o.id in worn:  # a cylinder's projection runs in metres: retile it at its new size
                                wear(mat_shaders[o.id], worn[o.id], object_extent(index))
                            elif o.name in looks_worn:
                                wear_look(look_shaders[o.name], mats["looks"][looks_worn[o.name]], object_extent(index))
                    world_sizes.update(revision=z[0], sizes=[[o.id, round(world.model.scales[i], 4)] for i, o in enumerate(scene.objects)])
                    session_record("SZ", z[0], world_sizes["sizes"])
        for f in feeds:
            f.poll()
        if want_backdrop_change:
            want_backdrop_change.clear()
            change_backdrop(frames)

        for cam, mount, local in riding:  # the state the links had one frame ago: see where the cameras are made
            cp, cq = compose(kinematics.link_pose(joints, posed_values, mount), local)
            cam.set_world_poses(positions=[list(cp)], orientations=[list(cq)])
        posed_values = state_values
        ts, qs = rest_t.copy(), rest_q.copy()
        for i, j in enumerate(moving):
            t_i, q_i = kinematics.local_motion(j, tuple(rest_t[i]), tuple(rest_q[i]), state_values.get(j.name, 0.0))
            ts[i], qs[i] = t_i, q_i
        view.set_local_poses(ts, qs)
        stamps.posed(state_sensed)
        frame_values.append(state_values)
        render_frame()
        capture = stamps.shown(lag)
        recording = session is not None and session.active and frames % max(1, args.record_every) == 0 and capture != camwire.UNSET
        if recording:
            with record_lock:
                if record_pending[0] >= 3:
                    recording = False
                    record_skipped[0] += 1
                    session_record("FD", time.monotonic_ns(), frames, "encoder-behind")
        arrays = {}
        for i, (_, sensor) in enumerate(sensors):
            c = specs[i]
            send = feeds[i].wanted()
            keep = recording and bool(c["record"])
            if not send and not keep:
                continue
            rgb, _ = read(sensor, "rgb")
            if rgb is None:
                if keep:
                    session_record("FD", time.monotonic_ns(), frames, f"{c['id']}-no-image")
                continue
            rgb = np.ascontiguousarray(rgb[..., :3])
            if camera_luts[i] is not None:
                rgb = camera_luts[i][rgb]
            if send:
                with pending_lock:
                    behind = pending[i] >= 2
                if behind:
                    skipped[i] += 1  # the encoder is behind: drop this frame rather than queue it
                else:
                    with pending_lock:
                        pending[i] += 1
                    encoders[i].submit(encode_and_send, i, rgb, capture)
            if keep:
                if "rgb" in c["record"]:
                    arrays[f"{c['id']}.rgb"] = ("rgb", rgb, c["recordQuality"])
                if "depth" in c["record"]:
                    dep, _ = read(sensor, "distance_to_image_plane")
                    if dep is not None:
                        arrays[f"{c['id']}.depth"] = ("png", depth_mm(dep), 0)
                if "mask" in c["record"]:
                    seg, info = read(sensor, "semantic_segmentation")
                    if seg is not None:
                        arrays[f"{c['id']}.mask"] = ("png", stable_mask(seg, (info or {}).get("idToLabels") or {}, table), 0)
        if recording and arrays:
            # The joint values the frame shows (the state posed lag frames ago): a session with no QS records (the JetRover's,
            # docs/adr/0034 J5) still knows what each frame shows.
            if lag < len(frame_values):
                session_record("VQ", frames, capture, [[name, round(v, 6)] for name, v in sorted(frame_values[-1 - lag].items())])
            flags = "B" if frames <= backdrop["settle_until"] else ""
            with record_lock:
                record_pending[0] += 1
            recorder.submit(encode_and_record, frames, capture, flags, arrays)
            recorded += 1
        frames += 1
        window_frames += 1

        now = time.perf_counter()
        if now >= report_at:
            age = (time.monotonic_ns() - state_sensed) / 1e6 if state_sensed != camwire.UNSET else float("nan")
            sent = ", ".join(f"{f.name} {f.frames_sent} sent ({f.bytes_sent / 1e6:.1f} MB)" for f in feeds)
            rate = window_frames / (now - report_at + 5.0)
            rec = f"; recording {session.directory} ({session.frames_written} written, {session.frames_dropped} dropped)" \
                if session is not None and session.active else ""
            print(f"[render] {rate:.1f} fps; states {states_in}, world {worlds_in}; newest state {age:.0f} ms old; {sent}; "
                  f"skipped {skipped}{rec}", flush=True)
            session_record("RH", time.monotonic_ns(), round(rate, 2), round(age, 1), skipped[0], skipped[-1], record_skipped[0], list(skipped))
            report_at, window_frames = now + 5.0, 0
        wait = start + frames * frame_s - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        elif wait < -0.5:
            start = time.perf_counter() - frames * frame_s  # far behind: don't try to catch up

    for e in encoders:
        e.shutdown(wait=True)
    recorder.shutdown(wait=True)
    if session is not None:
        session.close()
        for d, n, dropped in session.closed_sessions:
            print(f"[render] recorded {n} frames ({dropped} dropped) into {d}", flush=True)
    summary = "; ".join(f"{f.name}: {f.frames_sent} frames sent, {f.frames_paced} paced, {f.frames_failed} failed" for f in feeds)
    print(f"[render] stopped after {frames} frames; {summary}", flush=True)
    for f in feeds:
        f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
