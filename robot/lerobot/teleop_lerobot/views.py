"""The twin's view cameras as a take records them (docs/adr/0038 "V, views"; trainer/contract/SESSION_FORMAT.md ``VS``,
``VW``, ``VP``), read for a safety build that asks for them (``safety-build --views``). Plain Python, no numpy.

A view camera is not a ``CM`` camera: every other reader here keys on ``CM`` and skips the views' parts of ``VF``. Only
with ``--views`` does a build read them:

- ``VS`` ``change`` ``seed`` ``sha256`` ``count``: the views in force from that change (one per session's preamble, and one
  at each change);
- ``VW``, one per camera of that change: intrinsics and pose as ``CM``'s (the USD camera, looking along its -z with +y up, in
  the robot's base frame), ``streams``, ``mode`` (``static``, ``moving`` or ``fixed``), ``draw``, ``gain``, ``motion``,
  ``broken``;
- ``VP`` ``frame`` ``stateSensed`` ``poses``: each moving camera's pose at a recorded frame (the pose the frame's image was
  rendered from: SESSION_FORMAT.md's frame association).

A take holds the records of one change (its session's preamble). Records of more than one change are read as the last
``VS``'s, the views in force when the take ended, and refused when its ``VW`` lack a camera.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

MODES = ("static", "moving", "fixed")
DRAW = ("hfovDeg", "azimuthDeg", "elevationDeg", "distanceM", "rollDeg")
MOTION = ("periodS", "azAmpDeg", "elAmpDeg", "distAmpM", "targetAmpM", "phi1", "phi2", "phi3", "phi4")


class ViewsError(ValueError):
    """A take's view records cannot be read as SESSION_FORMAT.md defines them, or it has none."""


@dataclass(frozen=True)
class ViewCamera:
    """One ``VW``: a view camera of the take's change."""

    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    near: float
    far: float
    pose: Tuple[float, ...]  # x y z qw qx qy qz, base frame; a moving camera's loop centre
    streams: Tuple[str, ...]
    mode: str
    draw: Dict[str, object]
    gain: Optional[float]
    motion: Optional[Dict[str, float]]
    broken: Tuple[str, ...]


@dataclass
class Views:
    """A take's views: the ``VS`` in force, its cameras, and each moving camera's pose per recorded frame (``VP``)."""

    change: int
    seed: int
    sha256: str
    count: int
    cameras: List[ViewCamera]
    poses: Dict[int, Dict[str, Tuple[float, ...]]] = field(default_factory=dict)  # recorded frame -> camera -> pose
    source: Optional[str] = None  # isaac/views.json's "source": the spec's file name

    def camera(self, name: str) -> ViewCamera:
        return next(c for c in self.cameras if c.name == name)


def _floats(text: str, n: int, what: str) -> List[float]:
    parts = text.split(":")
    if len(parts) != n:
        raise ViewsError(f"{what} has {len(parts)} fields, not {n}: {text!r}")
    return [float(x) for x in parts]


def parse_vw(f: Sequence[str]) -> Tuple[int, ViewCamera]:
    """A ``VW`` record's fields (after its tag): its change and camera."""
    if len(f) < 22:
        raise ViewsError(f"a VW record has {len(f)} fields, not 22 or 23: {'|'.join(f)!r}")
    mode = f[18]
    if mode not in MODES:
        raise ViewsError(f"VW {f[1]}: mode {mode!r} is not one of {MODES}")
    d = _floats(f[19], 9, f"VW {f[1]}'s draw")
    draw: Dict[str, object] = {k: v for k, v in zip(DRAW, d[:5])}
    draw["target"] = d[5:8]
    draw["attempts"] = int(d[8])
    motion = {k: v for k, v in zip(MOTION, _floats(f[21], 9, f"VW {f[1]}'s motion"))} if f[21] else None
    if (motion is None) != (mode != "moving"):
        raise ViewsError(f"VW {f[1]}: mode {mode} with {'no ' if motion is None else ''}motion")
    gain = float(f[20]) if f[20] else None
    return int(f[0]), ViewCamera(f[1], int(f[2]), int(f[3]), float(f[4]), float(f[5]), float(f[6]), float(f[7]), float(f[8]),
                                 float(f[9]), tuple(float(x) for x in f[10:17]), tuple(s for s in f[17].split(";") if s), mode,
                                 draw, gain, motion, tuple(b for b in (f[22] if len(f) > 22 else "").split(";") if b))


def parse_vp(f: Sequence[str]) -> Tuple[int, Dict[str, Tuple[float, ...]]]:
    """A ``VP`` record's fields: its recorded frame and each moving camera's pose there."""
    out = {}
    for it in f[2].split(";"):
        if it:
            parts = it.split(":")
            if len(parts) != 8:
                raise ViewsError(f"VP of frame {f[0]}: {it!r} is not camera:x:y:z:qw:qx:qy:qz")
            out[parts[0]] = tuple(float(x) for x in parts[1:])
    return int(f[0]), out


def has_views(frames) -> bool:
    return bool(frames is not None and frames.tag("VS"))


def read(frames, path: Optional[str] = None) -> Views:
    """The take's views from its frames log (``Jlog``); ``path``: the take, for its ``isaac/views.json`` and messages."""
    where = path or "the take"
    vs = frames.tag("VS") if frames is not None else []
    if not vs:
        raise ViewsError(f"{where}: no view cameras (no VS record): it was not recorded with views (docs/adr/0038 \"V, views\")")
    change, seed, sha, count = int(vs[-1][0]), int(vs[-1][1]), vs[-1][2], int(vs[-1][3])
    cams = []
    for r in frames.tag("VW"):
        c, cam = parse_vw(r)
        if c == change:
            cams.append(cam)
    names = [c.name for c in cams]
    if len(cams) != count or len(set(names)) != len(names):
        raise ViewsError(f"{where}: VS of change {change} names {count} view camera(s), its VW records give {names}")
    poses: Dict[int, Dict[str, Tuple[float, ...]]] = {}
    if any(c.mode == "moving" for c in cams):
        for r in frames.tag("VP"):
            n, p = parse_vp(r)
            poses.setdefault(n, {}).update(p)
    source = None
    if path and os.path.exists(os.path.join(path, "isaac", "views.json")):
        with open(os.path.join(path, "isaac", "views.json"), encoding="utf-8") as f:
            source = json.load(f).get("source")
    return Views(change, seed, sha, count, cams, poses, source)


def pose_at(views: Views, camera: ViewCamera, frame: int) -> Optional[Tuple[float, ...]]:
    """The camera's pose for recorded frame ``frame``: ``VW``'s for a static or fixed camera, the frame's ``VP`` for a moving
    one (None when the frame has none)."""
    if camera.mode != "moving":
        return camera.pose
    return views.poses.get(frame, {}).get(camera.name)


def scene_entry(views: Views, camera: ViewCamera, frames: Sequence[int]) -> Dict:
    """The camera's ``scenes.jsonl`` entry (``robot/lerobot/README.md``): ``CM``'s fields as ``sidecars.cameras`` gives them,
    then what ``VS`` and ``VW`` add, and for a moving camera its pose at each episode frame (``frames``: each episode frame's
    recorded frame)."""
    entry = {"name": camera.name, "recorded": camera.name, "width": camera.width, "height": camera.height, "fx": camera.fx,
             "fy": camera.fy, "cx": camera.cx, "cy": camera.cy, "near": camera.near, "far": camera.far, "pose": list(camera.pose),
             "mount": None, "poseFrame": "base", "streams": list(camera.streams), "mode": camera.mode,
             "specSha256": views.sha256, "specSource": views.source, "viewsChange": views.change, "viewsSeed": views.seed,
             "draw": dict(camera.draw), "gain": camera.gain, "motion": camera.motion, "broken": list(camera.broken),
             "framePoses": None}
    if camera.mode == "moving":
        poses = [pose_at(views, camera, n) for n in frames]
        missing = [i for i, p in enumerate(poses) if p is None]
        if missing:
            raise ViewsError(f"moving view {camera.name}: no VP pose for episode frame(s) {missing[:5]}"
                             + (f" and {len(missing) - 5} more" if len(missing) > 5 else ""))
        entry["framePoses"] = [list(p) for p in poses]
    return entry


__all__ = ["MODES", "ViewCamera", "Views", "ViewsError", "has_views", "parse_vp", "parse_vw", "pose_at", "read", "scene_entry"]
