"""View cameras at random viewpoints, for scene reconstruction (docs/adr/0038 "V, views").

Opt-in. A spec file given to the renderer as ``--views FILE`` (``robot/isaac/views/recon.json``), or the same object as an
Isaac-only ``views`` section of the scene file, asks for ``count`` extra cameras, ``view0``, ``view1``, .... Each is placed at
every change (once per take) by :func:`draw`, a pure function of the backdrop seed (``BK``'s), the change and the camera's
index:

- a look-at point in ``target`` (a box over the arm's reach);
- a place on a shell around it: ``azimuthDeg`` (from +x toward +y), ``elevationDeg`` above the horizon, ``distanceM``;
- a roll of up to ``rollDeg`` about the optical axis, and a horizontal field of view in its slot's ``hfovDeg``;
- static for the take, or with probability ``moving.share`` moving on a closed loop around that place (:func:`pose_at`).

A slot with ``fixed`` (``position``, ``lookAt``, ``hfovDeg``, optional ``rollDeg`` and ``jitter``) is not drawn: the camera stands
where the spec puts it, every take the same unless the jitter moves it a little, and is always static (mode ``fixed``). The
rules are still run on it, but only to warn: what it breaks is logged and recorded (``VW``'s ``broken``), and it is never
moved. A spec may mix fixed and drawn slots; a drawn view keeps its separation from the fixed ones before it.

A spec may declare a ``heldOut`` region of viewing directions (an azimuth and an elevation range) and its ``split``: a
``train`` spec never draws a view whose azimuth and elevation fall in it, a ``test`` spec draws only there (its own ranges lie
inside it), so test sessions are recorded from directions training never saw (docs/adr/0038 V, "Views in safety batches").
:func:`split_of` tells which side of the region a recorded view is on.

A candidate is drawn again until :func:`check` passes it: outside the robot's body and the arm's reach, not just above a
surface, inside the room, the workspace in frame, unoccluded and big enough, and far enough in direction from the views drawn
before it. A moving view must pass at every phase of its loop.

Camera k's resolution is ``slots[k % len(slots)]``'s for the whole run: a render product cannot change size. Poses are USD
cameras' (looking along -z, +y up) in the robot's base frame, as ``CM``'s are. Without a spec nothing here runs, and the
renderer records exactly as before. Pure Python: the tests and the checks import it without Isaac.
"""

from __future__ import annotations

import hashlib
import json
import math
import random
from dataclasses import dataclass, field, replace
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

SECTION = "views"
FORMAT = "teleop-views"
MOVING_PHASES = 48  # where a moving view's loop is checked against the rules
FALLBACK = -1  # VW attempts: no candidate passed every rule, the one that broke the fewest was kept
STREAMS = ("rgb", "depth", "mask")
SIGHT_MARGIN_M = 0.5

Vec = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


class ViewsError(ValueError):
    pass


# --- the spec ----------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Fixed:
    """A camera the spec places itself (docs/adr/0038 "V, views", fixed views): never moved by the rules, which only warn."""
    position: Vec
    look_at: Vec
    hfov: float
    roll: float
    jitter_m: float = 0.0  # each take: the position moved by up to this along each axis (default none)
    jitter_deg: float = 0.0  # and the line of sight turned by up to this in azimuth and elevation, and the roll


@dataclass(frozen=True)
class Slot:
    width: int
    height: int
    hfov: Tuple[float, float]
    fixed: Optional[Fixed] = None


@dataclass(frozen=True)
class Moving:
    share: float
    period: Tuple[float, float]
    azimuth_amp: Tuple[float, float]
    elevation_amp: Tuple[float, float]
    distance_amp: Tuple[float, float]
    target_amp: Tuple[float, float]


@dataclass(frozen=True)
class Box:
    id: str
    lo: Vec
    hi: Vec
    margin: float = 0.0

    def contains(self, p: Sequence[float], grow: float = 0.0) -> bool:
        return all(self.lo[i] - grow <= p[i] <= self.hi[i] + grow for i in range(3))


@dataclass(frozen=True)
class Sphere:
    id: str
    center: Vec
    radius: float


@dataclass(frozen=True)
class Region:
    """Viewing directions held out of training: a view's drawn azimuth (from +x toward +y, wrapping at 360) and elevation."""
    azimuth: Tuple[float, float]
    elevation: Tuple[float, float]

    def contains(self, azimuth: float, elevation: float) -> bool:
        a0, a1 = self.azimuth
        return (azimuth - a0) % 360.0 <= a1 - a0 and self.elevation[0] <= elevation <= self.elevation[1]


@dataclass(frozen=True)
class Spec:
    count: int
    slots: Tuple[Slot, ...]
    target_lo: Vec
    target_hi: Vec
    azimuth: Tuple[float, float]
    elevation: Tuple[float, float]
    distance: Tuple[float, float]
    roll: float
    near: float
    far: float
    record: Tuple[str, ...]
    record_quality: int
    moving: Optional[Moving]
    boxes: Tuple[Box, ...]
    spheres: Tuple[Sphere, ...]
    surface_margin: float
    above_surfaces: float
    floor_clearance: float
    ceiling: float
    room_radius: float
    work_lo: Tuple[float, float]
    work_hi: Tuple[float, float]
    work_grid: int
    work_height: float
    work_extra: Tuple[Vec, ...]
    in_frame: float
    visible: float
    cover: float
    border: float
    separation: float
    max_attempts: int
    doc: Mapping = field(default_factory=dict, compare=False)  # as read, for views.json and VS's sha256
    held_out: Optional[Region] = None  # the viewing directions a train spec never draws and a test spec only draws
    split: str = ""  # "train" or "test" with a held-out region, else ""

    def slot(self, k: int) -> Slot:
        return self.slots[k % len(self.slots)]

    def with_count(self, n: int) -> "Spec":
        if not 0 <= n <= MAX_COUNT:
            raise ViewsError(f"views: count {n} is outside [0, {MAX_COUNT}]")
        doc = dict(self.doc)
        doc["count"] = n
        return parse(doc)

    @property
    def sha256(self) -> str:
        return hashlib.sha256(canonical(self.doc)).hexdigest()


MAX_COUNT = 8
TOP_FIELDS = {"format", "version", "count", "slots", "target", "azimuthDeg", "elevationDeg", "distanceM", "rollDeg", "near",
              "far", "record", "recordQuality", "moving", "keepOut", "room", "workspace", "accept", "notes", "heldOut", "split"}
SPLITS = ("train", "test")
HELD_OUT_MAX_SHARE = 0.9  # a train spec may hold out at most this share of its azimuth-elevation box (its draws must end)
REDRAW_MAX = 10000  # a train candidate's azimuth and elevation drawn again at most this often to fall outside the region


def canonical(doc: Mapping) -> bytes:
    return json.dumps(doc, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _num(v, where: str, lo: float, hi: float) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
        raise ViewsError(f"views: {where} must be a number")
    if not lo <= float(v) <= hi:
        raise ViewsError(f"views: {where} = {v} is outside [{lo}, {hi}]")
    return float(v)


def _int(v, where: str, lo: int, hi: int) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ViewsError(f"views: {where} must be an integer")
    if not lo <= v <= hi:
        raise ViewsError(f"views: {where} = {v} is outside [{lo}, {hi}]")
    return v


def _range(v, where: str, lo: float, hi: float) -> Tuple[float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise ViewsError(f"views: {where} must be [lo, hi]")
    a, b = _num(v[0], where, lo, hi), _num(v[1], where, lo, hi)
    if a > b:
        raise ViewsError(f"views: {where} has lo {a} above hi {b}")
    return a, b


def _vec(v, where: str, n: int = 3, bound: float = 10.0) -> Tuple[float, ...]:
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ViewsError(f"views: {where} must be {n} numbers")
    return tuple(_num(x, where, -bound, bound) for x in v)


def _obj(v, where: str, fields: Sequence[str], required: Sequence[str] = ()) -> Mapping:
    if not isinstance(v, Mapping):
        raise ViewsError(f"views: {where} must be an object")
    unknown = sorted(set(v) - set(fields))
    if unknown:
        raise ViewsError(f"views: {where} has unknown field(s) {unknown}")
    missing = [f for f in required if f not in v]
    if missing:
        raise ViewsError(f"views: {where} lacks {missing}")
    return v


def parse(doc) -> Spec:
    """A views spec, checked: every field known, every value within fixed bounds."""
    d = _obj(doc, "the spec", sorted(TOP_FIELDS), ("format", "version", "count", "slots", "target"))
    if d["format"] != FORMAT or d["version"] != 1:
        raise ViewsError(f"views: format must be {FORMAT!r} version 1")
    count = _int(d["count"], "count", 0, MAX_COUNT)
    if not isinstance(d["slots"], list) or not d["slots"]:
        raise ViewsError("views: slots must be a non-empty list")
    slots = []
    for i, s in enumerate(d["slots"]):
        s = _obj(s, f"slots[{i}]", ("width", "height", "hfovDeg", "fixed"),
                 ("width", "height") if "fixed" in s else ("width", "height", "hfovDeg"))
        fixed = None
        if "fixed" in s:
            w = f"slots[{i}].fixed"
            f = _obj(s["fixed"], w, ("position", "lookAt", "hfovDeg", "rollDeg", "jitter"), ("position", "lookAt", "hfovDeg"))
            pos, at = _vec(f["position"], w + ".position"), _vec(f["lookAt"], w + ".lookAt")
            if math.dist(pos, at) < 0.05:
                raise ViewsError(f"views: {w} looks at a point less than 5 cm from where it stands")
            j = _obj(f.get("jitter", {}), w + ".jitter", ("translationM", "rotationDeg"))
            fixed = Fixed(pos, at, _num(f["hfovDeg"], w + ".hfovDeg", 20.0, 120.0), _num(f.get("rollDeg", 0.0), w + ".rollDeg", -180.0, 180.0),
                          _num(j.get("translationM", 0.0), w + ".jitter.translationM", 0.0, 0.05),
                          _num(j.get("rotationDeg", 0.0), w + ".jitter.rotationDeg", 0.0, 5.0))
        hfov = _range(s["hfovDeg"], f"slots[{i}].hfovDeg", 20.0, 120.0) if "hfovDeg" in s else (fixed.hfov, fixed.hfov)
        slots.append(Slot(_int(s["width"], f"slots[{i}].width", 16, 4096), _int(s["height"], f"slots[{i}].height", 16, 4096),
                          hfov, fixed))
    if any(sl.fixed is not None for sl in slots) and count > len(slots):
        raise ViewsError(f"views: count {count} is more than the {len(slots)} slots, and a fixed slot cannot repeat")
    t = _obj(d["target"], "target", ("min", "max"), ("min", "max"))
    t_lo, t_hi = _vec(t["min"], "target.min"), _vec(t["max"], "target.max")
    if any(a > b for a, b in zip(t_lo, t_hi)):
        raise ViewsError("views: target.min is above target.max")
    record = d.get("record", list(STREAMS))
    if not isinstance(record, list) or not record or any(r not in STREAMS for r in record) or len(set(record)) != len(record):
        raise ViewsError(f"views: record must be a list of {STREAMS}")
    moving = None
    if "moving" in d:
        m = _obj(d["moving"], "moving", ("share", "periodS", "azimuthAmpDeg", "elevationAmpDeg", "distanceAmpM", "targetAmpM"),
                 ("share", "periodS"))
        moving = Moving(_num(m["share"], "moving.share", 0.0, 1.0), _range(m["periodS"], "moving.periodS", 2.0, 120.0),
                        _range(m.get("azimuthAmpDeg", [0, 0]), "moving.azimuthAmpDeg", 0.0, 90.0),
                        _range(m.get("elevationAmpDeg", [0, 0]), "moving.elevationAmpDeg", 0.0, 30.0),
                        _range(m.get("distanceAmpM", [0, 0]), "moving.distanceAmpM", 0.0, 1.0),
                        _range(m.get("targetAmpM", [0, 0]), "moving.targetAmpM", 0.0, 0.3))
    k = _obj(d.get("keepOut", {}), "keepOut", ("boxes", "spheres", "surfaceMarginM", "aboveSurfacesM"))
    boxes = []
    for i, b in enumerate(k.get("boxes", [])):
        b = _obj(b, f"keepOut.boxes[{i}]", ("id", "min", "max", "marginM"), ("id", "min", "max"))
        lo, hi = _vec(b["min"], f"keepOut.boxes[{i}].min"), _vec(b["max"], f"keepOut.boxes[{i}].max")
        if any(a > c for a, c in zip(lo, hi)):
            raise ViewsError(f"views: keepOut.boxes[{i}] has min above max")
        boxes.append(Box(str(b["id"]), lo, hi, _num(b.get("marginM", 0.0), f"keepOut.boxes[{i}].marginM", 0.0, 1.0)))
    spheres = []
    for i, s in enumerate(k.get("spheres", [])):
        s = _obj(s, f"keepOut.spheres[{i}]", ("id", "center", "radius"), ("id", "center", "radius"))
        spheres.append(Sphere(str(s["id"]), _vec(s["center"], f"keepOut.spheres[{i}].center"),
                              _num(s["radius"], f"keepOut.spheres[{i}].radius", 0.0, 3.0)))
    r = _obj(d.get("room", {}), "room", ("floorClearanceM", "ceilingM", "radiusM"))
    w = _obj(d.get("workspace", {}), "workspace", ("min", "max", "grid", "heightM", "extra"), ("min", "max"))
    w_lo, w_hi = _vec(w["min"], "workspace.min", 2), _vec(w["max"], "workspace.max", 2)
    if any(a > b for a, b in zip(w_lo, w_hi)):
        raise ViewsError("views: workspace.min is above workspace.max")
    extra = tuple(_vec(p, f"workspace.extra[{i}]") for i, p in enumerate(w.get("extra", [])))
    a = _obj(d.get("accept", {}), "accept", ("inFrame", "visible", "cover", "border", "separationDeg", "maxAttempts"))
    distance = _range(d.get("distanceM", [0.6, 2.0]), "distanceM", 0.2, 5.0)
    near, far = _num(d.get("near", 0.05), "near", 0.001, 1.0), _num(d.get("far", 6.0), "far", 1.0, 100.0)
    if far <= distance[1] + 1.0:
        raise ViewsError("views: far must reach past the farthest view distance by more than 1 m")
    azimuth = _range(d.get("azimuthDeg", [0, 360]), "azimuthDeg", -360.0, 720.0)
    elevation = _range(d.get("elevationDeg", [8, 80]), "elevationDeg", -10.0, 89.0)
    held_out, split = _held_out(d, azimuth, elevation, slots, moving)
    return Spec(
        count=count, slots=tuple(slots), target_lo=t_lo, target_hi=t_hi,
        azimuth=azimuth, elevation=elevation,
        distance=distance, roll=_num(d.get("rollDeg", 0.0), "rollDeg", 0.0, 30.0), near=near, far=far,
        record=tuple(record), record_quality=_int(d.get("recordQuality", 92), "recordQuality", 1, 100), moving=moving,
        boxes=tuple(boxes), spheres=tuple(spheres),
        surface_margin=_num(k.get("surfaceMarginM", 0.1), "keepOut.surfaceMarginM", 0.0, 1.0),
        above_surfaces=_num(k.get("aboveSurfacesM", 0.75), "keepOut.aboveSurfacesM", 0.0, 3.0),
        floor_clearance=_num(r.get("floorClearanceM", 0.3), "room.floorClearanceM", 0.0, 3.0),
        ceiling=_num(r.get("ceilingM", 2.6), "room.ceilingM", 1.0, 10.0),
        room_radius=_num(r.get("radiusM", 2.2), "room.radiusM", 0.3, 10.0),
        work_lo=w_lo, work_hi=w_hi, work_grid=_int(w.get("grid", 5), "workspace.grid", 2, 20),
        work_height=_num(w.get("heightM", 0.005), "workspace.heightM", 0.0, 1.0), work_extra=extra,
        in_frame=_num(a.get("inFrame", 0.8), "accept.inFrame", 0.0, 1.0),
        visible=_num(a.get("visible", 0.7), "accept.visible", 0.0, 1.0),
        cover=_num(a.get("cover", 0.12), "accept.cover", 0.0, 1.0),
        border=_num(a.get("border", 0.02), "accept.border", 0.0, 0.4),
        separation=_num(a.get("separationDeg", 15.0), "accept.separationDeg", 0.0, 90.0),
        max_attempts=_int(a.get("maxAttempts", 200), "accept.maxAttempts", 1, 10000),
        doc=json.loads(json.dumps(d)), held_out=held_out, split=split)


def _held_out(d: Mapping, azimuth: Tuple[float, float], elevation: Tuple[float, float], slots, moving) -> Tuple[Optional[Region], str]:
    """The spec's held-out region and split, checked: both or neither; a test spec's ranges inside the region, a train spec's
    not mostly in it; every view drawn (no fixed slot) and static (a moving loop could cross the region's edge)."""
    if ("heldOut" in d) != ("split" in d):
        raise ViewsError("views: heldOut and split come together")
    if "heldOut" not in d:
        return None, ""
    if d["split"] not in SPLITS:
        raise ViewsError(f"views: split must be one of {SPLITS}")
    h = _obj(d["heldOut"], "heldOut", ("azimuthDeg", "elevationDeg"), ("azimuthDeg", "elevationDeg"))
    region = Region(_range(h["azimuthDeg"], "heldOut.azimuthDeg", -360.0, 720.0),
                    _range(h["elevationDeg"], "heldOut.elevationDeg", -10.0, 89.0))
    if not 0.0 < region.azimuth[1] - region.azimuth[0] <= 360.0:
        raise ViewsError("views: heldOut.azimuthDeg must span more than 0 and at most 360 degrees")
    if any(sl.fixed is not None for sl in slots):
        raise ViewsError("views: a spec with heldOut draws every view: no fixed slots")
    if moving is not None and moving.share > 0:
        raise ViewsError("views: a spec with heldOut has static views only (moving.share 0)")
    if d["split"] == "test":
        inside_az = (azimuth[0] - region.azimuth[0]) % 360.0 + (azimuth[1] - azimuth[0]) <= region.azimuth[1] - region.azimuth[0]
        if not inside_az or not region.elevation[0] <= elevation[0] <= elevation[1] <= region.elevation[1]:
            raise ViewsError("views: a test spec's azimuthDeg and elevationDeg must lie inside heldOut")
    elif held_out_share(region, azimuth, elevation) > HELD_OUT_MAX_SHARE:
        raise ViewsError(f"views: heldOut covers more than {HELD_OUT_MAX_SHARE:.0%} of the train spec's directions")
    return region, d["split"]


def held_out_share(region: Region, azimuth: Tuple[float, float], elevation: Tuple[float, float], steps: int = 720) -> float:
    """The share of a uniform azimuth-elevation draw over these ranges that falls in the region (a midpoint grid)."""
    az = sum(1 for i in range(steps) if region.contains(azimuth[0] + (azimuth[1] - azimuth[0]) * (i + 0.5) / steps, region.elevation[0]))
    el = sum(1 for i in range(steps) if region.elevation[0] <= elevation[0] + (elevation[1] - elevation[0]) * (i + 0.5) / steps <= region.elevation[1])
    return az / steps * el / steps


def split_of(doc: Mapping, azimuth: float, elevation: float) -> str:
    """Which side of its spec's held-out region a recorded view is on, for a dataset reader: ``doc`` is the spec as a take's
    ``isaac/views.json`` holds it (its ``spec``), the angles are the view's ``VW`` draw ``azimuthDeg`` and ``elevationDeg``.
    "test" inside the region, "train" outside it, "" for a spec without one."""
    h = doc.get("heldOut")
    if not h:
        return ""
    return "test" if Region(tuple(h["azimuthDeg"]), tuple(h["elevationDeg"])).contains(azimuth, elevation) else "train"


def load(path: str = "", scene=None, count: int = -1) -> Optional[Spec]:
    """The views in force: the file at ``path`` if given, else the scene's ``views`` section, else None (no views: the renderer
    records exactly as before). ``count`` >= 0 replaces the spec's count; a count of 0 is no views."""
    doc = None
    if path:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    elif scene is not None:
        doc = (getattr(scene, "extra", None) or {}).get(SECTION)
    if doc is None:
        return None
    s = parse(doc)
    if count >= 0:
        s = s.with_count(count)
    return s if s.count > 0 else None


# --- geometry ----------------------------------------------------------------------------------------------------------

def q_mul(a: Sequence[float], b: Sequence[float]) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def q_matrix(q: Sequence[float]):
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def matrix_quat(m) -> Quat:
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
    q = tuple(c / n for c in q)
    return q if q[0] >= 0 else tuple(-c for c in q)


def look_at(position: Sequence[float], target: Sequence[float], roll_deg: float = 0.0) -> Quat:
    """A USD camera at ``position`` looking at ``target`` with +z up (as ``renderer.look_at``), then turned ``roll_deg``
    about its own optical axis (counter-clockwise as the camera sees it)."""
    f = [t - p for t, p in zip(target, position)]
    n = math.sqrt(sum(c * c for c in f))
    z = [-c / n for c in f]
    up = (0.0, 0.0, 1.0)
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(c * c for c in x))
    if n < 1e-9:  # looking straight down or up: any heading
        x = [1.0, 0.0, 0.0]
    else:
        x = [c / n for c in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    q = matrix_quat(((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2])))
    if roll_deg:
        h = math.radians(roll_deg) / 2
        q = q_mul(q, (math.cos(h), 0.0, 0.0, math.sin(h)))
        q = q if q[0] >= 0 else tuple(-c for c in q)
    return q


def intrinsics(width: int, height: int, hfov_deg: float) -> Dict[str, float]:
    """A pinhole with square pixels, its principal point at the centre and pixel centres at +0.5 (as the renderer's)."""
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return {"fx": fx, "fy": fx, "cx": width / 2.0, "cy": height / 2.0}


def place(target: Sequence[float], azimuth_deg: float, elevation_deg: float, distance: float) -> Vec:
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    return (target[0] + distance * math.cos(e) * math.cos(a), target[1] + distance * math.cos(e) * math.sin(a),
            target[2] + distance * math.sin(e))


def project(p: Sequence[float], position: Sequence[float], q: Sequence[float], intr: Mapping[str, float]):
    """(u, v, depth along the optical axis) of world point ``p``, or None behind the camera."""
    m = q_matrix(q)
    d = [p[i] - position[i] for i in range(3)]
    xc = [sum(m[r][c] * d[r] for r in range(3)) for c in range(3)]  # R^T d
    if xc[2] >= -1e-9:
        return None
    return intr["cx"] + intr["fx"] * xc[0] / -xc[2], intr["cy"] - intr["fy"] * xc[1] / -xc[2], -xc[2]


def segment_hits_box(a: Sequence[float], b: Sequence[float], box: Box) -> bool:
    """Whether the segment a-b passes through ``box`` (slab test)."""
    t0, t1 = 0.0, 1.0
    for i in range(3):
        d = b[i] - a[i]
        if abs(d) < 1e-12:
            if a[i] < box.lo[i] or a[i] > box.hi[i]:
                return False
            continue
        u, v = (box.lo[i] - a[i]) / d, (box.hi[i] - a[i]) / d
        if u > v:
            u, v = v, u
        t0, t1 = max(t0, u), min(t1, v)
        if t0 > t1:
            return False
    return True


@dataclass(frozen=True)
class Geometry:
    """What the rules test against: the scene's surfaces (table, bench) as slabs, the spec's keep-outs, the floor, and the
    workspace's points."""
    surfaces: Tuple[Box, ...]
    floor_z: float
    points: Tuple[Vec, ...]


def geometry(spec: Spec, scene) -> Geometry:
    surfaces = []
    for s in (scene.table, scene.bench):
        if s is None:
            continue
        lo = tuple(c - h / 2 for c, h in zip(s.center, s.size))
        hi = tuple(c + h / 2 for c, h in zip(s.center, s.size))
        surfaces.append(Box(s.id, lo, hi))
    floor = [s for s in scene.surfaces if s.id == "floor"]
    floor_z = floor[0].top if floor else scene.table.top - 0.85
    top = scene.table.top + spec.work_height
    g = spec.work_grid
    pts = [(spec.work_lo[0] + (spec.work_hi[0] - spec.work_lo[0]) * i / (g - 1),
            spec.work_lo[1] + (spec.work_hi[1] - spec.work_lo[1]) * j / (g - 1), top) for i in range(g) for j in range(g)]
    pts.extend(spec.work_extra)
    return Geometry(tuple(surfaces), floor_z, tuple(pts))


RULES = ("body", "arm", "surface", "floor", "ceiling", "walls", "in_frame", "visible", "cover", "separation")


def check(spec: Spec, geo: Geometry, position: Sequence[float], q: Sequence[float], width: int, height: int,
          hfov_deg: float, target: Sequence[float], others: Sequence[Vec] = ()) -> List[str]:
    """The rules ``position`` breaks (empty: it passes). ``others``: earlier views' lines of sight in this take."""
    broken = []
    if any(b.contains(position, b.margin) for b in spec.boxes):
        broken.append("body")
    if any(math.dist(position, s.center) < s.radius for s in spec.spheres):
        broken.append("arm")
    m = spec.surface_margin
    if any(s.lo[0] - m <= position[0] <= s.hi[0] + m and s.lo[1] - m <= position[1] <= s.hi[1] + m
           and position[2] < s.hi[2] + spec.above_surfaces for s in geo.surfaces):
        broken.append("surface")
    if position[2] < geo.floor_z + spec.floor_clearance:
        broken.append("floor")
    if position[2] > geo.floor_z + spec.ceiling:
        broken.append("ceiling")
    if math.hypot(position[0] - target[0], position[1] - target[1]) > spec.room_radius:
        broken.append("walls")
    intr = intrinsics(width, height, hfov_deg)
    bu, bv = spec.border * width, spec.border * height
    inside, seen, us, vs = 0, 0, [], []
    occluders = tuple(Box(b.id, b.lo, b.hi) for b in spec.boxes) + geo.surfaces
    for p in geo.points:
        pr = project(p, position, q, intr)
        if pr is not None and spec.near <= pr[2] <= spec.far and bu <= pr[0] <= width - bu and bv <= pr[1] <= height - bv:
            inside += 1
            us.append(pr[0])
            vs.append(pr[1])
        d = [p[i] - position[i] for i in range(3)]
        n = math.sqrt(sum(c * c for c in d))
        end = tuple(position[i] + d[i] * max(0.0, (n - 0.01)) / n for i in range(3))
        if not any(segment_hits_box(position, end, b) for b in occluders):
            seen += 1
    n = len(geo.points)
    if inside < spec.in_frame * n:
        broken.append("in_frame")
    if seen < spec.visible * n:
        broken.append("visible")
    area = (max(us) - min(us)) * (max(vs) - min(vs)) if us else 0.0
    if area < spec.cover * width * height:
        broken.append("cover")
    if others:
        sight = unit([t - p for t, p in zip(target, position)])
        if any(math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(sight, o)))))) < spec.separation
               for o in others):
            broken.append("separation")
    return broken


def unit(v: Sequence[float]) -> Vec:
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


# --- the draws ---------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Motion:
    period: float
    azimuth_amp: float
    elevation_amp: float
    distance_amp: float
    target_amp: float
    phases: Tuple[float, float, float, float]


@dataclass(frozen=True)
class View:
    """One view camera for one change: where it looks from, as drawn (rounded, so the records hold exactly what was used)."""
    index: int
    width: int
    height: int
    hfov: float
    target: Vec
    azimuth: float
    elevation: float
    distance: float
    roll: float
    motion: Optional[Motion]
    attempts: int  # candidates drawn until one passed; FALLBACK when none did; 0 for a fixed view
    position: Optional[Vec] = None  # a fixed view's place, exactly as the spec gives it (moved by its jitter, if any)
    broken: Tuple[str, ...] = ()  # the rules a fixed view breaks: warned and recorded, never acted on

    @property
    def id(self) -> str:
        return f"view{self.index}"

    @property
    def mode(self) -> str:
        if self.position is not None:
            return "fixed"
        return "moving" if self.motion is not None else "static"

    @property
    def intrinsics(self) -> Dict[str, float]:
        return intrinsics(self.width, self.height, self.hfov)

    def params_at(self, t: float) -> Tuple[Vec, float, float, float]:
        """(look-at point, azimuth, elevation, distance) at ``t`` seconds into its loop; the drawn ones when static."""
        m = self.motion
        if m is None:
            return self.target, self.azimuth, self.elevation, self.distance
        w = 2 * math.pi / m.period
        p1, p2, p3, p4 = m.phases
        tgt = (self.target[0] + m.target_amp * math.cos(w * t + p4), self.target[1] + m.target_amp * math.sin(w * t + p4),
               self.target[2])
        return (tgt, self.azimuth + m.azimuth_amp * math.sin(w * t + p1), self.elevation + m.elevation_amp * math.sin(w * t + p2),
                self.distance + m.distance_amp * math.sin(2 * w * t + p3))

    def pose_at(self, t: float = 0.0) -> Tuple[Vec, Quat]:
        if self.position is not None:
            return self.centre_pose()
        tgt, az, el, dist = self.params_at(t)
        p = place(tgt, az, el, dist)
        return p, look_at(p, tgt, self.roll)

    def centre_pose(self) -> Tuple[Vec, Quat]:
        p = self.position if self.position is not None else place(self.target, self.azimuth, self.elevation, self.distance)
        return p, look_at(p, self.target, self.roll)

    def sight(self) -> Vec:
        p = self.position if self.position is not None else place(self.target, self.azimuth, self.elevation, self.distance)
        return unit([t - c for t, c in zip(self.target, p)])


def pose_at_frame(view: View, frames_since_change: int, fps: float) -> Tuple[Vec, Quat]:
    """A view's pose at a render frame: a moving view's loop runs on frames since the change, at the renderer's fps."""
    return view.pose_at(frames_since_change / float(fps))


def _u(r: random.Random, lo_hi: Tuple[float, float]) -> float:
    return r.uniform(*lo_hi)


def _candidate(spec: Spec, k: int, r: random.Random, attempts: int, moving: bool) -> View:
    slot = spec.slot(k)
    target = tuple(round(r.uniform(a, b), 5) for a, b in zip(spec.target_lo, spec.target_hi))
    az = round(_u(r, spec.azimuth) % 360.0, 4)
    el = round(_u(r, spec.elevation), 4)
    if spec.split == "train":  # never a held-out direction, not even in a fallback: drawn again until outside the region
        for _ in range(REDRAW_MAX):
            if not spec.held_out.contains(az, el):
                break
            az = round(_u(r, spec.azimuth) % 360.0, 4)
            el = round(_u(r, spec.elevation), 4)
        else:
            raise ViewsError("views: no direction outside heldOut drawn")
    dist = round(_u(r, spec.distance), 5)
    roll = round(r.uniform(-spec.roll, spec.roll), 4)
    hfov = round(_u(r, slot.hfov), 4)
    motion = None
    if moving:
        m = spec.moving
        motion = Motion(round(_u(r, m.period), 4), round(_u(r, m.azimuth_amp), 4), round(_u(r, m.elevation_amp), 4),
                        round(_u(r, m.distance_amp), 5), round(_u(r, m.target_amp), 5),
                        tuple(round(r.uniform(0, 2 * math.pi), 6) for _ in range(4)))
    return View(k, slot.width, slot.height, hfov, target, az, el, dist, roll, motion, attempts)


def broken_rules(spec: Spec, geo: Geometry, v: View, others: Sequence[Vec] = ()) -> List[str]:
    """Every rule the view breaks: at its drawn place (with ``others``' separation), and for a moving view at every phase."""
    p, q = v.centre_pose()
    broken = check(spec, geo, p, q, v.width, v.height, v.hfov, v.target, others)
    if v.motion is not None:
        for i in range(MOVING_PHASES):
            t = v.motion.period * i / MOVING_PHASES
            tgt, _, _, _ = v.params_at(t)
            p, q = v.pose_at(t)
            for b in check(spec, geo, p, q, v.width, v.height, v.hfov, tgt):
                if b not in broken:
                    broken.append(b)
    return broken


def fixed_view(spec: Spec, k: int, seed: int, change: int) -> View:
    """Slot k's fixed camera for one change: where the spec puts it, moved by its optional jitter (from its own key). Its
    azimuth, elevation and distance from its look-at point are derived, for the record."""
    f = spec.slot(k).fixed
    pos, at, roll = f.position, f.look_at, f.roll
    if f.jitter_m or f.jitter_deg:
        r = random.Random(f"teleop-view-fixed:{seed}:{change}:{k}")
        pos = tuple(round(c + r.uniform(-f.jitter_m, f.jitter_m), 6) for c in pos)
        d = [a - p for a, p in zip(f.look_at, f.position)]
        n = math.sqrt(sum(c * c for c in d))
        az = math.atan2(d[1], d[0]) + math.radians(r.uniform(-f.jitter_deg, f.jitter_deg))
        el = math.asin(max(-1.0, min(1.0, d[2] / n))) + math.radians(r.uniform(-f.jitter_deg, f.jitter_deg))
        at = tuple(round(p + n * c, 6) for p, c in zip(pos, (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))))
        roll = round(roll + r.uniform(-f.jitter_deg, f.jitter_deg), 4)
    d = [p - a for p, a in zip(pos, at)]
    dist = math.sqrt(sum(c * c for c in d))
    slot = spec.slot(k)
    return View(k, slot.width, slot.height, f.hfov, tuple(at), round(math.degrees(math.atan2(d[1], d[0])) % 360.0, 6),
                round(math.degrees(math.asin(max(-1.0, min(1.0, d[2] / dist)))), 6), round(dist, 6), roll, None, 0,
                position=tuple(pos))


def draw(spec: Spec, geo: Geometry, seed: int, change: int, rejected: Optional[List[List[str]]] = None) -> List[View]:
    """Every view camera for one change, in index order, each from its own ``random.Random`` key: a pure function of the
    seed, the change and the camera's index (and the views before it, for the separation rule). Whether a view moves is
    drawn first, from its own key, so the rules (which a moving loop breaks more often) do not change the share.
    ``rejected``, if given, gets the rules each rejected candidate broke (for the offline statistics, views_stats.py)."""
    out: List[View] = []
    for k in range(spec.count):
        if spec.slot(k).fixed is not None:  # placed by the spec: the rules only warn (VW's broken), it is never moved
            v = fixed_view(spec, k, seed, change)
            out.append(replace(v, broken=tuple(broken_rules(spec, geo, v, [w.sight() for w in out]))))
            continue
        moving = spec.moving is not None and random.Random(f"teleop-view-mode:{seed}:{change}:{k}").random() < spec.moving.share
        r = random.Random(f"teleop-view:{seed}:{change}:{k}")
        others = [v.sight() for v in out]
        best, best_broken = None, None
        chosen = None
        for attempt in range(1, spec.max_attempts + 1):
            v = _candidate(spec, k, r, attempt, moving)
            broken = broken_rules(spec, geo, v, others)
            if not broken:
                chosen = v
                break
            if rejected is not None:
                rejected.append(broken)
            if best is None or len(broken) < len(best_broken):
                best, best_broken = v, broken
        if chosen is None:
            chosen = replace(best, attempts=FALLBACK)
        out.append(chosen)
    return out


# --- records (trainer/contract/SESSION_FORMAT.md) ----------------------------------------------------------------------

def sight(spec: Spec) -> Tuple[Vec, float]:
    """(a place, how far from it a view may see), for the renderer's out_of_sight: the look-at box's centre and the far
    plane plus the farthest a view stands from its look-at point (with a moving loop and the box's own size) and a margin."""
    centre = tuple((a + b) / 2 for a, b in zip(spec.target_lo, spec.target_hi))
    half = math.dist(spec.target_lo, spec.target_hi) / 2
    extra = spec.moving.distance_amp[1] + spec.moving.target_amp[1] if spec.moving is not None else 0.0
    reach = spec.distance[1] + extra + half
    for s in spec.slots:
        if s.fixed is not None:
            reach = max(reach, math.dist(centre, s.fixed.position) + math.sqrt(3) * s.fixed.jitter_m)
    return centre, spec.far + reach + SIGHT_MARGIN_M


def vs_fields(spec: Spec, seed: int, change: int) -> tuple:
    return (change, seed, spec.sha256, spec.count)


def vw_fields(spec: Spec, v: View, change: int, gain: float = 1.0) -> tuple:
    p, q = v.centre_pose()
    i = v.intrinsics
    m = v.motion
    motion = [[m.period, m.azimuth_amp, m.elevation_amp, m.distance_amp, m.target_amp, *m.phases]] if m is not None else ""
    return (change, v.id, v.width, v.height, round(i["fx"], 6), round(i["fy"], 6), i["cx"], i["cy"], spec.near, spec.far,
            *[round(c, 7) for c in (*p, *q)], [[s] for s in spec.record], v.mode,
            [[v.hfov, v.azimuth, v.elevation, v.distance, v.roll, *v.target, v.attempts]], round(gain, 3), motion,
            [[b] for b in v.broken])


def vp_items(poses: Sequence[Tuple[str, Tuple[Vec, Quat]]]) -> List[list]:
    return [[cid, *[round(c, 7) for c in (*p, *q)]] for cid, (p, q) in poses]


def views_json(spec: Spec, source: str) -> bytes:
    doc = {"format": "teleop-views-used", "version": 1, "source": source, "sha256": spec.sha256, "spec": spec.doc}
    return (json.dumps(doc, indent=1, sort_keys=True) + "\n").encode("utf-8")


def parse_vw(fields: Sequence[str]) -> Dict:
    """A VW record's fields (after the tag) as a dict, for the checks."""
    f = list(fields)
    draw_ = [float(x) for x in f[19].split(":")]
    motion = [float(x) for x in f[21].split(":")] if len(f) > 21 and f[21] else None
    return {"change": int(f[0]), "camera": f[1], "width": int(f[2]), "height": int(f[3]), "fx": float(f[4]),
            "fy": float(f[5]), "cx": float(f[6]), "cy": float(f[7]), "near": float(f[8]), "far": float(f[9]),
            "pose": tuple(float(x) for x in f[10:17]), "streams": f[17].split(";"), "mode": f[18],
            "hfovDeg": draw_[0], "azimuthDeg": draw_[1], "elevationDeg": draw_[2], "distanceM": draw_[3], "rollDeg": draw_[4],
            "target": tuple(draw_[5:8]), "attempts": int(draw_[8]), "gain": float(f[20]), "motion": motion,
            "broken": [b for b in f[22].split(";") if b] if len(f) > 22 else []}
