"""The scene file (``core/Scenes/*.json``, docs/adr/0031 amendment 2), as Isaac reads it.

The same rules as Core's reader (``Teleop.RobotArm/World/WorldScene.cs``), in the same order, with the same keyword at the
start of every error, so the two agree on every case in ``core/Scenes/testdata/vectors.json``. The canonical parse of the
shipped scene is pinned by ``golden/scene-fr3-duo-bench.json``, which Core writes. Isaac also reads the sections Core
skips (``backdrops``, ``cameras``, ``lights``, docs/adr/0032); they are kept as given in :attr:`Scene.extra`.

Layouts are RobotHost's to draw; this module has no copy of the draw.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

FORMAT = "teleop-scene"
VERSION = 2  # the newest version this reader knows; it reads every version from 1. Version 2 adds movable objects.
MAX_OBJECTS = 24
MAX_MASS = 20.0  # kg
TOLERANCE = 0.001
_NAME = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


class SceneError(ValueError):
    """The file is not a valid scene; the message starts with what is wrong (``"table: ..."``)."""


@dataclass(frozen=True)
class Surface:
    id: str
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]
    colour: Tuple[float, float, float]
    static_friction: float
    dynamic_friction: float
    collides: bool
    headset: str

    @property
    def x_min(self) -> float:
        return self.center[0] - self.size[0] / 2

    @property
    def x_max(self) -> float:
        return self.center[0] + self.size[0] / 2

    @property
    def y_min(self) -> float:
        return self.center[1] - self.size[1] / 2

    @property
    def y_max(self) -> float:
        return self.center[1] + self.size[1] / 2

    @property
    def top(self) -> float:
        return self.center[2] + self.size[2] / 2


@dataclass(frozen=True)
class Object:
    id: int
    name: str
    shape: str
    size: Tuple[float, float, float]
    position: Tuple[float, float, float]
    yaw: float
    movable: bool  # a body physics moves (version 2): it falls, and an arm can push or knock it over
    mass: float  # kg; 0 for a fixed object
    operator_movable: bool
    static_friction: float
    dynamic_friction: float
    colour: Tuple[float, float, float]
    jitter: Tuple[float, float, float]

    @property
    def footprint_radius(self) -> float:
        sx, sy, _ = self.size
        return 0.5 * math.sqrt(sx * sx + sy * sy) if self.shape == "box" else 0.5 * sx

    @property
    def nominal(self) -> Tuple[float, float, float, float, float, float, float]:
        """The nominal pose: x, y, z, then the quaternion w, x, y, z (a turn about the vertical)."""
        return (*self.position, math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2))


@dataclass(frozen=True)
class Scene:
    name: str
    robot_profile: str
    surfaces: Tuple[Surface, ...]
    table: Surface
    bench: Optional[Surface]
    objects: Tuple[Object, ...]
    region: Tuple[float, float, float, float]
    clearance: float
    extra: Dict[str, Any] = field(default_factory=dict)

    def fits_on_table(self, index: int, x: float, y: float, scale: float = 1.0) -> bool:
        r = self.objects[index].footprint_radius * scale
        t, (x0, x1, y0, y1) = self.table, self.region
        return (x - r >= max(t.x_min, x0) - 1e-9 and x + r <= min(t.x_max, x1) + 1e-9 and
                y - r >= max(t.y_min, y0) - 1e-9 and y + r <= min(t.y_max, y1) + 1e-9)

    def overlaps(self, a: int, ax: float, ay: float, b: int, bx: float, by: float, scale_a: float = 1.0, scale_b: float = 1.0) -> bool:
        reach = self.objects[a].footprint_radius * scale_a + self.objects[b].footprint_radius * scale_b + self.clearance
        return (ax - bx) ** 2 + (ay - by) ** 2 < reach * reach

    def pose_at(self, index: int, x: float, y: float, yaw: float, scale: float = 1.0) -> Tuple[float, ...]:
        """Object ``index`` upright on the table at (x, y), turned ``yaw``, resized by ``scale``: x, y, z, qw, qx, qy, qz."""
        return (x, y, self.table.top + self.objects[index].size[2] * scale / 2, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


# The sizes the headset may give an object (Core's WorldScene resize steps, docs/adr/0034 amendment 6): 1.25 to the power
# -3 to 5. Isaac builds a body for each and swaps them: a collider cannot change while PhysX simulates.
RESIZE_STEP, MIN_RESIZE_STEP, MAX_RESIZE_STEP = 1.25, -3, 5
RESIZE_SCALES = tuple(RESIZE_STEP ** k for k in range(MIN_RESIZE_STEP, MAX_RESIZE_STEP + 1))


def step_of(scale: float) -> int:
    return round(math.log(scale) / math.log(RESIZE_STEP)) if scale > 0 else MIN_RESIZE_STEP


def is_valid_scale(scale: float) -> bool:
    if not (math.isfinite(scale) and scale > 0):
        return False
    k = step_of(scale)
    return MIN_RESIZE_STEP <= k <= MAX_RESIZE_STEP and abs(scale - RESIZE_STEP ** k) <= 1e-4 * scale


def _bad(what: str, detail: str) -> SceneError:
    return SceneError(f"{what}: {detail}")


def _finite(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _vector(v: Any, n: int, what: str, where: str) -> Tuple[float, ...]:
    if not isinstance(v, list) or len(v) != n:
        raise _bad(what, f"{where} must have {n} numbers")
    if not all(_finite(x) for x in v):
        raise _bad(what, f"{where} must be finite")
    return tuple(float(x) for x in v)


def _colour(c: Any, where: str) -> Tuple[float, float, float]:
    if not isinstance(c, list) or len(c) != 3:
        raise _bad("colour", f"{where} needs [r, g, b]")
    if not all(_finite(x) and 0 <= x <= 1 for x in c):
        raise _bad("colour", f"{where}: each part runs from 0 to 1")
    return tuple(float(x) for x in c)


def _friction(s: Any, d: Any, where: str) -> None:
    if not (_finite(s) and _finite(d) and s >= 0 and d >= 0):
        raise _bad("friction", f"{where}: 0 or more, and finite")


def parse(doc: Dict[str, Any]) -> Scene:
    """Builds and validates a scene from the file's JSON object."""
    if doc.get("format", "") != FORMAT:
        raise _bad("format", f"'{doc.get('format', '')}', not '{FORMAT}'")
    version = doc.get("version", 0)
    if not isinstance(version, int) or isinstance(version, bool) or not 1 <= version <= VERSION:
        raise _bad("version", f"{version}; this reader knows versions 1 to {VERSION}")
    name = doc.get("name", "")
    if not isinstance(name, str) or not name.strip():
        raise _bad("name", "the scene has no name")
    robot_profile = doc.get("robotProfile", "")
    if not isinstance(robot_profile, str) or not robot_profile.strip():
        raise _bad("robotProfile", "the scene names no robot profile")

    surfaces: List[Surface] = []
    for i, s in enumerate(doc.get("surfaces", []) or []):
        if not isinstance(s, dict):
            raise _bad("surface", f"surface {i} is missing")
        sid = s.get("id", "")
        if not isinstance(sid, str) or not sid.strip():
            raise _bad("surface", f"surface {i} has no id")
        if any(x.id == sid for x in surfaces):
            raise _bad("surface", f"two surfaces are called '{sid}'")
        c = _vector(s.get("center"), 3, "surface", f"'{sid}' center")
        z = _vector(s.get("size"), 3, "surface", f"'{sid}' size")
        if not all(v > 0 for v in z):
            raise _bad("surface", f"'{sid}' has a size that is not positive")
        col = _colour(s.get("colour"), f"surface '{sid}'")
        _friction(s.get("staticFriction", 0.0), s.get("dynamicFriction", 0.0), f"surface '{sid}'")
        headset = s.get("headset", "show")
        if headset not in ("builtin", "show", "hide"):
            raise _bad("surface", f"'{sid}' has headset '{headset}': builtin, show or hide")
        surfaces.append(Surface(sid, c, z, col, float(s.get("staticFriction", 0.0)), float(s.get("dynamicFriction", 0.0)),
                                bool(s.get("collides", True)), headset))
    table = next((s for s in surfaces if s.id == "table"), None)
    bench = next((s for s in surfaces if s.id == "bench"), None)
    if table is None:
        raise _bad("table", "the scene has no surface called 'table'")
    if bench is not None:
        if abs(table.x_min - bench.x_max) > TOLERANCE:
            raise _bad("table", f"it must abut the bench's far edge: it starts at x = {table.x_min:.3f}, the bench ends at {bench.x_max:.3f}")
        if abs(table.top - bench.top) > TOLERANCE:
            raise _bad("table", f"its top ({table.top:.4f}) must be flush with the bench's ({bench.top:.4f})")

    p = doc.get("placement", {}) or {}
    x0, x1, y0, y1, clearance = (p.get(k, 0.0) for k in ("xMin", "xMax", "yMin", "yMax", "clearance"))
    if not (all(_finite(v) for v in (x0, x1, y0, y1, clearance)) and x0 < x1 and y0 < y1 and clearance >= 0):
        raise _bad("placement", "the region must have min < max on both axes and a clearance of 0 or more")
    if x1 < table.x_min or x0 > table.x_max or y1 < table.y_min or y0 > table.y_max:
        raise _bad("placement", "the region does not overlap the table")

    docs = doc.get("objects", []) or []
    if len(docs) < 1 or len(docs) > MAX_OBJECTS:
        raise _bad("objects", f"{len(docs)}; a scene holds 1 to {MAX_OBJECTS}")
    objects: List[Object] = []
    for i, o in enumerate(docs):
        if not isinstance(o, dict):
            raise _bad("objects", f"object {i} is missing")
        oid, oname = o.get("id", 0), o.get("name", "")
        if not isinstance(oid, int) or isinstance(oid, bool) or oid < 1 or oid > 254:
            raise _bad("object id", f"{oid}; ids run from 1 to 254")
        if not isinstance(oname, str) or not _NAME.match(oname):
            raise _bad("object name", f"'{oname}': letters, digits, '.', '_' and '-' only")
        for k in objects:
            if k.id == oid:
                raise _bad("object id", f"two objects have id {oid}")
            if k.name == oname:
                raise _bad("object name", f"two objects are called '{oname}'")
        shape = o.get("shape", "")
        if shape not in ("box", "cylinder", "sphere"):
            raise _bad("shape", f"'{oname}' has shape '{shape}': box, cylinder or sphere")
        z = _vector(o.get("size"), 3, "size", f"'{oname}'")
        if not all(v > 0 for v in z):
            raise _bad("size", f"'{oname}' has a size that is not positive")
        if shape == "cylinder" and abs(z[0] - z[1]) > 1e-9:
            raise _bad("size", f"cylinder '{oname}' must be [d, d, h]")
        if shape == "sphere" and (abs(z[0] - z[1]) > 1e-9 or abs(z[0] - z[2]) > 1e-9):
            raise _bad("size", f"sphere '{oname}' must be [d, d, d]")
        kind = o.get("kind", "")
        movable = kind == "movable"
        if kind != "fixed" and not (movable and version >= 2):
            raise _bad("kind", f"'{oname}' is '{kind}': fixed or movable" if version >= 2 else f"'{oname}' is '{kind}'; version 1 has fixed objects only")
        mass = o.get("mass", 0.0)
        if not isinstance(mass, (int, float)) or isinstance(mass, bool) or (not 0 < mass <= MAX_MASS if movable else mass != 0):
            raise _bad("mass", f"movable '{oname}' has mass {mass}: more than 0 and at most {MAX_MASS:g} kg" if movable
                       else f"fixed '{oname}' has a mass: only a movable object has one")
        pos = _vector(o.get("position"), 3, "pose", f"'{oname}' position")
        rpy = _vector(o.get("rpy"), 3, "pose", f"'{oname}' rpy")
        if abs(rpy[0]) > 1e-9 or abs(rpy[1]) > 1e-9:
            raise _bad("pose", f"'{oname}' must stand upright (roll and pitch 0)")
        if abs(pos[2] - (table.top + z[2] / 2)) > TOLERANCE:
            raise _bad("rests", f"'{oname}' must rest on the table: its centre at z = {table.top + z[2] / 2:.4f}, not {pos[2]:.4f}")
        col = _colour(o.get("colour"), f"object '{oname}'")
        _friction(o.get("staticFriction", 0.0), o.get("dynamicFriction", 0.0), f"object '{oname}'")
        j = _vector(o.get("jitter"), 3, "jitter", f"'{oname}'")
        if not all(v >= 0 for v in j):
            raise _bad("jitter", f"'{oname}' has a negative range")
        objects.append(Object(oid, oname, shape, z, pos, rpy[2], movable, float(mass), bool(o.get("operatorMovable", False)),
                              float(o.get("staticFriction", 0.0)), float(o.get("dynamicFriction", 0.0)), col, j))

    known = {"format", "version", "name", "robotProfile", "surfaces", "objects", "placement"}
    scene = Scene(name, robot_profile, tuple(surfaces), table, bench, tuple(objects), (float(x0), float(x1), float(y0), float(y1)),
                  float(clearance), {k: v for k, v in doc.items() if k not in known})
    for i, o in enumerate(objects):
        for sx in (-1, 0, 1):
            for sy in (-1, 0, 1):
                if not scene.fits_on_table(i, o.position[0] + sx * o.jitter[0], o.position[1] + sy * o.jitter[1]):
                    raise _bad("fits", f"'{o.name}' leaves the table or the placement region at its nominal pose or the edge of its jitter")
        for k in range(i):
            if scene.overlaps(i, o.position[0], o.position[1], k, objects[k].position[0], objects[k].position[1]):
                raise _bad("overlap", f"'{o.name}' and '{objects[k].name}' are closer than the clearance at their nominal poses")
    return scene


def load(path: str) -> Scene:
    import json
    with open(path, encoding="utf-8") as f:
        return parse(json.load(f))
