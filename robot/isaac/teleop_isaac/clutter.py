"""Flat clutter on the JetRover twin's table (docs/adr/0037, "Surfaces and flat clutter"): paper sheets and sticky notes, a
cutting mat, loose cables and a flattened plastic bag, as on the real lab's table, drawn by the renderer alone.

**Why the renderer alone.** The JetRover collision model's floor is the table top plus a 10 mm margin, which RobotHost's
refusal check and every SafetyScript pose keep the arm above, so nothing 6 mm or thinner lying on the table can be touched
in an accepted pose (and ``contacts.py`` ignores the robot on a surface anyway). Clutter that flat needs no body in the
bridge, no place in the planner and no contact label. :data:`MAX_HEIGHT` is that bound: :func:`spec` refuses a kind that
could be taller, and :func:`draw` keeps every piece's top, with whatever lies under it, within it.

**The draw** is a pure function of the scene file's ``clutter`` section, the seed and the change (``random.Random`` keyed by
them): how many placements (``countMin`` to ``countMax``), each one's kind by the kinds' ``weight`` (``maxCount`` caps a
kind), and each one's spot in ``region``, clear of ``keepOut`` (the robot) and of its kind's own ``keepOut``. Pieces go down
in layers, sheets (the thickest first: a mat under papers), then bags, then cables, each lying on what is under it:

- **sheet:** a stack of ``stackMin`` to ``stackMax`` sheets of ``size`` (width along x and length along y before it turns)
  and ``thickness``, each moved up to ``spread`` and turned up to ``fan`` from the stack's spot, wearing ``texture`` tinted
  by one of ``colours`` (flat ``[r, g, b, r, g, b, ...]``). ``yawJitter`` keeps it square to the table within that, else it
  turns any way. Sheets may lie under objects.
- **cable:** a tube ``diameterMin`` to ``diameterMax`` across, ``lengthMin`` to ``lengthMax`` long along a smooth seeded
  curve (its curvature a random walk); it steers around keep-outs, other cables and bags, and ends where it cannot go on.
- **bag:** a flattened plastic bag of ``size`` (each side within ``sizeJitter``), a wrinkled grid at most ``height`` tall,
  opaque and glossy (``roughness``).

The renderer rebuilds the pieces at each change under ``/World/Clutter`` (:func:`mesh` gives each one's geometry), labels
them mask class ``clutter`` and records them as ``CL`` (:func:`cl_items`).
"""

from __future__ import annotations

import math
import random
import re
from dataclasses import dataclass, replace
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

MAX_HEIGHT = 0.006  # m above the table top: under the collision model's 10 mm floor margin, so never touched
MAX_COUNT = 64
TRIES = 40
CABLE_STEP = 0.01  # m between a cable's centreline points
CABLE_SIDES = 8
CABLE_CLEAR = 0.001  # m between two cables, or a cable and a bag: a crossing would stand taller than MAX_HEIGHT
CABLE_TURNS = (0.0, 0.08, -0.08, 0.16, -0.16, 0.25, -0.25)  # rad a step may turn to steer around what it meets (a
# bend of 4 cm radius at the tightest); one that cannot ends there rather than kink
LOOK_AHEAD = 0.06  # m: a cable starts to bend this far before an edge or another cable
CURVATURE_STEP, CURVATURE_KEEP, CURVATURE_MAX = 1.5, 0.92, 12.0  # the curvature's random walk, 1/m
BAG_STEP = 0.01  # m between a bag's grid points
SHAPES = ("sheet", "cable", "bag")
LAYERS = {"sheet": 0, "bag": 1, "cable": 2}
KIND_ID = re.compile(r"^[a-z0-9][a-z0-9-]{0,39}$")
_SECTION = {"countMin", "countMax", "region", "keepOut", "kinds"}
_COMMON = {"kind", "shape", "weight", "maxCount", "keepOut", "colours"}
_FIELDS = {"sheet": _COMMON | {"size", "thickness", "stackMin", "stackMax", "fan", "spread", "yawJitter", "texture"},
           "cable": _COMMON | {"diameterMin", "diameterMax", "lengthMin", "lengthMax", "roughness"},
           "bag": _COMMON | {"size", "sizeJitter", "height", "yawJitter", "roughness"}}

Rgb = Tuple[float, float, float]


# --- geometry ------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Rect:
    """A rectangle on the table: centre, half width (along its own x), half length (along its own y), turn about z."""

    x: float
    y: float
    hw: float
    hl: float
    yaw: float = 0.0

    @staticmethod
    def bounds(x0: float, x1: float, y0: float, y1: float) -> "Rect":
        return Rect((x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, 0.0)

    def local(self, px: float, py: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.x, py - self.y
        return c * dx + s * dy, -s * dx + c * dy

    def world(self, u: float, v: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * u - s * v, self.y + s * u + c * v

    def contains(self, px: float, py: float, margin: float = 0.0) -> bool:
        u, v = self.local(px, py)
        return abs(u) <= self.hw + margin and abs(v) <= self.hl + margin

    def corners(self) -> List[Tuple[float, float]]:
        return [self.world(su * self.hw, sv * self.hl) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def overlap(a: Rect, b: Rect) -> bool:
    """Whether two rectangles share any area (the separating-axis test)."""
    ca, cb = a.corners(), b.corners()
    for r in (a, b):
        for ax, ay in ((math.cos(r.yaw), math.sin(r.yaw)), (-math.sin(r.yaw), math.cos(r.yaw))):
            pa = [x * ax + y * ay for x, y in ca]
            pb = [x * ax + y * ay for x, y in cb]
            if max(pa) <= min(pb) or max(pb) <= min(pa):
                return False
    return True


def inside(r: Rect, region: Sequence[float]) -> bool:
    x0, x1, y0, y1 = region
    return all(x0 - 1e-12 <= x <= x1 + 1e-12 and y0 - 1e-12 <= y <= y1 + 1e-12 for x, y in r.corners())


def _point_segment(p, a, b) -> float:
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / n))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _orient(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segment_distance(p1, q1, p2, q2) -> float:
    """The least distance between two segments in the plane (0 when they cross)."""
    if _orient(p1, q1, p2) * _orient(p1, q1, q2) < 0 and _orient(p2, q2, p1) * _orient(p2, q2, q1) < 0:
        return 0.0
    return min(_point_segment(p1, p2, q2), _point_segment(q1, p2, q2), _point_segment(p2, p1, q1), _point_segment(q2, p1, q1))


class _Segments:
    """Segments with a radius each, hashed on a grid, for what lies near a new one."""

    CELL = 0.02

    def __init__(self):
        self.cells: Dict[Tuple[int, int], List[tuple]] = {}
        self.widest = 0.0

    def _cells(self, p, q, reach: float) -> Iterable[Tuple[int, int]]:
        x0, x1 = min(p[0], q[0]) - reach, max(p[0], q[0]) + reach
        y0, y1 = min(p[1], q[1]) - reach, max(p[1], q[1]) + reach
        for i in range(int(math.floor(x0 / self.CELL)), int(math.floor(x1 / self.CELL)) + 1):
            for j in range(int(math.floor(y0 / self.CELL)), int(math.floor(y1 / self.CELL)) + 1):
                yield i, j

    def add(self, p, q, radius: float, tag: int = 0) -> None:
        entry = (p, q, radius, tag)
        for c in self._cells(p, q, 0.0):
            self.cells.setdefault(c, []).append(entry)
        self.widest = max(self.widest, radius)

    def near(self, p, q, reach: float) -> List[tuple]:
        seen, out = set(), []
        for c in self._cells(p, q, reach + self.widest):
            for e in self.cells.get(c, ()):
                if id(e) not in seen:
                    seen.add(id(e))
                    out.append(e)
        return out


# --- the scene's section ---------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Kind:
    kind: str
    shape: str  # sheet | cable | bag
    weight: float
    max_count: int  # 0: as many as the count draws
    keep_out: Tuple[Rect, ...]
    colours: Tuple[Rgb, ...]
    texture: str = ""
    size: Tuple[float, float] = (0.0, 0.0)  # sheet, bag: width along x, length along y, before it turns
    thickness: float = 0.0  # sheet
    stack: Tuple[int, int] = (1, 1)  # sheet
    fan: float = 0.0  # sheet: rad
    spread: float = 0.0  # sheet: m
    yaw_jitter: Optional[float] = None  # sheet, bag: square to the table within this; None: any way
    diameter: Tuple[float, float] = (0.0, 0.0)  # cable
    length: Tuple[float, float] = (0.0, 0.0)  # cable
    size_jitter: float = 0.0  # bag
    height: float = 0.0  # bag
    roughness: Optional[float] = None  # cable, bag

    @property
    def tallest(self) -> float:
        """The most this kind can stand above what it lies on."""
        return {"sheet": self.thickness * self.stack[1], "cable": self.diameter[1], "bag": self.height}[self.shape]


@dataclass(frozen=True)
class Spec:
    count: Tuple[int, int]
    region: Tuple[float, float, float, float]
    keep_out: Tuple[Rect, ...]
    kinds: Tuple[Kind, ...]


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _whole(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _rects(v, what: str) -> Tuple[Rect, ...]:
    if not isinstance(v, list) or len(v) % 4 or not all(_num(x) for x in v):
        raise ValueError(f"{what} holds xMin, xMax, yMin, yMax for each rectangle")
    out = []
    for k in range(0, len(v), 4):
        x0, x1, y0, y1 = (float(x) for x in v[k:k + 4])
        if not (x0 < x1 and y0 < y1):
            raise ValueError(f"{what}: each rectangle needs min < max on both axes")
        out.append(Rect.bounds(x0, x1, y0, y1))
    return tuple(out)


def _colours(v, what: str) -> Tuple[Rgb, ...]:
    if not isinstance(v, list) or not v or len(v) % 3 or not all(_num(x) and 0 <= x <= 1 for x in v):
        raise ValueError(f"{what} colours are [r, g, b, r, g, b, ...], each 0 to 1")
    return tuple((float(v[k]), float(v[k + 1]), float(v[k + 2])) for k in range(0, len(v), 3))


def _kind(d, textures: Sequence[str]) -> Kind:
    if not isinstance(d, dict):
        raise ValueError("clutter kinds: each is an object")
    name = d.get("kind")
    if not isinstance(name, str) or not KIND_ID.match(name):
        raise ValueError(f"clutter kind {name!r}: lowercase letters, digits and '-'")
    w = f"clutter {name!r}"
    shape = d.get("shape")
    if shape not in SHAPES:
        raise ValueError(f"{w}: shape {shape!r}, not one of {', '.join(SHAPES)}")
    unknown = sorted(set(d) - _FIELDS[shape])
    if unknown:
        raise ValueError(f"{w}: a {shape} has no field {', '.join(unknown)}")
    weight = d.get("weight", 1.0)
    if not (_num(weight) and weight > 0):
        raise ValueError(f"{w}: weight must be a positive number")
    max_count = d.get("maxCount", 0)
    if not (_whole(max_count) and max_count >= 0):
        raise ValueError(f"{w}: maxCount is a whole number, 0 for no cap")
    k = Kind(name, shape, float(weight), max_count, _rects(d.get("keepOut", []), f"{w} keepOut"),
             _colours(d.get("colours", [1.0, 1.0, 1.0]) if shape == "sheet" else d.get("colours"), w))

    def positive(key: str, top: float = math.inf) -> float:
        v = d.get(key)
        if not (_num(v) and 0 < v <= top):
            raise ValueError(f"{w}: {key} must be a positive number" + ("" if top == math.inf else f", at most {top:g}"))
        return float(v)

    def unit(key: str) -> Optional[float]:
        v = d.get(key)
        if v is not None and not (_num(v) and 0 <= v <= 1):
            raise ValueError(f"{w}: {key} is 0 to 1")
        return None if v is None else float(v)

    def jitter() -> Optional[float]:
        v = d.get("yawJitter")
        if v is not None and not (_num(v) and 0 <= v <= math.pi):
            raise ValueError(f"{w}: yawJitter is 0 to pi rad")
        return None if v is None else float(v)

    def size() -> Tuple[float, float]:
        v = d.get("size")
        if not (isinstance(v, list) and len(v) == 2 and all(_num(x) and x > 0 for x in v)):
            raise ValueError(f"{w}: size is [width, length], m")
        return float(v[0]), float(v[1])

    if shape == "sheet":
        tex = d.get("texture")
        if tex not in textures:
            raise ValueError(f"{w}: texture {tex!r} is not one of the textures")
        lo, hi = d.get("stackMin", 1), d.get("stackMax", 1)
        if not (_whole(lo) and _whole(hi) and 1 <= lo <= hi):
            raise ValueError(f"{w}: stackMin and stackMax are whole numbers, 1 <= stackMin <= stackMax")
        fan, spread = d.get("fan", 0.0), d.get("spread", 0.0)
        if not (_num(fan) and 0 <= fan <= math.pi and _num(spread) and 0 <= spread <= 0.5):
            raise ValueError(f"{w}: fan is 0 to pi rad, spread 0 to 0.5 m")
        k = replace(k, texture=tex, size=size(), thickness=positive("thickness"), stack=(lo, hi), fan=float(fan),
                    spread=float(spread), yaw_jitter=jitter())
    elif shape == "cable":
        dl, dh = positive("diameterMin"), positive("diameterMax")
        ll, lh = positive("lengthMin", 5.0), positive("lengthMax", 5.0)
        if dl > dh or ll > lh:
            raise ValueError(f"{w}: a min above its max")
        k = replace(k, diameter=(dl, dh), length=(ll, lh), roughness=unit("roughness"))
    else:
        sj = d.get("sizeJitter", 0.0)
        if not (_num(sj) and 0 <= sj < 0.5):
            raise ValueError(f"{w}: sizeJitter is 0 to 0.5")
        k = replace(k, size=size(), size_jitter=float(sj), height=positive("height"), yaw_jitter=jitter(), roughness=unit("roughness"))
    if k.tallest > MAX_HEIGHT + 1e-12:
        raise ValueError(f"{w}: {k.tallest * 1000:g} mm at its thickest, thicker than {MAX_HEIGHT * 1000:g} mm: the arm could "
                         f"touch it, and only the renderer draws clutter")
    return k


def spec(scene, textures: Optional[Sequence[str]] = None) -> Optional[Spec]:
    """The scene file's ``clutter`` section, checked, or None when it has none. ``textures``: the texture ids a sheet may
    wear (default: every one of ``materials.TEXTURES``)."""
    c = scene.extra.get("clutter")
    if c is None:
        return None
    if textures is None:
        from .materials import TEXTURES

        textures = tuple(TEXTURES)
    if not isinstance(c, dict):
        raise ValueError("clutter is an object")
    unknown = sorted(set(c) - _SECTION)
    if unknown:
        raise ValueError(f"clutter has no field {', '.join(unknown)}")
    lo, hi = c.get("countMin", 0), c.get("countMax", 0)
    if not (_whole(lo) and _whole(hi) and 0 <= lo <= hi <= MAX_COUNT):
        raise ValueError(f"clutter countMin and countMax are whole numbers, 0 <= countMin <= countMax <= {MAX_COUNT}")
    region = c.get("region")
    if not (isinstance(region, list) and len(region) == 4 and all(_num(x) for x in region)) or not (
            region[0] < region[1] and region[2] < region[3]):
        raise ValueError("clutter region is [xMin, xMax, yMin, yMax] with min < max")
    t = scene.table
    if region[0] < t.x_min - 1e-9 or region[1] > t.x_max + 1e-9 or region[2] < t.y_min - 1e-9 or region[3] > t.y_max + 1e-9:
        raise ValueError(f"clutter region must lie on the table (x {t.x_min:.3f} to {t.x_max:.3f}, y {t.y_min:.3f} to {t.y_max:.3f})")
    kinds = c.get("kinds")
    if not isinstance(kinds, list) or not kinds:
        raise ValueError("clutter kinds: a list of at least one")
    parsed = tuple(_kind(d, textures) for d in kinds)
    names = [k.kind for k in parsed]
    for n in names:
        if names.count(n) > 1:
            raise ValueError(f"clutter kinds: two are called {n!r}")
    return Spec((lo, hi), tuple(float(x) for x in region), _rects(c.get("keepOut", []), "clutter keepOut"), parsed)


def usable(s: Optional[Spec], textures: Iterable[str]) -> Optional[Spec]:
    """``s`` without the kinds whose texture is not among ``textures`` (not fetched), or None when no kind is left."""
    if s is None:
        return None
    have = set(textures)
    kinds = tuple(k for k in s.kinds if not k.texture or k.texture in have)
    return replace(s, kinds=kinds) if kinds else None


# --- the draw --------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Piece:
    """One piece of clutter. ``x``, ``y``, ``yaw`` are what ``CL`` records: a sheet's or bag's centre and turn, a cable's
    midpoint along its length and its direction there. ``base`` and ``top`` are its lowest and highest points above the
    table top. ``size`` is a sheet's or bag's (width, length), a cable's (diameter, length)."""

    kind: str
    shape: str
    x: float
    y: float
    yaw: float
    base: float
    top: float
    size: Tuple[float, float]
    colour: Rgb
    texture: str = ""
    roughness: Optional[float] = None
    path: Tuple[Tuple[float, float, float], ...] = ()  # a cable's centreline: x, y, and its centre's height above the table
    grid: Tuple[Tuple[float, ...], ...] = ()  # a bag's wrinkles: height above its base, one row per step along its length

    @property
    def rect(self) -> Rect:
        return Rect(self.x, self.y, self.size[0] / 2, self.size[1] / 2, self.yaw)


def _weighted(rng: random.Random, kinds: Sequence[Kind]) -> Kind:
    u = rng.random() * sum(k.weight for k in kinds)
    for k in kinds:
        if u < k.weight:
            return k
        u -= k.weight
    return kinds[-1]


def _yaw(rng: random.Random, jitter: Optional[float]) -> float:
    return rng.uniform(-jitter, jitter) if jitter is not None else rng.uniform(0.0, 2.0 * math.pi)


def _half_extents(w: float, l: float, yaw: float) -> Tuple[float, float]:
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return 0.5 * (w * c + l * s), 0.5 * (w * s + l * c)


def _flat_base(r: Rect, placed: Sequence[Piece]) -> float:
    return max((p.top for p in placed if p.shape in ("sheet", "bag") and overlap(r, p.rect)), default=0.0)


def _sheets(k: Kind, rng: random.Random, s: Spec, placed: List[Piece]) -> List[Piece]:
    w, l = k.size
    count = rng.randint(*k.stack)
    colour = k.colours[rng.randrange(len(k.colours))]
    keep = s.keep_out + k.keep_out
    x0, x1, y0, y1 = s.region
    for _ in range(TRIES):
        yaw0 = _yaw(rng, k.yaw_jitter)
        ex, ey = _half_extents(w, l, yaw0)
        if 2 * ex > x1 - x0 or 2 * ey > y1 - y0:
            continue
        cx, cy = rng.uniform(x0 + ex, x1 - ex), rng.uniform(y0 + ey, y1 - ey)
        stack: List[Piece] = []
        for j in range(count):
            if j == 0:
                r = Rect(cx, cy, w / 2, l / 2, yaw0)
            else:
                r = Rect(cx + rng.uniform(-k.spread, k.spread), cy + rng.uniform(-k.spread, k.spread), w / 2, l / 2,
                         yaw0 + rng.uniform(-k.fan, k.fan))
            if not inside(r, s.region) or any(overlap(r, b) for b in keep):
                if j == 0:
                    break
                continue
            base = _flat_base(r, placed + stack)
            if base + k.thickness > MAX_HEIGHT + 1e-12:
                if j == 0:
                    break
                continue
            stack.append(Piece(k.kind, "sheet", r.x, r.y, r.yaw, base, base + k.thickness, (w, l), colour, k.texture))
        if stack:
            return stack
    return []


def _lift(px: float, py: float, r: float, sheets: Sequence[Piece]) -> float:
    return max((p.top for p in sheets if p.rect.contains(px, py, r)), default=0.0)


def _cable(k: Kind, rng: random.Random, s: Spec, placed: List[Piece]) -> List[Piece]:
    colour = k.colours[rng.randrange(len(k.colours))]
    keep = s.keep_out + k.keep_out
    sheets = [p for p in placed if p.shape == "sheet"]
    bags = [p.rect for p in placed if p.shape == "bag"]
    others = _Segments()
    for p in placed:
        if p.shape == "cable":
            for a, b in zip(p.path, p.path[1:]):
                others.add(a[:2], b[:2], p.size[0] / 2)
    x0, x1, y0, y1 = s.region
    for _ in range(TRIES):
        d = rng.uniform(*k.diameter)
        r = d / 2
        steps = max(1, int(round(rng.uniform(*k.length) / CABLE_STEP)))

        def clear(px: float, py: float) -> bool:
            if not (x0 + r <= px <= x1 - r and y0 + r <= py <= y1 - r):
                return False
            if any(b.contains(px, py, r) for b in keep) or any(b.contains(px, py, r + CABLE_CLEAR) for b in bags):
                return False
            return _lift(px, py, r, sheets) + d <= MAX_HEIGHT + 1e-12

        own = _Segments()

        def free(a, b, n_own: int) -> bool:
            for p, q, ro, _ in others.near(a, b, r + CABLE_CLEAR):
                if segment_distance(a, b, p, q) < r + ro + CABLE_CLEAR:
                    return False
            for p, q, _, tag in own.near(a, b, r + CABLE_CLEAR):
                if tag < n_own - 3 and segment_distance(a, b, p, q) < d + CABLE_CLEAR:
                    return False
            return True

        start = (rng.uniform(x0 + r, x1 - r), rng.uniform(y0 + r, y1 - r))
        if not clear(*start) or not free(start, start, 0):
            continue
        def step(a, heading: float, curvature: float, n_own: int):
            """The next point and heading: the least turn that is clear, first of those still clear LOOK_AHEAD further
            on (so it bends early along an edge, as a cable lies along a desk's back), else of any; None: it ends."""
            for ahead in (LOOK_AHEAD, 0.0):
                for turn in CABLE_TURNS:
                    h = heading + curvature * CABLE_STEP + turn
                    c, sn = math.cos(h), math.sin(h)
                    b = (a[0] + CABLE_STEP * c, a[1] + CABLE_STEP * sn)
                    if clear(*b) and (ahead == 0.0 or clear(b[0] + ahead * c, b[1] + ahead * sn)) and free(a, b, n_own):
                        return h, b
            return None

        heading, curvature = rng.uniform(0.0, 2.0 * math.pi), 0.0
        pts = [start]
        for _ in range(steps):
            curvature = max(-CURVATURE_MAX, min(CURVATURE_MAX, curvature * CURVATURE_KEEP + rng.gauss(0.0, CURVATURE_STEP)))
            nxt = step(pts[-1], heading, curvature, len(pts) - 1)
            if nxt is None:
                break  # nowhere to go: the cable ends here
            own.add(pts[-1], nxt[1], r, len(pts) - 1)
            heading = nxt[0]
            pts.append(nxt[1])
        if (len(pts) - 1) * CABLE_STEP < k.length[0] - 1e-9:
            continue
        lift = [_lift(px, py, r, sheets) for px, py in pts]
        lift = [max(lift[max(0, i - 2):i + 3]) for i in range(len(lift))]  # it rises a little before what it lies on
        path = tuple((px, py, z + r) for (px, py), z in zip(pts, lift))
        m = len(pts) // 2
        a, b = pts[max(0, m - 1)], pts[min(len(pts) - 1, m + 1)]
        return [Piece(k.kind, "cable", pts[m][0], pts[m][1], math.atan2(b[1] - a[1], b[0] - a[0]), min(lift), max(lift) + d,
                      (d, (len(pts) - 1) * CABLE_STEP), colour, "", k.roughness, path=path)]
    return []


def _wrinkles(rng: random.Random, w: float, l: float, height: float) -> Tuple[Tuple[float, ...], ...]:
    """A flattened bag's height field over its rectangle: a few soft folds and fine ripples, low at its edges, its peak
    60-100% of ``height``."""
    nu, nv = max(2, int(math.ceil(w / BAG_STEP))), max(2, int(math.ceil(l / BAG_STEP)))
    reach = 0.3 * math.hypot(w, l)
    folds = []
    for _ in range(rng.randint(4, 8)):
        a = rng.uniform(0.0, math.pi)
        folds.append((math.cos(a), math.sin(a), rng.uniform(-reach, reach), rng.uniform(0.006, 0.03), rng.uniform(0.3, 1.0)))
    ripples = []
    for _ in range(3):
        b, k = rng.uniform(0.0, math.pi), 2.0 * math.pi / rng.uniform(0.02, 0.06)
        ripples.append((k * math.cos(b), k * math.sin(b), rng.uniform(0.0, 2.0 * math.pi), rng.uniform(0.05, 0.2)))
    raw = []
    for j in range(nv + 1):
        v = -l / 2 + l * j / nv
        row = []
        for i in range(nu + 1):
            u = -w / 2 + w * i / nu
            f = sum(amp * math.exp(-(((cu * u + sv * v) - off) / width) ** 2) for cu, sv, off, width, amp in folds)
            f += sum(amp * (0.5 + 0.5 * math.sin(kx * u + ky * v + ph)) for kx, ky, ph, amp in ripples)
            e = max(0.0, min(1.0, min(u + w / 2, w / 2 - u, v + l / 2, l / 2 - v) / 0.025))
            row.append(f * (0.25 + 0.75 * e * e * (3.0 - 2.0 * e)))
        raw.append(row)
    peak = max(max(row) for row in raw) or 1.0
    top = height * rng.uniform(0.6, 1.0)
    return tuple(tuple(max(0.0003, min(height, h / peak * top)) for h in row) for row in raw)


def _bag(k: Kind, rng: random.Random, s: Spec, placed: List[Piece]) -> List[Piece]:
    colour = k.colours[rng.randrange(len(k.colours))]
    keep = s.keep_out + k.keep_out
    x0, x1, y0, y1 = s.region
    for _ in range(TRIES):
        w = k.size[0] * (1.0 + rng.uniform(-k.size_jitter, k.size_jitter))
        l = k.size[1] * (1.0 + rng.uniform(-k.size_jitter, k.size_jitter))
        yaw = _yaw(rng, k.yaw_jitter)
        ex, ey = _half_extents(w, l, yaw)
        if 2 * ex > x1 - x0 or 2 * ey > y1 - y0:
            continue
        r = Rect(rng.uniform(x0 + ex, x1 - ex), rng.uniform(y0 + ey, y1 - ey), w / 2, l / 2, yaw)
        if any(overlap(r, b) for b in keep) or any(p.shape == "bag" and overlap(r, p.rect) for p in placed):
            continue
        base = _flat_base(r, placed)
        if base + k.height > MAX_HEIGHT + 1e-12:
            continue
        grid = _wrinkles(rng, w, l, k.height)
        return [Piece(k.kind, "bag", r.x, r.y, yaw, base, base + max(max(row) for row in grid), (w, l), colour, "",
                      k.roughness, grid=grid)]
    return []


_PLACE = {"sheet": _sheets, "cable": _cable, "bag": _bag}


def draw(s: Spec, seed: int, change: int) -> List[Piece]:
    """The clutter of change ``change``, from ``s``, the seed and the change alone, in the order it went down."""
    rng = random.Random(f"teleop-clutter:{seed}:{change}")
    counts: Dict[str, int] = {}
    chosen: List[Kind] = []
    for _ in range(rng.randint(*s.count)):
        pool = [k for k in s.kinds if k.max_count == 0 or counts.get(k.kind, 0) < k.max_count]
        if not pool:
            break
        k = _weighted(rng, pool)
        counts[k.kind] = counts.get(k.kind, 0) + 1
        chosen.append(k)
    placed: List[Piece] = []
    for i in sorted(range(len(chosen)), key=lambda i: (LAYERS[chosen[i].shape], -chosen[i].tallest, i)):
        # Each placement its own stream: one that finds no spot leaves every other's draw as it was.
        placed.extend(_PLACE[chosen[i].shape](chosen[i], random.Random(f"teleop-clutter:{seed}:{change}:{i}"), s, placed))
    return placed


def cl_items(pieces: Sequence[Piece]) -> List[list]:
    """``CL``'s list: ``kind``, x, y (m, the robot's base frame) and yaw (rad, 0 to 2 pi) of each piece."""
    return [[p.kind, round(p.x, 4), round(p.y, 4), round(p.yaw % (2.0 * math.pi), 4)] for p in pieces]


# --- the meshes ------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Mesh:
    """A piece's geometry in the scene's frame: points, faces (``counts`` and ``indices``), a normal per point, and texture
    coordinates per point in metres in the piece's own frame (a sheet's; empty for the rest)."""

    points: List[Tuple[float, float, float]]
    counts: List[int]
    indices: List[int]
    normals: List[Tuple[float, float, float]]
    st: List[Tuple[float, float]]


def _sheet_mesh(p: Piece, table_top: float) -> Mesh:
    r = p.rect
    hw, hl = r.hw, r.hl
    z0, z1 = table_top + p.base, table_top + p.top
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    faces = (  # (local corners u, v, z), local normal: the top and the four edges; nothing sees the underside
        (((-hw, -hl, z1), (hw, -hl, z1), (hw, hl, z1), (-hw, hl, z1)), (0.0, 0.0, 1.0)),
        (((-hw, -hl, z0), (hw, -hl, z0), (hw, -hl, z1), (-hw, -hl, z1)), (0.0, -1.0, 0.0)),
        (((hw, hl, z0), (-hw, hl, z0), (-hw, hl, z1), (hw, hl, z1)), (0.0, 1.0, 0.0)),
        (((-hw, hl, z0), (-hw, -hl, z0), (-hw, -hl, z1), (-hw, hl, z1)), (-1.0, 0.0, 0.0)),
        (((hw, -hl, z0), (hw, hl, z0), (hw, hl, z1), (hw, -hl, z1)), (1.0, 0.0, 0.0)),
    )
    points, normals, st = [], [], []
    for corners, (nx, ny, nz) in faces:
        for u, v, z in corners:
            points.append((*r.world(u, v), z))
            normals.append((c * nx - s * ny, s * nx + c * ny, nz))
            st.append((u + hw, v + hl))
    return Mesh(points, [4] * len(faces), list(range(4 * len(faces))), normals, st)


def _cable_mesh(p: Piece, table_top: float) -> Mesh:
    path = [(x, y, table_top + z) for x, y, z in p.path]
    r, n, sides = p.size[0] / 2, len(path), CABLE_SIDES
    points, normals, counts, indices = [], [], [], []
    tangents = []
    for i in range(n):
        a, b = path[max(0, i - 1)], path[min(n - 1, i + 1)]
        t = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        k = math.sqrt(sum(c * c for c in t)) or 1.0
        tangents.append(tuple(c / k for c in t))
    for (x, y, z), t in zip(path, tangents):
        h = math.hypot(t[0], t[1]) or 1.0
        n1 = (-t[1] / h, t[0] / h, 0.0)  # level, to the cable's left
        n2 = (t[1] * n1[2] - t[2] * n1[1], t[2] * n1[0] - t[0] * n1[2], t[0] * n1[1] - t[1] * n1[0])  # t x n1: up
        for j in range(sides):
            ph = 2.0 * math.pi * j / sides
            q = tuple(math.cos(ph) * a + math.sin(ph) * b for a, b in zip(n1, n2))
            points.append((x + r * q[0], y + r * q[1], z + r * q[2]))
            normals.append(q)
    for i in range(n - 1):
        for j in range(sides):
            k = (j + 1) % sides
            counts.append(4)
            indices += [i * sides + j, i * sides + k, (i + 1) * sides + k, (i + 1) * sides + j]
    for end, sign in ((0, -1.0), (n - 1, 1.0)):  # the two cut ends, each its own points so it shades flat
        t = tuple(sign * c for c in tangents[end])
        centre = len(points)
        points.append(path[end])
        normals.append(t)
        ring = len(points)
        for j in range(sides):
            points.append(points[end * sides + j])
            normals.append(t)
        for j in range(sides):
            k = (j + 1) % sides
            counts.append(3)
            indices += [centre, ring + k, ring + j] if sign < 0 else [centre, ring + j, ring + k]
    return Mesh(points, counts, indices, normals, [])


def _bag_mesh(p: Piece, table_top: float) -> Mesh:
    r, g = p.rect, p.grid
    nv, nu = len(g) - 1, len(g[0]) - 1
    w, l = p.size
    du, dv = w / nu, l / nv
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    points, normals, counts, indices = [], [], [], []
    for j in range(nv + 1):
        for i in range(nu + 1):
            u, v = -w / 2 + du * i, -l / 2 + dv * j
            points.append((*r.world(u, v), table_top + p.base + g[j][i]))
            hu = (g[j][min(nu, i + 1)] - g[j][max(0, i - 1)]) / (du * (min(nu, i + 1) - max(0, i - 1)))
            hv = (g[min(nv, j + 1)][i] - g[max(0, j - 1)][i]) / (dv * (min(nv, j + 1) - max(0, j - 1)))
            k = 1.0 / math.sqrt(hu * hu + hv * hv + 1.0)
            nx, ny = -hu * k, -hv * k
            normals.append((c * nx - s * ny, s * nx + c * ny, k))
    for j in range(nv):
        for i in range(nu):
            counts.append(4)
            a = j * (nu + 1) + i
            indices += [a, a + 1, a + nu + 2, a + nu + 1]
    return Mesh(points, counts, indices, normals, [])


def mesh(p: Piece, table_top: float) -> Mesh:
    """Piece ``p``'s geometry, on a table whose top is at ``table_top``."""
    return {"sheet": _sheet_mesh, "cable": _cable_mesh, "bag": _bag_mesh}[p.shape](p, table_top)
