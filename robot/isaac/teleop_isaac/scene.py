"""The scene file (``core/Scenes/*.json``, docs/adr/0031 amendment 2), as Isaac reads it.

The same rules as Core's reader (``Teleop.RobotArm/World/WorldScene.cs``), in the same order, with the same keyword at the
start of every error, so the two agree on every case in ``core/Scenes/testdata/vectors.json``. The canonical parse of the
shipped scene is pinned by ``golden/scene-fr3-duo-bench.json``, which Core writes. Isaac also reads the sections Core
skips (``backdrops``, ``cameras``, ``lights``, docs/adr/0032); they are kept as given in :attr:`Scene.extra`.

Layouts are RobotHost's to draw; this module has no copy of the draw. It has Core's rests (docs/adr/0037 amendment 6): the
ways an object may rest on the table, and the footprint, height and pose of each; and raised clutter's footprint, the rectangle
of its sides at its yaw (docs/adr/0037 "Raised clutter"), all pinned by ``golden/scene-rests-jetrover-bench.json``, which
Core writes too. A mixed pool (docs/adr/0038 decision 3) adds roles, a plank's supports, steps of an object's own, and its rules:
rectangles, heights, and stands (``golden/scene-rules-jetrover-mixed.json``, which Core writes).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

FORMAT = "teleop-scene"
VERSION = 2  # the newest version this reader knows; it reads every version from 1. Version 2 adds movable objects.
MAX_OBJECTS = 32  # Core's WorldScene.MaxObjects: 32 since raised clutter joined the JetRover bench's 24 (docs/adr/0037)
MAX_MASS = 20.0  # kg
TOLERANCE = 0.001
PARK_X, PARK_Z = 17.0, -5.0  # where a parked object rests (docs/adr/0037 decision 3): Core's WorldScene.ParkX / ParkZ
_NAME = re.compile(r"^[A-Za-z0-9._-]{1,64}$")
_ASSET = re.compile(r"^[a-z0-9_]{1,64}/[a-z0-9_]{1,64}$")
ROLES = ("", "obstacle", "distractor", "stand", "plank", "support")  # docs/adr/0038 decision 3: a mixed pool's (Core's SceneRole)
TALL_HEIGHT = 0.15  # m: an obstacle this tall at its file size is a tall thin one (a rod, a dowel, a post): Core's WorldScene.TallHeight

# The rests (docs/adr/0037 amendment 6, Core's WorldScene.RestRotation): each a turn of the object's own frame before its yaw.
# 0 upright (its +z up), 1 its +y up (+90 degrees about x), 2 its +x up (-90 degrees about y), 3 upside down, 4 its -y up,
# 5 its -x up. Rests 1, 2, 4 and 5 lie on a side.
REST_UPRIGHT, REST_COUNT = 0, 6
LIE_ASPECT = 1.25  # a side rest a layout draws: the object lying on it at most this many times as tall as its narrower side
MIN_LYING_HEIGHT = 0.03  # m: and at least this tall, for a touch
_H = 0.70710678118654752
_REST_ROTATIONS = {1: (_H, _H, 0.0, 0.0), 2: (_H, 0.0, -_H, 0.0), 3: (0.0, 1.0, 0.0, 0.0), 4: (_H, -_H, 0.0, 0.0), 5: (_H, 0.0, _H, 0.0)}


def rest_rotation(rest: int) -> Tuple[float, float, float, float]:
    """The turn of rest ``rest``, a unit quaternion w, x, y, z (Core's ``WorldScene.RestRotation``)."""
    return _REST_ROTATIONS.get(rest, (1.0, 0.0, 0.0, 0.0))


def rest_of(q) -> Tuple[int, float]:
    """The rest of a rotation (w, x, y, z, or a pose x, y, z, w, x, y, z: its last four), the one whose up axis is nearest the
    vertical, and the yaw left once that rest is taken out, in (-pi, pi]: ``Scene.pose_at`` gives the pose back. As Core's
    ``WorldScene.RestOf``, operation for operation."""
    w, x, y, z = q[-4:]
    zx, zy, zz = 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)  # the world z of each of its axes
    ax, ay, az = abs(zx), abs(zy), abs(zz)
    if az >= ax and az >= ay:
        rest = 0 if zz >= 0 else 3
    elif ay >= ax:
        rest = 1 if zy >= 0 else 4
    else:
        rest = 2 if zx >= 0 else 5
    rw, rx, ry, rz = rest_rotation(rest)
    tw, tz = w * rw + x * rx + y * ry + z * rz, z * rw - w * rz - x * ry + y * rx  # q times the rest's inverse
    yaw = 2 * math.atan2(tz, tw)
    if yaw > math.pi:
        yaw -= 2 * math.pi
    elif yaw <= -math.pi:
        yaw += 2 * math.pi
    return rest, yaw


def _rects_apart(ax: float, ay: float, ac: float, asn: float, ahx: float, ahy: float,
                 bx: float, by: float, bc: float, bsn: float, bhx: float, bhy: float, gap: float) -> bool:
    """Whether two rectangles, each its centre, the cosine and sine of its turn, and its half sides, lie at least ``gap`` apart
    along one of their four sides' directions (the separating-axis test; touching counts as apart). As Core's
    ``WorldScene.RectsApart``, operation for operation."""
    dx, dy = bx - ax, by - ay
    for nx, ny in ((ac, asn), (-asn, ac), (bc, bsn), (-bsn, bc)):  # A's x, A's y, B's x, B's y
        pa = ahx * abs(nx * ac + ny * asn) + ahy * abs(-nx * asn + ny * ac)
        pb = bhx * abs(nx * bc + ny * bsn) + bhy * abs(-nx * bsn + ny * bc)
        if abs(nx * dx + ny * dy) - pa - pb >= gap - 1e-12:
            return True
    return False


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
    asset: str = ""  # a mesh's asset, library/name (docs/adr/0037 decision 4); empty for the other shapes
    # Raised desk clutter (docs/adr/0037 "Raised clutter"): a movable object RobotHost's layouts stand in a pass of their own,
    # anywhere in the region, flat as the file has it (never on a side); not one of the pool's movable objects.
    clutter: bool = False
    # docs/adr/0038 decision 3, a mixed pool's: what a layout does with it ("obstacle", "distractor", "stand", "plank", "support";
    # "" for any other scene), a plank's supports (their ids), and the resize steps a layout draws its size from ((min, max), or ()).
    role: str = ""
    supports: Tuple[int, ...] = ()
    steps: Tuple[int, ...] = ()

    @property
    def footprint_radius(self) -> float:
        sx, sy, _ = self.size
        return 0.5 * math.sqrt(sx * sx + sy * sy) if self.shape in ("box", "mesh") else 0.5 * sx

    def rest_extents(self, rest: int) -> Tuple[float, float, float]:
        """Its extents on rest ``rest`` before its yaw, at its file size: along x and y (its footprint), and up."""
        sx, sy, sz = self.size
        if rest % 3 == 1:
            return sx, sz, sy
        if rest % 3 == 2:
            return sz, sy, sx
        return sx, sy, sz

    def footprint_radius_at(self, rest: int) -> float:
        """The radius of a circle that holds its footprint on rest ``rest`` at any yaw, at its file size."""
        if rest % 3 == 0 or self.shape == "sphere":
            return self.footprint_radius
        ex, ey, _ = self.rest_extents(rest)
        return 0.5 * math.sqrt(ex * ex + ey * ey)

    def can_lie(self, rest: int) -> bool:
        """Whether a layout may lay it on side rest ``rest``: movable, stable on it and at least ``MIN_LYING_HEIGHT`` tall. Never
        clutter: it lies flat as the file has it."""
        if not 0 <= rest < REST_COUNT or rest % 3 == 0 or not self.movable or self.clutter or self.shape == "sphere":
            return False
        ex, ey, ez = self.rest_extents(rest)
        return ez <= LIE_ASPECT * min(ex, ey) + 1e-12 and ez >= MIN_LYING_HEIGHT - 1e-12

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
    # A pool (docs/adr/0037 decision 3): (movableMin, movableMax, fixedMin, fixedMax); all 0 when the scene is not one.
    pool: Tuple[int, int, int, int] = (0, 0, 0, 0)
    # Where a pool's fixed objects stand instead of the region (docs/adr/0037 amendment 5): flat (xMin, xMax, yMin, yMax, ...).
    fixed_regions: Tuple[float, ...] = ()
    # The robot's footprint no object's footprint may reach (xMin, xMax, yMin, yMax), or () (docs/adr/0037 amendment 5).
    robot_keep_out: Tuple[float, ...] = ()
    # docs/adr/0037 amendment 6, for RobotHost's layouts: the chance that a movable object in reach joins a cluster, that one
    # with a stable side lies on it, and that a layout stands one more by the bench's edge, in edge_regions (flat xMin, xMax,
    # yMin, yMax, ...: outside the region, out of reach). A movable object may stand in edge_regions (fits_on_table).
    cluster_share: float = 0.0
    lying_share: float = 0.0
    edge_regions: Tuple[float, ...] = ()
    edge_share: float = 0.0
    # docs/adr/0037 "Raised clutter": how many clutter objects a layout stands, (clutterMin, clutterMax); RobotHost draws them
    # last, anywhere in the region. Given a yaw, the placement rules take a piece of clutter as its own rectangle.
    clutter: Tuple[int, int] = (0, 0)
    # docs/adr/0038 decision 3, a mixed pool: how many obstacles and distractors a layout stands ((min, max) each), the obstacles'
    # bearing range (rad) and shares (raised on a stand, a plank on its supports), and where distractors stand (flat rectangles holding
    # their centres) no nearer the base's axis than distractor_reach (m). RobotHost draws the layouts; the rules here take every
    # footprint of a mixed pool as its rectangle, and nothing one above the other overlaps (overlaps_at_heights).
    obstacles: Tuple[int, int] = (0, 0)
    distractors: Tuple[int, int] = (0, 0)
    reach_bearing: float = 0.0
    stand_share: float = 0.0
    rig_share: float = 0.0
    distractor_regions: Tuple[float, ...] = ()
    distractor_reach: float = 0.0
    # docs/adr/0038, the scene's second pass, for RobotHost's layouts: the chance an obstacle is a tall thin one, the working band tall ones
    # and planks are centred in ((min, max) m from the base's axis, bearing rad), stands' tops as fractions of their object's narrower side
    # ((min, max): narrower than it, which then stands upright on it), and the range a rig's underside height is drawn from ((min, max) m).
    # All 0: as before. No rule here takes them; the placements they lead to keep every rule above.
    tall_share: float = 0.0
    work_band: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    stand_top: Tuple[float, float] = (0.0, 0.0)
    rig_under: Tuple[float, float] = (0.0, 0.0)

    def clear_of_robot(self, x: float, y: float, r: float) -> bool:
        if len(self.robot_keep_out) != 4:
            return True
        x0, x1, y0, y1 = self.robot_keep_out
        dx, dy = max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1)
        return dx * dx + dy * dy >= r * r - 1e-12

    @property
    def is_pool(self) -> bool:
        return self.pool[1] + self.pool[3] > 0 or self.mixed

    @property
    def mixed(self) -> bool:
        """A mixed pool (docs/adr/0038 decision 3): obstacles over the arm's reachable workspace, distractors beyond its reach."""
        return self.obstacles[1] + self.distractors[1] > 0

    def parked_pose(self, index: int, scale: float = 1.0) -> Tuple[float, ...]:
        """Where object ``index`` rests while parked: upright on the bridge's hidden shelf, as Core's ``WorldScene.ParkedPose``."""
        return (PARK_X, (index - len(self.objects) / 2.0) * 1.0, PARK_Z + self.objects[index].size[2] * scale / 2 + 0.001,
                1.0, 0.0, 0.0, 0.0)

    def _surface_bounds(self) -> Tuple[float, float, float, float]:
        """The table and the bench beside it, together: where a fixture's or an edge object's footprint may lie."""
        t, b = self.table, self.bench
        sx0, sx1 = (min(t.x_min, b.x_min), max(t.x_max, b.x_max)) if b else (t.x_min, t.x_max)
        sy0, sy1 = (max(t.y_min, b.y_min), min(t.y_max, b.y_max)) if b else (t.y_min, t.y_max)
        return sx0, sx1, sy0, sy1

    def in_edge_regions(self, x: float, y: float) -> bool:
        e = self.edge_regions
        return any(e[k] - 1e-9 <= x <= e[k + 1] + 1e-9 and e[k + 2] - 1e-9 <= y <= e[k + 3] + 1e-9 for k in range(0, len(e) - 3, 4))

    def fits_on_table(self, index: int, x: float, y: float, scale: float = 1.0, rest: int = REST_UPRIGHT,
                      yaw: Optional[float] = None) -> bool:
        """Whether object ``index``'s footprint on rest ``rest`` at ``scale``, centred at (x, y), lies on the table and in the
        region (a pool's fixed object: in fixed_regions; a movable one but clutter may also stand in edge_regions, its footprint on
        the table or the bench), clear of the robot's keep-out. As Core's ``WorldScene.FitsOnTable``. Given its ``yaw``, a piece of
        clutter (docs/adr/0037 "Raised clutter") is its own rectangle turned by it, not the circle that holds it at any yaw."""
        o = self.objects[index]
        if self.mixed:
            return self._fits_mixed(index, x, y, scale, rest, yaw)
        if yaw is not None and o.clutter:
            return self._clutter_fits(index, x, y, yaw, scale, rest)
        r = o.footprint_radius_at(rest) * scale
        t, (x0, x1, y0, y1) = self.table, self.region
        if not self.clear_of_robot(x, y, r):
            return False
        if self.fixed_regions and not o.movable:
            # Its centre in a rectangle, its footprint on the table or on the bench beside it (flush with it).
            sx0, sx1, sy0, sy1 = self._surface_bounds()
            if x - r < sx0 - 1e-9 or x + r > sx1 + 1e-9 or y - r < sy0 - 1e-9 or y + r > sy1 + 1e-9:
                return False
            f = self.fixed_regions
            return any(f[k] - 1e-9 <= x <= f[k + 1] + 1e-9 and f[k + 2] - 1e-9 <= y <= f[k + 3] + 1e-9 for k in range(0, len(f) - 3, 4))
        if (x - r >= max(t.x_min, x0) - 1e-9 and x + r <= min(t.x_max, x1) + 1e-9 and
                y - r >= max(t.y_min, y0) - 1e-9 and y + r <= min(t.y_max, y1) + 1e-9):
            return True
        if not o.movable or o.clutter or not self.in_edge_regions(x, y):
            return False
        sx0, sx1, sy0, sy1 = self._surface_bounds()
        return x - r >= sx0 - 1e-9 and x + r <= sx1 + 1e-9 and y - r >= sy0 - 1e-9 and y + r <= sy1 + 1e-9

    def overlaps(self, a: int, ax: float, ay: float, b: int, bx: float, by: float, scale_a: float = 1.0, scale_b: float = 1.0,
                 rest_a: int = REST_UPRIGHT, rest_b: int = REST_UPRIGHT, yaw_a: Optional[float] = None,
                 yaw_b: Optional[float] = None) -> bool:
        """Whether two objects' footprints, each on its rest, come closer than the clearance (Core's ``WorldScene.Overlaps``).
        Given both yaws, a piece of clutter (docs/adr/0037 "Raised clutter") is its own rectangle turned by its yaw: another
        object's circle may come no closer to it than the clearance, and two pieces lie the clearance apart along one of their
        four sides' directions. Without them, or for two objects that are not clutter, every object is its circle."""
        ca, cb = self.objects[a].clutter, self.objects[b].clutter
        if yaw_a is None or yaw_b is None or not (ca or cb):
            reach = (self.objects[a].footprint_radius_at(rest_a) * scale_a + self.objects[b].footprint_radius_at(rest_b) * scale_b
                     + self.clearance)
            return (ax - bx) ** 2 + (ay - by) ** 2 < reach * reach
        if ca and cb:
            hxa, hya = self._clutter_half(a, rest_a, scale_a)
            hxb, hyb = self._clutter_half(b, rest_b, scale_b)
            return not _rects_apart(ax, ay, math.cos(yaw_a), math.sin(yaw_a), hxa, hya,
                                    bx, by, math.cos(yaw_b), math.sin(yaw_b), hxb, hyb, self.clearance)
        # One piece of clutter and one circle: how near the circle's centre comes to the rectangle, in the rectangle's frame.
        rect, circle = (a, b) if ca else (b, a)
        rx, ry, ryaw, cx, cy = (ax, ay, yaw_a, bx, by) if ca else (bx, by, yaw_b, ax, ay)
        hx, hy = self._clutter_half(rect, rest_a if ca else rest_b, scale_a if ca else scale_b)
        c, s, dx, dy = math.cos(ryaw), math.sin(ryaw), cx - rx, cy - ry
        qx, qy = max(abs(dx * c + dy * s) - hx, 0.0), max(abs(-dx * s + dy * c) - hy, 0.0)
        reach = self.objects[circle].footprint_radius_at(rest_b if ca else rest_a) * (scale_b if ca else scale_a) + self.clearance
        return qx * qx + qy * qy < reach * reach

    def _clutter_half(self, index: int, rest: int, scale: float) -> Tuple[float, float]:
        """Half the sides of object ``index``'s footprint on rest ``rest`` at ``scale``, along its own x and y."""
        ex, ey, _ = self.objects[index].rest_extents(rest)
        return ex * scale / 2, ey * scale / 2

    def _clutter_fits(self, index: int, x: float, y: float, yaw: float, scale: float, rest: int) -> bool:
        """A piece of clutter's rectangle turned by ``yaw``: inside the region on the table, clear of the robot's keep-out (Core's
        ``WorldScene.FitsOnTable`` with a yaw)."""
        hx, hy = self._clutter_half(index, rest, scale)
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        t, (x0, x1, y0, y1) = self.table, self.region
        if not (x - bx >= max(t.x_min, x0) - 1e-9 and x + bx <= min(t.x_max, x1) + 1e-9 and
                y - by >= max(t.y_min, y0) - 1e-9 and y + by <= min(t.y_max, y1) + 1e-9):
            return False
        if len(self.robot_keep_out) != 4:
            return True
        k0, k1, k2, k3 = self.robot_keep_out
        return _rects_apart(x, y, c, s, hx, hy, (k0 + k1) / 2, (k2 + k3) / 2, 1.0, 0.0, (k1 - k0) / 2, (k3 - k2) / 2, 0.0)

    # --- docs/adr/0038 decision 3: a mixed pool ------------------------------------------------------------------------------

    def in_steps(self, index: int, scale: float) -> bool:
        """Whether ``scale`` is one of object ``index``'s own steps (Core's ``WorldScene.InSteps``)."""
        o = self.objects[index]
        if not o.steps or not is_valid_scale(scale):
            return False
        return o.steps[0] <= step_of(scale) <= o.steps[1]

    def resting_z(self, index: int, scale: float = 1.0, rest: int = REST_UPRIGHT) -> float:
        """The height of object ``index``'s centre resting on the table on rest ``rest`` at ``scale``."""
        return self.table.top + self.objects[index].rest_extents(rest)[2] * scale / 2

    def is_elevated(self, index: int, z: float, scale: float = 1.0, rest: int = REST_UPRIGHT) -> bool:
        """Whether a placement with its centre at height ``z`` stands object ``index`` above the table (only in a mixed pool, more than
        ``TOLERANCE`` up), as Core's ``WorldScene.IsElevated``. The rules take it only from RobotHost's own layouts."""
        return self.mixed and z > self.resting_z(index, scale, rest) + TOLERANCE

    def holds_on_top(self, k: int, kx: float, ky: float, kz: float, kyaw: float, kscale: float, krest: int,
                     index: int, x: float, y: float, z: float, scale: float, rest: int) -> bool:
        """Whether stand ``k`` at (kx, ky, kz), turned ``kyaw``, holds object ``index`` at (x, y, z): its bottom on the stand's top
        within 2 mm, its centre over that top (Core's ``WorldScene.HoldsOnTop``)."""
        if self.objects[k].role != "stand":
            return False
        kex, key, kez = self.objects[k].rest_extents(krest)
        ez = self.objects[index].rest_extents(rest)[2]
        if abs(kz + kez * kscale / 2 - (z - ez * scale / 2)) > 2 * TOLERANCE:
            return False
        c, s, dx, dy = math.cos(kyaw), math.sin(kyaw), x - kx, y - ky
        return abs(dx * c + dy * s) <= kex * kscale / 2 + 1e-9 and abs(-dx * s + dy * c) <= key * kscale / 2 + 1e-9

    def overlaps_at_heights(self, a: int, ax: float, ay: float, az: float, yaw_a: float, b: int, bx: float, by: float, bz: float,
                            yaw_b: float, scale_a: float = 1.0, scale_b: float = 1.0, rest_a: int = REST_UPRIGHT,
                            rest_b: int = REST_UPRIGHT) -> bool:
        """``overlaps`` with each object's centre height (Core's ``WorldScene.OverlapsAtHeights``): in a mixed pool every footprint is
        its rectangle and two objects one above the other (touching within ``TOLERANCE``, or apart) never overlap; any other pool
        ignores the heights."""
        if not self.mixed:
            return self.overlaps(a, ax, ay, b, bx, by, scale_a, scale_b, rest_a, rest_b, yaw_a, yaw_b)
        eax, eay, eaz = self.objects[a].rest_extents(rest_a)
        ebx, eby, ebz = self.objects[b].rest_extents(rest_b)
        ha, hb = eaz * scale_a / 2, ebz * scale_b / 2
        if az - ha >= bz + hb - TOLERANCE or bz - hb >= az + ha - TOLERANCE:
            return False
        return not _rects_apart(ax, ay, math.cos(yaw_a), math.sin(yaw_a), eax * scale_a / 2, eay * scale_a / 2,
                                bx, by, math.cos(yaw_b), math.sin(yaw_b), ebx * scale_b / 2, eby * scale_b / 2, self.clearance)

    def is_tall(self, index: int) -> bool:
        """Whether object ``index`` is one of a mixed pool's tall thin obstacles (Core's ``WorldScene.IsTall``)."""
        o = self.objects[index]
        return o.role == "obstacle" and o.size[2] >= TALL_HEIGHT - 1e-12

    def in_distractor_regions(self, x: float, y: float) -> bool:
        d = self.distractor_regions
        return any(d[k] - 1e-9 <= x <= d[k + 1] + 1e-9 and d[k + 2] - 1e-9 <= y <= d[k + 3] + 1e-9 for k in range(0, len(d) - 3, 4))

    def _fits_mixed(self, index: int, x: float, y: float, scale: float, rest: int, yaw: Optional[float]) -> bool:
        """A mixed pool's rule (Core's ``WorldScene.FitsMixed``): on the table or the bench, clear of the robot's keep-out; a distractor
        centred in its regions no nearer the base's axis than ``distractor_reach``, anything else in the placement region. Its
        footprint is the circle that holds it at any yaw, or given ``yaw`` the rectangle of its sides turned by it."""
        o = self.objects[index]
        sx0, sx1, sy0, sy1 = self._surface_bounds()
        distractor = o.role == "distractor"
        if yaw is None:
            r = o.footprint_radius_at(rest) * scale
            if not self.clear_of_robot(x, y, r):
                return False
            if distractor:
                return (self.in_distractor_regions(x, y) and x - r >= sx0 - 1e-9 and x + r <= sx1 + 1e-9 and y - r >= sy0 - 1e-9
                        and y + r <= sy1 + 1e-9 and math.sqrt(x * x + y * y) - r >= self.distractor_reach - 1e-9)
            x0, x1, y0, y1 = self.region
            return (x - r >= max(sx0, x0) - 1e-9 and x + r <= min(sx1, x1) + 1e-9 and y - r >= max(sy0, y0) - 1e-9
                    and y + r <= min(sy1, y1) + 1e-9)
        hx, hy = self._clutter_half(index, rest, scale)
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        if distractor:
            x0, x1, y0, y1 = sx0, sx1, sy0, sy1
        else:
            x0, x1, y0, y1 = max(sx0, self.region[0]), min(sx1, self.region[1]), max(sy0, self.region[2]), min(sy1, self.region[3])
        if not (x - bx >= x0 - 1e-9 and x + bx <= x1 + 1e-9 and y - by >= y0 - 1e-9 and y + by <= y1 + 1e-9):
            return False
        if distractor:
            px, py = -x * c - y * s, x * s - y * c  # the base's axis in the rectangle's frame, clamped to it
            qx, qy = max(abs(px) - hx, 0.0), max(abs(py) - hy, 0.0)
            if not self.in_distractor_regions(x, y) or math.sqrt(qx * qx + qy * qy) < self.distractor_reach - 1e-9:
                return False
        if len(self.robot_keep_out) != 4:
            return True
        k0, k1, k2, k3 = self.robot_keep_out
        return _rects_apart(x, y, c, s, hx, hy, (k0 + k1) / 2, (k2 + k3) / 2, 1.0, 0.0, (k1 - k0) / 2, (k3 - k2) / 2, 0.0)

    def clamp_on_table(self, index: int, x: float, y: float, scale: float = 1.0) -> Optional[Tuple[float, float]]:
        """The nearest point to (x, y) where object ``index`` at ``scale`` fits on the table and in the region (1 mm in), as
        Core's ``WorldScene.TryClampOnTable``; None when it fits nowhere."""
        r = self.objects[index].footprint_radius * scale + 0.001
        t, (rx0, rx1, ry0, ry1) = self.table, self.region
        x0, x1 = max(t.x_min, rx0) + r, min(t.x_max, rx1) - r
        y0, y1 = max(t.y_min, ry0) + r, min(t.y_max, ry1) - r
        if x0 > x1 or y0 > y1:
            return None
        return min(max(x, x0), x1), min(max(y, y0), y1)

    def spots_near(self, index: int, x: float, y: float, scale: float = 1.0, step: float = 0.01) -> List[Tuple[float, float]]:
        """Every point of a ``step`` grid where object ``index`` at ``scale`` fits on the table and in the region, nearest
        (x, y) first, the clamped point itself first of all: where a resize looks for room (docs/adr/0034 amendment 7)."""
        at = self.clamp_on_table(index, x, y, scale)
        if at is None:
            return []
        r = self.objects[index].footprint_radius * scale + 0.001
        t, (rx0, rx1, ry0, ry1) = self.table, self.region
        x0, x1 = max(t.x_min, rx0) + r, min(t.x_max, rx1) - r
        y0, y1 = max(t.y_min, ry0) + r, min(t.y_max, ry1) - r
        nx, ny = int((x1 - x0) / step) + 1, int((y1 - y0) / step) + 1
        grid = [(x0 + i * step, y0 + j * step) for i in range(nx) for j in range(ny)]
        grid.sort(key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
        return [at] + grid

    def pose_at(self, index: int, x: float, y: float, yaw: float, scale: float = 1.0, rest: int = REST_UPRIGHT) -> Tuple[float, ...]:
        """Object ``index`` resting on the table at (x, y) on rest ``rest`` (upright unless given), then turned ``yaw`` about the
        vertical, resized by ``scale``, its box's lowest face on the table top: x, y, z, qw, qx, qy, qz. As Core's
        ``WorldScene.PoseAt``."""
        if rest == REST_UPRIGHT:
            return (x, y, self.table.top + self.objects[index].size[2] * scale / 2, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        rw, rx, ry, rz = rest_rotation(rest)
        c, s = math.cos(yaw / 2), math.sin(yaw / 2)
        ez = self.objects[index].rest_extents(rest)[2]
        return (x, y, self.table.top + ez * scale / 2, c * rw - s * rz, c * rx - s * ry, c * ry + s * rx, c * rz + s * rw)


# The sizes the headset may give an object (Core's WorldScene resize steps, docs/adr/0034 amendment 6): 1.25 to the power
# -3 to 5. Isaac builds a body for each and swaps them: a collider cannot change while PhysX simulates.
RESIZE_STEP, MIN_RESIZE_STEP, MAX_RESIZE_STEP = 1.25, -3, 8
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


def _share(v: Any, name: str, is_pool: bool) -> float:
    """A pool's chance (docs/adr/0037 amendment 6), as Core's ``WorldScene.Share``."""
    if not _finite(v) or not 0 <= v <= 1:
        raise _bad("pool", f"{name} is a chance from 0 to 1")
    if v > 0 and not is_pool:
        raise _bad("pool", f"{name} is only for a pool")
    return float(v)


def _mixed_share(v: Any, name: str, mixed: bool) -> float:
    """A mixed pool's chance (docs/adr/0038), as Core's ``WorldScene.MixedShare``."""
    if not _finite(v) or not 0 <= v <= 1:
        raise _bad("pool", f"{name} is a chance from 0 to 1")
    if v > 0 and not mixed:
        raise _bad("pool", f"{name} is only for a mixed pool")
    return float(v)


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
        if shape not in ("box", "cylinder", "sphere", "mesh"):
            raise _bad("shape", f"'{oname}' has shape '{shape}': box, cylinder, sphere or mesh")
        asset = o.get("asset", "")
        if not isinstance(asset, str) or (not _ASSET.match(asset) if shape == "mesh" else asset != ""):
            raise _bad("asset", f"mesh '{oname}' needs an asset, library/name in lowercase letters, digits and '_'" if shape == "mesh"
                       else f"'{oname}' is a {shape}: only a mesh has an asset")
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
        # docs/adr/0037 "Raised clutter": clutter is something the arm can push about, so it is a movable body.
        clutter = bool(o.get("clutter", False))
        if clutter and not movable:
            raise _bad("kind", f"'{oname}' is clutter, which is movable, not '{kind}'")
        # docs/adr/0038 decision 3: a mixed pool's roles, a plank's supports and the steps a layout draws a size from.
        role = o.get("role", "")
        role = "" if role is None else role
        if role not in ROLES:
            raise _bad("role", f"'{oname}' has role '{role}': obstacle, distractor, stand, plank or support")
        if role in ("stand", "plank", "support") and (movable or shape != "box"):
            raise _bad("role", f"'{oname}' is a {role}, which is a fixed box")
        sup = o.get("supports", []) or []
        if (len(sup) < 1 or len(sup) > 2) if role == "plank" else len(sup) > 0:
            raise _bad("role", f"plank '{oname}' needs one or two supports" if role == "plank"
                       else f"'{oname}' is no plank: only a plank has supports")
        for sid in sup:
            if not 1 <= sid <= 254:
                raise _bad("role", f"plank '{oname}' names support {sid}; ids run from 1 to 254")
        st = o.get("steps", []) or []
        if len(st) not in (0, 2):
            raise _bad("steps", f"'{oname}': steps is [min, max]")
        if len(st) == 2 and not (MIN_RESIZE_STEP <= st[0] <= st[1] <= MAX_RESIZE_STEP):
            raise _bad("steps", f"'{oname}': steps run from {MIN_RESIZE_STEP} to {MAX_RESIZE_STEP}, min <= max")
        if len(st) == 2 and (shape not in ("box", "cylinder") or clutter):
            raise _bad("steps", f"'{oname}': only a box or a cylinder that is not clutter takes steps")
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
                              float(o.get("staticFriction", 0.0)), float(o.get("dynamicFriction", 0.0)), col, j, asset, clutter,
                              role, tuple(int(v) for v in sup), tuple(int(v) for v in st)))

    pd = doc.get("pool", {}) or {}
    pool = tuple(pd.get(k, 0) for k in ("movableMin", "movableMax", "fixedMin", "fixedMax"))
    clutter_range = tuple(pd.get(k, 0) for k in ("clutterMin", "clutterMax"))
    mixed_counts = tuple(pd.get(k, 0) for k in ("obstacleMin", "obstacleMax", "distractorMin", "distractorMax"))  # docs/adr/0038
    size_min, size_max = pd.get("sizeMin", 0.0), pd.get("sizeMax", 0.0)
    reach_min, reach_max = pd.get("reachMin", 0.0), pd.get("reachMax", 0.0)
    counts = pool + clutter_range + mixed_counts
    if (not all(isinstance(v, int) and not isinstance(v, bool) for v in counts) or min(counts) < 0 or pool[0] > pool[1] or pool[2] > pool[3]
            or clutter_range[0] > clutter_range[1] or mixed_counts[0] > mixed_counts[1] or mixed_counts[2] > mixed_counts[3]):
        raise _bad("pool", "each kind needs 0 <= min <= max")
    # Clutter (docs/adr/0037 "Raised clutter") is not one of the pool's movable objects.
    clutters = sum(1 for o in objects if o.clutter)
    movables = sum(1 for o in objects if o.movable and not o.clutter)
    fixeds = len(objects) - movables - clutters
    if pool[1] > movables or pool[3] > fixeds:
        raise _bad("pool", f"it asks for up to {pool[1]} movable and {pool[3]} fixed objects; the scene has {movables} and {fixeds}")
    if clutter_range[1] > clutters:
        raise _bad("pool", f"it asks for up to {clutter_range[1]} clutter objects; the scene has {clutters}")
    mixed = mixed_counts[1] + mixed_counts[3] > 0  # docs/adr/0038 decision 3
    is_pool = pool[1] + pool[3] > 0 or mixed
    if is_pool and version < 2:
        raise _bad("pool", "a pool needs version 2")
    if (clutters > 0 or clutter_range[1] > 0) and not is_pool:
        raise _bad("pool", "clutter is only for a pool, whose layouts choose it")
    if not (_finite(size_min) and _finite(size_max)) or not ((size_min == 0 and size_max == 0) or 0 < size_min < size_max):
        raise _bad("pool", "sizeMin and sizeMax are both 0, or 0 < sizeMin < sizeMax")
    if size_max > 0 and not is_pool:
        raise _bad("pool", "sizes are drawn only for a pool")
    if not (_finite(reach_min) and _finite(reach_max)) or not ((reach_min == 0 and reach_max == 0) or 0 <= reach_min < reach_max):
        raise _bad("pool", "reachMin and reachMax are both 0, or 0 <= reachMin < reachMax")
    if reach_max > 0 and not is_pool:
        raise _bad("pool", "a reach band is only for a pool")
    knock = pd.get("knockMaxMass", 0.0)
    if not _finite(knock) or not 0 <= knock <= MAX_MASS:
        raise _bad("pool", f"knockMaxMass is 0 (no roles) to {MAX_MASS:g} kg")
    if knock > 0 and not is_pool:
        raise _bad("pool", "roles are only for a pool")
    fr = pd.get("fixedRegions", []) or []
    if not isinstance(fr, list) or len(fr) % 4 != 0:
        raise _bad("pool", "fixedRegions holds xMin, xMax, yMin, yMax for each rectangle")
    for k in range(0, len(fr), 4):
        a, b, c, d = fr[k:k + 4]
        if not all(_finite(v) for v in (a, b, c, d)) or not (a < b and c < d):
            raise _bad("pool", "each fixedRegions rectangle needs min < max on both axes")
        sx0 = min(table.x_min, bench.x_min) if bench else table.x_min
        sx1 = max(table.x_max, bench.x_max) if bench else table.x_max
        if b < sx0 or a > sx1 or d < table.y_min or c > table.y_max:
            raise _bad("pool", "a fixedRegions rectangle does not overlap the table or the bench")
    if fr and not is_pool:
        raise _bad("pool", "fixedRegions are only for a pool")
    ko = pd.get("robotKeepOut", []) or []
    if ko and (not isinstance(ko, list) or len(ko) != 4 or not all(_finite(v) for v in ko) or not (ko[0] < ko[1] and ko[2] < ko[3])):
        raise _bad("pool", "robotKeepOut is [xMin, xMax, yMin, yMax] with min < max, or absent")
    near = pd.get("fixedNearMax", 0.0)
    if not _finite(near) or near < 0:
        raise _bad("pool", "fixedNearMax is 0 (no bound) or a distance in metres")
    # docs/adr/0037 amendment 6, in Core's order.
    cluster_share = _share(pd.get("clusterShare", 0.0), "clusterShare", is_pool)
    lying_share = _share(pd.get("lyingShare", 0.0), "lyingShare", is_pool)
    er = pd.get("edgeRegions", []) or []
    if not isinstance(er, list) or len(er) % 4 != 0:
        raise _bad("pool", "edgeRegions holds xMin, xMax, yMin, yMax for each rectangle")
    for k in range(0, len(er), 4):
        a, b, c, d = er[k:k + 4]
        if not all(_finite(v) for v in (a, b, c, d)) or not (a < b and c < d):
            raise _bad("pool", "each edgeRegions rectangle needs min < max on both axes")
        sx0 = min(table.x_min, bench.x_min) if bench else table.x_min
        sx1 = max(table.x_max, bench.x_max) if bench else table.x_max
        if b < sx0 or a > sx1 or d < table.y_min or c > table.y_max:
            raise _bad("pool", "an edgeRegions rectangle does not overlap the table or the bench")
        if a < x1 and b > x0 and c < y1 and d > y0:
            raise _bad("pool", "an edgeRegions rectangle overlaps the placement region: edge objects stand outside it, out of reach")
    if er and not is_pool:
        raise _bad("pool", "edgeRegions are only for a pool")
    edge_share = _share(pd.get("edgeShare", 0.0), "edgeShare", is_pool)
    if edge_share > 0 and not er:
        raise _bad("pool", "edgeShare needs edgeRegions")
    # docs/adr/0038 decision 3, a mixed pool, in Core's order.
    if mixed and (pool[1] > 0 or pool[3] > 0 or clutter_range[1] > 0):
        raise _bad("pool", "a mixed pool draws obstacles and distractors: its movable, fixed and clutter counts are 0")
    if mixed and (cluster_share > 0 or edge_share > 0 or fr or er or size_max > 0):
        raise _bad("pool", "a mixed pool has no clusters, edge objects, fixed regions or pool sizes")
    bearing = pd.get("reachBearing", 0.0)
    if not _finite(bearing) or not 0 <= bearing <= math.pi:
        raise _bad("pool", "reachBearing is 0 to pi rad")
    if bearing > 0 and not mixed:
        raise _bad("pool", "reachBearing is only for a mixed pool")
    if mixed_counts[1] > 0 and not (reach_max > 0 and bearing > 0):
        raise _bad("pool", "obstacles need reachMin, reachMax and reachBearing")
    stand_share = _mixed_share(pd.get("standShare", 0.0), "standShare", mixed)
    rig_share = _mixed_share(pd.get("rigShare", 0.0), "rigShare", mixed)
    # docs/adr/0038, the scene's second pass, in Core's order: tall obstacles more often, them and the planks in the arm's working band,
    # stands narrower than what they hold, and rigs' heights uniform over a range.
    tall_share = _mixed_share(pd.get("tallShare", 0.0), "tallShare", mixed)
    work_band = tuple(pd.get(k, 0.0) for k in ("workMin", "workMax", "workBearing"))
    work = any(v != 0 for v in work_band)
    if work and not mixed:
        raise _bad("pool", "a work band (workMin, workMax, workBearing) is only for a mixed pool")
    if work and not (all(_finite(v) for v in work_band) and reach_min <= work_band[0] < work_band[1] <= reach_max
                     and 0 < work_band[2] <= bearing):
        raise _bad("pool", "a work band lies within the obstacles' band: reachMin <= workMin < workMax <= reachMax and 0 < workBearing <= reachBearing")
    stand_top = (pd.get("standTopMin", 0.0), pd.get("standTopMax", 0.0))
    narrow = any(v != 0 for v in stand_top)
    if narrow and not mixed:
        raise _bad("pool", "standTopMin and standTopMax are only for a mixed pool")
    if narrow and not (all(_finite(v) for v in stand_top) and 0 < stand_top[0] <= stand_top[1] <= 1):
        raise _bad("pool", "standTopMin and standTopMax are both 0, or 0 < standTopMin <= standTopMax <= 1")
    rig_under = (pd.get("rigUnderMin", 0.0), pd.get("rigUnderMax", 0.0))
    under = any(v != 0 for v in rig_under)
    if under and not mixed:
        raise _bad("pool", "rigUnderMin and rigUnderMax are only for a mixed pool")
    if under and not (all(_finite(v) for v in rig_under) and 0 < rig_under[0] < rig_under[1]):
        raise _bad("pool", "rigUnderMin and rigUnderMax are both 0, or 0 < rigUnderMin < rigUnderMax")
    dr = pd.get("distractorRegions", []) or []
    if not isinstance(dr, list) or len(dr) % 4 != 0:
        raise _bad("pool", "distractorRegions holds xMin, xMax, yMin, yMax for each rectangle")
    for k in range(0, len(dr), 4):
        a, b, c, d = dr[k:k + 4]
        if not all(_finite(v) for v in (a, b, c, d)) or not (a < b and c < d):
            raise _bad("pool", "each distractorRegions rectangle needs min < max on both axes")
        sx0 = min(table.x_min, bench.x_min) if bench else table.x_min
        sx1 = max(table.x_max, bench.x_max) if bench else table.x_max
        if b < sx0 or a > sx1 or d < table.y_min or c > table.y_max:
            raise _bad("pool", "a distractorRegions rectangle does not overlap the table or the bench")
    if dr and not mixed:
        raise _bad("pool", "distractorRegions are only for a mixed pool")
    far = pd.get("distractorReach", 0.0)
    if not _finite(far) or far < 0:
        raise _bad("pool", "distractorReach is 0 or a distance in metres")
    if far > 0 and not mixed:
        raise _bad("pool", "distractorReach is only for a mixed pool")
    if mixed_counts[3] > 0 and (not dr or not far > 0):
        raise _bad("pool", "distractors need distractorRegions and distractorReach")
    if far > 0 and far < reach_max:
        raise _bad("pool", "distractorReach lies within the obstacles' band (reachMax)")
    by_id = {o.id: o for o in objects}
    n_obstacles = n_distractors = n_planks = 0
    for o in objects:
        if mixed and not o.role:
            raise _bad("role", f"'{o.name}' has no role: every object of a mixed pool has one")
        if not mixed and o.role:
            raise _bad("role", f"'{o.name}' has a role: roles are for a mixed pool")
        if not mixed and o.steps:
            raise _bad("steps", f"'{o.name}' has steps: they are for a mixed pool's layouts")
        n_obstacles += o.role == "obstacle"
        n_distractors += o.role == "distractor"
        if o.role != "plank":
            continue
        n_planks += 1
        for sid in o.supports:
            if sid not in by_id:
                raise _bad("role", f"plank '{o.name}' names support {sid}, which is no object of the scene")
            if by_id[sid].role != "support":
                raise _bad("role", f"plank '{o.name}' names '{by_id[sid].name}', which is no support")
            if sum(q.supports.count(sid) for q in objects if q.role == "plank") > 1:
                raise _bad("role", f"support '{by_id[sid].name}' holds up two planks, or one twice")
        if len(o.supports) == 2:
            left, right = by_id[o.supports[0]], by_id[o.supports[1]]
            if any(abs(u - v) > 1e-9 for u, v in zip(left.size, right.size)) or left.steps != right.steps:
                raise _bad("role", f"plank '{o.name}''s two supports differ in size or steps: they hold it level")
    for o in objects:
        if o.role == "support" and not any(q.role == "plank" and o.id in q.supports for q in objects):
            raise _bad("role", f"support '{o.name}' holds up no plank")
    if mixed_counts[1] > n_obstacles + n_planks:
        raise _bad("pool", f"it asks for up to {mixed_counts[1]} obstacles; the scene has {n_obstacles} and {n_planks} planks")
    if mixed_counts[3] > n_distractors:
        raise _bad("pool", f"it asks for up to {mixed_counts[3]} distractors; the scene has {n_distractors}")
    known = {"format", "version", "name", "robotProfile", "surfaces", "objects", "placement", "pool"}
    scene = Scene(name, robot_profile, tuple(surfaces), table, bench, tuple(objects), (float(x0), float(x1), float(y0), float(y1)),
                  float(clearance), {k: v for k, v in doc.items() if k not in known}, pool, tuple(float(v) for v in fr),
                  tuple(float(v) for v in ko), cluster_share=cluster_share, lying_share=lying_share,
                  edge_regions=tuple(float(v) for v in er), edge_share=edge_share, clutter=clutter_range,
                  obstacles=(mixed_counts[0], mixed_counts[1]), distractors=(mixed_counts[2], mixed_counts[3]),
                  reach_bearing=float(bearing), stand_share=stand_share, rig_share=rig_share,
                  distractor_regions=tuple(float(v) for v in dr), distractor_reach=float(far), tall_share=tall_share,
                  work_band=tuple(float(v) for v in work_band), stand_top=tuple(float(v) for v in stand_top),
                  rig_under=tuple(float(v) for v in rig_under))
    for i, o in enumerate(objects):
        for sx in (-1, 0, 1):
            for sy in (-1, 0, 1):
                # A mixed pool's nominal poses are checked at their yaw, by the rectangles its layouts use (docs/adr/0038).
                if not scene.fits_on_table(i, o.position[0] + sx * o.jitter[0], o.position[1] + sy * o.jitter[1],
                                           yaw=o.yaw if mixed else None):
                    raise _bad("fits", f"'{o.name}' leaves the table or the placement region at its nominal pose or the edge of its jitter")
        for k in range(i if not is_pool else 0):  # a pool's objects start parked: their nominal poses may share the table
            if scene.overlaps(i, o.position[0], o.position[1], k, objects[k].position[0], objects[k].position[1]):
                raise _bad("overlap", f"'{o.name}' and '{objects[k].name}' are closer than the clearance at their nominal poses")
    return scene


def load(path: str) -> Scene:
    import json
    with open(path, encoding="utf-8") as f:
        return parse(json.load(f))
