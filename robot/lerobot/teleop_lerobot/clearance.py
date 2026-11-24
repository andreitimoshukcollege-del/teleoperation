"""How far each of the arm's links is from the scene, frame by frame (docs/adr/0038 decision 4): the signed distance from each
link to the nearest obstacle and to the table, from the take's own records and the twin's collision geometry.

:func:`compute` reads a take (``takes.read_take``) and the geometry cache ``teleop_isaac.clearance_geometry`` writes (default
:data:`DEFAULT_GEOMETRY`, on the GPU machine), and gives a :class:`Clearance`: for every frame of ``take.frames``, per link
in the URDF's order, the signed distance in millimetres (negative: the two overlap) to the nearest obstacle with that
obstacle's id, and to the nearest surface. A take that lacks what it needs raises :class:`ClearanceUnavailable`.

**At a frame** (its ``stateSensed``, the time the frame shows):

- the links' poses are the forward kinematics of the frame's ``VQ`` joint values (URDF joint names, radians, a finger in
  metres), and each link's colliders are its URDF boxes as Isaac's importer makes them (the cache's ``robot-<robot>.json``);
- every object's pose is ``WO``'s, interpolated to the frame's time (position linearly, rotation along the shorter arc), its
  size the scene file's times its factor in force (``WZ``; 1 before the take's first);
- an **obstacle** is every scene object but the table, movable or fixed, clutter included, that is not parked (on the bridge's
  shelf, 5 m below the floor: :data:`PARKED_BELOW_Z`). A mesh object is its asset's convex pieces (the cache's
  ``assets/<library>/<name>.json``) scaled by its entry's size over the asset's and by its factor; a box or a cylinder is its
  primitive (a cylinder a 32-sided prism); a sphere is exact;
- the **surfaces** are the scene file's colliding surfaces (the table, the bench beside it and the floor: the arm stands on
  them all, and ``WC`` reports them all as object 0). A body no joint moves (``fixed``, the robot's base) stands in the
  bench and PhysX never collides it with a surface, so its surface distance is NaN.

**The distance** between a link box and a convex piece is exact: GJK's when they are apart, and the separating-axis test's
penetration depth (face normals of both and the cross products of their edges) when they overlap. A link's distance to a
surface box is that of its nearest box corner, exact wherever the surface's top face is the nearest part of it (everywhere
the arm reaches). Distances beyond :data:`CUTOFF_MM` are reported as the cutoff (and the obstacle as :data:`NO_OBSTACLE`):
far enough to see a near miss (under 30 mm) and most of an approach. Each frame looks only at what bounding spheres and
the box's own axes cannot rule out, nearest first, so a 300-frame take takes about a second.

Pure numpy, no other dependency.
"""

from __future__ import annotations

import bisect
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .jlog import JlogError, items, read_jlog

CUTOFF_MM = 150.0
DEFAULT_GEOMETRY = os.path.expanduser("~/isaacsim/assets/clearance")
NO_OBSTACLE = -1  # obstacle_id when nothing is within the cutoff
PARKED_BELOW_Z = -2.0  # m: a parked object rests on the bridge's hidden shelf, z about -5 m (WO; docs/adr/0037 decision 3)
CYLINDER_SIDES = 32
METHOD = ("signed distance, link boxes (URDF, as Isaac imports them) to each obstacle's colliders: GJK apart, the separating "
          "axis's penetration depth overlapping; mesh objects as the cache's convex pieces")


class ClearanceUnavailable(Exception):
    """The take lacks what the clearance needs (its records, its scene, or the geometry for one of its objects)."""


@dataclass(frozen=True)
class Clearance:
    """Per frame of ``take.frames``, per link of ``links``: the signed distance (mm, negative when overlapping, at most
    ``cutoff_mm``) to the nearest obstacle and its scene id (:data:`NO_OBSTACLE` when none is within the cutoff), and to the
    nearest surface (NaN for a body PhysX never collides with a surface). ``fixed`` names the links no joint moves: a
    reader's overall minimum leaves them out (docs/adr/0038 L)."""

    links: Tuple[str, ...]
    obstacle_mm: List[Tuple[float, ...]]
    obstacle_id: List[Tuple[int, ...]]
    surface_mm: List[Tuple[float, ...]]
    cutoff_mm: float
    method: str
    fixed: Tuple[str, ...] = ()  # the links no joint moves (the base): left out of every overall minimum


# --- convex pieces --------------------------------------------------------------------------------------------------------

@dataclass
class Piece:
    """A convex polytope: its vertices, its outward face planes (``normals`` and ``offsets``: ``n.x <= d`` inside), its edges'
    directions (unit, one per direction), and a bounding sphere."""

    vertices: np.ndarray
    normals: np.ndarray
    offsets: np.ndarray
    edges: np.ndarray
    center: np.ndarray
    radius: float

    def placed(self, rotation: np.ndarray, translation: np.ndarray, scale: float) -> "Piece":
        """The piece scaled by ``scale`` about its frame's origin, then turned and moved."""
        v = (self.vertices * scale) @ rotation.T + translation
        n = self.normals @ rotation.T
        return Piece(v, n, self.offsets * scale + n @ translation, self.edges @ rotation.T,
                     rotation @ (self.center * scale) + translation, self.radius * scale)


def _unique_directions(d: np.ndarray, decimals: int = 6) -> np.ndarray:
    """Unit directions, one per line (a direction and its opposite are one)."""
    if len(d) == 0:
        return d.reshape(0, 3)
    n = np.linalg.norm(d, axis=1)
    d = d[n > 1e-12] / n[n > 1e-12, None]
    first = np.argmax(np.abs(d) > 1e-9, axis=1)
    sign = np.sign(d[np.arange(len(d)), first])
    d = d * sign[:, None]
    _, keep = np.unique(np.round(d, decimals), axis=0, return_index=True)
    return d[np.sort(keep)]


def piece_from_mesh(vertices: Sequence[Sequence[float]], faces: Sequence[Sequence[int]]) -> Piece:
    """A convex piece from a closed convex mesh (outward triangles, as the cache stores each hull)."""
    v = np.asarray(vertices, float).reshape(-1, 3)
    f = np.asarray(faces, int).reshape(-1, 3)
    if len(v) < 4 or len(f) < 4:
        raise ValueError("a convex piece needs at least 4 vertices and 4 faces")
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-14
    n, a, f = n[ok] / norm[ok, None], a[ok], f[ok]
    d = np.einsum("ij,ij->i", n, a)
    planes = np.c_[n, d]
    _, keep = np.unique(np.round(planes, 7), axis=0, return_index=True)
    keep = np.sort(keep)
    edges = np.concatenate([v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 1]], v[f[:, 0]] - v[f[:, 2]]])
    lo, hi = v.min(0), v.max(0)
    centre = (lo + hi) / 2
    return Piece(v, planes[keep, :3], planes[keep, 3], _unique_directions(edges), centre,
                 float(np.linalg.norm(v - centre, axis=1).max()))


def box_piece(size: Sequence[float]) -> Piece:
    """A box of ``size`` centred on its frame's origin, axis-aligned."""
    hx, hy, hz = (s / 2 for s in size)
    v = np.array([[x, y, z] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)], float)
    eye = np.eye(3)
    return Piece(v, np.concatenate([eye, -eye]), np.array([hx, hy, hz, hx, hy, hz]), eye.copy(), np.zeros(3),
                 float(math.sqrt(hx * hx + hy * hy + hz * hz)))


def cylinder_piece(diameter: float, height: float, sides: int = CYLINDER_SIDES) -> Piece:
    """A cylinder along its frame's z, centred on the origin, as a prism of ``sides`` sides with its corners on the circle."""
    r, h = diameter / 2, height / 2
    ang = np.arange(sides) * 2 * math.pi / sides
    ring = np.c_[r * np.cos(ang), r * np.sin(ang)]
    v = np.concatenate([np.c_[ring, np.full(sides, -h)], np.c_[ring, np.full(sides, h)]])
    mid = ang + math.pi / sides
    side_n = np.c_[np.cos(mid), np.sin(mid), np.zeros(sides)]
    normals = np.concatenate([side_n, [[0, 0, 1], [0, 0, -1]]])
    offsets = np.concatenate([np.full(sides, r * math.cos(math.pi / sides)), [h, h]])
    edges = _unique_directions(np.concatenate([np.roll(v[:sides], -1, axis=0) - v[:sides], [[0, 0, 1]]]))
    return Piece(v, normals, offsets, edges, np.zeros(3), float(math.sqrt(r * r + h * h)))


# --- box against convex piece -----------------------------------------------------------------------------------------------

def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _segment(a, b):
    """The point of segment ab nearest the origin, and the vertices it needs."""
    ab = _sub(b, a)
    den = _dot(ab, ab)
    t = -_dot(a, ab) / den if den > 0 else 0.0
    if t <= 0:
        return a, [a]
    if t >= 1:
        return b, [b]
    return (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]), [a, b]


def _triangle(a, b, c):
    """The point of triangle abc nearest the origin, and the vertices it needs (Ericson's closest point on a triangle)."""
    ab, ac = _sub(b, a), _sub(c, a)
    d1, d2 = -_dot(ab, a), -_dot(ac, a)
    if d1 <= 0 and d2 <= 0:
        return a, [a]
    d3, d4 = -_dot(ab, b), -_dot(ac, b)
    if d3 >= 0 and d4 <= d3:
        return b, [b]
    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        t = d1 / (d1 - d3) if d1 != d3 else 0.0
        return (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]), [a, b]
    d5, d6 = -_dot(ab, c), -_dot(ac, c)
    if d6 >= 0 and d5 <= d6:
        return c, [c]
    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        t = d2 / (d2 - d6) if d2 != d6 else 0.0
        return (a[0] + t * ac[0], a[1] + t * ac[1], a[2] + t * ac[2]), [a, c]
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        den = (d4 - d3) + (d5 - d6)
        t = (d4 - d3) / den if den != 0 else 0.0
        bc = _sub(c, b)
        return (b[0] + t * bc[0], b[1] + t * bc[1], b[2] + t * bc[2]), [b, c]
    den = va + vb + vc
    if den == 0:
        return _segment(a, b) if _dot(ab, ab) >= _dot(ac, ac) else _segment(a, c)
    v, w = vb / den, vc / den
    return (a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w), [a, b, c]


def _tetrahedron(a, b, c, d):
    """The point of tetrahedron abcd nearest the origin and the vertices it needs, or (None, []) with the origin inside."""
    best, keep, best_dd = None, [], math.inf
    for p, q, r, s in ((a, b, c, d), (a, c, d, b), (a, d, b, c), (b, d, c, a)):
        n = _cross(_sub(q, p), _sub(r, p))
        side_o, side_s = -_dot(p, n), _dot(_sub(s, p), n)
        if side_o * side_s < 0 or abs(side_s) < 1e-30:  # the origin beyond this face (or the tetrahedron is flat)
            x, k = _triangle(p, q, r)
            dd = _dot(x, x)
            if dd < best_dd:
                best, keep, best_dd = x, k, dd
    return best, keep


def gjk(support_a, support_b, direction, iterations: int = 64) -> Tuple[float, bool]:
    """GJK's distance between two convex sets given their support functions (a direction -> the farthest point of the set
    that way): (distance, whether they overlap). Converges to within a micrometre."""
    def support(dx, dy, dz):
        p, q = support_a((dx, dy, dz)), support_b((-dx, -dy, -dz))
        return (p[0] - q[0], p[1] - q[1], p[2] - q[2])

    v = support(*direction)
    simplex = [v]
    vv = _dot(v, v)
    for _ in range(iterations):
        if vv <= 1e-20:
            return 0.0, True
        w = support(-v[0], -v[1], -v[2])
        if vv - _dot(v, w) <= 1e-9 * vv + 1e-18:
            break
        simplex.append(w)
        if len(simplex) == 2:
            v, simplex = _segment(*simplex)
        elif len(simplex) == 3:
            v, simplex = _triangle(*simplex)
        else:
            v, simplex = _tetrahedron(*simplex)
            if v is None:
                return 0.0, True
        new = _dot(v, v)
        if new >= vv:
            break
        vv = new
    return math.sqrt(vv), False


@dataclass
class Box:
    """A link's collision box in the world: its centre, its axes (rows, unit) and half its sides."""

    center: np.ndarray
    axes: np.ndarray
    half: np.ndarray

    def support(self):
        c, a, h = tuple(self.center), [tuple(r) for r in self.axes], tuple(self.half)

        def f(d):
            x, y, z = c
            for i in range(3):
                ai = a[i]
                s = h[i] if ai[0] * d[0] + ai[1] * d[1] + ai[2] * d[2] >= 0 else -h[i]
                x, y, z = x + s * ai[0], y + s * ai[1], z + s * ai[2]
            return (x, y, z)
        return f

    def corners(self) -> np.ndarray:
        s = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
        return self.center + (s * self.half) @ self.axes


def _points_support(vertices: np.ndarray):
    def f(d):
        return tuple(vertices[int(np.argmax(vertices @ np.asarray(d)))])
    return f


def penetration(box: Box, piece: Piece) -> float:
    """The separating-axis test's least overlap of a box and a convex piece (m; negative: a gap along some axis): their
    penetration depth when they overlap, exact for convex polytopes with every face normal and edge-edge axis."""
    cross = np.cross(box.axes[:, None, :], piece.edges[None, :, :]).reshape(-1, 3)
    n = np.linalg.norm(cross, axis=1)
    axes = np.concatenate([box.axes, piece.normals, cross[n > 1e-9] / n[n > 1e-9, None]])
    centre = axes @ box.center
    reach = np.abs(axes @ box.axes.T) @ box.half
    proj = piece.vertices @ axes.T
    lo, hi = proj.min(axis=0), proj.max(axis=0)
    return float(np.minimum(centre + reach - lo, hi - (centre - reach)).min())


def box_piece_distance(box: Box, piece: Piece) -> float:
    """The signed distance (m) between a box and a convex piece: apart, the distance between them (GJK); overlapping, minus
    the penetration depth."""
    d, overlap = gjk(box.support(), _points_support(piece.vertices), tuple(box.center - piece.center))
    if not overlap and d > 1e-7:
        return d
    return -penetration(box, piece)


def box_sphere_distance(box: Box, centre: np.ndarray, radius: float) -> float:
    return float(point_box_distance(centre[None, :], box.center, box.axes, box.half)[0]) - radius


def point_box_distance(points: np.ndarray, centre: np.ndarray, axes: np.ndarray, half: np.ndarray) -> np.ndarray:
    """Signed distance (m) from each point to a box (negative inside: minus the depth to its nearest face)."""
    q = np.abs((points - centre) @ axes.T) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


# --- the robot, its kinematics ------------------------------------------------------------------------------------------------

def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis: Sequence[float], angles: np.ndarray) -> np.ndarray:
    """Rotation matrices (n, 3, 3) about ``axis`` by each angle (Rodrigues)."""
    a = np.asarray(axis, float)
    a = a / (np.linalg.norm(a) or 1.0)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s, c = np.sin(angles)[:, None, None], np.cos(angles)[:, None, None]
    return np.eye(3)[None] + s * k[None] + (1 - c) * (k @ k)[None]


@dataclass
class Robot:
    name: str
    links: Tuple[str, ...]
    joints: List[Dict]
    fixed: frozenset
    box_link: np.ndarray  # each box's link index
    box_center: np.ndarray  # (b, 3), in its link's frame
    box_axes: np.ndarray  # (b, 3, 3), rows
    box_half: np.ndarray  # (b, 3)

    def link_poses(self, values: Dict[str, np.ndarray], n: int) -> Tuple[np.ndarray, np.ndarray]:
        """Every link's rotation (n, links, 3, 3) and position (n, links, 3) in the base frame for ``n`` sets of joint values
        (joint name -> (n,) values; a joint not given is 0): each link its parent's pose, times its joint's origin, times
        the joint's motion about or along its axis."""
        index = {name: i for i, name in enumerate(self.links)}
        R = np.zeros((n, len(self.links), 3, 3))
        p = np.zeros((n, len(self.links), 3))
        done = set()
        for i, name in enumerate(self.links):
            if not any(j["child"] == name for j in self.joints):
                R[:, i] = np.eye(3)
                done.add(name)
        pending = list(self.joints)
        while pending:
            progressed = False
            for j in list(pending):
                if j["parent"] not in done:
                    continue
                a, b = index[j["parent"]], index[j["child"]]
                origin = _rpy(*j["rpy"])
                Ra = R[:, a] @ origin
                p[:, b] = p[:, a] + R[:, a] @ np.asarray(j["xyz"], float)
                q = np.asarray(values.get(j["name"], np.zeros(n)), float)
                if j["type"] in ("revolute", "continuous"):
                    R[:, b] = Ra @ _axis_angle(j["axis"], q)
                else:
                    R[:, b] = Ra
                    if j["type"] == "prismatic":
                        axis = np.asarray(j["axis"], float)
                        p[:, b] += (Ra @ axis) * q[:, None]
                done.add(j["child"])
                pending.remove(j)
                progressed = True
            if not progressed:
                raise ClearanceUnavailable(f"the robot's joints do not reach {[j['child'] for j in pending]} from its base")
        return R, p

    def boxes(self, R: np.ndarray, p: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """The boxes' centres (b, 3) and axes (b, 3, 3) in the base frame, for one frame's link poses ``R`` (links, 3, 3) and
        ``p`` (links, 3)."""
        Rl, pl = R[self.box_link], p[self.box_link]
        centres = np.einsum("bij,bj->bi", Rl, self.box_center) + pl
        axes = np.einsum("bkj,bij->bki", self.box_axes, Rl)
        return centres, axes


_CACHE: Dict[Tuple[str, str], object] = {}


def load_robot(geometry_dir: str, name: str = "jetrover") -> Robot:
    key = (os.path.abspath(geometry_dir), "robot:" + name)
    if key in _CACHE:
        return _CACHE[key]
    path = os.path.join(geometry_dir, f"robot-{name}.json")
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except OSError as e:
        raise ClearanceUnavailable(f"no robot geometry at {path} (just isaac-clearance-geometry): {e}") from e
    links = tuple(doc["links"])
    index = {name_: i for i, name_ in enumerate(links)}
    boxes = doc["boxes"]
    robot = Robot(doc.get("robot", name), links, doc["joints"], frozenset(doc.get("fixed", [])),
                  np.array([index[b["link"]] for b in boxes], int), np.array([b["center"] for b in boxes], float),
                  np.array([b["axes"] for b in boxes], float), np.array([b["half"] for b in boxes], float))
    _CACHE[key] = robot
    return robot


def load_asset(geometry_dir: str, asset: str) -> Tuple[List[Piece], Tuple[float, float, float], str]:
    """A mesh asset's convex pieces in its body frame at k = 1, the asset's size (its manifest's box), and how the cache made
    them (its ``method``)."""
    key = (os.path.abspath(geometry_dir), "asset:" + asset)
    if key in _CACHE:
        return _CACHE[key]
    library, name = asset.split("/")
    path = os.path.join(geometry_dir, "assets", library, f"{name}.json")
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except OSError as e:
        raise ClearanceUnavailable(f"no geometry for {asset} at {path} (just isaac-clearance-geometry): {e}") from e
    pieces = [piece_from_mesh(p["vertices"], p["faces"]) for p in doc["pieces"]]
    out = (pieces, tuple(float(v) for v in doc["size"]), str(doc.get("method", "?")))
    _CACHE[key] = out
    return out


# --- the take's world ----------------------------------------------------------------------------------------------------

def _quat_matrix(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass
class Body:
    """A scene object's collider in its body frame at the scene file's size: convex pieces, or a sphere."""

    id: int
    name: str
    pieces: List[Piece]
    base_scale: float  # what scales the pieces to the scene file's size (a mesh: its entry's size over its asset's)
    sphere: float  # radius at the file's size for a sphere, else 0
    radius: float  # bounding radius about the body's origin at the file's size


def _bodies(scene: Dict, geometry_dir: str) -> List[Body]:
    out = []
    for o in scene["objects"]:
        size = [float(v) for v in o["size"]]
        shape = o.get("shape")
        if shape == "mesh":
            pieces, asset_size, _ = load_asset(geometry_dir, o["asset"])
            ratios = [e / a for e, a in zip(size, asset_size)]
            if not all(abs(r / ratios[2] - 1.0) <= 0.03 for r in ratios):
                raise ClearanceUnavailable(f"{o['name']}: its size {size} is not its asset's {list(asset_size)} scaled uniformly")
            k = ratios[2]  # world.mesh_factor
            radius = max(float(np.linalg.norm(p.vertices, axis=1).max()) for p in pieces) * k
            out.append(Body(int(o["id"]), o["name"], pieces, k, 0.0, radius))
        elif shape == "box":
            out.append(Body(int(o["id"]), o["name"], [box_piece(size)], 1.0, 0.0, 0.5 * math.sqrt(sum(s * s for s in size))))
        elif shape == "cylinder":
            out.append(Body(int(o["id"]), o["name"], [cylinder_piece(size[0], size[2])], 1.0, 0.0,
                            math.sqrt((size[0] / 2) ** 2 + (size[2] / 2) ** 2)))
        elif shape == "sphere":
            out.append(Body(int(o["id"]), o["name"], [], 1.0, size[0] / 2, size[0] / 2))
        else:
            raise ClearanceUnavailable(f"{o.get('name')}: shape {shape!r} has no collider here")
    return out


def _surfaces(scene: Dict) -> List[Tuple[str, Piece]]:
    out = []
    for s in scene.get("surfaces", []):
        if s.get("collides", True):
            box = box_piece(s["size"])
            out.append((s["id"], box.placed(np.eye(3), np.asarray(s["center"], float), 1.0)))
    return out


class _Poses:
    """Every object's ``WO`` poses in time order, interpolated between records."""

    def __init__(self, rows: Iterable[Sequence[str]]):
        self.t: Dict[int, List[int]] = {}
        self.p: Dict[int, List[Tuple[float, ...]]] = {}
        for r in sorted(rows, key=lambda r: int(r[1])):
            sensed = int(r[1])
            for it in items(r[3] if len(r) > 3 else ""):
                if len(it) < 8:
                    continue
                oid, pose = int(it[0]), tuple(float(v) for v in it[1:8])
                ts = self.t.setdefault(oid, [])
                if ts and ts[-1] == sensed:
                    self.p[oid][-1] = pose
                else:
                    ts.append(sensed)
                    self.p.setdefault(oid, []).append(pose)

    def at(self, oid: int, t: float) -> Optional[Tuple[float, ...]]:
        ts = self.t.get(oid)
        if not ts:
            return None
        i = bisect.bisect_right(ts, t) - 1
        if i < 0:
            return self.p[oid][0]
        if i + 1 >= len(ts):
            return self.p[oid][i]
        a, b = self.p[oid][i], self.p[oid][i + 1]
        if a[2] < PARKED_BELOW_Z or b[2] < PARKED_BELOW_Z:  # to or from the shelf: a teleport, not a motion
            return a
        f = (t - ts[i]) / (ts[i + 1] - ts[i])
        qa, qb = a[3:], b[3:]
        if sum(x * y for x, y in zip(qa, qb)) < 0:
            qb = tuple(-x for x in qb)
        q = tuple(x + f * (y - x) for x, y in zip(qa, qb))
        return tuple(x + f * (y - x) for x, y in zip(a[:3], b[:3])) + q


class _Sizes:
    """Each object's size factor in force at a time: ``WZ`` (the backend's), else the renderer's first ``SZ``, else 1."""

    def __init__(self, wz: Sequence[Sequence[str]], sz: Sequence[Sequence[str]]):
        rows = sorted(((int(r[0]), {int(it[0]): float(it[1]) for it in items(r[2] if len(r) > 2 else "") if len(it) >= 2}) for r in wz),
                      key=lambda x: x[0])
        if not rows and sz and len(sz[0]) >= 2:
            rows = [(0, {int(it[0]): float(it[1]) for it in items(sz[0][1]) if len(it) >= 2})]
        self.t = [r[0] for r in rows]
        self.f = [r[1] for r in rows]

    def at(self, oid: int, t: float) -> float:
        if not self.t:
            return 1.0
        i = max(0, bisect.bisect_right(self.t, t) - 1)
        return self.f[i].get(oid, 1.0)


class TakeWorld:
    """A take's robot, objects and records, for distances at any time of it."""

    def __init__(self, path: str, geometry_dir: str = DEFAULT_GEOMETRY):
        self.path = path
        try:
            with open(os.path.join(path, "scene.json"), encoding="utf-8") as f:
                self.scene = json.load(f)
        except OSError as e:
            raise ClearanceUnavailable(f"{path}: no scene.json: {e}") from e
        self.robot = load_robot(geometry_dir, self.scene.get("robotProfile", "jetrover"))
        self.bodies = _bodies(self.scene, geometry_dir)
        meshes = sorted({load_asset(geometry_dir, o["asset"])[2] for o in self.scene["objects"] if o.get("shape") == "mesh"})
        self.method = METHOD + (f"; mesh pieces by {' | '.join(meshes)}" if meshes else "")
        self.surfaces = _surfaces(self.scene)
        try:
            host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
            frames = read_jlog(os.path.join(path, "isaac.jlog"), "isaac")
        except (OSError, JlogError) as e:
            raise ClearanceUnavailable(f"{path}: {e}") from e
        if not host.tag("WO"):
            raise ClearanceUnavailable(f"{path}: no object poses (WO)")
        self.poses = _Poses(host.tag("WO"))
        self.sizes = _Sizes(host.tag("WZ"), frames.tag("SZ"))
        self.contacts = sorted(((int(r[1]), r[2] if len(r) > 2 else "") for r in host.tag("WC")), key=lambda x: x[0])
        vq = sorted(((int(r[1]), int(r[0]), {it[0]: float(it[1]) for it in items(r[2]) if len(it) >= 2}) for r in frames.tag("VQ")),
                    key=lambda x: x[0])
        if not vq:
            raise ClearanceUnavailable(f"{path}: no per-frame joint values (VQ)")
        self.vq_t = np.array([x[0] for x in vq], float)
        self.vq_frame = {x[1]: i for i, x in enumerate(vq)}
        names = sorted({k for x in vq for k in x[2]})
        self.vq = {k: np.array([x[2].get(k, 0.0) for x in vq], float) for k in names}
        self._placed: Dict[Tuple, List[Piece]] = {}

    def joint_values_at(self, times: Sequence[float]) -> Dict[str, np.ndarray]:
        """Joint values at any times: ``VQ``'s, linear between the frames around each."""
        t = np.asarray(times, float)
        return {k: np.interp(t, self.vq_t, v) for k, v in self.vq.items()}

    def link_boxes(self, values: Dict[str, np.ndarray], n: int) -> Tuple[np.ndarray, np.ndarray]:
        """Each frame's box centres (n, b, 3) and axes (n, b, 3, 3)."""
        R, p = self.robot.link_poses(values, n)
        Rl, pl = R[:, self.robot.box_link], p[:, self.robot.box_link]
        centres = np.einsum("nbij,bj->nbi", Rl, self.robot.box_center) + pl
        axes = np.einsum("bkj,nbij->nbki", self.robot.box_axes, Rl)
        return centres, axes

    def obstacles_at(self, t: float) -> List[Tuple[Body, Optional[Tuple[float, ...]], float, List[Piece]]]:
        """Every obstacle at time ``t`` that is not parked: (body, pose, its size factor, its pieces placed in the world)."""
        out = []
        for b in self.bodies:
            pose = self.poses.at(b.id, t)
            if pose is None or pose[2] < PARKED_BELOW_Z:
                continue
            factor = self.sizes.at(b.id, t)
            k = b.base_scale * factor
            key = (b.id, pose, k)
            placed = self._placed.get(key)
            if placed is None:
                R, x = _quat_matrix(pose[3:]), np.asarray(pose[:3], float)
                placed = [pc.placed(R, x, k) for pc in b.pieces]
                if len(self._placed) > 4096:
                    self._placed.clear()
                self._placed[key] = placed
            out.append((b, pose, factor, placed))
        return out


def _box_index(robot: Robot) -> List[np.ndarray]:
    return [np.flatnonzero(robot.box_link == i) for i in range(len(robot.links))]


def _nearest(boxes: List[Box], obstacles, best: float) -> Tuple[float, int]:
    """The nearest obstacle to a link (its boxes): (signed distance m, at most ``best``; the obstacle's id or NO_OBSTACLE)."""
    best_id = NO_OBSTACLE
    if not obstacles or not boxes:
        return best, best_id
    centres = np.array([b.center for b in boxes])
    axes = np.array([b.axes for b in boxes])
    halves = np.array([b.half for b in boxes])
    link_c = centres.mean(axis=0)
    link_r = float(max(np.linalg.norm(c - link_c) + np.linalg.norm(h) for c, h in zip(centres, halves)))
    along = np.einsum("bij,bj->bi", axes, centres)  # each box's centre along its own axes
    cands = []  # (lower bound, box index, body, piece or sphere)
    for body, pose, factor, placed in obstacles:
        if body.sphere > 0:
            centre = np.asarray(pose[:3], float)
            r = body.sphere * factor
            for i, b in enumerate(boxes):
                d = box_sphere_distance(b, centre, r)
                cands.append((d, i, body, None, (centre, r)))
            continue
        if not placed:
            continue
        pc = np.array([p.center for p in placed])
        pr = np.array([p.radius for p in placed])
        near = np.linalg.norm(pc - link_c, axis=1) - pr - link_r < best
        if not near.any():
            continue
        idx = np.flatnonzero(near)
        # Lower bounds for each (box, piece): the box against the piece's bounding sphere, and the gap along the box's own axes.
        local = np.einsum("bij,pj->bpi", axes, pc[idx]) - along[:, None, :]
        sphere_lb = np.linalg.norm(np.maximum(np.abs(local) - halves[:, None, :], 0.0), axis=2) \
            + np.minimum((np.abs(local) - halves[:, None, :]).max(axis=2), 0.0) - pr[idx][None, :]
        for jj, j in enumerate(idx):
            proj = placed[j].vertices @ axes.reshape(-1, 3).T  # (v, b*3): each box's axes in turn
            lo, hi = proj.min(axis=0).reshape(len(boxes), 3), proj.max(axis=0).reshape(len(boxes), 3)
            gap = np.maximum(lo - (along + halves), (along - halves) - hi).max(axis=1)
            for i in range(len(boxes)):
                lb = max(float(sphere_lb[i, jj]), float(gap[i]))
                if lb < best:
                    cands.append((lb, i, body, placed[j], None))
    cands.sort(key=lambda x: x[0])
    for lb, i, body, piece, sphere in cands:
        if lb >= best:
            break
        d = lb if sphere is not None else box_piece_distance(boxes[i], piece)
        if d < best:
            best, best_id = d, body.id
    return best, best_id


def surface_distance(corners: np.ndarray, surfaces: Sequence[Tuple[str, Piece]]) -> float:
    """A link's signed distance (m) to the nearest surface: its nearest box corner's to each surface box (exact wherever a
    surface's face, its top, is the nearest part of it)."""
    best = math.inf
    for _, s in surfaces:
        half = (s.vertices.max(0) - s.vertices.min(0)) / 2
        d = point_box_distance(corners, s.center, np.eye(3), half)
        best = min(best, float(d.min()))
    return best


def compute(take, geometry_dir: str = DEFAULT_GEOMETRY, cutoff_mm: float = CUTOFF_MM) -> Clearance:
    """The take's clearance, frame by frame (see the module's notes). Raises :class:`ClearanceUnavailable` when the take
    lacks its records or scene, or the cache a geometry its objects need."""
    world = TakeWorld(take.path, geometry_dir)
    robot = world.robot
    cutoff = cutoff_mm / 1000.0
    index, per_link = [], _box_index(robot)
    for f in take.frames:
        number = int(f.timing["frame"])
        if number not in world.vq_frame:
            raise ClearanceUnavailable(f"{take.path}: frame {number} has no joint values (VQ)")
        index.append(world.vq_frame[number])
    n = len(index)
    values = {k: v[index] for k, v in world.vq.items()}
    centres, axes = world.link_boxes(values, n)
    halves = robot.box_half
    obstacle_mm, obstacle_id, surface_mm = [], [], []
    for fi, f in enumerate(take.frames):
        t = float(f.timing["sensed"])
        obstacles = world.obstacles_at(t)
        row_d, row_id, row_s = [], [], []
        for li, link in enumerate(robot.links):
            boxes = [Box(centres[fi, b], axes[fi, b], halves[b]) for b in per_link[li]]
            d, oid = _nearest(boxes, obstacles, cutoff)
            row_d.append(round(min(d, cutoff) * 1000.0, 2))
            row_id.append(oid if d < cutoff else NO_OBSTACLE)
            if link in robot.fixed or not boxes:
                row_s.append(float("nan"))
            else:
                corners = np.concatenate([b.corners() for b in boxes])
                row_s.append(round(min(surface_distance(corners, world.surfaces), cutoff) * 1000.0, 2))
        obstacle_mm.append(tuple(row_d))
        obstacle_id.append(tuple(row_id))
        surface_mm.append(tuple(row_s))
    return Clearance(robot.links, obstacle_mm, obstacle_id, surface_mm, float(cutoff_mm), world.method,
                     tuple(link for link in robot.links if link in robot.fixed))


def link_object_distance(world: TakeWorld, t: float, link: int, object_id: int, cutoff: float = 1.0) -> float:
    """The signed distance (m) from link ``link`` (its URDF index) to object ``object_id`` at time ``t``, joint values
    interpolated between frames; ``cutoff`` when it is further or the object is parked."""
    values = world.joint_values_at([t])
    centres, axes = world.link_boxes(values, 1)
    boxes = [Box(centres[0, b], axes[0, b], world.robot.box_half[b]) for b in _box_index(world.robot)[link]]
    obstacles = [o for o in world.obstacles_at(t) if o[0].id == object_id]
    d, _ = _nearest(boxes, obstacles, cutoff)
    return d


def link_surface_distance(world: TakeWorld, t: float, link: int) -> float:
    """The signed distance (m) from link ``link`` (its URDF index) to the nearest surface at time ``t``, joint values interpolated
    between frames; NaN for a body PhysX never collides with a surface."""
    if world.robot.links[link] in world.robot.fixed:
        return float("nan")
    values = world.joint_values_at([t])
    centres, axes = world.link_boxes(values, 1)
    boxes = [Box(centres[0, b], axes[0, b], world.robot.box_half[b]) for b in _box_index(world.robot)[link]]
    return surface_distance(np.concatenate([b.corners() for b in boxes]), world.surfaces)


__all__ = ["CUTOFF_MM", "Clearance", "ClearanceUnavailable", "DEFAULT_GEOMETRY", "NO_OBSTACLE", "TakeWorld", "compute",
           "link_object_distance", "link_surface_distance"]
