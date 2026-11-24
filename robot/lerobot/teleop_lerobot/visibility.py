"""Whether each object on the table could be seen, frame by frame and camera by camera (docs/adr/0038 L, "Visibility"):
``meta/teleop/visibility.parquet``, one row per (episode, frame, camera, object).

World-models' safety model sees only the images. This says, per frame, how much of each obstacle each camera showed:

- ``visible_px``: the object's pixels in that camera's instance mask (``isaac/<camera>.mask.frames``, id ``100 + n`` for
  scene object ``n``, trainer/contract/SESSION_FORMAT.md ``ID``);
- ``expected_px``: its whole silhouette in the image with nothing in front of it: its collision geometry (workstream C's
  convex pieces in the geometry cache for a mesh, the scene file's primitive for a box or cylinder, as
  :mod:`teleop_lerobot.clearance` builds them) at the frame's pose and size, projected through the take's ``CM`` camera
  (intrinsics, and pose: a room camera's in the base frame; a wrist camera's its mount link's forward kinematics at the
  frame's ``VQ`` joint values times its ``CM`` pose), clipped to the near plane, and rasterised as the union of the pieces'
  projected hulls, a pixel counted when its centre falls inside, as the renderer samples it;
- ``visible_fraction``: visible over expected (NaN when nothing is expected);
- ``in_frustum``: the silhouette covers at least one pixel of the image (``expected_px > 0``).

The objects are every scene object on the table at the frame (not parked on the bridge's shelf), with its ``role`` from the
scene file: ``obstacle``, ``stand``, ``plank`` and ``support`` are the obstacles in reach (:data:`OBSTACLE_ROLES`), and
``distractor`` the objects in view but out of reach, included for completeness. A scene without roles (before
docs/adr/0038) gives an empty role, counted as an obstacle by :func:`summary`. The table and the flat clutter are left out.

The object's pose is ``WO``'s interpolated to the frame's ``stateSensed`` and its size the ``WZ`` factor in force, exactly
as the clearance takes them. No depth test: the mask already resolves what is in front.

What cannot be had is NaN (null) with a ``reason``: a camera with no mask stream, a frame whose mask is missing or damaged,
an object with no geometry in the cache, a wrist camera at a frame with no joint values, and an object the renderer drew
without its mask label (its id in no mask of the take, though its silhouette reached :data:`UNLABELLED_PX` pixels). An episode whose objects cannot be
known at all (no scene file, no ``WO``, no ``CM``) has one row with no camera or object and the reason.

Pure numpy and OpenCV (the mask PNGs and the hulls).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import clearance as C
from .jlog import items

OBSTACLE_ROLES = ("obstacle", "stand", "plank", "support")  # a mixed pool's objects in reach (docs/adr/0038 decision 3)
SEEN = 0.5  # summary(): an object counts as seen by a camera when at least this share of it shows
MASK_OFFSET = 100  # an object's mask id is 100 + its scene id (SESSION_FORMAT.md, ID)
UNLABELLED_PX = 200  # an object never in the masks though its silhouette reached this many pixels was drawn unlabelled
SPHERE_POINTS = 96  # a sphere's silhouette: the hull of this many points on it
COLUMNS = ("episode_index", "frame_index", "camera", "object_id", "role", "visible_px", "expected_px", "visible_fraction",
           "in_frustum", "reason")
METHOD = ("expected: the object's collision geometry (convex pieces, or the scene file's primitive) at the frame's WO pose "
          "and WZ size, projected through the take's CM camera (a wrist camera through its mount link's forward kinematics "
          "at the frame's VQ), clipped to the near plane, the union of the pieces' hulls by pixel centres; visible: the "
          "object's pixels in the instance mask")


# --- projecting and rasterising ------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class View:
    """One recorded camera: its pinhole (SESSION_FORMAT.md ``CM``) and its pose, in the base frame or its mount link's."""

    name: str  # as the dataset names it (the twin's wrist is front)
    recorded: str  # as the take names it
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    near: float
    rotation: np.ndarray  # (3, 3): the camera's axes as columns (it looks along its -z, +y up)
    position: np.ndarray  # (3,)
    mount: Optional[str]  # the link it is fixed to, its pose then in that link's frame
    mask: bool  # it records a mask stream


def _quat(q: Sequence[float]) -> np.ndarray:
    return C._quat_matrix(q)


def views(frames, rename: Dict[str, str]) -> List[View]:
    """Every camera ``CM`` gives intrinsics and a pose for."""
    out = []
    for r in frames.tag("CM") if frames is not None else []:
        if len(r) < 16:
            continue
        streams = r[17].split(";") if len(r) > 17 else ["rgb"]
        mount = (r[16] or None) if len(r) > 16 else None
        frame = r[18] if len(r) > 18 and r[18] else "base"
        out.append(View(rename.get(r[0], r[0]), r[0], int(r[1]), int(r[2]), float(r[3]), float(r[4]), float(r[5]),
                        float(r[6]), float(r[7]), _quat([float(v) for v in r[12:16]]), np.array([float(v) for v in r[9:12]]),
                        mount if frame == "mount" else None, "mask" in streams))
    return out


def project_hull(camera_points: np.ndarray, view: View) -> Optional[np.ndarray]:
    """The image hull (k, 2) of a convex piece given as its vertices in the camera's frame, clipped to the near plane (the
    part at least ``near`` in front of the camera), in pixel coordinates (``u = cx + fx x/d``, ``v = cy - fy y/d``, ``d =
    -z``; a pixel's centre at +0.5). None when no part of it is in front."""
    d = -camera_points[:, 2]
    front = d >= view.near
    if not front.any():
        return None
    pts = camera_points
    if not front.all():
        # The clipped polytope's vertices are the front vertices and where segments between a front and a behind vertex
        # cross the plane: every edge is such a segment, and the rest lie inside the piece, so the hull is exact.
        a, b = camera_points[front], camera_points[~front]
        da, db = d[front][:, None], d[~front][None, :]
        s = (da - view.near) / (da - db)
        cut = a[:, None, :] + s[..., None] * (b[None, :, :] - a[:, None, :])
        pts = np.concatenate([a, cut.reshape(-1, 3)])
        d = -pts[:, 2]
    u = view.cx + view.fx * pts[:, 0] / d
    v = view.cy - view.fy * pts[:, 1] / d
    import cv2

    hull = cv2.convexHull(np.stack([u, v], axis=1).astype(np.float32)).reshape(-1, 2).astype(np.float64)
    return hull if len(hull) >= 3 else None


def spans(hull: np.ndarray, width: int, height: int) -> Optional[Tuple[int, np.ndarray, np.ndarray]]:
    """The pixels of a convex polygon (k, 2), by pixel centres, within a ``width`` x ``height`` image: (first row, each
    row's first column, each row's last column; a row's last below its first is empty). None when no row is covered."""
    x, y = hull[:, 0], hull[:, 1]
    j0 = max(0, int(math.ceil(y.min() - 0.5)))
    j1 = min(height - 1, int(math.floor(y.max() - 0.5)))
    if j1 < j0:
        return None
    area = float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))
    if abs(area) < 1e-12:
        return None
    sign = 1.0 if area > 0 else -1.0
    ax, ay = x, y
    dx, dy = np.roll(x, -1) - x, np.roll(y, -1) - y
    rows = np.arange(j0, j1 + 1, dtype=np.float64)[:, None] + 0.5
    k = sign * dy
    with np.errstate(divide="ignore", invalid="ignore"):
        bound = ax + dx * (rows - ay) / np.where(dy == 0, 1.0, dy)
    hi = np.min(np.where(k > 0, bound, np.inf), axis=1)
    lo = np.max(np.where(k < 0, bound, -np.inf), axis=1)
    flat = (k == 0) & (sign * dx * (rows - ay) < 0)  # a horizontal edge with the row outside it
    first = np.maximum(np.ceil(lo - 0.5), 0)
    last = np.minimum(np.floor(hi - 0.5), width - 1)
    last = np.where(flat.any(axis=1), first - 1, last)
    return j0, first.astype(np.int64), last.astype(np.int64)


def silhouette(hulls: Sequence[np.ndarray], width: int, height: int, canvas: bool = False):
    """The pixels the union of convex polygons covers in the image, by pixel centres: their count, and with ``canvas`` the
    full image mask (height, width) as well."""
    covered = [s for s in (spans(h, width, height) for h in hulls) if s is not None]
    if not covered and not canvas:
        return 0
    if len(covered) == 1 and not canvas:
        _, first, last = covered[0]
        return int(np.maximum(last - first + 1, 0).sum())
    r0 = min(s[0] for s in covered) if covered else 0
    r1 = max(s[0] + len(s[1]) - 1 for s in covered) if covered else -1
    c0 = int(min(s[1].min() for s in covered)) if covered else 0
    c1 = int(max(s[2].max() for s in covered)) if covered else -1
    box = np.zeros((max(r1 - r0 + 1, 0), max(c1 - c0 + 1, 0)), bool)
    cols = np.arange(c0, c1 + 1)[None, :]
    for j, first, last in covered:
        box[j - r0:j - r0 + len(first)] |= (cols >= first[:, None]) & (cols <= last[:, None])
    count = int(box.sum())
    if not canvas:
        return count
    full = np.zeros((height, width), bool)
    if box.size:
        full[r0:r1 + 1, c0:c1 + 1] = box
    return count, full


def _sphere_points(radius: float) -> np.ndarray:
    n = SPHERE_POINTS
    k = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * k / n)
    theta = math.pi * (1 + 5 ** 0.5) * k
    return radius * np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], axis=1)


def _frustum_planes(view: View) -> np.ndarray:
    """The image's four side planes through the camera, unit normals in (x, y, d) with d = -z, inside where n.(x, y, d) >= 0."""
    w, h = view.width, view.height
    planes = np.array([[view.fx, 0.0, view.cx], [-view.fx, 0.0, w - view.cx], [0.0, -view.fy, view.cy],
                       [0.0, view.fy, h - view.cy]])
    return planes / np.linalg.norm(planes, axis=1)[:, None]


# --- one take --------------------------------------------------------------------------------------------------------------

@dataclass
class _Object:
    id: int
    role: str
    body: Optional[C.Body]
    why: Optional[str]  # why it has no geometry
    vertices: Optional[np.ndarray] = None  # every piece's vertices, stacked, in its body frame at the file's size
    cuts: Tuple[int, ...] = ()  # where each piece's vertices start in ``vertices``


def _objects(scene: Dict, geometry_dir: Optional[str]) -> List[_Object]:
    out = []
    for o in scene.get("objects") or []:
        oid, role = int(o["id"]), str(o.get("role") or "")
        if not geometry_dir:
            out.append(_Object(oid, role, None, "no geometry folder given"))
            continue
        try:
            body = C._bodies({"objects": [o]}, geometry_dir)[0]
        except C.ClearanceUnavailable as e:
            out.append(_Object(oid, role, None, f"no geometry: {e}"))
            continue
        except (KeyError, ValueError, TypeError) as e:
            out.append(_Object(oid, role, None, f"no geometry: {type(e).__name__}: {e}"))
            continue
        pieces = [p.vertices * body.base_scale for p in body.pieces] if body.pieces else [_sphere_points(body.sphere)]
        cuts = tuple(int(c) for c in np.cumsum([0] + [len(p) for p in pieces])[:-1])
        out.append(_Object(oid, role, body, None, np.concatenate(pieces), cuts))
    return out


def _quantised(pose: Tuple[float, ...]) -> Tuple[float, ...]:
    """A pose rounded to 0.1 mm and 1e-5 of a quaternion: a resting object's WO wobbles below that, so a fixed camera's
    silhouette of it is made once (and the same whichever frame comes first)."""
    return tuple(round(v, 4) for v in pose[:3]) + tuple(round(v, 5) for v in pose[3:7])


def expected_px(obj: _Object, pose: Tuple[float, ...], factor: float, rotation: np.ndarray, position: np.ndarray,
                view: View, canvas: bool = False):
    """The object's silhouette in the view with the camera at (rotation, position) in the base frame: its pixel count (and
    with ``canvas`` the image mask)."""
    R_o, x_o = _quat(pose[3:7]), np.asarray(pose[:3], float)
    radius = obj.body.radius * factor if obj.body.pieces else obj.body.sphere * factor
    centre = (x_o - position) @ rotation
    xyd = np.array([centre[0], centre[1], -centre[2]])
    if xyd[2] + radius < view.near or (_frustum_planes(view) @ xyd < -radius).any():
        return (0, np.zeros((view.height, view.width), bool)) if canvas else 0
    pts = obj.vertices @ (factor * (R_o.T @ rotation)) + centre
    ends = list(obj.cuts[1:]) + [len(pts)]
    hulls = [h for h in (project_hull(pts[a:b], view) for a, b in zip(obj.cuts, ends)) if h is not None]
    return silhouette(hulls, view.width, view.height, canvas)


class _Masks:
    """A camera's mask sidecar, read once, and its frames decoded on demand."""

    def __init__(self, path: str):
        with open(path, "rb") as f:
            self.data = f.read()

    def counts(self, offset: int, length: int) -> np.ndarray:
        import cv2

        if offset < 12 or offset + length > len(self.data):
            raise ValueError(f"the record at {offset} is outside the sidecar")
        if int.from_bytes(self.data[offset - 4:offset], "little") != length:
            raise ValueError(f"the record at {offset} is not {length} bytes")
        image = cv2.imdecode(np.frombuffer(self.data, np.uint8, count=length, offset=offset), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f"the record at {offset} is not an image")
        return np.bincount(image.ravel())


@dataclass
class Visibility:
    """One episode's rows, column by column (:data:`COLUMNS` less ``episode_index``), and the method."""

    columns: Dict[str, list]
    method: str = METHOD


def _empty() -> Dict[str, list]:
    return {k: [] for k in COLUMNS if k != "episode_index"}


def unavailable(why: str) -> Visibility:
    """An episode whose objects cannot be known: one row with no camera or object, and why."""
    cols = _empty()
    for k in cols:
        cols[k].append(None)
    cols["reason"][0] = why
    return Visibility(cols)


def compute(take, geometry_dir: Optional[str], records=None, rename: Optional[Dict[str, str]] = None) -> Visibility:
    """The take's rows (see the module's notes), from its own files (``records``: ``sidecars.read_records``' if read)."""
    from . import sidecars

    rec = records or sidecars.read_records(take.path)
    rename = rename or {}
    if rec.scene is None:
        return unavailable("no scene.json")
    if rec.frames is None:
        return unavailable("no frames log")
    if not rec.host.tag("WO"):
        return unavailable("no object poses (WO)")
    cams = views(rec.frames, rename)
    if not cams:
        return unavailable("no camera with intrinsics and a pose (CM)")
    gdir = os.path.expanduser(geometry_dir) if geometry_dir else None
    if gdir and not os.path.isdir(gdir):
        gdir, no_dir = None, f"no geometry at {geometry_dir}"
    else:
        no_dir = None
    objects = _objects(rec.scene, gdir)
    if no_dir:
        for o in objects:
            o.why = no_dir
    poses = C._Poses(rec.host.tag("WO"))
    sizes = C._Sizes(rec.host.tag("WZ"), rec.frames.tag("SZ"))
    id_rows = rec.frames.tag("ID")
    labelled = None
    if id_rows and id_rows[-1]:
        labelled = {int(it[0]) for it in items(id_rows[-1][0]) if it and it[0].lstrip("-").isdigit()}

    # Each recorded frame's mask records, and joint values.
    wanted = {int(f.timing["frame"]) for f in take.frames}
    parts: Dict[int, Dict[str, Tuple[int, int]]] = {}
    for r in rec.frames.tag("VF"):
        n = int(r[1])
        if n not in wanted or n in parts:
            continue
        p = {}
        for stream, off, length in (x.rsplit(":", 2) for x in (r[4] if len(r) > 4 else "").split(";") if x):
            if stream.endswith(".mask"):
                p[stream[:-5]] = (int(off), int(length))
        parts[n] = p
    masks: Dict[str, Optional[_Masks]] = {}
    mask_why: Dict[str, str] = {}
    for v in cams:
        path = os.path.join(take.path, "isaac", v.recorded + ".mask.frames")
        if not v.mask:
            mask_why[v.name] = "the camera records no mask"
        elif not os.path.exists(path):
            mask_why[v.name] = f"no {v.recorded}.mask.frames"
        else:
            try:
                masks[v.name] = _Masks(path)
            except OSError as e:
                mask_why[v.name] = f"{v.recorded}.mask.frames unreadable: {e}"

    mounted = [v for v in cams if v.mount]
    mount_pose: Dict[Tuple[str, int], Tuple[np.ndarray, np.ndarray]] = {}
    mount_why = None
    if mounted:
        if not gdir:
            mount_why = no_dir or "no geometry folder given (the robot's kinematics)"
        else:
            try:
                robot = C.load_robot(gdir, rec.scene.get("robotProfile", "jetrover"))
            except C.ClearanceUnavailable as e:
                robot, mount_why = None, f"no robot kinematics: {e}"
            if robot is not None:
                vq = {}
                for r in rec.frames.tag("VQ"):
                    n = int(r[0])
                    if n in wanted:
                        vq[n] = {it[0]: float(it[1]) for it in items(r[2] if len(r) > 2 else "") if len(it) >= 2}
                numbers = sorted(vq)
                index = {name: i for i, name in enumerate(robot.links)}
                if numbers:
                    names = sorted({k for x in vq.values() for k in x})
                    R, p = robot.link_poses({k: np.array([vq[n].get(k, 0.0) for n in numbers]) for k in names}, len(numbers))
                    for v in mounted:
                        if v.mount not in index:
                            continue
                        li = index[v.mount]
                        for i, n in enumerate(numbers):
                            mount_pose[(v.mount, n)] = (R[i, li], p[i, li])
                if any(v.mount not in index for v in mounted):
                    mount_why = f"no link {[v.mount for v in mounted if v.mount not in index]} in the robot's kinematics"

    cache: Dict[Tuple, int] = {}
    per_frame: Dict[int, List[Tuple]] = {}
    cols = _empty()
    for fi, f in enumerate(take.frames):
        n, t = int(f.timing["frame"]), float(f.timing["sensed"])
        rows = per_frame.get(n)
        if rows is None:
            rows = []
            on_table = []
            for o in objects:
                pose = poses.at(o.id, t)
                if pose is None or pose[2] < C.PARKED_BELOW_Z:
                    continue
                on_table.append((o, _quantised(pose), sizes.at(o.id, t)))
            for v in cams:
                counts, why_v = None, mask_why.get(v.name)
                if why_v is None:
                    part = parts.get(n, {}).get(v.recorded)
                    if part is None:
                        why_v = f"frame {n} has no {v.recorded} mask"
                    else:
                        try:
                            counts = masks[v.name].counts(*part)
                        except ValueError as e:
                            why_v = f"frame {n}'s {v.recorded} mask: {e}"
                pose_cam, why_cam = None, None
                if v.mount:
                    link = mount_pose.get((v.mount, n))
                    if link is None:
                        why_cam = mount_why or f"frame {n} has no joint values (VQ)"
                    else:
                        pose_cam = (link[0] @ v.rotation, link[1] + link[0] @ v.position)
                else:
                    pose_cam = (v.rotation, v.position)
                for o, pose, factor in on_table:
                    mask_id = MASK_OFFSET + o.id
                    reasons = []
                    if counts is not None and labelled is not None and mask_id not in labelled:
                        visible, reasons = None, [f"mask id {mask_id} is not in the take's ID table"]
                    elif counts is not None:
                        visible = int(counts[mask_id]) if mask_id < len(counts) else 0
                    else:
                        visible = None
                        reasons.append(why_v)
                    expected = None
                    if o.body is None:
                        reasons.append(o.why)
                    elif pose_cam is None:
                        reasons.append(why_cam)
                    else:
                        key = None if v.mount else (v.name, o.id, pose, factor)
                        expected = cache.get(key) if key is not None else None
                        if expected is None:
                            expected = expected_px(o, pose, factor, pose_cam[0], pose_cam[1], v)
                            if key is not None:
                                cache[key] = expected
                    fraction = (visible / expected) if (visible is not None and expected) else float("nan")
                    rows.append((v.name, o.id, o.role, visible, expected, fraction,
                                 None if expected is None else expected > 0, "; ".join(reasons) or None))
            per_frame[n] = rows
        for row in rows:
            cols["frame_index"].append(fi)
            for k, x in zip(("camera", "object_id", "role", "visible_px", "expected_px", "visible_fraction", "in_frustum",
                             "reason"), row):
                cols[k].append(x)
    _unlabelled(cols)
    return Visibility(cols)


def _unlabelled(cols: Dict[str, list]) -> None:
    """An object whose mask id is in no camera's mask at any frame, though its silhouette covered at least
    :data:`UNLABELLED_PX` pixels of some camera at some frame, was drawn without its label (the twin's masks of 2026-11-24
    lose some objects' labels for whole takes: their silhouettes read 0, unlabelled, where the object stands in front of the
    table). Its counts are not measurements: null, with the reason."""
    seen, big = set(), set()
    for oid, vis, exp in zip(cols["object_id"], cols["visible_px"], cols["expected_px"]):
        if vis:
            seen.add(oid)
        if exp is not None and exp >= UNLABELLED_PX:
            big.add(oid)
    lost = big - seen
    for i, oid in enumerate(cols["object_id"]):
        if oid in lost and cols["visible_px"][i] is not None:
            cols["visible_px"][i] = None
            cols["visible_fraction"][i] = float("nan")
            why = (f"mask id {MASK_OFFSET + oid} is in no mask of the take though its silhouette reached {UNLABELLED_PX}+ px: "
                   f"drawn without its label")
            cols["reason"][i] = f"{cols['reason'][i]}; {why}" if cols["reason"][i] else why


# --- the dataset's file and its summary ----------------------------------------------------------------------------------

def write(path: str, written: Sequence[Tuple[int, Optional[Visibility]]]) -> None:
    """``visibility.parquet``: every episode's rows (an episode without any: one row saying visibility was not computed)."""
    import pandas as pd  # lerobot's own dependency

    data: Dict[str, list] = {k: [] for k in COLUMNS}
    for ep, vis in written:
        vis = vis or unavailable("visibility was not computed")
        n = len(vis.columns["frame_index"])
        data["episode_index"].extend([ep] * n)
        for k in COLUMNS[1:]:
            data[k].extend(vis.columns[k])
    df = pd.DataFrame({
        "episode_index": pd.array(data["episode_index"], dtype="int64"),
        "frame_index": pd.array(data["frame_index"], dtype="Int64"),
        "camera": pd.array(data["camera"], dtype="string"),
        "object_id": pd.array(data["object_id"], dtype="Int32"),
        "role": pd.array(data["role"], dtype="string"),
        "visible_px": pd.array(data["visible_px"], dtype="Int32"),
        "expected_px": pd.array(data["expected_px"], dtype="Int32"),
        "visible_fraction": pd.array([float("nan") if v is None else float(v) for v in data["visible_fraction"]],
                                     dtype="float64"),
        "in_frustum": pd.array(data["in_frustum"], dtype="boolean"),
        "reason": pd.array(data["reason"], dtype="string"),
    })
    df.to_parquet(path, index=False)


def summary(df, contacts: Sequence[Dict], fps: float) -> Dict:
    """What the visibility rows of a dataset say about its obstacles (roles :data:`OBSTACLE_ROLES`, or none given):

    - per obstacle-frame (an obstacle at one frame of one episode), in how many cameras it showed at least :data:`SEEN` of
      itself: two or more, one, none; and the share not seen at all (no pixel in any camera);
    - per camera, the share of obstacle-frames it showed at least :data:`SEEN` of, and the share it had in its frustum;
    - per contact with an object (``contacts.jsonl``'s kinds ``object`` and ``fixture``), whether its object showed at least
      :data:`SEEN` of itself in at least one camera at some frame of the second before the contact began, and at the
      frame just before. A contact at the episode's first frame has no second before, and is counted apart, as is one
      whose object was measured at no frame of that second.

    A row is measured when both its counts are known (an object out of the frustum is measured, and not seen). Obstacle-frames
    with no measured row (no mask, no geometry, or drawn without its label) are counted apart; the shares are of the rest."""
    import pandas as pd

    rows = df[df["object_id"].notna()]
    role = rows["role"].fillna("")
    obstacle = rows[role.isin(OBSTACLE_ROLES) | (role == "")]
    out: Dict = {"rows": int(len(df)), "rowsWithReason": int(df["reason"].notna().sum()),
                 "reasons": sorted({str(r) for r in df["reason"].dropna()})[:5]}
    if obstacle.empty:
        return {**out, "obstacleFrames": 0}
    measured = obstacle["visible_px"].notna() & obstacle["expected_px"].notna()
    o = obstacle.assign(seen=(obstacle["visible_fraction"] >= SEEN) & measured, measured=measured,
                        any_px=(obstacle["visible_px"].fillna(0) > 0) & measured)
    keys = ["episode_index", "frame_index", "object_id"]
    g = o.groupby(keys).agg(seen=("seen", "sum"), measured=("measured", "sum"), any_px=("any_px", "any"))
    m = g[g["measured"] > 0]
    n = len(m)
    share = (lambda k: round(float(k) / n, 4) if n else None)
    out.update({"obstacleFrames": int(len(g)), "unmeasured": int(len(g) - n), "episodes": int(o["episode_index"].nunique()),
                "seenInTwoOrMore": share((m["seen"] >= 2).sum()), "seenInOne": share((m["seen"] == 1).sum()),
                "seenInNone": share((m["seen"] == 0).sum()), "notSeenAtAll": share((~m["any_px"]).sum())})
    per_camera = {}
    for cam, c in o.groupby("camera"):
        cm = c[c["measured"]]
        per_camera[str(cam)] = {"seen": round(float(cm["seen"].mean()), 4) if len(cm) else None,
                                "inFrustum": round(float(c["in_frustum"].fillna(False).astype(bool).mean()), 4)}
    out["perCamera"] = dict(sorted(per_camera.items()))

    def frames_of(part):  # (episode, object) -> its frames in ``part``; a contact's object, whatever its role
        return {k: set(int(x) for x in f) for k, f in part.groupby(["episode_index", "object_id"])["frame_index"]}
    known = rows["visible_px"].notna() & rows["expected_px"].notna()
    seen_any, measured_any = frames_of(rows[known & (rows["visible_fraction"] >= SEEN)]), frames_of(rows[known])
    window = max(1, int(round(fps)))
    total = before = at_start = just_before = unmeasured = 0
    for c in contacts:
        if c.get("kind") not in ("object", "fixture") or c.get("startFrame") is None:
            continue
        total += 1
        start = int(c["startFrame"])
        if start <= 0:
            at_start += 1
            continue
        key = (int(c["episode"]), int(c["object"]))
        if not any(start - window <= x < start for x in measured_any.get(key, ())):
            unmeasured += 1
            continue
        frames = seen_any.get(key, set())
        if any(start - window <= x < start for x in frames):
            before += 1
        if (start - 1) in frames:
            just_before += 1
    counted = total - at_start - unmeasured
    out["contacts"] = {"total": total, "atStart": at_start, "unmeasured": unmeasured, "seenInSecondBefore": before,
                       "seenJustBefore": just_before,
                       "shareSeenInSecondBefore": round(before / counted, 4) if counted else None,
                       "shareSeenJustBefore": round(just_before / counted, 4) if counted else None}
    return out


def _pct(v) -> str:
    return "n/a" if v is None else f"{100 * v:.1f}%"


def summary_line(name: str, s: Dict) -> str:
    """safety-report's line for one dataset's :func:`summary`."""
    if not s.get("obstacleFrames"):
        return f"{name}: visibility: no obstacle-frames ({s.get('rows', 0)} rows" + (
            f"; why not: {s['reasons']}" if s.get("reasons") else "") + ")"
    c = s["contacts"]
    cams = ", ".join(f"{k} {_pct(v['seen'])} (in frustum {_pct(v['inFrustum'])})" for k, v in s["perCamera"].items())
    return (f"{name}: visibility of {s['obstacleFrames']} obstacle-frames in {s['episodes']} episode(s): at least "
            f"{int(SEEN * 100)}% visible in 2+ cameras {_pct(s['seenInTwoOrMore'])}, in 1 {_pct(s['seenInOne'])}, in none "
            f"{_pct(s['seenInNone'])}; not one pixel in any camera {_pct(s['notSeenAtAll'])}; per camera at least "
            f"{int(SEEN * 100)}%: {cams}; contacts whose object was at least {int(SEEN * 100)}% visible in 1+ camera in the "
            f"second before: {c['seenInSecondBefore']} of {c['total'] - c['atStart'] - c['unmeasured']} "
            f"({_pct(c['shareSeenInSecondBefore'])}), in the frame before: {_pct(c['shareSeenJustBefore'])}"
            + (f" ({c['atStart']} at the first frame left out)" if c["atStart"] else "")
            + (f" ({c['unmeasured']} unmeasured left out)" if c["unmeasured"] else "")
            + (f"; {s['unmeasured']} obstacle-frames unmeasured" if s.get("unmeasured") else "")
            + (f"; {s['rowsWithReason']} rows with a reason, e.g. {s['reasons'][:2]}" if s.get("rowsWithReason") else ""))


def dataset_line(root: str, name: str) -> Optional[str]:
    """The visibility line for one dataset from its files (``meta/teleop/visibility.parquet``, ``contacts.jsonl``,
    ``meta/info.json``'s fps), or None when it has no visibility file."""
    import json

    import pandas as pd

    teleop = os.path.join(root, "meta", "teleop")
    path = os.path.join(teleop, "visibility.parquet")
    if not os.path.exists(path):
        return None
    contacts = []
    if os.path.exists(os.path.join(teleop, "contacts.jsonl")):
        with open(os.path.join(teleop, "contacts.jsonl"), encoding="utf-8") as f:
            contacts = [json.loads(x) for x in f if x.strip()]
    fps = 15.0
    try:
        with open(os.path.join(root, "meta", "info.json"), encoding="utf-8") as f:
            fps = float(json.load(f).get("fps") or fps)
    except (OSError, ValueError):
        pass
    return summary_line(name, summary(pd.read_parquet(path), contacts, fps))


__all__ = ["COLUMNS", "METHOD", "OBSTACLE_ROLES", "SEEN", "View", "Visibility", "compute", "dataset_line", "expected_px",
           "project_hull", "silhouette", "spans", "summary", "summary_line", "unavailable", "views", "write"]
