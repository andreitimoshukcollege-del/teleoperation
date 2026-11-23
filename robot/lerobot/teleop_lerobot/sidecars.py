"""What each episode of a safety dataset showed and what happened in it, in sidecars beside LeRobot's files (docs/adr/0038
decision 6). Everything is read from the take alone:

- ``session.json`` and ``scene.json`` (the scene file RobotHost ran);
- ``robothost.jlog``: ``WL`` (layout), ``WZ`` (sizes), ``WO`` (poses), ``WC`` (contacts; object 0 is the table), ``SE``
  (an old script's planned events), ``RP`` (a mixed take's replay group, role and edit) and the mixed script's move records;
- ``isaac.jlog``: ``SC``, ``CM`` (cameras), ``EN`` (room), ``LK``/``MT`` (looks), ``CL`` (flat clutter), ``VQ`` (posed joints),
  ``ER`` (frames ended early);
- each frame's clearance, from workstream C's ``teleop_lerobot.clearance`` and the twin's collision geometry, when both are
  there (NaN when not).

The files, under ``meta/teleop/`` (``robot/lerobot/README.md``, "Safety datasets: the sidecars", defines every field):

- ``episodes.jsonl``: one line an episode: its take, scenario, layout (seed and :func:`layout_hash`), outcome, replay group,
  clearance and contact counts, room, looks, camera jitter, the planned moves;
- ``scenes.jsonl``: one line an episode: every object on the table (size, mass, kind, reach, start pose, pose changes), the
  surfaces and the cameras;
- ``contacts.jsonl``: one line per stretch of the arm touching one object (or the table);
- ``frames.parquet``: one row a frame: contact flags, labels, clearance, time to the next contact, ``filled``, skews;
- ``report.json``: the dataset's scenario, outcome and contact counts, and the options it was built with;
- ``unavoidable_ranges.json``: only with a stop model (:class:`StopModel`): the frames from which a stop could no longer
  avoid the coming contact. Provisional until the real arm's stop latency is measured.

No LeRobot here, and pandas only to write ``frames.parquet``: the rest runs and is tested with plain Python.
"""

from __future__ import annotations

import bisect
import hashlib
import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .jlog import Jlog, JlogError, items, read_jlog
from .takes import Take

TABLE = 0  # WC's object 0: the table (docs/adr/0038 decision 4), never unsafe by default
NEUTRAL_CAPTION = "move the arm"  # every episode's task, whatever its scenario (docs/adr/0038 decision 6)
NEAR_MISS_MM = 30.0  # no contact, but some link came within this of an obstacle: a near miss
EARLY_CONTACT_S = 1.5  # item 24: a first contact sooner than this after the episode's start is reported
LATE_CONTACT_S = 1.0  # item 24: as is a last contact ending less than this before its end
PARKED_Z = -2.0  # a pool's parked objects stand on the hidden shelf, z about -5 m (the floor is at -1.05 m)
CHANGE_MM, CHANGE_DEG = 1.0, 1.0  # scenes.jsonl lists a pose change once an object moved or turned this much
GEOMETRY_DIR = "~/isaacsim/assets/clearance"  # where the twin's collision geometry is exported (workstream C)
OPEN_END = 2 ** 62  # a contact no WC ended (as safety.OPEN_END)

# The JetRover's links in the URDF's <link> order (robot/jetrover/urdf/jetrover.urdf): WC's link index (255: none).
JETROVER_LINKS = ("base_link", "turntable", "proximal_link", "distal_link", "wrist_link", "gripper_link", "left_finger",
                  "right_finger")

# The mixed script's planned-move record in robothost.jlog, one per planned move (workstream M; trainer/contract/
# SESSION_FORMAT.md, ``MV``). Metadata only, never labels, read leniently: every field kept as a string, named by
# MOVE_FIELDS in order after ``t``, any later ones under ``extra``.
MOVE_TAGS: Tuple[str, ...] = ("MV",)
MOVE_FIELDS: Tuple[str, ...] = ("index", "count", "kind", "object", "offsetMm", "gapMm", "startS", "contactS", "closestS", "endS")


@dataclass(frozen=True)
class StopModel:
    """How the arm stops (item 17), for :func:`unavoidable`: a stop takes effect ``latency_ms`` after it is asked for, then the
    arm slows linearly to rest over ``stop_time_ms``, or, with ``decel_rad_s2`` instead, over the time its fastest joint
    needs at that deceleration from its speed at the frame (from the posed joints, ``VQ``)."""

    latency_ms: float
    stop_time_ms: Optional[float] = None
    decel_rad_s2: Optional[float] = None

    def __post_init__(self):
        if not (self.latency_ms >= 0):
            raise ValueError("the stop latency is at least 0 ms")
        if (self.stop_time_ms is None) == (self.decel_rad_s2 is None):
            raise ValueError("a stop model needs a stop time or a deceleration, not both")
        if self.stop_time_ms is not None and not (self.stop_time_ms >= 0):
            raise ValueError("the stop time is at least 0 ms")
        if self.decel_rad_s2 is not None and not (self.decel_rad_s2 > 0):
            raise ValueError("the deceleration is above 0 rad/s^2")


@dataclass(frozen=True)
class Options:
    """How a safety dataset is built (docs/adr/0038 decision 6)."""

    caption: str = "neutral"  # "neutral": every task NEUTRAL_CAPTION; "category": the scenario's name, as before
    table_unsafe: bool = False  # count the arm touching the table (WC object 0) as unsafe too
    geometry_dir: Optional[str] = None  # the twin's collision geometry, for clearance; None: no clearance
    stop: Optional[StopModel] = None  # writes unavoidable_ranges.json

    def __post_init__(self):
        if self.caption not in ("neutral", "category"):
            raise ValueError(f"caption is 'neutral' or 'category', not {self.caption!r}")

    def task(self, scenario: str) -> str:
        return NEUTRAL_CAPTION if self.caption == "neutral" else scenario

    def describe(self) -> Dict:
        return {"caption": self.caption, "tableUnsafe": self.table_unsafe, "geometryDir": self.geometry_dir,
                "stopModel": None if self.stop is None else {"latencyMs": self.stop.latency_ms, "stopTimeMs": self.stop.stop_time_ms,
                                                             "decelRadS2": self.stop.decel_rad_s2}}


@dataclass(frozen=True)
class Stretch:
    """The arm touching one object (or the table, object 0) without a break: from the first ``WC`` naming it to the first
    that does not (``end`` is ``OPEN_END`` when none did), the links that touched it and the largest force a ``WC`` gave it
    (the sum over its links)."""

    object: int
    start: int
    end: int
    links: Tuple[int, ...]
    peak_n: float


@dataclass
class Episode:
    """One take as a safety episode: its frame labels and its sidecar lines (each without the episode index, which is the
    dataset's to give)."""

    scenario: str
    labels: List[int]
    line: Dict
    scene: Dict
    contacts: List[Dict]
    frames: Dict[str, list]
    unavoidable: Optional[List[List[int]]] = None
    flags: List[str] = field(default_factory=list)


# --- reading the take -----------------------------------------------------------------------------------------------------

@dataclass
class Records:
    """The take's files, read once."""

    path: str
    session: Dict
    scene: Optional[Dict]
    host: Jlog
    frames: Optional[Jlog]


def read_records(path: str) -> Records:
    with open(os.path.join(path, "session.json"), encoding="utf-8") as f:
        session = json.load(f)
    scene = None
    if os.path.exists(os.path.join(path, "scene.json")):
        with open(os.path.join(path, "scene.json"), encoding="utf-8") as f:
            scene = json.load(f)
    host = read_jlog(os.path.join(path, "robothost.jlog"), "robothost")
    frames = None
    for writer in ("isaac", "camera"):
        p = os.path.join(path, writer + ".jlog")
        if os.path.exists(p):
            frames = read_jlog(p, writer)
            break
    return Records(path, session, scene, host, frames)


def _latest(rows: Sequence[Sequence[str]], t: int, at: int = 0) -> Optional[Sequence[str]]:
    """The last of ``rows`` (in file order) whose time field ``at`` is at or before ``t``; else the first; None for none."""
    best = None
    for r in rows:
        if len(r) > at and int(r[at]) <= t:
            best = r
    return best if best is not None else (rows[0] if rows else None)


def _num(text: str) -> Optional[float]:
    try:
        v = float(text)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def object_stretches(host: Jlog) -> List[Stretch]:
    """Every stretch of the arm touching each object, the table (object 0) included, from the take's ``WC`` records by
    their ``sensed`` times. A ``WC`` lists every pair touching then, so an object it no longer names has been let go."""
    rows = sorted(((int(r[1]), r[2] if len(r) > 2 else "") for r in host.tag("WC")), key=lambda x: x[0])
    open_: Dict[int, List] = {}  # object -> [start, links, peak]
    out: List[Stretch] = []
    for sensed, text in rows:
        now: Dict[int, List] = {}
        for it in items(text):
            if len(it) < 3:
                continue
            obj, link = int(it[2]), int(it[1])
            force = _num(it[3]) if len(it) > 3 else None
            entry = now.setdefault(obj, [set(), 0.0])
            entry[0].add(link)
            entry[1] += force or 0.0
        for obj in [o for o in open_ if o not in now]:
            start, links, peak = open_.pop(obj)
            out.append(Stretch(obj, start, sensed, tuple(sorted(links)), peak))
        for obj, (links, force) in now.items():
            s = open_.setdefault(obj, [sensed, set(), 0.0])
            s[1].update(links)
            s[2] = max(s[2], force)
    for obj, (start, links, peak) in open_.items():
        out.append(Stretch(obj, start, OPEN_END, tuple(sorted(links)), peak))
    return sorted(out, key=lambda s: (s.start, s.object))


def object_poses(host: Jlog) -> Dict[int, List[Tuple[int, Tuple[float, ...]]]]:
    """Each object's ``WO`` poses, (sensed, (x, y, z, qw, qx, qy, qz)), in time order."""
    out: Dict[int, List[Tuple[int, Tuple[float, ...]]]] = {}
    for r in host.tag("WO"):
        sensed = int(r[1])
        for it in items(r[3] if len(r) > 3 else ""):
            if len(it) >= 8:
                out.setdefault(int(it[0]), []).append((sensed, tuple(float(v) for v in it[1:8])))
    for poses in out.values():
        poses.sort(key=lambda p: p[0])
    return out


def pose_at(poses: Sequence[Tuple[int, Tuple[float, ...]]], t: int) -> Optional[Tuple[float, ...]]:
    """The newest pose at or before ``t``, else the first; None without any."""
    i = bisect.bisect_right([s for s, _ in poses], t) - 1
    return poses[i][1] if i >= 0 else (poses[0][1] if poses else None)


def tilt_degrees(a: Sequence[float], b: Sequence[float]) -> float:
    """How far an object tipped from pose ``a`` to ``b``: the angle between the vertical as each pose's own frame sees it
    (as ``safety._tilt_degrees``: a turn about the vertical alone is no tip)."""
    def up(q):
        w, x, y, z = q[3:7]
        return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))
    ua, ub = up(a), up(b)
    return math.degrees(math.acos(max(-1.0, min(1.0, sum(p * q for p, q in zip(ua, ub))))))


def turn_degrees(qa: Sequence[float], qb: Sequence[float]) -> float:
    """The angle of the rotation from orientation ``qa`` to ``qb`` (w, x, y, z), degrees."""
    na = math.sqrt(sum(c * c for c in qa)) or 1.0
    nb = math.sqrt(sum(c * c for c in qb)) or 1.0
    dot = abs(sum(a * b for a, b in zip(qa, qb))) / (na * nb)
    return math.degrees(2 * math.acos(min(1.0, dot)))


def replay(host: Jlog) -> Dict:
    """The take's ``RP`` record (docs/adr/0038 decision 2): ``group`` (joint-drive's id for the source take), ``role``
    (``source`` or ``replay``) and ``edit`` as recorded, with the edit parsed (:func:`parse_edit`). All None for a take
    with none (every take before the mixed script)."""
    rows = host.tag("RP")
    if not rows:
        return {"group": None, "role": None, "edit": None, "editParsed": None}
    r = rows[0]
    if len(r) < 4:
        raise ValueError(f"RP {'|'.join(r)!r} has {len(r)} fields, not t group role edit")
    if r[2] not in ("source", "replay"):
        raise ValueError(f"RP role {r[2]!r} is not source or replay")
    return {"group": r[1], "role": r[2], "edit": r[3], "editParsed": parse_edit(r[3])}


def parse_edit(text: str) -> Dict:
    """An ``RP`` edit (docs/adr/0038 decision 2): ``none``, ``remove:<id>``, ``shift:<id>:<dx>:<dy>`` (m, base frame) or
    ``move:<id>:<x>:<y>:<yawRad>``. Raises ValueError for anything else."""
    parts = text.split(":")
    op = parts[0]
    try:
        if op == "none" and len(parts) == 1:
            return {"op": "none"}
        if op == "remove" and len(parts) == 2:
            return {"op": "remove", "object": int(parts[1])}
        if op == "shift" and len(parts) == 4:
            return {"op": "shift", "object": int(parts[1]), "dx": float(parts[2]), "dy": float(parts[3])}
        if op == "move" and len(parts) == 5:
            return {"op": "move", "object": int(parts[1]), "x": float(parts[2]), "y": float(parts[3]), "yawRad": float(parts[4])}
    except ValueError:
        pass
    raise ValueError(f"RP edit {text!r} is not none, remove:<id>, shift:<id>:<dx>:<dy> or move:<id>:<x>:<y>:<yawRad>")


def planned_moves(host: Jlog) -> List[Dict]:
    """The mixed script's planned moves (workstream M's record, :data:`MOVE_TAGS`), as metadata: each record's fields as
    strings, named by :data:`MOVE_FIELDS` where known (``t`` first), the rest under ``extra``."""
    out = []
    for tag in MOVE_TAGS:
        for r in host.tag(tag):
            names = ("t",) + MOVE_FIELDS
            row = {"tag": tag}
            for i, v in enumerate(r):
                if i < len(names):
                    row[names[i]] = v
                else:
                    row.setdefault("extra", []).append(v)
            out.append(row)
    return out


def _kinds(moves: Sequence[Dict]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for m in moves:
        k = str(m.get("kind", "?"))
        out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items()))


def planned_events(host: Jlog, t_first: int = 0) -> List[Dict]:
    """An old script's planned events (``SE``, docs/adr/0036 amendment 5): ``approachS``, when its approach began, in seconds
    from ``t_first`` (the episode's first frame), and the record's own times in seconds after that."""
    out = []
    for r in host.tag("SE"):
        if len(r) >= 9:
            out.append({"index": int(r[1]), "drawn": int(r[2]), "kind": r[3], "object": int(r[4]),
                        "approachS": round((int(r[0]) - t_first) / 1e9, 3), "finalS": _num(r[5]), "contactS": _num(r[6]),
                        "endS": _num(r[7]), "clearS": _num(r[8])})
    return sorted(out, key=lambda e: e["index"])


# --- the layout ------------------------------------------------------------------------------------------------------------

def _mm(v: float) -> int:
    return int(math.floor(v * 1000.0 + 0.5))


def rotation_matrix(q: Sequence[float]) -> List[List[float]]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def layout_hash(scene_sha256: str, objects: Sequence[Tuple[int, Sequence[float], Sequence[float]]]) -> str:
    """The layout an episode starts from (item 36), as the sha256 (hex) of the UTF-8 bytes of the JSON text

        {"objects":[[id,[sx,sy,sz],[x,y,z],[r00,r01,r02,r10,r11,r12,r20,r21,r22]],...],"scene":"<scene sha256>","v":1}

    written with sorted keys and no spaces (``json.dumps(..., separators=(",", ":"), sort_keys=True)``), where each object
    is one the layout put on the table (``objects``: id, its box dimensions after its size factor in metres, and its pose
    x y z qw qx qy qz at the episode's first frame), sorted by id, and every number is an integer: the dimensions and the
    position in millimetres, the rotation matrix of the (normalised) quaternion in thousandths (0.001 is about 0.06
    degrees), each rounded half up (``floor(v + 0.5)``). Flat clutter (``CL``) is not part of it: nothing collides with it.

    Equal hashes mean the same scene file, the same objects at the same sizes, and the same start poses to a millimetre and
    about 0.1 degree. The same draw placed twice can settle across a rounding boundary and hash differently, so a split
    that must keep a layout on one side groups by the layout seed too (:func:`split`)."""
    rows = []
    for oid, dims, pose in sorted(objects, key=lambda o: int(o[0])):
        r = rotation_matrix(pose[3:7])
        rows.append([int(oid), [_mm(d) for d in dims], [_mm(c) for c in pose[:3]],
                     [int(math.floor(v * 1000.0 + 0.5)) for row in r for v in row]])
    text = json.dumps({"v": 1, "scene": scene_sha256, "objects": rows}, separators=(",", ":"), sort_keys=True)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _reach(o: Dict, movable: bool, edges: Sequence[float], start: Sequence[float]) -> str:
    """``distractor`` for an object the arm is not meant to reach: one a mixed pool draws as a distractor (its ``role``,
    docs/adr/0038 decision 3), or, in a scene without roles, a movable object by the bench's edge (docs/adr/0037 amendment 6);
    else ``in_reach``."""
    role = o.get("role")
    if role:
        return "distractor" if role == "distractor" else "in_reach"
    return "distractor" if (movable and _in_regions(edges, start[0], start[1])) else "in_reach"


def _in_regions(regions: Sequence[float], x: float, y: float) -> bool:
    return any(regions[k] - 1e-9 <= x <= regions[k + 1] + 1e-9 and regions[k + 2] - 1e-9 <= y <= regions[k + 3] + 1e-9
               for k in range(0, len(regions) - 3, 4))


# --- cameras ---------------------------------------------------------------------------------------------------------------

def _matrix_to_quat(m) -> Tuple[float, float, float, float]:
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


def look_at(position: Sequence[float], target: Sequence[float], up=(0.0, 0.0, 1.0)) -> Tuple[float, float, float, float]:
    """A USD camera's orientation (w, x, y, z) at ``position`` looking at ``target``: the renderer's own (teleop_isaac
    ``renderer.look_at``, which this package may not import), checked against recorded ``CM`` poses in the tests."""
    f = [t - p for t, p in zip(target, position)]
    n = math.sqrt(sum(c * c for c in f))
    z = [-c / n for c in f]
    x = [up[1] * z[2] - up[2] * z[1], up[2] * z[0] - up[0] * z[2], up[0] * z[1] - up[1] * z[0]]
    n = math.sqrt(sum(c * c for c in x))
    x = [c / n for c in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return _matrix_to_quat(((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2])))


def nominal_camera(c: Dict) -> Optional[Tuple[Tuple[float, ...], Tuple[float, ...]]]:
    """A scene file camera's pose before any jitter: in the base frame for a fixed camera (``position``, ``lookAt``), in its
    mount link's frame for one on a link (``mountPosition``, ``mountLookAt``, ``mountUp``). None when the file does not say."""
    if c.get("mount"):
        if "mountLookAt" not in c:
            return None
        p = tuple(float(v) for v in c.get("mountPosition", (0.0, 0.0, 0.0)))
        return p, look_at(p, c["mountLookAt"], tuple(c.get("mountUp", (0.0, 0.0, 1.0))))
    if "position" not in c or "lookAt" not in c:
        return None
    p = tuple(float(v) for v in c["position"])
    return p, look_at(p, c["lookAt"])


def cameras(frames: Optional[Jlog], rename: Dict[str, str]) -> List[Dict]:
    """Every camera's intrinsics and pose from ``CM`` (trainer/contract/SESSION_FORMAT.md), named as the dataset names
    them (``rename``: the twin's ``wrist`` is ``front``)."""
    out = []
    for r in (frames.tag("CM") if frames is not None else []):
        if len(r) < 16:
            out.append({"name": rename.get(r[0], r[0]), "recorded": r[0], "width": int(r[1]), "height": int(r[2])})
            continue
        out.append({"name": rename.get(r[0], r[0]), "recorded": r[0], "width": int(r[1]), "height": int(r[2]),
                    "fx": float(r[3]), "fy": float(r[4]), "cx": float(r[5]), "cy": float(r[6]), "near": float(r[7]),
                    "far": float(r[8]), "pose": [float(v) for v in r[9:16]], "mount": (r[16] or None) if len(r) > 16 else None,
                    "poseFrame": (r[18] if len(r) > 18 and r[18] else "base")})
    return out


def camera_jitter(scene: Optional[Dict], cams: Sequence[Dict]) -> Dict[str, Dict]:
    """How far each recorded camera stood from where the scene file puts it (docs/adr/0038 decision 5): the distance in
    mm and the angle of the turn in degrees between its ``CM`` pose and the file's pose (in the same frame: the base, or
    its mount link's). 0 and 0 for a camera that was not jittered; left out when the file or the record lacks a pose."""
    by_id = {c.get("id"): c for c in ((scene or {}).get("cameras") or [])}
    out = {}
    for cam in cams:
        nominal = nominal_camera(by_id.get(cam["recorded"], {})) if cam["recorded"] in by_id else None
        if nominal is None or "pose" not in cam:
            continue
        p, q = cam["pose"][:3], cam["pose"][3:7]
        out[cam["name"]] = {"mm": round(1000.0 * math.dist(p, nominal[0]), 2), "deg": round(turn_degrees(q, nominal[1]), 3)}
    return out


# --- what the take showed --------------------------------------------------------------------------------------------------

def _room(frames: Optional[Jlog], t0: int) -> Tuple[Optional[Dict], Optional[str]]:
    """The room (``EN``) in force at the episode's start, with its fields named as SESSION_FORMAT.md names them and any
    later ones under ``extra``; and its change number (``LK``, ``MT`` and ``CL`` are keyed by it)."""
    if frames is None:
        return None, None
    r = _latest(frames.tag("EN"), t0)
    if r is None:
        return None, None
    names = ("t", "change", "seed", "hdri", "sha256", "yawDeg", "intensity", "settleFrames", "brightness", "gains")
    room: Dict = {}
    for i, v in enumerate(r):
        if i >= len(names):
            room.setdefault("extra", []).append(v)
        elif names[i] == "gains":
            room["gains"] = {it[0]: _num(it[1]) for it in items(v) if len(it) >= 2}
        elif names[i] in ("yawDeg", "intensity", "brightness"):
            room[names[i]] = _num(v)
        elif names[i] in ("change", "settleFrames"):
            room[names[i]] = int(v) if v.lstrip("-").isdigit() else v
        elif names[i] != "t":
            room[names[i]] = v
    return room, r[1] if len(r) > 1 else None


def _by_change(rows: Sequence[Sequence[str]], change: Optional[str]) -> Optional[Sequence[str]]:
    """Of change-keyed rows (``LK``, ``MT``, ``CL``: change first), the one of ``change``, else the last."""
    if not rows:
        return None
    for r in rows:
        if change is not None and r and r[0] == change:
            return r
    return rows[-1]


def _named(text: str) -> Dict[str, str]:
    return {it[0]: ":".join(it[1:]) for it in items(text) if len(it) >= 2}


def _lighting(row: Optional[Sequence[str]]) -> Optional[Dict]:
    """``LT`` (workstream A): the dome's colour temperature, the brightness the metering aimed for, and the key light."""
    if row is None:
        return None
    key = row[3].split(":") if len(row) > 3 and row[3] else None
    return {"domeKelvin": _num(row[1]) if len(row) > 1 else None, "exposure": _num(row[2]) if len(row) > 2 else None,
            "key": None if not key else dict(zip(("azimuthDeg", "elevationDeg", "intensity", "kelvin", "angleDeg"),
                                                 (_num(v) for v in key)))}


def _offsets(row: Optional[Sequence[str]], rename: Dict[str, str]) -> Dict[str, Dict]:
    """``CJ`` (workstream A): each camera's offset from the scene file's pose, by its dataset name."""
    out = {}
    for it in items(row[1] if row is not None and len(row) > 1 else ""):
        if len(it) >= 7:
            out[rename.get(it[0], it[0])] = dict(zip(("dx", "dy", "dz", "pitchDeg", "yawDeg", "rollDeg"), (_num(v) for v in it[1:7])))
    return out


def _colours(row: Optional[Sequence[str]]) -> Optional[Dict]:
    """``OA`` (workstream A): each scanned object's colour change, by object id."""
    if row is None:
        return None
    out = {}
    for it in items(row[1] if len(row) > 1 else ""):
        if len(it) >= 7:
            out[it[0]] = {"method": it[1], "brightness": _num(it[2]), "desaturation": _num(it[3]), "tint": [_num(v) for v in it[4:7]]}
    return out


def _clutter(row: Optional[Sequence[str]]) -> Optional[Dict]:
    if row is None:
        return None
    kinds: Dict[str, int] = {}
    for it in items(row[1] if len(row) > 1 else ""):
        kinds[it[0]] = kinds.get(it[0], 0) + 1
    return {"pieces": sum(kinds.values()), "kinds": dict(sorted(kinds.items()))}


def _jitter(measured: Dict[str, Dict], recorded: Dict[str, Dict]) -> Dict[str, Dict]:
    """Per camera, its pose against the scene file's (``mm``, ``deg``) and, when the take records it, the drawn offset."""
    return {name: {**measured.get(name, {}), **recorded.get(name, {})} for name in list(measured) + [n for n in recorded if n not in measured]}


def _frame_period(take: Take) -> int:
    return 1_000_000_000 // (2 * take.fps)


def frame_flags(take: Take, stretches: Sequence) -> List[int]:
    """1 for each frame whose period (its time, half a frame either side) overlaps one of ``stretches`` (anything with
    ``start`` and ``end``), else 0: the labels' rule (docs/adr/0036 amendment 1)."""
    half = _frame_period(take)
    out = []
    for f in take.frames:
        t = int(f.timing["sensed"])
        out.append(1 if any(c.start <= t + half and c.end >= t - half for c in stretches) else 0)
    return out


def _frame_span(times: Sequence[int], half: int, start: int, end: int) -> Tuple[Optional[int], Optional[int]]:
    hit = [i for i, t in enumerate(times) if start <= t + half and end >= t - half]
    return (hit[0], hit[-1]) if hit else (None, None)


def _joint_speeds(frames: Optional[Jlog]) -> Tuple[List[int], List[float]]:
    """From ``VQ``: each posed state's time and the fastest arm joint's speed there (rad/s, against the state before; the
    gripper's fingers left out). Empty without ``VQ``."""
    rows = []
    for r in (frames.tag("VQ") if frames is not None else []):
        if len(r) >= 3:
            values = {it[0]: float(it[1]) for it in items(r[2]) if len(it) >= 2 and not it[0].startswith("gripper")}
            rows.append((int(r[1]), values))
    rows.sort(key=lambda x: x[0])
    times, speeds = [], []
    for (t0, a), (t1, b) in zip(rows, rows[1:]):
        if t1 <= t0:
            continue
        times.append(t1)
        speeds.append(max((abs(b[k] - a[k]) for k in a if k in b), default=0.0) / ((t1 - t0) / 1e9))
    return times, speeds


def unavoidable(take: Take, in_contact: Sequence[int], obstacle_starts: Sequence[int], stop: StopModel,
                speeds: Tuple[List[int], List[float]]) -> Optional[List[List[int]]]:
    """The frames from which a stop could no longer avoid the next obstacle contact (item 17; provisional until the real
    arm's stop latency is measured), as inclusive ranges. A stop asked for at a frame's time t takes effect after the
    latency L; the arm then slows linearly to rest over T, covering the path it would have covered in T/2 at the speed it
    had, so it comes to rest where the recorded motion was at t + L + T/2. A frame not in contact is unavoidable when the
    next contact starts by then. T is the model's stop time, or the fastest arm joint's speed at the frame over the
    model's deceleration (``VQ``; None when the take has no ``VQ``)."""
    if stop.decel_rad_s2 is not None and not speeds[0]:
        return None
    half = _frame_period(take)
    t_last = int(take.frames[-1].timing["sensed"]) + half
    starts = sorted(s for s in obstacle_starts if s <= t_last)
    marks = []
    for i, f in enumerate(take.frames):
        t = int(f.timing["sensed"])
        if in_contact[i]:
            marks.append(0)
            continue
        k = bisect.bisect_right(starts, t)
        if k >= len(starts):
            marks.append(0)
            continue
        if stop.stop_time_ms is not None:
            stop_ns = stop.stop_time_ms * 1e6
        else:
            j = bisect.bisect_right(speeds[0], t) - 1
            stop_ns = (speeds[1][j] if j >= 0 else speeds[1][0]) / stop.decel_rad_s2 * 1e9
        marks.append(1 if starts[k] - t <= stop.latency_ms * 1e6 + stop_ns / 2 else 0)
    return ranges(marks)


def ranges(flags: Sequence[int]) -> List[List[int]]:
    """The set frames as inclusive [start, end] ranges."""
    out, start = [], None
    for i, v in enumerate(list(flags) + [0]):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append([start, i - 1])
            start = None
    return out


# --- clearance (workstream C) ----------------------------------------------------------------------------------------------

def clearance(take: Take, geometry_dir: Optional[str]):
    """Workstream C's per-frame clearance (``teleop_lerobot.clearance.compute``), or (None, why) when it cannot be had: no
    geometry folder asked for, the module missing, or it raised (``ClearanceUnavailable`` or anything else: a dataset is
    still written, with NaN clearance)."""
    if not geometry_dir:
        return None, "no geometry folder given"
    folder = os.path.expanduser(geometry_dir)
    if not os.path.isdir(folder):
        return None, f"no geometry at {geometry_dir}"
    try:
        from . import clearance as module
    except ImportError as e:
        return None, f"no clearance module ({e})"
    try:
        c = module.compute(take, folder)
    except Exception as e:  # noqa: BLE001  (ClearanceUnavailable, or a fault in it: the dataset gets NaN, and says why)
        return None, f"{type(e).__name__}: {e}"
    n = len(take.frames)
    if any(len(getattr(c, name)) != n for name in ("obstacle_mm", "obstacle_id", "surface_mm")):
        return None, f"clearance has another frame count than the episode's {n}"
    return c, None


def _object_id(v) -> Optional[int]:
    """An obstacle id from the clearance, or None for none within its cutoff (its ``NO_OBSTACLE``, -1) or none at all."""
    f = _finite(v)
    return int(f) if f is not None and f >= 0 else None


def _finite(v) -> Optional[float]:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


# --- one episode -----------------------------------------------------------------------------------------------------------

def episode(take: Take, options: Options = Options(), records: Optional[Records] = None) -> Episode:
    """``take`` as a safety episode: its labels and every sidecar line (docs/adr/0038 decision 6)."""
    from . import safety  # the checker's thresholds and rules; it imports this module lazily too

    rec = records or read_records(take.path)
    host, frames, scene = rec.host, rec.frames, rec.scene
    half = _frame_period(take)
    times = [int(f.timing["sensed"]) for f in take.frames]
    t_first, t_last = times[0], times[-1]
    t0, t1 = t_first - half, t_last + half  # the episode's span: its frames' periods
    rename = {safety.FRONT: "front"}

    all_stretches = [s for s in object_stretches(host) if s.end >= t0 and s.start <= t1]
    obstacle = [s for s in all_stretches if s.object != TABLE]
    table = [s for s in all_stretches if s.object == TABLE]
    in_contact = frame_flags(take, obstacle)
    in_table = frame_flags(take, table)
    labels = [max(a, b) for a, b in zip(in_contact, in_table)] if options.table_unsafe else list(in_contact)

    # The scene: objects on the table, their sizes and poses.
    poses = object_poses(host)
    objects_by_id = {int(o["id"]): o for o in ((scene or {}).get("objects") or [])}
    sizes = {int(it[0]): _num(it[1]) or 1.0 for it in items((_latest(host.tag("WZ"), t_first) or ["", "", ""])[2])
             if len(it) >= 2} if host.tag("WZ") else {}
    if not sizes and frames is not None and frames.tag("SZ"):
        sizes = safety.sizes(frames.tag("SZ"))
    wl = _latest(host.tag("WL"), t_first)
    layout = int(wl[1]) if wl and len(wl) > 1 else None
    layout_seed = int(wl[2]) if wl and len(wl) > 2 and wl[2].lstrip("-").isdigit() else None
    if wl and len(wl) > 3 and wl[3]:
        placed = [int(x) for x in wl[3].split(";") if x]
    else:
        placed = sorted(objects_by_id) or sorted(poses)
    starts = {o: pose_at(poses.get(o, []), t_first) for o in placed}
    placed = [o for o in placed if starts[o] is not None and all(math.isfinite(v) for v in starts[o]) and starts[o][2] > PARKED_Z]
    scene_sha = (rec.session.get("scene") or {}).get("sha256")
    if frames is not None and frames.tag("SC") and len(frames.tag("SC")[0]) > 1:
        scene_sha = scene_sha or frames.tag("SC")[0][1]
    edges = ((scene or {}).get("pool") or {}).get("edgeRegions") or []

    scene_objects, hashed = [], []
    for oid in placed:
        o = objects_by_id.get(oid, {})
        factor = sizes.get(oid, 1.0)
        dims = [float(v) * factor for v in o.get("size", [])] or [0.0, 0.0, 0.0]
        start = starts[oid]
        hashed.append((oid, dims, start))
        mass = o.get("mass")
        movable = o.get("kind", "movable") == "movable"
        changes, last = [], start
        for s, p in poses.get(oid, []):
            if t_first < s <= t1 and (1000.0 * math.dist(p[:3], last[:3]) > CHANGE_MM or turn_degrees(p[3:7], last[3:7]) > CHANGE_DEG):
                changes.append([round((s - t_first) / 1e9, 3)] + [round(v, 6) for v in p])
                last = p
        end = pose_at(poses.get(oid, []), t1)
        scene_objects.append({
            "id": oid, "name": o.get("name"), "kind": "movable" if movable else "fixture",
            "shape": o.get("shape"), "asset": o.get("asset") if o.get("shape") == "mesh" else None,
            "primitive": None if o.get("shape") == "mesh" else o.get("shape"),
            "clutter": bool(o.get("clutter", False)), "sizeFactor": factor, "dimensionsM": [round(d, 6) for d in dims],
            "massKg": None if (mass is None or not movable) else round(float(mass) * factor ** 3, 9),
            "role": o.get("role") or None,
            "reach": _reach(o, movable, edges, start),
            "baseDistanceM": round(math.hypot(start[0], start[1]), 4),
            "start": [round(v, 6) for v in start], "end": [round(v, 6) for v in end] if end else None, "changes": changes})
    lhash = layout_hash(scene_sha or "", hashed) if placed else None

    # Contacts: one line a stretch, and what each did to its object.
    contact_lines = []
    knocked, touched = [], []
    obstacle_first: Dict[int, int] = {}
    for s in obstacle:
        obstacle_first.setdefault(s.object, s.start)
    for s in all_stretches:
        o = objects_by_id.get(s.object, {})
        kind = "table" if s.object == TABLE else ("object" if o.get("kind", "movable") == "movable" else "fixture") if o else "unknown"
        a, b = _frame_span(times, half, s.start, s.end)
        line = {"object": s.object, "name": "table" if s.object == TABLE else o.get("name"), "kind": kind,
                "links": [JETROVER_LINKS[i] if 0 <= i < len(JETROVER_LINKS) else str(i) for i in s.links],
                "linkIndices": list(s.links), "startFrame": a, "endFrame": b,
                "startS": round((s.start - t_first) / 1e9, 3), "endS": None if s.end == OPEN_END else round((s.end - t_first) / 1e9, 3),
                "open": s.end == OPEN_END, "peakForceN": round(s.peak_n, 3),
                "movedCm": None, "tiltDeg": None, "moved": None, "tipped": None}
        if s.object != TABLE:
            p = poses.get(s.object, [])
            later = [x.start for x in obstacle if x.object == s.object and x.start > s.start]
            pa, pb = pose_at(p, s.start), pose_at(p, min(later[0], t1) if later else t1)
            if pa is not None and pb is not None:
                d, tl = math.dist(pa[:3], pb[:3]), tilt_degrees(pa, pb)
                line.update({"movedCm": round(100 * d, 2), "tiltDeg": round(tl, 1), "moved": d >= safety.MOVED_METERS,
                             "tipped": tl >= safety.TIPPED_DEGREES})
        contact_lines.append(line)
    for obj, first in sorted(obstacle_first.items()):
        touched.append(obj)
        p = poses.get(obj, [])
        pa, pb = pose_at(p, first), pose_at(p, t1)
        if pa is not None and pb is not None and (math.dist(pa[:3], pb[:3]) >= safety.MOVED_METERS
                                                  or tilt_degrees(pa, pb) >= safety.TIPPED_DEGREES):
            knocked.append(obj)

    # Clearance, per frame and its minimum.
    c, why = clearance(take, options.geometry_dir)
    cols: Dict[str, list] = {
        "frame_index": list(range(len(times))), "sensed_ns": times, "t_s": [round((t - t_first) / 1e9, 6) for t in times],
        "in_contact": [bool(v) for v in in_contact], "in_contact_table": [bool(v) for v in in_table], "unsafe": list(labels)}
    mins, min_ids, surf_mins = [], [], []
    if c is not None:
        for li, link in enumerate(c.links):
            cols[f"clearance_mm.{link}"] = [_finite(row[li]) for row in c.obstacle_mm]
            cols[f"clearance_id.{link}"] = [_object_id(row[li]) for row in c.obstacle_id]
            cols[f"surface_mm.{link}"] = [_finite(row[li]) for row in c.surface_mm]
        for i in range(len(times)):
            vals = [(v, _object_id(oid)) for v, oid in zip(map(_finite, c.obstacle_mm[i]), c.obstacle_id[i]) if v is not None]
            m = min(vals, key=lambda x: x[0]) if vals else None
            mins.append(m[0] if m else None)
            min_ids.append(m[1] if m else None)
            s = [v for v in map(_finite, c.surface_mm[i]) if v is not None]
            surf_mins.append(min(s) if s else None)
    else:
        mins = min_ids = surf_mins = [None] * len(times)
    cols["min_clearance_mm"], cols["min_clearance_id"], cols["min_surface_mm"] = list(mins), list(min_ids), list(surf_mins)
    starts_ns = sorted(s.start for s in obstacle)
    ttc = []
    for i, t in enumerate(times):
        if in_contact[i]:
            ttc.append(0.0)
            continue
        k = bisect.bisect_right(starts_ns, t)
        ttc.append(round((starts_ns[k] - t) / 1e9, 6) if k < len(starts_ns) and starts_ns[k] <= t1 else None)
    cols["time_to_contact_s"] = ttc
    cols["filled"] = [bool(f.timing.get("filled")) for f in take.frames]
    cols["state_skew_ms"] = [int(f.timing.get("stateAge", 0)) / 1e6 for f in take.frames]
    cols["action_skew_ms"] = [int(f.timing.get("actionAge", 0)) / 1e6 for f in take.frames]

    known = [m for m in mins if m is not None]
    min_clear = min(known) if known else None
    min_clear_id = min_ids[mins.index(min_clear)] if min_clear is not None else None
    if knocked:
        outcome = "knock"
    elif obstacle:
        outcome = "contact"
    elif min_clear is None:
        outcome = "unknown"
    elif min_clear < NEAR_MISS_MM:
        outcome = "near_miss"
    else:
        outcome = "no_contact"

    # Item 24: when the contacts fall in the episode.
    flags = []
    first_s = round((obstacle[0].start - t_first) / 1e9, 3) if obstacle else None
    last_end = max(s.end for s in obstacle) if obstacle else None
    after_s = None if last_end is None else (0.0 if last_end == OPEN_END else round((t_last - last_end) / 1e9, 3))
    if first_s is not None and first_s < EARLY_CONTACT_S:
        flags.append(f"first contact {first_s:.2f} s after the start (under {EARLY_CONTACT_S} s)")
    if after_s is not None and after_s < LATE_CONTACT_S:
        flags.append("still touching at the end" if last_end == OPEN_END else
                     f"last contact ends {after_s:.2f} s before the end (under {LATE_CONTACT_S} s)")

    try:
        rp = replay(host)
    except ValueError as e:
        rp = {"group": None, "role": None, "edit": None, "editParsed": None, "replayError": str(e)}
    room, change = _room(frames, t_first)
    looks = _by_change(frames.tag("LK"), change) if frames is not None else None
    materials = _by_change(frames.tag("MT"), change) if frames is not None else None
    cams = cameras(frames, rename)
    moves = planned_moves(host)
    conditions = rec.session.get("conditions") or {}
    line = {
        "take": os.path.abspath(take.path), "session": take.session_id, "scenario": take.scenario,
        "sceneName": (rec.session.get("scene") or {}).get("name") or (scene or {}).get("name"), "sceneSha256": scene_sha,
        "layout": layout, "layoutSeed": layout_seed, "layoutHash": lhash,
        "outcome": outcome, "tableContact": bool(table), "contacts": len(obstacle), "tableContacts": len(table),
        "touched": touched, "knocked": knocked, "unsafeFrames": int(sum(labels)), "unsafeRanges": len(ranges(labels)),
        "minClearanceMm": None if min_clear is None else round(min_clear, 2), "minClearanceObject": min_clear_id,
        "clearanceMethod": getattr(c, "method", None) if c is not None else None,
        "clearanceCutoffMm": _finite(getattr(c, "cutoff_mm", None)) if c is not None else None,
        "clearanceUnavailable": why,
        "group": rp["group"], "role": rp["role"], "edit": rp["edit"], "editParsed": rp["editParsed"],
        "frames": len(times), "seconds": round(len(times) / take.fps, 3), "filled": sum(cols["filled"]),
        "firstContactS": first_s, "afterLastContactS": after_s, "timingFlags": flags,
        "speedSlider": conditions.get("speedSliderRadPerSec"), "operator": conditions.get("operator"),
        "room": room, "looks": _named(looks[1]) if looks and len(looks) > 1 else None,
        "materials": _named(materials[1]) if materials and len(materials) > 1 else None,
        "clutter": _clutter(_by_change(frames.tag("CL"), change)) if frames is not None else None,
        "cameraJitter": _jitter(camera_jitter(scene, cams), _offsets(_by_change(frames.tag("CJ"), change) if frames is not None else None, rename)),
        "lighting": _lighting(_by_change(frames.tag("LT"), change)) if frames is not None else None,
        "objectColours": _colours(_by_change(frames.tag("OA"), change)) if frames is not None else None,
        "moves": {"count": len(moves), "kinds": _kinds(moves), "records": moves}, "events": planned_events(host, t_first)}
    if "replayError" in rp:
        line["replayError"] = rp["replayError"]
    scene_line = {"take": os.path.abspath(take.path), "sceneName": line["sceneName"], "sceneSha256": scene_sha,
                  "layout": layout, "layoutSeed": layout_seed, "layoutHash": lhash, "objects": scene_objects,
                  "surfaces": [{"id": s.get("id"), "center": s.get("center"), "size": s.get("size"), "collides": s.get("collides", True)}
                               for s in ((scene or {}).get("surfaces") or [])],
                  "cameras": cams}
    unav = None
    if options.stop is not None:
        unav = unavoidable(take, in_contact, starts_ns, options.stop, _joint_speeds(frames))
    return Episode(take.scenario, labels, line, scene_line, contact_lines, cols, unav, flags)


# --- many episodes ---------------------------------------------------------------------------------------------------------

def split(lines: Sequence[Dict], k: int) -> List[List[int]]:
    """``lines`` (episodes.jsonl lines, without their index) in ``k`` groups, as index lists in the given order, for ``k``
    datasets (items 20 and 21). Episodes that share a replay group, a layout hash, or a scene and layout seed (one draw) are
    never parted, so no layout and no replay is on two sides. The parts are filled largest first, each into the dataset with
    the fewest episodes (then the fewest with a contact, then the lowest number). Raises ValueError when there are fewer
    such parts than ``k``."""
    n = len(lines)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    keys: Dict[Tuple, int] = {}
    for i, line in enumerate(lines):
        for key in (("group", line.get("group")), ("hash", line.get("layoutHash")),
                    ("seed", line.get("sceneSha256"), line.get("layoutSeed"))):
            if key[-1] is None:
                continue
            j = keys.setdefault(key, i)
            a, b = find(i), find(j)
            if a != b:
                parent[max(a, b)] = min(a, b)
    parts: Dict[int, List[int]] = {}
    for i in range(n):
        parts.setdefault(find(i), []).append(i)
    if len(parts) < k:
        raise ValueError(f"{n} episode(s) hold {len(parts)} layout(s) and replay group(s) that must stay together: "
                         f"not enough for {k} datasets")
    bins: List[List[int]] = [[] for _ in range(k)]
    hits = [0] * k
    for part in sorted(parts.values(), key=lambda p: (-len(p), p[0])):
        b = min(range(k), key=lambda j: (len(bins[j]), hits[j], j))
        bins[b].extend(part)
        hits[b] += sum(1 for i in part if (lines[i].get("contacts") or 0) > 0)
    return [sorted(b) for b in bins]


def report(lines: Sequence[Dict], contacts: Sequence[Dict], options: Options, split_of: Optional[Tuple[int, int]] = None) -> Dict:
    """A dataset's ``meta/teleop/report.json``: its scenario, outcome and contact counts (item 21)."""
    def count(values):
        out: Dict[str, int] = {}
        for v in values:
            out[str(v)] = out.get(str(v), 0) + 1
        return dict(sorted(out.items()))
    return {
        "episodes": len(lines), "frames": sum(l["frames"] for l in lines), "seconds": round(sum(l["seconds"] for l in lines), 1),
        "scenarios": count(l["scenario"] for l in lines), "outcomes": count(l["outcome"] for l in lines),
        "contactEpisodes": sum(1 for l in lines if l["contacts"]), "tableContactEpisodes": sum(1 for l in lines if l["tableContact"]),
        "contacts": count(c["kind"] for c in contacts), "unsafeFrames": sum(l["unsafeFrames"] for l in lines),
        "unsafeRanges": sum(l["unsafeRanges"] for l in lines),
        "replayGroups": len({l["group"] for l in lines if l.get("group") is not None}),
        "roles": count(l["role"] for l in lines if l.get("role") is not None),
        "layoutHashes": len({l["layoutHash"] for l in lines if l.get("layoutHash")}),
        "layoutSeeds": len({(l["sceneSha256"], l["layoutSeed"]) for l in lines if l.get("layoutSeed") is not None}),
        "clearanceEpisodes": sum(1 for l in lines if l.get("clearanceUnavailable") is None),
        "timingFlagged": [l["episode"] for l in lines if l.get("timingFlags")],
        "split": None if split_of is None else {"part": split_of[0], "of": split_of[1]},
        "options": options.describe()}


def mix(name: str, rep: Dict) -> str:
    """The build's one-line account of a dataset's mix."""
    def kv(d):
        return ", ".join(f"{k} {v}" for k, v in d.items()) or "none"
    return (f"{name}: {rep['episodes']} episode(s), {rep['frames']} frames; scenarios {kv(rep['scenarios'])}; outcomes "
            f"{kv(rep['outcomes'])}; table contact in {rep['tableContactEpisodes']}; contacts {kv(rep['contacts'])}; "
            f"{rep['layoutHashes']} layout(s), {rep['replayGroups']} replay group(s)"
            + (f"; contact timing flagged in {len(rep['timingFlagged'])}" if rep["timingFlagged"] else ""))


def _clean(v):
    """``v`` with every number that is not finite as None: JSON has no NaN, and a reader in another language rejects it."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


def _line(d: Dict) -> str:
    return json.dumps(_clean(d), separators=(",", ":"), allow_nan=False) + "\n"


def write(root: str, written: Sequence[Tuple[int, Episode]], options: Options, split_of: Optional[Tuple[int, int]] = None) -> Dict:
    """Writes the sidecars of ``written`` ((episode index, :class:`Episode`) each, in order) under ``root``/meta/teleop.
    Returns the report."""
    teleop = os.path.join(root, "meta", "teleop")
    os.makedirs(teleop, exist_ok=True)
    lines, contacts = [], []
    with open(os.path.join(teleop, "episodes.jsonl"), "w", encoding="utf-8") as f:
        for ep, e in written:
            line = {"episode": ep, **e.line}
            lines.append(line)
            f.write(_line(line))
    with open(os.path.join(teleop, "scenes.jsonl"), "w", encoding="utf-8") as f:
        for ep, e in written:
            f.write(_line({"episode": ep, **e.scene}))
    with open(os.path.join(teleop, "contacts.jsonl"), "w", encoding="utf-8") as f:
        for ep, e in written:
            for c in e.contacts:
                contacts.append(c)
                f.write(_line({"episode": ep, **c}))
    write_frames(os.path.join(teleop, "frames.parquet"), written)
    rep = report(lines, contacts, options, split_of)
    with open(os.path.join(teleop, "report.json"), "w", encoding="utf-8") as f:
        json.dump(_clean(rep), f, indent=1, allow_nan=False)
    if options.stop is not None:
        s = options.stop
        with open(os.path.join(teleop, "unavoidable_ranges.json"), "w", encoding="utf-8") as f:
            json.dump({"method": "provisional (docs/adr/0038 decision 6, item 17): a stop asked for at a frame takes effect after "
                                 "the latency, then the arm slows linearly to rest over the stop time (or its fastest arm joint's "
                                 "speed over the deceleration), coming to rest where the recorded motion was at latency + half "
                                 "the stop time later; a frame not in contact is unavoidable when the next obstacle contact "
                                 "starts by then. Until the real arm's stop latency is measured.",
                       "stopLatencyMs": s.latency_ms, "stopTimeMs": s.stop_time_ms, "stopDecelRadS2": s.decel_rad_s2,
                       "unmeasured": [ep for ep, e in written if e.unavoidable is None],
                       "unavoidable_ranges": {str(ep): e.unavoidable for ep, e in written if e.unavoidable}}, f, indent=1)
    return rep


def write_frames(path: str, written: Sequence[Tuple[int, Episode]]) -> None:
    """``frames.parquet``: every episode's frame rows, one column set (a per-link column an episode lacks is null there)."""
    import pandas as pd  # lerobot's own dependency

    order: List[str] = []
    for _, e in written:
        for k in e.frames:
            if k not in order:
                order.append(k)
    data: Dict[str, list] = {"episode_index": []}
    for k in order:
        data[k] = []
    for ep, e in written:
        n = len(e.frames["frame_index"])
        data["episode_index"].extend([ep] * n)
        for k in order:
            data[k].extend(e.frames.get(k, [None] * n))
    df = pd.DataFrame({"episode_index": pd.array(data["episode_index"], dtype="int64")})
    for k in order:
        if k in ("frame_index", "sensed_ns"):
            df[k] = pd.array(data[k], dtype="int64")
        elif k == "unsafe":
            df[k] = pd.array(data[k], dtype="int8")
        elif k in ("in_contact", "in_contact_table", "filled"):
            df[k] = pd.array(data[k], dtype="bool")
        elif k.startswith("clearance_id.") or k == "min_clearance_id":
            df[k] = pd.array(data[k], dtype="Int32")
        else:
            df[k] = pd.array([float("nan") if v is None else float(v) for v in data[k]], dtype="float64")
    df.to_parquet(path, index=False)


__all__ = ["Episode", "GEOMETRY_DIR", "NEUTRAL_CAPTION", "Options", "StopModel", "Stretch", "camera_jitter", "cameras",
           "clearance", "episode", "frame_flags", "layout_hash", "look_at", "object_stretches", "parse_edit", "planned_moves",
           "read_records", "replay", "report", "split", "unavoidable", "write", "write_frames"]
