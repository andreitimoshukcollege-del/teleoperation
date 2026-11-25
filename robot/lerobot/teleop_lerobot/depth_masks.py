"""Each camera's depth and instance masks, downsampled, beside a safety dataset (``safety-build --depth-masks WxH``; world-models'
request of 2026-11-25, docs/adr/0038 "V, views", "In the safety datasets"). They tell whether a projected point is hidden in
a camera: its depth against the depth map there, and whose surface the map shows (the mask's id).

**Files.** ``meta/teleop/depth_masks/episode_<NNNNNN>.npz``, one an episode, and ``meta/teleop/depth_masks.json`` describing
them. ``numpy.load(path)`` reads an episode (no pickles). Its arrays:

- ``<camera>.depth``: ``(frames, H, W)`` uint16, the distance to the image plane in mm (trainer/contract/SESSION_FORMAT.md's
  depth stream, exactly: 0 where nothing was hit), for every camera of the dataset that records depth and masks: the room
  cameras, ``front`` (the twin's wrist), and the view cameras when built ``--views``;
- ``<camera>.mask``: the same shape, uint16 stable ids (SESSION_FORMAT.md ``ID``: 7 arm, 8 hand, 10 table, 11 bench, 15 flat
  clutter, ``100 + n`` scene object ``n``, 0 unlabelled);
- ``source_frame`` (int64) and ``filled`` (bool): each episode frame's recorded frame and whether it repeats an earlier render,
  as ``episode_<n>.timing.jsonl`` and ``frames.parquet`` give them. Frame ``i`` of every array is the dataset's frame ``i``.

**Downsampling: nearest, one source pixel for both.** Low-res pixel ``(i, j)`` is the full-res pixel ``(rows[i], cols[j])``,
``rows[i] = floor((i + 0.5) h / H)`` and ``cols[j] = floor((j + 0.5) w / W)``: the source pixel nearest the low-res pixel's
centre (for 640x360 to 160x90, every 4th from the 3rd). Its depth and its id are that one pixel's, never blended, so:

- an id is always one the renderer wrote, and never a mix of two objects' ids (a mode or an average would make one up, or
  let a large neighbour swallow a small object's pixel unpredictably);
- the depth and the id at a pixel are of one surface: a point hidden behind the depth there is hidden behind the object the
  id names. A minimum or a median of a block would pair one pixel's depth with another's id at every edge;
- a reader projects a point with the camera's full-res intrinsics (``scenes.jsonl``) to ``(u, v)`` and reads the low-res
  pixel ``(floor(v H / h), floor(u W / w))``, or ray-casts exactly through ``(cols[j] + 0.5, rows[i] + 0.5)``.

What it costs: a thin object narrower than ``h / H`` pixels can fall between samples, and the depth across an edge is
whichever side the sample hit.

**Storage.** A zip of ``.npy`` arrays, deflated (zlib's default level), written with fixed timestamps, so a rebuild writes
the same bytes. Not LeRobot features: see ``robot/lerobot/README.md`` and docs/adr/0038 for the sizes measured both ways.
"""

from __future__ import annotations

import io
import json
import math
import os
import zipfile
from typing import Dict, List, Optional, Sequence, Tuple

FOLDER = os.path.join("meta", "teleop", "depth_masks")
DESCRIPTOR = os.path.join("meta", "teleop", "depth_masks.json")
STREAMS = ("depth", "mask")
TABLE_ID, ARM_IDS, CLUTTER_ID = 10, (7, 8), 15
METHOD = ("nearest: low-res pixel (i, j) is full-res pixel (floor((i + 0.5) h / H), floor((j + 0.5) w / W)) for both the depth "
          "and the mask, so ids are never blended and a pixel's depth and id are one surface's")
DEPTH = "uint16 mm, the distance to the image plane (SESSION_FORMAT.md depth), 0 where nothing was hit"
MASK = ("uint16 stable ids (SESSION_FORMAT.md ID: 7 arm, 8 hand, 10 table, 11 bench, 15 flat clutter, 100 + n scene object n), "
        "0 unlabelled")
ZIP_TIME = (1980, 1, 1, 0, 0, 0)


def parse_size(text: str) -> Tuple[int, int]:
    """``WxH`` (e.g. ``160x90``) as (W, H)."""
    try:
        w, h = (int(x) for x in str(text).lower().split("x"))
    except ValueError:
        raise ValueError(f"--depth-masks takes WxH, e.g. 160x90, not {text!r}") from None
    if w < 1 or h < 1:
        raise ValueError(f"--depth-masks {text}: both sides at least 1")
    return w, h


def sample_indices(source: int, size: int):
    """The full-res indices the low-res ones sample along one axis: ``floor((k + 0.5) source / size)``."""
    import numpy as np

    if size > source:
        raise ValueError(f"cannot downsample {source} pixels to {size}")
    return np.floor((np.arange(size) + 0.5) * source / size).astype(np.int64)


def downsample(image, size: Tuple[int, int]):
    """``image`` (h, w) at ``size`` (W, H), by :func:`sample_indices` on both axes."""
    import numpy as np

    h, w = image.shape[:2]
    return image[np.ix_(sample_indices(h, size[1]), sample_indices(w, size[0]))]


def cameras_of(take, frames_log, rename: Dict[str, str]) -> List[Tuple[str, str, int, int]]:
    """The dataset's cameras that record depth and masks, in the dataset's order: (dataset name, recorded name, width, height).
    A ``CM`` camera's streams are its record's; a view camera's (``take.cameras``' ``view``) its ``VW``'s."""
    streams: Dict[str, List[str]] = {}
    for r in frames_log.tag("CM"):
        streams[r[0]] = r[17].split(";") if len(r) > 17 else ["rgb"]
    if any(c.view for c in take.cameras):
        from .views import read

        for c in read(frames_log, take.path).cameras:
            streams[c.name] = list(c.streams)
    out = []
    for c in take.cameras:
        if all(s in streams.get(c.name, []) for s in STREAMS):
            out.append((rename.get(c.name, c.name), c.name, c.width, c.height))
    return out


class _Sidecar:
    """One stream's TFRM sidecar, read once."""

    def __init__(self, path: str):
        with open(path, "rb") as f:
            self.data = f.read()

    def image(self, offset: int, length: int, width: int, height: int):
        import cv2
        import numpy as np

        if offset < 12 or offset + length > len(self.data) or int.from_bytes(self.data[offset - 4:offset], "little") != length:
            raise ValueError(f"the record at {offset} ({length} bytes) is not in the sidecar")
        image = cv2.imdecode(np.frombuffer(self.data, np.uint8, count=length, offset=offset), cv2.IMREAD_UNCHANGED)
        if image is None or image.dtype != np.uint16 or image.shape != (height, width):
            raise ValueError(f"the record at {offset} is not a {width}x{height} 16-bit PNG")
        return image


def episode_arrays(take, size: Tuple[int, int], rename: Dict[str, str]) -> Dict[str, object]:
    """The episode's arrays (see the module's notes), in the order they are written."""
    import numpy as np

    from .jlog import read_jlog

    frames_log = read_jlog(os.path.join(take.path, "isaac.jlog"), "isaac")
    cams = cameras_of(take, frames_log, rename)
    if not cams:
        raise ValueError(f"{take.path}: no camera of the episode records depth and masks")
    numbers = [int(f.timing["frame"]) for f in take.frames]
    wanted = set(numbers)
    parts: Dict[int, Dict[str, Tuple[int, int]]] = {}
    for r in frames_log.tag("VF"):
        n = int(r[1])
        if n in wanted and n not in parts:
            parts[n] = {s: (int(o), int(length)) for s, o, length in (p.rsplit(":", 2) for p in r[4].split(";") if p)}
    out: Dict[str, object] = {}
    for name, recorded, width, height in cams:
        for stream in STREAMS:
            sidecar = _Sidecar(os.path.join(take.path, "isaac", f"{recorded}.{stream}.frames"))
            arr = np.zeros((len(numbers), size[1], size[0]), np.uint16)
            done: Dict[int, int] = {}
            for i, n in enumerate(numbers):
                if n in done:
                    arr[i] = arr[done[n]]
                    continue
                part = parts.get(n, {}).get(f"{recorded}.{stream}")
                if part is None:
                    raise ValueError(f"{take.path}: recorded frame {n} has no {recorded}.{stream} part")
                try:
                    arr[i] = downsample(sidecar.image(*part, width, height), size)
                except ValueError as e:
                    raise ValueError(f"{take.path}: frame {n}'s {recorded}.{stream}: {e}") from e
                done[n] = i
            out[f"{name}.{stream}"] = arr
    out["source_frame"] = np.asarray(numbers, np.int64)
    out["filled"] = np.asarray([bool(f.timing["filled"]) for f in take.frames], bool)
    return out


def write_npz(path: str, arrays: Dict[str, object]) -> int:
    """``arrays`` as a ``.npz`` ``numpy.load`` reads, deflated, with fixed timestamps (the same arrays, the same bytes). Returns
    its size."""
    import numpy as np

    tmp = path + ".tmp"
    with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_DEFLATED, allowZip64=True) as z:
        for name, arr in arrays.items():
            info = zipfile.ZipInfo(name + ".npy", date_time=ZIP_TIME)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            buf = io.BytesIO()
            np.lib.format.write_array(buf, np.ascontiguousarray(arr), allow_pickle=False)
            z.writestr(info, buf.getvalue())
    os.replace(tmp, path)
    return os.path.getsize(path)


def episode_path(root: str, episode: int) -> str:
    return os.path.join(root, FOLDER, f"episode_{episode:06d}.npz")


def write_episode(take, path: str, size: Tuple[int, int], rename: Dict[str, str]) -> Dict:
    """One episode's file (a worker's job): its size, frames and cameras."""
    arrays = episode_arrays(take, size, rename)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    n = write_npz(path, arrays)
    cams = [k[:-len(".depth")] for k in arrays if k.endswith(".depth")]
    return {"bytes": n, "frames": len(take.frames), "cameras": cams,
            "sources": {name: [int(arrays[name + ".depth"].shape[2]), int(arrays[name + ".depth"].shape[1])] for name in cams},
            "full": {rename.get(c.name, c.name): [c.width, c.height] for c in take.cameras}}


def _job(args):
    take, path, size, rename = args
    try:
        return write_episode(take, path, size, rename)
    except Exception as e:  # noqa: BLE001  (re-raised with the take's path)
        raise RuntimeError(f"{take.path}: its depth and masks could not be written ({type(e).__name__}: {e})") from e


def write_all(root: str, episodes: Sequence[Tuple[int, object]], size: Tuple[int, int], rename: Dict[str, str], jobs: int = 1,
              executor=None) -> List[Dict]:
    """Every episode's file under ``root`` (``episodes``: (index, take) each), by ``jobs`` spawned workers or ``executor``
    (tests), then ``depth_masks.json``. Returns each episode's :func:`write_episode` summary, in order."""
    work = [(take, episode_path(root, ep), size, rename) for ep, take in episodes]
    if executor is not None and len(work) > 1:
        done = list(executor.map(_job, work))
    elif jobs > 1 and len(work) > 1:
        import multiprocessing
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=min(jobs, len(work)), mp_context=multiprocessing.get_context("spawn")) as pool:
            done = list(pool.map(_job, work))
    else:
        done = [_job(w) for w in work]
    cams: Dict[str, List[int]] = {}
    for d in done:
        for name in d["cameras"]:
            cams.setdefault(name, d["full"][name])
    descriptor = {
        "size": list(size), "files": os.path.join(FOLDER, "episode_<NNNNNN>.npz").replace(os.sep, "/"), "depth": DEPTH,
        "mask": MASK, "method": METHOD,
        "cameras": {name: {"source": wh, "rows": sample_indices(wh[1], size[1]).tolist(),
                           "cols": sample_indices(wh[0], size[0]).tolist()} for name, wh in cams.items()}}
    with open(os.path.join(root, DESCRIPTOR), "w", encoding="utf-8") as f:
        json.dump(descriptor, f, indent=1)
    return done


# --- the projection check -------------------------------------------------------------------------------------------------

def _quat_matrix(q: Sequence[float]):
    import numpy as np

    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


TABLE_EDGE_M = 0.01  # the table top's footprint, this far inside its edges: its sides are table pixels too
TABLE_TOL_MM = 3.0  # a sampled table pixel's depth against the table plane's, through the recorded pose
TABLE_AGREE = 0.99  # share of the pixels showing the table (depth on its plane) whose id is the table's
SEE_THROUGH = 0.01  # share of the table's pixels whose depth is behind its plane: the pose, depth or sample is wrong
ARM_AGREE = 0.85  # share of the arm's unoccluded link-box centres that land on an arm or hand id


def table_check(depth, mask, cam: Dict, pose: Sequence[float], source: Sequence[int], top: float,
                footprint: Tuple[float, float, float, float]) -> Dict[str, int]:
    """One low-res frame's table pixels through ``pose``: each low-res pixel's ray through its sampled full-res pixel's centre
    is cut with the table top's plane (``top``, within ``footprint`` x0 x1 y0 y1). Where that point is the surface the depth
    shows (within :data:`TABLE_TOL_MM`), its id must be the table's (or the flat clutter's on it); a depth behind the plane is
    seeing through the table. Counts: ``onPlane``, ``tableId``, ``behind``, ``inFront`` (something stands on or before it),
    ``predicted``."""
    import numpy as np

    H, W = mask.shape
    rows, cols = sample_indices(source[1], H), sample_indices(source[0], W)
    uu, vv = np.meshgrid(cols + 0.5, rows + 0.5)
    ray = np.stack([(uu - cam["cx"]) / cam["fx"], -(vv - cam["cy"]) / cam["fy"], -np.ones_like(uu)], axis=-1)  # d = 1
    R, p = _quat_matrix(pose[3:7]), np.asarray(pose[:3], float)
    world = ray @ R.T
    with np.errstate(divide="ignore", invalid="ignore"):
        d = (top - p[2]) / world[..., 2]
    hit = p + world * d[..., None]
    x0, x1, y0, y1 = footprint
    on = (d > 0) & (hit[..., 0] > x0) & (hit[..., 0] < x1) & (hit[..., 1] > y0) & (hit[..., 1] < y1)
    got = depth.astype(np.float64)
    want = d * 1000.0
    plane = on & (np.abs(got - want) <= TABLE_TOL_MM)
    behind = on & (got > want + TABLE_TOL_MM)
    front = on & (got > 0) & (got < want - TABLE_TOL_MM)
    table = plane & np.isin(mask, (TABLE_ID, CLUTTER_ID))
    return {"predicted": int(on.sum()), "onPlane": int(plane.sum()), "tableId": int(table.sum()), "behind": int(behind.sum()),
            "inFront": int(front.sum())}


def arm_check(depth, mask, cam: Dict, pose: Sequence[float], source: Sequence[int], centres, radii) -> Dict[str, int]:
    """The arm's link-box centres (base frame, (b, 3)) through ``pose``, each read at the low-res pixel it falls in: an arm or
    hand id there is a hit; a depth more than its box's half-diagonal (``radii``) and 1 cm in front of it is something
    occluding it; otherwise a miss. Counts: ``inImage``, ``hits``, ``occluded``, ``misses``."""
    import numpy as np

    H, W = mask.shape
    R, p = _quat_matrix(pose[3:7]), np.asarray(pose[:3], float)
    c = (np.asarray(centres, float) - p) @ R
    d = -c[:, 2]
    out = {"inImage": 0, "hits": 0, "occluded": 0, "misses": 0}
    for k in range(len(c)):
        if d[k] <= 0.05:
            continue
        u = cam["cx"] + cam["fx"] * c[k, 0] / d[k]
        v = cam["cy"] - cam["fy"] * c[k, 1] / d[k]
        if not (0 <= u < source[0] and 0 <= v < source[1]):
            continue
        i, j = int(v * H / source[1]), int(u * W / source[0])
        out["inImage"] += 1
        if mask[i, j] in ARM_IDS:
            out["hits"] += 1
        elif 0 < depth[i, j] < 1000.0 * (d[k] - radii[k] - 0.01):
            out["occluded"] += 1
        else:
            out["misses"] += 1
    return out


def projection_check(root: str, geometry_dir: Optional[str] = None, frames_per_episode: int = 12) -> Dict:
    """Whether a dataset's downsampled depth and masks and its cameras' recorded poses agree (``safety-views-check``): for
    every camera in the base frame (the room cameras and the views; a moving view at each frame's ``framePoses``), at
    ``frames_per_episode`` frames of each episode, the table's points (:func:`table_check`) and, with ``geometry_dir`` and the
    take's ``VQ``, the arm's (:func:`arm_check`). Returns per camera the counts and shares, and ``problems``."""
    import numpy as np

    with open(os.path.join(root, DESCRIPTOR), encoding="utf-8") as f:
        desc = json.load(f)
    with open(os.path.join(root, "meta", "teleop", "scenes.jsonl"), encoding="utf-8") as f:
        scenes = [json.loads(x) for x in f if x.strip()]
    robot = None
    if geometry_dir:
        from . import clearance as C

        robot = C.load_robot(os.path.expanduser(geometry_dir))
        fixed = robot.fixed or frozenset({"base_link"})
        moving = np.array([robot.links[k] not in fixed for k in robot.box_link], bool)  # the base is the body's, not arm
        radii = np.linalg.norm(robot.box_half, axis=1)[moving]
    per: Dict[str, Dict[str, int]] = {}
    problems: List[str] = []
    for line in scenes:
        ep = int(line["episode"])
        table = next((s for s in line.get("surfaces") or [] if s.get("id") == "table"), None)
        if table is None:
            problems.append(f"episode {ep}: no table surface in scenes.jsonl")
            continue
        top = table["center"][2] + table["size"][2] / 2
        foot = (table["center"][0] - table["size"][0] / 2 + TABLE_EDGE_M, table["center"][0] + table["size"][0] / 2 - TABLE_EDGE_M,
                table["center"][1] - table["size"][1] / 2 + TABLE_EDGE_M, table["center"][1] + table["size"][1] / 2 - TABLE_EDGE_M)
        data = np.load(episode_path(root, ep))
        n = len(data["source_frame"])
        picks = sorted({round(k * (n - 1) / max(1, frames_per_episode - 1)) for k in range(frames_per_episode)}) if n else []
        boxes = {}
        if robot is not None:
            from .jlog import read_jlog

            vq = {}
            for r in read_jlog(os.path.join(line["take"], "isaac.jlog"), "isaac").tag("VQ"):
                vq.setdefault(int(r[0]), {it.split(":")[0]: float(it.split(":")[1]) for it in r[2].split(";") if it})
            numbers = [int(data["source_frame"][i]) for i in picks]
            have = [k for k in numbers if k in vq]
            if have:
                names = sorted({j for k in have for j in vq[k]})
                R, p = robot.link_poses({j: np.array([vq[k].get(j, 0.0) for k in have]) for j in names}, len(have))
                for idx, k in enumerate(have):
                    boxes[k] = robot.boxes(R[idx], p[idx])[0][moving]
        for cam in (line.get("cameras") or []) + (line.get("views") or []):
            name = cam["name"]
            if f"{name}.depth" not in data.files or cam.get("poseFrame", "base") != "base" or "pose" not in cam:
                continue
            source = (cam["width"], cam["height"])
            counts = per.setdefault(name, {})
            for i in picks:
                pose = cam["framePoses"][i] if cam.get("framePoses") else cam["pose"]
                depth, mask = data[f"{name}.depth"][i], data[f"{name}.mask"][i]
                for k, v in table_check(depth, mask, cam, pose, source, top, foot).items():
                    counts["table." + k] = counts.get("table." + k, 0) + v
                frame = int(data["source_frame"][i])
                if frame in boxes:
                    for k, v in arm_check(depth, mask, cam, pose, source, boxes[frame], radii).items():
                        counts["arm." + k] = counts.get("arm." + k, 0) + v
    out = {"size": desc.get("size"), "episodes": len(scenes), "cameras": {}, "problems": problems}
    for name, c in sorted(per.items()):
        on = c.get("table.onPlane", 0)
        pred = c.get("table.predicted", 0)
        r = dict(c)
        r["tableIdShare"] = round(c.get("table.tableId", 0) / on, 4) if on else None
        r["seeThroughShare"] = round(c.get("table.behind", 0) / pred, 4) if pred else None
        unocc = c.get("arm.hits", 0) + c.get("arm.misses", 0)
        r["armHitShare"] = round(c.get("arm.hits", 0) / unocc, 4) if unocc else None
        out["cameras"][name] = r
        if not on:
            problems.append(f"{name}: no table pixel on the table's plane to check")
        elif r["tableIdShare"] < TABLE_AGREE:
            problems.append(f"{name}: {r['tableIdShare']:.3f} of the pixels on the table's plane have the table's id")
        if r["seeThroughShare"] is not None and r["seeThroughShare"] > SEE_THROUGH:
            problems.append(f"{name}: {r['seeThroughShare']:.3f} of the table's pixels are seen through it")
        if r["armHitShare"] is not None and r["armHitShare"] < ARM_AGREE:
            problems.append(f"{name}: {r['armHitShare']:.3f} of the arm's unoccluded box centres land on its ids")
        if robot is not None and r["armHitShare"] is None:
            problems.append(f"{name}: no arm box centre in view to check")
    return out


__all__ = ["DESCRIPTOR", "FOLDER", "METHOD", "arm_check", "cameras_of", "downsample", "episode_arrays", "episode_path",
           "parse_size", "projection_check", "sample_indices", "table_check", "write_all", "write_episode", "write_npz"]
