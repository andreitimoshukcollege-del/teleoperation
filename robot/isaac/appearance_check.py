"""Takes recorded with an appearance section (docs/adr/0038 "A, appearance"), checked from what they recorded. Read-only: run on
the GPU machine from robot/isaac with a Python that has numpy and OpenCV (the LeRobot venv), over take folders:

    ~/isaacsim/lerobot-venv/bin/python appearance_check.py ~/data/takes/jetrover-isaac/mixed/2026112* [--sheet sheet.jpg]

It prints one ``RESULT {...}`` line and exits non-zero when a check fails:

1. **Records.** Every take has ``LT``, ``CJ`` and ``OA`` for the parts its scene's section has, and each equals the draw
   ``teleop_isaac.appearance`` makes from ``BK``'s seed and the change. Each camera's ``CM`` pose is the scene file's pose
   moved by its ``CJ`` offset (within 0.1 mm and 0.01 degree).
2. **The pose recorded is the pose rendered.** Every camera's depth on the table (mask id 10), back-projected through its
   ``CM`` pose (a mounted camera's through its link's forward kinematics at the frame's ``VQ`` values), lands on the table top:
   p99 under 3 mm, as ``verify_session.py`` holds it. The same through the scene file's pose, ignoring the jitter, is reported
   beside it: a 2 degree turn tilts the table by centimetres there, so this tells whether the renderer moved the camera.
3. **Frame pacing** (item 28): the median frame interval, the share of intervals over 55 ms, and frames recorded against the
   recorded span at 30 fps. Before this change (240 takes of 2026-11-22, two twins): 36 ms, 1.6%, 90%.

``--sheet`` writes a contact sheet: each scanned object a row, each take a column, the object's crop from the first still
camera that shows it, captioned with its drawn brightness and desaturation; and the takes' first wrist frames. Whether a
colour change took (MaterialX in particular) is for the eye: a look at the sheet, or a run with exaggerated ranges.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import appearance, kinematics  # noqa: E402
from teleop_isaac import scene as scene_module  # noqa: E402
from teleop_isaac.isaac_session import field  # noqa: E402
from teleop_isaac.renderer import look_at, mounted_camera_local  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
URDF = os.path.join(REPO, "robot", "jetrover", "urdf", "jetrover.urdf")
TABLE_ID = 10
FRAMES_PER_TAKE = 12  # frames sampled a take for the depth check


def records(path):
    out = {}
    with open(path, encoding="ascii", errors="replace") as f:
        for line in f:
            parts = line.rstrip("\n").split("|")
            out.setdefault(parts[0], []).append(parts[1:])
    return out


def quat_matrix(w, x, y, z):
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def angle_deg(q1, q2):
    return math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(q1, q2))))))


def payload(take, stream, part):
    offset, length = part
    with open(os.path.join(take, "isaac", stream + ".frames"), "rb") as f:
        f.seek(offset)
        return f.read(length)


def table_heights(depth_mm, mask, cam, pose):
    """Every table pixel's height (m) in the base frame, back-projected through ``pose`` (position, w-x-y-z)."""
    import numpy as np

    vv, uu = np.nonzero((mask == TABLE_ID) & (depth_mm > 0))
    if len(uu) == 0:
        return np.zeros(0)
    if len(uu) > 4000:
        pick = np.linspace(0, len(uu) - 1, 4000).astype(int)
        vv, uu = vv[pick], uu[pick]
    d = depth_mm[vv, uu].astype(np.float64) / 1000.0
    xc = (uu + 0.5 - cam["cx"]) * d / cam["fx"]
    yc = -(vv + 0.5 - cam["cy"]) * d / cam["fy"]
    zc = -d
    r = np.array(quat_matrix(*pose[1]))
    pts = (r @ np.stack([xc, yc, zc])).T + np.array(pose[0])
    return pts[:, 2]


def check_take(take, joints, sheet_rows, wrist_frames, baseline=False):
    import cv2
    import numpy as np

    scene = scene_module.load(os.path.join(take, "scene.json"))
    spec = appearance.spec(scene)
    rec = records(os.path.join(take, "isaac.jlog"))
    out = {"take": take, "problems": []}
    if spec is None:
        if not baseline:
            out["problems"].append("its scene has no appearance section")
            return out
        spec = appearance.parse({})  # a take from before: nothing drawn, every camera where the scene file puts it
    bk = rec.get("BK", [[]])[-1]
    seed, change = int(bk[2]), int(bk[1])
    out["seed"], out["change"] = seed, change
    cams = {}
    for f in rec.get("CM", []):
        cams[f[0]] = {"fx": float(f[3]), "fy": float(f[4]), "cx": float(f[5]), "cy": float(f[6]),
                      "pose": (tuple(float(v) for v in f[9:12]), tuple(float(v) for v in f[12:16])), "mount": f[16],
                      "streams": f[17].split(";") if len(f) > 17 else []}
    specs = {c["id"]: c for c in scene.extra.get("cameras") or []}

    # 1. The records equal the draws, and CM is the scene file's pose moved by CJ.
    def last(tag):
        rows = [r for r in rec.get(tag, []) if r and r[0] == str(change)]
        return rows[-1] if rows else None

    if spec.lighting is not None:
        lt = last("LT")
        want = appearance.lighting(spec.lighting, seed, change, appearance.key_reference(scene))
        if lt is None or lt[1:] != [field(v) for v in appearance.lt_fields(want)]:
            out["problems"].append(f"LT {lt} is not the draw {[field(v) for v in appearance.lt_fields(want)]}")
    jit = {}
    if spec.room is not None or spec.mounted is not None:
        cj = last("CJ")
        if cj is None:
            out["problems"].append("no CJ")
        else:
            for item in cj[1].split(";"):
                cid, *vals = item.split(":")
                jit[cid] = appearance.CameraJitter(*(float(v) for v in vals))
                want = appearance.camera_jitter(spec.jitter_for(bool(specs.get(cid, {}).get("mount"))), seed, change, cid)
                if jit[cid] != want:
                    out["problems"].append(f"CJ {cid} {jit[cid]} is not the draw {want}")
    nominal, worst = {}, [0.0, 0.0]
    for cid, cam in cams.items():
        c = specs.get(cid)
        if c is None:
            out["problems"].append(f"camera {cid} is not in the scene file")
            continue
        nominal[cid] = mounted_camera_local(c) if c.get("mount") else (tuple(c["position"]), look_at(c["position"], c["lookAt"]))
        moved = appearance.jittered(nominal[cid], jit.get(cid, appearance.NO_JITTER))
        mm = 1000.0 * math.dist(moved[0], cam["pose"][0])
        deg = angle_deg(moved[1], cam["pose"][1])
        worst = [max(worst[0], mm), max(worst[1], deg)]
        if mm > 0.1 or deg > 0.01:
            out["problems"].append(f"CM {cid} is {mm:.3f} mm and {deg:.4f} deg from the scene file's pose moved by CJ")
    out["cm_vs_cj"] = [round(worst[0], 4), round(worst[1], 5)]
    if spec.objects is not None:
        oa = last("OA")
        if oa is None:
            out["problems"].append("no OA")
        else:
            for item in oa[1].split(";") if oa[1] else []:
                oid, method, b, d, *tint = item.split(":")
                want = appearance.object_colour(spec.objects, seed, change, int(oid))
                d_want = 0.0 if method == "preview" else want.desaturation
                if (float(b), float(d), tuple(float(t) for t in tint)) != (want.brightness, d_want, want.tint):
                    out["problems"].append(f"OA {oid} is not the draw {want}")
            out["objects_varied"] = len(oa[1].split(";")) if oa[1] else 0

    # 2. Depth on the table, through CM and through the scene file's pose.
    vq = {int(r[0]): {k: float(v) for k, v in (it.split(":") for it in r[2].split(";") if it)} for r in rec.get("VQ", [])}
    vf = [r for r in rec.get("VF", []) if "B" not in r[3]]
    picks = [vf[int(i)] for i in np.linspace(0, len(vf) - 1, min(FRAMES_PER_TAKE, len(vf)))] if vf else []
    err = {cid: {"cm": [], "file": []} for cid in cams}
    top = scene.table.top
    for r in picks:
        frame = int(r[1])
        parts = {}
        for p in r[4].split(";"):
            if p:
                name, off, length = p.rsplit(":", 2)
                parts[name] = (int(off), int(length))
        values = vq.get(frame, {})
        for cid, cam in cams.items():
            if f"{cid}.depth" not in parts or f"{cid}.mask" not in parts or cid not in nominal:
                continue
            depth = cv2.imdecode(np.frombuffer(payload(take, f"{cid}.depth", parts[f"{cid}.depth"]), np.uint8), cv2.IMREAD_UNCHANGED)
            mask = cv2.imdecode(np.frombuffer(payload(take, f"{cid}.mask", parts[f"{cid}.mask"]), np.uint8), cv2.IMREAD_UNCHANGED)
            for which, pose in (("cm", cam["pose"]), ("file", nominal[cid])):
                if cam["mount"]:
                    link = kinematics.link_pose(joints, values, cam["mount"])
                    d = kinematics.q_rotate(link[1], pose[0])
                    pose = (tuple(a + b for a, b in zip(link[0], d)), kinematics.q_mul(link[1], tuple(pose[1])))
                h = table_heights(depth, mask, cam, pose)
                err[cid][which].extend(np.abs(h - top).tolist())
    out["table_mm"] = {}
    for cid, e in err.items():
        if not e["cm"]:
            continue
        p99 = lambda xs: 1000.0 * float(np.percentile(xs, 99))  # noqa: E731
        out["table_mm"][cid] = {"cm_p99": round(p99(e["cm"]), 2), "file_p99": round(p99(e["file"]), 2), "pixels": len(e["cm"])}
        if p99(e["cm"]) >= 3.0:
            out["problems"].append(f"{cid}: table depth through CM is {p99(e['cm']):.1f} mm off at p99 (>= 3)")
    if not out["table_mm"]:
        out["problems"].append("no camera saw the table in depth")

    # 3. Frame pacing.
    sensed = [int(r[2]) for r in rec.get("VF", [])]
    gaps = [(b - a) / 1e6 for a, b in zip(sensed, sensed[1:])]
    if gaps:
        out["pacing"] = {"frames": len(sensed), "median_ms": round(statistics.median(gaps), 2),
                         "over_55ms": round(sum(g > 55 for g in gaps) / len(gaps), 4),
                         "recorded_share": round(len(sensed) / ((sensed[-1] - sensed[0]) / 1e9 * 30 + 1), 3)}

    # The sheet: each scanned object's crop from the first still camera frame that shows it, and the first wrist frame.
    if sheet_rows is not None and picks:
        r = picks[len(picks) // 2]
        parts = {p.rsplit(":", 2)[0]: (int(p.rsplit(":", 2)[1]), int(p.rsplit(":", 2)[2])) for p in r[4].split(";") if p}
        oa = {int(it.split(":")[0]): it.split(":") for it in (last("OA") or ["", ""])[1].split(";") if it}
        for cid, cam in cams.items():
            if cam["mount"] and f"{cid}.rgb" in parts and cid not in wrist_frames.setdefault(take, {}):
                wrist_frames[take][cid] = cv2.imdecode(np.frombuffer(payload(take, f"{cid}.rgb", parts[f"{cid}.rgb"]), np.uint8),
                                                       cv2.IMREAD_COLOR)
        for cid, cam in cams.items():
            if cam["mount"] or f"{cid}.mask" not in parts or f"{cid}.rgb" not in parts:
                continue
            mask = cv2.imdecode(np.frombuffer(payload(take, f"{cid}.mask", parts[f"{cid}.mask"]), np.uint8), cv2.IMREAD_UNCHANGED)
            rgb = cv2.imdecode(np.frombuffer(payload(take, f"{cid}.rgb", parts[f"{cid}.rgb"]), np.uint8), cv2.IMREAD_COLOR)
            for oid in sorted({int(v) - 100 for v in np.unique(mask) if 100 <= v < 400}):  # every scene object in view
                if (take, oid) in sheet_rows:
                    continue
                vv, uu = np.nonzero(mask == 100 + oid)
                if len(uu) < 150:
                    continue
                y0, y1, x0, x1 = vv.min(), vv.max() + 1, uu.min(), uu.max() + 1
                caption = f"b{oa[oid][2]} d{oa[oid][3]}" if oid in oa else cid
                sheet_rows[(take, oid)] = (rgb[y0:y1, x0:x1].copy(), caption)
    return out


def write_sheet(path, takes, sheet_rows, wrist_frames):
    import cv2
    import numpy as np

    cell = 120
    oids = sorted({oid for _, oid in sheet_rows})
    rows = []
    for oid in oids:
        row = []
        for t in takes:
            tile = np.full((cell, cell, 3), 40, np.uint8)
            got = sheet_rows.get((t, oid))
            if got is not None:
                img, caption = got
                k = (cell - 18) / max(img.shape[:2])
                small = cv2.resize(img, (max(1, int(img.shape[1] * k)), max(1, int(img.shape[0] * k))), interpolation=cv2.INTER_AREA)
                tile[:small.shape[0], :small.shape[1]] = small
                cv2.putText(tile, caption, (2, cell - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            row.append(tile)
        label = np.full((cell, 60, 3), 40, np.uint8)
        cv2.putText(label, str(oid), (4, cell // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        rows.append(np.hstack([label] + row))
    wrist = []
    for t in takes:
        frames = list(wrist_frames.get(t, {}).values())
        tile = cv2.resize(frames[0], (cell, cell * 9 // 16)) if frames else np.full((cell * 9 // 16, cell, 3), 40, np.uint8)
        wrist.append(np.vstack([tile, np.full((cell - tile.shape[0], cell, 3), 40, np.uint8)]))
    if wrist:
        rows.append(np.hstack([np.full((cell, 60, 3), 40, np.uint8)] + wrist))
    if rows:
        cv2.imwrite(path, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="takes recorded with an appearance section, checked from their records and frames")
    ap.add_argument("takes", nargs="+")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--sheet", default="", help="write a contact sheet of each scanned object across the takes (JPEG)")
    ap.add_argument("--baseline", action="store_true", help="also take takes whose scene has no appearance section (nothing drawn): "
                                                            "their CM must be the scene file's poses, and the depth and pacing checks run")
    args = ap.parse_args(argv)
    joints = kinematics.read_urdf(args.urdf)
    takes = [t.rstrip("/") for t in args.takes if os.path.exists(os.path.join(t, "isaac.jlog"))]
    sheet_rows, wrist_frames = ({}, {}) if args.sheet else (None, {})
    results = [check_take(t, joints, sheet_rows, wrist_frames, args.baseline) for t in takes]
    if args.sheet:
        write_sheet(args.sheet, takes, sheet_rows, wrist_frames)
    pace = [r["pacing"] for r in results if "pacing" in r]
    summary = {
        "takes": len(results),
        "takes_with_problems": sum(1 for r in results if r["problems"]),
        "problems": [f"{os.path.basename(r['take'])}: {p}" for r in results for p in r["problems"]][:40],
        "table_mm_cm_p99_worst": max((v["cm_p99"] for r in results for v in r.get("table_mm", {}).values()), default=None),
        "table_mm_file_p99_median": statistics.median([v["file_p99"] for r in results for v in r.get("table_mm", {}).values()])
        if any(r.get("table_mm") for r in results) else None,
        "pacing_median_ms": statistics.median([p["median_ms"] for p in pace]) if pace else None,
        "pacing_over_55ms": round(sum(p["over_55ms"] * p["frames"] for p in pace) / max(1, sum(p["frames"] for p in pace)), 4) if pace else None,
        "pacing_recorded_share": round(statistics.median([p["recorded_share"] for p in pace]), 3) if pace else None,
        "per_take": results,
    }
    print("RESULT " + json.dumps(summary))
    return 0 if results and summary["takes_with_problems"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
