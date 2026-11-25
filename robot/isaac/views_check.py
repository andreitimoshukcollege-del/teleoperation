"""Check takes recorded with view cameras (docs/adr/0038 "V, views"), read-only. Run on the GPU machine with the LeRobot
venv (numpy, OpenCV), from robot/isaac:

    ~/isaacsim/lerobot-venv/bin/python views_check.py <take>... [--sheet /tmp/views-sheet.jpg]

For each take:

1. **Records against the draws.** It needs ``isaac/views.json``, one ``VS`` and a ``VW`` per camera, and ``VW`` must equal
   ``views.draw`` of the spec, the scene, ``BK``'s seed and the change: the same place, look-at point, field of view, mode and
   motion, and the same pose to 0.01 mm and 0.001°. It also needs no fallback draw, and with a spec's held-out region
   (``heldOut``, ``split``) every view on its split's side of it.
2. **Every recorded frame carries every view's streams**, and a ``VP`` for each moving view.
3. **Each view's pose in its own depth.** Table pixels (mask id 10) are back-projected through the view's pose, the ``VW``
   pose for a static view and the frame's ``VP`` for a moving one. On the table top's footprint they must land on the top
   within 3 mm at p99. For a moving view, the frames where it moved most (at least 2 mm against each neighbour) are also
   back-projected through the ``VP`` of the frames before and after. If the frame association is right, its own frame
   fits best (the lowest mean height error).
4. **What the views see:** each view's share of pixels on the table, the arm (7, 8) and the objects (100+).

Across the takes given, **each counterfactual replay must have exactly its source's views** (docs/adr/0038 V, views in safety
batches). Takes are paired by ``robothost.jlog``'s ``RP`` (the group, the role); a replay's ``BK`` seed and change, its ``VS``
and its ``VW`` records (pose, intrinsics, draw, gain) must equal its source's. A replay whose source is not among the takes
is counted, not checked.

And **each replay group must have a view of its own** (docs/adr/0038 V, "A new view for every source take"): a take outside
any group is a group of its own, and two groups whose takes have the same viewpoint (every view camera's size, intrinsics
and pose in ``VW``) are a problem when the spec promises a new view each change (a drawn slot, or a fixed one with jitter;
fixed views without jitter stand still by design and are only counted). The summary gives, per twin (the takes root, e.g.
``jetrover-isaac-twin1``), its takes, replay groups and distinct viewpoints: with a drawn spec, viewpoints equal groups.

``--sheet`` writes a contact sheet: a row per take, a column per view, each the take's first recorded frame captioned with
the view's mode, azimuth, elevation, distance and field of view.

It prints one ``RESULT {...}`` line and exits non-zero if any take has a problem (CLAUDE.md invariant 10: nothing measured
is a failure, never a pass).
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

from appearance_check import payload, records, table_heights, table_top  # noqa: E402
from teleop_isaac import appearance, views  # noqa: E402
from teleop_isaac import scene as scene_module  # noqa: E402

TABLE_ID, ARM_IDS, OBJECT_BASE = 10, (7, 8), 100
DEPTH_P99_MM = 3.0
POSE_MM, POSE_DEG = 0.01, 0.001
FRAMES_PER_TAKE = 12
MIN_STEP_MM = 2.0  # a moving view's frames used for the association: moved at least this far against each neighbour


def parts_of(vf):
    out = {}
    for p in vf[4].split(";"):
        if p:
            name, off, length = p.rsplit(":", 2)
            out[name] = (int(off), int(length))
    return out


def vp_poses(row):
    """A VP record's poses: camera -> ((x, y, z), (w, x, y, z))."""
    out = {}
    for it in row[2].split(";"):
        if it:
            f = it.split(":")
            v = [float(x) for x in f[1:8]]
            out[f[0]] = (tuple(v[:3]), tuple(v[3:]))
    return out


def p99(xs):
    s = sorted(xs)
    return s[min(len(s) - 1, int(0.99 * len(s)))] if s else float("nan")


def check_take(take, sheet):
    import cv2
    import numpy as np

    out = {"take": take, "problems": [], "views": {}}
    rec = records(os.path.join(take, "isaac.jlog"))
    path = os.path.join(take, "isaac", "views.json")
    if not os.path.exists(path) or not rec.get("VS") or not rec.get("VW"):
        out["problems"].append("no views.json, VS or VW: not a take with view cameras")
        return out
    with open(path, encoding="utf-8") as f:
        used = json.load(f)
    spec = views.parse(used["spec"])
    if spec.sha256 != used["sha256"] or rec["VS"][-1][2] != spec.sha256:
        out["problems"].append("views.json's spec does not hash to VS's sha256")
    scene = scene_module.load(os.path.join(take, "scene.json"))
    bk = rec["BK"][-1]
    seed, change = int(bk[2]), int(bk[1])
    vw = [views.parse_vw(r) for r in rec["VW"] if int(r[0]) == change]
    drawn = views.draw(spec, views.geometry(spec, scene), seed, change)
    if [r["camera"] for r in vw] != [v.id for v in drawn]:
        out["problems"].append(f"VW cameras {[r['camera'] for r in vw]} are not the drawn {[v.id for v in drawn]}")
        return out
    for r, v in zip(vw, drawn):
        p, q = v.centre_pose()
        mm = 1000.0 * math.dist(r["pose"][:3], p)
        deg = appearance.turn_deg(r["pose"][3:], q)
        same = (r["hfovDeg"], r["azimuthDeg"], r["elevationDeg"], r["distanceM"], r["rollDeg"], r["target"], r["mode"]) == \
               (v.hfov, v.azimuth, v.elevation, v.distance, v.roll, v.target, v.mode)
        if not same or mm > POSE_MM or deg > POSE_DEG:
            out["problems"].append(f"{v.id}'s VW is not its draw ({mm:.4f} mm, {deg:.5f} deg)")
        if r["attempts"] == views.FALLBACK:
            out["problems"].append(f"{v.id} kept a draw that broke a rule")
        out["views"][v.id] = {"mode": v.mode, "azimuthDeg": v.azimuth, "elevationDeg": v.elevation, "distanceM": v.distance,
                              "hfovDeg": v.hfov, "size": [v.width, v.height], "gain": r["gain"], "attempts": r["attempts"],
                              "broken": r["broken"]}
        if list(v.broken) != r["broken"]:
            out["problems"].append(f"{v.id}'s VW broken rules {r['broken']} are not the check's {list(v.broken)}")
        if spec.split:  # a train spec's views outside its held-out region, a test spec's inside it
            side = views.split_of(spec.doc, r["azimuthDeg"], r["elevationDeg"])
            out["views"][v.id]["split"] = side
            if side != spec.split:
                out["problems"].append(f"{v.id} is on the {side} side of the held-out region, in a {spec.split} spec")

    vfs = rec.get("VF", [])
    vps = {int(r[0]): vp_poses(r) for r in rec.get("VP", [])}
    moving = [v for v in drawn if v.motion is not None]
    missing_streams = sum(1 for r in vfs if any(f"{v.id}.{st}" not in parts_of(r) for v in drawn for st in spec.record))
    missing_vp = sum(1 for r in vfs if moving and any(v.id not in vps.get(int(r[1]), {}) for v in moving))
    out["frames"] = len(vfs)
    if not vfs:
        out["problems"].append("no recorded frames")
        return out
    if missing_streams:
        out["problems"].append(f"{missing_streams} of {len(vfs)} frames lack a view's stream")
    if missing_vp:
        out["problems"].append(f"{missing_vp} of {len(vfs)} frames lack a moving view's VP")

    top, *footprint = table_top(scene)
    by_frame = {int(r[1]): r for r in vfs}
    picks = [vfs[i] for i in sorted({round(k * (len(vfs) - 1) / max(1, FRAMES_PER_TAKE - 1)) for k in range(FRAMES_PER_TAKE)})]
    images = {}

    def image(v, frame):
        """(depth mm, mask) of view ``v`` at recorded frame ``frame``, or None."""
        key = (v.id, frame)
        if key not in images:
            parts = parts_of(by_frame[frame]) if frame in by_frame else {}
            if f"{v.id}.depth" not in parts or f"{v.id}.mask" not in parts:
                images[key] = None
            else:
                images[key] = tuple(cv2.imdecode(np.frombuffer(payload(take, f"{v.id}.{st}", parts[f"{v.id}.{st}"]), np.uint8),
                                                 cv2.IMREAD_UNCHANGED) for st in ("depth", "mask"))
        return images[key]

    for r, v in zip(vw, drawn):
        cam = {"fx": r["fx"], "fy": r["fy"], "cx": r["cx"], "cy": r["cy"]}
        errs, shares = [], {"table": [], "arm": [], "objects": []}
        for vf in picks:
            frame = int(vf[1])
            got = image(v, frame)
            if got is None:
                continue
            depth, mask = got
            n = float(mask.size)
            shares["table"].append(float((mask == TABLE_ID).sum()) / n)
            shares["arm"].append(float(np.isin(mask, ARM_IDS).sum()) / n)
            shares["objects"].append(float((mask >= OBJECT_BASE).sum()) / n)
            pose = (r["pose"][:3], r["pose"][3:]) if v.motion is None else vps.get(frame, {}).get(v.id)
            if pose is not None:
                errs.extend(1000.0 * abs(h - top) for h in table_heights(depth, mask, cam, pose, footprint))
        info = out["views"][v.id]
        info.update({"depth_p99_mm": round(p99(errs), 3) if errs else None,
                     **{f"{k}_share": round(statistics.median(xs), 4) if xs else None for k, xs in shares.items()}})
        if not errs:
            out["problems"].append(f"{v.id}: no table pixels on the top's footprint to check its pose with")
        elif info["depth_p99_mm"] > DEPTH_P99_MM:
            out["problems"].append(f"{v.id}: table depth through its pose is {info['depth_p99_mm']} mm off at p99")
        if v.motion is not None:
            # The frame association: the frames where the camera moved most against both neighbours, each back-projected
            # through its own VP and through its neighbours'. Its own must fit best (mean height error).
            def step(a, b):
                pa, pb = vps.get(a, {}).get(v.id), vps.get(b, {}).get(v.id)
                return 1000.0 * math.dist(pa[0], pb[0]) if pa and pb else 0.0
            fast = sorted(((min(step(f, f - 1), step(f, f + 1)), f) for f in by_frame), reverse=True)
            fast = [f for moved, f in fast if moved >= MIN_STEP_MM][:FRAMES_PER_TAKE]
            means = {}
            for off in (-1, 0, 1):
                xs = []
                for f in fast:
                    got, pose = image(v, f), vps.get(f + off, {}).get(v.id)
                    if got is not None and pose is not None:
                        xs.extend(1000.0 * abs(h - top) for h in table_heights(got[0], got[1], cam, pose, footprint))
                if xs:
                    means[off] = round(statistics.fmean(xs), 3)
            info["association"] = {"frames": len(fast), "mean_mm_by_offset": means}
            if len(means) == 3 and min(means[-1], means[1]) < means[0]:
                out["problems"].append(f"{v.id}: a neighbouring frame's VP fits its depth better ({means}): the frame association is off")
    if sheet is not None:
        first = vfs[0]
        row = []
        for v in drawn:
            part = parts_of(first).get(f"{v.id}.rgb")
            img = cv2.imdecode(np.frombuffer(payload(take, f"{v.id}.rgb", part), np.uint8), cv2.IMREAD_COLOR) if part else None
            row.append((v, img))
        sheet.append((os.path.basename(take), row))
    return out


def view_records(take):
    """(BK's seed and change, the last VS, the VW rows of that change) of a take, as recorded."""
    rec = records(os.path.join(take, "isaac.jlog"))
    if not rec.get("BK") or not rec.get("VS"):
        return None
    bk = rec["BK"][-1]
    change = bk[1]
    return (bk[2], change), rec["VS"][-1], [r for r in rec.get("VW", []) if r[0] == change]


def replay_role(take):
    """(group, role) from the take's RP record, or None for a take outside a replay group."""
    path = os.path.join(take, "robothost.jlog")
    if not os.path.exists(path):
        return None
    rp = records(path).get("RP")
    return (rp[0][1], rp[0][2]) if rp and len(rp[0]) >= 3 else None


def check_replays(takes):
    """Every replay among ``takes`` against its source among them: the same BK seed and change, VS and VW rows."""
    roles = {t: replay_role(t) for t in takes}
    sources = {r[0]: t for t, r in roles.items() if r and r[1] == "source"}
    out = {"pairs": 0, "identical": 0, "without_source": 0, "problems": []}
    for t, r in roles.items():
        if not r or r[1] != "replay":
            continue
        src = sources.get(r[0])
        if src is None:
            out["without_source"] += 1
            continue
        out["pairs"] += 1
        a, b = view_records(src), view_records(t)
        if a is None or b is None:
            out["problems"].append(f"replay {os.path.basename(t)} or its source has no BK or VS")
        elif a != b:
            what = [n for n, x, y in zip(("BK seed and change", "VS", "VW"), a, b) if x != y]
            out["problems"].append(f"replay {os.path.basename(t)} (group {r[0]}) does not have its source's views: {', '.join(what)} differ")
        else:
            out["identical"] += 1
    return out


def twin_of(take):
    """The takes root a take was recorded into (one per twin, e.g. jetrover-isaac, jetrover-isaac-twin1):
    <root>/<category>/<take>, or <root>/_discarded/<category>/<take> for one the checker set aside."""
    parts = os.path.normpath(os.path.abspath(take)).split(os.sep)
    if len(parts) >= 4 and parts[-4] != "" and parts[-3] == "_discarded":
        return parts[-4]
    return parts[-3] if len(parts) >= 3 else ""


def promises_new_views(spec):
    """Whether the spec draws its views afresh at every change: a drawn slot, or a fixed one with jitter."""
    return any(spec.slot(k).fixed is None or spec.slot(k).fixed.jitter_m or spec.slot(k).fixed.jitter_deg
               for k in range(spec.count))


def check_groups(takes):
    """Each replay group's viewpoint against the other groups', and the distinct viewpoints per twin. A viewpoint is every
    view camera's VW size, intrinsics and pose (not its change or gain); a take outside a group is a group of its own."""
    out = {"twins": {}, "groups_sharing_a_view": 0, "problems": []}
    holders = {}  # viewpoint -> {group: first take}
    promised = {}
    for t in takes:
        got = view_records(t)
        if got is None:
            continue
        path = os.path.join(t, "isaac", "views.json")
        try:
            with open(path, encoding="utf-8") as f:
                spec = views.parse(json.load(f)["spec"])
        except (OSError, ValueError, KeyError):
            continue
        role = replay_role(t)
        group = f"RP {role[0]}" if role else os.path.basename(t)
        twin = twin_of(t)
        viewpoint = tuple(tuple(r[1:17]) for r in got[2])
        info = out["twins"].setdefault(twin, {"takes": 0, "groups": set(), "viewpoints": set()})
        info["takes"] += 1
        info["groups"].add(group)
        info["viewpoints"].add(viewpoint)
        holders.setdefault(viewpoint, {}).setdefault(group, t)
        promised[viewpoint] = promised.get(viewpoint, False) or promises_new_views(spec)
    for viewpoint, groups in holders.items():
        if len(groups) > 1 and promised[viewpoint]:
            out["groups_sharing_a_view"] += len(groups)
            names = ", ".join(f"{g} ({os.path.basename(t)})" for g, t in sorted(groups.items()))
            out["problems"].append(f"{len(groups)} replay groups share one view, where the spec draws a new one each: {names}")
    out["twins"] = {k: {"takes": v["takes"], "groups": len(v["groups"]), "viewpoints": len(v["viewpoints"])}
                    for k, v in sorted(out["twins"].items())}
    return out


def write_sheet(path, rows):
    import cv2
    import numpy as np

    w, h = 320, 180
    tiles = []
    for name, row in rows:
        cells = []
        for v, img in row:
            cell = np.zeros((h + 36, w, 3), np.uint8)
            if img is not None:
                cell[:h] = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            cv2.putText(cell, f"{name[-14:]} {v.id} {v.mode}", (4, h + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(cell, f"az {v.azimuth:.0f} el {v.elevation:.0f} d {v.distance:.2f} hfov {v.hfov:.0f} {v.width}x{v.height}",
                        (4, h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            cells.append(cell)
        tiles.append(np.hstack(cells))
    width = max(t.shape[1] for t in tiles)
    tiles = [np.pad(t, ((0, 0), (0, width - t.shape[1]), (0, 0))) for t in tiles]
    cv2.imwrite(path, np.vstack(tiles))


def main(argv=None) -> int:
    a = argparse.ArgumentParser(description="check takes recorded with view cameras (docs/adr/0038 V)")
    a.add_argument("takes", nargs="+")
    a.add_argument("--sheet", default="")
    args = a.parse_args(argv)
    sheet = [] if args.sheet else None
    results = [check_take(t, sheet) for t in args.takes]
    for r in results:
        print(json.dumps(r, sort_keys=True))
    if args.sheet and sheet:
        write_sheet(args.sheet, sheet)
    replays = check_replays(args.takes)
    groups = check_groups(args.takes)
    depth = [v["depth_p99_mm"] for r in results for v in r["views"].values() if v.get("depth_p99_mm") is not None]
    modes = [v["mode"] for r in results for v in r["views"].values()]
    summary = {"takes": len(results), "with_problems": sum(1 for r in results if r["problems"]),
               "views": len(modes), "moving": modes.count("moving"),
               "depth_p99_mm_worst": max(depth) if depth else None,
               "replays": {k: replays[k] for k in ("pairs", "identical", "without_source")},
               "viewpoints": groups["twins"], "groups_sharing_a_view": groups["groups_sharing_a_view"],
               "problems": ([p for r in results for p in r["problems"]] + replays["problems"] + groups["problems"])[:20]}
    print("RESULT " + json.dumps(summary, sort_keys=True))
    return 0 if results and not summary["with_problems"] and not replays["problems"] and not groups["problems"] else 1


if __name__ == "__main__":
    sys.exit(main())
