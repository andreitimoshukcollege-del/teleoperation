"""Checks one recorded session's Isaac frames against the robot's truth (docs/adr/0032 decision 6, ``just isaac-record-check``).

Run on the GPU machine with Isaac's Python (numpy, Pillow, OpenCV), from robot/isaac:

    ~/isaacsim/venv/bin/python verify_session.py ~/isaacsim/recordings/<session>

Prints one ``RESULT {...}`` line and exits non-zero when any check fails or has nothing to measure (CLAUDE.md invariant 10):

- ``files.sha256`` verifies, the frame sidecars included;
- ``isaac.jlog`` has its trailer, its hash is right, and every ``FS`` line matches its file;
- the frames written plus the frames dropped (``FD``) are at least 98% of the recording's duration x fps / record-every,
  and the frames written at least 95%;
- every frame's ``stateSensed`` is the ``sensed`` of a ``QS`` record (at least 99.5% of them; the design is all);
- depth on the table mat, against the table's plane through the recorded camera: p99 under 3 mm;
- a point inside each hand (5 cm along the hand's axis from its flange), from the forward kinematics of the frame's ``QS``
  state, projected through the recorded camera, lands within 8 px of that hand's mask in at least 95% of the frames
  where the hand moved and its mask shows (at least 20 of them). That is what says each frame shows the state it is
  stamped with.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import kinematics  # noqa: E402
from teleop_isaac import profile as profile_module  # noqa: E402
from teleop_isaac import scene as scene_module  # noqa: E402
from teleop_isaac.isaac_session import read_sidecar  # noqa: E402
from teleop_isaac.renderer import MASK_IDS, MotorMap  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
HAND_POINT = (0.0, 0.0, 0.05)


def pct(xs, p):
    if not xs:
        return None
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def quat_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def read_lines(path):
    with open(path, encoding="ascii") as f:
        return f.read().splitlines()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--urdf", default=os.path.join(REPO, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"))
    ap.add_argument("--every", type=int, default=3, help="decode every this many frames for the image checks")
    args = ap.parse_args()
    d = os.path.abspath(os.path.expanduser(args.session))
    out = {"session": os.path.basename(d)}
    checks = {}

    # --- integrity --------------------------------------------------------------------------------------------------
    bad, n = [], 0
    for line in read_lines(os.path.join(d, "files.sha256")):
        sha, name = line.split("  ", 1)
        with open(os.path.join(d, name), "rb") as f:
            if hashlib.sha256(f.read()).hexdigest() != sha:
                bad.append(name)
        n += 1
    out["files_checked"] = n
    out["files_sidecars"] = sum(1 for line in read_lines(os.path.join(d, "files.sha256")) if line.endswith(".frames"))
    checks["files_sha256"] = not bad and out["files_sidecars"] > 0

    log = read_lines(os.path.join(d, "isaac.jlog"))
    end = log[-1].split("|")
    body = ("\n".join(log[:-1]) + "\n").encode("ascii")
    checks["isaac_trailer"] = end[0] == "END" and end[3] == hashlib.sha256(body).hexdigest()
    fs_ok = True
    for line in log:
        if line.startswith("FS|"):
            _, name, size, sha = line.split("|")
            with open(os.path.join(d, name), "rb") as f:
                data = f.read()
            fs_ok &= len(data) == int(size) and hashlib.sha256(data).hexdigest() == sha
    checks["isaac_fs"] = fs_ok
    out["isaac_notes"] = [x for x in log if x.startswith("ER|")]

    # --- the records -------------------------------------------------------------------------------------------------
    iv = next(x.split("|") for x in log if x.startswith("IV|"))
    fps, record_every, lag, agree = float(iv[2]), int(iv[3]), int(iv[4]), float(iv[5])
    out.update(isaac_version=iv[1], fps=fps, record_every=record_every, render_lag=lag, render_lag_agree=agree)
    cams = {}
    for x in log:
        if x.startswith("CM|"):
            f = x.split("|")
            cams[f[1]] = {"w": int(f[2]), "h": int(f[3]), "fx": float(f[4]), "fy": float(f[5]), "cx": float(f[6]), "cy": float(f[7]),
                          "p": np.array([float(v) for v in f[10:13]]), "R": quat_matrix(*[float(v) for v in f[13:17]])}
    vfs = []
    for x in log:
        if x.startswith("VF|"):
            f = x.split("|")
            parts = {}
            for item in f[5].split(";") if f[5] else []:
                name, off, length = item.rsplit(":", 2)
                parts[name] = (int(off), int(length))
            vfs.append({"t": int(f[1]), "frame": int(f[2]), "sensed": int(f[3]), "flags": f[4], "parts": parts})
    fds = [x for x in log if x.startswith("FD|")]
    out["frames_written"], out["frames_dropped"] = len(vfs), len(fds)
    if len(vfs) >= 2:
        span = (vfs[-1]["frame"] - vfs[0]["frame"]) / fps
        expected = span * fps / record_every + 1
        out["recorded_seconds"] = round(span, 2)
        out["frames_expected"] = round(expected, 1)
        checks["frame_count"] = (len(vfs) + len(fds)) >= 0.98 * expected and len(vfs) >= 0.95 * expected
    else:
        out["recorded_seconds"] = None

    qs = {}
    for line in read_lines(os.path.join(d, "robothost.jlog")):
        if line.startswith("QS|"):
            f = line.split("|")
            qs[int(f[2])] = [(int(a), float(b)) for a, b in (item.split(":") for item in f[3].split(";"))]
    matched = sum(1 for v in vfs if v["sensed"] in qs)
    out["stamps_matched"] = round(matched / len(vfs), 5) if vfs else None
    checks["stamps_match_qs"] = bool(vfs) and matched / len(vfs) >= 0.995

    # --- images ------------------------------------------------------------------------------------------------------
    import cv2

    scene = scene_module.load(os.path.join(d, "scene.json"))
    lift = float(((scene.extra.get("backdrops") or {}).get("mat") or {}).get("lift", 0.0005))
    top = scene.table.top + lift
    p = profile_module.load(os.path.join(d, "robot-profile.json"))
    motors = MotorMap(p)
    joints = kinematics.read_urdf(args.urdf)
    head = cams.get("head")
    sidecars = {}

    def blob(name, part):
        if name not in sidecars:
            path = os.path.join(d, "isaac", name + ".frames")
            sidecars[name] = dict(read_sidecar(path)) if os.path.exists(path) else {}
        return sidecars[name].get(part[0])

    def project(cam, x):
        xc = cam["R"].T @ (np.asarray(x, float) - cam["p"])
        if xc[2] >= -1e-6:
            return None
        return cam["cx"] + cam["fx"] * xc[0] / -xc[2], cam["cy"] - cam["fy"] * xc[1] / -xc[2]

    depth_err, hand_dist, last_point = [], {"left": [], "right": []}, {}
    sides = {"left": MASK_IDS["left_hand"], "right": MASK_IDS["right_hand"]}
    if head is not None:
        us, vs = np.meshgrid(np.arange(0, head["w"], 4), np.arange(0, head["h"], 4))
        for v in vfs[::max(1, args.every)]:
            if "head.mask" not in v["parts"] or "head.depth" not in v["parts"] or v["sensed"] not in qs:
                continue
            mask = cv2.imdecode(np.frombuffer(blob("head.mask", v["parts"]["head.mask"]), np.uint8), cv2.IMREAD_UNCHANGED)
            depth = cv2.imdecode(np.frombuffer(blob("head.depth", v["parts"]["head.depth"]), np.uint8), cv2.IMREAD_UNCHANGED)
            sel = mask[vs, us] == MASK_IDS["mat"]
            if sel.sum() > 50 and "B" not in v["flags"]:
                u, w = us[sel] + 0.5, vs[sel] + 0.5
                d_cam = np.stack([(u - head["cx"]) / head["fx"], -(w - head["cy"]) / head["fy"], -np.ones_like(u)], axis=0)
                d_w = head["R"] @ d_cam
                t = (top - head["p"][2]) / d_w[2]
                measured = depth[vs[sel], us[sel]].astype(float)
                ok = measured > 0
                depth_err.extend(np.abs(measured[ok] - t[ok] * 1000.0)[::5].tolist())
            values = motors.values(qs[v["sensed"]])
            for side, mid in sides.items():
                hp, hq = kinematics.link_pose(joints, values, f"{side}_fr3v2_hand")
                point = np.array(hp) + quat_matrix(*hq) @ np.array(HAND_POINT)
                moved = side in last_point and np.linalg.norm(point - last_point[side]) > 0.002
                last_point[side] = point
                vv, uu = np.nonzero(mask == mid)
                if not moved or len(uu) < 30:
                    continue
                pr = project(head, point)
                if pr is None or not (0 <= pr[0] < head["w"] and 0 <= pr[1] < head["h"]):
                    continue
                hand_dist[side].append(float(np.min(np.hypot(uu + 0.5 - pr[0], vv + 0.5 - pr[1]))))
    out["depth_err_mm_p50"] = round(pct(depth_err, 50), 3) if depth_err else None
    out["depth_err_mm_p99"] = round(pct(depth_err, 99), 3) if depth_err else None
    out["depth_samples"] = len(depth_err)
    checks["depth"] = bool(depth_err) and pct(depth_err, 99) < 3.0
    all_hands = hand_dist["left"] + hand_dist["right"]
    out["hand_samples"] = {k: len(v) for k, v in hand_dist.items()}
    out["hand_px_p50"] = round(pct(all_hands, 50), 2) if all_hands else None
    out["hand_px_p95"] = round(pct(all_hands, 95), 2) if all_hands else None
    within = sum(1 for x in all_hands if x <= 8.0)
    out["hand_within_8px"] = round(within / len(all_hands), 4) if all_hands else None
    checks["hands_in_their_masks"] = len(all_hands) >= 20 and within / len(all_hands) >= 0.95

    out["checks"] = checks
    out["missing"] = sorted(k for k, v in out.items() if v is None)
    out["passes"] = all(checks.values()) and len(checks) == 8 and not out["missing"]
    print("RESULT " + json.dumps(out, sort_keys=True))
    return 0 if out["passes"] else 1


if __name__ == "__main__":
    sys.exit(main())
