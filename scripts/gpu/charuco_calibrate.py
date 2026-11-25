"""Calibrate the robot's cameras from a ChArUco recording made by `just camera-calib-record`.

    python charuco_calibrate.py <recording folder> [--min-corners 8] [--sync-ms 120] [--still-px 1.5]

The folder holds board.json (the board: squares_x, squares_y, square_m, marker_m, dictionary, legacy_pattern),
cameras.csv, and one folder per camera with JPEGs and frames.csv (capture stamps on the robot's clock, one clock
for all three cameras). Needs OpenCV >= 4.7 with aruco (the GPU machine's ~/isaacsim/lerobot-venv has 4.12).

What it computes, written to <folder>/calibration.json and summarised on stdout:
- Each camera's intrinsics (pinhole + 5 distortion terms) from up to 80 still views spread over the recording, refit once
  without views whose reprojection error is over 2.5x the median.
- Each camera's pose relative to the reference camera (the first in cameras.csv that sees the board most, normally left):
  from pairs of still frames of two cameras whose capture stamps are within --sync-ms (120 ms: the board is held still), each pair giving
  T_ref_cam = T_ref_board * inv(T_cam_board); the pose is the robust mean of the pairs (rotations averaged as
  quaternions after dropping pairs more than 3 deg / 2 cm from the median), with the spread reported.
- A loop check for three cameras: front->left->right->front should be the identity.
- Then a joint solve (bundle adjustment, intrinsics fixed) over every group of still views two or more cameras share:
  the cameras' poses and each group's board pose together, Huber-weighted reprojection error. Its poses are the result
  (poses_in_reference); the pairwise ones are kept as poses_pairwise.

"Still" means the board's corners moved less than --still-px from the same camera's previous frame, so a moving
board (motion blur, the cameras' unsynchronised exposures) does not enter either step.

Not computed: the cameras' poses in the robot's base frame. That needs a link to the robot (the arm seen in the images
with its joint states, or the board placed at a known spot on the robot) and is left for later.
"""
import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np


def load_board(folder):
    b = json.load(open(os.path.join(folder, "board.json")))
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b["dictionary"]))
    board = cv2.aruco.CharucoBoard((b["squares_x"], b["squares_y"]), b["square_m"], b["marker_m"], d)
    board.setLegacyPattern(bool(b.get("legacy_pattern", False)))
    return b, board


def cameras(folder):
    with open(os.path.join(folder, "cameras.csv")) as f:
        return [r["camera"] for r in csv.DictReader(f)]


def detect(folder, cam, board, min_corners):
    det = cv2.aruco.CharucoDetector(board)
    out = []
    size = None
    with open(os.path.join(folder, cam, "frames.csv")) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        img = cv2.imread(os.path.join(folder, cam, r["file"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = img.shape[::-1]
        cc, ci, _, _ = det.detectBoard(img)
        if ci is not None and len(ci) >= min_corners:
            out.append({"t": int(r["capture_ticks"]), "file": r["file"], "corners": cc.reshape(-1, 2), "ids": ci.flatten()})
    return rows, out, size


def mark_still(views, still_px):
    prev = None
    for v in views:
        v["still"] = False
        if prev is not None:
            common, a, b = np.intersect1d(prev["ids"], v["ids"], return_indices=True)
            if len(common) >= 6:
                moved = np.linalg.norm(prev["corners"][a] - v["corners"][b], axis=1)
                v["still"] = bool(np.median(moved) < still_px)
        prev = v


def spread(views, n):
    if len(views) <= n:
        return list(views)
    idx = np.linspace(0, len(views) - 1, n).round().astype(int)
    return [views[i] for i in sorted(set(idx))]


def calibrate(views, board, size):
    pts = board.getChessboardCorners()
    obj = [pts[v["ids"]].astype(np.float32) for v in views]
    img = [v["corners"].astype(np.float32) for v in views]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None)
    errs = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, dist)
        errs.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i) ** 2, axis=1)))))
    return rms, K, dist, np.array(errs)


def board_pose(v, board, K, dist):
    obj = board.getChessboardCorners()[v["ids"]].astype(np.float32)
    ok, r, t = cv2.solvePnP(obj, v["corners"].astype(np.float32), K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    p, _ = cv2.projectPoints(obj, r, t, K, dist)
    err = float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - v["corners"]) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(r)[0]
    T[:3, 3] = t.ravel()
    return T, err


def quat(R):
    q = np.empty(4)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q[:] = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return q / np.linalg.norm(q)


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def robust_mean(Ts):
    """Mean of rigid transforms: quaternion mean and translation mean after dropping outliers from the median."""
    qs = np.array([quat(T[:3, :3]) for T in Ts])
    qs[qs @ qs[0] < 0] *= -1
    ts = np.array([T[:3, 3] for T in Ts])
    def mean(qsel, tsel):
        q = np.linalg.eigh(qsel.T @ qsel)[1][:, -1]
        T = np.eye(4)
        T[:3, :3] = rot(q / np.linalg.norm(q))
        T[:3, 3] = tsel.mean(axis=0)
        return T
    med = np.eye(4)
    med[:3, :3] = rot(np.median(qs, axis=0) / np.linalg.norm(np.median(qs, axis=0)))
    med[:3, 3] = np.median(ts, axis=0)
    keep = [i for i, T in enumerate(Ts) if angle_deg(T[:3, :3], med[:3, :3]) < 3 and np.linalg.norm(T[:3, 3] - med[:3, 3]) < 0.02]
    if len(keep) < 3:
        keep = list(range(len(Ts)))
    M = mean(qs[keep], ts[keep])
    rot_err = [angle_deg(Ts[i][:3, :3], M[:3, :3]) for i in keep]
    tr_err = [float(np.linalg.norm(Ts[i][:3, 3] - M[:3, 3])) for i in keep]
    return M, len(keep), float(np.median(rot_err)), float(np.median(tr_err))


def to_T(x):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(x[:3], dtype=np.float64))[0]
    T[:3, 3] = x[3:6]
    return T


def from_T(T):
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def joint_refine(groups, cams, ref, init_cam_ref, board, intr, iters=30, huber_px=1.5):
    """Bundle adjustment with the intrinsics fixed: unknowns are T_cam_ref for every camera but the reference and
    T_ref_board for every group; residuals are every detected corner's reprojection error, Huber-weighted.
    Levenberg-Marquardt with block-wise numerical Jacobians (a board pose touches only its group's residuals)."""
    pts = board.getChessboardCorners().astype(np.float64)
    others = [c for c in cams if c != ref]
    cam_x = {c: from_T(init_cam_ref[c]) for c in others}
    grp_x = [from_T(g["T_ref_board"]) for g in groups]

    def project(cam, Tcb, ids):
        Xc = (Tcb[:3, :3] @ pts[ids].T + Tcb[:3, 3:4]).T
        K, dist = intr[cam]
        p, _ = cv2.projectPoints(Xc, np.zeros(3), np.zeros(3), K, dist)
        return p.reshape(-1, 2)

    def obs_res(cam, cx, gx, v):
        Tcr = np.eye(4) if cam == ref else to_T(cx)
        return (project(cam, Tcr @ to_T(gx), v["ids"]) - v["corners"]).ravel()

    n_cam = 6 * len(others)
    def unpack(x):
        cx = {c: x[6 * i:6 * i + 6] for i, c in enumerate(others)}
        gx = [x[n_cam + 6 * k:n_cam + 6 * k + 6] for k in range(len(groups))]
        return cx, gx

    x = np.concatenate([cam_x[c] for c in others] + grp_x)
    lam = 1e-3
    def cost_and_blocks(x, want_jac):
        cx, gx = unpack(x)
        rows, J_rows, total = [], [], 0.0
        for k, g in enumerate(groups):
            for cam, v in g["views"].items():
                r = obs_res(cam, cx.get(cam), gx[k], v)
                a = np.abs(r)
                w = np.where(a <= huber_px, 1.0, huber_px / np.maximum(a, 1e-12))
                total += float(np.sum(np.where(a <= huber_px, 0.5 * r * r, huber_px * (a - 0.5 * huber_px))))
                if want_jac:
                    sw = np.sqrt(w)
                    blocks = {}
                    eps = 1e-6
                    Jg = np.empty((len(r), 6))
                    for j in range(6):
                        d = np.zeros(6); d[j] = eps
                        Jg[:, j] = (obs_res(cam, cx.get(cam), gx[k] + d, v) - r) / eps
                    blocks[("g", k)] = Jg * sw[:, None]
                    if cam != ref:
                        Jc = np.empty((len(r), 6))
                        for j in range(6):
                            d = np.zeros(6); d[j] = eps
                            Jc[:, j] = (obs_res(cam, cx[cam] + d, gx[k], v) - r) / eps
                        blocks[("c", cam)] = Jc * sw[:, None]
                    rows.append(r * sw)
                    J_rows.append(blocks)
        return total, rows, J_rows

    cost, rows, J_rows = cost_and_blocks(x, True)
    nx = len(x)
    for it in range(iters):
        H = np.zeros((nx, nx)); g = np.zeros(nx)
        for r, blocks in zip(rows, J_rows):
            items = []
            for key, Jb in blocks.items():
                off = n_cam + 6 * key[1] if key[0] == "g" else 6 * others.index(key[1])
                items.append((off, Jb))
            for oa, Ja in items:
                g[oa:oa + 6] += Ja.T @ r
                for ob, Jb in items:
                    H[oa:oa + 6, ob:ob + 6] += Ja.T @ Jb
        improved = False
        for _ in range(10):
            step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-9), -g)
            new_cost, *_ = cost_and_blocks(x + step, False)
            if new_cost < cost:
                x = x + step; lam = max(lam / 3, 1e-7); improved = True
                break
            lam *= 5
        if not improved or np.linalg.norm(step) < 1e-9:
            break
        cost, rows, J_rows = cost_and_blocks(x, True)
    cx, gx = unpack(x)
    per_cam = {c: [] for c in cams}
    for k, gr in enumerate(groups):
        for cam, v in gr["views"].items():
            r = obs_res(cam, cx.get(cam), gx[k], v).reshape(-1, 2)
            per_cam[cam].extend(np.linalg.norm(r, axis=1).tolist())
    T_ref_cam = {ref: np.eye(4)}
    for c in others:
        T_ref_cam[c] = np.linalg.inv(to_T(cx[c]))
    stats = {c: {"corners": len(e), "rms_px": float(np.sqrt(np.mean(np.square(e)))) if e else None,
                 "p95_px": float(np.percentile(e, 95)) if e else None} for c, e in per_cam.items()}
    return T_ref_cam, stats, it + 1


def make_groups(seen, ref, init_ref_cam, sync_ns):
    """Groups of still views, one per camera, within sync_ns of each other, seen by at least two cameras."""
    allv = sorted(((v["t"], cam, v) for cam, vs in seen.items() for v in vs), key=lambda e: e[0])
    groups, i = [], 0
    while i < len(allv):
        t0 = allv[i][0]
        g = {}
        j = i
        while j < len(allv) and allv[j][0] - t0 <= sync_ns:
            _, cam, v = allv[j]
            if cam not in g or v.get("err", 9) < g[cam].get("err", 9):
                g[cam] = v
            j += 1
        if len(g) >= 2 and any(c in init_ref_cam for c in g):
            best = min((c for c in g if c in init_ref_cam), key=lambda c: g[c].get("err", 9))
            groups.append({"views": g, "T_ref_board": init_ref_cam[best] @ g[best]["T"]})
        i = j
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--min-corners", type=int, default=8)
    ap.add_argument("--sync-ms", type=float, default=120)
    ap.add_argument("--still-px", type=float, default=1.5)
    a = ap.parse_args()
    spec, board = load_board(a.folder)
    cams = cameras(a.folder)
    result = {"board": spec, "cameras": {}, "extrinsics": {}, "settings": vars(a)}
    seen = {}
    for cam in cams:
        rows, views, size = detect(a.folder, cam, board, a.min_corners)
        mark_still(views, a.still_px)
        still = [v for v in views if v["still"]]
        print(f"{cam}: {len(views)}/{len(rows)} frames see the board, {len(still)} still")
        if len(still) < 8:
            print(f"  {cam}: too few still views to calibrate")
            continue
        sel = spread(still, 80)
        rms, K, dist, errs = calibrate(sel, board, size)
        good = [v for v, e in zip(sel, errs) if e <= 2.5 * np.median(errs)]
        if len(good) < len(sel):
            rms, K, dist, errs = calibrate(good, board, size)
            sel = good
        hfov = float(np.degrees(2 * np.arctan(size[0] / (2 * K[0, 0]))))
        vfov = float(np.degrees(2 * np.arctan(size[1] / (2 * K[1, 1]))))
        print(f"  intrinsics from {len(sel)} views: rms {rms:.3f} px, fx {K[0,0]:.1f} fy {K[1,1]:.1f} cx {K[0,2]:.1f} cy {K[1,2]:.1f}, "
              f"FOV {hfov:.1f} x {vfov:.1f} deg")
        result["cameras"][cam] = {"width": size[0], "height": size[1], "K": K.tolist(), "dist": dist.ravel().tolist(),
                                  "rms_px": rms, "views": len(sel), "frames_with_board": len(views), "still_frames": len(still),
                                  "fov_deg": [hfov, vfov]}
        for v in still:
            p = board_pose(v, board, K, dist)
            v["T"] = None if p is None or p[1] > 2.0 else p[0]
            v["err"] = 9.0 if p is None else p[1]
        seen[cam] = [v for v in still if v.get("T") is not None]
    calibrated = [c for c in cams if c in seen]
    if len(calibrated) < 2:
        print("fewer than two calibrated cameras: no extrinsics")
    else:
        ref = max(calibrated, key=lambda c: len(seen[c]))
        result["reference"] = ref
        pair_T = {}
        for x in calibrated:
            for y in calibrated:
                if x >= y:
                    continue
                ty = np.array([v["t"] for v in seen[y]])
                Ts = []
                for vx in seen[x]:
                    k = int(np.argmin(np.abs(ty - vx["t"])))
                    if abs(ty[k] - vx["t"]) <= a.sync_ms * 1e6:
                        Ts.append(vx["T"] @ np.linalg.inv(seen[y][k]["T"]))  # T_x_y
                if len(Ts) >= 3:
                    M, n, re, te = robust_mean(Ts)
                    pair_T[(x, y)] = M
                    pair_T[(y, x)] = np.linalg.inv(M)
                    print(f"{x}<-{y}: {len(Ts)} pairs, {n} kept, spread median {re:.2f} deg / {te * 1000:.1f} mm, "
                          f"baseline {np.linalg.norm(M[:3, 3]) * 100:.1f} cm")
                    result["extrinsics"][f"T_{x}_{y}"] = {"T": M.tolist(), "pairs": len(Ts), "kept": n,
                                                          "spread_deg": re, "spread_m": te}
                else:
                    print(f"{x}<-{y}: {len(Ts)} pairs, too few")
        poses = {ref: np.eye(4)}
        for c in calibrated:
            if c != ref and (ref, c) in pair_T:
                poses[c] = pair_T[(ref, c)]
        for c in calibrated:
            if c not in poses:  # chain through a third camera when there is no direct pair
                for m in calibrated:
                    if m in poses and (m, c) in pair_T:
                        poses[c] = poses[m] @ pair_T[(m, c)]
                        break
        result["poses_pairwise"] = {c: T.tolist() for c, T in poses.items()}
        intr = {c: (np.array(result["cameras"][c]["K"]), np.array(result["cameras"][c]["dist"])) for c in calibrated}
        groups = make_groups({c: seen[c] for c in poses}, ref, poses, a.sync_ms * 1e6)
        n_by = {c: sum(1 for g in groups if c in g["views"]) for c in poses}
        print(f"joint solve: {len(groups)} groups seen by two or more cameras ({', '.join(f'{c} {n}' for c, n in n_by.items())})")
        joint_T, jstats, its = joint_refine(groups, list(poses), ref, {c: np.linalg.inv(T) for c, T in poses.items()}, board, intr)
        for c in poses:
            d_r = angle_deg(joint_T[c][:3, :3], poses[c][:3, :3]); d_t = np.linalg.norm(joint_T[c][:3, 3] - poses[c][:3, 3])
            st = jstats[c]
            print(f"  {c}: reprojection rms {st['rms_px']:.2f} px, p95 {st['p95_px']:.2f} px over {st['corners']} corners; "
                  f"moved {d_r:.2f} deg / {d_t * 1000:.1f} mm from the pairwise pose; {np.linalg.norm(joint_T[c][:3, 3]) * 100:.1f} cm from {ref}")
        result["poses_in_reference"] = {c: T.tolist() for c, T in joint_T.items()}
        result["joint"] = {"groups": len(groups), "groups_by_camera": n_by, "iterations": its, "per_camera": jstats}
        if len(calibrated) == 3 and all((calibrated[i], calibrated[j]) in pair_T for i in range(3) for j in range(3) if i != j):
            c0, c1, c2 = calibrated
            L = pair_T[(c0, c1)] @ pair_T[(c1, c2)] @ pair_T[(c2, c0)]
            le_r, le_t = angle_deg(L[:3, :3], np.eye(3)), float(np.linalg.norm(L[:3, 3]))
            print(f"loop {c0}->{c1}->{c2}->{c0}: {le_r:.2f} deg, {le_t * 1000:.1f} mm from identity")
            result["loop_error"] = {"deg": le_r, "m": le_t}
    with open(os.path.join(a.folder, "calibration.json"), "w") as f:
        json.dump(result, f, indent=1)
    print(f"wrote {os.path.join(a.folder, 'calibration.json')}")
    return 0 if len(result["cameras"]) == len(cams) else 1


if __name__ == "__main__":
    sys.exit(main())
