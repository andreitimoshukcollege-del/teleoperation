"""Statistics of depth images in the terms ADR 0039 measured the Azure Kinect / Femto Bolt in, so the twin's modelled depth
(``<camera>.depth_femto``) and a real Femto Bolt's (``just femto-record``: ``depth/NNNNNN.png``) can be compared number for
number:

- **invalid**: the share of pixels with 0, inside the illuminated region (pixels valid in any frame of the set);
- **edges**: along every row and column, the transitions between "stable" pixels (valid, both neighbours within
  max(6 mm, 0.6%)) more than 40 mm apart and at most 16 px apart: the gap's width, how many of its pixels are invalid, and
  how many are valid at a depth between the two sides (more than 10 mm from either: "flying"), by the jump's size;
- **temporal noise** (a sequence from a camera standing still): each pixel's standard deviation over the frames, by depth;
- **spatial noise**: on the dominant plane (RANSAC), the variance of first differences of the residual at lags 1-8 px,
  from which the per-pixel noise and its correlation follow (V(k) = 2 s^2 (1 - rho_k) + c k^2).

    python3 depth_stats.py PNG_DIR [--intrinsics fx,fy,cx,cy] [--out stats.json]   # a real recording's depth/ folder
    python3 depth_stats.py --take TAKE_DIR --camera third-left [--stream depth_femto]    # a twin take's frames

Pure numpy and OpenCV (PNG decoding). :func:`synthetic_depth` ray-casts a table with boxes and upright cylinders, for the
tests and for fitting the edge parameters.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import struct
import sys
from collections import Counter, defaultdict
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

JUMP_BINS = ((40, 100, "40-100"), (100, 300, "100-300"), (300, 1e9, "300+"))


def edge_transitions(depth_mm, max_gap: int = 16, jump_mm: float = 40.0) -> Dict:
    """Row and column transitions between stable pixels of one depth image (uint16 mm, 0 invalid)."""
    import numpy as np

    out = {"transitions": 0, "gap": Counter(), "invalid": 0, "valid": 0, "flying": 0,
           "byJump": {name: [0, 0, 0, 0] for _, _, name in JUMP_BINS}}
    z = np.asarray(depth_mm, np.float64)
    for img in (z, z.T):
        v = img > 0
        dl = np.abs(img[:, 1:-1] - img[:, :-2])
        dr = np.abs(img[:, 1:-1] - img[:, 2:])
        tol = np.maximum(6.0, 0.006 * img[:, 1:-1])
        stable = np.zeros_like(v)
        stable[:, 1:-1] = v[:, 1:-1] & v[:, :-2] & v[:, 2:] & (dl <= tol) & (dr <= tol)
        rows, cols = np.nonzero(stable)
        if len(rows) < 2:
            continue
        same = rows[1:] == rows[:-1]
        p, q, rr = cols[:-1][same], cols[1:][same], rows[:-1][same]
        za, zb = img[rr, p], img[rr, q]
        jump = np.abs(zb - za)
        sel = (jump > jump_mm) & (q - p <= max_gap)
        for r_, p_, q_, a_, b_ in zip(rr[sel], p[sel], q[sel], za[sel], zb[sel]):
            gap = img[r_, p_ + 1:q_]
            n_inv = int((gap == 0).sum())
            lo, hi = min(a_, b_), max(a_, b_)
            gv = gap[gap > 0]
            fly = int(((gv > lo + 10) & (gv < hi - 10)).sum())
            out["transitions"] += 1
            out["gap"][len(gap)] += 1
            out["invalid"] += n_inv
            out["valid"] += len(gap) - n_inv
            out["flying"] += fly
            name = next(nm for a, b, nm in JUMP_BINS if a <= hi - lo < b)
            e = out["byJump"][name]
            e[0] += 1
            e[1] += len(gap)
            e[2] += n_inv
            e[3] += fly
    return out


def merge_edges(parts: Sequence[Dict]) -> Dict:
    out = {"transitions": 0, "gap": Counter(), "invalid": 0, "valid": 0, "flying": 0,
           "byJump": {name: [0, 0, 0, 0] for _, _, name in JUMP_BINS}}
    for p in parts:
        for k in ("transitions", "invalid", "valid", "flying"):
            out[k] += p[k]
        out["gap"].update(p["gap"])
        for name, e in p["byJump"].items():
            for i in range(4):
                out["byJump"][name][i] += e[i]
    return out


def edge_summary(e: Dict) -> Dict:
    gap_px = e["invalid"] + e["valid"]
    return {"transitions": e["transitions"],
            "gapInvalidShare": round(e["invalid"] / gap_px, 4) if gap_px else None,
            "flyingShareOfValid": round(e["flying"] / e["valid"], 4) if e["valid"] else None,
            "byJump": {name: {"n": v[0], "meanGap": round(v[1] / v[0], 3), "invalidPerTransition": round(v[2] / v[0], 3),
                              "flyingPerTransition": round(v[3] / v[0], 3)} for name, v in e["byJump"].items() if v[0]}}


def boundary_profile(depth_mm, obj_mask, exact_hit=None, reach: int = 6) -> Dict[int, Tuple[int, int]]:
    """Invalid pixels by signed distance (px) to the object mask's boundary, negative inside the objects: {d: (invalid,
    total)}. Only where the exact depth had a hit (``exact_hit``), when given."""
    import cv2
    import numpy as np

    obj = np.asarray(obj_mask, bool)
    din = cv2.distanceTransform(obj.astype(np.uint8), cv2.DIST_L2, 3)
    dout = cv2.distanceTransform((~obj).astype(np.uint8), cv2.DIST_L2, 3)
    sd = np.round(np.where(obj, -din, dout)).astype(int)
    valid = np.asarray(depth_mm) > 0
    base = np.ones_like(valid) if exact_hit is None else np.asarray(exact_hit, bool)
    out = {}
    for k in range(-reach, reach + 1):
        if k == 0:
            continue
        sel = base & (sd == k)
        out[k] = (int((~valid[sel]).sum()), int(sel.sum()))
    return out


def plane_residual(depth_mm, intr: Mapping, seed: int = 0, iters: int = 200, tol_mm: float = 6.0, rays=None):
    """(residual mm, inlier mask, unit normal, cos of incidence) of the dominant plane in a depth image: its rays are
    ``rays`` (undistorted x, y per pixel, e.g. depth_model.geometry(mode)), or a pinhole ``intr``'s (centres at +0.5)."""
    import numpy as np

    d = np.asarray(depth_mm, np.float64)
    h, w = d.shape
    if rays is not None:
        x, y = (np.asarray(r, np.float64) for r in rays)
    else:
        u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        x, y = (u - intr["cx"]) / intr["fx"], (v - intr["cy"]) / intr["fy"]
    valid = d > 0
    idx = np.flatnonzero(valid)
    if len(idx) < 100:
        return None
    pts = np.stack([x.ravel()[idx] * d.ravel()[idx], y.ravel()[idx] * d.ravel()[idx], d.ravel()[idx]], 1)
    rng = np.random.default_rng(seed)
    best = None
    sub = pts[rng.choice(len(pts), min(len(pts), 20000), replace=False)]
    for _ in range(iters):
        s = sub[rng.choice(len(sub), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        off = -n @ s[0]
        cnt = int((np.abs(sub @ n + off) < tol_mm).sum())
        if best is None or cnt > best[0]:
            best = (cnt, n, off)
    _, n, off = best
    for _ in range(3):
        inl = np.abs(pts @ n + off) < tol_mm
        c0 = pts[inl].mean(0)
        n = np.linalg.svd(pts[inl] - c0, full_matrices=False)[2][2]
        off = -n @ c0
    den = n[0] * x + n[1] * y + n[2]
    with np.errstate(invalid="ignore", divide="ignore"):
        zp = -off / den
    ray = np.stack([x, y, np.ones_like(x)], -1)
    ray /= np.linalg.norm(ray, axis=-1, keepdims=True)
    cosi = np.abs(ray @ n)
    res = np.where(valid, d - zp, np.nan)
    return res, valid & (np.abs(d - zp) < 2 * tol_mm), n, cosi


def lag_variances(resid, good, lags=(1, 2, 3, 4, 6, 8)) -> Dict[int, float]:
    """Variance of first differences of a residual along rows at each lag, over pixel pairs both ``good``."""
    import numpy as np

    out = {}
    for k in lags:
        gg = good[:, :-k] & good[:, k:]
        if gg.sum() > 200:
            out[k] = float(np.var((resid[:, k:] - resid[:, :-k])[gg]))
    return out


def noise_from_lags(V: Mapping[int, float]) -> Optional[Dict]:
    """Per-pixel noise and its correlation from :func:`lag_variances` (V(k) = 2 s^2 (1 - rho_k) + c k^2, rho gone by 4 px);
    quantisation's 1/12 mm^2 taken out of s."""
    import numpy as np

    ks = np.array([k for k in V if k >= 4], float)
    if len(ks) < 2:
        return None
    a, c = np.linalg.lstsq(np.stack([np.ones_like(ks), ks ** 2], 1), np.array([V[int(k)] for k in ks]), rcond=None)[0]
    if a <= 0:
        return None
    return {"sigmaMm": round(math.sqrt(max(0.0, a / 2 - 1 / 12)), 3),
            "rho": {int(k): round(1 - (V[k] - c * k * k) / a, 3) for k in sorted(V)}}


def temporal_noise(frames: Sequence, bins_mm=(300, 500, 700, 900, 1100, 1300, 1600, 2000, 3000)) -> Dict:
    """Each pixel's standard deviation over a still sequence (pixels valid in every frame), by its mean depth."""
    import numpy as np

    a = np.stack([np.asarray(f, np.float64) for f in frames])
    ok = (a > 0).all(0)
    mean, sd = a.mean(0), a.std(0)
    out = {}
    for lo, hi in zip(bins_mm[:-1], bins_mm[1:]):
        sel = ok & (mean >= lo) & (mean < hi)
        if sel.sum() > 100:
            out[f"{lo}-{hi}mm"] = {"sdMm": round(float(np.median(sd[sel])), 3), "px": int(sel.sum())}
    return out


def summarise(frames: Sequence, intr: Optional[Mapping] = None, still: bool = False, rays=None) -> Dict:
    """Every statistic of a set of depth images (uint16 mm). ``intr`` or ``rays`` (a raw image's undistorted rays) enable
    the plane noise; ``still`` the temporal."""
    import numpy as np

    stack = [np.asarray(f) for f in frames]
    lit = np.zeros(stack[0].shape, bool)
    for f in stack:
        lit |= f > 0
    inv = [float((f[lit] == 0).mean()) for f in stack]
    out = {"frames": len(stack), "litShare": round(float(lit.mean()), 4),
           "invalidInLit": {"mean": round(float(np.mean(inv)), 4), "p10": round(float(np.percentile(inv, 10)), 4),
                            "p90": round(float(np.percentile(inv, 90)), 4)},
           "edges": edge_summary(merge_edges([edge_transitions(f) for f in stack[:: max(1, len(stack) // 40)]]))}
    valid = np.concatenate([f[f > 0][::37] for f in stack])
    if len(valid):
        out["depthMm"] = {str(p): float(np.percentile(valid, p)) for p in (1, 50, 99)}
        steps = np.diff(np.unique(valid))
        out["quantisationMm"] = float(np.median(steps)) if len(steps) else None
    if intr is not None or rays is not None:
        lags = defaultdict(list)
        for f in stack[:: max(1, len(stack) // 20)]:
            got = plane_residual(f, intr, rays=rays)
            if got is None:
                continue
            res, good, _, cosi = got
            good &= cosi > 0.5
            for k, v in lag_variances(np.nan_to_num(res), good).items():
                lags[k].append(v)
        V = {k: float(np.mean(v)) for k, v in lags.items()}
        out["planeNoise"] = {"V": {k: round(v, 4) for k, v in V.items()}, "fit": noise_from_lags(V)}
    if still and len(stack) >= 10:
        out["temporalNoise"] = temporal_noise(stack)
    return out


# --- reading ---------------------------------------------------------------------------------------------------------------

def read_png_dir(path: str, limit: int = 0) -> List:
    import cv2

    files = sorted(glob.glob(os.path.join(path, "*.png")))
    if limit:
        files = files[:limit]
    return [cv2.imread(f, cv2.IMREAD_UNCHANGED) for f in files]


def read_take_stream(take: str, camera: str, stream: str, limit: int = 0, writer: str = "isaac"):
    """A twin take's frames of ``camera.stream`` (the VF parts in isaac.jlog into isaac/<camera>.<stream>.frames), and
    the intrinsics its FM (or CM) record gives."""
    import cv2
    import numpy as np

    name = f"{camera}.{stream}"
    intr = None
    parts = []
    with open(os.path.join(take, f"{writer}.jlog"), encoding="ascii") as f:
        for line in f:
            r = line.rstrip("\n").split("|")
            if r[0] == "FM" and r[1] == camera and r[2] == stream and intr is None:
                intr = {"width": int(r[4]), "height": int(r[5]), "fx": float(r[6]), "fy": float(r[7]), "cx": float(r[8]), "cy": float(r[9])}
            elif r[0] == "CM" and r[1] == camera and intr is None and stream == "depth":
                intr = {"width": int(r[2]), "height": int(r[3]), "fx": float(r[4]), "fy": float(r[5]), "cx": float(r[6]), "cy": float(r[7])}
            elif r[0] == "VF":
                for p in r[5].split(";"):
                    if p.startswith(name + ":"):
                        _, off, length = p.rsplit(":", 2)
                        parts.append((int(off), int(length)))
    if limit:
        parts = parts[:limit]
    out = []
    with open(os.path.join(take, writer, name + ".frames"), "rb") as f:
        for off, length in parts:
            f.seek(off)
            out.append(cv2.imdecode(np.frombuffer(f.read(length), np.uint8), cv2.IMREAD_UNCHANGED))
    return out, intr


# --- a synthetic scene -------------------------------------------------------------------------------------------------------

def synthetic_depth(intr: Mapping, shape: Tuple[int, int], cam_pos: Sequence[float], look_at: Sequence[float],
                    boxes: Sequence[Tuple[Sequence[float], Sequence[float]]] = (),
                    cylinders: Sequence[Tuple[Sequence[float], float, float]] = (), table_z: float = 0.0,
                    table_half: Optional[Tuple[float, float]] = None):
    """Exact depth (m to the image plane, inf where nothing is hit) and the object mask of a pinhole camera at ``cam_pos``
    looking at ``look_at`` (world z up; image +x right), over a table top at ``table_z`` (infinite, or ``table_half``
    = (half x, half y) about the origin), axis-aligned ``boxes`` (min corner, max corner) and upright ``cylinders`` (base
    centre, radius, height). Ray-cast in numpy, no Isaac."""
    import numpy as np

    h, w = shape
    c = np.asarray(cam_pos, float)
    fwd = np.asarray(look_at, float) - c
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    x, y = (u - intr["cx"]) / intr["fx"], (v - intr["cy"]) / intr["fy"]
    d = x[..., None] * right + y[..., None] * down + fwd  # z (forward) component 1: t along d is the depth Z
    best = np.full((h, w), np.inf)
    obj = np.zeros((h, w), bool)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (table_z - c[2]) / d[..., 2]
        ok = t > 0
        if table_half is not None:
            px, py = c[0] + t * d[..., 0], c[1] + t * d[..., 1]
            ok &= (np.abs(px) <= table_half[0]) & (np.abs(py) <= table_half[1])
        best = np.where(ok, t, best)
        for lo, hi in boxes:
            lo, hi = np.asarray(lo, float), np.asarray(hi, float)
            t1 = (lo - c) / d
            t2 = (hi - c) / d
            tn = np.nanmax(np.minimum(t1, t2), axis=-1)
            tf = np.nanmin(np.maximum(t1, t2), axis=-1)
            hitb = (tf >= tn) & (tn > 0) & (tn < best)
            best = np.where(hitb, tn, best)
            obj |= hitb  # objects stand on the table, so one hit is always nearer than the table
        for base, radius, height in cylinders:
            b = np.asarray(base, float)
            ox, oy = c[0] - b[0], c[1] - b[1]
            qa = d[..., 0] ** 2 + d[..., 1] ** 2
            qb = 2 * (ox * d[..., 0] + oy * d[..., 1])
            qc = ox * ox + oy * oy - radius * radius
            disc = qb * qb - 4 * qa * qc
            ts = (-qb - np.sqrt(np.maximum(disc, 0))) / (2 * qa)
            zs = c[2] + ts * d[..., 2]
            side = (disc >= 0) & (ts > 0) & (zs >= b[2]) & (zs <= b[2] + height)
            ttop = (b[2] + height - c[2]) / d[..., 2]
            px, py = c[0] + ttop * d[..., 0] - b[0], c[1] + ttop * d[..., 1] - b[1]
            top = (ttop > 0) & (px * px + py * py <= radius * radius)
            tc = np.where(side, ts, np.inf)
            tc = np.where(top & (ttop < tc), ttop, tc)
            hitc = tc < best
            best = np.where(hitc, tc, best)
            obj = np.where(hitc, True, obj)
    return best.astype(np.float32), obj


def random_scene(rng, intr: Mapping, shape: Tuple[int, int], distance: float, pitch_deg: float, n_objects: int = 6):
    """A random tabletop like HILO's: the camera ``distance`` m from the table's centre, ``pitch_deg`` below the horizon,
    boxes and upright cylinders 3-25 cm tall within 0.35 m of the centre."""
    az = rng.uniform(0, 2 * math.pi)
    el = math.radians(pitch_deg)
    cam = (distance * math.cos(el) * math.cos(az), distance * math.cos(el) * math.sin(az), distance * math.sin(el))
    boxes, cyls = [], []
    for _ in range(n_objects):
        cx, cy = rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)
        hgt = rng.uniform(0.03, 0.25)
        if rng.random() < 0.5:
            sx, sy = rng.uniform(0.03, 0.12), rng.uniform(0.03, 0.12)
            boxes.append(((cx - sx, cy - sy, 0.0), (cx + sx, cy + sy, hgt)))
        else:
            cyls.append(((cx, cy, 0.0), rng.uniform(0.025, 0.08), hgt))
    return synthetic_depth(intr, shape, cam, (0.0, 0.0, 0.05), boxes, cyls, 0.0, (0.6, 0.6))


def main(argv=None) -> int:
    a = argparse.ArgumentParser(description="depth statistics in ADR 0039's terms (real Femto Bolt PNGs or a twin take)")
    a.add_argument("pngs", nargs="?", default="", help="a folder of 16-bit depth PNGs in mm (femto-record's depth/)")
    a.add_argument("--take", default="", help="a twin take's folder instead")
    a.add_argument("--camera", default="third-left")
    a.add_argument("--stream", default="depth_femto")
    a.add_argument("--intrinsics", default="", help="fx,fy,cx,cy of the PNGs (enables the plane noise)")
    a.add_argument("--calibration", default="", help="femto-record's calibration.json: the raw depth camera (intrinsics and "
                                                      "distortion) from it")
    a.add_argument("--still", action="store_true", help="the camera and scene stood still: also the temporal noise")
    a.add_argument("--limit", type=int, default=0)
    a.add_argument("--out", default="")
    args = a.parse_args(argv)
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from teleop_isaac import depth_model

    intr, rays = None, None
    if args.take:
        frames, intr = read_take_stream(args.take, args.camera, args.stream, args.limit)
        used = os.path.join(args.take, "isaac", "depth_model.json")
        if args.stream == depth_model.STREAM and os.path.exists(used):  # the raw grid of the model the take ran
            with open(used, encoding="utf-8") as f:
                spec = depth_model.parse(json.load(f)["spec"])
            if (spec.rig(args.camera) or depth_model.Rig("", "same", (), False)).geometry == "sensor":
                g = depth_model.geometry(spec.mode)
                rays = (g.x, g.y)
    else:
        frames = read_png_dir(args.pngs, args.limit)
    if args.intrinsics:
        fx, fy, cx, cy = (float(v) for v in args.intrinsics.split(","))
        intr = {"fx": fx, "fy": fy, "cx": cx, "cy": cy}
    elif args.calibration:  # a real Femto's raw depth: its rays through the calibration's distortion
        with open(args.calibration, encoding="utf-8") as f:
            cal = json.load(f)
        with open(os.path.join(depth_model.default_dir(), "femto-nfov.json"), encoding="utf-8") as f:
            spec = depth_model.parse(depth_model.from_calibration(json.load(f), cal))
        g = depth_model.geometry(spec.mode)
        rays = (g.x, g.y)
    if not frames:
        print("error: no frames", file=sys.stderr)
        return 1
    out = summarise(frames, intr, args.still, rays)
    text = json.dumps(out, indent=1)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
