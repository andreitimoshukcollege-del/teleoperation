"""Fit the depth model's edge parameters (docs/adr/0039) to what a real NFOV-unbinned depth engine leaves at depth steps.

The targets are HILO's raw Azure Kinect depth (CC BY 4.0, zenodo 15263380; 162 frames of tabletop scenes at 0.2-1.9 m,
measured with ``depth_stats``' statistics; ADR 0039 has the table). The model sees HILO-like scenes instead: a table with
boxes and upright cylinders and a floor 0.75 m below, ray-cast in the render pinhole the twin renders and resampled onto
the raw grid exactly as the renderer's frames are (``depth_model.to_raw``), from 0.5-1.0 m at 35-65 degrees down. A
coordinate descent over ``blurPx``, ``windowPx``, ``bg``, ``fg``, ``r50Mm``, ``rWidthMm``, ``mixMm`` and ``mixWidthMm`` minimises the squared
relative misfit of each jump bin's gap width, invalid and flying pixels per transition, and the invalid share at 1-3 px
outside an object's silhouette and 1 px inside it.

    python3 depth_fit.py depth_models/femto-nfov.json [--frames 8] [--rounds 2]   # prints the best edges section

Light: a few minutes of one core.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import depth_model  # noqa: E402
import depth_stats  # noqa: E402

# HILO's raw depth, 162 frames (ADR 0039, "measured").
TARGET = {
    "byJump": {"40-100": (5.80, 0.221, 3.358), "100-300": (6.34, 0.774, 3.369), "300+": (6.32, 3.268, 0.707)},
    "outside": {1: 0.28, 2: 0.21, 3: 0.11},  # invalid share at 1-3 px outside an object's silhouette (baseline ~0.05 off)
    "inside": {-1: 0.037},
}
BASELINE = 0.05  # HILO's invalid share 5-8 px outside objects, which the edge rules do not cause
GRID = {"blurPx": [2, 3, 4, 5, 6, 7], "windowPx": [3, 5, 7, 9], "bg": [0.3, 0.5, 0.7, 0.9, 1.0], "fg": [0.0, 0.05, 0.15, 0.3],
        "r50Mm": [60, 120, 200, 300, 450], "rWidthMm": [30, 60, 120, 240], "mixMm": [60, 100, 150, 250, 400, 100000],
        "mixWidthMm": [20, 40, 80]}


def scenes(spec, n: int, seed: int = 11):
    import numpy as np

    g = depth_model.geometry(spec.mode)
    intr = {"fx": g.render_f, "fy": g.render_f, "cx": g.render_width / 2, "cy": g.render_height / 2}
    rng = np.random.default_rng(seed)
    out = []
    for k in range(n):
        dist, pitch = rng.uniform(0.5, 1.0), rng.uniform(35, 65)
        az = rng.uniform(0, 2 * math.pi)
        el = math.radians(pitch)
        cam = (dist * math.cos(el) * math.cos(az), dist * math.cos(el) * math.sin(az), dist * math.sin(el))
        boxes, cyls = [], []
        for _ in range(7):
            cx, cy, hgt = rng.uniform(-0.25, 0.25), rng.uniform(-0.25, 0.25), rng.uniform(0.03, 0.25)
            if rng.random() < 0.5:
                sx, sy = rng.uniform(0.02, 0.1), rng.uniform(0.02, 0.1)
                boxes.append(((cx - sx, cy - sy, 0.0), (cx + sx, cy + sy, hgt)))
            else:
                cyls.append(((cx, cy, 0.0), rng.uniform(0.02, 0.07), hgt))
        z, obj = depth_stats.synthetic_depth(intr, (g.render_height, g.render_width), cam, (0.0, 0.0, 0.05), boxes, cyls, 0.0, (0.45, 0.45))
        zf, _ = depth_stats.synthetic_depth(intr, (g.render_height, g.render_width), cam, (0.0, 0.0, 0.05), (), (), -0.75, None)
        table_or_obj = np.isfinite(z)
        z = np.where(table_or_obj, z, zf)  # beyond the table, the floor
        out.append((depth_model.to_raw(spec.mode, z), depth_model.to_raw(spec.mode, obj.astype(np.uint8)).astype(bool)))
    return out


def evaluate(spec, frames):
    edges, prof = [], {}
    for k, (z, obj) in enumerate(frames):
        fr = depth_model.sensor_frame(spec, "fit", 3, 0, k)
        d = depth_model.apply(spec, z, fr)
        edges.append(depth_stats.edge_transitions(d))
        hit = (z < 1e9)
        for dd, (i, t) in depth_stats.boundary_profile(d, obj, hit & fr_lit(spec), 4).items():
            a = prof.setdefault(dd, [0, 0])
            a[0] += i
            a[1] += t
    s = depth_stats.edge_summary(depth_stats.merge_edges(edges))
    return s, {k: (v[0] / v[1] if v[1] else float("nan")) for k, v in prof.items()}


def fr_lit(spec):
    g = depth_model.geometry(spec.mode)
    return depth_model._mask_for(spec.polygon, ("raw", spec.mode), g.x, g.y)


def loss(summary, prof) -> float:
    err = 0.0
    for name, (gap, inv, fly) in TARGET["byJump"].items():
        got = summary["byJump"].get(name)
        if not got:
            err += 3.0
            continue
        err += ((got["meanGap"] - gap) / gap) ** 2 + ((got["invalidPerTransition"] - inv) / max(inv, 0.3)) ** 2 \
            + ((got["flyingPerTransition"] - fly) / max(fly, 0.5)) ** 2
    for d, want in TARGET["outside"].items():
        err += ((prof.get(d, 0.0) - (want - BASELINE)) / 0.1) ** 2
    for d, want in TARGET["inside"].items():
        err += ((prof.get(d, 0.0) - want) / 0.05) ** 2
    return err


def with_edges(doc, edges):
    d = copy.deepcopy(doc)
    d["edges"].update(edges)
    return depth_model.parse(d)


def main(argv=None) -> int:
    a = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    a.add_argument("model")
    a.add_argument("--frames", type=int, default=8)
    a.add_argument("--rounds", type=int, default=2)
    args = a.parse_args(argv)
    with open(args.model, encoding="utf-8") as f:
        doc = json.load(f)
    spec = depth_model.parse(doc)
    frames = scenes(spec, args.frames)
    best = dict(doc["edges"])
    s, p = evaluate(with_edges(doc, best), frames)
    best_loss = loss(s, p)
    print(f"start {best} loss {best_loss:.3f}", flush=True)
    for rnd in range(args.rounds):
        for key, values in GRID.items():
            for v in values:
                if v == best[key]:
                    continue
                trial = dict(best, **{key: v})
                s, p = evaluate(with_edges(doc, trial), frames)
                lo = loss(s, p)
                if lo < best_loss:
                    best, best_loss = trial, lo
                    print(f"round {rnd} {key}={v}: loss {lo:.3f}", flush=True)
    s, p = evaluate(with_edges(doc, best), frames)
    print(json.dumps({"edges": best, "loss": round(best_loss, 4), "model": s, "boundary": {k: round(v, 4) for k, v in sorted(p.items())},
                      "target": {"byJump": TARGET["byJump"], "outsideMinusBaseline": {k: round(v - BASELINE, 3) for k, v in TARGET["outside"].items()},
                                 "inside": TARGET["inside"]}}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
