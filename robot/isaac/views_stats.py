"""Offline statistics of a views spec's draws (docs/adr/0038 "V, views"), without Isaac: what the renderer would draw over many
takes, for the ADR's figures and for choosing a spec. Pure Python, from robot/isaac:

    python3 views_stats.py views/safety.json [--scene jetrover-mixed] [--takes 1000] [--region views/safety-holdout.json]

Take n (1..takes) is drawn with seed n and change n % 13, as ``views.draw`` does in the renderer. It prints one JSON object:

- ``views``, how many are static, moving or fixed, and how many fell back (no candidate passed every rule);
- ``attempts``: candidates drawn until one passed, median, p90, p99 and max, over the drawn views;
- ``rejected``: candidates rejected in all, and how often each rule refused one (a candidate can break several);
- ``azimuthDeg`` (30° bins), ``elevationDeg`` (10° bins) and ``distanceM`` (0.2 m bins): where the accepted views are;
- ``heldOut``: with a region (the spec's own, or ``--region``'s), how many views fell inside it, and the share of a uniform
  azimuth-elevation draw over the spec's ranges that it covers;
- ``msPerView``: Python time per view, as the renderer spends it between takes.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from teleop_isaac import scene as scene_module  # noqa: E402
from teleop_isaac import views  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))


def quantile(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else None


def stats(spec: views.Spec, scene, takes: int, region: views.Region = None) -> dict:
    geo = views.geometry(spec, scene)
    rejected, drawn = [], []
    t0 = time.perf_counter()
    for n in range(1, takes + 1):
        drawn.extend(views.draw(spec, geo, n, n % 13, rejected))
    ms = (time.perf_counter() - t0) * 1e3 / max(1, len(drawn))
    region = region or spec.held_out
    attempts = [v.attempts for v in drawn if v.attempts > 0]
    rules = Counter(r for broken in rejected for r in broken)
    out = {
        "takes": takes, "views": len(drawn), "modes": dict(Counter(v.mode for v in drawn)),
        "fallbacks": sum(1 for v in drawn if v.attempts == views.FALLBACK),
        "attempts": {"median": statistics.median(attempts) if attempts else None, "p90": quantile(attempts, 0.9),
                     "p99": quantile(attempts, 0.99), "max": max(attempts) if attempts else None},
        "rejected": {"candidates": len(rejected), "byRule": {r: rules[r] for r in views.RULES if rules[r]}},
        "azimuthDeg": {f"{a}-{a + 30}": sum(1 for v in drawn if a <= v.azimuth < a + 30) for a in range(0, 360, 30)},
        "elevationDeg": {f"{e}-{e + 10}": sum(1 for v in drawn if e <= v.elevation < e + 10) for e in range(-10, 90, 10)
                         if any(e <= v.elevation < e + 10 for v in drawn)},
        "distanceM": {f"{d / 10:.1f}-{(d + 2) / 10:.1f}": sum(1 for v in drawn if d / 10 <= v.distance < (d + 2) / 10)
                      for d in range(2, 50, 2) if any(d / 10 <= v.distance < (d + 2) / 10 for v in drawn)},
        "msPerView": round(ms, 2),
    }
    if region is not None:
        out["heldOut"] = {"azimuthDeg": list(region.azimuth), "elevationDeg": list(region.elevation),
                          "viewsInside": sum(1 for v in drawn if region.contains(v.azimuth, v.elevation)),
                          "uniformShare": round(views.held_out_share(region, spec.azimuth, spec.elevation), 4)}
    return out


def main(argv=None) -> int:
    a = argparse.ArgumentParser(description="offline statistics of a views spec's draws (docs/adr/0038 V)")
    a.add_argument("spec")
    a.add_argument("--scene", default="jetrover-mixed", help="a scene name under core/Scenes, or a path")
    a.add_argument("--takes", type=int, default=1000)
    a.add_argument("--count", type=int, default=-1, help="the views per take (-1: the spec's)")
    a.add_argument("--region", default="", help="another spec whose heldOut region to count views in")
    args = a.parse_args(argv)
    path = args.scene if os.path.exists(args.scene) else os.path.join(REPO, "core", "Scenes", args.scene + ".json")
    spec = views.load(args.spec, count=args.count)
    if spec is None:
        print("error: the spec has no views", file=sys.stderr)
        return 1
    region = views.load(args.region).held_out if args.region else None
    out = stats(spec, scene_module.load(path), args.takes, region)
    out["spec"], out["sha256"], out["scene"] = os.path.basename(args.spec), spec.sha256, os.path.basename(path)
    print(json.dumps(out, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
