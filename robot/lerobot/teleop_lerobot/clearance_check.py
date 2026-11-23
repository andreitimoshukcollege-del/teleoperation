"""The clearance against PhysX's own contacts, on recorded takes (docs/adr/0038, workstream C): what the distances say where
PhysX said the arm touched, and where it said nothing touched.

Run on the GPU machine in a checkout, with the lerobot venv (``just isaac-clearance-check``):

    cd robot/lerobot && ~/isaacsim/lerobot-venv/bin/python -m teleop_lerobot.clearance_check \\
        [--geometry DIR] [--dataset DATASET | TAKE ...] [--traces N] [--json OUT]

It reads the takes (a safety dataset's ``meta/teleop/takes.jsonl``, or take folders) and prints:

- **at every touch onset** (a ``WC`` record naming a (link, object) pair the one before it did not): the signed distance from
  that link to that object at the record's time (joint values interpolated between the frames around it), its median and
  90th percentile of the absolute value, by object kind and link;
- **in contact-free frames** (no ``WC`` contact with an object within half a frame and :data:`MARGIN_NS` of the frame's
  time): the share whose nearest obstacle overlaps a link (negative clearance), a false overlap;
- **in contact**: each frame's clearance of the link ``WC`` names, against that object;
- **around the first touches** (``--traces``): the distance from 300 ms before to 300 ms after each;
- how long :func:`clearance.compute` took a take, and the surfaces' nearest approach (the table was not reported before
  docs/adr/0038, so this is a sanity check only: PhysX keeps the arm out of it).
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence

import numpy as np

from . import clearance
from .jlog import items
from .takes import read_take

MARGIN_NS = 50_000_000  # a frame within this (and half a frame) of a contact is not contact-free
TRACE_MS = (-300, -200, -100, -50, -20, -10, 0, 10, 20, 50, 100, 200, 300)


def onsets(world: clearance.TakeWorld, table: bool = False) -> List[Dict]:
    """Every (link, object) pair's first record in each run of ``WC`` records naming it: its time, link, object and force."""
    out, before = [], set()
    for sensed, text in world.contacts:
        now = {}
        for it in items(text):
            if len(it) >= 4 and (table or int(it[2]) != 0):
                now[(int(it[1]), int(it[2]))] = float(it[3])
        for pair, force in sorted(now.items()):
            if pair not in before:
                out.append({"t": sensed, "link": pair[0], "object": pair[1], "force": force})
        before = set(now)
    return out


def stretches(world: clearance.TakeWorld, table: bool = False) -> List[Dict]:
    """Each run of ``WC`` records with any (link, object) pair, from its first record to the one that ends it, and the pairs."""
    out, start, pairs = [], None, set()
    for sensed, text in world.contacts:
        now = {(int(it[1]), int(it[2])) for it in items(text) if len(it) >= 4 and (table or int(it[2]) != 0)}
        if now and start is None:
            start, pairs = sensed, set()
        if now:
            pairs |= now
        elif start is not None:
            out.append({"start": start, "end": sensed, "pairs": sorted(pairs)})
            start = None
    if start is not None:
        out.append({"start": start, "end": math.inf, "pairs": sorted(pairs)})
    return out


def check_take(path: str, geometry: str, traces: int = 0) -> Dict:
    take = read_take(path)
    t0 = time.monotonic()
    c = clearance.compute(take, geometry)
    seconds = time.monotonic() - t0
    world = clearance.TakeWorld(path, geometry)
    kinds = {int(o["id"]): (o.get("name", "?"), o.get("kind", "?")) for o in world.scene["objects"]}
    rows = []
    for e in onsets(world):
        d = clearance.link_object_distance(world, e["t"], e["link"], e["object"])
        name, kind = kinds.get(e["object"], ("?", "?"))
        row = {**e, "name": name, "kind": kind, "mm": round(d * 1000, 2)}
        if len(rows) < traces:
            row["trace"] = [round(clearance.link_object_distance(world, e["t"] + ms * 1e6, e["link"], e["object"]) * 1000, 2)
                            for ms in TRACE_MS]
        rows.append(row)
    runs = stretches(world)
    half = 1e9 / (2 * take.fps)
    free, touching = [], []
    obstacle = np.array(c.obstacle_mm)
    records = [(sensed, [(int(it[1]), int(it[2])) for it in items(text) if len(it) >= 4 and int(it[2]) != 0])
               for sensed, text in world.contacts]
    times = [r[0] for r in records]
    for i, f in enumerate(take.frames):
        t = float(f.timing["sensed"])
        hit = [r for r in runs if r["start"] - half - MARGIN_NS <= t <= r["end"] + half + MARGIN_NS]
        if not hit:
            free.append(float(obstacle[i].min()))
        k = bisect.bisect_right(times, t) - 1  # the WC record in force at the frame: the pairs touching then
        for link, obj in (records[k][1] if k >= 0 else []):
            touching.append({"mm": round(clearance.link_object_distance(world, t, link, obj) * 1000, 2),
                             "kind": kinds.get(obj, ("?", "?"))[1]})
    surface = np.array(c.surface_mm, float)
    return {"take": path, "scenario": take.scenario, "frames": len(take.frames), "seconds": round(seconds, 3), "onsets": rows,
            "free": free, "touching": touching, "surfaceMinMm": float(np.nanmin(surface)) if np.isfinite(surface).any() else None}


def _pct(values: Sequence[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, float), q)) if len(values) else float("nan")


def summary(results: Sequence[Dict]) -> List[str]:
    out = []
    ons = [r for res in results for r in res["onsets"]]
    absd = [abs(r["mm"]) for r in ons]
    out.append(f"takes {len(results)}, frames {sum(r['frames'] for r in results)}, compute {np.mean([r['seconds'] for r in results]):.2f} s "
               f"a take (max {max(r['seconds'] for r in results):.2f} s)")
    if ons:
        out.append(f"touch onsets {len(ons)}: |d| median {_pct(absd, 50):.2f} mm, p90 {_pct(absd, 90):.2f} mm; signed median "
                   f"{_pct([r['mm'] for r in ons], 50):.2f} mm, p10 {_pct([r['mm'] for r in ons], 10):.2f}, p90 {_pct([r['mm'] for r in ons], 90):.2f}")
        for label, sel in (("fixed", lambda r: r["kind"] == "fixed"), ("movable", lambda r: r["kind"] == "movable"),
                           ("finger pads", lambda r: r["link"] in (6, 7)), ("other links", lambda r: r["link"] not in (6, 7)),
                           ("0 N at onset", lambda r: r["force"] == 0), ("force at onset", lambda r: r["force"] > 0)):
            s = [r["mm"] for r in ons if sel(r)]
            if s:
                out.append(f"  {label:14s} {len(s):4d}: |d| median {_pct(np.abs(s), 50):.2f} p90 {_pct(np.abs(s), 90):.2f}; "
                           f"signed median {_pct(s, 50):.2f} mm")
        by_name: Dict[str, List[float]] = {}
        for r in ons:
            by_name.setdefault(r["name"], []).append(r["mm"])
        out.append("  by object (signed median mm, n): " + ", ".join(f"{k} {_pct(v, 50):.1f} ({len(v)})" for k, v in sorted(by_name.items())))
    free = [v for res in results for v in res["free"]]
    if free:
        out.append(f"contact-free frames {len(free)}: nearest obstacle < 0 mm {np.mean(np.asarray(free) < 0):.2%}, < -1 mm "
                   f"{np.mean(np.asarray(free) < -1):.2%}, < -3 mm {np.mean(np.asarray(free) < -3):.2%}; under 10 mm "
                   f"{np.mean(np.asarray(free) < 10):.1%}, under 30 mm {np.mean(np.asarray(free) < 30):.1%}")
    for label, kinds in (("", ("fixed", "movable", "?")), (" fixed", ("fixed",)), (" movable", ("movable",))):
        touching = [v["mm"] for res in results for v in res["touching"] if v["kind"] in kinds]
        if touching:
            out.append(f"frames in contact{label} {len(touching)} (link, object pairs WC names then): the link's distance median "
                       f"{_pct(touching, 50):.2f} mm, p10 {_pct(touching, 10):.2f}, p90 {_pct(touching, 90):.2f}, > 3 mm "
                       f"{np.mean(np.asarray(touching) > 3):.1%}, > 10 mm {np.mean(np.asarray(touching) > 10):.1%}")
    surf = [r["surfaceMinMm"] for r in results if r["surfaceMinMm"] is not None]
    if surf:
        out.append(f"surfaces: nearest approach a take median {_pct(surf, 50):.1f} mm, min {min(surf):.1f} mm")
    traced = [r for res in results for r in res["onsets"] if "trace" in r]
    if traced:
        out.append("traces (mm at " + ", ".join(f"{ms:+d}" for ms in TRACE_MS) + " ms from the onset):")
        for r in traced:
            out.append(f"  {os.path.basename(r.get('take', ''))} link {r['link']} -> {r['name']}: " + " ".join(f"{v:.1f}" for v in r["trace"]))
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    a = argparse.ArgumentParser(description="The clearance against PhysX's contacts (docs/adr/0038)")
    a.add_argument("takes", nargs="*")
    a.add_argument("--dataset", default="", help="a safety dataset: its takes (meta/teleop/takes.jsonl)")
    a.add_argument("--geometry", default=clearance.DEFAULT_GEOMETRY)
    a.add_argument("--traces", type=int, default=2, help="per take, the first N touches' distance over time")
    a.add_argument("--limit", type=int, default=0, help="at most this many takes")
    a.add_argument("--json", default="", help="write every row here")
    args = a.parse_args(argv)
    paths = list(args.takes)
    if args.dataset:
        with open(os.path.join(args.dataset, "meta", "teleop", "takes.jsonl"), encoding="utf-8") as f:
            paths += [json.loads(line)["take"] for line in f if line.strip()]
    if args.limit:
        paths = paths[:args.limit]
    results, failed = [], 0
    for p in paths:
        try:
            r = check_take(p, args.geometry, args.traces)
        except (clearance.ClearanceUnavailable, ValueError, OSError) as e:
            failed += 1
            print(f"[clearance-check] {p}: {e}", file=sys.stderr)
            continue
        for row in r["onsets"]:
            row["take"] = p
        results.append(r)
    if not results:
        print("[clearance-check] no take could be checked", file=sys.stderr)
        return 1
    for line in summary(results):
        print(line)
    if failed:
        print(f"[clearance-check] {failed} take(s) could not be checked")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
