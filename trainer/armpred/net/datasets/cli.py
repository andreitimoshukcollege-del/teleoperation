"""``python -m armpred.net.datasets``: fetch, convert, split and report the network datasets (docs/adr/0035 N2)."""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter, defaultdict
from typing import Dict, List, Optional

from . import fcc_mba, fetching, g1050, mahimahi, ours, puffer, ripe_atlas, split
from . import index as I

SOURCES = {m.NAME: m for m in (g1050, mahimahi, puffer, fcc_mba, ripe_atlas)}
OURS = "ours-wifi"
ALL = list(SOURCES) + [OURS]


def _names(given: List[str], allowed: List[str]) -> List[str]:
    names = given or allowed
    bad = [n for n in names if n not in allowed]
    if bad:
        raise SystemExit(f"unknown dataset(s) {bad}; known: {allowed}")
    return names


def convert_ours(root: str, files: List[str]) -> Dict:
    made = []
    for path in sorted(files):
        stem = os.path.splitext(os.path.basename(path))[0]
        for k, (t, info) in enumerate(ours.pieces(path)):
            info = {**info, "file": os.path.basename(path), "kind": "recorded"}
            made.append((f"{stem}-{k}", stem, t, info))
    return I.emit(root, OURS, made, {"source": "the headset's link traces (docs/adr/0035 amendment 1)"})


def status(root: str) -> int:
    print(f"root {root} ({fetching.disk_free_gb(root):.0f} GB free)")
    worst = 0
    for name in ALL:
        m = fetching.load_manifest(root, name)
        doc = I.load(root, name)
        files = (m or {}).get("files", [])
        size = sum(f.get("bytes", 0) for f in files)
        line = f"{name:12s} fetched {len(files):3d} files {size / 1e6:9.1f} MB"
        if m:
            bad = [f["path"] for f in files if not os.path.exists(os.path.join(root, name, f["path"]))
                   or fetching.sha256_of(os.path.join(root, name, f["path"])) != f["sha256"]]
            if bad:
                line += f"  CHANGED OR MISSING: {bad[:3]}"
                worst = 1
        if doc:
            tiers = Counter(e["tier"] for e in doc["traces"])
            disagree = sum(1 for e in doc["traces"] if e["tier"] != e["measuredTier"])
            line += f" | {len(doc['traces'])} traces: " + ", ".join(f"{t} {tiers.get(t, 0)}" for t in ("good", "poor", "very-bad"))
            if disagree:
                line += f" ({disagree} where the source's tier differs from the measured one)"
        elif m:
            line += " | not converted"
        print(line)
        if m:
            print(f"{'':12s} licence: {m.get('licence', '')[:140]}")
    p = os.path.join(root, split.FILE)
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            s = json.load(f)
        print(f"split ({s['madeUtc']}, salt {s['salt']}, fraction {s['fraction']}):")
        for name, tiers in s["counts"].items():
            print(f"  {name:12s} " + "  ".join(f"{t}: train {c['train']} / test {c['test']} ({c['groupsTest']} of {c['groups']} groups)" for t, c in sorted(tiers.items())))
    return worst


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m armpred.net.datasets", description=__doc__)
    ap.add_argument("--root", default=fetching.DEFAULT_ROOT)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fetch")
    f.add_argument("names", nargs="*")
    c = sub.add_parser("convert")
    c.add_argument("names", nargs="*")
    sub.add_parser("split")
    sub.add_parser("status")
    o = sub.add_parser("ours")
    o.add_argument("files", nargs="+")
    a = sub.add_parser("assess", help="one trace's assessment (any .trace v2), as JSON")
    a.add_argument("files", nargs="+")
    args = ap.parse_args(argv)
    root = os.path.expanduser(args.root)
    if args.cmd == "fetch":
        failed = []
        for name in _names(args.names, list(SOURCES)):
            print(f"[fetch] {name} ...", flush=True)
            try:
                m = SOURCES[name].fetch(root)
                print(f"[fetch] {name}: {len(m.files)} files, {sum(x.bytes for x in m.files) / 1e6:.1f} MB", flush=True)
            except Exception as e:  # noqa: BLE001 -- one source failing must not stop the others; the exit code says so
                print(f"[fetch] {name} FAILED: {e}", file=sys.stderr, flush=True)
                failed.append(name)
        return 1 if failed else 0
    if args.cmd == "convert":
        failed = []
        for name in _names(args.names, list(SOURCES)):
            print(f"[convert] {name} ...", flush=True)
            try:
                doc = SOURCES[name].convert(root)
                tiers = Counter(e["tier"] for e in doc["traces"])
                print(f"[convert] {name}: {len(doc['traces'])} traces {dict(tiers)}", flush=True)
                if not doc["traces"]:
                    failed.append(name)
            except Exception as e:  # noqa: BLE001
                print(f"[convert] {name} FAILED: {type(e).__name__}: {e}", file=sys.stderr, flush=True)
                failed.append(name)
        return 1 if failed else 0
    if args.cmd == "split":
        doc = split.make(root, ALL)
        if not doc["traces"]:
            print("no converted traces to split", file=sys.stderr)
            return 1
        return status(root)
    if args.cmd == "status":
        return status(root)
    if args.cmd == "ours":
        doc = convert_ours(root, args.files)
        for e in doc["traces"]:
            a = e["assessment"]
            print(f"{e['id']}: {a['seconds']:.0f} s, tier {e['tier']} ({a['g1050_profile']}, Y.1541 class {a['y1541_class']}), "
                  f"delay p50 {a['delay_ms']['p50']} p99 {a['delay_ms']['p99']} ms, worst-minute IPDV {a['worst_minute']['ipdv_ms']} ms, "
                  f"loss {a['loss_ratio']:.4%}, bursts {a['burst_count']}")
        return 0 if doc["traces"] else 1
    if args.cmd == "assess":
        from . import trace as T
        from .tiering import assess
        for p in args.files:
            print(json.dumps({"file": p, **assess(T.read(p)).to_json()}))
        return 0
    return 2
