"""What a batch of JetRover twin takes holds, read-only, on the GPU machine (docs/agent-operations.md).

    ssh andrei@10.188.58.133 'python3 - <since> [root...]' < scripts/gpu/take_report.py

<since> is a take-name prefix such as 20261121T182900Z: every take under each root's <category>/ and _discarded/<category>/
named at or after it is read. Roots default to both twins' takes folders. Standard library only.

Per take it reports:
- the checker's verdict (safety-check.json);
- the planned events (SE);
- the contact stretches (WC): each one's start, length, peak force, its lead-in (time before any force), and the objects;
- the layout from the take's first WO: lying objects, the object by the bench's edge (the scene's pool edgeRegions),
  raised clutter (objects the scene marks "clutter") and its settled tilt;
- placements the twin refused (WP);
- the looks and flat clutter the renderer drew (LK, CL).
Then it prints the totals.
"""
import collections
import glob
import json
import math
import os
import statistics
import sys

CATS = ["reach_no_obstacle", "stop_before_collision", "collide_and_recover", "collide_with_obstacle"]
since = sys.argv[1] if len(sys.argv) > 1 else ""
roots = sys.argv[2:] or [os.path.expanduser("~/data/takes/jetrover-isaac"), os.path.expanduser("~/data/takes/jetrover-isaac-twin1")]


def records(path, tag):
    out = []
    try:
        with open(path, errors="replace") as f:
            for line in f:
                if line.startswith(tag + "|"):
                    out.append(line.rstrip("\n").split("|"))
    except FileNotFoundError:
        pass
    return out


def items(s):
    return [it.split(":") for it in s.split(";") if it]


def tilt_deg(qx, qy):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))


def stretches(host):
    rows = sorted((int(r[2]), r[3] if len(r) > 3 else "") for r in records(host, "WC") if len(r) > 2)
    out, cur = [], None
    for t, s in rows:
        its = items(s)
        if its:
            force = max((float(i[3]) for i in its if len(i) > 3), default=0.0)
            if cur is None:
                cur = {"start": t, "end": None, "peak": 0.0, "first_force": None, "objects": set()}
            cur["peak"] = max(cur["peak"], force)
            cur["objects"] |= {int(i[2]) for i in its if len(i) > 2}
            if force > 0 and cur["first_force"] is None:
                cur["first_force"] = t
        elif cur is not None:
            cur["end"] = t
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    return out


def scene(take):
    try:
        return json.load(open(os.path.join(take, "scene.json")))
    except Exception:
        return {}


def in_edge(regions, x, y):
    for k in range(0, len(regions) - 3, 4):
        x0, x1, y0, y1 = regions[k:k + 4]
        if min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1):
            return True
    return False


rows = []
for root in roots:
    for cat in CATS:
        for kept, base in ((True, os.path.join(root, cat)), (False, os.path.join(root, "_discarded", cat))):
            for d in sorted(glob.glob(os.path.join(base, "*-take*"))):
                name = os.path.basename(d)
                if name < since:
                    continue
                host, iso = os.path.join(d, "robothost.jlog"), os.path.join(d, "isaac.jlog")
                try:
                    verdict = json.load(open(os.path.join(d, "safety-check.json")))
                except Exception:
                    verdict = {}
                sc = scene(d)
                objs = {o["id"]: o for o in sc.get("objects", []) if "id" in o}
                edge_regions = sc.get("pool", {}).get("edgeRegions", [])
                wl = records(host, "WL")
                placed = [int(x) for x in wl[-1][4].split(";") if x] if wl and len(wl[-1]) > 4 else []
                wo = records(host, "WO")
                poses = {int(i[0]): [float(v) for v in i[1:8]] for i in items(wo[0][4]) if len(i) >= 8} if wo and len(wo[0]) > 4 else {}
                lying, edge, clutter = [], [], []
                for oid in placed:
                    p = poses.get(oid)
                    if not p or p[2] < -1.0:
                        continue  # parked on the shelf
                    t = tilt_deg(p[5], p[6])
                    if objs.get(oid, {}).get("clutter"):
                        clutter.append((oid, round(t)))
                    elif t > 45:
                        lying.append(oid)
                    if in_edge(edge_regions, p[0], p[1]):
                        edge.append(oid)
                lk = records(iso, "LK")
                looks = dict(it[:2] for it in items(lk[-1][2]) if len(it) >= 2) if lk and len(lk[-1]) > 2 else {}
                cl = records(iso, "CL")
                pieces = collections.Counter(p[0].split("-")[0] for p in (items(cl[-1][2]) if cl and len(cl[-1]) > 2 else []))
                se = records(host, "SE")
                wp = collections.Counter(r[6] for r in records(host, "WP") if len(r) > 6)
                rows.append(dict(root=os.path.basename(root), cat=cat, name=name, kept=kept, reason=verdict.get("reason", ""),
                                 clutter_ids={i for i, o in objs.items() if o.get("clutter")},
                                 st=stretches(host), placed=placed, lying=lying, edge=edge, clutter=clutter, looks=looks,
                                 pieces=pieces, events=len({r[2] for r in se if len(r) > 2}), wp=wp))

print(f"{len(rows)} takes since {since or 'the start'} in {', '.join(os.path.basename(r) for r in roots)}")
for r in rows:
    print(f"{r['root'][-6:]:6} {r['cat'][:14]:14} {r['name'][-7:]} {'KEPT ' if r['kept'] else 'ASIDE'} events {r['events']} | "
          f"{r['reason'][:70]:70} | placed {len(r['placed'])} lying {r['lying']} edge {r['edge']} clutter {r['clutter']} | "
          f"table {r['looks'].get('table', '-')}, bench {r['looks'].get('bench', '-')} | flat {dict(r['pieces'])}"
          + ("" if set(r["wp"]) <= {"applied"} else f" | WP {dict(r['wp'])}"))
    for s in r["st"]:
        length = ((s["end"] or s["start"]) - s["start"]) / 1e6
        lead = ((s["first_force"] or s["end"] or s["start"]) - s["start"]) / 1e6
        print(f"{'':30}contact {length:.0f} ms, peak {s['peak']:.2f} N, lead-in {lead:.0f} ms, objects {sorted(s['objects'])}")

print("\n== kept / total per category")
for cat in CATS:
    rs = [r for r in rows if r["cat"] == cat]
    print(f"  {cat:22} {sum(r['kept'] for r in rs)} / {len(rs)}")
allst = [s for r in rows for s in r["st"]]
if allst:
    leads = [((s["first_force"] or s["end"] or s["start"]) - s["start"]) / 1e6 for s in allst]
    print(f"== contact stretches {len(allst)}: lead-in median {statistics.median(leads):.0f} ms, max {max(leads):.0f} ms; "
          f"{sum(1 for s in allst if s['first_force'] is None)} never had force")
if rows:
    print(f"== lying objects in {sum(1 for r in rows if r['lying'])}/{len(rows)} takes; edge object in {sum(1 for r in rows if r['edge'])}/{len(rows)}; "
          f"raised clutter per take {[len(r['clutter']) for r in rows]}, max settled tilt {max([t for r in rows for _, t in r['clutter']], default=None)}")
    print("== table looks", dict(collections.Counter(r["looks"].get("table", "-") for r in rows)))
    print("== bench looks", dict(collections.Counter(r["looks"].get("bench", "-") for r in rows)))
    refused = sum((r["wp"] for r in rows), collections.Counter())
    refused.pop("applied", None)
    print("== placements not applied:", dict(refused) or "none")
    hits = [(r["name"][-7:], sorted(s["objects"] & r["clutter_ids"])) for r in rows for s in r["st"] if s["objects"] & r["clutter_ids"]]
    print("== contacts with raised clutter:", hits or "none")
