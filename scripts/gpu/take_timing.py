"""Where a twin take's time goes, read-only, on the GPU machine (docs/agent-operations.md).

    ssh andrei@10.188.58.133 'python3 - <since> [root...]' < scripts/gpu/take_timing.py

For every take named at or after <since> (roots default to both twins' takes folders) it prints:
- the take's length;
- the idle at its start: frames until any arm joint has moved 0.5 degrees from the first frame's pose (VQ);
- frames flagged B (a room still settling);
- when each contact stretch begins (WC), in seconds and as a share of the take;
- the planned contact times (SE).
Standard library only.
"""
import glob
import os
import sys

CATS = ["reach_no_obstacle", "stop_before_collision", "collide_and_recover", "collide_with_obstacle", "mixed"]
since = sys.argv[1] if len(sys.argv) > 1 else ""
roots = sys.argv[2:] or [os.path.expanduser("~/data/takes/jetrover-isaac"), os.path.expanduser("~/data/takes/jetrover-isaac-twin1")]


def recs(path, tag):
    out = []
    try:
        with open(path, errors="replace") as f:
            for line in f:
                if line.startswith(tag + "|"):
                    out.append(line.rstrip("\n").split("|"))
    except FileNotFoundError:
        pass
    return out


for root in roots:
    for cat in CATS:
        for d in sorted(glob.glob(f"{root}/{cat}/*-take*") + glob.glob(f"{root}/_discarded/{cat}/*-take*")):
            if os.path.basename(d) < since:
                continue
            iso, host = os.path.join(d, "isaac.jlog"), os.path.join(d, "robothost.jlog")
            vq, vf = recs(iso, "VQ"), recs(iso, "VF")
            poses = []
            for r in vq:
                vals = {kv.split(":")[0]: float(kv.split(":")[1]) for kv in r[3].split(";") if kv and "finger" not in kv}
                poses.append((int(r[2]), vals))
            if not poses:
                continue
            t0, p0 = poses[0]
            t1 = poses[-1][0]
            span = max(t1 - t0, 1)
            move = next((t for t, p in poses if any(abs(p[k] - p0.get(k, p[k])) > 0.0087 for k in p)), t1)
            wc = sorted((int(r[2]), r[3] if len(r) > 3 else "") for r in recs(host, "WC") if len(r) > 2)
            starts, on = [], False
            for t, s in wc:
                # WC object 0 is the table (docs/adr/0038 decision 4): never an obstacle contact.
                s = ";".join(p for p in s.split(";") if p and p.split(":")[2:3] != ["0"])
                if s and not on:
                    starts.append(t)
                    on = True
                elif not s:
                    on = False
            # SE: t index drawn kind object finalS contactS endS clearS; the times are seconds after t (RobotHost's clock, ns).
            planned = [r for r in recs(host, "SE") if len(r) > 7]

            def planned_at(r):
                try:
                    return f"{(int(r[1]) + float(r[7]) * 1e9 - t0) / 1e9:.1f}s"
                except ValueError:
                    return "none"

            contacts = ", ".join(f"{(t - t0) / 1e9:.1f}s ({(t - t0) / span:.0%})" for t in starts) or "-"
            plan = ", ".join(f"{r[4]} {r[5]} at {planned_at(r)}" for r in planned) or "-"
            b = sum(1 for r in vf if len(r) > 4 and "B" in r[4])
            print(f"{os.path.basename(root)[-6:]:6} {cat[:16]:16} {os.path.basename(d)[-7:]} length {(t1 - t0) / 1e9:5.1f}s  "
                  f"idle {(move - t0) / 1e9:4.1f}s  B {b:3d}  contacts {contacts}  planned {plan}")
