"""Did the arm live through the uplink its session says (docs/adr/0035 N4)? For a session recorded with a tier
schedule (``UP`` records naming ``trace:<dataset>/<id>``), each segment's commands as RobotHost recorded them (``JC``:
``t`` when the socket read it, ``tDeliver`` when the impairment released it; a command whose sequence never appears was
lost) against the trace's own delays and losses over the same span of time (the trace restarts at each segment).

A segment passes when its delivered commands' median delay is within ``TOLERANCE_MS`` + 15 % of the trace's median over
that span and its loss is within ``LOSS_TOLERANCE`` of the trace's. Commands sample the trace at their own rate, and the
plant's loop takes a released command up to a few ms late, so the comparison is of distributions, not datagram by
datagram. Exits non-zero when a segment fails or there is nothing to check.
"""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional

import numpy as np

from . import trace as T

TOLERANCE_MS = 5.0
LOSS_TOLERANCE = 0.03
MIN_COMMANDS = 50


def read_session(path: str):
    tps = None
    ups, jc = [], []
    with open(os.path.join(path, "robothost.jlog"), encoding="utf-8", errors="replace") as f:
        for line in f:
            r = line.rstrip("\n").split("|")
            if r[0] == "JSES":
                tps = int(r[3])
            elif r[0] == "UP" and len(r) >= 3:
                ups.append((int(r[1]), r[2]))
            elif r[0] == "JC" and len(r) >= 6:
                jc.append((int(r[1]), int(r[2]), int(r[3])))
    return tps, ups, jc


def check(session: str, root: str) -> Dict:
    tps, ups, jc = read_session(session)
    if tps is None:
        raise ValueError(f"{session}: no JSES header")
    segments = []
    for k, (t0, profile) in enumerate(ups):
        t1 = ups[k + 1][0] if k + 1 < len(ups) else (jc[-1][0] + 1 if jc else t0)
        if not profile.startswith("trace:"):
            continue
        ds, tid = profile[len("trace:"):].split("/", 1)
        tr = T.read(os.path.join(root, ds, "traces", tid + ".trace"))
        rows = [(t, d, s) for t, d, s in jc if t0 <= t < t1]
        seg = {"profile": profile, "seconds": round((t1 - t0) / tps, 1), "commands": len(rows)}
        if len(rows) < MIN_COMMANDS:
            seg["verdict"] = "too few commands"
            segments.append(seg)
            continue
        delays = np.array([(d - t) / tps * 1000.0 for t, d, _ in rows])
        seqs = np.array(sorted(s for _, _, s in rows))
        expected = int(seqs[-1] - seqs[0] + 1)
        lost = 1.0 - len(np.unique(seqs)) / expected if expected > 0 else 0.0
        # The trace over the same span: samples 0 .. span / interval (it restarts at the segment, wraps at its end).
        n = int(np.ceil((t1 - t0) / tps / (tr.interval_ms / 1000.0)))
        idx = np.arange(n) % len(tr.delays)
        span = tr.delays[idx]
        trace_lost = float((span == T.LOST).mean())
        trace_ms = span[span != T.LOST] * 1000.0 / tr.ticks_per_second
        want = float(np.median(trace_ms)) if len(trace_ms) else float("nan")
        got = float(np.median(delays))
        ok_delay = abs(got - want) <= TOLERANCE_MS + 0.15 * want
        ok_loss = abs(lost - trace_lost) <= LOSS_TOLERANCE
        seg.update(delay_ms={"p50": round(got, 1), "p95": round(float(np.percentile(delays, 95)), 1), "max": round(float(delays.max()), 1)},
                   trace_delay_ms={"p50": round(want, 1), "p95": round(float(np.percentile(trace_ms, 95)), 1) if len(trace_ms) else None},
                   loss=round(lost, 4), trace_loss=round(trace_lost, 4),
                   verdict="ok" if ok_delay and ok_loss else ("delay differs" if not ok_delay else "loss differs"))
        segments.append(seg)
    checked = [s for s in segments if "delay_ms" in s]
    return {"session": os.path.basename(os.path.normpath(session)), "segments": segments,
            "pass": bool(checked) and all(s["verdict"] == "ok" for s in checked)}
