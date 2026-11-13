"""Our own Wi-Fi, from the headset's link traces (docs/adr/0035 decision 3, amendment 1): ``.ltrace`` to ``.trace`` v2.

Each joint-state frame the headset received gives its one-way delay on the headset's clock (arrival minus the send that
``ClockSync`` put on the headset's clock); a sequence number that never arrived is a loss. Datagram i of the trace is
sequence ``first + i``, and the send interval is the median spacing of the robot's own send stamps: the robot sends on a
timer, so the trace's regular spacing is an approximation that the index records.

A trace is cut where the headset was disturbing its own link (the console's sliders, ADDED DELAY), where the robot
started counting again, and before ``ClockSync`` first synced; pieces shorter than ``MIN_PIECE_S`` are dropped. A
negative delay (``ClockSync``'s offset error is larger than the delay) is clamped to 0, and the index counts them.

**The order the frames arrived in is kept.** ``ClockSync``'s offset moves by fractions of a millisecond as it learns,
so two frames released together after a stall can come out with delays that put the later one first. The headset's own
arrival stamps say which came first; where the delays disagree, the later frame's delay is raised until it arrives with
the one before (``reorderingRemoved`` counts them), so a replay never reorders what arrived in order.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np

from .. import linktrace as L
from .trace import LOST, Trace

MIN_PIECE_S = 30.0
MAX_STEP = 1000  # as linktrace.summarize: a bigger jump in sequence is the robot counting again


def pieces(path: str) -> List[Tuple[Trace, Dict]]:
    header, records = L.read(path)
    htps = int(header["headsetTps"])
    out: List[Tuple[Trace, Dict]] = []
    disturbed = False
    run: List[L.Record] = []
    newest = [0]
    have = set()

    def close() -> None:
        if len(run) >= 2:
            t, info = _piece(run, htps)
            if t.seconds >= MIN_PIECE_S:
                info.update({"file": path, "startedUtc": header.get("startedUtc"), "remoteHost": header.get("remoteHost"),
                             "editor": header.get("editor")})
                out.append((t, info))
        run.clear()
        have.clear()

    for r in records:
        if r.kind == L.CONDITIONS:
            now = any((r.a, r.b, r.c, r.d))
            if now and not disturbed:
                close()
            disturbed = now
        elif r.kind == L.FRAME and not disturbed and r.d != L.MIN_I64:
            if run:
                step = (r.sequence - newest[0]) & 0xFFFFFFFF
                if step >= 0x80000000:  # older than the newest: late, out of order, or a duplicate
                    if (newest[0] - r.sequence) & 0xFFFFFFFF > MAX_STEP or r.sequence in have:
                        continue
                elif step == 0:
                    continue
                elif step > MAX_STEP:
                    close()
            if not run or ((r.sequence - newest[0]) & 0xFFFFFFFF) < 0x80000000:
                newest[0] = r.sequence
            have.add(r.sequence)
            run.append(r)
    close()
    return out


def _piece(frames: List[L.Record], htps: int) -> Tuple[Trace, Dict]:
    base = frames[0].sequence
    first = min(frames, key=lambda f: ((f.sequence - base + 0x80000000) & 0xFFFFFFFF)).sequence
    n = max(((f.sequence - first) & 0xFFFFFFFF) for f in frames) + 1
    owd = np.full(n, LOST, dtype=np.int64)
    frames = sorted(frames, key=lambda f: (f.sequence - first) & 0xFFFFFFFF)
    sends = []
    clamped = 0
    tps = 10_000_000
    for f in frames:
        i = (f.sequence - first) & 0xFFFFFFFF
        d = (f.c - f.d) * tps // htps
        if d < 0:
            clamped += 1
            d = 0
        owd[i] = d
        sends.append((i, f.a, f.b))
    gaps = [(a2 - a1) / b2 / (i2 - i1) for (i1, a1, b1), (i2, a2, b2) in zip(sends, sends[1:]) if b1 == b2 and b2 > 0 and i2 > i1]
    interval_s = float(np.median(gaps)) if gaps else 0.02
    interval = int(round(interval_s * tps))
    # Keep the arrival order the headset saw (frames are in sequence order here; c is the arrival stamp).
    raised = 0
    by_seq = sorted(frames, key=lambda f: (f.sequence - first) & 0xFFFFFFFF)
    prev_i, prev_c = None, None
    for f in by_seq:
        i = (f.sequence - first) & 0xFFFFFFFF
        if prev_i is not None and f.c >= prev_c:
            floor = owd[prev_i] - (i - prev_i) * interval
            if owd[i] < floor:
                owd[i] = floor
                raised += 1
        prev_i, prev_c = i, f.c
    t = Trace(tps, interval, owd)
    return t, {"sendIntervalMs": round(interval_s * 1000.0, 3), "negativeDelaysClamped": clamped, "reorderingRemoved": raised,
               "assumption": "datagrams evenly spaced at the robot's median send interval; delay is ClockSync's one-way estimate"}
