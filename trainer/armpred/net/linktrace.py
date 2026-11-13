"""The headset's link trace (docs/adr/0035 N1, contract/LINK_TRACE_FORMAT.md): the real network as the headset saw it.

A file is one header line (JSON: ``headsetTps``, ``remoteHost``, ``startedUtc``, ...) and then 48-byte little-endian
records, ``u8 kind | u8[3] 0 | u32 sequence | i64 a b c d e``, written by the headset's Core ``LinkTraceCodec``
(``contract/golden/linktrace-v1.json`` pins this reader to it):

- kind 1, a frame: a robot send (robot ticks), b robot tps, c arrival (headset ticks), d the send on the headset's clock
  by ``ClockSync`` (``MIN_I64`` before it synced), e the reading's sensed time (robot ticks; 0 none);
- kind 2, a keepalive echo: a the headset's keepalive send, b the robot's receipt, c the robot's send, d robot tps, e the
  arrival: what ``ClockSync`` takes, so a reader can redo it;
- kind 3, the headset's own disturbance changed: a lag µs, b jitter µs, c loss ppm, d added delay µs, e when. While any is
  non-zero, frames are not the real network, and :func:`summarize` leaves them out.

:func:`summarize` gives docs/metrics.md section 3's quantities over a trace (one-way delay as ``owd_downlink_ms``, RFC 3550
jitter from raw stamps, loss by sequence) and what bursty Wi-Fi looks like: stalls (arrival gaps over 100, 250, 500 ms) and
the frames that arrive together after one.
"""

from __future__ import annotations

import json
import struct
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

RECORD = struct.Struct("<B3xIqqqqq")
RECORD_BYTES = RECORD.size  # 48
MIN_I64 = -(2 ** 63)
FRAME, ROUND_TRIP, CONDITIONS = 1, 2, 3


@dataclass(frozen=True)
class Record:
    kind: int
    sequence: int
    a: int
    b: int
    c: int
    d: int
    e: int


def decode(data: bytes) -> Optional[Record]:
    if len(data) < RECORD_BYTES or not FRAME <= data[0] <= CONDITIONS:
        return None
    return Record(*RECORD.unpack_from(data, 0))


def read(path: str) -> Tuple[Dict, List[Record]]:
    """A trace's header and records; a torn last record (the app stopped mid-write) is dropped."""
    with open(path, "rb") as f:
        blob = f.read()
    end = blob.index(b"\n")
    header = json.loads(blob[:end].decode("utf-8"))
    if header.get("format") != "teleop-link-trace" or header.get("version") != 1:
        raise ValueError(f"{path}: not a teleop-link-trace version 1")
    body = blob[end + 1:]
    records = []
    for at in range(0, len(body) - RECORD_BYTES + 1, RECORD_BYTES):
        r = decode(body[at:at + RECORD_BYTES])
        if r is not None:
            records.append(r)
    return header, records


def _pct(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    s = sorted(values)
    return round(s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))], 2)


def summarize(header: Dict, records: Iterable[Record]) -> Dict:
    htps = float(header["headsetTps"])
    disturbed = False
    frames: List[Record] = []
    rtts: List[float] = []
    skipped = 0
    for r in records:
        if r.kind == CONDITIONS:
            disturbed = any((r.a, r.b, r.c, r.d))
        elif r.kind == ROUND_TRIP and r.d > 0:
            rtts.append(((r.e - r.a) / htps - (r.c - r.b) / r.d) * 1000.0)
        elif r.kind == FRAME:
            if disturbed:
                skipped += 1
            else:
                frames.append(r)
    owd = [(f.c - f.d) / htps * 1000.0 for f in frames if f.d != MIN_I64]
    gaps, d_abs = [], []
    jitter = 0.0
    lost = expected = 0
    stalls = {100: 0, 250: 0, 500: 0}
    bursts: List[int] = []
    for prev, cur in zip(frames, frames[1:]):
        step = (cur.sequence - prev.sequence) & 0xFFFFFFFF
        if step == 0 or step > 1000:  # a duplicate, or the robot started counting again
            continue
        expected += step
        lost += step - 1
        gap = (cur.c - prev.c) / htps * 1000.0
        gaps.append(gap)
        if cur.b == prev.b and cur.b > 0:
            d = abs(gap - (cur.a - prev.a) / cur.b * 1000.0)
            d_abs.append(d)
            jitter += (d - jitter) / 16.0
        for t in stalls:
            if gap > t:
                stalls[t] += 1
    # A burst: the frames that arrive within 5 ms of each other right after a stall of more than 100 ms.
    i = 0
    while i < len(gaps):
        if gaps[i] > 100.0:
            n = 1
            while i + n < len(gaps) and gaps[i + n] < 5.0:
                n += 1
            bursts.append(n)
            i += n
        else:
            i += 1
    seconds = (frames[-1].c - frames[0].c) / htps if len(frames) > 1 else 0.0
    return {
        "remoteHost": header.get("remoteHost"), "startedUtc": header.get("startedUtc"), "editor": header.get("editor"),
        "seconds": round(seconds, 1), "frames": len(frames), "framesDisturbed": skipped,
        "owd_downlink_ms": {"p50": _pct(owd, .5), "p95": _pct(owd, .95), "p99": _pct(owd, .99), "max": _pct(owd, 1.0), "n": len(owd)},
        "rtt_ms": {"p50": _pct(rtts, .5), "p95": _pct(rtts, .95), "max": _pct(rtts, 1.0), "n": len(rtts)},
        "net_downlink_jitter_ms": round(jitter, 2),
        "jitter_abs_d_ms": {"p50": _pct(d_abs, .5), "p95": _pct(d_abs, .95), "p99": _pct(d_abs, .99)},
        "arrival_gap_ms": {"p50": _pct(gaps, .5), "p95": _pct(gaps, .95), "p99": _pct(gaps, .99), "max": _pct(gaps, 1.0)},
        "loss": round(lost / expected, 5) if expected else None,
        "stalls_over_ms": stalls,
        "stalls_per_minute_over_250ms": round(stalls[250] / (seconds / 60.0), 2) if seconds > 0 else None,
        "burst_frames": {"count": len(bursts), "p50": _pct([float(b) for b in bursts], .5), "max": max(bursts) if bursts else 0},
    }


def main(argv: Optional[List[str]] = None) -> int:
    paths = argv if argv is not None else sys.argv[1:]
    if not paths:
        print("usage: python -m armpred.net.linktrace <file.ltrace> ...", file=sys.stderr)
        return 2
    for p in paths:
        header, records = read(p)
        print(json.dumps({"file": p, **summarize(header, records)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
