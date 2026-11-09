"""The headset's link trace (armpred/net/linktrace.py, docs/adr/0035 N1)."""

from __future__ import annotations

import json
import os

from armpred.net import linktrace as L

GOLDEN = os.path.join(os.path.dirname(__file__), "..", "contract", "golden", "linktrace-v1.json")
HTPS, RTPS = 10_000_000, 1_000_000_000


def test_every_golden_record_decodes_as_the_headsets_codec_wrote_it():
    doc = json.load(open(GOLDEN, encoding="utf-8"))
    assert doc["recordBytes"] == L.RECORD_BYTES == 48
    for r in doc["records"]:
        got = L.decode(bytes.fromhex(r["hex"]))
        assert (got.kind, got.sequence, got.a, got.b, got.c, got.d, got.e) == (r["kind"], r["sequence"], r["a"], r["b"], r["c"], r["d"], r["e"])
    assert L.decode(bytes(48)) is None


def write(tmp_path, records, header=None):
    head = {"format": "teleop-link-trace", "version": 1, "headsetTps": HTPS, "remoteHost": "10.0.0.68", "recordBytes": 48}
    head.update(header or {})
    p = tmp_path / "t.ltrace"
    with open(p, "wb") as f:
        f.write((json.dumps(head) + "\n").encode())
        for r in records:
            f.write(L.RECORD.pack(*r))
        f.write(b"\x01\x00")  # a torn last record, as when the app stops mid-write
    return str(p)


def frames(n, owd_ms, every_ms=20.0, skip=(), late=None):
    """``n`` frames sent every ``every_ms``, each ``owd_ms`` on the way (plus ``late(i)``), the clocks synced."""
    out = []
    for i in range(n):
        if i in skip:
            continue
        send_h = int(i * every_ms * HTPS / 1000)
        arr = send_h + int((owd_ms + (late(i) if late else 0.0)) * HTPS / 1000)
        out.append((L.FRAME, 1000 + i, int(i * every_ms * RTPS / 1000) + 7 * RTPS, RTPS, arr, send_h, 0))
    return out


def test_a_steady_link_has_its_delay_no_jitter_no_loss_no_stalls(tmp_path):
    h, rs = L.read(write(tmp_path, frames(500, 35.0)))
    s = L.summarize(h, rs)
    assert (s["frames"], s["loss"], s["stalls_over_ms"][100]) == (500, 0.0, 0)
    assert s["owd_downlink_ms"]["p50"] == 35.0 and s["net_downlink_jitter_ms"] < 0.1


def test_loss_is_counted_by_sequence_and_a_stall_then_burst_is_seen(tmp_path):
    # Frames 100-114 held 300 ms then released together (Wi-Fi power save); frames 300-309 lost.
    def late(i):
        return max(0.0, 300.0 - (i - 100) * 20.0) if 100 <= i < 115 else 0.0

    h, rs = L.read(write(tmp_path, frames(500, 10.0, skip=range(300, 310), late=late)))
    s = L.summarize(h, rs)
    assert s["loss"] == round(10 / 499, 5)
    assert s["stalls_over_ms"][250] == 1 and s["burst_frames"]["count"] >= 1 and s["burst_frames"]["max"] >= 10
    assert s["owd_downlink_ms"]["max"] == 310.0


def test_frames_under_the_headsets_own_disturbance_are_left_out_and_round_trips_are_measured(tmp_path):
    fs = frames(300, 20.0)
    on = (L.CONDITIONS, 0, 150_000, 20_000, 0, 0, fs[100][4])
    off = (L.CONDITIONS, 0, 0, 0, 0, 0, fs[200][4])
    rt = (L.ROUND_TRIP, 0, 0, 7 * RTPS, 7 * RTPS + 1_000_000, RTPS, int(0.041 * HTPS))  # 41 ms less 1 ms at the robot
    h, rs = L.read(write(tmp_path, fs[:100] + [on] + fs[100:200] + [off] + fs[200:] + [rt]))
    s = L.summarize(h, rs)
    assert (s["frames"], s["framesDisturbed"]) == (200, 100)
    assert s["rtt_ms"]["p50"] == 40.0
