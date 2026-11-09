"""docs/adr/0035 N2's shared parts: trace v2, the tiers by the standards, our own traces, the split."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from armpred.net import linktrace as L
from armpred.net.datasets import index as I
from armpred.net.datasets import ours, split
from armpred.net.datasets import standards as S
from armpred.net.datasets import trace as T
from armpred.net.datasets.tiering import assess


def test_the_standards_numbers_are_the_recommendations():
    assert [(c.number, c.mean_delay_ms, c.ipdv_ms, c.loss_ratio) for c in S.Y1541_CLASSES] == [
        (0, 100.0, 50.0, 1e-3), (1, 400.0, 50.0, 1e-3), (2, 100.0, None, 1e-3), (3, 400.0, None, 1e-3),
        (4, 1000.0, None, 1e-3), (5, None, None, None)]
    a, b, c = S.G1050_PROFILES
    assert (a.latency_ms, a.jitter_pp_ms, a.burst_loss_ms, a.random_loss_ratio, a.reorder_ratio) == (300.0, 50.0, None, 0.0005, 0.00001)
    assert (b.latency_ms, b.jitter_pp_ms, b.burst_loss_ms, b.burst_rate_per_s, b.random_loss_ratio, b.reorder_ratio) == (400.0, 150.0, 200.0, 1e-3, 0.02, 0.0001)
    assert (c.latency_ms, c.jitter_pp_ms, c.burst_loss_ms, c.burst_rate_per_s, c.random_loss_ratio, c.reorder_ratio) == (500.0, 500.0, 10_000.0, 0.1, 0.20, 0.001)
    assert S.TIER_OF_PROFILE == {"w": "good", "p": "poor", "u": "very-bad"}


def test_trace_v2_round_trips_losses_and_v1_still_reads(tmp_path):
    t = T.from_ms([10.0, None, 12.5, float("nan"), 0.0], 20.0)
    p = str(tmp_path / "a.trace")
    T.write(p, t)
    assert open(p).read().splitlines() == ["TRACE|2|10000000|200000", "100000", "L", "125000", "L", "0"]
    back = T.read(p)
    assert back.send_interval_ticks == 200000 and list(back.lost) == [False, True, False, True, False]
    (tmp_path / "b.trace").write_text("TRACE|1|1000\n5\n7\n")
    v1 = T.read(str(tmp_path / "b.trace"))
    assert (v1.ticks_per_second, v1.send_interval_ticks, list(v1.delays)) == (1000, 0, [5, 7])
    (tmp_path / "c.trace").write_text("TRACE|1|1000\nL\n")
    with pytest.raises(ValueError):
        T.read(str(tmp_path / "c.trace"))
    with pytest.raises(ValueError):
        T.from_ms([-1.0], 20.0)


def steady(n, base, rng=None, spread=0.0):
    rng = rng or np.random.default_rng(1)
    return [base + (rng.uniform(0, spread) if spread else 0.0) for _ in range(n)]


def test_a_clean_link_is_well_managed_and_class_0():
    a = assess(T.from_ms(steady(9000, 30.0, spread=20.0), 20.0))  # 3 minutes, 20 ms of spread
    assert (a.g1050_profile, a.tier, a.y1541_class) == ("A well-managed", "good", 0)
    assert len(a.to_json(with_windows=True)["windows"]) == 3


def test_jitter_over_50_ms_is_partially_managed_and_one_bad_minute_decides():
    d = steady(9000, 30.0, spread=10.0)
    for i in range(3000, 3020):  # a 100 ms spike in the second minute only
        d[i] = 130.0 - (i - 3000) * 5.0
    a = assess(T.from_ms(d, 20.0))
    assert (a.g1050_profile, a.tier, a.y1541_class) == ("B partially-managed", "poor", 2)
    assert a.worst_minute["ipdv_ms"] > 50


def test_a_300_ms_outage_is_burst_loss_beyond_partially_managed():
    d = steady(9000, 30.0, spread=5.0)
    for i in range(4000, 4015):  # 15 datagrams = 300 ms lost in a row
        d[i] = None
    a = assess(T.from_ms(d, 20.0))
    assert a.burst_count == 1 and a.longest_burst_ms == 300.0 and a.random_loss_ratio == 0.0
    assert (a.g1050_profile, a.tier) == ("C unmanaged", "very-bad")


def test_single_losses_are_random_and_a_lost_minute_is_beyond_c():
    d = steady(9000, 30.0)
    d[100] = None
    a = assess(T.from_ms(d, 20.0))
    assert a.burst_count == 0 and a.random_loss_ratio == pytest.approx(1 / 9000, abs=1e-6)
    assert a.g1050_profile == "A well-managed"  # 1/3000 in its minute < 0.05%
    d = steady(9000, 30.0)
    for i in range(3000, 6000):
        d[i] = None
    assert assess(T.from_ms(d, 20.0)).g1050_profile == "beyond-C"


def test_reordering_counts_against_the_profile():
    d = steady(3000, 30.0)
    for i in range(0, 3000, 300):
        d[i] = 55.0  # arrives after the next one, sent 20 ms later at 30 ms
    a = assess(T.from_ms(d, 20.0))
    assert a.reorder_ratio == pytest.approx(10 / 3000, abs=1e-6)
    assert a.g1050_profile == "beyond-C"  # 0.33% > C's 0.1%


HTPS, RTPS = 10_000_000, 1_000_000_000


def write_ltrace(path, frames, conditions=()):
    head = {"format": "teleop-link-trace", "version": 1, "headsetTps": HTPS, "remoteHost": "x", "startedUtc": "t", "editor": False}
    recs = sorted(list(frames) + list(conditions), key=lambda r: r[4] if r[0] == L.FRAME else r[6])
    with open(path, "wb") as f:
        f.write((json.dumps(head) + "\n").encode())
        for r in recs:
            f.write(L.RECORD.pack(*r))


def test_our_trace_keeps_arrival_order_marks_losses_and_cuts_out_our_own_disturbance(tmp_path):
    frames = []
    for i in range(6000):  # 2 minutes at 20 ms
        if i in (500, 501):
            continue  # lost
        send_h = i * 200_000
        owd = 150_000
        if 1000 <= i < 1015:
            owd = 150_000 + (1015 - i) * 200_000  # held, then released together
        noise = (i % 3) * 5_000 if 1000 <= i < 1015 else 0  # ClockSync's offset wobbling by half a millisecond
        arrival = send_h + owd
        frames.append((L.FRAME, 1000 + i, i * 20_000_000 + 7 * RTPS, RTPS, arrival + (i - 1015 if 1000 <= i < 1015 else 0), send_h - noise, 0))
    on = (L.CONDITIONS, 0, 100_000, 0, 0, 0, 3000 * 200_000 + 1)
    off = (L.CONDITIONS, 0, 0, 0, 0, 0, 3100 * 200_000 + 1)
    p = str(tmp_path / "x.ltrace")
    write_ltrace(p, frames, [on, off])
    got = ours.pieces(p)
    assert len(got) == 2  # cut at the disturbance; both pieces >= 30 s
    t, info = got[0]
    assert info["sendIntervalMs"] == 20.0 and t.lost[500] and t.lost[501] and not t.lost[499]
    a = assess(t)
    assert a.reorder_ratio == 0.0 and info["reorderingRemoved"] > 0 and a.burst_count == 1  # only 500-501 lost


def test_a_frame_that_really_came_late_keeps_its_slot_and_its_reordering(tmp_path):
    frames = []
    for i in range(3000):
        owd = 400_000 if i == 1000 else 150_000  # 1000 arrives at 20.040 s, after 1001 at 20.035 s
        frames.append((L.FRAME, 5 + i, i * 20_000_000, RTPS, i * 200_000 + owd, i * 200_000, 0))
    p = str(tmp_path / "late.ltrace")
    write_ltrace(p, frames)
    (t, info), = ours.pieces(p)
    assert not t.lost.any() and t.delays[1000] == 400_000
    assert assess(t).reorder_ratio == pytest.approx(1 / 3000, abs=1e-6)


def test_the_split_holds_out_by_group_and_every_cell_has_a_test_side(tmp_path):
    root = str(tmp_path)
    items = []
    for g in range(12):
        tr = T.from_ms(steady(3000, 30.0 if g % 2 else 120.0, spread=5.0), 20.0)
        for k in range(2):
            items.append((f"g{g}-{k}", f"group{g}", tr, {}))
    I.emit(root, "demo", items, {})
    doc = split.make(root, ["demo"])
    sides = doc["traces"]
    for g in range(12):
        assert sides[f"demo/g{g}-0"] == sides[f"demo/g{g}-1"]  # a group never straddles the split
    for tier, c in doc["counts"]["demo"].items():
        assert c["groupsTest"] >= 1 and c["train"] > 0
    assert split.make(root, ["demo"])["traces"] == sides  # deterministic


def test_a_group_with_traces_in_two_tiers_is_on_one_side_in_both(tmp_path):
    root = str(tmp_path)
    items = []
    for g in range(8):
        good = T.from_ms(steady(3000, 30.0, spread=5.0), 20.0)
        poor = T.from_ms([30.0 + (100.0 if i % 500 == 0 else 0.0) for i in range(3000)], 20.0)  # 100 ms spikes: another tier
        items += [(f"g{g}-a", f"case{g}", good, {}), (f"g{g}-b", f"case{g}", poor, {})]
    I.emit(root, "demo", items, {})
    doc = split.make(root, ["demo"], salt="x")
    for g in range(8):
        assert doc["traces"][f"demo/g{g}-a"] == doc["traces"][f"demo/g{g}-b"]
    assert len(doc["counts"]["demo"]) == 2 and all(c["groupsTest"] >= 1 for c in doc["counts"]["demo"].values())
