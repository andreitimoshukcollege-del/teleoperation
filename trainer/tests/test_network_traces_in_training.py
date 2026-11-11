"""docs/adr/0035 N5: the network datasets' traces in the training mix, the simulator's uplink, the forecast truth and the
per-tier test schedules."""

from __future__ import annotations

import numpy as np
import pytest

from armpred.data import schedule as S
from armpred.net import forecast as FC
from armpred.net.datasets import index as I
from armpred.net.datasets import split
from armpred.net.datasets import trace as T
from armpred.pipeline import trace_test_schedules


def _root(tmp_path):
    """Six groups per tier, two traces each: good is 30 ms flat; poor 30 ms with a 100 ms spike every 10 s; very bad
    loses one datagram in four and stalls 400 ms."""
    root = str(tmp_path / "datasets")
    items = []
    n = 3000  # 60 s at 20 ms
    for g in range(6):
        good = T.from_ms([30.0] * n, 20.0)
        poor = T.from_ms([30.0 + (100.0 - (i % 500) * 20.0 if i % 500 < 5 else 0.0) for i in range(n)], 20.0)
        bad = T.from_ms([None if i % 4 == 0 else (400.0 if 1000 <= i < 1020 else 30.0) for i in range(n)], 20.0)
        for k in range(2):
            items += [(f"g{g}-{k}", f"g{g}", good, {}), (f"p{g}-{k}", f"p{g}", poor, {}), (f"b{g}-{k}", f"b{g}", bad, {})]
    doc = I.emit(root, "demo", items, {})
    assert {e["tier"] for e in doc["traces"]} == {"good", "poor", "very-bad"}
    split.make(root, ["demo"])
    return root


def test_a_trace_segment_replays_the_trace_by_time_with_its_losses(tmp_path):
    root = _root(tmp_path)
    seg = S.Segment(10.0, "trace:demo/b0-0+20.0", False, trace="demo/b0-0", trace_root=root, trace_offset_s=0.0)
    link = S.ScheduledLink(S.Schedule(segments=(S.Segment(0.0, "lan", True), seg)), seed=1, tps=1_000_000_000, traces_dir=None)
    ns = 1_000_000_000
    # 10 s + 20 ms: sample 1 (30 ms); 10 s + 0 ms: sample 0, lost; 10 s + 20.0 s: sample 1000, the stall.
    assert link.transmit_one(int(10.02 * ns), 10.02) == (True, int(10.02 * ns) + 30_000_000, 1)
    assert link.transmit_one(int(10.0 * ns), 10.0)[0] is False
    ok, arrival, _ = link.transmit_one(int(30.02 * ns), 30.02)
    assert ok and arrival - int(30.02 * ns) == 400_000_000
    assert seg.base_profile(ns) is None and seg.impairments(ns, None) == []


def test_the_grammar_names_a_trace_only_with_a_root(tmp_path):
    root = _root(tmp_path)
    sch = S.parse("trace:demo/g0-0@0", root)
    assert sch.segments[0].trace == "demo/g0-0" and not sch.segments[0].named and not sch.only_named
    assert sch.describe() == "trace:demo/g0-0@0"
    with pytest.raises(ValueError):
        S.parse("trace:demo/g0-0@0")
    with pytest.raises(ValueError):
        S.parse("trace:../x@0", root)


def test_the_mix_draws_train_side_traces_of_the_weighted_tiers(tmp_path):
    root = _root(tmp_path)
    side = __import__("json").load(open(f"{root}/split.json"))["traces"]
    tiers = {e["id"]: e["tier"] for e in I.load(root, "demo")["traces"]}
    cfg = S.RandomScheduleConfig(share_named=0, share_parametric=0, share_added_delay=0, share_queue=0, share_outage=0,
                                 share_trace=1.0, trace_tier_weights=(0.0, 0.0, 1.0), datasets_root=root)
    sch = S.random_schedule(300.0, np.random.default_rng(3), cfg)
    assert all(s.trace for s in sch.segments)
    for s in sch.segments:
        assert side[s.trace] == "train" and tiers[s.trace.split("/")[1]] == "very-bad"
        assert 0.0 <= s.trace_offset_s <= 60.0
    up = S.random_uplink_schedule(120.0, np.random.default_rng(4), cfg)
    assert all(s.trace for s in up.segments) and S.Schedule.from_json(up.to_json()) == up  # survives the simulator's JSON
    with pytest.raises(ValueError):
        S.random_schedule(30.0, np.random.default_rng(1), S.RandomScheduleConfig(share_trace=0.5))  # no root


def test_the_forecast_truth_of_a_trace_segment_is_the_trace_around_that_moment(tmp_path):
    root = _root(tmp_path)
    sch = S.Schedule(segments=(S.Segment(0.0, "trace:demo/b1-0", False, trace="demo/b1-0", trace_root=root),))
    tps = 1_000_000_000
    truth = FC.LinkTruth(sch, tps, to_sched_s=lambda t: t / tps, to_op=lambda x: int(x * tps))
    q, loss, floor = truth.value(0, int(5.0 * tps))
    assert q[1] == 30.0 and floor == 30.0 and abs(loss - 0.25) < 0.05
    q_stall, _, _ = truth.value(0, int(20.2 * tps))  # inside the 400 ms stall
    assert q_stall[-1] == 400.0


def test_the_per_tier_test_schedules_are_held_out_traces(tmp_path):
    root = _root(tmp_path)
    side = __import__("json").load(open(f"{root}/split.json"))["traces"]
    tests = trace_test_schedules(root, per_tier=2)
    assert sorted(tests.values()) == ["good", "good", "poor", "poor", "very-bad", "very-bad"]
    for sched in tests:
        ref = sched[len("trace:"):-len("@0")]
        assert side[ref] == "test"
    assert trace_test_schedules(root, per_tier=2) == tests
