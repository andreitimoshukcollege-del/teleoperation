from __future__ import annotations

import numpy as np

from armpred.net.impairments import EmulatedLink
from armpred.net.profiles import NetworkProfile, create_impairments, read_trace, rescale_trace, resolve_impairments

from .conftest import TRACES, load


def _impairments(name: str, tps: int):
    if name.startswith("custom:"):
        # TrainerGoldenBuilder's hand-built reorder set: 40 ms, +-5 ms, GE 0.02/0.3, 10% reordered by 30 ms.
        return create_impairments(NetworkProfile(tps // 25, tps // 200, 0.02, 0.3, 0.1, tps * 3 // 100))
    return resolve_impairments(name, tps, traces_dir=TRACES)


def test_every_datagram_fate_matches_core():
    cases = load("impairments-v1.json")["cases"]
    assert len(cases) > 0
    for case in cases:
        tps = int(case["ticksPerSecond"])
        link = EmulatedLink(_impairments(case["profile"], tps), int(case["seed"]))
        result = link.transmit(np.array(case["send"], dtype=np.int64))
        label = f"{case['profile']} @ {tps} seed {case['seed']}"
        assert result.delivered.astype(int).tolist() == case["delivered"], label
        expected_arrival = np.array(case["arrival"], dtype=np.int64)
        assert np.array_equal(np.where(result.delivered, result.arrival, 0), expected_arrival), label
        assert result.delivery_order.tolist() == case["order"], label


def test_a_dropped_datagram_consumes_no_delivery_draw():
    # Core drops at send, before the loopback: the jitter substream must not advance for it.
    tps = 1_000_000
    lossy = EmulatedLink(create_impairments(NetworkProfile(0, 1000, 0.5, 0.5)), 7)
    r = lossy.transmit(np.arange(200, dtype=np.int64) * 20_000)
    clean = EmulatedLink(create_impairments(NetworkProfile(0, 1000, 0.0, 0.0)), 7)
    c = clean.transmit(np.arange(int(r.delivered.sum()), dtype=np.int64) * 20_000)
    jitter_lossy = (r.arrival - np.arange(200) * 20_000)[r.delivered]
    jitter_clean = c.arrival - np.arange(int(r.delivered.sum())) * 20_000
    assert np.array_equal(jitter_lossy, jitter_clean)


def test_trace_rescales_between_tick_rates():
    trace_tps, samples = read_trace(f"{TRACES}/synthetic-burst.trace")
    assert trace_tps == 10_000_000
    scaled = rescale_trace(samples, trace_tps, 1_000_000_000)
    assert scaled[0] == samples[0] * 100
