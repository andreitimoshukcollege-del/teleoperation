from __future__ import annotations

from armpred.net.clocksync import ClockSync, ClockSyncConfig, trunc_div

from .conftest import load


def test_every_round_trip_matches_core():
    for case in load("clocksync-v1.json")["cases"]:
        c = case["config"]
        sync = ClockSync(ClockSyncConfig(
            history_capacity=c["historyCapacity"],
            smoothing_alpha=c["smoothingAlpha"],
            max_acceptable_rtt_ticks=c["maxAcceptableRttTicks"],
            outlier_rtt_multiple=float(c["outlierRttMultiple"]),
            min_samples_before_trusted=c["minSamplesBeforeTrusted"],
        ))
        op_tps, robot_tps = case["operatorTicksPerSecond"], case["robotTicksPerSecond"]
        for i, s in enumerate(case["steps"]):
            accepted = sync.add_round_trip(s["operatorSend"], op_tps, s["robotRecv"], s["robotSend"], robot_tps,
                                           s["operatorRecv"])
            where = f"{case['name']} step {i}"
            assert accepted == s["accepted"], where
            assert sync.offset_ticks == s["offset"], where
            assert sync.uncertainty_ticks == s["uncertainty"], where
            assert sync.last_rtt_ticks == s["lastRtt"], where
            assert sync.min_rtt_ticks == s["minRtt"], where
            assert sync.accepted == s["acceptedCount"], where
            assert sync.rejected == s["rejectedCount"], where
            assert sync.is_synced == s["synced"], where
            assert sync.to_operator_ticks(s["probeRobot"], robot_tps, op_tps) == s["probeOperator"], where


def test_trunc_div_truncates_toward_zero_like_csharp():
    assert trunc_div(-7, 2) == -3
    assert trunc_div(7, 2) == 3
    assert trunc_div(-7, -2) == 3
