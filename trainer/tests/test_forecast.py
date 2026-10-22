"""The network forecasters (ADR 0021 decision 12): the oracle is the truth, each degraded level has the
error statistics its row declares, and every level is deterministic for a seed."""

from __future__ import annotations

import math

import numpy as np
import pytest

from armpred.contract.profile import JETROVER
from armpred.data import schedule as S
from armpred.data.dataset import view_forecasts
from armpred.data.views import synthesize, view_observations, view_truths
from armpred.net import forecast as F
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session, write_session
from armpred.contract.session import load_session

from .conftest import TRACES

NS = 1_000_000_000
P50 = F.QUANTILES.index(0.50)


def _alternating(duration_s: float = 600.0, dwell_s: float = 2.0) -> S.Schedule:
    names = ("lan", "150ms-20j-0.5loss", "50ms-5j")
    n = int(duration_s / dwell_s)
    return S.Schedule(segments=tuple(S.Segment(k * dwell_s, names[k % 3], True) for k in range(n)))


def _truth(schedule: S.Schedule) -> F.LinkTruth:
    return F.LinkTruth(schedule, NS, to_sched_s=lambda t: t / NS, to_op=lambda x: int(round(x * NS)))


def _issue(duration_s: float = 600.0) -> np.ndarray:
    return F.issue_times(0, int(duration_s * NS), NS)


def test_profile_quantiles_match_a_sampled_distribution():
    rng = np.random.default_rng(0)
    x = np.maximum(0.0, 150.0 + rng.uniform(-20.0, 20.0, 400_000) + 30.0 * (rng.random(400_000) < 0.1))
    q = F.profile_quantiles_ms(150.0, 20.0, 0.1, 30.0)
    assert np.allclose(q, np.percentile(x, [100 * p for p in F.QUANTILES]), atol=0.3)
    assert F.stationary_loss(0.005, 0.005) == pytest.approx(0.005)
    assert F.stationary_loss(0.00612, 0.7) == pytest.approx(0.00612 / (1 - 0.7 + 0.00612))


def test_the_oracle_is_the_truth_at_every_horizon():
    truth = _truth(_alternating(60.0))
    t_issue = _issue(60.0)
    fc = F.make_forecasts("oracle", t_issue, (truth, truth), None, seed=1)
    for i in range(0, len(t_issue), 7):
        for h, h_ms in enumerate(F.HORIZONS_MS):
            s = int(t_issue[i]) + int(h_ms / 1000.0 * NS)
            q, loss, _ = truth.value(truth.segment_at(s), s)
            assert np.allclose(fc.q_ms[i, F.UP, h], q, atol=1e-3)
            assert fc.loss[i, F.UP, h] == pytest.approx(loss, abs=1e-6)
    assert fc.valid_q.all()


def _announced_share(level: str, seed: int) -> float:
    """At 50 ms before each change, does the forecast for 100 ms ahead already show it?"""
    truth = _truth(_alternating())
    changes = truth.changes()
    t_issue = np.array([c - int(0.05 * NS) for c in changes], dtype=np.int64)
    fc = F.make_forecasts(level, t_issue, (truth, truth), None, seed=seed)
    h = F.HORIZONS_MS.index(100)
    seen = 0
    for i, c in enumerate(changes):
        after = truth.value(truth.segment_at(c + 1), c + 1)[0][P50]
        before = truth.value(truth.segment_at(c - 1), c - 1)[0][P50]
        got = fc.q_ms[i, F.UP, h, P50]
        seen += abs(math.log(max(got, 1e-3) / after)) < abs(math.log(max(got, 1e-3) / before))
    return seen / len(changes)


@pytest.mark.parametrize("level,share", [("good", 0.7), ("fair", 0.4), ("poor", 0.1)])
def test_a_degraded_level_announces_its_share_of_changes(level, share):
    # Lead times are U(0.1, 1.0) s, so every announced change is visible 50 ms before it.
    assert abs(_announced_share(level, seed=3) - share) < 0.1


def test_an_unannounced_change_is_known_only_after_the_latency():
    truth = _truth(_alternating())
    level = F.DEGRADED["poor"]
    t_issue = _issue()
    fc = F.make_forecasts("poor", t_issue, (truth, truth), None, seed=4)
    plan = F._plan(level, truth, np.random.default_rng(np.random.PCG64([4, F.UP, 0xF0CA57])),
                   (int(t_issue[0]) - 2 * NS, int(t_issue[-1]) + 2 * NS))
    near_alarm = lambda c: any(a - lead - NS <= c <= b + 2 * NS for a, b, lead, _ in plan.alarms)  # noqa: E731
    checked = 0
    for j, c in enumerate(truth.changes()):
        if plan.announced[j] or near_alarm(c):
            continue
        before = truth.value(truth.segment_at(c - 1), c - 1)[0][P50]
        after = truth.value(truth.segment_at(c + 1), c + 1)[0][P50]
        early = int(np.searchsorted(t_issue, c + int(0.5 * NS)))  # inside the 1 s latency
        late = int(np.searchsorted(t_issue, c + int(1.15 * NS)))  # past it
        if late >= len(t_issue):
            continue
        got_early, got_late = fc.q_ms[early, F.UP, F.H0, P50], fc.q_ms[late, F.UP, F.H0, P50]
        assert abs(math.log(got_early / before)) < abs(math.log(got_early / after))
        assert abs(math.log(got_late / after)) < abs(math.log(got_late / before))
        checked += 1
    assert checked > 50


def test_the_magnitude_error_has_the_declared_spread():
    truth = _truth(_alternating())
    t_issue = _issue()
    for level in ("good", "fair", "poor"):
        fc = F.make_forecasts(level, t_issue, (truth, truth), None, seed=5)
        logs = []
        for i in range(0, len(t_issue), 5):
            # A horizon far enough back to be known at every level.
            s = int(t_issue[i]) + F.HORIZONS_MS[0] * 1_000_000
            q, _, floor = truth.value(truth.segment_at(s), s)
            got = fc.q_ms[i, F.UP, 0, P50]
            if q[P50] - floor > 5.0 and got - floor > 0.1:
                logs.append(math.log((got - floor) / (q[P50] - floor)))
        sigma = F.DEGRADED[level].sigma
        # False alarms add outliers; the median absolute deviation sees past them.
        mad = np.median(np.abs(np.array(logs) - np.median(logs))) * 1.4826
        assert 0.6 * sigma < mad < 1.5 * sigma, (level, mad, sigma)


def test_levels_are_deterministic_and_seeded():
    truth = _truth(_alternating(60.0))
    t_issue = _issue(60.0)
    a = F.make_forecasts("fair", t_issue, (truth, truth), None, seed=7)
    b = F.make_forecasts("fair", t_issue, (truth, truth), None, seed=7)
    c = F.make_forecasts("fair", t_issue, (truth, truth), None, seed=8)
    assert np.array_equal(a.q_ms, b.q_ms) and not np.array_equal(a.q_ms, c.q_ms)
    absent = F.make_forecasts("absent", t_issue, (truth, truth), None, seed=7)
    assert not absent.valid_q.any() and not absent.q_ms.any()


@pytest.fixture(scope="module")
def stepped_view(tmp_path_factory):
    """A simulated session whose physical uplink steps from the LAN to 150 ms and to 300 ms."""
    up = S.parse("lan@0;150ms-20j-0.5loss@10;lan@20;300ms-60j-2loss-bursty@30")
    p = sample_params(21, duration_s=40.0, slider=2.5, payload_grams=0.0, hand_events_per_min=0.0,
                      uplink_schedule=up.to_json())
    d = str(tmp_path_factory.mktemp("stepped") / "sim-21")
    write_session(simulate_session(p), d)
    return synthesize(load_session(d), JETROVER, S.parse("50ms-5j@0"), seed=3, traces_dir=TRACES)


def test_the_view_knows_each_commands_true_uplink_delay(stepped_view):
    v = stepped_view
    up, _ = view_truths(v)
    rel = (v.cmd_t - v.cmd_t[0]) / v.tps
    ok = ~v.cmd_lost
    for (a, b), profile_ms in (((12, 19), 150.0), ((32, 39), 300.0)):
        sel = ok & (rel >= a) & (rel < b)
        measured = np.median(v.cmd_true_uplink[sel]) / v.tps * 1e3
        t_mid = int(v.cmd_t[np.nonzero(sel)[0][len(np.nonzero(sel)[0]) // 2]])
        truth = up.value(up.segment_at(t_mid), t_mid)[0][P50]
        assert abs(truth - profile_ms) < 10.0 and abs(measured - truth) < 8.0
    assert v.cmd_lost.sum() > 0 and v.meta["commandsLost"] == int(v.cmd_lost.sum())


def test_persistence_follows_an_uplink_step_and_rtt_half_does_not(stepped_view):
    v = stepped_view
    t_issue = F.issue_times(int(v.cmd_t[0]), int(v.cmd_t[-1]), v.tps)
    obs = view_observations(v)
    per = F.make_forecasts("persistence", t_issue, None, obs, seed=0)
    half = F.make_forecasts("rtt-half", t_issue, None, obs, seed=0)
    rel = (t_issue - v.cmd_t[0]) / v.tps
    lan = (rel > 5) & (rel < 9)
    slow = (rel > 35) & (rel < 39)
    p = lambda fc, sel: float(np.median(fc.q_ms[sel & fc.valid_q[:, F.UP], F.UP, F.H0, P50]))  # noqa: E731
    assert p(per, slow) - p(per, lan) > 200.0  # it sees the 300 ms uplink
    assert abs(p(half, slow) - p(half, lan)) < 30.0  # ClockSync rejects the slow round trips


def test_effective_times_follow_the_forecast_and_never_run_backwards(stepped_view):
    v = stepped_view
    for level in ("oracle", "poor", "persistence"):
        fc, eff = view_forecasts(v, level, seed=1)
        assert np.all(eff >= v.cmd_t) and np.all(np.diff(eff) >= 0)
        k = np.searchsorted(fc.t, v.cmd_t, side="right") - 1
        i = len(v.cmd_t) // 2
        if fc.valid_q[k[i], F.UP]:
            expected = v.cmd_t[i] + int(round(float(fc.q_ms[k[i], F.UP, F.H0, P50]) / 1e3 * v.tps))
            assert eff[i] >= expected
    _, eff = view_forecasts(v, "absent", seed=1)
    assert np.array_equal(eff, v.cmd_eff)


def test_a_forecast_stream_round_trips_through_forecast_jlog(stepped_view, tmp_path):
    from armpred.contract.forecast_log import read_forecast_log, write_forecast_log

    v = stepped_view
    t_issue = F.issue_times(int(v.cmd_t[0]), int(v.cmd_t[0]) + 5 * v.tps, v.tps)
    fc = F.make_forecasts("persistence", t_issue, None, view_observations(v), seed=0)  # some fields invalid
    path = str(tmp_path / "forecast.jlog")
    write_forecast_log(fc, path, "s", v.tps)
    back = read_forecast_log(path)
    assert np.array_equal(back.t, fc.t)
    for name in ("valid_q", "valid_loss", "valid_outage", "q_ms", "loss", "floor_ms", "outage"):
        assert np.array_equal(getattr(back, name), getattr(fc, name)), name
