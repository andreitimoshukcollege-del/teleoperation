from __future__ import annotations

import numpy as np

from armpred.contract.profile import JETROVER
from armpred.data import schedule as S
from armpred.data.dataset import SampleConfig, view_samples
from armpred.data.views import load_view, save_view, synthesize
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session

from .conftest import TRACES


def test_the_simulator_is_deterministic():
    p = sample_params(5, duration_s=5.0, slider=2.5, payload_grams=0.0, hand_events_per_min=0.0)
    a, b = simulate_session(p), simulate_session(p)
    assert a.robothost == b.robothost and a.driver == b.driver


def test_a_payload_slows_and_lowers_the_arm():
    # Same everything, with and without 250 g: the shoulder carries more, so it droops.
    light = simulate_session(sample_params(6, duration_s=8.0, slider=2.5, payload_grams=0.0, hand_events_per_min=0.0))
    heavy = simulate_session(sample_params(6, duration_s=8.0, slider=2.5, payload_grams=250.0, hand_events_per_min=0.0))
    load = lambda out: np.mean([abs(r[5]) for r in out.latents if r[0] == "LJ" and r[2] == 2])  # noqa: E731
    assert load(heavy) > load(light)


def test_a_lan_view_delivers_everything_and_syncs(sim_session):
    v = synthesize(sim_session, JETROVER, S.parse("lan@0"), seed=1, traces_dir=TRACES)
    assert v.frame_delivered.all()
    assert v.sync_accepted.sum() >= 3
    # A state reaches the predictor only once synced, so never before the third accepted round trip.
    third = v.sync_t[np.nonzero(v.sync_accepted)[0][2]]
    assert v.st_t.min() >= third
    # Effective time is send plus RTT/2 once synced: never before the send.
    assert np.all(v.cmd_eff >= v.cmd_t)
    assert np.all(np.diff(v.cmd_t) >= 0)


def test_a_lossy_profile_drops_frames_and_a_view_is_seeded(sim_session):
    a = synthesize(sim_session, JETROVER, S.parse("300ms-60j-2loss-bursty@0"), seed=4, traces_dir=TRACES)
    b = synthesize(sim_session, JETROVER, S.parse("300ms-60j-2loss-bursty@0"), seed=4, traces_dir=TRACES)
    c = synthesize(sim_session, JETROVER, S.parse("300ms-60j-2loss-bursty@0"), seed=5, traces_dir=TRACES)
    assert 0.9 < a.frame_delivered.mean() < 1.0
    assert np.array_equal(a.st_sensed, b.st_sensed) and np.array_equal(a.cmd_eff, b.cmd_eff)
    assert not np.array_equal(a.frame_arrival, c.frame_arrival)


def test_a_view_round_trips_through_its_file(sim_session, tmp_path):
    v = synthesize(sim_session, JETROVER, S.parse("50ms-5j@0"), seed=2, traces_dir=TRACES)
    path = str(tmp_path / "v.npz")
    save_view(v, path)
    w = load_view(path)
    assert w.meta == v.meta and np.array_equal(w.st_value, v.st_value) and w.events_kind == v.events_kind


def test_uplink_shifts_are_slope_limited():
    import pytest

    with pytest.raises(ValueError):
        S.UplinkShift(((0.0, 0.0), (1.0, 100.0)))  # 100 ms in a second is twice the limit
    S.UplinkShift(((0.0, 0.0), (10.0, 400.0)))


def test_samples_are_finite_and_the_prior_beats_hold_last_on_a_slow_link(sim_session):
    v = synthesize(sim_session, JETROVER, S.parse("150ms-20j-0.5loss@0"), seed=3, traces_dir=TRACES)
    d = view_samples(v, np.random.default_rng(0), SampleConfig(queries_per_s=40.0))
    assert len(d) > 200
    assert np.isfinite(d.joints).all() and np.isfinite(d.arm).all()
    ok = d.valid
    prior = np.percentile(np.abs(d.prior - d.truth)[ok], 50)
    hold = np.percentile(np.abs(d.newest - d.truth)[ok], 50)
    assert prior < hold
