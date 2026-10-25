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
    # The arm joints: a still gripper's exact zeros would otherwise set both medians.
    arm = d.joint < 5
    ok = d.valid & arm
    prior = np.percentile(np.abs(d.prior - d.truth)[ok], 50)
    hold = np.percentile(np.abs(d.newest - d.truth)[ok], 50)
    assert prior < hold
    # The gripper is labelled too (layout v3), in servo radians: 0 open .. 1 / GRIPPER_FRACTION_PER_RAD closed.
    from armpred.features.layout import GRIPPER_FRACTION_PER_RAD
    g = d.joint == 5
    assert g.sum() > 20
    assert (d.truth[g] >= -1e-9).all() and (d.truth[g] <= 1.0 / GRIPPER_FRACTION_PER_RAD + 1e-9).all()


def test_a_scheduled_uplink_is_logged_and_every_send_is_in_the_operator_log(tmp_path):
    from armpred.contract.check import check_session_dir
    from armpred.contract.session import load_session
    from armpred.sim.robot import write_session

    up = S.parse("lan@0;150ms-20j-0.5loss@4;lan@8")
    p = sample_params(31, duration_s=12.0, slider=2.5, payload_grams=0.0, hand_events_per_min=0.0,
                      uplink_schedule=up.to_json())
    out = simulate_session(p)
    ups = [r for r in out.robothost if r[0] == "UP"]
    assert [r[2] for r in ups] == ["none", "150ms-20j-0.5loss", "none"]
    d = str(tmp_path / "s")
    write_session(out, d)
    assert check_session_dir(d, allow_read_gaps=True).ok
    s = load_session(d)
    assert len(s.operator_commands) >= len(s.commands) and len(s.uplink_changes) == 3
    v = synthesize(s, JETROVER, S.parse("lan@0"), seed=1, traces_dir=TRACES)
    assert len(v.cmd_t) == len(s.operator_commands)
    assert int(v.cmd_lost.sum()) == len(s.operator_commands) - len(set(s.commands.seq.tolist()))
    assert sum(k == "uplink" for k in v.events_kind) == 3


def test_a_queue_fills_and_an_outage_holds_then_bursts():
    from armpred.net.profiles import NetworkProfile

    tps = 1_000_000_000
    queue = S.Segment(0.0, "queue", False, profile=NetworkProfile(10_000_000, 0, 0.0, 0.0),
                      queue_peak_ms=200.0, queue_fill_ms_per_s=400.0)
    outage = S.Segment(10.0, "outage", False, profile=NetworkProfile(10_000_000, 0, 0.0, 0.0),
                       outage_at_s=12.0, outage_s=1.0)
    sched = S.Schedule(segments=(queue, outage))
    send_s = np.arange(0.0, 20.0, 0.02)
    send = (send_s * tps).astype(np.int64)
    delivered, arrival, _ = S.build_link(sched, send, send_s, seed=3, tps=tps, traces_dir=None)
    delay_ms = (arrival - send) / 1e6
    in_queue = (send_s < 10.0) & delivered
    assert delay_ms[in_queue].max() > 180.0 and delay_ms[in_queue].min() < 20.0  # the sawtooth
    assert 0.0 < 1.0 - delivered[send_s < 10.0].mean() < 0.1  # tail drop near the top only
    held = (send_s >= 12.0) & (send_s < 13.0) & delivered
    assert np.all(arrival[held] >= 13.0 * tps)  # nothing arrives before the outage ends
    assert np.all(delay_ms[(send_s > 14.0) & delivered] < 20.0)


def test_a_version_1_view_still_loads(sim_session, tmp_path):
    import json

    v = synthesize(sim_session, JETROVER, S.parse("lan@0"), seed=2, traces_dir=TRACES)
    arrays = {name: getattr(v, name) for name in ("cmd_t", "cmd_eff", "cmd_speed", "cmd_count", "cmd_motor", "cmd_value",
                                                  "st_t", "st_sensed", "st_count", "st_motor", "st_value", "st_flags",
                                                  "tr_t", "tr_motor", "tr_value", "frame_send", "frame_delivered",
                                                  "frame_arrival", "frame_segment", "sync_t", "sync_offset",
                                                  "sync_uncertainty", "sync_accepted", "events_t")}
    meta = {k: v.meta[k] for k in ("session", "source", "schedule", "namedOnly", "seed", "operatorTicksPerSecond",
                                   "robotTicksPerSecond", "operatorUptimeNs", "operatorDriftPpm", "conditions")}
    meta.update({"format": "jetrover-view", "version": 1})
    arrays["meta_json"] = np.frombuffer(json.dumps(meta).encode("utf-8"), dtype=np.uint8)
    arrays["events_kind"] = np.array(v.events_kind, dtype="U32")
    arrays["events_detail"] = np.array(v.events_detail, dtype="U64")
    path = str(tmp_path / "v1.npz")
    np.savez(path, **arrays)
    w = load_view(path)
    assert w.meta["version"] == 1 and not w.cmd_lost.any() and (w.cmd_true_uplink == -1).all()
    assert np.array_equal(w.cmd_eff, v.cmd_eff)
