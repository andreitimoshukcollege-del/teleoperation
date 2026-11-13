"""Where the evaluation's event metrics start counting (t_info, evaluate._t_info), on an impaired link.

The 2026-11 bug hunt, T-01: t_info was the first frame whose ClockSync-estimated sensed time was past the event. On a
150-300 ms link ClockSync's estimate is biased, and 36 of 37 events picked a frame actually sensed before the event,
66-145 ms early, which skewed joint_event_error_deg, the recovery time and uplink_change_window_deg. The view now
carries each frame's true sensed time (st_sensed_true) and t_info is measured against it.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from armpred.contract.profile import JETROVER
from armpred.contract.session import load_session
from armpred.data import schedule as S
from armpred.data.views import load_view, save_view, synthesize
from armpred.evaluate import _t_info
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session, write_session

from .conftest import TRACES


@pytest.fixture(scope="module")
def impaired_view(tmp_path_factory):
    """Hand events and an uplink that steps to 150 and 300 ms, seen through a 300 ms downlink: ClockSync's worst case."""
    up = S.parse("lan@0;150ms-20j-0.5loss@10;lan@20;300ms-60j-2loss-bursty@30")
    p = sample_params(23, duration_s=40.0, slider=2.5, payload_grams=0.0, hand_events_per_min=12.0, uplink_schedule=up.to_json())
    d = str(tmp_path_factory.mktemp("impaired") / "sim-23")
    write_session(simulate_session(p), d)
    return synthesize(load_session(d), JETROVER, S.parse("300ms-60j-2loss-bursty@0"), seed=5, traces_dir=TRACES)


def test_the_view_carries_each_frames_true_sensed_time(impaired_view):
    v = impaired_view
    assert v.st_sensed_true is not None and v.st_sensed_true.shape == v.st_sensed.shape
    # ClockSync's estimate is not the truth on this link: they differ, by more than a millisecond somewhere.
    assert np.abs(v.st_sensed.astype(np.int64) - v.st_sensed_true).max() > v.tps // 1000


def test_every_events_t_info_frame_was_truly_sensed_at_or_after_the_event(impaired_view):
    v = impaired_view
    events = [e for e in _t_info(v) if e[1] != "network"]
    assert len(events) >= 5, "the fixture should have hand and uplink events"
    for t_event, kind, t_info in events:
        at = np.nonzero(v.st_t == t_info)[0]
        assert at.size, f"{kind} at {t_event}: no state released at t_info"
        assert (v.st_sensed_true[at] >= t_event).any(), f"{kind} at {t_event}: t_info's frame was sensed before it"
        # and it is the first such frame: none released earlier was sensed after the event
        earlier = v.st_t < t_info
        assert not (v.st_sensed_true[earlier] >= t_event).any(), f"{kind} at {t_event}: an earlier frame already showed it"


def test_the_true_sensed_time_survives_saving_and_an_old_view_still_loads(impaired_view, tmp_path):
    path = str(tmp_path / "v.npz")
    save_view(impaired_view, path)
    assert np.array_equal(load_view(path).st_sensed_true, impaired_view.st_sensed_true)
    impaired_view_old = impaired_view.__class__(**{**impaired_view.__dict__, "st_sensed_true": None})
    old = str(tmp_path / "old.npz")
    save_view(impaired_view_old, old)
    assert load_view(old).st_sensed_true is None
    assert _t_info(load_view(old))  # falls back to the estimate, as before
