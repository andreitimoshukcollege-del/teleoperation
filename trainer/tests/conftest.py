from __future__ import annotations

import os

import pytest

from armpred.contract.session import load_session
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session, write_session

TRACES = os.path.join(os.path.dirname(__file__), "..", "contract", "traces")


@pytest.fixture(scope="session")
def sim_session_dir(tmp_path_factory):
    """One short simulated session with a payload and hand resistance, shared by the tests."""
    d = tmp_path_factory.mktemp("sessions") / "sim-11"
    params = sample_params(11, duration_s=25.0, slider=2.5, payload_grams=100.0, hand_events_per_min=6.0)
    write_session(simulate_session(params), str(d))
    return str(d)


@pytest.fixture(scope="session")
def sim_session(sim_session_dir):
    return load_session(sim_session_dir)
