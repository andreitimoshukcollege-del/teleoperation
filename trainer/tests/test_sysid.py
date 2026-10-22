"""sysid recovers what the simulator hid: the servo lag, the order of the reading ages (motors are
read one after another, so motor 1's reading is the oldest), and the bus timing."""

from __future__ import annotations

import numpy as np

from armpred import sysid
from armpred.contract.session import load_session
from armpred.sim.params import sample_params
from armpred.sim.robot import simulate_session, write_session


def test_sysid_recovers_the_simulators_lag_ages_and_bus(tmp_path):
    d = str(tmp_path / "s")
    params = sample_params(41, duration_s=40.0, slider=3.8, payload_grams=0.0, hand_events_per_min=0.0)
    write_session(simulate_session(params), d)
    result = sysid.identify([d], servo_seconds=40.0)
    true_lag = np.array(params.servo_lag_s)
    fitted = np.array([result["servo"][str(m)][0]["lag_s"] for m in (1, 2, 3, 4, 5)])
    assert np.all(np.abs(fitted - true_lag) <= np.maximum(0.006, 0.35 * true_lag)), (fitted, true_lag)
    ages = [result["readingAgesS"][str(m)] for m in (1, 2, 3, 4, 5)]
    assert ages[0] > ages[-1] > 0.0
    assert abs(result["bus"]["read_ms"] - params.read_ms) < 1.5
    assert abs(result["robothostLoopMs"] - params.robothost_loop_ms) < 0.2
    # The simulator's pipeline: half a ~5.2 ms loop, the relay's 20 ms poll (10 ms on average), the
    # driver's queue: tens of milliseconds, never more.
    assert 0.003 < result["pipelineDelayS"] < 0.05, result["pipelineDelayS"]
    servo = result["relayReplayServo"]
    assert abs(servo["servoLagSeconds"] - float(np.median(true_lag))) <= max(0.006, 0.35 * float(np.median(true_lag)))
    js, tm = sysid.write(result, str(tmp_path / "out"))
    assert "[ranges]" in open(tm).read()
    import json

    prior = json.load(open(str(tmp_path / "out" / "prior.json")))
    assert prior["relayReplayServo"]["servoPipelineDelaySeconds"] == servo["servoPipelineDelaySeconds"]
