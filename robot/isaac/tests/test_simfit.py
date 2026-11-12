"""The sim-against-real replay and fit (teleop_isaac/simfit.py, docs/metrics.md section 10), with a servo that follows its ramp."""

import pytest

from teleop_isaac import servo, simfit


class RampSim:
    """A servo that is exactly its timed ramp, ``lag_ms`` behind it: what a perfect fit of a lagging servo looks like."""

    dt = 0.001

    def __init__(self, lag_ms=0.0):
        self.lag_steps = int(round(lag_ms))
        self.history = {}
        self.setpoints = {}
        self.on = True

    def reset(self, pulses, params):
        self.setpoints = {m: servo.TimedSetpoint(p) for m, p in pulses.items()}
        self.history = {m: [p] for m, p in pulses.items()}
        self.on = True

    def command(self, motor, pulse, pps):
        if self.on and motor in self.setpoints:
            self.setpoints[motor].move(pulse, pps)

    def write(self, motor, pulse, seconds):
        if self.on and motor in self.setpoints:
            self.setpoints[motor].move_over(pulse, seconds)

    def torque(self, on):
        self.on = on

    def step(self):
        for m, sp in self.setpoints.items():
            self.history[m].append(sp.step(self.dt))

    def read(self, motor):
        h = self.history[motor]
        return h[max(0, len(h) - 1 - self.lag_steps)]


def recorded(lag_ms, seconds=3.0):
    """A synthetic session recorded from a servo ``lag_ms`` behind its ramp: motor 2 stepped from 500 to 700 at 1 s, read
    with the real driver's cycles and sent every 20 ms."""
    model = servo.SimModel.load()
    tps = 1_000_000_000
    sim = RampSim(lag_ms)
    start = {m: 500.0 for m in model.motors}
    sim.reset(start, simfit.Params())
    fb = servo.FeedbackEmulator(model)
    events, t, sent = [], 0.0, False
    while t <= seconds:
        if not sent and t >= 1.0:
            events.append([int(t * tps), "RO", [[2, 700.0, 400.0]]])
            sim.command(2, 700.0, 400.0)
            sent = True
        mid = fb.due_read(t)
        while mid is not None:
            fb.record(mid, sim.read(mid), t)
            mid = fb.due_read(t)
        out = fb.due_feedback(t)
        if out is not None:
            events.append([int(t * tps), "RI", [[m, int(v), p] for m, v, p in out if m != 10 or v]])
        sim.step()
        t = round(t + 0.001, 6)
    return {"name": "synthetic", "tps": tps, "events": events}


def test_a_simulator_that_is_the_robot_scores_nothing_and_a_lagging_one_scores_its_lag():
    model = servo.SimModel.load()
    session = recorded(lag_ms=40)
    r = simfit.Replayer(model, gate=False)  # an ungated recording, replayed as before amendment 13
    same = r.run(session, RampSim(40), simfit.Params()).summary()["sim_tracking_error_deg"]["pooled1to4"]
    assert same["n"] > 100 and same["p95"] <= r.deg(1.0) + 1e-9  # whole pulses only
    off = r.run(session, RampSim(0), simfit.Params()).summary()
    # 40 ms at 400 pulses/s is 16 pulses, about 3.8 degrees, while the step ramps; nothing once settled.
    assert off["sim_tracking_error_deg"]["motors"]["2"]["p95"] == pytest.approx(r.deg(16), abs=r.deg(2))
    assert off["sim_settle_error_deg"]["motors"]["2"]["p95"] <= r.deg(1.0) + 1e-9


def test_the_command_delay_parameter_reproduces_a_lagging_servo():
    model = servo.SimModel.load()
    session = recorded(lag_ms=40)
    r = simfit.Replayer(model, gate=False)  # an ungated recording, replayed as before amendment 13
    delayed = r.run(session, RampSim(0), simfit.Params(delay_ms=40)).summary()["sim_tracking_error_deg"]["motors"]["2"]
    assert delayed["p95"] <= r.deg(5.0)  # the driver's read phase: a few whole pulses (16 without the delay)


def test_the_fit_finds_the_delay_and_is_deterministic():
    model = servo.SimModel.load()
    session = recorded(lag_ms=40)
    r = simfit.Replayer(model, gate=False)  # an ungated recording, replayed as before amendment 13

    def evaluate(p):
        return r.run(session, RampSim(0), p).objective()

    best, score, history = simfit.fit(evaluate, rounds=2)
    assert abs(best.delay_ms - 40) <= 10 and score <= r.deg(4.0)
    again, _, _ = simfit.fit(evaluate, rounds=2)
    assert again == best and len(history) > 5


def test_a_session_extracts_its_relay_records_in_time_order(tmp_path):
    d = tmp_path / "20261026T000000Z-x"
    d.mkdir()
    (d / "robothost.jlog").write_text("JSES|1|robothost|1000000000|CLOCK_MONOTONIC|x\n"
                                      "RI|200|1:1:500;10:0:0\nRO|100|1:499.5:1432.4;10:129:1432.4\nTQ|150|0\nJC|160|junk\n")
    s = simfit.extract_session(str(d))
    assert s["tps"] == 1_000_000_000 and s["name"] == "20261026T000000Z-x"
    assert [e[1] for e in s["events"]] == ["RO", "TQ", "RI"]
    assert s["events"][0][2][0] == [1, 499.5, 1432.4] and s["events"][2][2][1] == [10, 0, 0.0]


def recorded_gated(seconds=3.0):
    """A synthetic session from an arm behind the real driver (docs/adr/0034 amendment 13): motor 2 commanded along a
    fast sweep every 20 ms, so the gate's cooldown and the single thread decide when each move starts. Its read cycles are
    kept as a real session's driver.jlog keeps them."""
    model = servo.SimModel.load()
    tps = 1_000_000_000
    sim = RampSim(0)
    start = {m: 500.0 for m in model.motors}
    sim.reset(start, simfit.Params())
    fb = servo.FeedbackEmulator(model)
    drv = servo.DriverEmulator(model, fb, servo_move=sim.write, servo_command=sim.command, servo_torque=sim.torque)
    events, t, k = [], 0.0, 0
    cycles = []
    while t <= seconds:
        if t >= 0.5 and t < 2.5 and abs(t - (0.5 + 0.02 * k)) < 1e-6:
            pulse = 500.0 + 150.0 * (1 if (k // 15) % 2 == 0 else -1) * ((k % 15) / 15.0)
            events.append([round(t * tps), "RO", [[2, pulse, 900.0]]])
            drv.command(t, [(2, pulse, 900.0)])
            k += 1
        mid = drv.due_read(t)
        while mid is not None:
            if fb._turn == 1 or (len(fb.order) == 1):  # the cycle's first read
                cycles.append([round(t * tps), None, []])
            cycles[-1][2].append([round(t * tps), mid])
            cycles[-1][1] = round((t + fb.read_seconds) * tps)
            drv.record(mid, sim.read(mid), t)
            mid = drv.due_read(t)
        out = fb.due_feedback(t)
        if out is not None:
            events.append([round(t * tps), "RI", [[m, int(v), p] for m, v, p in out if m != 10 or v]])
        sim.step()
        t = round(t + 0.001, 6)
    return {"name": "synthetic-gated", "tps": tps, "events": events, "cycles": cycles}


def test_a_gated_arm_is_reproduced_by_the_gated_replay_and_not_by_the_ungated_one():
    model = servo.SimModel.load()
    session = recorded_gated()
    def p95(replayer):
        return replayer.run(session, RampSim(0), simfit.Params()).summary()["sim_tracking_error_deg"]["motors"]["2"]

    one = simfit.Replayer(model).deg(1.0)
    gated = p95(simfit.Replayer(model))  # the gate, on the session's own read cycles
    assert gated["n"] > 100 and gated["p95"] <= one + 1e-9
    assert p95(simfit.Replayer(model, gate=False))["p95"] > 3 * one
    # The gate on cycles of the replay's own, out of phase with the recording's: close, not exact (a few targets differ).
    assert 0 < p95(simfit.Replayer(model, cycles="emulated"))["p95"] <= 4 * one
