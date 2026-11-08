"""The JetRover's servos and driver as the Isaac bridge stands in for them (teleop_isaac/servo.py), without Isaac."""

import math

import pytest

from teleop_isaac import servo


def model():
    return servo.SimModel.load()


def test_a_pulse_means_the_angle_generic_arm_plant_means():
    # GenericArmPlant: pulse = zeroPulse + (angle + zeroOffset) * pulsePerRadian; 1000 pulses span 240 degrees.
    m = model()
    assert m.pulse_per_radian == pytest.approx(1000 / math.radians(240), rel=1e-6)
    assert m.value(1, 500.0) == pytest.approx(0.0)
    assert m.value(2, 500.0) == pytest.approx(-0.12217305)  # the shoulder's mounting offset
    for mid in (1, 2, 3, 4, 5):
        for a in (-1.0, 0.0, 0.7):
            assert m.value(mid, m.pulse(mid, a)) == pytest.approx(a)


def test_the_gripper_maps_open_and_closed_pulses_onto_finger_travel_and_never_reports():
    m = model()
    g = m.motors[10]
    assert g.is_gripper and not g.feedback
    assert m.value(10, g.open_pulse) == 0.0
    assert m.value(10, g.closed_pulse) == pytest.approx(g.finger_travel)
    assert m.value(10, (g.open_pulse + g.closed_pulse) / 2) == pytest.approx(g.finger_travel / 2)


def test_a_servo_ramps_to_its_target_over_delta_over_speed_and_a_new_move_starts_where_the_ramp_is():
    s = servo.TimedSetpoint(500)
    s.move(700, 400)  # 200 pulses at 400/s: 0.5 s
    for _ in range(250):
        s.step(0.001)
    assert s.pulse == pytest.approx(600, abs=1e-6)
    s.move(500, 1000)  # from 600, back to 500 at 1000/s: 0.1 s
    for _ in range(99):
        s.step(0.001)
    assert s.pulse > 500
    s.step(0.002)
    assert s.pulse == 500
    s.hold(512.4)
    s.step(1.0)
    assert (s.pulse, s.target) == (512.4, 512.4)


def test_motors_are_read_in_turn_every_10_ms_and_reported_every_50_ms_in_whole_pulses():
    m = model()
    fb = servo.FeedbackEmulator(m)
    sent = []
    reads = []
    t = 0.0
    while t < 0.5:
        mid = fb.due_read(t)
        if mid is not None:
            reads.append(mid)
            fb.record(mid, 500.4 + mid, t)
        out = fb.due_feedback(t)
        if out is not None:
            sent.append((t, out))
        t = round(t + 0.001, 6)
    assert reads[:6] == [1, 2, 3, 4, 5, 1]
    assert len(sent) == 10  # every 50 ms
    last = dict((mid, (valid, pulse)) for mid, valid, pulse in sent[-1][1])
    assert last[10] == (False, 0.0)
    assert last[3] == (True, 503.0)  # whole pulses
    # A motor not read for longer than 0.2 s goes invalid.
    fb2 = servo.FeedbackEmulator(m)
    fb2.record(1, 500, 0.0)
    out = fb2.due_feedback(0.25)
    assert dict((mid, valid) for mid, valid, _ in out)[1] is False


def test_the_relay_protocol_is_the_ros_relays_own():
    rp = servo.relay_protocol()
    assert rp.VERSION == 5
    data = rp.encode_feedback([(1, True, 500.0), (10, False, 0.0)])
    assert rp.kind_of(data) == rp.KIND_FEEDBACK


def test_home_is_the_collision_models_home_pose():
    m = model()
    home = servo.home_pulses(m, (0.0, 1.0472, -1.5708, 0.5236, 0.0))
    assert m.value(2, home[2]) == pytest.approx(1.0472)
    assert home[10] == m.motors[10].open_pulse


def test_the_checks_robothost_side_of_the_protocol_is_what_the_relay_reads():
    import jetrover_check

    rp = servo.relay_protocol()
    data = jetrover_check.encode_command([(3, 612.0, 400.0), (10, 116.0, 2200.0)])
    assert rp.decode_arm_command(data) == [(3, 612.0, 400.0), (10, 116.0, 2200.0)]
    fb = rp.encode_feedback([(1, True, 501.0), (10, False, 0.0)])
    assert jetrover_check.decode_feedback(rp, fb) == [(1, True, 501.0), (10, False, 0.0)]
