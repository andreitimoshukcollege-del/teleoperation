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
    assert g.is_gripper and g.feedback  # read by the real driver too, since PR #70 (FEEDBACK_MOTOR_IDS)
    assert m.value(10, g.open_pulse) == pytest.approx(-g.finger_open)  # open: past the CAD's pose
    assert m.value(10, g.closed_pulse) == pytest.approx(g.finger_travel)
    assert m.value(10, (g.open_pulse + g.closed_pulse) / 2) == pytest.approx((g.finger_travel - g.finger_open) / 2)
    for pulse in (g.open_pulse, 300.0, g.closed_pulse):
        assert m.pulse(10, m.value(10, pulse)) == pytest.approx(pulse)


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


def test_a_cycle_reads_motors_1_to_5_and_the_gripper_back_to_back_every_20_ms_and_the_relay_reports_every_20_ms_in_whole_pulses():
    # The real driver (robot_controller_node.py, pollRate 0.02) and relay (relay_node.py, 0.02 s), since 2026-10-25.
    m = model()
    fb = servo.FeedbackEmulator(m)
    sent = []
    reads = []
    t = 0.0
    while t < 0.5:
        mid = fb.due_read(t)
        if mid is not None:
            reads.append((mid, t))
            fb.record(mid, 500.4 + mid, t)
        out = fb.due_feedback(t)
        if out is not None:
            sent.append((t, out))
        t = round(t + 0.001, 6)
    assert [mid for mid, _ in reads][:7] == [1, 2, 3, 4, 5, 10, 1]
    times = [t for _, t in reads]
    assert times[6] - times[0] == pytest.approx(0.02, abs=0.0015)  # a cycle every 20 ms
    assert times[4] - times[0] == pytest.approx(4 * 0.003, abs=0.004)  # its reads back to back, ~2.9 ms each
    assert len(sent) == 25  # every 20 ms
    last = dict((mid, (valid, pulse)) for mid, valid, pulse in sent[-1][1])
    assert last[10] == (True, 510.0)
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


def test_a_conversion_is_named_by_the_urdf_and_its_meshes(tmp_path):
    # A re-exported mesh keeps its name; Isaac must convert again (docs/adr/0034 amendment 7).
    from teleop_isaac.jetrover_bridge import urdf_digest

    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "a.obj").write_text("v 0 0 0\n")
    urdf = tmp_path / "r.urdf"
    urdf.write_text('<robot><link name="a"><visual><geometry><mesh filename="meshes/a.obj"/></geometry></visual></link></robot>')
    before = urdf_digest(str(urdf))
    (tmp_path / "meshes" / "a.obj").write_text("v 1 0 0\n")
    assert urdf_digest(str(urdf)) != before
