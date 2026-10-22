"""The exact lines RobotHost's recorder writes (core/Teleop.RobotHost.Tests/Recording/
SessionRecorderTests.TheLineFormats_AreTheSessionFormats asserts the same strings on the C# side),
parsed by this package's session reader: the two sides of session format v1 meet here."""

from __future__ import annotations

import numpy as np

from armpred.contract.session import parse_commands, parse_frames, _servo_rows

JC = ["JC|100|105|7|99|A|1:0.25:2.5;10:1:0", "JC|1|1|0|0|S|"]
RO = ["RO|5|1:560.5:597.2"]
RI = ["RI|6|1:1:512;2:0:0"]
FR = ["FR|7|3|7|6|5|4|1:1:-0.1;10:0:0.5"]


def _rows(lines):
    return [line.split("|")[1:] for line in lines]


def test_joint_commands_parse():
    c = parse_commands(_rows(JC))
    assert c.t.tolist() == [100, 1] and c.t_deliver.tolist() == [105, 1]
    assert c.seq.tolist() == [7, 0] and c.capture.tolist() == [99, 0]
    assert [chr(o) for o in c.outcome] == ["A", "S"]
    assert c.count.tolist() == [2, 0]
    assert c.motor[0, :2].tolist() == [1, 10]
    assert np.allclose(c.angle[0, :2], [0.25, 1.0]) and np.allclose(c.speed[0, :2], [2.5, 0.0])


def test_relay_rows_parse():
    out = _servo_rows(_rows(RO), 1, 3, 2)
    assert out.t.tolist() == [5] and out.motor.tolist() == [1]
    assert np.isclose(out.pulse[0], 560.5) and np.isclose(out.extra[0], 597.2, rtol=1e-6)
    inp = _servo_rows(_rows(RI), 1, 3, 1)
    assert inp.motor.tolist() == [1, 2] and inp.pulse.tolist() == [512.0, 0.0] and inp.extra.tolist() == [1.0, 0.0]


def test_frames_parse():
    f = parse_frames(_rows(FR))
    assert (f.t[0], f.seq[0], f.robot_send[0], f.sensed[0], f.echo_operator[0], f.echo_robot_recv[0]) == (7, 3, 7, 6, 5, 4)
    assert f.motor[0, :2].tolist() == [1, 10] and f.flags[0, :2].tolist() == [1, 0]
    assert np.allclose(f.value[0, :2], [-0.1, 0.5])
