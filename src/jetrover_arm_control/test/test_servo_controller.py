"""Unit tests for ServoController.setPos (wire v3,
teleoperation's docs/adr/0010-absolute-joint-targets-over-local-relay.md, plus the 2026-08-13
constant-speed follow-up): it receives an already-resolved absolute pulse target per call and
writes it directly, computing its own move duration from the distance to travel at a constant
speed (PULSES_PER_SECOND) -- no belief tracking, relative-delta arithmetic, seed-from-read
fallback, or externally-supplied duration. That state now lives entirely in Teleop.RobotHost's
JetRoverPlant on the .NET side.

Mirrors teleoperation's JetRoverPlantTests.cs in spirit: a fake/stub hardware boundary (here, a
stub board object) so this class's own logic -- the write-gate, the speed-based duration
computation, and the write cooldown (which predates and survives both the wire v3 and
constant-speed changes) -- is proven correct without any real serial hardware, ROS node, or
sleep. See ServoController's own class doc for the full rationale.
"""

import pytest

from jetrover_arm_control.servo_controller import servo_controller as servo_controller_module
from jetrover_arm_control.servo_controller.servo_controller import (
    MIN_MOVE_DURATION_SECONDS,
    PULSES_PER_SECOND,
    ServoController,
)

SERVO_ID = 1


class StubBoard:
    """Records writes and returns a controllable (or failing) read, standing in for the real
    STM32 board over serial."""

    def __init__(self):
        self.written = []
        self.read_return = None
        self.read_call_count = 0

    def bus_servo_set_position(self, servo_speed, positions):
        self.written.append((servo_speed, positions))

    def bus_servo_read_position(self, servo_id):
        self.read_call_count += 1
        if self.read_return is None:
            return None
        return [self.read_return]


class StubBoardMgr:
    """Duck-types BoardManager -- ServoController.__init__ only ever calls getBoard()."""

    def __init__(self, board):
        self._board = board

    def getBoard(self):
        return self._board


class FakeClock:
    """A settable stand-in for time.monotonic(), so cooldown behavior is deterministic and the
    tests need no real sleeps."""

    def __init__(self, start=0.0):
        self.now = start

    def __call__(self):
        return self.now


def make_controller(monkeypatch, board, start_time=0.0):
    clock = FakeClock(start_time)
    monkeypatch.setattr(servo_controller_module.time, "monotonic", clock)
    controller = ServoController(boardMgr=StubBoardMgr(board))
    return controller, clock


def test_setpos_writes_the_given_absolute_pulse_directly(monkeypatch):
    """No read, no belief, no relative arithmetic: the given target is written as-is."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=550)
    assert board.read_call_count == 0  # setPos never reads hardware

    clock.now += 10.0  # clear of any cooldown
    controller.setPos(servoID=SERVO_ID, direction=600)

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 550]],
        [[SERVO_ID, 600]],
    ]
    assert board.read_call_count == 0


def test_setpos_first_call_for_a_servo_uses_the_minimum_duration(monkeypatch):
    """No prior sent value means no real distance to measure a speed against -- falls back to
    the minimum duration rather than an undefined or zero-distance one."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=999)  # first-ever call, any target

    assert board.written == [(MIN_MOVE_DURATION_SECONDS, [[SERVO_ID, 999]])]


def test_setpos_duration_is_proportional_to_distance_at_the_configured_speed(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0  # clear of the (short, first-move) cooldown

    distance = 300
    controller.setPos(servoID=SERVO_ID, direction=500 + distance)

    expected_duration = distance / PULSES_PER_SECOND
    assert board.written[-1] == (pytest.approx(expected_duration), [[SERVO_ID, 800]])


def test_setpos_small_delta_still_uses_the_minimum_duration_floor(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0

    # delta=10 -> 10/PULSES_PER_SECOND would be well under MIN_MOVE_DURATION_SECONDS unclamped.
    controller.setPos(servoID=SERVO_ID, direction=510)

    assert board.written[-1] == (MIN_MOVE_DURATION_SECONDS, [[SERVO_ID, 510]])


def test_setpos_has_no_maximum_duration_for_a_large_delta(monkeypatch):
    """Regression test locking in an explicit design choice: unlike the minimum, there is no
    maximum duration clamp -- a large move takes proportionally longer, not a capped amount."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=0)
    clock.now += 10.0

    distance = 900
    controller.setPos(servoID=SERVO_ID, direction=distance)

    expected_duration = distance / PULSES_PER_SECOND  # 3.0s -- deliberately uncapped
    assert board.written[-1] == (pytest.approx(expected_duration), [[SERVO_ID, 900]])


def test_setpos_clamps_the_given_target_to_the_valid_pulse_range(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=-50)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=1200)

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 0]],
        [[SERVO_ID, 1000]],
    ]


def test_a_meaningful_change_from_the_last_sent_value_still_writes(monkeypatch):
    """A target MIN_MEANINGFUL_PULSE_DELTA or more away from the last value actually sent must
    produce a real write, even though it's a small absolute change."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=525)  # delta 25 from last sent

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 500]],
        [[SERVO_ID, 525]],
    ]


def test_a_change_below_the_write_gate_threshold_does_not_write(monkeypatch):
    """Purely a bus-traffic optimization now (docs/adr/0010 -- there is no accumulation left for
    floating-point noise to drift into), but still worth keeping: a target within
    MIN_MEANINGFUL_PULSE_DELTA of the last value actually sent should not produce a redundant
    physical write."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=503)  # delta 3, below the gate (5)

    assert len(board.written) == 1


def test_cooldown_suppresses_a_second_setpos_within_the_window(monkeypatch):
    """Calling setPos again before the servo's cooldown expires is a no-op: no write at all --
    the safe coalesce fix-servo-command-rate-loss relies on. Unrelated to and unaffected by the
    wire v3 belief-removal or constant-speed changes."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=800)  # distance 300 -> duration 1.0s
    assert len(board.written) == 2

    clock.now += 0.5  # still well inside the 1.0s cooldown from the 800 move
    controller.setPos(servoID=SERVO_ID, direction=900)  # would otherwise be a real move

    assert len(board.written) == 2  # no third write


def test_getRawPos_has_no_effect_on_setPos_write_gating_or_duration(monkeypatch):
    """A deliberate behavior change from the pre-wire-v3 design, worth locking in: reads no
    longer correct any write-side state at all. getRawPos exists purely for
    robot_controller_node.py's own feedback-publisher timers now."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500)
    clock.now += 10.0

    board.read_return = 999  # a real feedback read, unrelated to what was actually sent
    assert controller.getRawPos(servoID=SERVO_ID) == 999

    # The write gate and duration still compare against the last value setPos actually sent
    # (500), not the value getRawPos just observed (999) -- a small delta from 500 is still
    # gated.
    controller.setPos(servoID=SERVO_ID, direction=503)
    assert len(board.written) == 1
