"""Unit tests for ServoController.setPos (wire v4,
teleoperation's docs/adr/0011-generic-robot-arm-profiles.md): it receives an already-resolved
absolute pulse target AND an explicit move speed per call and writes it directly -- no belief
tracking, relative-delta arithmetic, seed-from-read fallback, or locally hardcoded speed constant.
That state now lives entirely in Teleop.RobotHost's GenericArmPlant on the .NET side, which also
supplies the speed explicitly over the wire now (previously a module constant here).

Mirrors teleoperation's GenericArmPlantTests.cs in spirit: a fake/stub hardware boundary (here, a
stub board object) so this class's own logic -- the write-gate, the speed-based duration
computation, and the write cooldown (which predates and survives the wire v3/v4 and constant-speed
changes) -- is proven correct without any real serial hardware, ROS node, or sleep. See
ServoController's own class doc for the full rationale.
"""

import pytest

from jetrover_arm_control.servo_controller import servo_controller as servo_controller_module
from jetrover_arm_control.servo_controller.servo_controller import (
    MIN_MOVE_DURATION_SECONDS,
    WRITE_SETTLE_SECONDS,
    ServoController,
)

SERVO_ID = 1
PULSES_PER_SECOND = 300  # an arbitrary test speed -- no longer a module constant in production code


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

    controller.setPos(servoID=SERVO_ID, direction=550, pulses_per_second=PULSES_PER_SECOND)
    assert board.read_call_count == 0  # setPos never reads hardware

    clock.now += 10.0  # clear of any cooldown
    controller.setPos(servoID=SERVO_ID, direction=600, pulses_per_second=PULSES_PER_SECOND)

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

    controller.setPos(servoID=SERVO_ID, direction=999, pulses_per_second=PULSES_PER_SECOND)  # first-ever call

    assert board.written == [(MIN_MOVE_DURATION_SECONDS, [[SERVO_ID, 999]])]


def test_setpos_duration_is_proportional_to_distance_at_the_given_speed(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0  # clear of the (short, first-move) cooldown

    distance = 300
    controller.setPos(servoID=SERVO_ID, direction=500 + distance, pulses_per_second=PULSES_PER_SECOND)

    expected_duration = distance / PULSES_PER_SECOND
    assert board.written[-1] == (pytest.approx(expected_duration), [[SERVO_ID, 800]])


def test_setpos_a_different_speed_changes_the_computed_duration(monkeypatch):
    """The speed is now supplied per call (docs/adr/0011), not a fixed module constant -- two
    calls covering the same distance at different speeds must produce different durations."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=100)
    clock.now += 10.0

    controller.setPos(servoID=SERVO_ID, direction=800, pulses_per_second=100)
    assert board.written[-1] == (pytest.approx(300 / 100), [[SERVO_ID, 800]])

    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=100)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=800, pulses_per_second=600)
    assert board.written[-1] == (pytest.approx(300 / 600), [[SERVO_ID, 800]])


def test_setpos_small_delta_still_uses_the_minimum_duration_floor(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0

    # delta=10 -> 10/PULSES_PER_SECOND would be well under MIN_MOVE_DURATION_SECONDS unclamped.
    controller.setPos(servoID=SERVO_ID, direction=510, pulses_per_second=PULSES_PER_SECOND)

    assert board.written[-1] == (MIN_MOVE_DURATION_SECONDS, [[SERVO_ID, 510]])


def test_setpos_has_no_maximum_duration_for_a_large_delta(monkeypatch):
    """Regression test locking in an explicit design choice: unlike the minimum, there is no
    maximum duration clamp -- a large move takes proportionally longer, not a capped amount."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=0, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0

    distance = 900
    controller.setPos(servoID=SERVO_ID, direction=distance, pulses_per_second=PULSES_PER_SECOND)

    expected_duration = distance / PULSES_PER_SECOND  # 3.0s -- deliberately uncapped
    assert board.written[-1] == (pytest.approx(expected_duration), [[SERVO_ID, 900]])


def test_setpos_clamps_the_given_target_to_the_valid_pulse_range(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=-50, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=1200, pulses_per_second=PULSES_PER_SECOND)

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 0]],
        [[SERVO_ID, 1000]],
    ]


def test_a_meaningful_change_from_the_last_sent_value_still_writes(monkeypatch):
    """A target MIN_MEANINGFUL_PULSE_DELTA or more away from the last value actually sent must
    produce a real write, even though it's a small absolute change."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=525, pulses_per_second=PULSES_PER_SECOND)  # delta 25

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 500]],
        [[SERVO_ID, 525]],
    ]


def test_a_change_below_the_write_gate_threshold_does_not_write(monkeypatch):
    """Purely a bus-traffic optimization now (docs/adr/0010/0011 -- there is no accumulation left
    for floating-point noise to drift into), but still worth keeping: a target within
    MIN_MEANINGFUL_PULSE_DELTA of the last value actually sent should not produce a redundant
    physical write."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=503, pulses_per_second=PULSES_PER_SECOND)  # delta 3, below the gate (5)

    assert len(board.written) == 1


def test_cooldown_defers_a_second_setpos_within_the_window(monkeypatch):
    """Calling setPos again before the servo's cooldown expires writes nothing yet -- the
    coalesce fix-servo-command-rate-loss relies on. The target is kept for flushPending."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0
    controller.setPos(servoID=SERVO_ID, direction=800, pulses_per_second=PULSES_PER_SECOND)  # distance 300 -> duration 1.0s
    assert len(board.written) == 2

    clock.now += 0.5  # still well inside the 1.0s cooldown from the 800 move
    controller.setPos(servoID=SERVO_ID, direction=900, pulses_per_second=PULSES_PER_SECOND)  # would otherwise be a real move

    assert len(board.written) == 2  # no third write


def test_getRawPos_has_no_effect_on_setPos_write_gating_or_duration(monkeypatch):
    """A deliberate behavior change locked in since wire v3, still true under v4: reads no longer
    correct any write-side state at all. getRawPos exists purely for robot_controller_node.py's
    own feedback-polling loop now, returning raw pulse directly (no degree conversion)."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 10.0

    board.read_return = 999  # a real feedback read, unrelated to what was actually sent
    assert controller.getRawPos(servoID=SERVO_ID) == 999

    # The write gate and duration still compare against the last value setPos actually sent
    # (500), not the value getRawPos just observed (999) -- a small delta from 500 is still
    # gated.
    controller.setPos(servoID=SERVO_ID, direction=503, pulses_per_second=PULSES_PER_SECOND)
    assert len(board.written) == 1


def test_flush_writes_the_pending_target_once_the_cooldown_ends(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)  # cooldown 0.05s
    clock.now += 0.01
    controller.setPos(servoID=SERVO_ID, direction=530, pulses_per_second=PULSES_PER_SECOND)
    controller.flushPending()
    assert len(board.written) == 1  # still cooling down

    clock.now += 0.05
    controller.flushPending()
    assert board.written[-1] == (pytest.approx(30 / PULSES_PER_SECOND), [[SERVO_ID, 530]])

    controller.flushPending()
    assert len(board.written) == 2  # written once, not again


def test_the_newest_command_in_a_cooldown_wins(monkeypatch):
    """On the single-threaded executor commands arrive in bursts; the servo must chase the
    newest of them, not the first."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.01
    for target in (510, 520, 540):
        controller.setPos(servoID=SERVO_ID, direction=target, pulses_per_second=PULSES_PER_SECOND)

    clock.now += 0.05
    controller.flushPending()
    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 500]],
        [[SERVO_ID, 540]],
    ]


def test_a_direct_write_supersedes_the_pending_target(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.01
    controller.setPos(servoID=SERVO_ID, direction=520, pulses_per_second=PULSES_PER_SECOND)  # pending
    clock.now += 0.05
    controller.setPos(servoID=SERVO_ID, direction=560, pulses_per_second=PULSES_PER_SECOND)  # cooldown over: written
    clock.now += 10.0
    controller.flushPending()

    assert [positions for _, positions in board.written] == [
        [[SERVO_ID, 500]],
        [[SERVO_ID, 560]],
    ]


def test_pending_targets_are_per_servo(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=1, direction=500, pulses_per_second=PULSES_PER_SECOND)
    controller.setPos(servoID=2, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.01
    controller.setPos(servoID=1, direction=530, pulses_per_second=PULSES_PER_SECOND)
    controller.setPos(servoID=2, direction=470, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.05
    controller.flushPending()

    assert board.written[2:] == [(pytest.approx(30 / PULSES_PER_SECOND), [[1, 530], [2, 470]])]  # one packet


def test_a_pending_target_within_the_write_gate_is_not_written(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.01
    controller.setPos(servoID=SERVO_ID, direction=503, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.05
    controller.flushPending()

    assert len(board.written) == 1


def test_reset_drops_pending_targets(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.01
    controller.setPos(servoID=SERVO_ID, direction=600, pulses_per_second=PULSES_PER_SECOND)
    controller.resetArm()
    clock.now += 10.0
    controller.flushPending()

    assert all(positions != [[SERVO_ID, 600]] for _, positions in board.written)


def test_servos_that_move_together_share_one_packet(monkeypatch):
    """With the headset streaming, one packet per servo then a read lost the read; servos that
    may move at the same moment, with the same duration, go out together."""
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)

    written = controller.setPositions([(1, 500, PULSES_PER_SECOND), (2, 600, PULSES_PER_SECOND), (3, 700, PULSES_PER_SECOND)])

    assert written == 1
    assert board.written == [(MIN_MOVE_DURATION_SECONDS, [[1, 500], [2, 600], [3, 700]])]


def test_different_durations_go_in_separate_packets(monkeypatch):
    board = StubBoard()
    controller, clock = make_controller(monkeypatch, board)
    controller.setPositions([(1, 500, PULSES_PER_SECOND), (2, 500, PULSES_PER_SECOND)])
    clock.now += 10.0

    written = controller.setPositions([(1, 800, PULSES_PER_SECOND), (2, 510, PULSES_PER_SECOND)])

    assert written == 2
    packets = {positions[0][0]: (duration, positions) for duration, positions in board.written[1:]}
    assert packets[1] == (pytest.approx(300 / PULSES_PER_SECOND), [[1, 800]])
    assert packets[2] == (MIN_MOVE_DURATION_SECONDS, [[2, 510]])


def test_a_read_waits_for_the_settle_time_after_a_write(monkeypatch):
    board = StubBoard()
    board.read_return = 500
    controller, clock = make_controller(monkeypatch, board)
    slept = []
    monkeypatch.setattr(servo_controller_module.time, "sleep", slept.append)

    controller.setPos(servoID=SERVO_ID, direction=500, pulses_per_second=PULSES_PER_SECOND)
    clock.now += 0.001
    controller.getRawPos(servoID=SERVO_ID)
    assert slept == [pytest.approx(WRITE_SETTLE_SECONDS - 0.001)]

    clock.now += 1.0
    controller.getRawPos(servoID=SERVO_ID)
    assert len(slept) == 1  # long after the write: no wait
