"""Unit tests for ServoController's local position-belief tracking.

Mirrors teleoperation's JetRoverPlantTests.cs in spirit: a fake/stub hardware boundary (here, a
stub board object) so the belief-tracking logic itself -- the actual fix for
fix-servo-command-rate-loss -- is proven correct without any real serial hardware, ROS node, or
sleep. See ServoController's own class doc for the full rationale.
"""

from jetrover_arm_control.servo_controller import servo_controller as servo_controller_module
from jetrover_arm_control.servo_controller.servo_controller import ServoController

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


def test_first_setpos_reads_once_then_never_again(monkeypatch):
    """The seed read happens exactly once per servoID; later calls use belief, not hardware."""
    board = StubBoard()
    board.read_return = 500
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.1)
    assert board.read_call_count == 1

    clock.now += 1.0  # clear of the cooldown
    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.1)

    # Two real moves happened (550 then 600), but only the very first setPos call should ever
    # have needed a real read.
    assert board.read_call_count == 1
    assert board.written == [
        (0.1, [[SERVO_ID, 550]]),
        (0.1, [[SERVO_ID, 600]]),
    ]


def test_belief_arithmetic_matches_expected_clamped_sums(monkeypatch):
    """A sequence of directions accumulates on belief exactly like the old read-every-time code
    would have, including clamping to [0, 1000]. (Starting at 900, not e.g. 980: the write-gate
    `abs(nextPos - currentPos) >= stepSize` can suppress a move entirely when the unclamped
    target overshoots the boundary by less than stepSize -- a pre-existing quirk of this gate,
    unrelated to this fix, that a boundary-hugging start value would otherwise trip over.)"""
    board = StubBoard()
    board.read_return = 900
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.01)  # 900 -> 950
    clock.now += 1.0
    controller.setPos(servoID=SERVO_ID, direction=2, servoSpeed=0.01)  # 950 -> 1050 -> 1000 (clamped)

    assert board.written == [
        (0.01, [[SERVO_ID, 950]]),
        (0.01, [[SERVO_ID, 1000]]),
    ]
    assert controller._beliefPos[SERVO_ID] == 1000


def test_cooldown_suppresses_a_second_setpos_within_the_window(monkeypatch):
    """Calling setPos again before the servo's cooldown expires is a no-op: no read, no write,
    belief unchanged -- the safe coalesce fix-servo-command-rate-loss relies on."""
    board = StubBoard()
    board.read_return = 500
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.3)
    assert board.written == [(0.3, [[SERVO_ID, 550]])]

    clock.now += 0.1  # still well inside the 0.3s cooldown
    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.3)

    assert board.written == [(0.3, [[SERVO_ID, 550]])]  # no second write
    assert controller._beliefPos[SERVO_ID] == 550


def test_a_successful_real_read_overwrites_belief_even_mid_cooldown(monkeypatch):
    """Real feedback always wins over the optimistic belief, at any time -- the opportunistic
    correction the 50Hz feedback-publisher timers rely on."""
    board = StubBoard()
    board.read_return = 500
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.3)
    assert controller._beliefPos[SERVO_ID] == 550

    board.read_return = 700  # what a real feedback-publisher read would now observe
    result = controller.getRawPos(servoID=SERVO_ID)

    assert result == 700
    assert controller._beliefPos[SERVO_ID] == 700


def test_fallback_seed_is_used_and_persisted_when_the_initial_read_fails(monkeypatch):
    """A servo whose reads never succeed (the real, observed middle-arm case) must still be
    writable: the fallback seed lets it, and the seed is only ever consulted once."""
    board = StubBoard()
    board.read_return = None  # every read fails, matching the middle-arm servo
    controller, clock = make_controller(monkeypatch, board)

    controller.setPos(servoID=SERVO_ID, direction=0, servoSpeed=0.1)  # direction=0: no real move

    assert board.written == []  # no move commanded, so nothing written
    assert controller._beliefPos[SERVO_ID] == 500  # but the fallback seed was still recorded
    assert board.read_call_count == 1

    # A second call, even immediately, must not attempt another failing read -- belief already
    # holds the fallback seed.
    controller.setPos(servoID=SERVO_ID, direction=1, servoSpeed=0.1)
    assert board.written == [(0.1, [[SERVO_ID, 550]])]
    assert board.read_call_count == 1
