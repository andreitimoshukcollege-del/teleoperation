import time
from time import sleep

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager

# setPos's write-gate threshold, in pulse units. A bus-traffic optimization only (skip a physical
# write that wouldn't move the servo perceptibly) -- see docs/adr/0010-absolute-joint-targets-over-local-relay.md
# for why this stopped being a correctness requirement once setPos began receiving already-resolved
# absolute pulse targets instead of relative deltas.
MIN_MEANINGFUL_PULSE_DELTA = 5

# Minimum move duration regardless of distance, seconds -- avoids an implausibly fast/jerky
# instantaneous move for a near-zero delta. The move *speed* itself (pulses/second) is no longer a
# module constant here: docs/adr/0011-generic-robot-arm-profiles.md moves it to an explicit
# per-call parameter (setPos's own pulses_per_second), supplied by GenericArmPlant over the wire,
# so the .NET side and this side no longer have to keep a duplicated PULSES_PER_SECOND number in
# sync by hand.
MIN_MOVE_DURATION_SECONDS = 0.05


class ServoController:
    """setPos (wire v4, docs/adr/0011-generic-robot-arm-profiles.md) receives an already-resolved
    absolute pulse target and an explicit move speed per call and writes it directly -- no belief,
    no relative-delta arithmetic, no seed-from-read fallback, no locally hardcoded speed constant.
    `Teleop.RobotHost`'s `GenericArmPlant` is the one place that tracks "where do we currently
    believe this joint is" now; this class only needs a plain per-servo cache of the last pulse it
    actually *sent* (self._lastSentPulse) -- not a belief combining any assumption about where the
    servo physically is -- so `MIN_MEANINGFUL_PULSE_DELTA` can skip a physical write that wouldn't
    move the servo perceptibly.

    The gripper is no longer a special case: it used to have its own `setGripperPos` method
    (absolute *degrees*, a different unit space and speed convention from every other joint).
    Since the wire and the .NET-side plant now treat the gripper as just another joint in pulse
    space, `setGripperPos` is gone -- `servo_sub_callback.py` calls `setPos` for it exactly like
    every other joint.

    The per-servo write cooldown (self._nextAllowedWriteAt) is unrelated to any of that: it is
    real hardware pacing (the STM32 firmware paces the physical move itself via the commanded
    duration, so a new command arriving before the previous one's move has finished is coalesced
    rather than queued) and predates this class's belief-tracking history entirely.

    Coalesced means the newest target wins (2026-10-20). A command that arrives during the
    cooldown is kept as that servo's pending target, replacing any older one, and
    `flushPending()` writes it once the cooldown has expired. Before, it was dropped. On the
    single-threaded executor, commands are handled in bursts between feedback polls, so the
    first command of each burst, the oldest, was the one written; the target the servo chased
    was up to a poll cycle stale, and a final correction inside the cooldown was lost
    (robot/README.md has the earlier incident).
    """

    def __init__(self, boardMgr: BoardManager):

        self.boardMgr = boardMgr
        self.board = self.getBoard()

        self.pwm_min = 0
        self.pwm_max = 1000

        # See class doc: the last pulse actually sent per servoID (used only for the write gate
        # now, not a position belief), and the monotonic timestamp (time.monotonic()) after which
        # the next real write for that servo is due.
        self._lastSentPulse = {}
        self._nextAllowedWriteAt = {}
        # The newest target that arrived during a servo's cooldown: servoID -> (pulse, speed).
        self._pending = {}

        # Hardcoded motor ids for this JetRover's bulk diagnostic reads (temp/volt/torque) only --
        # servo_id_enum.SERVO_ENUM is gone (docs/adr/0011: motor ids now arrive directly on the
        # wire for position control, so nothing constructs a command by looking up a name
        # anymore). These four numbers are what SERVO_ENUM.BASE_SERVO/MIDDLE_ARM/UPPER_ARM/
        # GRIPPER_BASE/GRIPPER_MAIN used to resolve to.
        self.servoList = [1, 3, 4, 5, 10]

    def getPos(self, servoID: int):
        return self.getRawPos(servoID=servoID)

    def setPos(self, servoID: int, direction: int, pulses_per_second: float):
        """Moves the servo directly to an absolute pulse target, at the given constant speed --
        the move duration is computed from how far this specific servo actually needs to travel
        and the caller-supplied speed (docs/adr/0011-generic-robot-arm-profiles.md; previously a
        module constant, see MIN_MOVE_DURATION_SECONDS's own comment for why it moved).

        Arguments:
            servoID (int) - id for the servo
            direction (float) - absolute pulse target (0-1000), despite the name kept from the
                pre-wire-v3 relative-delta protocol
            pulses_per_second (float) - this move's speed; must be > 0
        """
        now = time.monotonic()
        nextPos = int(direction)
        nextPos = 0 if nextPos < 0 else 1000 if nextPos > 1000 else nextPos
        if now < self._nextAllowedWriteAt.get(servoID, 0.0):
            # Still cooling down from this servo's last real move: keep it as the pending target,
            # newest wins, for flushPending to write when the cooldown ends. See class doc.
            self._pending[servoID] = (nextPos, pulses_per_second)
            return

        self._pending.pop(servoID, None)
        self._write(servoID, nextPos, pulses_per_second, now)

    def flushPending(self):
        """Writes every pending target whose servo's cooldown has expired. Cheap when nothing is
        pending; robot_controller_node.py calls it between feedback reads, since on the
        single-threaded executor no command callback runs while the feedback poll does."""
        if not self._pending:
            return

        now = time.monotonic()
        for servoID in list(self._pending):
            if now < self._nextAllowedWriteAt.get(servoID, 0.0):
                continue
            nextPos, pulses_per_second = self._pending.pop(servoID)
            self._write(servoID, nextPos, pulses_per_second, now)

    def _write(self, servoID: int, nextPos: int, pulses_per_second: float, now: float):
        """One physical move, unless it is within the write gate of what was last sent."""
        lastSent = self._lastSentPulse.get(servoID)
        if lastSent is not None and abs(nextPos - lastSent) < MIN_MEANINGFUL_PULSE_DELTA:
            # Not a correctness concern (see MIN_MEANINGFUL_PULSE_DELTA's own comment) -- just
            # skips a physical write that wouldn't move the servo perceptibly.
            return

        # No prior sent value for this servo (its very first move) -- there is no real distance
        # to measure a speed against yet, so this one call falls back to the minimum duration
        # rather than an undefined or zero-distance one.
        pulseDelta = abs(nextPos - lastSent) if lastSent is not None else 0
        duration = max(MIN_MOVE_DURATION_SECONDS, pulseDelta / pulses_per_second)

        self.board.bus_servo_set_position(duration, [[servoID, nextPos]])

        # The STM32 firmware paces the physical move itself via duration (the parameter
        # bus_servo_set_position encodes into the packet) -- no need to block this thread waiting
        # for it. Track when the next write may go out instead.
        self._lastSentPulse[servoID] = nextPos
        self._nextAllowedWriteAt[servoID] = now + duration

    def resetArm(self):
        # A reset overrides whatever was still waiting to be written.
        self._pending.clear()

        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

    def getRawPos(self, servoID: int):
        if(servoID):
            readPos = self.board.bus_servo_read_position(servo_id= servoID)
            if(readPos):
                # setPos no longer tracks a position belief for this to correct -- see class doc.
                # This read is purely for robot_controller_node.py's own feedback-polling loop now,
                # returned as raw pulse directly (no pulseToDeg conversion -- docs/adr/0011 moves
                # feedback to pulse units uniformly, since this read is already raw pulse).
                return readPos[0]

        else:
            return -1

    def getServoTemp(self):
        temps= ""
        for servoID in self.servoList:
            temp = self.board.bus_servo_read_temp(servoID)[0]
            temps = str(temps)+","+ str(temp)

        return temps

    def getServoVolt(self):
        volts= ""
        for servoID in self.servoList:
            volt = self.board.bus_servo_read_vin(servoID)[0]
            volts = str(volts)+","+ str(volt)

        return volts

    def getServoTorque(self):
        torques= ""
        for servoID in self.servoList:
            torque = self.board.bus_servo_read_torque_state(servoID)[0]
            torques = str(torques)+","+ str(torque)

        return torques

    def setTorqueState(self, state: int):

        for servoID in self.servoList:
            if(state == 1):
                self.board.bus_servo_enable_torque(servoID, 1)
            else:
                self.board.bus_servo_enable_torque(servoID, 0)


    def getBoard(self):
        return self.boardMgr.getBoard()
