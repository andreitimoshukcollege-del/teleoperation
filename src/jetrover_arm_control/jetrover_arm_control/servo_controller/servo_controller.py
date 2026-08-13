import time
from time import sleep

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM

# setPos's write-gate threshold, in pulse units. Originally existed to filter per-call
# floating-point truncation noise from JetRoverPlant's belief *convergence* toward a target
# (2026-08-08) -- a real correctness concern back when this side reconstructed an absolute
# position from a relative delta accumulated onto its own belief, since repeated truncation could
# compound into real drift over many calls. Since teleoperation's
# docs/adr/0010-absolute-joint-targets-over-local-relay.md (wire v3), setPos receives the
# already-resolved absolute pulse target directly every time -- nothing here accumulates, so
# there is nothing for truncation noise to drift into. This gate is now a bus-traffic
# optimization only (skip a physical write that wouldn't move the servo perceptibly), not a
# correctness requirement -- kept at the same threshold since it's still a reasonable one for
# that purpose.
MIN_MEANINGFUL_PULSE_DELTA = 5

# setPos's move speed, in pulses/second, and the minimum move duration regardless of distance.
# Found 2026-08-13: a fixed move duration (this file's own history had it hardcoded to 0.3s)
# means a bigger move must travel *faster* to finish in the same window than a smaller one --
# inconsistent, and visible on real hardware as the arm moving in discrete "step, pause, step,
# pause" jumps once a move needed more than one command's worth of distance, since each
# fixed-duration step finished early and sat idle waiting for the next command. A constant speed
# instead means duration scales with distance: a bigger move just takes proportionally longer, at
# the same physical rate, so a move spanning multiple commands (see JetRoverPlantConfig's
# MaxDirectionMagnitude, which bounds how much distance one command can cover) is one continuous
# motion at a steady rate instead of separately-timed chunks. No maximum duration is applied --
# only a minimum, to avoid an implausibly fast/jerky instantaneous move for a near-zero delta.
PULSES_PER_SECOND = 300
MIN_MOVE_DURATION_SECONDS = 0.05


class ServoController:
    """setPos (wire v3, docs/adr/0010-absolute-joint-targets-over-local-relay.md, plus the
    constant-speed follow-up above) receives an already-resolved absolute pulse target per call
    and writes it directly -- no belief, no relative-delta arithmetic, no seed-from-read fallback.
    `Teleop.RobotHost`'s `JetRoverPlant` is the one place that tracks "where do we currently
    believe this joint is" now; this class only needs a plain per-servo cache of the last pulse it
    actually *sent* (self._lastSentPulse) -- not a belief combining any assumption about where the
    servo physically is -- for two purposes: MIN_MEANINGFUL_PULSE_DELTA can skip a physical write
    that wouldn't move the servo perceptibly, and PULSES_PER_SECOND can size this specific move's
    duration from how far it actually needs to travel.

    The per-servo write cooldown (self._nextAllowedWriteAt) is unrelated to any of that: it is
    real hardware pacing (the STM32 firmware paces the physical move itself via the commanded
    duration, so a new command arriving before the previous one's move has finished is coalesced
    rather than queued) and predates this class's belief-tracking history entirely.
    """

    def __init__(self, boardMgr: BoardManager):

        self.boardMgr = boardMgr
        self.board = self.getBoard()

        self.pwm_min = 0
        self.pwm_max = 1000
        self.deg_min = 0
        self.deg_max = 180

        # See class doc: the last pulse actually sent per servoID (used for the write gate and
        # for sizing this move's speed-based duration, not a position belief), and the monotonic
        # timestamp (time.monotonic()) after which the next real write for that servo is due.
        self._lastSentPulse = {}
        self._nextAllowedWriteAt = {}

        # self.servoList = [SERVO_ENUM.BASE_SERVO.value, SERVO_ENUM.LOWER_ARM.value, \
        #                   SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

        self.servoList = [SERVO_ENUM.BASE_SERVO.value, \
                          SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

    def getPos(self, servoID: int):
        currentPos = self.getRawPos(servoID=servoID)
        deg = self.pulseToDeg(currentPos)

        return deg
        
    def setPos(self, servoID: int, direction: int):
        """Moves the servo directly to an absolute pulse target, at a constant speed
        (PULSES_PER_SECOND) -- the move duration is computed from how far this specific servo
        actually needs to travel, not passed in by the caller (see PULSES_PER_SECOND's own
        comment for why).

        Arguments:
            servoID (int) - ID for the servo
            direction (float) - absolute pulse target (0-1000), despite the name kept from the
                pre-wire-v3 relative-delta protocol -- servo_sub_callback.py's ROS callbacks pass
                the raw Float32 message value straight through unchanged, and that value's
                *meaning* is what changed (docs/adr/0010-absolute-joint-targets-over-local-relay.md
                on the teleoperation side); renaming this parameter is out of scope for this
                change alone since it's also this method's public argument name elsewhere.
        """
        now = time.monotonic()
        if now < self._nextAllowedWriteAt.get(servoID, 0.0):
            # Still cooling down from this servo's last real move -- coalesces a burst of
            # commands into one physical move instead of blocking the ROS executor on a
            # redundant serial round trip for each one. See class doc.
            return

        nextPos = int(direction)
        nextPos = 0 if nextPos < 0 else 1000 if nextPos > 1000 else nextPos

        lastSent = self._lastSentPulse.get(servoID)
        if lastSent is not None and abs(nextPos - lastSent) < MIN_MEANINGFUL_PULSE_DELTA:
            # Not a real correctness concern anymore (see MIN_MEANINGFUL_PULSE_DELTA's own
            # comment) -- just skips a physical write that wouldn't move the servo perceptibly.
            return

        # No prior sent value for this servo (its very first move) -- there is no real distance
        # to measure a speed against yet, so this one call falls back to the minimum duration
        # rather than an undefined or zero-distance one.
        pulseDelta = abs(nextPos - lastSent) if lastSent is not None else 0
        duration = max(MIN_MOVE_DURATION_SECONDS, pulseDelta / PULSES_PER_SECOND)

        self.board.bus_servo_set_position(duration, [[servoID, nextPos]])

        # The STM32 firmware paces the physical move itself via duration (the parameter
        # bus_servo_set_position encodes into the packet) -- no need to block this thread waiting
        # for it. Track when the next write may go out instead.
        self._lastSentPulse[servoID] = nextPos
        self._nextAllowedWriteAt[servoID] = now + duration

    def setGripperPos(self, servoID: int, direction: int, servoSpeed: float = 0.5):
        pulse_pos = self.degToPulse(direction)
        # print(pulse_pos, direction)
        self.board.bus_servo_set_position(servoSpeed, [[servoID, pulse_pos]])

    def resetArm(self):
        
        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

    def getRawPos(self, servoID: int):
        if(servoID):
            readPos = self.board.bus_servo_read_position(servo_id= servoID)
            if(readPos):
                # setPos (wire v3) no longer tracks a position belief for this to correct -- see
                # class doc. This read is purely for robot_controller_node.py's own 50Hz
                # feedback-publisher timers now.
                return readPos[0]

            # return self.board.bus_servo_read_position(servo_id= servoID)[0]
        else:
            return -1

    def degToPulse(self, data: float):
        val = ( data / self.deg_max ) * self.pwm_max
        return int(val)

    def pulseToDeg(self, data: int):
        val = (data / self.pwm_max) * self.deg_max
        return int(val)

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
