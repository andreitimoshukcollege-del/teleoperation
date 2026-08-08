import time
from time import sleep

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM

class ServoController:
    """setPos tracks a local, optimistic belief of each servo's position (keyed by servoID,
    self._beliefPos) instead of re-reading real hardware before every write. A blocking serial
    read (up to a 1s timeout, see board_manager/robot_controller_sdk.py) inside every single
    write was the actual cause of real commands being silently lost at teleop rate: the ROS
    executor is single-threaded, so a command arriving while a previous one was still blocked in
    that read would overwrite the queued-but-unprocessed message under depth-1 QoS before its
    callback ever ran (see the fix-servo-command-rate-loss branch/PR for the full diagnosis).

    Belief is corrected opportunistically whenever a real read *does* succeed (getRawPos, called
    independently by robot_controller_node.py's 50Hz feedback-publisher timers) -- real feedback
    always wins over the optimistic value, mirroring the .NET side of this same pipeline
    (core/Teleop.RobotHost/Plant/JetRoverPlant.cs's sensed-over-commanded preference). This only
    works because robot_controller_node.py constructs one ServoController and shares it between
    the write path and those feedback timers -- a second, separate instance would never see the
    correction.
    """

    def __init__(self, boardMgr: BoardManager):

        self.boardMgr = boardMgr
        self.board = self.getBoard()

        self.pwm_min = 0
        self.pwm_max = 1000
        self.deg_min = 0
        self.deg_max = 180

        # See class doc: optimistic write-side belief, keyed by servoID, and the monotonic
        # timestamp (time.monotonic()) after which the next real write for that servo is due.
        self._beliefPos = {}
        self._nextAllowedWriteAt = {}

        # self.servoList = [SERVO_ENUM.BASE_SERVO.value, SERVO_ENUM.LOWER_ARM.value, \
        #                   SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

        self.servoList = [SERVO_ENUM.BASE_SERVO.value, \
                          SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

    def getPos(self, servoID: int):
        currentPos = self.getRawPos(servoID=servoID)
        deg = self.pulseToDeg(currentPos)

        return deg
        
    def setPos(self, servoID:int, direction: int, servoSpeed: float = 0.02):
        """This class sets the position of the servo motor.
            Arguments:
                servoID (int) - ID for the servo
                pos (int) - position in PWM
                servoSpeed (int) - servo speed; DEFAULT = 1
        """
        now = time.monotonic()
        if now < self._nextAllowedWriteAt.get(servoID, 0.0):
            # Still cooling down from this servo's last real move. Belief already reflects
            # that write, so there is nothing new to compute -- returning here coalesces a
            # burst of same-direction commands into one physical move instead of blocking the
            # ROS executor on a redundant serial round trip for each one. See class doc.
            return

        # Fetch the current position: belief if we have one, else a real read to seed it.
        currentPos = self._beliefPos.get(servoID)
        if currentPos is None:
            currentPos = self.getRawPos(servoID=servoID)
            if currentPos is None:
                # No belief yet and the seed read failed -- a real, observed case (the
                # middle-arm servo's reads never succeed at all). Falling back to center
                # rather than giving up lets this servo still receive writes, unlike before.
                currentPos = 500
            self._beliefPos[servoID] = currentPos
        stepSize = 50

        # Calculate the new target position based on direction
        nextPos = int(currentPos + (stepSize * direction))

        # print("currentPos:", currentPos, "Direction:", direction, "Target Position:", nextPos)

        nextPos = 0 if nextPos < 0 else 1000 if nextPos > 1000 else nextPos

        # if toPos > 600:
        #     toPos = 600
        # elif toPos < 400:
        #     toPos = 400

        if abs(nextPos - currentPos) >= stepSize:
            # Set servo position only if the next position is within limits
            self.board.bus_servo_set_position(servoSpeed, [[servoID, nextPos]])

            # The STM32 firmware paces the physical move itself via servoSpeed (the `duration`
            # parameter bus_servo_set_position encodes into the packet) -- no need to block
            # this thread waiting for it. Track when the next write may go out instead.
            self._beliefPos[servoID] = nextPos
            self._nextAllowedWriteAt[servoID] = now + servoSpeed

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
                # Real feedback always overwrites the optimistic belief setPos tracks -- see
                # class doc. Whether this call came from setPos's own seed read or from
                # robot_controller_node.py's independent 50Hz feedback timers, the effect is
                # the same: the write path's next command starts from ground truth.
                self._beliefPos[servoID] = readPos[0]
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
