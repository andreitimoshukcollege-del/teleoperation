import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.robot_data.robot_val import RobotVal
from jetrover_arm_control.servo_controller.servo_sub_callback import ServoSubscriberCallback
from jetrover_arm_control.servo_controller.servo_controller import ServoController

# Which motor ids to poll for feedback -- Jetson-local, deliberately separate from the
# operator-side RobotArmProfile (docs/adr/0011-generic-robot-arm-profiles.md): this node no
# longer needs to know joint *roles*, only which raw motor ids exist on this bus to read back.
# Matches JetRover's four arm joints (base/lower/middle/upper, motor ids 1-4); gripper feedback
# stays disabled, same as before this generalization -- GenericArmPlant sends the gripper
# open-loop, with no belief tracking that would need it.
# Motor 10 (the gripper) is not read: its servo never answers position reads, and every read
# waits for its timeout, which held each feedback cycle (and every command behind it, on this one
# thread) for a second. The headset models the gripper from its commands (docs/adr/0018).
FEEDBACK_MOTOR_IDS = [1, 2, 3, 4, 5]

# How often the node logs its feedback-loop statistics and the battery, seconds. When the board
# stops answering (robot/README.md), the journal then shows when the reads started failing and the
# voltage just before.
STATS_PERIOD_SECONDS = 5.0


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.board = None
        self.boardMgr = BoardManager()
        self.board = self.boardMgr.getBoard()

        self.get_logger().info("Initializing Servo Data Manager...")

        self.robotData = RobotVal(boardMgr = self.boardMgr)
        # One shared ServoController for both writes (via servoSubManager) and the feedback-poll
        # timer below -- both need the same underlying board connection.
        self.servoController = ServoController(boardMgr = self.boardMgr)
        self.servoSubManager = ServoSubscriberCallback(servoController = self.servoController)
        self._resetStats(time.monotonic())
        self._lastCycleEnd = None
        # The board's last battery report, mV, for the stats line: when the board goes silent, the
        # journal then shows the voltage just before it did.
        self._lastBatteryMv = None

        # Subscribers
        self.get_logger().info("Initializing Servo Subscribers...")
        self.resetArmSub = self.create_subscription(Float32, "/arm/servo/reset", self.servoSubManager.resetArmCbk, 5)

        # A single generic command topic (docs/adr/0011-generic-robot-arm-profiles.md) replaces
        # the old per-joint topics (base/lower/middle/upper/gripper-base/gripper-main) -- one
        # subscription instead of six, motor-id-keyed rather than joint-name-keyed. depth=3, not
        # 1: defense-in-depth against a transient executor stall -- setPos computes its own move
        # duration from distance and the supplied speed, so a shallow backlog being drained late
        # just replays a couple of recent-but-slightly-stale absolute targets in order, each
        # independently valid. A deep queue would still let the arm visibly keep moving on stale
        # input after the operator stops sending, hence keeping this shallow rather than
        # removing the limit.
        self.armCommandSub = self.create_subscription(
            Float32MultiArray, "/arm/servo/command_array", self._timedCommandCbk, 3)

        self.buzzerSub = self.create_subscription(Bool, "/robot/buzzer", self.robotData.setBuzzerCbk, 1)
        self.torqueState = self.create_subscription(Int32, "/arm/servo/set/torque", self.servoSubManager.setServoTorqueCbk, 1)

        self.get_logger().info("Servo Subscribers initialized.")

        # Publishers
        self.get_logger().info("Initializing Servo Publishers.")

        self.pollRate = 0.02

        # A single generic feedback topic replaces the old per-joint publishers (basePublisher,
        # lowerArmPublisher, middleArmPublisher, upperArmPublisher) -- see feedbackPublisher's
        # own doc.
        self.feedbackPub = self.create_publisher(Float32MultiArray, "/unity/robot/servo/feedback_array", 1)
        self.create_timer(self.pollRate, self.feedbackPublisher)

        self.batteryPub = self.create_publisher(Int32, "/robot/battery", 1)
        self.create_timer(5, self.batteryPublisher)

        self.get_logger().info("Servo Publishers initialized.")

        self.get_logger().info("Servo Data Manager initialized.")

    # --------------------Statistics--------------------------- #

    def _resetStats(self, now):
        self._statsSince = now
        self._cycles = 0
        self._readOk = {m: 0 for m in FEEDBACK_MOTOR_IDS}
        self._readMissed = {m: 0 for m in FEEDBACK_MOTOR_IDS}
        self._slowestReadMs = {m: 0.0 for m in FEEDBACK_MOTOR_IDS}
        self._slowestCycleMs = 0.0
        self._longestGapMs = 0.0
        self._commands = 0
        self._slowestCommandMs = 0.0
        self._packets = 0

    def _timedCommandCbk(self, msg):
        start = time.monotonic()
        self._packets += self.servoSubManager.armCommandArrayCbk(msg) or 0
        self._commands += 1
        self._slowestCommandMs = max(self._slowestCommandMs, (time.monotonic() - start) * 1000.0)

    def _logStats(self, now):
        if now - self._statsSince < STATS_PERIOD_SECONDS:
            return
        seconds = now - self._statsSince
        reads = " ".join(
            f"{m}:{self._readOk[m]}/{self._readMissed[m]}/{self._slowestReadMs[m]:.0f}ms" for m in FEEDBACK_MOTOR_IDS)
        self.get_logger().info(
            f"[stats {seconds:.1f}s] cycles {self._cycles} ({self._cycles / seconds:.1f} Hz), slowest cycle "
            f"{self._slowestCycleMs:.0f} ms, longest gap between cycles {self._longestGapMs:.0f} ms; "
            f"reads ok/missed/slowest {reads}; commands {self._commands} (slowest {self._slowestCommandMs:.1f} ms); "
            f"move packets {self._packets}; battery {self._lastBatteryMv if self._lastBatteryMv is not None else '?'} mV")
        self._resetStats(now)

    # --------------------Publishers--------------------------- #

    def feedbackPublisher(self):
        """Generic replacement for the old per-joint publisher methods (basePublisher,
        lowerArmPublisher, middleArmPublisher, upperArmPublisher) -- polls every id in
        FEEDBACK_MOTOR_IDS in one loop and publishes raw pulse directly (no pulseToDeg
        conversion: docs/adr/0011-generic-robot-arm-profiles.md moves feedback to pulse units
        uniformly, since bus_servo_read_position already returns raw pulse). A motor whose read
        fails this cycle is simply omitted, not published as a fabricated zero -- relay_node.py's
        own StaleTrackedValue is what turns "missing" into "invalid," not this method.
        """
        cycleStart = time.monotonic()
        self._logStats(cycleStart)
        if self._lastCycleEnd is not None:
            self._longestGapMs = max(self._longestGapMs, (cycleStart - self._lastCycleEnd) * 1000.0)
        readings = []
        for motor_id in FEEDBACK_MOTOR_IDS:
            # A command callback cannot run during this poll (single-threaded executor), so write
            # any target whose servo's cooldown has just ended here, between reads. getRawPos
            # then waits a few ms after the packet before it asks.
            self._packets += self.servoController.flushPending()
            readStart = time.monotonic()
            pulse = self.servoController.getRawPos(motor_id)
            self._slowestReadMs[motor_id] = max(self._slowestReadMs[motor_id], (time.monotonic() - readStart) * 1000.0)
            if pulse is None:
                self._readMissed[motor_id] += 1
                continue
            self._readOk[motor_id] += 1
            readings.append((motor_id, pulse))

        self._cycles += 1
        self._lastCycleEnd = time.monotonic()
        self._slowestCycleMs = max(self._slowestCycleMs, (self._lastCycleEnd - cycleStart) * 1000.0)

        msg = Float32MultiArray()
        data = [float(len(readings))]
        for motor_id, pulse in readings:
            data.extend([float(motor_id), float(pulse)])
        msg.data = data
        self.feedbackPub.publish(msg)

    def batteryPublisher(self):
        data = self.robotData.getBattery()
        if data is None:
            return
        self._lastBatteryMv = data
        msg = Int32()
        msg.data = data

        self.batteryPub.publish(msg)


def main():
    rclpy.init()
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Node interrupted by user keyboard")
    finally:
        robot_controller.get_logger().info("Shutting Down ROS Subscriber Node")
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
