import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from datetime import datetime, timezone

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.robot_data.robot_val import RobotVal
from jetrover_arm_control.servo_controller.servo_sub_callback import ServoSubscriberCallback
from jetrover_arm_control.servo_controller.servo_controller import ServoController
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.board = None
        self.boardMgr = BoardManager()
        self.board = self.boardMgr.getBoard()
        
        self.get_logger().info("Initializing Servo Data Manager...")

        self.robotData = RobotVal(boardMgr = self.boardMgr)
        # One shared ServoController for both writes (via servoSubManager) and the 50Hz feedback
        # reads below -- required for ServoController's local position belief to be corrected by
        # those reads (see fix-servo-command-rate-loss / ServoController's own class doc). Two
        # separate instances (as before) would mean writes never saw the correction.
        self.servoController = ServoController(boardMgr = self.boardMgr)
        self.servoSubManager = ServoSubscriberCallback(servoController = self.servoController)

        # Subscribers
        self.get_logger().info("Initializing Servo Subscribers...")
        self.resetArmSub = self.create_subscription(Float32, "/arm/servo/reset", self.servoSubManager.resetArmCbk, 5)
        
        # depth=3, not 1: defense-in-depth against a transient executor stall (e.g. a servo's
        # one-time belief-seed read in ServoController.setPos) now that the steady-state cost
        # of each callback is near-zero -- see fix-servo-command-rate-loss. Deliberately shallow:
        # these are relative direction steps, so draining a short backlog late doesn't overshoot
        # (order-independent sum), but a deep queue would let the arm visibly keep moving on
        # stale input after the operator stops sending.
        self.setBaseSub = self.create_subscription(Float32, "/arm/servo/base", self.servoSubManager.setBaseCbk, 3)
        self.setLowerArmSub = self.create_subscription(Float32, "/arm/servo/joint/lower", self.servoSubManager.setJointLowerCbk, 3)
        self.setMiddleArmSub = self.create_subscription(Float32, "/arm/servo/joint/middle", self.servoSubManager.setJointMiddleCbk, 3)
        self.setUpperArmSub = self.create_subscription(Float32, "/arm/servo/joint/upper", self.servoSubManager.setJointUpperCbk, 3)
        self.setGripperBaseSub = self.create_subscription(Float32, "/arm/servo/gripper/base", self.servoSubManager.setGripperBaseCbk, 1)
        self.setGripperMainSub = self.create_subscription(Float32, "/arm/servo/gripper/main", self.servoSubManager.setGripperMainCbk, 1)

        self.buzzerSub = self.create_subscription(Bool, "/robot/buzzer", self.robotData.setBuzzerCbk, 1)
        self.torqueState = self.create_subscription(Int32, "/arm/servo/set/torque", self.servoSubManager.setServoTorqueCbk, 1)

        self.get_logger().info("Servo Subscribers initialized.")

        # Publishers
        self.get_logger().info("Initializing Servo Publishers.")
        self.baseName = "Base Servo"
        self.lowerArmName = "Lower Arm"
        self.middleArmName = "Middle Arm"
        self.upperArmName = "Upper Arm"
        self.gripperBaseName = "Gripper Base"
        self.gripperMainName = "Gripper Main"

        self.batteryName = "Robot Battery"
        self.tempName = "Robot Temp"
        self.voltName = "Robot Volt"
        self.torqueName = "Robot Torque"
        
        self.pollRate = 0.02
        self.stepSize = 50

        self.lastBasePos = None

        self.baseTopic = "/unity/robot/servo/base"
        self.lowerArmTopic = "/unity/robot/servo/joint/lower"
        self.middleArmTopic = "/unity/robot/servo/joint/middle"
        self.upperArmTopic = "/unity/robot/servo/joint/upper"
        self.gripperBaseTopic = "/unity/robot/servo/gripper/base"
        self.gripperMainTopic = "/unity/robot/servo/gripper/main"

        self.servoTempTopic = "/robot/servo/temperature"
        self.servoVoltTopic = "/robot/servo/voltage"
        self.servoTorqueTopic = "/robot/servo/torque"

        self.batteryTopic = "/robot/battery"

        self.basePub = self.createPublisher(self.baseName, Int32, self.baseTopic, self.pollRate, \
                                            self.basePublisher)
        # lower/middle/upper re-enabled now that the real root cause (a single-slot response
        # queue with no request/response correlation, silently dropping a fresh response
        # whenever a previous request's response had arrived late -- see
        # board_manager/robot_controller_sdk.py's bus_servo_read_and_unpack) is fixed. Polling
        # several servos concurrently at 50Hz each is no longer a problem once each read drains
        # any stale queued response before waiting for its own.
        self.lowerArmPub = self.createPublisher(self.lowerArmName, Int32, self.lowerArmTopic, self.pollRate, \
                                            self.lowerArmPublisher)
        self.middleArmPub = self.createPublisher(self.middleArmName, Int32, self.middleArmTopic, self.pollRate, \
                                            self.middleArmPublisher)
        self.upperArmPub = self.createPublisher(self.upperArmName, Int32, self.upperArmTopic, self.pollRate, \
                                            self.upperArmPublisher)
        # Gripper feedback stays disabled -- teleoperation's Phase 2 sends the gripper as an
        # absolute, open-loop command (no delta tracking needed), so it doesn't consume this.
        #self.gripperBasePub = self.createPublisher(self.gripperBaseName, Int32, self.gripperBaseTopic, self.pollRate, \
        #                                    self.gripperBasePublisher)
        #self.gripperMainPub = self.createPublisher(self.gripperMainName, Int32, self.gripperMainTopic, self.pollRate, \
        #                                    self.gripperMainPublisher)

        self.batteryPub = self.createPublisher(self.batteryName, Int32, self.batteryTopic, 5, \
                                            self.batteryPublisher)  
        #self.tempPub = self.createPublisher(self.tempName, String, self.servoTempTopic, 5, \
        #                                   self.servoTempPublisher)
        #self.voltPub = self.createPublisher(self.voltName, String, self.servoVoltTopic, 5, \
        #                                   self.servoVoltPublisher)
        #self.torquePub = self.createPublisher(self.torqueName, String, self.servoTorqueTopic, 5, \
        #                                    self.servoTorquePublisher)                                      
        self.get_logger().info("Servo Publishers initialized.")

        self.get_logger().info("Servo Data Manager initialized.")

    
    # --------------------Publishers--------------------------- #
        
    def basePublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.BASE_SERVO.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.basePub.publish(msg)

    def lowerArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.LOWER_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos


        self.lowerArmPub.publish(msg)

    def middleArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.MIDDLE_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.middleArmPub.publish(msg)

    def upperArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.UPPER_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.upperArmPub.publish(msg)

    def gripperBasePublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_BASE.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.gripperBasePub.publish(msg)

    def gripperMainPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_MAIN.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.gripperMainPub.publish(msg)

    def batteryPublisher(self):
        data = self.robotData.getBattery()
        if data is None:
            return
        msg = Int32()
        msg.data = data

        self.batteryPub.publish(msg)

    def servoTempPublisher(self):
        data = self.servoController.getServoTemp()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.tempPub.publish(msg)

    def servoVoltPublisher(self):
        data = self.servoController.getServoVolt()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.voltPub.publish(msg)

    def servoTorquePublisher(self):
        data = self.servoController.getServoTorque()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.torquePub.publish(msg)

    def createPublisher(self, publisherName, msgType, msgTopic, pollRate: int, clbFunc, queueSize=1):
        dataPub = self.create_publisher(msgType, msgTopic, queueSize)
        self.get_logger().info(f"Publisher - {publisherName} Created")
        self.timer = self.create_timer(pollRate, clbFunc) 

        return dataPub

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
    
