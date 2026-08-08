

from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM
from jetrover_arm_control.servo_controller.servo_controller import ServoController

class ServoSubscriberCallback:

    def __init__(self, servoController: ServoController):
        # Takes an injected, shared ServoController rather than constructing its own -- the
        # 50Hz feedback-publisher timers in robot_controller_node.py read through the SAME
        # instance, which is what lets ServoController's local position belief (see its own
        # class doc) get opportunistically corrected by those reads at no extra bus cost.
        # A separate instance here would mean writes never see those corrections.
        self.servoSpeed = 0.3
        self.servoController = servoController

    
    # --------------------Subscribers--------------------------- #
    
    def resetArmCbk(self, msg):
        print("Reset Triggered")
        self.servoController.resetArm() 

    def setBaseCbk(self, msg):
        
        data = msg.data
        self.servoController.setPos(servoID= SERVO_ENUM.BASE_SERVO.value, direction=data, servoSpeed=self.servoSpeed)

    def setJointLowerCbk(self, msg):
        data = msg.data
        self.servoController.setPos(servoID= SERVO_ENUM.LOWER_ARM.value, direction=data, servoSpeed=self.servoSpeed)

    def setJointMiddleCbk(self, msg):
        data = msg.data
        self.servoController.setPos(servoID= SERVO_ENUM.MIDDLE_ARM.value, direction=data, servoSpeed=self.servoSpeed)

    def setJointUpperCbk(self, msg):
        data = msg.data
        self.servoController.setPos(servoID= SERVO_ENUM.UPPER_ARM.value, direction=data, servoSpeed=self.servoSpeed)

    def setGripperBaseCbk(self, msg):
        data = msg.data
        self.servoController.setPos(servoID= SERVO_ENUM.GRIPPER_BASE.value, direction=data, servoSpeed=self.servoSpeed)

    def setGripperMainCbk(self, msg):
        data = msg.data
        self.servoController.setGripperPos(servoID= SERVO_ENUM.GRIPPER_MAIN.value, direction=data, servoSpeed=self.servoSpeed)

    def setServoSpeed(self, msg):
        data = msg.data
        self.servoSpeed = data

    def setServoTorqueCbk(self, msg):
        data = msg.data
        self.servoController.setTorqueState(data)


    # # --------------------Publishers--------------------------- #

    # def basePublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.BASE_SERVO.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos
    #     # print(msg.data)

    
    #     self.basePub.publish(msg)

    # def lowerArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.LOWER_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.lowerArmPub.publish(msg)

    # def middleArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.MIDDLE_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.middleArmPub.publish(msg)

    # def upperArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.UPPER_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.upperArmPub.publish(msg)

    # def gripperBasePublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.GRIPPER_BASE.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos
        
        

    #     self.gripperBasePub.publish(msg)

    # def gripperMainPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.GRIPPER_MAIN.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos


    #     self.gripperMainPub.publish(msg)