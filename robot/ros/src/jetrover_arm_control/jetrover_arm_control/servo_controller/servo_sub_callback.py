class ServoSubscriberCallback:

    def __init__(self, servoController):
        # Takes an injected, shared ServoController rather than constructing its own -- the
        # feedback-polling timer in robot_controller_node.py reads through the SAME instance.
        self.servoController = servoController

    # --------------------Subscribers--------------------------- #

    def resetArmCbk(self, msg):
        print("Reset Triggered")
        self.servoController.resetArm()

    def armCommandArrayCbk(self, msg):
        """Generic replacement for the old per-joint callbacks (setBaseCbk, setJointLowerCbk,
        setJointMiddleCbk, setJointUpperCbk, setGripperBaseCbk, setGripperMainCbk) --
        docs/adr/0011-generic-robot-arm-profiles.md. `msg.data` is a plain
        std_msgs/Float32MultiArray laid out as [count, motor_id_0, pulse_0, speed_0, motor_id_1,
        ...] (see relay_node.py's own encode/decode helpers for the matching shape). The gripper
        is no longer special-cased -- it arrives as just another entry in this same array and
        flows through the identical `setPos` call as every other joint, since `setGripperPos`
        (a separate absolute-degrees code path) is gone.
        """
        data = msg.data
        if len(data) < 1:
            return 0

        count = int(data[0])
        offset = 1
        targets = []
        for _ in range(count):
            if offset + 2 >= len(data):
                break
            targets.append((int(data[offset]), data[offset + 1], data[offset + 2]))
            offset += 3
        # One board packet for every servo that may move now, rather than one each.
        return self.servoController.setPositions(targets)

    def setServoTorqueCbk(self, msg):
        data = msg.data
        self.servoController.setTorqueState(data)
