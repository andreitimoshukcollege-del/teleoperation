from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Starts jetrover_arm_control's board/servo node and teleop_relay's bridge node together --
    the full ROS-side chain Teleop.RobotHost talks to (docs/adr/0007-jetrover-plant-and-robot-host.md
    in the teleoperation repo).
    """
    robot_sdk = Node(
        package="jetrover_arm_control",
        executable="robot_controller_manager",
        name="robot_controller",
    )

    relay = Node(
        package="teleop_relay",
        executable="relay_node",
        name="teleop_relay",
    )

    return LaunchDescription([robot_sdk, relay])
