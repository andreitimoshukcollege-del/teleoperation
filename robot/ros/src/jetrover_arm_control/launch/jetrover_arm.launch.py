from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_sdk = Node(
        package="jetrover_arm_control",
        executable="robot_controller_manager",
        name="robot_controller",
    )

    return LaunchDescription([robot_sdk])
