import os
from glob import glob
from setuptools import setup

package_name = 'jetrover_arm_control'
board = 'jetrover_arm_control/board_manager'
servo_controller = 'jetrover_arm_control/servo_controller'
robot_data = 'jetrover_arm_control/robot_data'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, board, servo_controller, robot_data],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description="ROS 2 control for the Hiwonder JetRover's 4-DOF arm and gripper.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller_manager=jetrover_arm_control.robot_controller_node:main',
        ],
    },
)
