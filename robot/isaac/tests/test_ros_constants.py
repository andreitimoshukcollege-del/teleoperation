"""The Isaac JetRover's copies of the real ROS nodes' timing and queue constants (docs/adr/0034 amendment 13).

relay_node.py and robot_controller_node.py import rclpy at the top, so the emulation cannot import their constants
and copies them instead. This reads the nodes' source and fails when a copy no longer matches: a change to the robot's
relay poll or driver queue must reach the emulation in the same commit, or the simulated robot quietly stops being
the real one.
"""

import inspect
import json
import os
import re

from conftest import REPO
from teleop_isaac import servo

ROS = os.path.join(REPO, "robot", "ros", "src")
RELAY = os.path.join(ROS, "teleop_relay", "teleop_relay", "relay_node.py")
DRIVER = os.path.join(ROS, "jetrover_arm_control", "jetrover_arm_control", "robot_controller_node.py")


def _source(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _constant(path: str, pattern: str) -> float:
    found = re.findall(pattern, _source(path), re.MULTILINE)
    assert len(found) == 1, f"{os.path.basename(path)}: expected one match of {pattern!r}, found {found}"
    return float(found[0])


def _sim() -> dict:
    with open(servo.DEFAULT_SIM, encoding="utf-8") as f:
        return json.load(f)


def test_the_relay_poll_is_the_relays():
    assert servo.RelayEmulator.POLL_SECONDS == _constant(RELAY, r"^_COMMAND_POLL_PERIOD_SECONDS\s*=\s*([0-9.]+)")


def test_the_relay_feedback_period_is_the_relays():
    assert _sim()["feedbackPeriodSeconds"] == _constant(RELAY, r"^_FEEDBACK_SEND_PERIOD_SECONDS\s*=\s*([0-9.]+)")


def test_a_reading_is_valid_as_long_as_the_relay_keeps_it():
    default = inspect.signature(servo.FeedbackEmulator.__init__).parameters["valid_seconds"].default
    assert default == _constant(RELAY, r"^_STALENESS_TIMEOUT_SECONDS\s*=\s*([0-9.]+)")


def test_the_driver_cycle_is_the_drivers_poll_rate():
    assert _sim()["driverCycleSeconds"] == _constant(DRIVER, r"self\.pollRate\s*=\s*([0-9.]+)")


def test_the_driver_queues_are_the_drivers():
    assert servo.DriverEmulator.COMMAND_DEPTH == _constant(
        DRIVER, r'"/arm/servo/command_array",\s*self\._timedCommandCbk,\s*(\d+)\)')
    assert servo.DriverEmulator.TORQUE_DEPTH == _constant(
        DRIVER, r'"/arm/servo/set/torque",\s*self\._torqueCbk,\s*(\d+)\)')
