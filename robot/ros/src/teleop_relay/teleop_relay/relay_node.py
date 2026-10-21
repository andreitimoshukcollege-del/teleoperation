"""Bridges Teleop.RobotHost (a .NET process, the real IRobotPlant/ITransport endpoint -- see
teleoperation's docs/adr/0007-jetrover-plant-and-robot-host.md) to jetrover_arm_control, over a
local Unix domain socket (relay_protocol.py). Deliberately thin: no staleness/sequencing/
gap-policy logic here at all -- that's GenericArmPlant's job, upstream of this socket. This node
only ever translates "here are the current per-motor-id pulse/speed targets" into one ROS topic
carrying the whole array, and relays servo feedback back the same way.

Wire v4 (teleoperation's docs/adr/0011-generic-robot-arm-profiles.md): a count-prefixed array of
(motor_id, pulse_target, pulses_per_second) tuples, replacing the old fixed 4-joint-plus-gripper
struct entirely. This node is now genuinely profile-agnostic -- it never looks up a joint name,
only forwards whatever motor ids show up on the wire. The internal hop to
jetrover_arm_control uses a plain std_msgs/Float32MultiArray (not a new custom .msg type) to
avoid introducing rosidl message-generation build machinery for a purely internal, single-hop
array -- see this module's own encode/decode helpers below for the exact layout, which mirrors
relay_protocol.py's own tuple shapes.

Each motor id's feedback validity is time-boxed (StaleTrackedValue), not "has a reading ever
arrived": a joint whose real feedback silently stops (e.g. a servo whose reads never succeed) must
eventually report invalid again, not keep replaying its one lucky reading forever -- a real bug
found via move-arm 0.1 0.1 0.1 no longer converging on 2026-08-08/09, even though the .NET side's
belief tracking was correct.
"""

import os
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from teleop_relay import relay_protocol
from teleop_relay.stale_tracked_value import StaleTrackedValue

LOCAL_RELAY_SOCKET = "/tmp/jetrover_relay.sock"
HOST_SOCKET = "/tmp/teleop_robot_host.sock"

_COMMAND_POLL_PERIOD_SECONDS = 0.02  # 50 Hz
_FEEDBACK_SEND_PERIOD_SECONDS = 0.05  # 20 Hz

# ~10x robot_controller_node.py's 50Hz feedback poll rate -- generous enough to absorb normal
# jitter or an occasional dropped read, tight enough to flag a genuinely dead servo within a
# fraction of a second instead of never.
_STALENESS_TIMEOUT_SECONDS = 0.2

# How often the node logs its statistics, seconds (2026-10-20: while the headset streamed
# commands, RobotHost got a valid reading only about once a second although the driver published
# 50 a second; these say where they are lost).
_STATS_PERIOD_SECONDS = 5.0


def _encode_command_array(targets) -> Float32MultiArray:
    """[count, motor_id_0, pulse_0, speed_0, motor_id_1, ...] -- a plain std_msgs array instead of
    a custom .msg type, see this module's own doc for why."""
    msg = Float32MultiArray()
    data = [float(len(targets))]
    for motor_id, pulse, speed in targets:
        data.extend([float(motor_id), float(pulse), float(speed)])
    msg.data = data
    return msg


def _decode_feedback_array(msg: Float32MultiArray):
    """Inverse of robot_controller_node.py's own feedback_array encoding: [count, motor_id_0,
    pulse_0, motor_id_1, pulse_1, ...]. Returns a list of (motor_id, pulse) pairs, tolerating a
    malformed/truncated array by simply stopping rather than raising."""
    data = msg.data
    if len(data) < 1:
        return []

    count = int(data[0])
    readings = []
    offset = 1
    for _ in range(count):
        if offset + 1 >= len(data):
            break
        readings.append((int(data[offset]), data[offset + 1]))
        offset += 2
    return readings


class TeleopRelayNode(Node):
    def __init__(self):
        super().__init__("teleop_relay_node")

        # A stale socket file left over from a crashed previous run must not block binding --
        # same reasoning as UdsRelayClient's own constructor on the .NET side.
        if os.path.exists(LOCAL_RELAY_SOCKET):
            os.remove(LOCAL_RELAY_SOCKET)

        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.bind(LOCAL_RELAY_SOCKET)
        self._socket.setblocking(False)

        self._command_pub = self.create_publisher(Float32MultiArray, "/arm/servo/command_array", 3)
        self.create_subscription(
            Float32MultiArray, "/unity/robot/servo/feedback_array", self._on_feedback_array, 1)

        # Motor-id-keyed, created lazily per id the first time it's seen -- this node no longer
        # needs a fixed, hand-maintained list of joint names to track feedback for.
        self._feedback_by_motor_id = {}

        self.create_timer(_COMMAND_POLL_PERIOD_SECONDS, self._poll_local_socket)
        self.create_timer(_FEEDBACK_SEND_PERIOD_SECONDS, self._send_feedback)

        self.get_logger().info(
            f"Relay listening on {LOCAL_RELAY_SOCKET}, forwarding feedback to {HOST_SOCKET}"
        )
        self._reset_stats(time.monotonic())
        self._last_feedback_at = None

    def _reset_stats(self, now):
        self._stats_since = now
        self._feedback_msgs = 0
        self._longest_feedback_gap_ms = 0.0
        self._sends = 0
        self._sends_all_valid = 0
        self._send_failures = 0
        self._datagrams = 0
        self._commands = 0
        self._slowest_callback_ms = 0.0

    def _note_callback(self, start):
        self._slowest_callback_ms = max(self._slowest_callback_ms, (time.monotonic() - start) * 1000.0)
        if start - self._stats_since < _STATS_PERIOD_SECONDS:
            return
        seconds = start - self._stats_since
        self.get_logger().info(
            f"[stats {seconds:.1f}s] feedback messages {self._feedback_msgs} "
            f"({self._feedback_msgs / seconds:.1f} Hz), longest gap {self._longest_feedback_gap_ms:.0f} ms; "
            f"feedback sent {self._sends}, all readings valid in {self._sends_all_valid}, "
            f"send failures {self._send_failures}; "
            f"command datagrams {self._datagrams}, commands published {self._commands}; "
            f"slowest callback {self._slowest_callback_ms:.1f} ms")
        self._reset_stats(start)

    def _on_feedback_array(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        self._feedback_msgs += 1
        if self._last_feedback_at is not None:
            self._longest_feedback_gap_ms = max(self._longest_feedback_gap_ms, (now - self._last_feedback_at) * 1000.0)
        self._last_feedback_at = now
        for motor_id, pulse in _decode_feedback_array(msg):
            tracked = self._feedback_by_motor_id.get(motor_id)
            if tracked is None:
                tracked = StaleTrackedValue(_STALENESS_TIMEOUT_SECONDS)
                self._feedback_by_motor_id[motor_id] = tracked
            tracked.update(pulse, now)
        self._note_callback(now)

    def _poll_local_socket(self) -> None:
        # Drain every pending datagram but act only on the most recent -- IRelayClient's
        # contract is "here is the current setpoint," not a queue of every one ever sent, so a
        # backlog here (this node briefly starved of CPU) must not replay stale commands once
        # it catches up.
        start = time.monotonic()
        latest = None
        max_size = relay_protocol.command_encoded_size(relay_protocol.MAX_JOINTS_PER_MESSAGE)
        while True:
            try:
                latest, _ = self._socket.recvfrom(max_size)
                self._datagrams += 1
            except BlockingIOError:
                break

        if latest is None:
            self._note_callback(start)
            return

        targets = relay_protocol.decode_arm_command(latest)
        if targets is None:
            self._note_callback(start)
            return

        self._command_pub.publish(_encode_command_array(targets))
        self._commands += 1
        self._note_callback(start)

    def _send_feedback(self) -> None:
        now = time.monotonic()
        entries = []
        for motor_id, tracked in self._feedback_by_motor_id.items():
            valid, pulse = tracked.read(now)
            entries.append((motor_id, valid, pulse))

        self._sends += 1
        if entries and all(valid for _, valid, _ in entries):
            self._sends_all_valid += 1
        payload = relay_protocol.encode_feedback(entries)
        try:
            self._socket.sendto(payload, HOST_SOCKET)
        except OSError:
            self._send_failures += 1
            # Teleop.RobotHost isn't up yet, or has restarted -- fire-and-forget, matching
            # UdsRelayClient.Send's own tolerance for the mirror-image case.
            pass


def main():
    rclpy.init()
    node = TeleopRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
