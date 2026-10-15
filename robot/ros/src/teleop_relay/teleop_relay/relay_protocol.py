"""Fixed-header, count-prefixed, versioned, little-endian wire format for the local relay
channel.

Must match core/Teleop.RobotHost/Relay/RelayProtocol.cs in the `teleoperation` repo exactly --
this is the one wire format shared between the two languages, kept deliberately tiny (no
sequence numbers, no staleness/coast logic) because it only ever crosses a local Unix domain
socket on one machine. See that file's own doc comment for the full reasoning.
"""

import struct

# v4 (teleoperation's docs/adr/0011-generic-robot-arm-profiles.md): a structurally different
# layout from v3 -- a count-prefixed array of (motor_id, pulse, speed) tuples instead of a fixed
# 4-joint-plus-gripper struct, so any robot profile's joint count/motor ids can be carried without
# a wire-format change. Must still fail closed on a version mismatch, same as every prior bump.
VERSION = 4

_HEADER_FORMAT = "<BB"  # version, count
HEADER_SIZE = struct.calcsize(_HEADER_FORMAT)

# motor_id, pulse_target, pulses_per_second -- both pulse-unit fields, continuing
# docs/adr/0010's reasoning for choosing pulse over radians on this hop (avoids duplicating
# PulsePerRadian/ZeroPulse on this side). The gripper is no longer a separate degrees-based
# special case -- it is just another entry in this same array.
_COMMAND_TUPLE_FORMAT = "<Bff"
COMMAND_TUPLE_SIZE = struct.calcsize(_COMMAND_TUPLE_FORMAT)

# motor_id, valid (0/1), pulse -- raw pulse directly, no degree round-trip (bus_servo_read_position
# already returns raw pulse; the old wire only used degrees because that was the fixed protocol's
# field choice, not because the underlying read was ever in degrees).
_FEEDBACK_TUPLE_FORMAT = "<Bbf"
FEEDBACK_TUPLE_SIZE = struct.calcsize(_FEEDBACK_TUPLE_FORMAT)

# Generous relative to any profile this platform realistically targets (JetRover has 5 joints) --
# this hop is a local Unix domain socket, not a UDP link with a practical MTU to budget against.
MAX_JOINTS_PER_MESSAGE = 32


def command_encoded_size(count: int) -> int:
    return HEADER_SIZE + count * COMMAND_TUPLE_SIZE


def feedback_encoded_size(count: int) -> int:
    return HEADER_SIZE + count * FEEDBACK_TUPLE_SIZE


def decode_arm_command(data: bytes):
    """Returns a list of (motor_id, pulse_target, pulses_per_second) tuples, or None if `data`
    isn't a well-formed, version-matching, length-consistent record -- mirrors
    RelayProtocol.TryDecodeCommand's "reject, don't throw" contract.
    """
    if len(data) < HEADER_SIZE:
        return None
    version, count = struct.unpack_from(_HEADER_FORMAT, data, 0)
    if version != VERSION or count > MAX_JOINTS_PER_MESSAGE or len(data) < command_encoded_size(count):
        return None

    targets = []
    offset = HEADER_SIZE
    for _ in range(count):
        motor_id, pulse, speed = struct.unpack_from(_COMMAND_TUPLE_FORMAT, data, offset)
        targets.append((motor_id, pulse, speed))
        offset += COMMAND_TUPLE_SIZE
    return targets


def encode_feedback(entries) -> bytes:
    """`entries`: an iterable of (motor_id, valid, pulse) tuples."""
    entries = list(entries)
    data = struct.pack(_HEADER_FORMAT, VERSION, len(entries))
    for motor_id, valid, pulse in entries:
        data += struct.pack(_FEEDBACK_TUPLE_FORMAT, motor_id, 1 if valid else 0, pulse)
    return data
