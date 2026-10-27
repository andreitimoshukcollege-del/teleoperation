"""Fixed-header, count-prefixed, versioned, little-endian wire format for the local relay
channel.

Must match core/Teleop.RobotHost/Relay/RelayProtocol.cs in the `teleoperation` repo exactly --
this is the one wire format shared between the two languages, kept deliberately tiny (no
sequence numbers, no staleness/coast logic) because it only ever crosses a local Unix domain
socket on one machine. See that file's own doc comment for the full reasoning.

Every message is `u8 version | u8 kind | body`:
- KIND_COMMAND (RobotHost -> relay): u8 count, then count x (u8 motor_id, f32 pulse, f32 pulses/s)
- KIND_FEEDBACK (relay -> RobotHost): u8 count, then count x (u8 motor_id, u8 valid, f32 pulse)
- KIND_TORQUE (RobotHost -> relay): u8 on (1 on, 0 off), published on the driver's
  /arm/servo/set/torque (teleoperation docs/adr/0023 decision 5)
"""

import struct

# v4 (teleoperation's docs/adr/0011-generic-robot-arm-profiles.md): a count-prefixed array of
# (motor_id, pulse, speed) tuples instead of a fixed 4-joint-plus-gripper struct.
# v5 (2026-10-26, docs/adr/0023 decision 5): a kind byte after the version, and the torque message.
#
# BOTH ENDS DEPLOY TOGETHER: this file and RelayProtocol.cs change in the same commit and go out
# together (just deploy-robot; or just deploy-ros, then just deploy-robothost). A mismatch fails
# closed: each end rejects every message whose version or kind it does not know, so nothing moves
# and nothing is read, rather than a layout being misread.
VERSION = 5

KIND_COMMAND = 1
KIND_FEEDBACK = 2
KIND_TORQUE = 3

_PREFIX_FORMAT = "<BB"  # version, kind
PREFIX_SIZE = struct.calcsize(_PREFIX_FORMAT)

_HEADER_FORMAT = "<BBB"  # version, kind, count
HEADER_SIZE = struct.calcsize(_HEADER_FORMAT)

_TORQUE_FORMAT = "<BBB"  # version, kind, on
TORQUE_SIZE = struct.calcsize(_TORQUE_FORMAT)

# motor_id, pulse_target, pulses_per_second -- both pulse-unit fields, continuing
# docs/adr/0010's reasoning for choosing pulse over radians on this hop (avoids duplicating
# PulsePerRadian/ZeroPulse on this side).
_COMMAND_TUPLE_FORMAT = "<Bff"
COMMAND_TUPLE_SIZE = struct.calcsize(_COMMAND_TUPLE_FORMAT)

# motor_id, valid (0/1), pulse -- raw pulse directly, no degree round-trip.
_FEEDBACK_TUPLE_FORMAT = "<Bbf"
FEEDBACK_TUPLE_SIZE = struct.calcsize(_FEEDBACK_TUPLE_FORMAT)

# Generous relative to any profile this platform realistically targets (JetRover has 6 joints) --
# this hop is a local Unix domain socket, not a UDP link with a practical MTU to budget against.
MAX_JOINTS_PER_MESSAGE = 32


def command_encoded_size(count: int) -> int:
    return HEADER_SIZE + count * COMMAND_TUPLE_SIZE


def feedback_encoded_size(count: int) -> int:
    return HEADER_SIZE + count * FEEDBACK_TUPLE_SIZE


def kind_of(data: bytes) -> int:
    """The kind byte of a message of this version, or 0 when it is too short or another version
    -- mirrors RelayProtocol.KindOf."""
    if len(data) < PREFIX_SIZE:
        return 0
    version, kind = struct.unpack_from(_PREFIX_FORMAT, data, 0)
    return kind if version == VERSION else 0


def decode_arm_command(data: bytes):
    """Returns a list of (motor_id, pulse_target, pulses_per_second) tuples, or None if `data`
    isn't a well-formed, version- and kind-matching, length-consistent record -- mirrors
    RelayProtocol.TryDecodeCommand's "reject, don't throw" contract.
    """
    if len(data) < HEADER_SIZE or kind_of(data) != KIND_COMMAND:
        return None
    _, _, count = struct.unpack_from(_HEADER_FORMAT, data, 0)
    if count > MAX_JOINTS_PER_MESSAGE or len(data) < command_encoded_size(count):
        return None

    targets = []
    offset = HEADER_SIZE
    for _ in range(count):
        motor_id, pulse, speed = struct.unpack_from(_COMMAND_TUPLE_FORMAT, data, offset)
        targets.append((motor_id, pulse, speed))
        offset += COMMAND_TUPLE_SIZE
    return targets


def decode_torque(data: bytes):
    """True for torque on, False for off, or None for anything that is not a well-formed torque
    message of this version (including a value other than 0 and 1) -- mirrors
    RelayProtocol.TryDecodeTorque."""
    if len(data) < TORQUE_SIZE or kind_of(data) != KIND_TORQUE:
        return None
    _, _, on = struct.unpack_from(_TORQUE_FORMAT, data, 0)
    if on > 1:
        return None
    return on == 1


def encode_torque(on: bool) -> bytes:
    return struct.pack(_TORQUE_FORMAT, VERSION, KIND_TORQUE, 1 if on else 0)


def sort_datagrams(datagrams):
    """What the relay does with one drain of its socket: a list of actions in arrival order,
    ("torque", on) for every torque message and ("command", targets) for the newest command only,
    plus how many datagrams were rejected. A backlog must not replay stale commands (only the newest
    is current), but every torque switch counts, in order, and none may be lost behind a command
    (relay_node.py's _poll_local_socket)."""
    actions = []
    newest_command = None
    rejected = 0
    for data in datagrams:
        kind = kind_of(data)
        if kind == KIND_TORQUE:
            on = decode_torque(data)
            if on is None:
                rejected += 1
            else:
                actions.append(("torque", on))
        elif kind == KIND_COMMAND:
            targets = decode_arm_command(data)
            if targets is None:
                rejected += 1
            else:
                newest_command = (len(actions), targets)
        else:
            rejected += 1
    if newest_command is not None:
        index, targets = newest_command
        # Placed after the torque switches that arrived before it, before the ones after it.
        actions.insert(index, ("command", targets))
    return actions, rejected


def encode_feedback(entries) -> bytes:
    """`entries`: an iterable of (motor_id, valid, pulse) tuples."""
    entries = list(entries)
    data = struct.pack(_HEADER_FORMAT, VERSION, KIND_FEEDBACK, len(entries))
    for motor_id, valid, pulse in entries:
        data += struct.pack(_FEEDBACK_TUPLE_FORMAT, motor_id, 1 if valid else 0, pulse)
    return data
