"""Fixed-size, versioned, little-endian wire format for the local relay channel.

Must match core/Teleop.RobotHost/Relay/RelayProtocol.cs in the `teleoperation` repo exactly --
this is the one wire format shared between the two languages, kept deliberately tiny (no
sequence numbers, no staleness/coast logic) because it only ever crosses a local Unix domain
socket on one machine. See that file's own doc comment for the full reasoning.
"""

import struct

# v3: base/lower/middle/upper are absolute pulse targets (0-1000, hardware units), not relative
# direction deltas -- teleoperation's docs/adr/0010-absolute-joint-targets-over-local-relay.md.
# Byte layout is unchanged from v2 (still 5 floats); only the meaning of the first four inverts,
# which is exactly why the version must still bump -- a stale peer must reject, not
# misinterpret an absolute target as a delta or vice versa.
VERSION = 3

# version, base/lower/middle/upper absolute pulse target, gripper degrees (absolute)
_ARM_COMMAND_FORMAT = "<Bfffff"
ARM_COMMAND_SIZE = struct.calcsize(_ARM_COMMAND_FORMAT)

# version, then 4x (valid_byte, degrees_int32) for base/lower/middle/upper
_FEEDBACK_FORMAT = "<B" + "Bi" * 4
FEEDBACK_SIZE = struct.calcsize(_FEEDBACK_FORMAT)


def decode_arm_command(data: bytes):
    """Returns (base_pulse, lower_pulse, middle_pulse, upper_pulse, gripper_degrees), or None if
    `data` isn't a well-formed, version-matching LocalArmCommand -- mirrors
    RelayProtocol.TryDecodeCommand's "reject, don't throw" contract.
    """
    if len(data) < ARM_COMMAND_SIZE:
        return None
    version, base_pulse, lower_pulse, middle_pulse, upper_pulse, gripper_degrees = \
        struct.unpack_from(_ARM_COMMAND_FORMAT, data, 0)
    if version != VERSION:
        return None
    return base_pulse, lower_pulse, middle_pulse, upper_pulse, gripper_degrees


def encode_feedback(
    base_valid: bool, base_degrees: int,
    lower_valid: bool, lower_degrees: int,
    middle_valid: bool, middle_degrees: int,
    upper_valid: bool, upper_degrees: int,
) -> bytes:
    return struct.pack(
        _FEEDBACK_FORMAT,
        VERSION,
        1 if base_valid else 0, int(base_degrees),
        1 if lower_valid else 0, int(lower_degrees),
        1 if middle_valid else 0, int(middle_degrees),
        1 if upper_valid else 0, int(upper_degrees),
    )
