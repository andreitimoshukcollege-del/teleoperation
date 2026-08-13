"""Mirrors teleoperation's RelayProtocolTests.cs -- same wire format, verified from both sides."""

import struct

from teleop_relay import relay_protocol


def test_decode_arm_command_round_trips_with_the_dotnet_encoder_layout():
    # version=3, base/lower/middle/upper ABSOLUTE PULSE targets (not direction deltas --
    # teleoperation's docs/adr/0010-absolute-joint-targets-over-local-relay.md), gripper degrees,
    # little-endian -- matches RelayProtocol.EncodeCommand's exact byte layout in the .NET project.
    data = struct.pack("<Bfffff", 3, -3.25, 1.5, -0.75, 2.0, 120.0)

    result = relay_protocol.decode_arm_command(data)

    assert result == (-3.25, 1.5, -0.75, 2.0, 120.0)


def test_decode_arm_command_rejects_wrong_version():
    data = struct.pack("<Bfffff", relay_protocol.VERSION - 1, 1.0, 0.0, 0.0, 0.0, 90.0)

    assert relay_protocol.decode_arm_command(data) is None


def test_decode_arm_command_rejects_too_short_buffer():
    assert relay_protocol.decode_arm_command(b"\x02\x00\x00") is None


def test_encode_feedback_matches_dotnet_layout():
    encoded = relay_protocol.encode_feedback(
        base_valid=True, base_degrees=-42,
        lower_valid=False, lower_degrees=0,
        middle_valid=True, middle_degrees=88,
        upper_valid=True, upper_degrees=-5,
    )

    version, base_valid, base_degrees, lower_valid, lower_degrees, middle_valid, middle_degrees, upper_valid, upper_degrees = \
        struct.unpack("<BBiBiBiBi", encoded)
    assert version == relay_protocol.VERSION
    assert (base_valid, base_degrees) == (1, -42)
    assert (lower_valid, lower_degrees) == (0, 0)
    assert (middle_valid, middle_degrees) == (1, 88)
    assert (upper_valid, upper_degrees) == (1, -5)
