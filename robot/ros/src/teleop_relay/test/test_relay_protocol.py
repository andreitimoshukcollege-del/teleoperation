"""Mirrors teleoperation's RelayProtocolTests.cs -- same wire format, verified from both sides."""

import struct

from teleop_relay import relay_protocol


def test_decode_arm_command_round_trips_with_the_dotnet_encoder_layout():
    # version=4, count=2, then two (motor_id, pulse_target, pulses_per_second) tuples --
    # matches RelayProtocol.EncodeCommand's exact byte layout in the .NET project
    # (docs/adr/0011-generic-robot-arm-profiles.md).
    data = struct.pack("<BB", 4, 2) + struct.pack("<Bff", 1, -3.25, 300.0) + struct.pack("<Bff", 2, 1.5, 250.0)

    result = relay_protocol.decode_arm_command(data)

    assert result == [(1, -3.25, 300.0), (2, 1.5, 250.0)]


def test_decode_arm_command_empty_array_decodes_to_empty_list():
    data = struct.pack("<BB", 4, 0)

    assert relay_protocol.decode_arm_command(data) == []


def test_decode_arm_command_rejects_wrong_version():
    data = struct.pack("<BB", relay_protocol.VERSION - 1, 1) + struct.pack("<Bff", 1, 1.0, 300.0)

    assert relay_protocol.decode_arm_command(data) is None


def test_decode_arm_command_rejects_too_short_buffer():
    # Shorter than the 2-byte header itself -- b"\x04\x00" (2 bytes) is a legitimate, complete
    # empty-array record (see test_decode_arm_command_empty_array_decodes_to_empty_list), not a
    # too-short one.
    assert relay_protocol.decode_arm_command(b"\x04") is None
    assert relay_protocol.decode_arm_command(b"") is None


def test_decode_arm_command_rejects_declared_count_exceeding_actual_data():
    # Header claims 5 joints but only carries data for 1 -- must reject, not read past the buffer.
    data = struct.pack("<BB", relay_protocol.VERSION, 5) + struct.pack("<Bff", 1, 1.0, 300.0)

    assert relay_protocol.decode_arm_command(data) is None


def test_encode_feedback_matches_dotnet_layout():
    encoded = relay_protocol.encode_feedback([
        (1, True, -42.0),
        (2, False, 0.0),
        (3, True, 88.0),
        (4, True, -5.0),
    ])

    version, count = struct.unpack_from("<BB", encoded, 0)
    assert version == relay_protocol.VERSION
    assert count == 4

    offset = 2
    decoded = []
    for _ in range(count):
        motor_id, valid, pulse = struct.unpack_from("<Bbf", encoded, offset)
        decoded.append((motor_id, bool(valid), pulse))
        offset += relay_protocol.FEEDBACK_TUPLE_SIZE

    assert decoded == [(1, True, -42.0), (2, False, 0.0), (3, True, 88.0), (4, True, -5.0)]


def test_encode_feedback_empty_list_still_produces_a_valid_header_only_record():
    encoded = relay_protocol.encode_feedback([])

    assert encoded == struct.pack("<BB", relay_protocol.VERSION, 0)
