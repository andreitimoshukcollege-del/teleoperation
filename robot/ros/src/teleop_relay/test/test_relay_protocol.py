"""Mirrors teleoperation's RelayProtocolTests.cs -- same wire format, verified from both sides."""

import struct

from teleop_relay import relay_protocol


def _command(count, *tuples, version=relay_protocol.VERSION, kind=relay_protocol.KIND_COMMAND):
    data = struct.pack("<BBB", version, kind, count)
    for t in tuples:
        data += struct.pack("<Bff", *t)
    return data


def test_version_is_5():
    assert relay_protocol.VERSION == 5


def test_decode_arm_command_round_trips_with_the_dotnet_encoder_layout():
    # version=5, kind=1 (command), count=2, then two (motor_id, pulse_target, pulses_per_second)
    # tuples -- matches RelayProtocol.EncodeCommand's exact byte layout in the .NET project.
    data = _command(2, (1, -3.25, 300.0), (2, 1.5, 250.0))

    result = relay_protocol.decode_arm_command(data)

    assert result == [(1, -3.25, 300.0), (2, 1.5, 250.0)]


def test_decode_arm_command_matches_the_dotnet_bytes():
    # RelayProtocolTests.Command_LayoutIsVersionKindCount: EncodeCommand([7, 500, 119]).
    data = bytes([5, 1, 1, 7]) + struct.pack("<ff", 500.0, 119.0)

    assert relay_protocol.decode_arm_command(data) == [(7, 500.0, 119.0)]


def test_decode_arm_command_empty_array_decodes_to_empty_list():
    assert relay_protocol.decode_arm_command(_command(0)) == []


def test_decode_arm_command_rejects_wrong_version():
    assert relay_protocol.decode_arm_command(_command(1, (1, 1.0, 300.0), version=relay_protocol.VERSION - 1)) is None


def test_decode_arm_command_rejects_a_version_4_command():
    # The old layout (version, count, tuples): a v4 RobotHost must move nothing.
    data = struct.pack("<BB", 4, 1) + struct.pack("<Bff", 1, 1.0, 300.0)

    assert relay_protocol.decode_arm_command(data) is None


def test_decode_arm_command_rejects_another_kind():
    assert relay_protocol.decode_arm_command(_command(1, (1, 1.0, 300.0), kind=relay_protocol.KIND_FEEDBACK)) is None
    assert relay_protocol.decode_arm_command(relay_protocol.encode_torque(True)) is None


def test_decode_arm_command_rejects_too_short_buffer():
    # Shorter than the 3-byte header itself -- b"\x05\x01\x00" is a complete empty-array record.
    assert relay_protocol.decode_arm_command(b"\x05\x01") is None
    assert relay_protocol.decode_arm_command(b"") is None
    assert relay_protocol.decode_arm_command(b"\x05\x01\x00") == []


def test_decode_arm_command_rejects_declared_count_exceeding_actual_data():
    # Header claims 5 joints but only carries data for 1 -- must reject, not read past the buffer.
    assert relay_protocol.decode_arm_command(_command(5, (1, 1.0, 300.0))) is None


def test_encode_feedback_matches_dotnet_layout():
    encoded = relay_protocol.encode_feedback([
        (1, True, -42.0),
        (2, False, 0.0),
        (3, True, 88.0),
        (4, True, -5.0),
    ])

    version, kind, count = struct.unpack_from("<BBB", encoded, 0)
    assert version == relay_protocol.VERSION
    assert kind == relay_protocol.KIND_FEEDBACK
    assert count == 4

    offset = 3
    decoded = []
    for _ in range(count):
        motor_id, valid, pulse = struct.unpack_from("<Bbf", encoded, offset)
        decoded.append((motor_id, bool(valid), pulse))
        offset += relay_protocol.FEEDBACK_TUPLE_SIZE

    assert decoded == [(1, True, -42.0), (2, False, 0.0), (3, True, 88.0), (4, True, -5.0)]


def test_encode_feedback_empty_list_still_produces_a_valid_header_only_record():
    encoded = relay_protocol.encode_feedback([])

    assert encoded == struct.pack("<BBB", relay_protocol.VERSION, relay_protocol.KIND_FEEDBACK, 0)


def test_torque_matches_the_dotnet_bytes():
    # RelayProtocolTests.Torque_RoundTrips_AndHasAFixedLayout.
    assert relay_protocol.encode_torque(False) == bytes([5, 3, 0])
    assert relay_protocol.encode_torque(True) == bytes([5, 3, 1])
    assert relay_protocol.decode_torque(bytes([5, 3, 0])) is False
    assert relay_protocol.decode_torque(bytes([5, 3, 1])) is True


def test_decode_torque_fails_closed():
    assert relay_protocol.decode_torque(bytes([5, 3])) is None
    assert relay_protocol.decode_torque(bytes([4, 3, 0])) is None
    assert relay_protocol.decode_torque(bytes([5, 1, 0])) is None
    assert relay_protocol.decode_torque(bytes([5, 3, 2])) is None
    assert relay_protocol.decode_torque(b"") is None


def test_kind_of():
    assert relay_protocol.kind_of(_command(0)) == relay_protocol.KIND_COMMAND
    assert relay_protocol.kind_of(relay_protocol.encode_feedback([])) == relay_protocol.KIND_FEEDBACK
    assert relay_protocol.kind_of(relay_protocol.encode_torque(True)) == relay_protocol.KIND_TORQUE
    assert relay_protocol.kind_of(bytes([4, 1])) == 0
    assert relay_protocol.kind_of(bytes([5])) == 0


def test_sort_datagrams_keeps_every_torque_switch_and_only_the_newest_command_in_order():
    old = _command(1, (1, 100.0, 300.0))
    new = _command(1, (1, 200.0, 300.0))
    off = relay_protocol.encode_torque(False)
    on = relay_protocol.encode_torque(True)

    actions, rejected = relay_protocol.sort_datagrams([old, on, new, off, b"\x04\x01\x00"])

    assert actions == [("torque", True), ("command", [(1, 200.0, 300.0)]), ("torque", False)]
    assert rejected == 1


def test_sort_datagrams_with_nothing_valid():
    assert relay_protocol.sort_datagrams([bytes([5, 3, 9]), b""]) == ([], 2)
    assert relay_protocol.sort_datagrams([]) == ([], 0)
