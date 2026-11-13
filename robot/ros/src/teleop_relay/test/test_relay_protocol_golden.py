"""relay_protocol.py against the golden file RobotHost's codec writes (teleoperation's
core/Teleop.RobotHost.Tests/Relay/RelayProtocolGoldenTests.cs; regenerate with just gen-relay-golden).

The two ends change together and deploy together. This file is what makes a mismatch fail a test instead of a
robot: every message the C# encodes, this module must decode (or encode) to the same fields and bytes, and every
message the C# rejects, this module must reject.
"""

import json
import os

from teleop_relay import relay_protocol

GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "golden", "relay-protocol-v5.json")


def _golden():
    with open(GOLDEN, encoding="utf-8") as f:
        return json.load(f)


def test_the_golden_file_is_this_version():
    assert _golden()["version"] == relay_protocol.VERSION


def test_every_message_has_the_same_fields_and_bytes():
    kinds = {"command": relay_protocol.KIND_COMMAND, "feedback": relay_protocol.KIND_FEEDBACK,
             "torque": relay_protocol.KIND_TORQUE}
    seen = set()
    for message in _golden()["messages"]:
        data = bytes.fromhex(message["hex"])
        kind = message["kind"]
        seen.add(kind)
        assert relay_protocol.kind_of(data) == kinds[kind]
        if kind == "command":
            expected = [(t[0], float(t[1]), float(t[2])) for t in message["targets"]]
            assert relay_protocol.decode_arm_command(data) == expected
            assert len(data) == relay_protocol.command_encoded_size(len(expected))
        elif kind == "feedback":
            entries = [(e[0], e[1], float(e[2])) for e in message["entries"]]
            assert relay_protocol.encode_feedback(entries) == data
            assert len(data) == relay_protocol.feedback_encoded_size(len(entries))
        else:
            assert relay_protocol.encode_torque(message["on"]) == data
            assert relay_protocol.decode_torque(data) is message["on"]
    assert seen == set(kinds)


def test_what_the_dotnet_codec_rejects_this_module_rejects():
    decoders = {"command": relay_protocol.decode_arm_command, "torque": relay_protocol.decode_torque}
    rejects = _golden()["rejects"]
    assert rejects
    for reject in rejects:
        assert decoders[reject["decoder"]](bytes.fromhex(reject["hex"])) is None, reject["why"]
