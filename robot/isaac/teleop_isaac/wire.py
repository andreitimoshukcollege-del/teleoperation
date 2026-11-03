"""The arm-server wire, version 1 (docs/adr/0030 decision 5): a copy of RobotHost's ``ArmServer/ArmServerWire.cs``.

Pinned to it by ``golden/arm-server-wire-v1.json``, which the C# tests write: change the two together. Little-endian;
every datagram starts ``'A' 'S'``, the version, the kind and a uint32 sequence. Float values are float32 on the wire.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

VERSION = 1
HEADER = struct.Struct("<2sBBI")
MAX_BYTES = 512
MAX_MOTORS = 36
MAX_NAME_BYTES = 64
MAX_DESCRIPTION_BYTES = 256

HELLO, WELCOME, TARGETS, ENABLE, STATE, HEALTH = 1, 2, 3, 4, 5, 6
FLAG_ENABLED, FLAG_BEHIND, FLAG_ERROR = 1, 2, 4
SERVER_ISAAC, SERVER_FCI, SERVER_SIMULATE = 1, 2, 3


def _header(kind: int, sequence: int) -> bytes:
    return HEADER.pack(b"AS", VERSION, kind, sequence & 0xFFFFFFFF)


def kind_of(datagram: bytes) -> int:
    """The datagram's kind, or 0 when it is not an arm-server datagram of this version."""
    if len(datagram) < HEADER.size:
        return 0
    magic, version, kind, _ = HEADER.unpack_from(datagram)
    return kind if magic == b"AS" and version == VERSION else 0


def sequence_of(datagram: bytes) -> int:
    return HEADER.unpack_from(datagram)[3] if len(datagram) >= HEADER.size else 0


@dataclass(frozen=True)
class Hello:
    host_ticks: int
    ticks_per_second: int
    profile: str


@dataclass(frozen=True)
class Targets:
    host_ticks: int
    targets: Tuple[Tuple[int, float], ...]  # (motor id, value)


@dataclass(frozen=True)
class Enable:
    host_ticks: int
    enabled: bool


def encode_hello(sequence: int, host_ticks: int, ticks_per_second: int, profile: str) -> bytes:
    name = profile.encode("utf-8")
    if len(name) > MAX_NAME_BYTES:
        raise ValueError("the profile's name is too long")
    return _header(HELLO, sequence) + struct.pack("<qqB", host_ticks, ticks_per_second, len(name)) + name


def decode_hello(d: bytes) -> Optional[Hello]:
    if kind_of(d) != HELLO or len(d) < 25:
        return None
    host_ticks, tps, n = struct.unpack_from("<qqB", d, 8)
    if n > MAX_NAME_BYTES or len(d) != 25 + n or tps <= 0:
        return None
    return Hello(host_ticks, tps, d[25:25 + n].decode("utf-8"))


def encode_welcome(sequence: int, server: int, server_now_ticks: int, motor_ids: Sequence[int], description: str) -> bytes:
    text = description.encode("utf-8")
    if len(motor_ids) > MAX_MOTORS or len(text) > MAX_DESCRIPTION_BYTES:
        raise ValueError("too many motors, or too long a description")
    return (_header(WELCOME, sequence) + struct.pack("<BqB", server, server_now_ticks, len(motor_ids)) + bytes(motor_ids)
            + struct.pack("<H", len(text)) + text)


def decode_welcome(d: bytes) -> Optional[Tuple[int, int, Tuple[int, ...], str]]:
    """(server kind, server clock now, motor ids, description)."""
    if kind_of(d) != WELCOME or len(d) < 20:
        return None
    server, now, motors = struct.unpack_from("<BqB", d, 8)
    at = 18 + motors
    if motors > MAX_MOTORS or len(d) < at + 2:
        return None
    (n,) = struct.unpack_from("<H", d, at)
    if n > MAX_DESCRIPTION_BYTES or len(d) != at + 2 + n:
        return None
    return server, now, tuple(d[18:at]), d[at + 2:at + 2 + n].decode("utf-8")


def encode_targets(sequence: int, host_ticks: int, targets: Sequence[Tuple[int, float]]) -> bytes:
    if len(targets) > MAX_MOTORS:
        raise ValueError("too many motors")
    body = b"".join(struct.pack("<Bf", m, v) for m, v in targets)
    return _header(TARGETS, sequence) + struct.pack("<qB", host_ticks, len(targets)) + body


def decode_targets(d: bytes) -> Optional[Targets]:
    if kind_of(d) != TARGETS or len(d) < 17:
        return None
    host_ticks, n = struct.unpack_from("<qB", d, 8)
    if n > MAX_MOTORS or len(d) != 17 + 5 * n:
        return None
    return Targets(host_ticks, tuple(struct.unpack_from("<Bf", d, 17 + 5 * i) for i in range(n)))


def encode_enable(sequence: int, host_ticks: int, enabled: bool) -> bytes:
    return _header(ENABLE, sequence) + struct.pack("<qB", host_ticks, 1 if enabled else 0)


def decode_enable(d: bytes) -> Optional[Enable]:
    if kind_of(d) != ENABLE or len(d) != 17 or d[16] > 1:
        return None
    (host_ticks,) = struct.unpack_from("<q", d, 8)
    return Enable(host_ticks, d[16] == 1)


def encode_state(sequence: int, sensed_ticks: int, server_ticks: int, flags: int,
                 motors: Sequence[Tuple[int, float, float, float]]) -> bytes:
    """``motors``: (id, position, velocity, commanded position) each."""
    if len(motors) > MAX_MOTORS:
        raise ValueError("too many motors")
    body = b"".join(struct.pack("<Bfff", m, q, v, c) for m, q, v, c in motors)
    return _header(STATE, sequence) + struct.pack("<qqBB", sensed_ticks, server_ticks, flags, len(motors)) + body


def decode_state(d: bytes) -> Optional[Tuple[int, int, int, List[Tuple[int, float, float, float]]]]:
    if kind_of(d) != STATE or len(d) < 26:
        return None
    sensed, server, flags, n = struct.unpack_from("<qqBB", d, 8)
    if n > MAX_MOTORS or len(d) != 26 + 13 * n:
        return None
    return sensed, server, flags, [struct.unpack_from("<Bfff", d, 26 + 13 * i) for i in range(n)]


def encode_health(sequence: int, real_time_factor: float, lag_ms: float, late_steps: int, steps: int) -> bytes:
    return _header(HEALTH, sequence) + struct.pack("<ffII", real_time_factor, lag_ms, late_steps & 0xFFFFFFFF, steps & 0xFFFFFFFF)


def decode_health(d: bytes) -> Optional[Tuple[float, float, int, int]]:
    if kind_of(d) != HEALTH or len(d) != 24:
        return None
    return struct.unpack_from("<ffII", d, 8)


# --- the world, kinds 7-11 (docs/adr/0031 amendment 2; RobotHost's ArmServer/ArmServerWorld.cs) -------------------------

WORLD_INFO, WORLD, PLACE, PLACED, CONTACTS = 7, 8, 9, 10, 11
MAX_OBJECTS_PER_WORLD = 16
MAX_CONTACTS = 64
CAUSE_LAYOUT, CAUSE_HEADSET, CAUSE_RESET = 1, 2, 3
(APPLIED, OVERLAPS_ARM, OVERLAPS_OBJECT, OUTSIDE_REGION, UNKNOWN_OBJECT, NOT_MOVABLE, ARM_TOUCHING, BUSY, INVALID_POSE,
 TIMED_OUT) = range(10)
FLAG_KINEMATIC = 1
_POSE = struct.Struct("<7f")  # x y z, qw qx qy qz


@dataclass(frozen=True)
class ObjectPose:
    id: int
    flags: int
    pose: Tuple[float, float, float, float, float, float, float]  # x, y, z, qw, qx, qy, qz


def encode_world_info(sequence: int, scene_sha256: bytes, ids: Sequence[int]) -> bytes:
    if len(scene_sha256) != 32 or len(ids) > 255:
        raise ValueError("a 32-byte sha256 and at most 255 ids")
    return _header(WORLD_INFO, sequence) + scene_sha256 + bytes([len(ids)]) + bytes(ids)


def decode_world_info(d: bytes) -> Optional[Tuple[bytes, Tuple[int, ...]]]:
    if kind_of(d) != WORLD_INFO or len(d) < 41 or len(d) != 41 + d[40]:
        return None
    return d[8:40], tuple(d[41:])


def encode_world(sequence: int, sensed_ticks: int, snapshot: int, revision: int, part: int, parts: int,
                 objects: Sequence[ObjectPose]) -> bytes:
    if len(objects) > MAX_OBJECTS_PER_WORLD or parts < 1 or not 0 <= part < parts:
        raise ValueError("at most 16 objects per part, and part < parts")
    body = b"".join(struct.pack("<BB", o.id, o.flags) + _POSE.pack(*o.pose) for o in objects)
    return (_header(WORLD, sequence) + struct.pack("<qIIBBB", sensed_ticks, snapshot & 0xFFFFFFFF, revision & 0xFFFFFFFF,
                                                    part, parts, len(objects)) + body)


def decode_world(d: bytes) -> Optional[Tuple[int, int, int, int, int, List[ObjectPose]]]:
    """(sensed ticks, snapshot, revision, part, parts, objects)."""
    if kind_of(d) != WORLD or len(d) < 27:
        return None
    sensed, snapshot, revision, part, parts, n = struct.unpack_from("<qIIBBB", d, 8)
    if n > MAX_OBJECTS_PER_WORLD or len(d) != 27 + 30 * n or parts == 0 or part >= parts:
        return None
    objects = [ObjectPose(d[27 + 30 * i], d[28 + 30 * i], _POSE.unpack_from(d, 29 + 30 * i)) for i in range(n)]
    return sensed, snapshot, revision, part, parts, objects


def encode_place(sequence: int, host_ticks: int, request: int, cause: int, object_id: int, pose: Sequence[float]) -> bytes:
    return _header(PLACE, sequence) + struct.pack("<qIBB", host_ticks, request & 0xFFFFFFFF, cause, object_id) + _POSE.pack(*pose)


def decode_place(d: bytes) -> Optional[Tuple[int, int, int, int, Tuple[float, ...]]]:
    """(host ticks, request, cause, object id, pose)."""
    if kind_of(d) != PLACE or len(d) != 50 or not 1 <= d[20] <= 3:
        return None
    host_ticks, request, cause, object_id = struct.unpack_from("<qIBB", d, 8)
    return host_ticks, request, cause, object_id, _POSE.unpack_from(d, 22)


def encode_placed(sequence: int, request: int, object_id: int, outcome: int, revision: int, pose: Sequence[float]) -> bytes:
    return (_header(PLACED, sequence) + struct.pack("<IBBI", request & 0xFFFFFFFF, object_id, outcome, revision & 0xFFFFFFFF)
            + _POSE.pack(*pose))


def decode_placed(d: bytes) -> Optional[Tuple[int, int, int, int, Tuple[float, ...]]]:
    """(request, object id, outcome, revision, pose)."""
    if kind_of(d) != PLACED or len(d) != 46 or d[13] > TIMED_OUT:
        return None
    request, object_id, outcome, revision = struct.unpack_from("<IBBI", d, 8)
    return request, object_id, outcome, revision, _POSE.unpack_from(d, 18)


def encode_contacts(sequence: int, sensed_ticks: int, contacts: Sequence[Tuple[int, int, int, float]]) -> bytes:
    """``contacts``: (arm, link, object (0 = table), normal force N) each."""
    if len(contacts) > MAX_CONTACTS:
        raise ValueError("too many contacts")
    body = b"".join(struct.pack("<BBBf", a, l, o, f) for a, l, o, f in contacts)
    return _header(CONTACTS, sequence) + struct.pack("<qB", sensed_ticks, len(contacts)) + body


def decode_contacts(d: bytes) -> Optional[Tuple[int, List[Tuple[int, int, int, float]]]]:
    if kind_of(d) != CONTACTS or len(d) < 17:
        return None
    sensed, n = struct.unpack_from("<qB", d, 8)
    if n > MAX_CONTACTS or len(d) != 17 + 7 * n:
        return None
    return sensed, [struct.unpack_from("<BBBf", d, 17 + 7 * i) for i in range(n)]
