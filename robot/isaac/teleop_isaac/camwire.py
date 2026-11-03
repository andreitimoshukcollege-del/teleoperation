"""ADR 0014's camera wire, as the Isaac renderer sends it (docs/adr/0032 decision 3).

A copy of Core's ``Camera/CameraChunkCodec.cs`` and ``CameraSubscribeCodec.cs``, pinned by ``golden/camera-wire-v1.json``
(written by ``CameraWireGoldenTests``, ``just gen-camera-wire-golden``). Little-endian throughout.

A frame's chunk: a 41-byte header, then a slice of its JPEG; every chunk but the last carries exactly
:data:`MAX_PAYLOAD_BYTES`, so a chunk's index alone places its bytes.

    0 u16 magic 0x5643, 2 u8 version, 3 u32 frameId, 7 u16 chunkIndex, 9 u16 chunkCount, 11 i64 captureTicks,
    19 i64 sendTicks, 27 i64 ticksPerSecond, 35 u16 width, 37 u16 height, 39 u16 payloadBytes, 41 payload

The viewer's keepalive: 0 u16 magic 0x5343, 2 u8 version, 3 u16 maxFramesPerSecond (0: the sender's rate), 5 i64 sendTicks.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

CHUNK_MAGIC = 0x5643
SUBSCRIBE_MAGIC = 0x5343
VERSION = 1
HEADER_BYTES = 41
MAX_DATAGRAM_BYTES = 1200
MAX_PAYLOAD_BYTES = MAX_DATAGRAM_BYTES - HEADER_BYTES
MAX_FRAME_BYTES = 0xFFFF * MAX_PAYLOAD_BYTES
SUBSCRIBE_BYTES = 13
UNSET = -1  # LatencyTrace.Unset: no capture stamp

_HEADER = struct.Struct("<HBIHHqqqHHH")
_SUBSCRIBE = struct.Struct("<HBHq")


@dataclass(frozen=True)
class FrameStamp:
    frame_id: int
    capture_ticks: int
    send_ticks: int
    ticks_per_second: int
    width: int
    height: int


def chunk_count(frame_bytes: int) -> int:
    return 0 if frame_bytes <= 0 else (frame_bytes + MAX_PAYLOAD_BYTES - 1) // MAX_PAYLOAD_BYTES


def encode_chunks(stamp: FrameStamp, jpeg: bytes) -> List[bytes]:
    """Every chunk of one frame, all with the same stamp (the reassembler depends on it)."""
    n = chunk_count(len(jpeg))
    if n == 0 or len(jpeg) > MAX_FRAME_BYTES or stamp.width <= 0 or stamp.height <= 0:
        raise ValueError("an empty or oversized frame, or no size")
    out = []
    for i in range(n):
        payload = jpeg[i * MAX_PAYLOAD_BYTES:(i + 1) * MAX_PAYLOAD_BYTES]
        out.append(_HEADER.pack(CHUNK_MAGIC, VERSION, stamp.frame_id & 0xFFFFFFFF, i, n, stamp.capture_ticks, stamp.send_ticks,
                                stamp.ticks_per_second, stamp.width, stamp.height, len(payload)) + payload)
    return out


def decode_chunk(d: bytes) -> Optional[Tuple[FrameStamp, int, int, bytes]]:
    """(stamp, chunk index, chunk count, payload), or None for anything that is not exactly a well-formed chunk."""
    if len(d) < HEADER_BYTES:
        return None
    magic, version, frame_id, index, count, capture, send, tps, width, height, size = _HEADER.unpack_from(d)
    last = index == count - 1
    if (magic != CHUNK_MAGIC or version != VERSION or count == 0 or index >= count or size == 0 or size > MAX_PAYLOAD_BYTES
            or len(d) != HEADER_BYTES + size or (not last and size != MAX_PAYLOAD_BYTES) or tps <= 0 or width == 0 or height == 0):
        return None
    return FrameStamp(frame_id, capture, send, tps, width, height), index, count, d[HEADER_BYTES:]


def encode_subscribe(max_fps: int, send_ticks: int) -> bytes:
    return _SUBSCRIBE.pack(SUBSCRIBE_MAGIC, VERSION, max_fps, send_ticks)


def decode_subscribe(d: bytes) -> Optional[Tuple[int, int]]:
    """(max frames per second, send ticks), or None."""
    if len(d) != SUBSCRIBE_BYTES:
        return None
    magic, version, fps, ticks = _SUBSCRIBE.unpack(d)
    return (fps, ticks) if magic == SUBSCRIBE_MAGIC and version == VERSION else None
