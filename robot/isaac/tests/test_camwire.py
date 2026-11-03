"""The renderer's copy of ADR 0014's camera wire against the C# (golden/camera-wire-v1.json, CameraWireGoldenTests), and one
feed end to end over loopback UDP."""

import socket
import time

from conftest import golden
from teleop_isaac import camfeed, camwire


def pattern(n):
    return bytes((7 * i + 3) & 0xFF for i in range(n))


def test_every_golden_frame_chunks_byte_for_byte():
    g = golden("camera-wire-v1.json")
    assert g["maxDatagramBytes"] == camwire.MAX_DATAGRAM_BYTES and g["headerBytes"] == camwire.HEADER_BYTES
    for f in g["frames"]:
        stamp = camwire.FrameStamp(f["frameId"], f["captureTicks"], f["sendTicks"], f["ticksPerSecond"], f["width"], f["height"])
        chunks = camwire.encode_chunks(stamp, pattern(f["length"]))
        assert [c.hex() for c in chunks] == f["chunks"]
        payload = b""
        for i, c in enumerate(chunks):
            s, index, count, part = camwire.decode_chunk(c)
            assert (s, index, count) == (stamp, i, len(chunks))
            payload += part
        assert payload == pattern(f["length"])


def test_every_golden_keepalive_encodes_and_decodes():
    for k in golden("camera-wire-v1.json")["keepalives"]:
        d = camwire.encode_subscribe(k["maxFramesPerSecond"], k["sendTicks"])
        assert d.hex() == k["hex"]
        assert camwire.decode_subscribe(d) == (k["maxFramesPerSecond"], k["sendTicks"])


def test_the_pacer_decides_as_the_csharp_does():
    g = golden("camera-wire-v1.json")["pacer"]
    pacer = camfeed.FramePacer(g["ticksPerSecond"])
    for ticks, fps, reset, send in g["steps"]:
        if reset:
            pacer.reset()
        got = pacer.should_send(ticks, fps)
        if got:
            pacer.mark_sent(ticks)
        assert got == send, (ticks, fps)


def test_malformed_chunks_are_refused():
    stamp = camwire.FrameStamp(1, 2, 3, 1_000_000_000, 640, 360)
    first, last = camwire.encode_chunks(stamp, pattern(1500))
    assert camwire.decode_chunk(first[:-1]) is None  # a short non-final chunk
    assert camwire.decode_chunk(b"\x00" + last[1:]) is None  # wrong magic
    assert camwire.decode_chunk(camwire.encode_chunks(camwire.FrameStamp(1, 2, 3, 0, 640, 360), b"x")[0]) is None  # no clock rate
    assert camwire.decode_subscribe(b"\x43\x53\x01" + b"\x00" * 9) is None  # 12 bytes


def test_a_feed_streams_to_its_newest_subscriber_with_one_stamp_per_frame():
    feed = camfeed.CameraFeed("test", 0, subscriber_timeout_s=1.0, bind="127.0.0.1")
    port = feed._sock.getsockname()[1]
    viewer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    viewer.bind(("127.0.0.1", 0))
    viewer.settimeout(1.0)
    try:
        assert feed.offer(b"\xff\xd8x", 5, 640, 360) == "no-subscriber"
        viewer.sendto(camwire.encode_subscribe(0, 1), ("127.0.0.1", port))
        viewer.sendto(b"junk", ("127.0.0.1", port))
        time.sleep(0.05)
        feed.poll()
        assert feed.subscribes == 1 and feed.ignored == 1 and feed.wanted()
        first_id = feed.next_frame_id
        jpeg = pattern(3000)
        assert feed.offer(jpeg, 123, 640, 360) == "sent"
        got = [camwire.decode_chunk(viewer.recv(2048)) for _ in range(camwire.chunk_count(len(jpeg)))]
        assert len({c[0] for c in got}) == 1  # one stamp, sendTicks included
        stamp = got[0][0]
        assert (stamp.frame_id, stamp.capture_ticks, stamp.width, stamp.ticks_per_second) == (first_id, 123, 640, 1_000_000_000)
        assert b"".join(c[3] for c in sorted(got, key=lambda c: c[1])) == jpeg
        assert feed.next_frame_id == (first_id + 1) & 0xFFFFFFFF
    finally:
        viewer.close()
        feed.close()
