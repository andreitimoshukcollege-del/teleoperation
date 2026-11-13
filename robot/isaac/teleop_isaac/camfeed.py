"""One live camera feed on ADR 0014's wire (docs/adr/0032 decision 3): the sender side of ``Teleop.CameraHost``.

Copies of ``Streaming/SubscriberTracker.cs``, ``FramePacer.cs`` and ``CameraStreamer.cs``, with the socket. The pacer is
pinned by ``golden/camera-wire-v1.json``; the rules the viewer's reassembler depends on are kept as the C# keeps them:

- every chunk of a frame carries the same stamp, ``sendTicks`` included, read once before the first chunk;
- frame ids never restart below where they were: the first is milliseconds since boot, so a restarted sender starts
  ahead of every id it sent before, and every frame that starts sending consumes one, even one that fails partway.

The newest keepalive's sender is the subscriber, until none has come for the timeout: a second viewer takes the stream
over rather than sharing it. Ticks are CLOCK_MONOTONIC nanoseconds, the clock RobotHost reads on Linux.
"""

from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional, Tuple

from . import camwire

TICKS_PER_SECOND = 1_000_000_000


class FramePacer:
    """Thins frames to the viewer's requested rate, on capture time, with a tenth of a period of slack."""

    def __init__(self, ticks_per_second: int):
        self.tps = ticks_per_second
        self.has_sent = False
        self.last = 0

    def should_send(self, frame_ticks: int, max_fps: int) -> bool:
        if not self.has_sent or max_fps == 0:
            return True
        period = self.tps // max_fps
        return frame_ticks - self.last >= period - period // 10

    def mark_sent(self, frame_ticks: int) -> None:
        self.has_sent = True
        self.last = frame_ticks

    def reset(self) -> None:
        self.has_sent = False


class SubscriberTracker:
    def __init__(self, timeout_ticks: int):
        if timeout_ticks <= 0:
            raise ValueError("the timeout must be positive")
        self.timeout = timeout_ticks
        self.target = None
        self.last_seen = 0
        self.max_fps = 0

    def observe(self, sender, max_fps: int, now: int) -> bool:
        changed = self.target is None or self.target != sender or now - self.last_seen > self.timeout
        self.target, self.last_seen, self.max_fps = sender, now, max_fps
        return changed

    def active(self, now: int) -> Optional[Tuple[object, int]]:
        if self.target is None or now - self.last_seen > self.timeout:
            return None
        return self.target, self.max_fps


class CameraFeed:
    """A UDP port a viewer subscribes to; :meth:`offer` sends it frames. Thread-safe: the render loop polls keepalives,
    an encoder thread offers frames."""

    def __init__(self, name: str, port: int, subscriber_timeout_s: float = 3.0, clock: Callable[[], int] = time.monotonic_ns,
                 bind: str = "0.0.0.0"):
        self.name = name
        self.port = port
        self._clock = clock
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self._sock.bind((bind, port))
        self._sock.setblocking(False)
        self._subscribers = SubscriberTracker(int(subscriber_timeout_s * TICKS_PER_SECOND))
        self._pacer = FramePacer(TICKS_PER_SECOND)
        self._lock = threading.Lock()
        self.next_frame_id = (clock() // 1_000_000) & 0xFFFFFFFF
        self.frames_sent = self.frames_paced = self.frames_failed = self.frames_without_subscriber = 0
        self.bytes_sent = self.subscribes = self.ignored = 0
        self.subscriber_changes = []  # (ticks, address) each time the target changed

    def poll(self) -> None:
        """Takes every waiting keepalive; anything else is counted and ignored."""
        while True:
            try:
                data, sender = self._sock.recvfrom(64)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return  # an ICMP error from a viewer that went away
            sub = camwire.decode_subscribe(data)
            with self._lock:
                if sub is None:
                    self.ignored += 1
                    continue
                self.subscribes += 1
                if self._subscribers.observe(sender, sub[0], self._clock()):
                    self._pacer.reset()
                    self.subscriber_changes.append((self._clock(), sender))

    def wanted(self) -> bool:
        """Whether a frame offered now would go anywhere (so the render loop can skip encoding)."""
        with self._lock:
            return self._subscribers.active(self._clock()) is not None

    def offer(self, jpeg: bytes, capture_ticks: int, width: int, height: int) -> str:
        """``sent``, ``no-subscriber``, ``paced``, ``send-failed`` or ``unsendable``."""
        with self._lock:
            now = self._clock()
            active = self._subscribers.active(now)
            if active is None:
                self.frames_without_subscriber += 1
                return "no-subscriber"
            target, max_fps = active
            pace = now if capture_ticks == camwire.UNSET else capture_ticks
            if not self._pacer.should_send(pace, max_fps):
                self.frames_paced += 1
                return "paced"
            if camwire.chunk_count(len(jpeg)) == 0 or len(jpeg) > camwire.MAX_FRAME_BYTES or width <= 0 or height <= 0:
                self.frames_failed += 1
                return "unsendable"
            stamp = camwire.FrameStamp(self.next_frame_id, capture_ticks, self._clock(), TICKS_PER_SECOND, width, height)
            self.next_frame_id = (self.next_frame_id + 1) & 0xFFFFFFFF
            for chunk in camwire.encode_chunks(stamp, jpeg):
                try:
                    self._sock.sendto(chunk, target)
                except OSError:
                    self.frames_failed += 1
                    return "send-failed"
                self.bytes_sent += len(chunk)
            self._pacer.mark_sent(pace)
            self.frames_sent += 1
            return "sent"

    def close(self) -> None:
        self._sock.close()
