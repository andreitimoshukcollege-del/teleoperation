"""The Isaac renderer's half of a recorded session (docs/adr/0032 decision 6, trainer/contract/SESSION_FORMAT.md).

Follows RobotHost's control file (``~/isaacsim/recordings/.active``, the session directory RobotHost writes when it opens a
session and removes when it closes it), as the JetRover driver's ``session_log.py`` does. While a session is open it
writes into its directory:

- ``isaac.jlog``: a ``JSES`` header, one ``|``-separated record per line, then one ``FS`` line per sidecar (its name,
  size and sha256) and an ``END`` trailer with the record count, the drop count and the sha256 of every line before it;
- ``isaac/<camera>.<stream>.frames``: the sidecars. Each is ``TFRM``, a uint16 version (1) and a uint16 0, then records,
  each a uint32 little-endian length and that many bytes. A ``VF`` record says where each of a frame's records starts
  (the byte after its length) and how long it is.

**It never slows the render loop.** :meth:`record` and :meth:`frame` only put on bounded queues; when the frame queue is
full the frame is dropped and an ``FD`` record says so. A background thread writes, hashes and flushes.

**What is on disk is safe.** A log that exists is never truncated: a second writer in the same session continues in
``isaac.2.jlog`` and ``isaac/<camera>.<stream>.2.frames``. An I/O error, or less than ``min_free_bytes`` left on the disk,
closes the session with its trailer if the disk takes it, and the session named in the control file is not reopened.

Every timestamp is ``time.monotonic_ns()``: CLOCK_MONOTONIC, RobotHost's clock.
"""

from __future__ import annotations

import hashlib
import os
import queue
import shutil
import struct
import sys
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

TICKS_PER_SECOND = 1_000_000_000
CLOCK = "CLOCK_MONOTONIC"
SIDECAR_MAGIC = b"TFRM"
SIDECAR_VERSION = 1
SIDECAR_HEADER = SIDECAR_MAGIC + struct.pack("<HH", SIDECAR_VERSION, 0)
_LEN = struct.Struct("<I")
_UNREADABLE = object()


def field(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return ";".join(":".join(field(part) for part in item) if isinstance(item, (list, tuple)) else field(item) for item in value)
    return str(value)


def say(message: str) -> None:
    sys.stderr.write(f"[isaac-session] {message}\n")
    sys.stderr.flush()


class Sidecar:
    def __init__(self, path: str):
        self.path = path
        self.file = open(path, "xb")
        self.hash = hashlib.sha256()
        self.size = 0
        self._write(SIDECAR_HEADER)

    def _write(self, data: bytes) -> None:
        self.file.write(data)
        self.hash.update(data)
        self.size += len(data)

    def append(self, payload: bytes) -> Tuple[int, int]:
        """Writes one record; (offset of its payload, its length)."""
        self._write(_LEN.pack(len(payload)))
        offset = self.size
        self._write(payload)
        return offset, len(payload)

    def close(self) -> None:
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()


def read_sidecar(path: str) -> List[Tuple[int, bytes]]:
    """Every record of a sidecar: (payload offset, payload). For checks and tests."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != SIDECAR_MAGIC or struct.unpack_from("<H", data, 4)[0] != SIDECAR_VERSION:
        raise ValueError(f"{path} is not a TFRM sidecar, version {SIDECAR_VERSION}")
    out, at = [], len(SIDECAR_HEADER)
    while at < len(data):
        (n,) = _LEN.unpack_from(data, at)
        out.append((at + 4, data[at + 4:at + 4 + n]))
        at += 4 + n
    return out


class IsaacSession:
    """``streams``: the sidecars a frame may carry, as ``camera.stream`` names (``head.rgb``, ``head.depth``, ...).
    ``preamble()`` gives the records written right after each session's header; ``on_closed(directory)`` runs on the
    writer thread after each session's trailer."""

    def __init__(self, control_path: str, streams: Sequence[str], preamble: Callable[[], List[Sequence]] = lambda: [],
                 on_closed: Callable[[str], None] = lambda d: None, writer: str = "isaac", frame_capacity: int = 90,
                 record_capacity: int = 100_000, poll_seconds: float = 0.5, min_free_bytes: int = 20 * 1024 ** 3,
                 free_bytes: Callable[[str], int] = lambda d: shutil.disk_usage(d).free):
        self._control = control_path
        self._streams = list(streams)
        self._preamble = preamble
        self._on_closed = on_closed
        self._writer = writer
        self._frames: "queue.Queue" = queue.Queue(maxsize=frame_capacity)
        self._records: "queue.Queue" = queue.Queue(maxsize=record_capacity)
        self._poll_seconds = poll_seconds
        self._min_free = min_free_bytes
        self._free = free_bytes
        self._stop = threading.Event()
        self.active = False
        self.directory: Optional[str] = None
        self.records = self.dropped = self.frames_written = self.frames_dropped = 0
        self.closed_sessions: List[Tuple[str, int, int]] = []  # (directory, frames, dropped)
        self._log = None
        self._hash = None
        self._sidecars: Dict[str, Sidecar] = {}
        self._failed: Optional[str] = None
        self._thread = threading.Thread(target=self._run, name="isaac-session", daemon=True)
        self._thread.start()

    @staticmethod
    def now() -> int:
        return time.monotonic_ns()

    # --- the render loop's side: never blocks --------------------------------------------------------------------

    def record(self, tag: str, *fields) -> None:
        if not self.active:
            return
        try:
            self._records.put_nowait((tag, fields))
        except queue.Full:
            self.dropped += 1

    def frame(self, frame: int, state_sensed: int, flags: str, blobs: Dict[str, bytes]) -> bool:
        """Queues one frame's encoded streams; False (and an ``FD`` record) when the writer is behind."""
        if not self.active:
            return False
        try:
            self._frames.put_nowait((self.now(), frame, state_sensed, flags, blobs))
            return True
        except queue.Full:
            self.frames_dropped += 1
            self.record("FD", self.now(), frame, "writer-behind")
            return False

    def close(self, timeout: float = 30.0) -> None:
        """Stops the writer, closing any open session with its trailer."""
        self._stop.set()
        self._thread.join(timeout=timeout)

    # --- the writer thread ---------------------------------------------------------------------------------------

    def _target(self):
        try:
            with open(self._control, "r") as f:
                text = f.read().strip()
            return text or None
        except FileNotFoundError:
            return None
        except OSError:
            return _UNREADABLE

    def _open(self, directory: str) -> None:
        if self._free(directory) < self._min_free:
            raise OSError(f"less than {self._min_free / 1024 ** 3:.0f} GB free")
        os.makedirs(os.path.join(directory, "isaac"), exist_ok=True)
        n = 1
        while True:
            suffix = "" if n == 1 else f".{n}"
            path = os.path.join(directory, f"{self._writer}{suffix}.jlog")
            try:
                log = open(path, "xb")
                break
            except FileExistsError:
                n += 1
        if n > 1:
            say(f"{directory}: {self._writer}.jlog exists already (the renderer restarted in this session?); continuing in {os.path.basename(path)}")
        self._sidecars = {s: Sidecar(os.path.join(directory, "isaac", f"{s}{suffix}.frames")) for s in self._streams}
        self._log = log
        self._hash = hashlib.sha256()
        self.directory = directory
        self.records = self.dropped = self.frames_written = self.frames_dropped = 0
        session = os.path.basename(directory.rstrip("/"))
        self._line("|".join(["JSES", "1", self._writer, str(TICKS_PER_SECOND), CLOCK, session]))
        self.active = True
        for r in self._preamble():
            self._emit(r[0], r[1:])

    def _line(self, text: str) -> None:
        data = (text + "\n").encode("ascii")
        self._hash.update(data)
        self._log.write(data)

    def _emit(self, tag: str, fields) -> None:
        self._line("|".join([tag] + [field(f) for f in fields]))
        self.records += 1

    def _write_frame(self, item) -> None:
        t, frame, sensed, flags, blobs = item
        where = []
        for name, data in blobs.items():
            sidecar = self._sidecars.get(name)
            if sidecar is None:
                continue
            offset, length = sidecar.append(data)
            where.append((name, offset, length))
        self._emit("VF", (t, frame, sensed, flags, where))
        self.frames_written += 1

    def _drain(self) -> None:
        while True:
            try:
                tag, fields = self._records.get_nowait()
            except queue.Empty:
                break
            if self._log is not None:
                self._emit(tag, fields)
        while True:
            try:
                item = self._frames.get_nowait()
            except queue.Empty:
                break
            if self._log is not None:
                self._write_frame(item)

    def _finish(self, reason: str) -> None:
        """Every sidecar's FS line, then the trailer; the session is let go either way."""
        if self._log is None:
            return
        self.active = False
        directory = self.directory
        written = True
        try:
            self._drain()
            if reason:
                self._emit("ER", (self.now(), reason))
            for name, s in self._sidecars.items():
                s.close()
                self._emit("FS", (f"isaac/{os.path.basename(s.path)}", s.size, s.hash.hexdigest()))
            trailer = f"END|{self.records}|{self.dropped + self.frames_dropped}|{self._hash.hexdigest()}\n".encode("ascii")
            self._log.write(trailer)
            self._log.flush()
            os.fsync(self._log.fileno())
        except OSError as e:
            written = False
            say(f"{directory}: the trailer was not written ({e})")
        try:
            self._log.close()
        except OSError:
            pass
        self.closed_sessions.append((directory, self.frames_written, self.frames_dropped))
        self._log, self._sidecars, self.directory = None, {}, None
        if written:
            try:
                self._on_closed(directory)
            except Exception as e:  # noqa: BLE001 -- the caller's hook must not end recording
                say(f"after closing {directory}: {e}")

    def _poll(self) -> None:
        target = self._target()
        if target is _UNREADABLE:
            return
        if target is None:
            self._failed = None
        if target != self.directory:
            self._finish("")
            if target is not None and target != self._failed:
                try:
                    self._open(target)
                except OSError as e:
                    say(f"cannot record into {target}: {e}")
                    self._failed = target
                    self._log, self.directory = None, None
        elif self.directory is not None and self._free(self.directory) < self._min_free:
            self._failed = self.directory
            self._finish(f"less than {self._min_free / 1024 ** 3:.0f} GB free")
            say("recording stopped: the disk is nearly full")

    def _run(self) -> None:
        last_poll = 0.0
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                if now - last_poll >= self._poll_seconds:
                    last_poll = now
                    self._poll()
                if self._log is not None:
                    self._drain()
                    self._log.flush()
                time.sleep(0.01)
            except OSError as e:
                directory = self.directory
                self._failed = directory
                say(f"write failed in {directory}: {e}; closing the session")
                self._finish(f"write failed: {e}")
            except Exception as e:  # noqa: BLE001 -- nothing may end this thread silently
                say(f"unexpected: {e!r}")
        self._finish("")
