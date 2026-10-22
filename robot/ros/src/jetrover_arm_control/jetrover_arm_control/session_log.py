"""The driver's half of a recorded session (teleoperation docs/adr/0021 decision 3).

Writes ``driver.jlog`` into the session directory named by the control file (default
``~/recordings/.active``, which ``just record-start`` writes and ``just record-stop`` removes), so
recording starts and stops without restarting this node. The format is the trainer's session
format v1 (trainer/contract/SESSION_FORMAT.md): a ``JSES`` header, one ``|``-separated record per
line, an ``END`` trailer with the record count, the drop count and a SHA-256.

**It must not change the timing it records.** The driver is single-threaded and its feedback cycle
is what is being measured, so ``record()`` only puts a tuple on a bounded queue (dropping and
counting when it is full, never blocking), and a background thread formats, hashes (``hashlib``,
in C, outside the GIL) and writes. A session with any drop fails ``just recording-check``.

Every timestamp is ``time.monotonic_ns()``: CLOCK_MONOTONIC, the same clock RobotHost's
``Stopwatch`` reads, so the two logs line up without a correspondence protocol.

Python 3.8 (ROS 2 Foxy on the Jetson).
"""

import hashlib
import os
import queue
import threading
import time

CONTROL_PATH_DEFAULT = os.path.expanduser("~/recordings/.active")
TICKS_PER_SECOND = 1000000000
CLOCK = "CLOCK_MONOTONIC"


def _field(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, (list, tuple)):
        # A list field: items joined by ';', each item's parts by ':'.
        return ";".join(":".join(_field(part) for part in item) for item in value)
    return str(value)


class NullSessionLog:
    """What the controller holds when nothing records: every call is free."""

    active = False

    @staticmethod
    def now():
        return time.monotonic_ns()

    def record(self, tag, *fields):
        pass

    def close(self):
        pass


class SessionLog:
    def __init__(self, control_path=CONTROL_PATH_DEFAULT, writer="driver", capacity=200000, poll_seconds=0.5,
                 flush_seconds=1.0):
        self._control_path = control_path
        self._writer = writer
        self._queue = queue.Queue(maxsize=capacity)
        self._poll_seconds = poll_seconds
        self._flush_seconds = flush_seconds
        self._stop = threading.Event()
        self.active = False
        self.dropped = 0
        self.records = 0
        self._file = None
        self._hash = None
        self._directory = None
        self._thread = threading.Thread(target=self._run, name="session-log", daemon=True)
        self._thread.start()

    @staticmethod
    def now():
        return time.monotonic_ns()

    def record(self, tag, *fields):
        """Called from the driver thread: never blocks, never formats."""
        if not self.active:
            return
        try:
            self._queue.put_nowait((tag, fields))
        except queue.Full:
            self.dropped += 1

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5.0)

    # -- the writer thread -----------------------------------------------------------------

    def _target(self):
        try:
            with open(self._control_path, "r") as f:
                text = f.read().strip()
            return text or None
        except OSError:
            return None

    def _open(self, directory):
        os.makedirs(directory, exist_ok=True)
        self._file = open(os.path.join(directory, self._writer + ".jlog"), "wb")
        self._hash = hashlib.sha256()
        self._directory = directory
        self.records = 0
        self.dropped = 0
        session_id = os.path.basename(directory.rstrip("/"))
        self._emit("|".join(["JSES", "1", self._writer, str(TICKS_PER_SECOND), CLOCK, session_id]))
        self.active = True

    def _close(self):
        if self._file is None:
            return
        self.active = False
        self._drain(block=False)
        trailer = "END|%d|%d|%s\n" % (self.records, self.dropped, self._hash.hexdigest())
        self._file.write(trailer.encode("ascii"))
        self._file.flush()
        os.fsync(self._file.fileno())
        self._file.close()
        self._file = None
        self._directory = None

    def _emit(self, line):
        data = (line + "\n").encode("ascii")
        self._hash.update(data)
        self._file.write(data)

    def _drain(self, block):
        wrote = False
        timeout = 0.1 if block else 0.0
        while True:
            try:
                tag, fields = self._queue.get(block=block, timeout=timeout) if block else self._queue.get_nowait()
            except queue.Empty:
                return wrote
            block = False
            if self._file is not None:
                self._emit("|".join([tag] + [_field(f) for f in fields]))
                self.records += 1
                wrote = True

    def _run(self):
        last_poll = 0.0
        last_flush = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_poll >= self._poll_seconds:
                last_poll = now
                target = self._target()
                if target != self._directory:
                    self._close()
                    if target is not None:
                        try:
                            self._open(target)
                        except OSError:
                            self._file = None
                            self._directory = None
            self._drain(block=True)
            if self._file is not None and time.monotonic() - last_flush >= self._flush_seconds:
                self._file.flush()
                last_flush = time.monotonic()
        self._close()


def from_environment():
    """The node's log, watching the control file (``JETROVER_SESSION_CONTROL``, default
    ``~/recordings/.active``). Always on: a thread polling one small file costs nothing, and a log
    that decided at start-up whether recording could ever happen would need a restart to change its
    mind, which is what the control file exists to avoid."""
    return SessionLog(control_path=os.environ.get("JETROVER_SESSION_CONTROL", CONTROL_PATH_DEFAULT))
