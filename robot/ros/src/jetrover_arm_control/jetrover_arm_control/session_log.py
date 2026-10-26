"""The driver's half of a recorded session (teleoperation docs/adr/0021 decision 3).

Writes ``driver.jlog`` into the session directory named by the control file (default
``~/recordings/.active``, which ``just record-start`` writes and ``just record-stop`` removes, and
RobotHost writes and removes for the sessions it opens itself, docs/adr/0023 decision 4), so
recording starts and stops without restarting this node. The format is the trainer's session
format v1 (trainer/contract/SESSION_FORMAT.md): a ``JSES`` header, one ``|``-separated record per
line, an ``END`` trailer with the record count, the drop count and a SHA-256.

**It must not change the timing it records.** The driver is single-threaded and its feedback cycle
is what is being measured, so ``record()`` only puts a tuple on a bounded queue (dropping and
counting when it is full, never blocking), and a background thread formats, hashes (``hashlib``,
in C, outside the GIL) and writes. A session with any drop fails ``just recording-check``.

**What is on disk is safe** (docs/adr/0023 decision 4):

- a log that exists is never truncated. When this node restarts during a session, the control file
  still names it and ``driver.jlog`` is already there (cut off, without a trailer): the writer
  continues in ``driver.2.jlog`` (then ``.3``, ...), a complete file of its own, and leaves the
  first as it was;
- an I/O error (a full disk) closes the session, with its trailer if the disk still takes it, says
  so on stderr, and keeps the thread alive: the session named in the control file is not reopened,
  but the next one is recorded.

Every timestamp is ``time.monotonic_ns()``: CLOCK_MONOTONIC, the same clock RobotHost's
``Stopwatch`` reads, so the two logs line up without a correspondence protocol.

Python 3.8 (ROS 2 Foxy on the Jetson).
"""

import hashlib
import os
import queue
import sys
import threading
import time

CONTROL_PATH_DEFAULT = os.path.expanduser("~/recordings/.active")
TICKS_PER_SECOND = 1000000000
CLOCK = "CLOCK_MONOTONIC"
_UNREADABLE = object()  # the control file could not be read this time: change nothing


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


def _say(message):
    # stderr, flushed: the service journal is where anyone will look (stdout is buffered there).
    sys.stderr.write("[session_log] %s\n" % message)
    sys.stderr.flush()


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
                 flush_seconds=1.0, open_file=None):
        self._control_path = control_path
        self._writer = writer
        self._queue = queue.Queue(maxsize=capacity)
        self._poll_seconds = poll_seconds
        self._flush_seconds = flush_seconds
        # Tests only: opens a new log file (it must fail if the file exists).
        self._open_file = open_file or (lambda path: open(path, "xb"))
        self._stop = threading.Event()
        self.active = False
        self.dropped = 0
        self.records = 0
        self.path = None
        self._file = None
        self._hash = None
        self._directory = None
        self._failed_directory = None
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
        except FileNotFoundError:
            return None
        except OSError:
            return _UNREADABLE

    def _new_log(self, directory):
        """The first of driver.jlog, driver.2.jlog, ... that does not exist yet: never truncates one."""
        n = 1
        while True:
            name = self._writer + (".jlog" if n == 1 else ".%d.jlog" % n)
            path = os.path.join(directory, name)
            try:
                return path, self._open_file(path)
            except FileExistsError:
                n += 1
                if n > 999:
                    raise

    def _open(self, directory):
        os.makedirs(directory, exist_ok=True)
        path, f = self._new_log(directory)
        if os.path.basename(path) != self._writer + ".jlog":
            _say("%s/%s.jlog already exists (the driver restarted during the session?): kept as it is; continuing in %s"
                 % (directory, self._writer, os.path.basename(path)))
        session_id = os.path.basename(directory.rstrip("/"))
        header = ("|".join(["JSES", "1", self._writer, str(TICKS_PER_SECOND), CLOCK, session_id]) + "\n").encode("ascii")
        try:
            f.write(header)
        except OSError:
            f.close()
            raise
        self._file = f
        self._hash = hashlib.sha256(header)
        self._directory = directory
        self.path = path
        self.records = 0
        self.dropped = 0
        self.active = True

    def _trailer_and_close(self):
        """The trailer, if the disk still takes it, then the file is let go. True if the trailer was written."""
        self.active = False
        trailer = ("END|%d|%d|%s\n" % (self.records, self.dropped, self._hash.hexdigest())).encode("ascii")
        written = True
        try:
            self._file.write(trailer)
            self._file.flush()
            os.fsync(self._file.fileno())
        except OSError:
            written = False
        try:
            self._file.close()
        except OSError:
            pass
        self._file = None
        self._directory = None
        return written

    def _close(self):
        if self._file is None:
            return
        self.active = False
        self._drain(block=False)
        if self._file is not None:  # the drain can fail and close it
            self._trailer_and_close()

    def _fail(self, error):
        """An I/O error: close the session cleanly, and do not reopen it while the control file names it."""
        if self._file is None:
            return
        directory = self._directory
        _say("write failed in %s: %s; closing the session" % (directory, error))
        written = self._trailer_and_close()
        _say("closed after the error: %d records, %s" % (self.records, "with its trailer" if written else "without a trailer (the disk refused it)"))
        self._failed_directory = directory

    def _emit(self, line):
        """Writes one line; on an I/O error closes the session and returns False. The hash takes the line
        first, so a file that lost bytes fails the trainer's check rather than passing silently."""
        data = (line + "\n").encode("ascii")
        self._hash.update(data)
        try:
            self._file.write(data)
            return True
        except OSError as e:
            self._fail(e)
            return False

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
                if self._emit("|".join([tag] + [_field(f) for f in fields])):
                    self.records += 1
                    wrote = True

    def _poll(self):
        target = self._target()
        if target is _UNREADABLE:
            return
        if target is None:
            self._failed_directory = None
        if target != self._directory:
            self._close()
            if target is not None and target != self._failed_directory:
                try:
                    self._open(target)
                except OSError as e:
                    _say("cannot record into %s: %s" % (target, e))
                    self._file = None
                    self._directory = None
                    self._failed_directory = target

    def _run(self):
        last_poll = 0.0
        last_flush = time.monotonic()
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                if now - last_poll >= self._poll_seconds:
                    last_poll = now
                    self._poll()
                self._drain(block=True)
                if self._file is not None and time.monotonic() - last_flush >= self._flush_seconds:
                    last_flush = time.monotonic()
                    try:
                        self._file.flush()
                    except OSError as e:
                        self._fail(e)
            except Exception as e:  # noqa: BLE001 -- nothing may end this thread: recording would stop silently for good
                _say("unexpected %s: %s" % (type(e).__name__, e))
                if self._file is not None:
                    self._fail(e)
                time.sleep(0.1)
        try:
            self._close()
        except Exception as e:  # noqa: BLE001
            _say("could not close the session on stop: %s" % e)


def from_environment():
    """The node's log, watching the control file (``JETROVER_SESSION_CONTROL``, default
    ``~/recordings/.active``). Always on: a thread polling one small file costs nothing, and a log
    that decided at start-up whether recording could ever happen would need a restart to change its
    mind, which is what the control file exists to avoid."""
    return SessionLog(control_path=os.environ.get("JETROVER_SESSION_CONTROL", CONTROL_PATH_DEFAULT))
