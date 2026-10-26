"""SessionLog (teleoperation docs/adr/0021): driver.jlog is written while the control file names a
session directory, closed with a correct trailer when it goes away, and counts what it drops instead
of blocking the driver. The file must parse exactly as the trainer's session format v1 reads it."""

import hashlib
import os
import time

from jetrover_arm_control.session_log import NullSessionLog, SessionLog


def _wait(predicate, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _retry(action, seconds=3.0):
    """Windows refuses to replace or remove a file another thread has open for a moment (the writer
    polling the control file); Linux, where this runs for real, never does."""
    end = time.monotonic() + seconds
    while True:
        try:
            return action()
        except PermissionError:
            if time.monotonic() > end:
                raise
            time.sleep(0.01)


def _closed(path):
    if not os.path.exists(path):
        return False
    return open(path, "rb").read().rstrip(b"\n").split(b"\n")[-1].startswith(b"END|")


def _parse(path):
    data = open(path, "rb").read()
    lines = data.decode("ascii").split("\n")
    assert lines[-1] == ""
    trailer = lines[-2].split("|")
    body = data[: data.rfind(b"\n", 0, len(data) - 1) + 1]
    return lines[0].split("|"), [l.split("|") for l in lines[1:-2]], trailer, body


def test_records_only_while_the_control_file_names_a_session(tmp_path):
    control = tmp_path / ".active"
    session = tmp_path / "jetrover" / "20261021T150000Z-test"
    log = SessionLog(control_path=str(control), poll_seconds=0.02, flush_seconds=0.05)
    try:
        log.record("RD", 1, 2, 1, 500)  # not recording yet: dropped silently, not counted
        assert not os.path.exists(str(session))
        control.write_text(str(session) + "\n")
        assert _wait(lambda: log.active)
        log.record("RD", 10, 20, 1, 512)
        log.record("RD", 30, 40, 2, None)
        log.record("WR", 50, 50, [(1, 600), (2, 410)])
        log.record("CI", 45, [(1, 600, 597.0)])
        _retry(control.unlink)
        assert _wait(lambda: not log.active and os.path.exists(str(session / "driver.jlog"))
                     and open(str(session / "driver.jlog"), "rb").read().rstrip(b"\n").split(b"\n")[-1].startswith(b"END|"))
    finally:
        log.close()
    header, records, trailer, body = _parse(str(session / "driver.jlog"))
    assert header == ["JSES", "1", "driver", "1000000000", "CLOCK_MONOTONIC", "20261021T150000Z-test"]
    assert records == [["RD", "10", "20", "1", "512"], ["RD", "30", "40", "2", ""], ["WR", "50", "50", "1:600;2:410"],
                       ["CI", "45", "1:600:597.0"]]
    assert trailer[:3] == ["END", "4", "0"]
    assert trailer[3] == hashlib.sha256(body).hexdigest()


def test_a_full_queue_drops_and_counts_instead_of_blocking(tmp_path):
    control = tmp_path / ".active"
    session = tmp_path / "s"
    log = SessionLog(control_path=str(control), capacity=1, poll_seconds=0.02, flush_seconds=0.05)
    try:
        control.write_text(str(session))
        assert _wait(lambda: log.active)
        start = time.monotonic()
        for k in range(20000):
            log.record("RD", k, k, 1, 500)
        assert time.monotonic() - start < 2.0  # never waited on the writer
        dropped = log.dropped
        _retry(control.unlink)
        assert _wait(lambda: not log.active)
        time.sleep(0.1)
    finally:
        log.close()
    _, records, trailer, _ = _parse(str(session / "driver.jlog"))
    assert dropped > 0
    assert int(trailer[2]) >= dropped and int(trailer[1]) == len(records)


def test_the_null_log_costs_nothing():
    log = NullSessionLog()
    log.record("RD", 1, 2, 3, 4)
    assert not log.active and isinstance(log.now(), int)


def _wait_trailer(path, seconds=3.0):
    return _wait(lambda: _closed(path), seconds)


def test_a_log_that_exists_is_never_truncated(tmp_path):
    # The driver restarted while a session ran: the control file still names it.
    session = tmp_path / "20261026T100000Z-teleop"
    session.mkdir()
    before = b"JSES|1|driver|1000000000|CLOCK_MONOTONIC|20261026T100000Z-teleop\nRD|1|2|1|500\n"
    (session / "driver.jlog").write_bytes(before)
    control = tmp_path / ".active"
    control.write_text(str(session) + "\n")
    log = SessionLog(control_path=str(control), poll_seconds=0.02, flush_seconds=0.05)
    try:
        assert _wait(lambda: log.active)
        log.record("RD", 3, 4, 1, 501)
        _retry(control.unlink)
        assert _wait_trailer(str(session / "driver.2.jlog"))
    finally:
        log.close()
    assert (session / "driver.jlog").read_bytes() == before
    header, records, trailer, body = _parse(str(session / "driver.2.jlog"))
    assert header[-1] == "20261026T100000Z-teleop"
    assert records == [["RD", "3", "4", "1", "501"]]
    assert trailer[:3] == ["END", "1", "0"] and trailer[3] == hashlib.sha256(body).hexdigest()


class _FullDisk:
    """A log file whose writes fail as a full disk's do, past a byte budget: once, or for good."""

    def __init__(self, f, budget, once):
        self._f, self._budget, self._once, self._written, self._failed = f, budget, once, 0, False

    def write(self, data):
        if self._written + len(data) > self._budget and not (self._once and self._failed):
            self._failed = True
            raise OSError(28, "No space left on device")
        self._written += len(data)
        return self._f.write(data)

    def flush(self):
        if self._failed and not self._once:
            raise OSError(28, "No space left on device")
        self._f.flush()

    def fileno(self):
        return self._f.fileno()

    def close(self):
        self._f.close()


def _fail_in(name, once):
    def open_file(path):
        f = open(path, "xb")
        return _FullDisk(f, 200, once) if os.path.basename(os.path.dirname(path)) == name else f
    return open_file


def test_an_io_error_closes_the_session_with_its_trailer_and_the_thread_keeps_recording(tmp_path, capsys):
    control = tmp_path / ".active"
    bad, good = tmp_path / "bad", tmp_path / "good"
    log = SessionLog(control_path=str(control), poll_seconds=0.02, flush_seconds=0.05, open_file=_fail_in("bad", once=True))
    try:
        control.write_text(str(bad))
        assert _wait(lambda: log.active)
        for k in range(50):
            log.record("RD", k, k, 1, 500)
        assert _wait(lambda: not log.active and _closed(str(bad / "driver.jlog")))
        time.sleep(0.2)
        assert not log.active  # not reopened while the control file still names it
        assert not os.path.exists(str(bad / "driver.2.jlog"))
        assert log._thread.is_alive()
        _retry(lambda: control.write_text(str(good)))
        assert _wait(lambda: log.active)
        log.record("RD", 1, 2, 1, 500)
        _retry(control.unlink)
        assert _wait_trailer(str(good / "driver.jlog"))
    finally:
        log.close()
    _, records, trailer, body = _parse(str(bad / "driver.jlog"))
    assert int(trailer[1]) == len(records)
    assert trailer[3] != hashlib.sha256(body).hexdigest()  # the refused line is in the hash: the check fails it
    _, records, trailer, body = _parse(str(good / "driver.jlog"))
    assert records == [["RD", "1", "2", "1", "500"]] and trailer[3] == hashlib.sha256(body).hexdigest()
    assert "No space left on device" in capsys.readouterr().err


def test_a_disk_that_refuses_even_the_trailer_does_not_stop_the_thread(tmp_path):
    control = tmp_path / ".active"
    bad, good = tmp_path / "bad", tmp_path / "good"
    log = SessionLog(control_path=str(control), poll_seconds=0.02, flush_seconds=0.05, open_file=_fail_in("bad", once=False))
    try:
        control.write_text(str(bad))
        assert _wait(lambda: log.active)
        for k in range(50):
            log.record("RD", k, k, 1, 500)
        assert _wait(lambda: not log.active and log._file is None)
        assert b"END|" not in (bad / "driver.jlog").read_bytes()
        _retry(lambda: control.write_text(str(good)))
        assert _wait(lambda: log.active)
        _retry(control.unlink)
        assert _wait_trailer(str(good / "driver.jlog"))
        assert log._thread.is_alive()
    finally:
        log.close()
