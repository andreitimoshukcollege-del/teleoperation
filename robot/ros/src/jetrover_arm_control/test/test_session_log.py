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
        control.unlink()
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
        control.unlink()
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
