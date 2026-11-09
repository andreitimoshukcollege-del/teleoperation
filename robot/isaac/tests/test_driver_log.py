"""The stand-in driver's log (teleop_isaac/driver_log.py, docs/adr/0035 N4): driver.jlog as the real driver writes it, from
the bridge's emulated reads, cycles, commands and moves."""

import hashlib
import time

from teleop_isaac.driver_log import DriverLog
from teleop_isaac.isaac_session import IsaacSession


class Recorder:
    def __init__(self):
        self.rows = []

    def record(self, tag, *fields):
        self.rows.append((tag,) + fields)


def test_reads_end_when_the_bridge_samples_and_a_cycle_spans_its_reads():
    rec = Recorder()
    log = DriverLog(rec, read_seconds=0.0029, cycle_motors=[1, 2, 3])
    t = 1_000_000_000
    for k, m in enumerate((1, 2, 3)):
        log.read(t + k * 2_900_000, m, 500.4 + m)
    assert rec.rows[0] == ("RD", t - 2_900_000, t, 1, 501)
    assert rec.rows[1] == ("RD", t, t + 2_900_000, 2, 502)
    assert rec.rows[3] == ("CY", t - 2_900_000, t + 5_800_000)  # first read's start to last read's end
    log.read(t + 20_000_000, 1, 300.0)
    assert rec.rows[-1][0] == "RD" and sum(1 for r in rec.rows if r[0] == "CY") == 1  # a new cycle has begun


def test_commands_and_moves():
    rec = Recorder()
    log = DriverLog(rec, read_seconds=0.0029, cycle_motors=[1])
    log.command(5, [(1, 612.7, 907.18), (2, 400, 907.18)])
    log.moved(5, 1, 612.7, 500.0, 907.18 * 1.5625)
    assert rec.rows[0] == ("CI", 5, [(1, 612, 907.18), (2, 400, 907.18)])
    assert rec.rows[1] == ("WR", 5, int(112.7 / (907.18 * 1.5625) * 1000), [(1, 613)])
    log.command(6, [])
    assert len(rec.rows) == 2  # an empty array writes nothing


def test_it_writes_driver_jlog_into_the_open_session_with_the_real_drivers_header_and_trailer(tmp_path):
    control = tmp_path / ".active"
    session = IsaacSession(str(control), [], writer="driver", poll_seconds=0.02, free_bytes=lambda d: 10 ** 12)
    d = tmp_path / "20261109T230000Z-auto"
    d.mkdir()
    try:
        control.write_text(str(d))
        end = time.monotonic() + 5
        while not session.active and time.monotonic() < end:
            time.sleep(0.02)
        log = DriverLog(session, read_seconds=0.0029, cycle_motors=[1, 2])
        log.read(10_000_000, 1, 500)
        log.read(12_900_000, 2, 501)
        log.command(13_000_000, [(1, 600, 907.0)])
        time.sleep(0.2)
        control.unlink()
        end = time.monotonic() + 5
        while session.active and time.monotonic() < end:
            time.sleep(0.02)
    finally:
        session.close()
    lines = (d / "driver.jlog").read_text(encoding="ascii").splitlines()
    assert lines[0] == f"JSES|1|driver|1000000000|CLOCK_MONOTONIC|{d.name}"
    assert lines[1:5] == ["RD|7100000|10000000|1|500", "RD|10000000|12900000|2|501", "CY|7100000|12900000", "CI|13000000|1:600:907.0"]
    body = "".join(l + "\n" for l in lines[:-1]).encode("ascii")
    assert lines[-1] == f"END|4|0|{hashlib.sha256(body).hexdigest()}"
