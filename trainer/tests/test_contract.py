from __future__ import annotations

import os
import shutil

import pytest

from armpred.contract.check import check_session_dir
from armpred.contract.jlog import JlogError, JlogWriter, read_jlog, split_items, join_items


def test_a_jlog_round_trips(tmp_path):
    path = str(tmp_path / "x.jlog")
    with JlogWriter(path, "driver", 1_000_000_000, "CLOCK_MONOTONIC", "s1") as w:
        w.record("RD", 10, 20, 1, 512)
        w.record("RD", 30, 40, 2, None)
        w.record("WR", 50, 50, join_items([(1, 600), (2, 410)]))
    log = read_jlog(path, "driver")
    assert log.header.ticks_per_second == 1_000_000_000 and log.record_count == 3 and log.dropped == 0
    assert log.tag("RD") == [["10", "20", "1", "512"], ["30", "40", "2", ""]]
    assert split_items(log.tag("WR")[0][2]) == [["1", "600"], ["2", "410"]]


def test_an_edited_file_fails_its_checksum(tmp_path):
    path = str(tmp_path / "x.jlog")
    with JlogWriter(path, "driver", 1_000_000_000, "CLOCK_MONOTONIC", "s1") as w:
        w.record("RD", 10, 20, 1, 512)
    data = open(path, "rb").read().replace(b"|512", b"|513")
    open(path, "wb").write(data)
    with pytest.raises(JlogError, match="checksum"):
        read_jlog(path)


def test_a_file_without_its_trailer_is_rejected(tmp_path):
    path = str(tmp_path / "x.jlog")
    w = JlogWriter(path, "driver", 1_000_000_000, "CLOCK_MONOTONIC", "s1")
    w.record("RD", 10, 20, 1, 512)
    w._file.close()  # the writer died: no END line
    with pytest.raises(JlogError):
        read_jlog(path)


def test_a_simulated_session_passes_the_check(sim_session_dir):
    report = check_session_dir(sim_session_dir, allow_read_gaps=True)
    assert report.ok, report.lines()
    assert report.stats["commands"] > 1000 and report.stats["frames"] > 1000


def test_logs_from_two_different_sessions_fail_clock_causality(sim_session_dir, tmp_path):
    # Pair this session's RobotHost log with another session's driver log: values and times no
    # longer line up, which is exactly what the shared-clock check exists to catch.
    from armpred.sim.params import sample_params
    from armpred.sim.robot import simulate_session, write_session

    other = str(tmp_path / "other")
    write_session(simulate_session(sample_params(99, duration_s=25.0)), other)
    mixed = str(tmp_path / "mixed")
    shutil.copytree(sim_session_dir, mixed)
    shutil.copy(os.path.join(other, "driver.jlog"), os.path.join(mixed, "driver.jlog"))
    report = check_session_dir(mixed, allow_read_gaps=True)
    assert not report.ok
    assert any("causality" in f for f in report.failures) or any("session" in f for f in report.failures)


def test_an_automatic_session_with_unknown_conditions_is_read_and_checked(sim_session_dir, tmp_path):
    # What RobotHost writes for a session it opened itself (docs/adr/0023 decision 4): label and
    # motion "auto", load "unknown", the sender's address, and nulls for what nobody on the robot knows.
    import json

    from armpred.data.build import condition_tags
    from armpred.evaluate import condition_key

    auto = str(tmp_path / "20261026T120000Z-auto")
    shutil.copytree(sim_session_dir, auto)
    meta_path = os.path.join(auto, "session.json")
    with open(meta_path) as f:
        meta = json.load(f)
    meta.update({
        "id": "20261026T120000Z-auto", "source": "robot", "label": "auto", "stoppedUtc": "2026-10-26T12:30:00Z",
        "builds": {"robothost": {"sha": "abc", "dirty": False}, "driver": {"sha": "def", "dirty": False}},
    })
    meta["conditions"].update({
        "speedSliderRadPerSec": "unknown", "load": {"kind": "unknown", "payloadGrams": None},
        "network": {"uplinkProfile": "lan", "uplinkSeed": 0, "addedDelayMs": None},
        "motion": "auto", "operator": "auto 10.0.0.5:6002",
    })
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    report = check_session_dir(auto, allow_read_gaps=True)
    assert report.ok, report.lines()
    tags = condition_tags(meta)
    assert tags["slider"] == -1.0 and tags["payload_g"] == 0.0 and tags["hand"] == 0.0 and tags["added_delay_ms"] == 0.0
    assert condition_key(meta["conditions"]) == "slider=unknown load=unknown"
    meta["conditions"]["speedSliderRadPerSec"] = 2.5  # set from the commands' speeds on close
    meta["conditions"]["network"]["uplinkProfile"] = "varied"  # the robot-side uplink changed during it
    assert condition_tags(meta)["slider"] == 2.5 and condition_tags(meta)["uplink_impaired"] == 1.0


def _rewrite_with(path, extra):
    """Rewrites a .jlog with its own records and then `extra` (tag, *fields), re-signed."""
    with open(path, encoding="ascii") as f:
        lines = f.read().split("\n")[:-1]
    _, _, writer, tps, clock, session_id = lines[0].split("|")
    w = JlogWriter(path, writer, int(tps), clock, session_id)
    for line in lines[1:-1]:
        w._emit(line)
        w._records += 1
    for rec in extra:
        w.record(*rec)
    w.close()


def test_a_session_with_park_and_relax_records_is_read_and_checked(sim_session_dir, tmp_path):
    # docs/adr/0023 decisions 4 and 5: RobotHost's PQ, PS, TQ, PK and OM records, a joint command
    # ignored while parking (outcome P), and the driver's TQ. Readers ignore the tags they do not use.
    from armpred.contract.session import load_session

    d = str(tmp_path / "park")
    shutil.copytree(sim_session_dir, d)
    host = os.path.join(d, "robothost.jlog")
    jc = [line for line in open(host, encoding="ascii").read().split("\n") if line.startswith("JC|")]
    t = int(jc[-1].split("|")[2]) + 1
    _rewrite_with(host, [
        ("OM", t, "headset", "off", "10.0.0.5:6002"),
        ("PQ", t, 41, "relax", 123),
        ("PS", t, "parking", "none"),
        ("PK", t + 1, "1:0.5:0.5;2:1.25:0.5"),
        ("JC", t + 2, t + 2, 999999, 124, "P", "1:0:0"),
        ("PS", t + 3, "relaxed", "none"),
        ("TQ", t + 3, 0),
        ("PS", t + 4, "holding", "path-collides"),
    ])
    driver = os.path.join(d, "driver.jlog")
    rd = [line for line in open(driver, encoding="ascii").read().split("\n") if line.startswith("RD|")]
    _rewrite_with(driver, [("TQ", int(rd[-1].split("|")[2]) + 1, 0)])

    session = load_session(d)
    assert chr(session.commands.outcome[-1]) == "P"
    report = check_session_dir(d, allow_read_gaps=True)
    assert report.ok, report.lines()
    assert any("parking or relaxed" in w for w in report.warnings)


def test_a_session_with_course_labels_reads_its_rounds(sim_session_dir, tmp_path):
    # docs/adr/0024: RobotHost's TK records, the headset's training course. A new tag: the checker
    # passes the session as before, and the reader gives the rounds.
    from armpred.contract.session import load_session

    d = str(tmp_path / "course")
    shutil.copytree(sim_session_dir, d)
    host = os.path.join(d, "robothost.jlog")
    jc = [line for line in open(host, encoding="ascii").read().split("\n") if line.startswith("JC|")]
    t = int(jc[-1].split("|")[2]) + 1
    _rewrite_with(host, [
        ("TK", t, 77, 1, "reach", "active", "train", "2.5", "0", 0),
        ("TK", t + 1, 77, 1, "reach", "done", "train", "2.5", "8", 0),
        ("TK", t + 2, 77, 2, "trace", "active", "test", "varied", "0", 1),
    ])

    session = load_session(d)
    assert [(r.round, r.task, r.phase, r.purpose) for r in session.rounds] == [
        (1, "reach", "active", "train"), (1, "reach", "done", "train"), (2, "trace", "active", "test")]
    assert session.rounds[1].score == 8.0 and session.rounds[0].level == 2.5
    assert session.rounds[2].level is None and session.rounds[2].disturbed
    report = check_session_dir(d, allow_read_gaps=True)
    assert report.ok, report.lines()
