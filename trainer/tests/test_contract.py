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
