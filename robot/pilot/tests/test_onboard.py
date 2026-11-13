"""robot/pilot/onboard.py, the runner that records drive sessions on the Jetson by itself, without a robot: its
recordings folder is a temporary one, and a stand-in plays RobotHost (it completes an automatic session when the
control file goes, and the logs carry END trailers). onboard.py keeps its session handling the same as
`just record-start` / `just record-stop` by hand; these tests hold its own half still."""

import hashlib
import json
import os
import pytest

from conftest import PILOT
import onboard
import pilot


@pytest.fixture
def recordings(tmp_path, monkeypatch):
    root = tmp_path / "recordings"
    root.mkdir()
    for name, value in {"RECORDINGS": root, "ACTIVE": root / ".active", "ACTIVE_AUTO": root / ".active-auto",
                        "UPLINK": root / ".uplink", "STOP": root / ".pilot-stop", "HOME": tmp_path}.items():
        monkeypatch.setattr(onboard, name, str(value))
    monkeypatch.setattr(onboard.time, "sleep", lambda s: None)
    return root


def session(label="drive-1", slider="2.5", motion="drive:waypoints", options=()):
    return pilot.Session(label, slider, 5.0, motion, list(options), "")


def jlog(path, lines, end=True):
    with open(path, "w", newline="\n") as f:  # LF, as the robot writes them
        f.write("JSES|1|test|1000000000|CLOCK_MONOTONIC|x\n")
        for line in lines:
            f.write(line + "\n")
        if end:
            f.write("END|1|0\n")


def test_every_committed_plan_parses_and_its_drive_options_are_ones_a_drive_takes():
    for name in sorted(os.listdir(PILOT)):
        if name.startswith("pilot-") and name.endswith(".txt"):
            for s in pilot.read_plan(os.path.join(PILOT, name)):
                if s.motion.startswith("drive:"):
                    onboard.options(s)  # raises SystemExit on an option onboard.py would refuse on the robot


def test_session_json_records_the_plans_conditions():
    o = onboard.options(session(options=["load=payload", "payload_g=120", "network=schedule:bursty", "notes=test"]))
    meta = onboard.session_json("20261112T000000Z-drive-1", session(), o, "ab" * 32, "seed 7")
    assert meta["format"] == "jetrover-session" and meta["version"] == 1 and meta["source"] == "robot"
    c = meta["conditions"]
    assert c["speedSliderRadPerSec"] == 2.5
    assert c["load"] == {"kind": "payload", "payloadGrams": 120.0}
    assert c["network"] == {"uplinkProfile": "schedule", "uplinkSchedule": "bursty", "uplinkSeed": 1, "addedDelayMs": 0.0}
    assert c["motion"] == "joint-drive:waypoints" and c["operator"] == "joint-drive@robot"
    assert meta["notes"] == "joint-drive ran on the robot, to RobotHost over loopback (no Wi-Fi); seed 7; test"


def test_an_unknown_option_or_load_is_refused():
    with pytest.raises(SystemExit):
        onboard.options(session(options=["speed=3"]))
    with pytest.raises(SystemExit):
        onboard.options(session(options=["load=anvil"]))


def test_an_automatic_session_is_closed_first_and_the_control_file_taken(recordings, monkeypatch):
    auto = recordings / "auto" / "20261112T000000Z-auto"
    auto.mkdir(parents=True)
    (recordings / ".active").write_text(str(auto) + "\n")
    (recordings / ".active-auto").write_text(str(auto) + "\n")

    def robothost(seconds):  # while onboard.py waits: RobotHost completes the automatic session once its control file is gone
        if not (recordings / ".active").exists():
            (auto / "files.sha256").write_text("")

    monkeypatch.setattr(onboard.time, "sleep", robothost)
    manual = recordings / "jetrover" / "20261112T000100Z-drive-1"
    manual.mkdir(parents=True)
    onboard.take_control_file(str(manual))
    assert (auto / "files.sha256").exists()
    assert (recordings / ".active").read_text().strip() == str(manual)


def test_a_manual_session_is_never_taken_over(recordings):
    other = recordings / "jetrover" / "someone-else"
    other.mkdir(parents=True)
    (recordings / ".active").write_text(str(other) + "\n")
    with pytest.raises(SystemExit):
        onboard.take_control_file(str(recordings / "jetrover" / "mine"))
    assert (recordings / ".active").read_text().strip() == str(other)


def test_closing_writes_the_stop_time_the_measured_slider_and_every_files_hash(recordings):
    d = recordings / "jetrover" / "20261112T000100Z-drive-1"
    d.mkdir(parents=True)
    o = onboard.options(session())
    (d / "session.json").write_text(json.dumps(onboard.session_json(d.name, session(slider="2.5"), o, "00" * 32)))
    (d / "robot-profile.json").write_text("{}")
    jlog(d / "robothost.jlog", ["JC|1|2|3|4|5|1:500:3.8;2:400:3.8;10:116:0"])
    jlog(d / "driver.jlog", ["BV|10|11800"])
    (recordings / ".active").write_text(str(d) + "\n")
    (recordings / ".uplink").write_text("50ms-5j 1\n")

    onboard.close_session(str(d))

    assert not (recordings / ".active").exists() and not (recordings / ".uplink").exists()
    meta = json.loads((d / "session.json").read_text())
    assert meta["stoppedUtc"].endswith("Z")
    assert meta["conditions"]["speedSliderRadPerSec"] == 3.8  # what the commands carried, the gripper's 0 aside
    assert "the plan said 2.5" in meta["notes"]
    listed = dict(reversed(line.split("  ", 1)) for line in (d / "files.sha256").read_text().splitlines())
    assert sorted(listed) == ["driver.jlog", "robot-profile.json", "robothost.jlog", "session.json"]
    for name, digest in listed.items():
        assert hashlib.sha256((d / name).read_bytes()).hexdigest() == digest
    assert onboard.battery_mv(str(d)) == 11800
    assert onboard.trailer(str(d / "driver.jlog")) == "END|1|0"


def test_varied_commands_record_a_varied_slider(recordings):
    d = recordings / "s"
    d.mkdir()
    jlog(d / "robothost.jlog", ["JC|1|2|3|4|5|1:500:1.0", "JC|1|2|3|4|5|1:500:6.0"])
    meta = {"conditions": {"speedSliderRadPerSec": "varied"}, "notes": ""}
    onboard.slider_from_commands(str(d), meta)
    assert meta["conditions"]["speedSliderRadPerSec"] == "varied" and meta["notes"] == ""


def test_a_course_sessions_seed_comes_from_its_label_and_a_variant_changes_it():
    course = session(label="course-1", slider="varied", motion="drive:course")
    assert pilot.drive_seed(course) == pilot.drive_seed(course)
    assert pilot.drive_seed(course) != pilot.drive_seed(session(label="course-2", slider="varied", motion="drive:course"))
    assert pilot.drive_seed(course, "run-a") != pilot.drive_seed(course, "run-b")
    assert pilot.drive_seed(session()) == "1"
    assert pilot.drive_speed(course) == "6" and pilot.drive_speed(session()) == "2.5"
