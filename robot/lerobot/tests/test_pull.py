"""Copying the real robot's takes (docs/adr/0036 decision 7), against a stand-in for ssh and rsync: a folder plays the
robot's ~/recordings, and the listing is the one LISTING prints there."""

import os
import shutil
import subprocess

import pytest

from conftest import jetrover_take
from teleop_lerobot.pull import LISTING, RobotPuller, parse_listing
from teleop_lerobot.takes import checksum_id


class FakeRobot:
    """``recordings`` is the robot's ~/recordings. Answers at ``address`` only."""

    def __init__(self, recordings, address="192.168.8.158"):
        self.recordings, self.address, self.calls, self.fail_rsync = recordings, address, [], False

    def run(self, cmd, timeout):
        self.calls.append(cmd)
        if cmd[0] == "ssh":
            target = cmd[-2]
            if not target.endswith("@" + self.address):
                return 255, "ssh: connect to host: No route to host"
            return 0, self.listing()
        if cmd[0] == "rsync":
            if self.fail_rsync:
                return 23, "rsync error"
            src, dst = cmd[-2], cmd[-1]
            path = src.split(":", 1)[1].rstrip("/")
            shutil.copytree(os.path.join(os.path.dirname(self.recordings), path.replace("recordings/", os.path.basename(self.recordings) + "/", 1)),
                            dst.rstrip("/"), dirs_exist_ok=True)
            return 0, ""
        raise AssertionError(cmd)

    def listing(self):
        """What LISTING prints, worked out here rather than run (the test machine may have no bash)."""
        root = os.path.join(self.recordings, "scenarios")
        if not os.path.isdir(root):
            return "NOROOT\n"
        active = open(os.path.join(self.recordings, ".active")).read().strip() if os.path.exists(os.path.join(self.recordings, ".active")) else ""
        lines = [f"ACTIVE {active}"]
        for scenario in sorted(os.listdir(root)):
            if scenario.startswith("_"):
                continue
            for take in sorted(os.listdir(os.path.join(root, scenario))):
                if os.path.exists(os.path.join(root, scenario, take, "files.sha256")):
                    lines.append(f"KEPT {scenario}/{take} {checksum_id(os.path.join(root, scenario, take))}")
        away = os.path.join(root, "_discarded")
        if os.path.isdir(away):
            for scenario in sorted(os.listdir(away)):
                for take in sorted(os.listdir(os.path.join(away, scenario))):
                    lines.append(f"DISCARDED {scenario}/{take}")
        return "\n".join(lines) + "\n"


@pytest.fixture
def robot(tmp_path):
    recordings = tmp_path / "robot" / "recordings"
    (recordings / "scenarios").mkdir(parents=True)
    return FakeRobot(str(recordings))


def puller(tmp_path, robot, hosts=("10.0.0.9", "192.168.8.158")):
    hosts_file = tmp_path / "robot-hosts"
    hosts_file.write_text("\n".join(hosts) + "\n")
    logs = []
    p = RobotPuller(str(tmp_path / "scenarios-real"), hosts_file=str(hosts_file), run=robot.run, log=logs.append)
    return p, logs


def test_complete_takes_are_copied_once_and_land_only_after_they_check_out(tmp_path, robot):
    root = os.path.join(robot.recordings, "scenarios")
    jetrover_take(root, source="robot")
    jetrover_take(root, name="20261112T100500Z-take002", source="robot")
    p, logs = puller(tmp_path, robot)
    assert p.pass_once(force=True) == 2
    here = tmp_path / "scenarios-real" / "reach"
    assert sorted(os.listdir(here)) == ["20261112T100000Z-take001", "20261112T100500Z-take002"]
    assert not os.listdir(tmp_path / "scenarios-real" / ".incoming" / "reach")
    assert "robot: reached at 192.168.8.158" in logs  # the first address did not answer, the second did
    assert p.pass_once(force=True) == 0  # nothing new


def test_the_take_being_recorded_and_open_takes_are_never_copied(tmp_path, robot):
    root = os.path.join(robot.recordings, "scenarios")
    recording = jetrover_take(root, source="robot")
    open(os.path.join(robot.recordings, ".active"), "w").write(recording + "\n")
    unfinished = jetrover_take(root, name="20261112T100500Z-take002", source="robot")
    os.remove(os.path.join(unfinished, "files.sha256"))
    p, _ = puller(tmp_path, robot)
    assert p.pass_once(force=True) == 0
    assert not (tmp_path / "scenarios-real" / "reach").exists()


def test_a_copy_that_does_not_check_out_is_dropped_and_tried_again(tmp_path, robot):
    root = os.path.join(robot.recordings, "scenarios")
    take = jetrover_take(root, source="robot")
    with open(os.path.join(take, "robothost.jlog"), "ab") as f:
        f.write(b"!")  # the robot's copy no longer matches its files.sha256
    p, logs = puller(tmp_path, robot)
    assert p.pass_once(force=True) == 0
    assert any("did not check out" in line for line in logs)
    assert not (tmp_path / "scenarios-real" / "reach").exists()
    assert not os.listdir(tmp_path / "scenarios-real" / ".incoming" / "reach")


def test_a_take_set_aside_on_the_robot_is_set_aside_here(tmp_path, robot):
    root = os.path.join(robot.recordings, "scenarios")
    take = jetrover_take(root, source="robot")
    p, _ = puller(tmp_path, robot)
    p.pass_once(force=True)
    os.makedirs(os.path.join(root, "_discarded", "reach"))
    shutil.move(take, os.path.join(root, "_discarded", "reach"))
    p.pass_once(force=True)
    assert os.listdir(tmp_path / "scenarios-real" / "reach") == []
    assert os.listdir(tmp_path / "scenarios-real" / "_discarded" / "reach") == ["20261112T100000Z-take001"]
    assert p.pass_once(force=True) == 0  # and it is not copied back


def test_an_unreachable_robot_is_logged_once_and_passes_wait_their_turn(tmp_path, robot):
    p, logs = puller(tmp_path, robot, hosts=("10.0.0.9",))
    p.mdns = ""
    clock = {"t": 1000.0}
    p.clock = lambda: clock["t"]
    p.pass_once()
    p.pass_once()  # not due yet: no ssh at all
    clock["t"] += 61
    p.pass_once()
    assert len([c for c in robot.calls if c[0] == "ssh"]) == 2
    assert len([line for line in logs if "not reachable" in line]) == 1


def test_names_that_are_not_takes_never_become_paths():
    active, kept, discarded = parse_listing(
        "ACTIVE /home/jetson/recordings/scenarios/reach/20261112T100000Z-take001\n"
        "KEPT reach/20261112T100000Z-take001 " + "a" * 64 + "\n"
        "KEPT ../etc/20261112T100000Z-take001 " + "b" * 64 + "\n"
        "KEPT reach/notatake " + "c" * 64 + "\n"
        "DISCARDED reach/../../x\n")
    assert list(kept) == [("reach", "20261112T100000Z-take001")] and discarded == set()
    assert active.endswith("take001")


@pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="needs bash")
def test_the_listing_script_prints_what_the_puller_reads(robot, tmp_path):
    root = os.path.join(robot.recordings, "scenarios")
    jetrover_take(root, source="robot")
    os.makedirs(os.path.join(root, "_discarded", "reach", "20261112T090000Z-take000"))
    open(os.path.join(root, "_discarded", "reach", "20261112T090000Z-take000", "files.sha256"), "w").write("")
    env = dict(os.environ, HOME=os.path.dirname(robot.recordings))
    out = subprocess.run(["bash", "-c", LISTING], capture_output=True, text=True, env=env).stdout
    active, kept, discarded = parse_listing(out)
    assert active is None
    assert kept == {("reach", "20261112T100000Z-take001"): checksum_id(os.path.join(root, "reach", "20261112T100000Z-take001"))}
    assert discarded == {("reach", "20261112T090000Z-take000")}
