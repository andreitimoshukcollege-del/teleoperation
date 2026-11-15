"""probe.py's pure parts, with sample data: no /proc, no ssh, nothing signalled. Runs on Windows' Python too."""

import json
import os
import types

import pytest

import probe

SIM = "[18:22:58]   20/204 sessions (20s)"
STEP = ("[19:19:01] step  148000 loss 0.1584 val p50/p95/p99 0.154/2.197/6.900 deg (prior p95 6.269, hold-last 15.360); "
        "gripper p95 2.803 servo deg 2387.2s")
EVALUATING = "[19:19:37] evaluating 264 views (44 test sessions x 6 schedules)"
EVAL_DONE = "[20:26:25] evaluation done (4008s): runs/x/eval/report.md"


# -- training_progress ---------------------------------------------------------------------------------------------


def test_simulating_counts_sessions_and_extrapolates_the_rate():
    p = probe.training_progress(["[18:22:38] simulating 204 sessions (204 planned: 134 train / 26 val / 44 test), 10 min each", SIM],
                                150000, None)
    assert (p["phase"], p["done"], p["total"]) == ("simulating", 20, 204)
    assert p["eta_s"] == pytest.approx(20 / 20 * (204 - 20))


def test_simulating_before_the_first_count_has_no_estimate():
    p = probe.training_progress(["[18:22:38] simulating 96 FR3 sessions (64 train / 16 val / 16 test), 10 min each"], 150000, None)
    assert (p["phase"], p["done"], p["eta_s"]) == ("simulating", None, None)


def test_training_counts_steps_against_the_configs_total():
    p = probe.training_progress([SIM, STEP], 150000, None)
    assert (p["phase"], p["done"], p["total"]) == ("training", 148000, 150000)
    assert p["eta_s"] == pytest.approx(2387.2 / 148000 * 2000)


def test_training_without_a_known_total_has_no_estimate():
    p = probe.training_progress([STEP], None, None)
    assert (p["phase"], p["done"], p["total"], p["eta_s"]) == ("training", 148000, None, None)


def test_evaluating_estimates_from_the_last_evaluations_rate():
    rate = 4008 / 264
    p = probe.training_progress([SIM, STEP, EVALUATING], 150000, rate, (19, 29, 37))  # 600 s into it
    assert (p["phase"], p["done"], p["total"]) == ("evaluating", None, 264)
    assert p["eta_s"] == pytest.approx(4008 - 600)
    assert "15.2 s a view" in p["note"]


def test_evaluating_across_midnight_and_overrunning_its_estimate():
    line = "[23:59:00] evaluating 10 views (2 test sessions x 5 schedules)"
    assert probe.training_progress([line], 1, 30.0, (0, 1, 0))["eta_s"] == pytest.approx(300 - 120)  # 120 s in, past midnight
    assert probe.training_progress([line], 1, 6.0, (0, 1, 0))["eta_s"] == 0.0  # 120 s of an estimated 60: none left, not -60


def test_evaluating_without_an_earlier_evaluation_says_so():
    p = probe.training_progress([EVALUATING], 150000, None, (19, 20, 0))
    assert p["eta_s"] is None and "no earlier evaluation" in p["note"]


def test_done_after_the_evaluation():
    p = probe.training_progress([SIM, STEP, "[19:19:36] exported model.onnx (sha256 8071b2cecab5...)", EVALUATING, EVAL_DONE],
                                150000, 15.0, (20, 30, 0))
    assert (p["phase"], p["eta_s"], p["done"], p["total"]) == ("done", 0.0, None, None)


def test_eval_rate_comes_from_the_newest_log_with_a_finished_evaluation(tmp_path):
    older = tmp_path / "train-sim-a.log"
    older.write_text("\n".join(["run: trainer/runs/a", EVALUATING, EVAL_DONE]) + "\n", encoding="utf-8")
    newer = tmp_path / "train-sim-b.log"
    newer.write_text("\n".join(["run: trainer/runs/b", SIM, STEP, EVALUATING]) + "\n", encoding="utf-8")  # still evaluating
    os.utime(older, (1000, 1000))
    os.utime(newer, (2000, 2000))
    assert probe.eval_rate([str(newer), str(older)]) == pytest.approx(4008 / 264)
    assert probe.eval_rate([str(newer)]) is None


# -- classify ------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("cmd, kind", [
    ("/home/andrei/isaacsim/venv/bin/python -m teleop_isaac.jetrover_bridge --socket /tmp/teleop-jetrover-relay-andrei.sock", "twin"),
    ("/home/andrei/isaacsim/venv/bin/python -m teleop_isaac.renderer --sim ../jetrover/urdf/jetrover.sim.json", "twin"),
    ("dotnet /home/andrei/isaacsim/robothost/Teleop.RobotHost.dll --local-port 6000 --relay-socket /tmp/x.sock", "twin"),
    ("/home/andrei/isaacsim/lerobot-venv/bin/python -m teleop_lerobot watch --root /home/andrei/data/takes/x", "watcher"),
    ("/home/andrei/isaacsim/lerobot-venv/bin/python -m teleop_lerobot safety-build --out /x/y /x/t1", "safety-build"),
    ("/home/andrei/isaacsim/lerobot-venv/bin/python -m teleop_lerobot safety-check --set-aside /x/t1", "safety-check"),
    ("bash trainer/train-sim.sh sim-pretrain-pilot1  --workers 24", "training"),
    ("/home/andrei/teleoperation/trainer/.venv/bin/python .venv/bin/armpred finetune --data x --out runs/y", "training"),
    ("/home/andrei/teleoperation/trainer/.venv/bin/python /home/andrei/teleoperation/trainer/.venv/bin/armpred sim-pretrain "
     "--config configs/sim-pretrain-pilot1.toml --out runs/sim-pretrain-pilot1-abc1234", "training"),
    # The fetch, not a training run: "-m armpred" would match it too (the bug before netfetch was checked first).
    ("/home/andrei/teleoperation/trainer/.venv/bin/python -m armpred.net.datasets fetch mahimahi", "netfetch"),
    ("/home/andrei/isaacsim/venv/bin/python -m teleop_isaac.simfit_run /home/andrei/analysis/simfit/sessions.json", "simfit"),
    ("/home/andrei/isaacsim/venv/bin/python /home/andrei/isaacsim/venv/bin/pip install isaacsim[all,extscache]==5.0.0", "isaac-setup"),
    # A shell's inline script names what it starts; it is not one of those processes itself.
    ("bash -c set -euo pipefail repo=$HOME/repos/isaac ... -m teleop_isaac.bridge ... Teleop.RobotHost.dll", "machinery"),
    ("bash -c unset PYTHONPATH; '/home/andrei/teleoperation/trainer/.venv/bin/python' -m armpred.net.datasets fetch ; x", "machinery"),
    ("/bin/sh -c bash /home/andrei/repos/lerobot/robot/lerobot/watch.sh keep >> /home/andrei/logs/lerobot/keeper.log", "machinery"),
    ("python3 -", "machinery"),
    ("python3 - act training STOP 4242", "machinery"),
    ("bash -s /home/andrei/data/takes/jetrover-isaac/reach_no_obstacle/20261115T152031Z-take009", "machinery"),
    ("sshd: andrei@notty", "machinery"),
    ("sleep 0.5", "machinery"),
    # Ours, and nothing the probe knows: listed under "other", not hidden as machinery.
    ("python3 -m http.server 9000", "other"),
    ("/usr/lib/openssh/sftp-server", "other"),
])
def test_classify(cmd, kind):
    assert probe.classify(cmd) == kind


def _proc(pid, cmd, pgid=None, started=1000.0, state="S"):
    return {"pid": pid, "ppid": 1, "pgid": pgid or pid, "state": state, "started": started, "cpu_s": 0.0, "rss": 1 << 20,
            "cmd": cmd, "kind": probe.classify(cmd)}


def test_the_twin_is_its_real_processes_and_robothost_on_the_relay_is_the_jetrovers(tmp_path):
    procs = [_proc(10, "bash -c set -euo pipefail ... -m teleop_isaac.jetrover_bridge ... Teleop.RobotHost.dll", started=900.0),
             _proc(12, "dotnet /home/andrei/isaacsim/robothost/Teleop.RobotHost.dll --relay-socket /tmp/r.sock", pgid=10)]
    p = {"isaac_logs": str(tmp_path), "sessions_jetrover": str(tmp_path), "sessions_fr3": str(tmp_path)}
    job = probe.job_twin(procs, p)
    assert job["title"] == "Isaac twin (JetRover)"
    assert job["pids"] == [12] and job["started"] == 1000.0
    assert job["detail"][0] == "JetRover twin: RobotHost (no renderer: no cameras)"


# -- toml_steps and run_steps --------------------------------------------------------------------------------------


def test_toml_steps_reads_the_train_table_only(tmp_path):
    cfg = tmp_path / "config.toml"
    cfg.write_text('[matrix]\nsteps = 5\nname = "a # b"\n\n[train]  # the training loop\narch = "mlp"\nwarmup_steps = 3\n'
                   'steps = 150_000  # about 40 min\n\n[ranges]\nsteps = 7\n', encoding="utf-8")
    assert probe.toml_steps(str(cfg)) == 150000


def test_toml_steps_without_a_train_steps(tmp_path):
    cfg = tmp_path / "config.toml"
    cfg.write_text("[train]\narch = \"mlp\"\n[sample]\nsteps = 9\n", encoding="utf-8")
    assert probe.toml_steps(str(cfg)) is None
    assert probe.toml_steps(str(tmp_path / "missing.toml")) is None


def test_run_steps_prefers_an_override(tmp_path):
    (tmp_path / "config.toml").write_text("[train]\nsteps = 150000\n", encoding="utf-8")
    cmd = "/x/.venv/bin/python /x/.venv/bin/armpred sim-pretrain --config c.toml --out runs/r"
    assert probe.run_steps(str(tmp_path), cmd) == 150000
    (tmp_path / "overrides.json").write_text(json.dumps(["train.arch=conv", "train.steps=2000"]), encoding="utf-8")
    assert probe.run_steps(str(tmp_path), cmd) == 2000
    assert probe.run_steps(str(tmp_path), cmd + " --set train.steps=500") == 500


# -- the safety build's progress -----------------------------------------------------------------------------------

SVT = ["Svt[info]: -------------------------------------------",
       "Svt[info]: SVT [config]: width / height / fps numerator / fps denominator 		: 640 / 360 / 15 / 1"]


def build_log_lines(episodes):
    lines = []
    for n in episodes:
        lines += SVT * 20 + [f"2026-11-15T16:18:{n % 60:02d}Z episode {n}: collide_with_obstacle, 194 frames, 6 unsafe "
                             f"(20261115T155441Z-take{n:03d})"]
    return lines


def test_episodes_done_is_the_highest_episode_plus_one():
    assert probe.episodes_done(build_log_lines(range(68))) == 68
    assert probe.episodes_done(build_log_lines(range(60, 68))) == 68  # a tail that lost the first 60
    assert probe.episodes_done(SVT) == 0
    assert probe.episodes_done([]) == 0


def _build(tmp_path, out_name, takes=10, started=1000.0):
    out = tmp_path / out_name
    paths = [str(tmp_path / "takes" / f"20261115T1500{i:02d}Z-take{i:03d}") for i in range(takes)]
    cmd = f"/home/andrei/isaacsim/lerobot-venv/bin/python -m teleop_lerobot safety-build --out {out} " + " ".join(paths)
    return [_proc(41, "bash -s " + " ".join(paths), pgid=40, started=started), _proc(42, cmd, pgid=40, started=started)]


def test_job_safety_build_from_its_log(tmp_path):
    (tmp_path / "jetrover_20261115_152024_53eebf.build-1.log").write_text("\n".join(build_log_lines(range(4))) + "\n", encoding="utf-8")
    job = probe.job_safety_build(_build(tmp_path, "jetrover_20261115_152024_53eebf"), {}, now=1100.0)
    assert job["progress"] == {"done": 4, "total": 10, "unit": "episodes"}
    assert job["eta_s"] == pytest.approx(100.0 / 4 * 6)
    assert job["pids"] == [42] and job["actions"] == [] and job["id"] == "safety-build-40"


def test_job_safety_build_retry_reads_its_names_second_log(tmp_path):
    first = tmp_path / "jetrover_X.build-1.log"
    first.write_text("\n".join(build_log_lines(range(7))) + "\nTraceback: FileNotFoundError\n", encoding="utf-8")
    second = tmp_path / "jetrover_X.build-2.log"
    second.write_text("\n".join(build_log_lines(range(3))) + "\n", encoding="utf-8")
    os.utime(first, (1000, 1000))
    os.utime(second, (2000, 2000))
    job = probe.job_safety_build(_build(tmp_path, "jetrover_X-retry"), {}, now=1100.0)
    assert job["progress"]["done"] == 3
    assert job["detail"][1].endswith("jetrover_X.build-2.log")


def test_job_safety_build_before_its_log_exists(tmp_path):
    job = probe.job_safety_build(_build(tmp_path, "jetrover_Y"), {}, now=1100.0)
    assert job["progress"] == {"done": 0, "total": 10, "unit": "episodes"} and job["eta_s"] is None
    assert job["detail"][1] == "no log yet"


# -- act's refusals ------------------------------------------------------------------------------------------------


@pytest.fixture
def no_signals(monkeypatch):
    """os.killpg recorded, not called (Windows has none), and the signal numbers Windows lacks."""
    sent = []
    monkeypatch.setattr(probe.os, "killpg", lambda pgid, sig: sent.append((pgid, sig)), raising=False)
    monkeypatch.setattr(probe, "signal", types.SimpleNamespace(SIGTERM=15, SIGSTOP=19, SIGCONT=18))
    return sent


@pytest.mark.parametrize("kind, sig, pgid", [
    ("twin", "TERM", 4242),       # not a kind act signals: the twin stops through just isaac-stop
    ("watcher", "STOP", 4242),
    ("safety-build", "TERM", 4242),
    ("training", "KILL", 4242),   # not a signal act sends
    ("training", "HUP", 4242),
    ("training", "term", 4242),
    ("training", "TERM", 1),      # killpg(1) would be kill(-1): everything this account may signal
    ("training", "TERM", 0),      # killpg(0): the probe's own group
    ("training", "TERM", -7),
])
def test_act_refuses_without_looking_or_sending(kind, sig, pgid, no_signals, monkeypatch):
    monkeypatch.setattr(probe, "processes", lambda *a, **k: pytest.fail("act looked at processes"))
    r = probe.act(kind, sig, pgid)
    assert r["ok"] is False and r["message"]
    assert no_signals == []


def test_act_refuses_a_group_that_is_not_that_job_alone(no_signals, monkeypatch):
    procs = [_proc(501, "/x/.venv/bin/python -m armpred.net.datasets fetch", pgid=500),
             _proc(601, "/x/.venv/bin/python /x/.venv/bin/armpred sim-pretrain --out runs/a", pgid=600),
             _proc(602, "/home/andrei/isaacsim/venv/bin/python -m teleop_isaac.renderer --sim x", pgid=600),
             _proc(701, "/x/.venv/bin/python /x/.venv/bin/armpred sim-pretrain --out runs/b", pgid=700)]
    monkeypatch.setattr(probe, "processes", lambda *a, **k: procs)
    monkeypatch.setattr(probe, "foreign_in_group", lambda pgid: pgid == 700)
    assert "no training job" in probe.act("training", "TERM", 500)["message"]      # a fetch, not a training run
    assert "also holds a twin" in probe.act("training", "TERM", 600)["message"]    # the twin is in that group
    assert "another account" in probe.act("training", "STOP", 700)["message"]
    assert "no training job" in probe.act("training", "TERM", 800)["message"]      # gone
    assert no_signals == []


def test_act_stops_a_paused_job_too(no_signals, monkeypatch):
    procs = [_proc(901, "/x/.venv/bin/python /x/.venv/bin/armpred sim-pretrain --out runs/a", pgid=900, state="T"),
             _proc(902, "/x/.venv/bin/python /x/.venv/bin/armpred sim-pretrain --out runs/a", pgid=900, state="T")]
    monkeypatch.setattr(probe, "processes", lambda *a, **k: procs)
    monkeypatch.setattr(probe, "foreign_in_group", lambda pgid: False)
    assert probe.act("training", "STOP", 900)["ok"] is True
    assert probe.act("training", "TERM", 900)["ok"] is True
    assert no_signals == [(900, 19), (900, 15), (900, 18)]  # STOP; then TERM and the CONT that lets a stopped run act on it


def test_main_refuses_malformed_act_arguments(monkeypatch):
    monkeypatch.setattr(probe, "processes", lambda *a, **k: pytest.fail("looked at processes"))
    assert probe.main(["act", "training", "TERM"])["ok"] is False
    assert "not a process group" in probe.main(["act", "training", "TERM", "12; rm -rf ~"])["message"]
