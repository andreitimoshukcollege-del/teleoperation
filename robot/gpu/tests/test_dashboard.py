"""dashboard.py with fakes for ssh and the recipes: nothing reaches the GPU machine and no recipe runs. One test serves the
page on 127.0.0.1 (an ephemeral port) to check the Host and token rules through real HTTP; nothing else opens a socket."""

import http.client
import json
import os
import threading

import pytest

import dashboard as d

NOW = 2_000_000.0
REMOTE = {
    "time": NOW - 2, "host": "seaqueue", "user": "andrei", "load": [3.2, 3.0, 2.9], "cores": 32,
    "disk": {"free": 300 << 30, "total": 1300 << 30}, "layout": "new", "other": [],
    "gpu": {"name": "NVIDIA GeForce RTX 5090", "util": 47, "mem_used": 8033, "mem_total": 32607, "temp": 56, "apps": []},
    "jobs": [
        {"id": "twin", "kind": "twin", "title": "Isaac twin (JetRover)", "status": "running", "actions": ["stop"]},
        {"id": "watcher", "kind": "watcher", "title": "LeRobot watcher", "status": "running", "actions": ["stop"]},
        {"id": "training-4242", "kind": "training", "pgid": 4242, "title": "Training: x", "status": "running",
         "actions": ["pause", "stop"]},
        {"id": "netfetch-77", "kind": "netfetch", "pgid": 77, "title": "Network datasets fetch", "status": "running",
         "actions": ["stop"]},
        {"id": "simfit-88", "kind": "simfit", "pgid": 88, "title": "Sim fit", "status": "running", "actions": ["stop"]},
        {"id": "safety-build-9", "kind": "safety-build", "title": "Safety dataset build", "status": "running", "actions": []},
        # Listed with an action by the machine, but not on the page's list: still refused.
        {"id": "rebuild", "kind": "rebuild", "title": "Dataset rebuild", "status": "running", "actions": ["stop"]},
        {"id": "training-1", "kind": "training", "pgid": 1, "title": "Training: bad", "status": "running", "actions": ["stop"]},
    ],
}


def batch_file(repo, **fields):
    data = {"recipe": "isaac-safety-batch", "pid": 31337, "name": "jetrover_20261115_160000_abcdef", "total": 40, "take": 10,
            "kept": 8, "setAside": 1, "unplanned": 1, "unchecked": 0, "started": NOW - 600, "updated": NOW, "phase": "takes",
            "dataset": None}
    data.update(fields)
    drive = os.path.join(str(repo), "recordings", ".drive")
    os.makedirs(drive, exist_ok=True)
    with open(os.path.join(drive, "safety-batch.json"), "w", encoding="utf-8") as f:
        json.dump(data, f)
    return data


@pytest.fixture
def dash(tmp_path):
    """A dashboard on tmp_path as its repo, with a fresh status, whose ssh and recipes are recorded, not run."""
    dsh = d.Dashboard("andrei@10.0.0.1", str(tmp_path), interval=5, port=8765, probe_source=b"# the probe",
                      clock=lambda: NOW, alive=lambda pid: pid == 31337)
    dsh.remote, dsh.remote_at = json.loads(json.dumps(REMOTE)), NOW - 2
    dsh.calls = []

    def ssh(remote, data, timeout):
        dsh.calls.append(("ssh", remote, timeout))
        return 0, json.dumps({"ok": True, "message": "SIGSTOP sent to process group 4242 (3 process(es))"}) + "\n", ""

    def recipe(name, timeout):
        dsh.calls.append(("just", name, timeout))
        return 0, "[isaac-stop] stopped\n"

    dsh.ssh, dsh.run_recipe = ssh, recipe
    dsh.spawn = lambda fn: fn()  # run each action at once, in the test's thread
    return dsh


def post(dsh, job, action, token=True):
    return dsh.handle_action(dsh.token if token is True else token, json.dumps({"job": job, "action": action}).encode())


# -- Host and token --------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("host, ok", [
    ("127.0.0.1:8765", True), ("localhost:8765", True), ("LOCALHOST:8765", True),
    ("127.0.0.1:8766", False), ("127.0.0.1", False), ("localhost", False), ("evil.example:8765", False),
    ("127.0.0.1:8765.evil.example", False), ("[::1]:8765", False), ("", False), (None, False),
])
def test_host_header(dash, host, ok):
    assert dash.host_ok(host) is ok


def test_a_post_without_the_token_runs_nothing(dash):
    for token in (None, "", "x" * 43, dash.token[:-1]):
        code, answer = post(dash, "twin", "stop", token=token)
        assert code == 403 and answer["ok"] is False
    assert dash.calls == [] and dash.actions_log == []


def test_a_body_that_is_not_json_is_refused(dash):
    assert dash.handle_action(dash.token, b"job=twin&action=stop")[0] == 400
    assert dash.handle_action(dash.token, b'["twin", "stop"]')[0] == 400
    assert dash.calls == []


# -- the whitelist ---------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("job, action", [
    ("nope", "stop"),            # not in the status
    ("twin", "pause"),           # not offered by that job
    ("twin", "start"),
    ("watcher", "start"),        # offered only while it is stopped
    ("training-4242", "resume"),  # offered only while it is paused
    ("netfetch-77", "pause"),
    ("safety-build-9", "stop"),  # offers nothing
    ("rebuild", "stop"),         # offered by the machine, but not on the page's list
    ("training-1", "stop"),      # process group 1 is never signalled
    ("twin", "rm -rf ~"),
    (["twin"], "stop"),
    ("twin", None),
])
def test_anything_off_the_list_is_refused(dash, job, action):
    code, answer = post(dash, job, action)
    assert code == 409 and answer["ok"] is False and answer["message"].startswith("refused")
    assert dash.calls == [] and dash.actions_log == []


def test_the_twin_stops_through_its_recipe(dash):
    code, answer = post(dash, "twin", "stop")
    assert code == 202 and answer == {"ok": True, "message": "started: just isaac-stop"}
    assert dash.calls == [("just", "isaac-stop", 300)]
    entry = dash.actions_log[-1]
    assert (entry["job"], entry["action"], entry["ok"], entry["time"]) == ("twin", "stop", True, NOW)
    assert "stopped" in entry["output"] and entry["finished"] == NOW
    assert dash.inflight == set()


def test_the_watcher_stops_and_starts_through_its_recipes(dash):
    assert post(dash, "watcher", "stop")[0] == 202
    dash.remote["jobs"][1].update(status="stopped", actions=["start"])
    assert post(dash, "watcher", "start")[0] == 202
    assert dash.calls == [("just", "lerobot-watch-stop", 720), ("just", "lerobot-watch-start", 300)]


@pytest.mark.parametrize("job, action, sig, pgid, kind", [
    ("training-4242", "pause", "STOP", 4242, "training"),
    ("training-4242", "stop", "TERM", 4242, "training"),
    ("netfetch-77", "stop", "TERM", 77, "netfetch"),
    ("simfit-88", "stop", "TERM", 88, "simfit"),
])
def test_signals_go_through_the_probes_act(dash, job, action, sig, pgid, kind):
    assert post(dash, job, action)[0] == 202
    assert dash.calls == [("ssh", ["python3", "-", "act", kind, sig, str(pgid)], d.ACT_TIMEOUT_S)]
    assert dash.actions_log[-1]["ok"] is True


def test_a_paused_run_offers_resume(dash):
    dash.remote["jobs"][2].update(status="paused", actions=["resume", "stop"])
    assert post(dash, "training-4242", "pause")[0] == 409
    assert post(dash, "training-4242", "resume")[0] == 202
    assert dash.calls[-1][1] == ["python3", "-", "act", "training", "CONT", "4242"]


def test_an_act_the_machine_refuses_is_logged_as_failed(dash):
    dash.ssh = lambda remote, data, timeout: (0, '{"ok": false, "message": "process group 4242 is no training job"}\n', "")
    assert post(dash, "training-4242", "stop")[0] == 202  # started; the outcome is in the log
    assert dash.actions_log[-1]["ok"] is False and "no training job" in dash.actions_log[-1]["output"]


def test_a_stale_status_allows_no_remote_action(dash):
    dash.remote_at = NOW - 3600
    code, answer = post(dash, "twin", "stop")
    assert code == 409 and "3600 s old" in answer["message"]
    dash.remote, dash.remote_at = None, None
    assert post(dash, "twin", "stop")[0] == 409
    assert dash.calls == []


def test_one_action_at_a_time_per_job(dash):
    started = []
    dash.spawn = started.append  # the first never finishes
    assert post(dash, "twin", "stop")[0] == 202
    code, answer = post(dash, "twin", "stop")
    assert code == 409 and "still running" in answer["message"]
    assert len(started) == 1 and dash.actions_log[-1]["ok"] is None


def test_status_shape(dash, tmp_path):
    batch_file(tmp_path)
    s = dash.status()
    assert set(s) >= {"remote", "remote_error", "remote_age_s", "batch", "actions_log", "host", "interval_s"}
    assert s["remote_age_s"] == 2.0 and s["remote_error"] is None and s["host"] == "andrei@10.0.0.1"
    assert s["batch"]["id"] == "batch-jetrover_20261115_160000_abcdef"


def test_a_failed_probe_keeps_the_last_answer(dash):
    dash.ssh = lambda remote, data, timeout: (255, "", "ssh: connect to host 10.0.0.1 port 22: Connection timed out\n")
    dash.poll_once()
    s = dash.status()
    assert s["remote"]["host"] == "seaqueue" and s["remote_age_s"] == 2.0
    assert "Connection timed out" in s["remote_error"]
    dash.ssh = lambda remote, data, timeout: (0, json.dumps(dict(REMOTE, host="again")), "")
    dash.poll_once()
    s = dash.status()
    assert s["remote"]["host"] == "again" and s["remote_error"] is None and s["remote_age_s"] == 0.0


def test_the_probe_runs_over_the_multiplexed_connection(dash):
    dash.control_dir = "/tmp/gpu-ui-x"
    argv = dash.ssh_argv(["python3", "-"])
    assert argv[:1] == ["ssh"] and argv[-3:] == ["andrei@10.0.0.1", "python3", "-"]
    for opt in ("ControlMaster=auto", "ControlPath=/tmp/gpu-ui-x/cm-%C", "ControlPersist=120", "BatchMode=yes", "ConnectTimeout=8"):
        assert argv[argv.index(opt) - 1] == "-o"


# -- the local safety batch ------------------------------------------------------------------------------------------


def job_of(data, now=NOW, alive=True, pause=False, stop=False):
    return d.batch_job(data, now, lambda pid: alive, pause, stop)


BATCH = {"pid": 31337, "name": "b1", "total": 40, "take": 10, "kept": 8, "setAside": 1, "unplanned": 1, "unchecked": 0,
         "started": NOW - 600, "updated": NOW, "phase": "takes", "dataset": None}


def test_a_batch_taking_takes(dash):
    job = job_of(BATCH)
    assert (job["id"], job["kind"], job["title"], job["status"]) == ("batch-b1", "batch", "Safety batch b1", "running")
    assert job["progress"] == {"done": 10, "total": 40, "unit": "takes"}
    assert job["actions"] == ["pause", "stop"]
    # 60 s a take for the 30 to come, then 13 s a kept take: 8 kept and 0.8 of the 30 to come.
    assert job["eta_s"] == pytest.approx(60 * 30 + 13 * (8 + 0.8 * 30))
    assert "60 s a take" in job["eta_note"]
    assert job["detail"][0] == "take 10 of 40: 8 kept, 1 set aside, 1 not planned"


def test_a_batch_asked_to_stop_counts_only_the_build(dash):
    job = job_of(BATCH, stop=True)
    assert job["eta_s"] == pytest.approx(13 * 8)
    assert job["actions"] == [] and any("stop asked for" in line for line in job["detail"])
    assert job_of(BATCH, stop=True, pause=True)["actions"] == ["resume"]


def test_a_batch_asked_to_pause_and_paused(dash):
    job = job_of(BATCH, pause=True)
    assert job["status"] == "running" and job["actions"] == ["resume", "stop"]
    assert any("pause asked for" in line for line in job["detail"])
    job = job_of(dict(BATCH, phase="paused"), pause=True)
    assert job["status"] == "paused" and job["actions"] == ["resume", "stop"] and job["eta_note"].startswith("once resumed")
    job = job_of(dict(BATCH, phase="paused"), pause=False)  # resume asked for, not yet seen by the recipe
    assert job["actions"] == ["pause", "stop"] and any("resume asked for" in line for line in job["detail"])


def test_a_batch_building_its_dataset(dash):
    job = job_of(dict(BATCH, phase="building", take=40, updated=NOW - 50, kept=30))
    assert job["status"] == "running" and job["actions"] == []
    assert job["eta_s"] == pytest.approx(13 * 30 - 50)
    assert job_of(dict(BATCH, phase="building", updated=NOW - 1000))["eta_s"] == 0.0


def test_a_finished_batch(dash):
    done = job_of(dict(BATCH, phase="done", take=40, dataset="/home/andrei/data/lerobot/jetrover-isaac-safety/b1"), alive=False)
    assert done["status"] == "done" and done["actions"] == [] and done["eta_s"] is None
    assert "dataset: /home/andrei/data/lerobot/jetrover-isaac-safety/b1" in done["detail"]
    failed = job_of(dict(BATCH, phase="failed"), alive=False)
    assert failed["status"] == "failed" and failed["actions"] == [] and "no dataset written" in failed["detail"]


def test_a_batch_whose_recipe_is_gone(dash):
    job = job_of(BATCH, alive=False)
    assert job["status"] == "stopped" and job["actions"] == [] and job["eta_s"] is None
    assert "not running any more" in job["detail"][-1]
    assert job_of(dict(BATCH, pid=None))["status"] == "stopped"


def test_a_batch_before_its_first_take(dash):
    job = job_of(dict(BATCH, take=0, kept=0, updated=NOW - 600))
    assert job["eta_s"] is None and job["eta_note"] == "no estimate before the first take"
    assert job_of(None) is None and job_of(["not", "a", "dict"]) is None


def test_the_batch_file_half_written_keeps_the_last_good_read(dash, tmp_path):
    assert dash.batch() is None
    batch_file(tmp_path, take=12)
    assert dash.batch()["progress"]["done"] == 12
    with open(os.path.join(str(tmp_path), "recordings", ".drive", "safety-batch.json"), "w", encoding="utf-8") as f:
        f.write('{"take": 1')
    assert dash.batch()["progress"]["done"] == 12


def test_batch_actions_write_and_remove_its_files(dash, tmp_path):
    batch_file(tmp_path)
    drive = os.path.join(str(tmp_path), "recordings", ".drive")
    job_id = "batch-jetrover_20261115_160000_abcdef"
    assert post(dash, job_id, "resume")[0] == 409  # not paused
    assert post(dash, job_id, "pause")[0] == 202
    assert os.path.exists(os.path.join(drive, "PAUSE")) and dash.batch()["actions"] == ["resume", "stop"]
    assert post(dash, job_id, "resume")[0] == 202
    assert not os.path.exists(os.path.join(drive, "PAUSE"))
    assert post(dash, job_id, "stop")[0] == 202
    assert os.path.exists(os.path.join(drive, "STOP")) and dash.batch()["actions"] == []
    assert post(dash, job_id, "stop")[0] == 409  # asked already
    assert dash.calls == [] and [e["ok"] for e in dash.actions_log] == [True, True, True]


def test_batch_actions_need_no_fresh_remote_status_but_a_live_recipe(dash, tmp_path):
    batch_file(tmp_path)
    dash.remote, dash.remote_at = None, None
    assert post(dash, "batch-jetrover_20261115_160000_abcdef", "pause")[0] == 202
    batch_file(tmp_path, pid=999)  # gone
    assert post(dash, "batch-jetrover_20261115_160000_abcdef", "stop")[0] == 409
    batch_file(tmp_path, name="another")  # a new batch since the page last looked
    assert post(dash, "batch-jetrover_20261115_160000_abcdef", "stop")[0] == 409


# -- through HTTP, on 127.0.0.1 ----------------------------------------------------------------------------------------


def test_http_host_and_token_rules(dash):
    server = d.Server(("127.0.0.1", 0), d.make_handler(dash))
    dash.port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    good = f"127.0.0.1:{dash.port}"

    def request(method, path, host=good, body=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", dash.port, timeout=5)
        conn.request(method, path, body=body, headers=dict({"Host": host}, **(headers or {})))
        r = conn.getresponse()
        data = r.read()
        conn.close()
        return r.status, r.getheader("Content-Security-Policy"), data

    try:
        code, policy, page = request("GET", "/")
        assert code == 200 and dash.token.encode() in page and b"__TOKEN__" not in page and b"__NONCE__" not in page
        assert "frame-ancestors 'none'" in policy and "script-src 'nonce-" in policy
        assert request("GET", "/", host=f"localhost:{dash.port}")[0] == 200
        assert request("GET", "/", host=f"evil.example:{dash.port}")[0] == 403
        assert request("GET", "/api/status", host="evil.example")[0] == 403
        code, _, status = request("GET", "/api/status")
        assert code == 200 and json.loads(status)["remote"]["host"] == "seaqueue"
        body = json.dumps({"job": "twin", "action": "stop"}).encode()
        assert request("POST", "/api/action", body=body)[0] == 403
        assert request("POST", "/api/action", body=body, headers={"X-Token": "wrong"})[0] == 403
        assert request("POST", "/api/action", host=f"evil.example:{dash.port}", body=body,
                       headers={"X-Token": dash.token})[0] == 403
        assert dash.calls == []
        code, _, answer = request("POST", "/api/action", body=json.dumps({"job": "nope", "action": "stop"}).encode(),
                                  headers={"X-Token": dash.token, "Content-Type": "application/json"})
        assert code == 409 and json.loads(answer)["ok"] is False
        code, _, answer = request("POST", "/api/action", body=body, headers={"X-Token": dash.token})
        assert code == 202 and dash.calls == [("just", "isaac-stop", 300)]
    finally:
        server.shutdown()
        server.server_close()
