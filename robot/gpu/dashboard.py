#!/usr/bin/env python3
"""The GPU machine at a glance: a local page of what runs in our account there, how far along each job is and about how
long is left, with buttons for the few things it may stop, pause or start. `just gpu-ui` opens it.

    python3 robot/gpu/dashboard.py --host andrei@10.188.58.133 --port 8765 --repo <repo root> [--interval 5] [--open]

Every --interval seconds it pipes probe.py to the machine over one multiplexed ssh connection and keeps the answer; a
probe that fails keeps the last good answer, with its age and the error. It also reads
<repo>/recordings/.drive/safety-batch.json, the progress file of a local `just isaac-safety-batch`, and
safety-batch-<N>.json, that of a batch on twin N (docs/adr/0037 amendment 7). Standard library only: it runs on WSL's
python3.

The page is served on 127.0.0.1 only, and only to requests whose Host is 127.0.0.1:<port> or localhost:<port> (so no
other site reaches it by DNS rebinding). A POST needs the random token the page was served with (so no other page can
press its buttons). An action runs only if the job is in the latest status and offers it, and only from the three tables
below: nothing else is ever run.
"""

from __future__ import annotations

import argparse
import hmac
import json
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Dict, List, Optional, Set, Tuple
from urllib.parse import urlsplit

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE_TIMEOUT_S = 25
ACT_TIMEOUT_S = 30
# The safety dataset build, per kept take (2026-11-15: 68 episodes in 14.5 min).
BUILD_SECONDS_PER_TAKE = 13.0
LOG_KEEP = 50
TAIL_CHARS = 2000
MAX_BODY = 4096
HOST_RE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9._-]*@[A-Za-z0-9._-]+|[A-Za-z0-9_][A-Za-z0-9._-]*")

# The whitelist, by (job kind, action). A local recipe, run in --repo with GPU_HOST set to --host, and its timeout (a
# twin's stop names the twin when it is not twin 0: `just isaac-stop 1`):
JUST_ACTIONS: Dict[Tuple[str, str], Tuple[str, int]] = {
    ("twin", "stop"): ("isaac-stop", 300),
    ("watcher", "stop"): ("lerobot-watch-stop", 720),  # watch.sh waits up to 10 min for the take it is writing
    ("watcher", "start"): ("lerobot-watch-start", 300),
}
# A signal to the job's process group, by the probe's `act` on the machine (which checks the group again there):
SIGNALS: Dict[Tuple[str, str], str] = {
    ("training", "pause"): "STOP",
    ("training", "resume"): "CONT",
    ("training", "stop"): "TERM",
    ("netfetch", "stop"): "TERM",
    ("simfit", "stop"): "TERM",
}
# The local safety batch's control files, in <repo>/recordings/.drive (the recipe reads them between takes); twin N's
# batch has its own, suffixed -N (PAUSE-1, STOP-1):
BATCH_ACTIONS: Dict[str, str] = {
    "pause": "write recordings/.drive/PAUSE",
    "resume": "remove recordings/.drive/PAUSE",
    "stop": "write recordings/.drive/STOP",
}
BATCH_FILE = re.compile(r"^safety-batch(?:-([1-9]))?\.json$")


class Refused(Exception):
    """An action this page will not run, with the reason the page shows."""


def tail_text(text: str, n: int = TAIL_CHARS) -> str:
    text = (text or "").strip()
    return text if len(text) <= n else "..." + text[-n:]


def last_json(text: str) -> Optional[dict]:
    """The last line of ``text`` that is a JSON object (the probe prints one; ssh may print warnings around it)."""
    for line in reversed((text or "").strip().splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except ValueError:
                return None
            return value if isinstance(value, dict) else None
    return None


# ---------------------------------------------------------------------------------------------------------------------
# The local safety batch


def batch_paths(repo: str, twin: int = 0) -> Dict[str, str]:
    """A batch's progress file and control files: twin 0's as they always were, twin N's suffixed -N."""
    d = os.path.join(repo, "recordings", ".drive")
    sfx = "" if twin == 0 else f"-{twin}"
    return {"file": os.path.join(d, f"safety-batch{sfx}.json"), "pause": os.path.join(d, f"PAUSE{sfx}"),
            "stop": os.path.join(d, f"STOP{sfx}")}


def batch_twins(repo: str) -> List[int]:
    """The twins with a batch progress file in <repo>/recordings/.drive, twin 0 first."""
    try:
        names = os.listdir(os.path.join(repo, "recordings", ".drive"))
    except OSError:
        return []
    return sorted(int(m.group(1) or 0) for m in map(BATCH_FILE.match, names) if m)


def pid_alive(pid: int) -> bool:
    """Whether local process ``pid`` exists (the batch recipe's bash: a WSL pid)."""
    if os.name != "posix":  # on Windows os.kill(pid, 0) would end the process, and a WSL pid means nothing there
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:  # it exists, as someone else's
        return True
    except OSError:
        return False
    return True


def _num(v) -> Optional[float]:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _count(v) -> int:
    return int(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else 0


def _span(seconds: float) -> str:
    return f"{seconds:.0f} s" if seconds < 120 else f"{seconds / 60:.1f} min"


def batch_eta(phase: str, started: Optional[float], updated: Optional[float], take: int, total: int, kept: int,
              now: float, stopping: bool) -> Tuple[Optional[float], str]:
    """About how long a batch has left (seconds, None when nothing tells) and what that is made of.

    While taking: the takes to come at the rate so far, (updated - started) / take each, then the dataset build at about
    13 s a kept take, counting the takes to come as kept at the rate so far (none to come once a stop is asked for).
    While building: what is left of the build, counted from the file's last update."""
    build = f"about {BUILD_SECONDS_PER_TAKE:.0f} s a kept take for the dataset build"
    if phase in ("takes", "paused"):
        if take <= 0 or started is None or updated is None or updated <= started:
            return None, "no estimate before the first take"
        per_take = (updated - started) / take
        remaining = 0 if stopping else max(0, total - take)
        expected_kept = kept + kept / take * remaining
        eta = per_take * remaining + BUILD_SECONDS_PER_TAKE * expected_kept
        note = f"{_span(per_take)} a take, then {build}"
        return eta, ("once resumed: " + note) if phase == "paused" else note
    if phase == "building" and updated is not None:
        return max(0.0, BUILD_SECONDS_PER_TAKE * kept - (now - updated)), build
    return None, ""


PHASE_TEXT = {
    "takes": "recording takes",
    "paused": "paused: it waits before its next take",
    "building": "building the dataset from the kept takes (on the GPU machine)",
}


def batch_job(data: Optional[dict], now: float, alive: Callable[[int], bool], pause_file: bool,
              stop_file: bool, twin: int = 0) -> Optional[dict]:
    """The local safety batch, from its progress file's ``data``, as a job card (None without a file).

    A job while its recipe's pid is alive and the phase is not done or failed: pause and resume (the PAUSE file) and
    stop (the STOP file) between takes, nothing while it builds. A finished batch shows its dataset and offers nothing;
    so does one whose recipe is gone without saying it finished."""
    if not isinstance(data, dict):
        return None
    name = str(data.get("name") or "?")
    phase = str(data.get("phase") or "")
    total, take, kept = _count(data.get("total")), _count(data.get("take")), _count(data.get("kept"))
    set_aside, unplanned, unchecked = _count(data.get("setAside")), _count(data.get("unplanned")), _count(data.get("unchecked"))
    started, updated = _num(data.get("started")), _num(data.get("updated"))
    pid = data.get("pid") if isinstance(data.get("pid"), int) and not isinstance(data.get("pid"), bool) else None
    dataset = data.get("dataset") if isinstance(data.get("dataset"), str) and data.get("dataset") else None
    detail = [f"take {take} of {total}: {kept} kept, {set_aside} set aside, {unplanned} not planned"
              + (f", {unchecked} not checked yet" if unchecked else "")]
    title = f"Safety batch {name}" if twin == 0 else f"Safety batch on twin {twin}: {name}"
    job = {"id": f"batch-{name}", "kind": "batch", "twin": twin, "title": title, "phase": phase, "started": started,
           "pids": [pid] if pid else [], "progress": {"done": take, "total": total, "unit": "takes"} if total > 0 else None,
           "eta_s": None, "eta_note": "", "actions": [], "detail": detail}
    if phase in ("done", "failed"):
        job["status"] = phase
        detail.append(f"dataset: {dataset}" if dataset else "no dataset written")
        return job
    if pid is None or not alive(pid):
        job["status"] = "stopped"
        detail.append(f"its recipe ({f'pid {pid}' if pid else 'no pid'}) is not running any more: it ended without "
                      "saying it finished")
        return job
    job["status"] = "paused" if phase == "paused" else "running"
    detail.append(PHASE_TEXT.get(phase, f"phase: {phase or '?'}"))
    if phase in ("takes", "paused"):
        if pause_file and phase == "takes":
            detail.append("pause asked for: it waits after the take in progress")
        if not pause_file and phase == "paused":
            detail.append("resume asked for: it goes on shortly")
        if stop_file:
            detail.append("stop asked for: no take after the one in progress, then the dataset from the kept takes")
            job["actions"] = ["resume"] if pause_file else []
        else:
            job["actions"] = ["resume" if pause_file else "pause", "stop"]
    if dataset:
        detail.append(f"dataset: {dataset}")
    job["eta_s"], job["eta_note"] = batch_eta(phase, started, updated, take, total, kept, now, stop_file)
    return job


# ---------------------------------------------------------------------------------------------------------------------
# The whitelist


def _twin_number(job: dict) -> int:
    """A job's twin (0 when it names none), refused unless it is a small whole number."""
    twin = job.get("twin", 0)
    if not isinstance(twin, int) or isinstance(twin, bool) or not 0 <= twin <= 9:
        raise Refused(f"refused: {job.get('title') or job.get('id')} names no twin")
    return twin


def plan_action(remote: Optional[dict], batches, job_id: object, action: object) -> dict:
    """What ``action`` on ``job_id`` runs, decided from the latest status only: the job must be in it and list the
    action, and the (kind, action) must be in the tables above. ``batches`` are the local batches' jobs (one, a list, or
    None). Raises Refused otherwise."""
    if not isinstance(job_id, str) or not isinstance(action, str):
        raise Refused("refused: job and action must be strings")
    for batch in [batches] if isinstance(batches, dict) else (batches or []):
        if not (isinstance(batch, dict) and batch.get("id") == job_id):
            continue
        title = str(batch.get("title") or job_id)
        if action not in BATCH_ACTIONS or action not in (batch.get("actions") or []):
            raise Refused(f"refused: {title} does not offer {action!r} now")
        twin = _twin_number(batch)
        what = BATCH_ACTIONS[action] + ("" if twin == 0 else f"-{twin}")
        return {"type": "batch", "action": action, "twin": twin, "title": title, "what": what}
    jobs = remote.get("jobs") if isinstance(remote, dict) else None
    job = next((j for j in jobs or [] if isinstance(j, dict) and j.get("id") == job_id), None)
    if job is None:
        raise Refused(f"refused: no job {job_id!r} in the latest status")
    title = str(job.get("title") or job_id)
    if action not in (job.get("actions") or []):
        raise Refused(f"refused: {title} does not offer {action!r} now")
    kind = job.get("kind")
    if (kind, action) in JUST_ACTIONS:
        recipe, timeout = JUST_ACTIONS[(kind, action)]
        if kind == "twin" and _twin_number(job) != 0:
            recipe = f"{recipe} {_twin_number(job)}"
        return {"type": "just", "recipe": recipe, "timeout": timeout, "title": title, "what": f"just {recipe}"}
    if (kind, action) in SIGNALS:
        pgid = job.get("pgid")
        if not isinstance(pgid, int) or isinstance(pgid, bool) or pgid <= 1:
            raise Refused(f"refused: {title} names no process group")
        sig = SIGNALS[(kind, action)]
        return {"type": "signal", "kind": kind, "signal": sig, "pgid": pgid, "title": title,
                "what": f"SIG{sig} to process group {pgid} on the GPU machine"}
    raise Refused(f"refused: {action!r} on a {kind} job is not on this page's list")


# ---------------------------------------------------------------------------------------------------------------------
# The dashboard


class Dashboard:
    def __init__(self, host: str, repo: str, interval: float = 5.0, port: int = 8765, probe_source: Optional[bytes] = None,
                 page_path: Optional[str] = None, clock: Callable[[], float] = time.time,
                 alive: Callable[[int], bool] = pid_alive):
        self.host = host
        self.repo = os.path.abspath(repo)
        self.interval = float(interval)
        self.port = port
        if probe_source is None:
            with open(os.path.join(HERE, "probe.py"), "rb") as f:
                probe_source = f.read()
        self.probe = probe_source
        self.page_path = page_path or os.path.join(HERE, "index.html")
        self.clock = clock
        self.alive = alive
        self.token = secrets.token_urlsafe(32)
        # An action needs a status no older than this: acting on a stale view is how the wrong thing gets stopped.
        self.max_age = max(60.0, 4 * self.interval)
        self.lock = threading.Lock()
        self.remote: Optional[dict] = None
        self.remote_at: Optional[float] = None
        self.remote_error: Optional[str] = None
        self.actions_log: List[dict] = []
        self.inflight: Set[str] = set()
        self.next_id = 1
        self.batch_last: Dict[int, Optional[dict]] = {}  # each batch file's last good read (a read can meet it half-written)
        self.control_dir: Optional[str] = None
        self.stopping = threading.Event()
        self.wake = threading.Event()
        self.children: Set[subprocess.Popen] = set()  # ssh clients running now, ended on close
        self.recipes: Dict[subprocess.Popen, str] = {}  # recipes running now, left to finish on close
        self.poller: Optional[threading.Thread] = None
        # Seams for the tests: how a remote command and a local recipe run, and how an action's worker starts.
        self.ssh: Callable[[List[str], bytes, float], Tuple[int, str, str]] = self._ssh
        self.run_recipe: Callable[[str, float], Tuple[Optional[int], str]] = self._run_recipe
        self.spawn: Callable[[Callable[[], None]], None] = lambda fn: threading.Thread(target=fn, daemon=True).start()

    # -- requests --------------------------------------------------------------------------------------------------

    def host_ok(self, host_header: Optional[str]) -> bool:
        return (host_header or "").strip().lower() in (f"127.0.0.1:{self.port}", f"localhost:{self.port}")

    def token_ok(self, token: Optional[str]) -> bool:
        return isinstance(token, str) and hmac.compare_digest(token.encode("utf-8"), self.token.encode("utf-8"))

    def page(self) -> Tuple[bytes, str]:
        """index.html with this run's token and a fresh nonce for its inline script and style (read on each request)."""
        nonce = secrets.token_urlsafe(18)
        with open(self.page_path, encoding="utf-8") as f:
            html = f.read()
        return html.replace("__TOKEN__", self.token).replace("__NONCE__", nonce).encode("utf-8"), nonce

    def status(self) -> dict:
        now = self.clock()
        with self.lock:
            remote, at, error = self.remote, self.remote_at, self.remote_error
            log = [dict(e) for e in self.actions_log]
        batches = self.batches(now)
        return {"remote": remote, "remote_error": error, "remote_age_s": None if at is None else round(max(0.0, now - at), 1),
                "batch": next((b for b in batches if b.get("twin") == 0), None), "batches": batches, "actions_log": log,
                "host": self.host, "interval_s": self.interval, "now": now}

    def batch(self, now: Optional[float] = None, twin: int = 0) -> Optional[dict]:
        """Twin ``twin``'s batch (twin 0's by default) as a job card, from its progress file; None without one."""
        paths = batch_paths(self.repo, twin)
        try:
            with open(paths["file"], encoding="utf-8") as f:
                data = json.load(f)
            self.batch_last[twin] = data
        except FileNotFoundError:
            self.batch_last.pop(twin, None)
            return None
        except (OSError, ValueError):
            data = self.batch_last.get(twin)
        return batch_job(data, self.clock() if now is None else now, self.alive, os.path.exists(paths["pause"]),
                         os.path.exists(paths["stop"]), twin)

    def batches(self, now: Optional[float] = None) -> List[dict]:
        """Every batch with a progress file (one a twin, docs/adr/0037 amendment 7), twin 0's first."""
        return [b for b in (self.batch(now, t) for t in batch_twins(self.repo)) if b is not None]

    def handle_action(self, token: Optional[str], body: bytes) -> Tuple[int, dict]:
        """POST /api/action: (HTTP status, {ok, message}). Starts the action and answers at once."""
        if not self.token_ok(token):
            return 403, {"ok": False, "message": "refused: no valid X-Token (reload the page)"}
        try:
            data = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return 400, {"ok": False, "message": "refused: the body is not JSON"}
        if not isinstance(data, dict):
            return 400, {"ok": False, "message": "refused: the body is not {job, action}"}
        try:
            return 202, {"ok": True, "message": self.start_action(data.get("job"), data.get("action"))}
        except Refused as e:
            return 409, {"ok": False, "message": str(e)}

    # -- actions ---------------------------------------------------------------------------------------------------

    def start_action(self, job_id: object, action: object) -> str:
        now = self.clock()
        with self.lock:
            remote, at = self.remote, self.remote_at
        plan = plan_action(remote, self.batches(now), job_id, action)
        if plan["type"] != "batch":
            age = None if at is None else now - at
            if age is None or age > self.max_age:
                old = "missing" if age is None else f"{age:.0f} s old"
                raise Refused(f"refused: the GPU machine's last status is {old}; wait for a fresh one")
        with self.lock:
            if job_id in self.inflight:
                raise Refused(f"refused: an action on {plan['title']} is still running")
            self.inflight.add(str(job_id))
            entry = {"id": self.next_id, "time": now, "job": job_id, "title": plan["title"], "action": action,
                     "what": plan["what"], "ok": None, "output": "", "finished": None}
            self.next_id += 1
            self.actions_log.append(entry)
            del self.actions_log[:-LOG_KEEP]
        self.spawn(lambda: self._execute(entry, plan))
        return f"started: {plan['what']}"

    def _execute(self, entry: dict, plan: dict) -> None:
        try:
            if plan["type"] == "just":
                code, output = self.run_recipe(plan["recipe"], plan["timeout"])
                ok = code == 0
                if code is None:
                    output = f"{output}\ntimed out after {plan['timeout']} s".strip()
            elif plan["type"] == "signal":
                ok, output = self._signal(plan)
            else:
                ok, output = self._batch_file(plan["action"], plan.get("twin", 0))
        except Exception as e:  # whatever happens, the outcome lands in the log
            ok, output = False, f"{type(e).__name__}: {e}"
        with self.lock:
            entry.update(ok=ok, output=tail_text(output), finished=self.clock())
            self.inflight.discard(str(entry["job"]))
        self.wake.set()  # look at the machine again now, so the page shows the effect

    def _signal(self, plan: dict) -> Tuple[bool, str]:
        remote = ["python3", "-", "act", plan["kind"], plan["signal"], str(plan["pgid"])]
        try:
            code, out, err = self.ssh(remote, self.probe, ACT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return False, f"no answer from {self.host} within {ACT_TIMEOUT_S} s"
        except OSError as e:
            return False, f"ssh did not start: {e}"
        answer = last_json(out)
        if answer is None:
            return False, f"ssh exited {code}: {tail_text(err or out, 600) or 'no message'}"
        return bool(answer.get("ok")), str(answer.get("message", ""))

    def _batch_file(self, action: str, twin: int = 0) -> Tuple[bool, str]:
        paths = batch_paths(self.repo, twin)
        sfx = "" if twin == 0 else f"-{twin}"
        stamp = time.strftime("%H:%M:%S")
        if action == "pause":
            os.makedirs(os.path.dirname(paths["pause"]), exist_ok=True)
            with open(paths["pause"], "w", encoding="utf-8") as f:
                f.write(f"paused from the GPU dashboard at {stamp}\n")
            return True, f"wrote recordings/.drive/PAUSE{sfx}: the batch waits after the take in progress"
        if action == "resume":
            try:
                os.remove(paths["pause"])
            except FileNotFoundError:
                return True, f"there was no recordings/.drive/PAUSE{sfx}: nothing held the batch"
            return True, f"removed recordings/.drive/PAUSE{sfx}: the batch goes on"
        if action == "stop":
            os.makedirs(os.path.dirname(paths["stop"]), exist_ok=True)
            with open(paths["stop"], "w", encoding="utf-8") as f:
                f.write(f"stopped from the GPU dashboard at {stamp}\n")
            return True, f"wrote recordings/.drive/STOP{sfx}: the batch stops after the take in progress and builds its dataset"
        return False, f"not a batch action: {action!r}"

    # -- processes -------------------------------------------------------------------------------------------------

    def ssh_argv(self, remote: List[str]) -> List[str]:
        return ["ssh", "-o", "ControlMaster=auto", "-o", f"ControlPath={self.control_dir}/cm-%C", "-o", "ControlPersist=120",
                "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", self.host] + remote

    def _ssh(self, remote: List[str], data: bytes, timeout: float) -> Tuple[int, str, str]:
        with self.lock:
            if self.stopping.is_set():  # close() has ended the connection: start nothing new on it
                raise OSError("the dashboard is stopping")
            p = subprocess.Popen(self.ssh_argv(remote), stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.children.add(p)
        try:
            out, err = p.communicate(data, timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            p.communicate()
            raise
        finally:
            with self.lock:
                self.children.discard(p)
        return p.returncode, out.decode("utf-8", "replace"), err.decode("utf-8", "replace")

    def _run_recipe(self, recipe: str, timeout: float) -> Tuple[Optional[int], str]:
        """`just <recipe>` in the repo; (exit code or None on a timeout, its output). In a session of its own, writing
        to a file, so that Ctrl+C here or the page going away leaves a half-done stop to finish."""
        just = shutil.which("just")
        if not just:
            return 127, "just is not on PATH"
        env = dict(os.environ, GPU_HOST=self.host)  # the recipes act on the machine this page shows
        argv = recipe.split()  # a recipe and its arguments, from the whitelist only (a twin's number at most)
        fd, out_path = tempfile.mkstemp(prefix=f"{argv[0]}-", suffix=".log", dir=self.control_dir)
        with os.fdopen(fd, "wb") as out:
            p = subprocess.Popen([just] + argv, cwd=self.repo, stdin=subprocess.DEVNULL, stdout=out,
                                 stderr=subprocess.STDOUT, env=env, start_new_session=True)
        with self.lock:
            self.recipes[p] = out_path
        try:
            code: Optional[int] = p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            code = None
            try:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                p.kill()
        finally:
            with self.lock:
                self.recipes.pop(p, None)
        with open(out_path, "rb") as f:
            output = f.read().decode("utf-8", "replace")
        os.unlink(out_path)
        return code, output

    # -- the poller ------------------------------------------------------------------------------------------------

    def poll_once(self) -> None:
        data, error = None, None
        try:
            code, out, err = self.ssh(["python3", "-"], self.probe, PROBE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            error = f"no answer from {self.host} within {PROBE_TIMEOUT_S} s"
        except OSError as e:
            error = f"ssh did not start: {e}"
        else:
            if code != 0:
                error = f"ssh exited {code}: {tail_text(err or out, 400) or 'no message'}"
            else:
                data = last_json(out)
                if data is None or not isinstance(data.get("jobs"), list):
                    data, error = None, f"the probe gave no status: {tail_text(err or out, 400) or 'no output'}"
        with self.lock:
            if data is not None:
                self.remote, self.remote_at, self.remote_error = data, self.clock(), None
            else:
                self.remote_error = error

    def _poll_forever(self) -> None:
        while not self.stopping.is_set():
            self.wake.clear()
            self.poll_once()
            self.wake.wait(self.interval)

    def start(self) -> None:
        # Private (0700) and short: a unix socket's path must stay under about 100 bytes.
        self.control_dir = tempfile.mkdtemp(prefix="gpu-ui-", dir="/tmp" if os.path.isdir("/tmp") else None)
        self.poller = threading.Thread(target=self._poll_forever, name="probe", daemon=True)
        self.poller.start()

    def close(self) -> None:
        with self.lock:  # with _ssh's check: no ssh client starts after this
            self.stopping.set()
            children, recipes = list(self.children), dict(self.recipes)
        self.wake.set()
        for p in children:
            try:
                p.kill()
            except OSError:
                pass
        if self.poller:
            self.poller.join(timeout=10)
        if not self.control_dir:
            return
        # The connection's master outlives its last client by ControlPersist: end it now.
        try:
            subprocess.run(["ssh", "-o", f"ControlPath={self.control_dir}/cm-%C", "-O", "exit", self.host],
                           stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass
        if recipes:
            for p, out_path in recipes.items():
                print(f"[gpu-ui] still running, left to finish: {' '.join(map(str, p.args))} (pid {p.pid}), output in {out_path}")
            return
        for name in os.listdir(self.control_dir):
            try:
                os.unlink(os.path.join(self.control_dir, name))
            except OSError:
                pass
        try:
            os.rmdir(self.control_dir)
        except OSError:
            pass


# ---------------------------------------------------------------------------------------------------------------------
# HTTP


def csp(nonce: str) -> str:
    return (f"default-src 'none'; script-src 'nonce-{nonce}'; style-src 'nonce-{nonce}'; connect-src 'self'; "
            "img-src data:; base-uri 'none'; form-action 'none'; frame-ancestors 'none'")


def make_handler(dash: Dashboard) -> type:
    class Handler(BaseHTTPRequestHandler):
        server_version = "gpu-ui"
        sys_version = ""

        def _send(self, code: int, body: bytes, ctype: str, headers: Optional[Dict[str, str]] = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "no-referrer")
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload: dict) -> None:
            self._send(code, json.dumps(payload).encode("utf-8"), "application/json")

        def _text(self, code: int, text: str) -> None:
            self._send(code, (text + "\n").encode("utf-8"), "text/plain; charset=utf-8")

        def _host_ok(self) -> bool:
            if dash.host_ok(self.headers.get("Host")):
                return True
            self._text(403, f"refused: this page answers only to 127.0.0.1:{dash.port} or localhost:{dash.port}")
            return False

        def do_GET(self) -> None:
            if not self._host_ok():
                return
            path = urlsplit(self.path).path
            if path in ("/", "/index.html"):
                try:
                    body, nonce = dash.page()
                except OSError as e:
                    return self._text(500, f"cannot read the page: {e}")
                return self._send(200, body, "text/html; charset=utf-8", {"Content-Security-Policy": csp(nonce)})
            if path == "/api/status":
                return self._json(200, dash.status())
            self._text(404, "not found")

        def do_POST(self) -> None:
            if not self._host_ok():
                return
            if urlsplit(self.path).path != "/api/action":
                return self._text(404, "not found")
            if not dash.token_ok(self.headers.get("X-Token")):
                return self._json(403, {"ok": False, "message": "refused: no valid X-Token (reload the page)"})
            try:
                n = int(self.headers.get("Content-Length") or "")
            except ValueError:
                return self._json(411, {"ok": False, "message": "refused: no Content-Length"})
            if not 0 < n <= MAX_BODY:
                return self._json(413, {"ok": False, "message": "refused: the body is empty or too long"})
            code, payload = dash.handle_action(self.headers.get("X-Token"), self.rfile.read(n))
            self._json(code, payload)

        def log_request(self, code="-", size="-") -> None:
            # The page asks every 3 s: log only what changes things, and what went wrong.
            try:
                failed = int(code) >= 400
            except (TypeError, ValueError):
                failed = False
            if self.command == "POST" or failed:
                super().log_request(code, size)

    return Handler


class Server(ThreadingHTTPServer):
    daemon_threads = True


def open_browser(url: str) -> bool:
    """Opens ``url`` in Windows' default browser from WSL."""
    exe = shutil.which("explorer.exe")
    if not exe:
        return False
    try:
        subprocess.Popen([exe, url], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    except OSError:
        return False
    return True


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="The GPU machine at a glance, on a local page (robot/gpu/dashboard.py).")
    ap.add_argument("--host", required=True, help="the GPU machine, as ssh reaches it: user@address")
    ap.add_argument("--port", type=int, default=8765, help="the local port (127.0.0.1 only)")
    ap.add_argument("--repo", required=True, help="the repo root: where `just` runs, and where recordings/.drive is")
    ap.add_argument("--interval", type=float, default=5.0, help="seconds between two looks at the machine")
    ap.add_argument("--open", action="store_true", help="open the page in Windows' browser (from WSL)")
    a = ap.parse_args(argv)
    if not HOST_RE.fullmatch(a.host):
        ap.error(f"--host is user@address, got {a.host!r}")
    if not 0 < a.port < 65536:
        ap.error("--port is 1 to 65535")
    if a.interval < 1:
        ap.error("--interval is at least 1 s")
    if not os.path.isdir(a.repo):
        ap.error(f"--repo {a.repo} is not a folder")
    if not os.path.isfile(os.path.join(a.repo, "justfile")):
        print(f"[gpu-ui] warning: no justfile in {a.repo}: the twin's and the watcher's buttons will fail", file=sys.stderr)
    dash = Dashboard(a.host, a.repo, a.interval, a.port)
    try:
        server = Server(("127.0.0.1", a.port), make_handler(dash))
    except OSError as e:
        print(f"[gpu-ui] cannot listen on 127.0.0.1:{a.port}: {e} (another dashboard? try --port)", file=sys.stderr)
        return 1
    dash.port = server.server_address[1]
    dash.start()
    url = f"http://127.0.0.1:{dash.port}/"
    print(f"[gpu-ui] {url} shows {a.host}, looked at every {a.interval:g} s. Ctrl+C stops it", flush=True)
    if a.open and not open_browser(url):
        print(f"[gpu-ui] open {url} in a browser", flush=True)

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, stop)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\n[gpu-ui] stopping", flush=True)
    finally:
        server.server_close()
        dash.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
