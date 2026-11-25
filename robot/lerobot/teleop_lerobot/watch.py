"""Finds complete takes and keeps every scenario's LeRobot dataset up to date (docs/adr/0036 decision 6).

Each pass:

1. Every take folder under the scenario roots (``<root>/<scenario>/<UTC>-take<n>/``; ``_discarded`` and other ``_`` or
   ``.`` folders are not scenarios) whose ``files.sha256`` has existed for ``settle`` seconds is a candidate. The
   settle time lets a DISCARD LAST pressed straight after a take move it away first.
2. A candidate is checked once: ``files.sha256`` must hold, and it must read as a take (``takes.read_take``). One that
   fails is written to ``skipped.jsonl`` with its reason and never retried, unless its content changes.
3. Per dataset (``<out>/<robot>-<source>/<scenario>/``):
   - a take in its ledger that is no longer kept (set aside, or gone), or a ledger that disagrees with LeRobot's own
     episode count (a write that died), means a rebuild from the kept takes;
   - otherwise each kept take not yet in the ledger is appended, oldest first. A take that reads but cannot be written
     is tried for ``APPEND_ATTEMPTS`` passes, then skipped like an unreadable one, so it never holds up the takes after it.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import time
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from .jlog import JlogError
from .safety import SCENARIOS as SAFETY_CATEGORIES  # the four categories and the mixed script (docs/adr/0038)
from .safety import GROUP_SUFFIX as SAFETY_GROUP_SUFFIX
from .takes import Take, TakeError, checksum_id, read_take, verify_checksums
from .writer import append_take, read_ledger, rebuild, recorded_episodes

TAKE = re.compile(r"^\d{8}T\d{6}Z-take\d+$")

# A take that reads but cannot be written is tried this many passes, then skipped (written to skipped.jsonl).
APPEND_ATTEMPTS = 3


def complete_takes(roots: Iterable[str], now: float, settle: float) -> List[str]:
    """Every take folder whose ``files.sha256`` is at least ``settle`` seconds old, in scenario then name order."""
    out = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        for scenario in sorted(os.listdir(root)):
            folder = os.path.join(root, scenario)
            # The safety scenarios' takes become one dataset per batch (safety.py, docs/adr/0036 amendment 1; the mixed
            # script's too, docs/adr/0038), not a dataset per scenario.
            if scenario.startswith(("_", ".")) or scenario in SAFETY_CATEGORIES or not os.path.isdir(folder):
                continue
            for name in sorted(os.listdir(folder)):
                take = os.path.join(folder, name)
                done = os.path.join(take, "files.sha256")
                if TAKE.match(name) and os.path.isfile(done) and now - os.path.getmtime(done) >= settle:
                    out.append(os.path.abspath(take))
    return out


class Converter:
    def __init__(self, roots: List[str], out: str, writer, fps: int = 15, settle: float = 60.0,
                 log: Callable[[str], None] = print, clock: Callable[[], float] = time.time, puller=None,
                 archive: Optional[List[str]] = None, data_root: str = ""):
        self.roots, self.out, self.writer, self.puller = roots, out, writer, puller
        # The cluster archive (docs/explorer-cluster.md "Data from the GPU machine"): a command that takes paths under
        # data_root, run in the background after a pass changed datasets, at most one at a time; empty = off.
        self.archive, self.data_root = list(archive or []), data_root or os.path.dirname(os.path.abspath(out))
        self._archive_proc: Optional[subprocess.Popen] = None
        self._archive_pending: set = set()
        self.fps, self.settle, self.log, self.clock = fps, settle, log, clock
        self.checked: Dict[Tuple[str, str], Optional[Tuple[str, str]]] = {}  # (take, content) -> (group, scenario) or None
        self.failed_rebuilds: Dict[str, Tuple] = {}
        self.append_failures: Dict[Tuple[str, str], int] = {}  # (take, content) -> failed writes so far
        self.stopping = False  # SIGTERM: finish the take being written, then stop
        os.makedirs(out, exist_ok=True)
        self._load_skipped()

    def _load_skipped(self) -> None:
        path = os.path.join(self.out, "skipped.jsonl")
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                for line in f:
                    if line.strip():
                        s = json.loads(line)
                        self.checked[(s["take"], s["filesSha256"])] = None

    def _skip(self, take: str, content: str, reason: str) -> None:
        reason = reason[len(take) + 2:] if reason.startswith(take + ": ") else reason  # the path is logged once
        self.checked[(take, content)] = None
        self.log(f"skipped {take}: {reason}")
        with open(os.path.join(self.out, "skipped.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps({"take": take, "filesSha256": content, "reason": reason,
                                "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}) + "\n")

    def _read(self, take: str) -> Take:
        return read_take(take, self.fps)

    def kept(self) -> Dict[str, List[str]]:
        """Dataset folder -> its kept takes, oldest first (every complete take that checks out)."""
        datasets: Dict[str, List[str]] = {}
        for take in complete_takes(self.roots, self.clock(), self.settle):
            try:
                content = checksum_id(take)
            except OSError:
                continue
            key = (take, content)
            if key not in self.checked:
                try:
                    verify_checksums(take)
                    t = self._read(take)
                    self.checked[key] = (t.dataset_group, t.scenario)
                except (TakeError, JlogError, OSError, ValueError, KeyError) as e:
                    self._skip(take, content, str(e))
            where = self.checked.get(key)
            if where is not None:
                datasets.setdefault(os.path.join(self.out, *where), []).append(take)
        for takes in datasets.values():
            takes.sort(key=os.path.basename)
        return datasets

    def known_datasets(self) -> List[str]:
        out = []
        if os.path.isdir(self.out):
            for group in sorted(os.listdir(self.out)):
                g = os.path.join(self.out, group)
                # A safety group (<robot>-<source>-safety, safety.py) holds datasets the safety builder writes once from
                # takes this watcher never converts: taken for its own, every take in them looked set aside and the
                # dataset was "rebuilt" empty, deleting it (2026-11-15, twice, once mid-build).
                if os.path.isdir(g) and not group.startswith((".", "_")) and not group.endswith(SAFETY_GROUP_SUFFIX):
                    out += [os.path.join(g, s) for s in sorted(os.listdir(g))
                            if os.path.isdir(os.path.join(g, s)) and ".rebuild-" not in s and ".old-" not in s]
        return out

    def pass_once(self) -> int:
        """One pass. Returns how many episodes were written; then sends what it changed to the cluster archive."""
        changed: set = set()
        try:
            return self._pass(changed)
        finally:
            self._archive_changed(changed)

    def _archive_changed(self, changed: set) -> None:
        if not self.archive:
            return
        rel = {os.path.relpath(p, self.data_root) for p in changed}
        self._archive_pending |= {r for r in rel if not r.startswith("..")}
        if self._archive_proc is not None and self._archive_proc.poll() is None:
            return  # one at a time; the next pass sends what piled up
        if self._archive_proc is not None and self._archive_proc.returncode:
            self.log(f"cluster archive: the last run exited {self._archive_proc.returncode}; its paths are retried")
        self._archive_proc = None
        if not self._archive_pending:
            return
        paths = sorted(self._archive_pending)
        self._archive_pending = set()
        log = os.path.join(os.path.dirname(os.path.abspath(self.out)), "..", "logs", "cluster", "watcher-archive.log")
        try:
            os.makedirs(os.path.dirname(log), exist_ok=True)
            with open(log, "a", encoding="utf-8") as f:
                self._archive_proc = subprocess.Popen(self.archive + paths, stdout=f, stderr=subprocess.STDOUT,
                                                      stdin=subprocess.DEVNULL)
            self.log(f"cluster archive: sending {len(paths)} folder(s) ({', '.join(paths[:4])}{' ...' if len(paths) > 4 else ''})")
        except OSError as e:
            self._archive_pending |= set(paths)
            self.log(f"cluster archive: could not start {self.archive[0]}: {e!r}")

    def _pass(self, changed: set) -> int:
        written = 0
        if self.puller is not None:
            try:
                self.puller.pass_once()  # the real robot's new takes (pull.py), at most once a minute
            except Exception as e:  # noqa: BLE001 - an unreachable robot must not stop the Isaac takes
                self.log(f"robot: the copy pass failed: {e!r}")
        kept = self.kept()
        for root in sorted(set(kept) | set(self.known_datasets())):
            takes = kept.get(root, [])
            ledger = read_ledger(root)
            listed = {(e["take"], e["filesSha256"]) for e in ledger}
            current = {(t, checksum_id(t)) for t in takes}
            stale = [e for e in ledger if (e["take"], e["filesSha256"]) not in current]
            # Broken when LeRobot's episodes and the ledger disagree, an empty ledger included: a take set aside while it
            # was the scenario's first episode left one episode and no ledger line, and nothing caught it (the 2026-11 bug
            # hunt, D-03). A ledger with no dataset at all (deleted) is broken too.
            episodes = recorded_episodes(root)
            broken = (episodes is None and bool(ledger)) or (episodes is not None and episodes != len(ledger))
            if stale or broken:
                signature = (tuple(sorted(current)), tuple((e["take"], e["filesSha256"]) for e in ledger))
                if self.failed_rebuilds.get(root) == signature:
                    continue  # it failed for exactly this state; wait for a change
                why = f"{len(stale)} take(s) set aside" if stale else "its ledger and LeRobot's episodes disagree"
                self.log(f"rebuilding {root}: {why}")
                try:
                    lines = rebuild(self.writer, root, [self._read(t) for t in takes], self.log)
                    written += len(lines)
                    changed.add(root)
                    changed.update(os.path.dirname(t) for t in takes)
                    self.failed_rebuilds.pop(root, None)
                except Exception as e:  # noqa: BLE001 - the watcher must outlive one bad dataset
                    self.failed_rebuilds[root] = signature
                    self.log(f"rebuild of {root} failed: {e!r}")
                continue
            for take in takes:
                if self.stopping:
                    return written
                if (take, checksum_id(take)) in listed:
                    continue
                try:
                    line = append_take(self.writer, root, self._read(take))
                    written += 1
                    changed.update((root, os.path.dirname(take)))
                    self.log(f"{root}: episode {line['episode']} from {take} ({line['frames']} frames, "
                             f"{line['filled']} filled)")
                except Exception as e:  # noqa: BLE001
                    # One take that cannot be written must not hold up its scenario: it was retried forever, the takes
                    # after it never ran, and a later discard's rebuild failed on it (the 2026-11 bug hunt, D-02). After
                    # APPEND_ATTEMPTS failures it is skipped like an unreadable take; a full disk gets a few passes first.
                    key = (take, checksum_id(take))
                    self.append_failures[key] = self.append_failures.get(key, 0) + 1
                    if self.append_failures[key] >= APPEND_ATTEMPTS:
                        self.append_failures.pop(key, None)
                        self._skip(take, key[1], f"writing it into {root} failed {APPEND_ATTEMPTS} times; last: {e!r}")
                    else:
                        self.log(f"writing {take} into {root} failed ({self.append_failures[key]} of {APPEND_ATTEMPTS}): {e!r}; "
                                 "the next pass rebuilds the dataset if it is broken")
                    break
        return written

    def run(self, every: float = 10.0, once: bool = False) -> None:
        self.log(f"watching {', '.join(self.roots)} -> {self.out} (fps {self.fps}, settle {self.settle:g} s)")
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, "stopping", True))
        while not self.stopping:
            try:
                self.pass_once()
            except Exception as e:  # noqa: BLE001
                self.log(f"pass failed: {e!r}")
            if once:
                return
            for _ in range(int(every * 10)):
                if self.stopping:
                    break
                time.sleep(0.1)
        self.log("stopped")
