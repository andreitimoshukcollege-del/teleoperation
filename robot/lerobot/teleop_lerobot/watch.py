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
   - otherwise each kept take not yet in the ledger is appended, oldest first.
"""

from __future__ import annotations

import json
import os
import re
import signal
import time
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from .jlog import JlogError
from .takes import Take, TakeError, checksum_id, read_take, verify_checksums
from .writer import append_take, read_ledger, rebuild, recorded_episodes

TAKE = re.compile(r"^\d{8}T\d{6}Z-take\d+$")


def complete_takes(roots: Iterable[str], now: float, settle: float) -> List[str]:
    """Every take folder whose ``files.sha256`` is at least ``settle`` seconds old, in scenario then name order."""
    out = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        for scenario in sorted(os.listdir(root)):
            folder = os.path.join(root, scenario)
            if scenario.startswith(("_", ".")) or not os.path.isdir(folder):
                continue
            for name in sorted(os.listdir(folder)):
                take = os.path.join(folder, name)
                done = os.path.join(take, "files.sha256")
                if TAKE.match(name) and os.path.isfile(done) and now - os.path.getmtime(done) >= settle:
                    out.append(os.path.abspath(take))
    return out


class Converter:
    def __init__(self, roots: List[str], out: str, writer, fps: int = 15, settle: float = 60.0,
                 log: Callable[[str], None] = print, clock: Callable[[], float] = time.time, puller=None):
        self.roots, self.out, self.writer, self.puller = roots, out, writer, puller
        self.fps, self.settle, self.log, self.clock = fps, settle, log, clock
        self.checked: Dict[Tuple[str, str], Optional[Tuple[str, str]]] = {}  # (take, content) -> (group, scenario) or None
        self.failed_rebuilds: Dict[str, Tuple] = {}
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
                if os.path.isdir(g) and not group.startswith((".", "_")):
                    out += [os.path.join(g, s) for s in sorted(os.listdir(g))
                            if os.path.isdir(os.path.join(g, s)) and ".rebuild-" not in s and ".old-" not in s]
        return out

    def pass_once(self) -> int:
        """One pass. Returns how many episodes were written."""
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
            broken = bool(ledger) and recorded_episodes(root) != len(ledger)
            if stale or broken:
                signature = (tuple(sorted(current)), tuple((e["take"], e["filesSha256"]) for e in ledger))
                if self.failed_rebuilds.get(root) == signature:
                    continue  # it failed for exactly this state; wait for a change
                why = f"{len(stale)} take(s) set aside" if stale else "its ledger and LeRobot's episodes disagree"
                self.log(f"rebuilding {root}: {why}")
                try:
                    lines = rebuild(self.writer, root, [self._read(t) for t in takes], self.log)
                    written += len(lines)
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
                    self.log(f"{root}: episode {line['episode']} from {take} ({line['frames']} frames, "
                             f"{line['filled']} filled)")
                except Exception as e:  # noqa: BLE001
                    self.log(f"writing {take} into {root} failed: {e!r}; the next pass rebuilds it if it is broken")
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
