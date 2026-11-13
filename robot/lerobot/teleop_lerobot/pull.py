"""The real robot's takes, copied to the GPU machine for the converter (docs/adr/0036 decision 7).

The Jetson keeps takes in ``~/recordings/scenarios/<scenario>/<UTC>-take<n>/`` (docs/adr/0033 amendment 1). At most
every ``every`` seconds, a pass:

1. **Finds the robot.** It tries each address in ``hosts_file`` (written by ``just lerobot-robot-key``; the robot is on
   DHCP), then its mDNS name. It uses the first that answers.
2. **Lists the robot's takes** in one ssh call:
   - the complete ones (with ``files.sha256``), each with that file's sha256;
   - the ones set aside (``_discarded``);
   - the session being recorded now.
3. **Copies each complete take it does not have yet,** never the one being recorded. Each goes into
   ``<local>/.incoming/`` first and moves into ``<local>/<scenario>/`` only once ``files.sha256`` holds there. The
   converter never sees a half-copied take, since it skips ``.`` folders.
4. **Follows the robot's DISCARD LAST.** A take the robot has set aside, and that sits here among the kept ones, is
   moved into ``<local>/_discarded/``. The converter then rebuilds its dataset without it.

Nothing on the robot is changed: deleting its copies is ``just robot-prune-takes``, by hand.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from .takes import TakeError, checksum_id, verify_checksums

NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,23}$")  # a scenario name (docs/adr/0033 decision 2)
TAKE = re.compile(r"^\d{8}T\d{6}Z-take\d+$")

LISTING = r"""
cd ~/recordings/scenarios 2>/dev/null || { echo NOROOT; exit 0; }
echo "ACTIVE $(cat ~/recordings/.active 2>/dev/null)"
for f in */*-take*/files.sha256; do
    [ -f "$f" ] || continue
    case "$f" in _*) continue ;; esac
    echo "KEPT ${f%/files.sha256} $(sha256sum "$f" | cut -c1-64)"
done
for f in _discarded/*/*-take*/files.sha256; do
    [ -f "$f" ] || continue
    d=${f#_discarded/}
    echo "DISCARDED ${d%/files.sha256}"
done
"""

Run = Callable[[Sequence[str], float], Tuple[int, str]]


def run_command(cmd: Sequence[str], timeout: float) -> Tuple[int, str]:
    try:
        p = subprocess.run(list(cmd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout, text=True)
        return p.returncode, p.stdout
    except subprocess.TimeoutExpired:
        return 124, "timed out"


def _take(path: str) -> Optional[Tuple[str, str]]:
    parts = path.split("/")
    if len(parts) == 2 and NAME.match(parts[0]) and TAKE.match(parts[1]):
        return parts[0], parts[1]
    return None


def parse_listing(text: str) -> Tuple[Optional[str], Dict[Tuple[str, str], str], Set[Tuple[str, str]]]:
    """(the take being recorded, kept takes -> sha256 of their files.sha256, discarded takes). Malformed names are
    ignored: they never become paths here."""
    active, kept, discarded = None, {}, set()
    for line in text.splitlines():
        kind, _, rest = line.partition(" ")
        if kind == "ACTIVE":
            active = rest.strip().rstrip("/") or None
        elif kind == "KEPT":
            path, _, sha = rest.partition(" ")
            t = _take(path)
            if t and re.fullmatch(r"[0-9a-f]{64}", sha.strip()):
                kept[t] = sha.strip()
        elif kind == "DISCARDED":
            t = _take(rest.strip())
            if t:
                discarded.add(t)
    return active, kept, discarded


class RobotPuller:
    def __init__(self, local_root: str, hosts_file: Optional[str] = None, mdns: str = "jetrover-sinrg.local",
                 user: str = "jetson", key: Optional[str] = None, every: float = 60.0, run: Run = run_command,
                 log: Callable[[str], None] = print, clock: Callable[[], float] = time.time):
        self.local, self.hosts_file, self.mdns, self.user, self.key = local_root, hosts_file, mdns, user, key
        self.every, self.run, self.log, self.clock = every, run, log, clock
        self._next = 0.0
        self._said = ""  # the last state logged, so an unreachable robot is logged once, not every minute

    def _say(self, state: str, message: str) -> None:
        if state != self._said:
            self._said = state
            self.log(message)

    def hosts(self) -> List[str]:
        out = []
        if self.hosts_file and os.path.exists(self.hosts_file):
            with open(self.hosts_file, encoding="utf-8") as f:
                out += [h.strip() for h in f if re.fullmatch(r"[A-Za-z0-9.:-]+", h.strip())]
        if self.mdns:
            out.append(self.mdns)
        return list(dict.fromkeys(out))

    def _ssh(self) -> List[str]:
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new"]
        if self.key:
            cmd += ["-i", self.key, "-o", "IdentitiesOnly=yes"]
        return cmd

    def pass_once(self, force: bool = False) -> int:
        """One pass, if due. Returns how many takes it copied."""
        now = self.clock()
        if not force and now < self._next:
            return 0
        self._next = now + self.every
        if self.key and not os.path.exists(self.key):
            self._say("nokey", f"robot: no key at {self.key} (just lerobot-robot-key): not copying the robot's takes")
            return 0
        listing, host = None, None
        for h in self.hosts():
            code, out = self.run(self._ssh() + [f"{self.user}@{h}", LISTING], 30)  # the robot's shell runs the script
            if code == 0:
                listing, host = out, h
                break
        if listing is None:
            self._say("unreachable", f"robot: not reachable at {', '.join(self.hosts()) or 'no address'}; trying again every {self.every:g} s")
            return 0
        if "NOROOT" in listing.split():
            self._say("noroot", f"robot {host}: no ~/recordings/scenarios yet (RobotHost without --scenarios?)")
            return 0
        self._say("ok:" + host, f"robot: reached at {host}")
        active, kept, discarded = parse_listing(listing)
        active_take = _take("/".join(active.replace("\\", "/").split("/")[-2:])) if active else None
        copied = 0
        for (scenario, name), sha in sorted(kept.items()):
            here = os.path.join(self.local, scenario, name)
            if (scenario, name) == active_take or os.path.exists(here) \
                    or os.path.exists(os.path.join(self.local, "_discarded", scenario, name)):
                continue
            if self._copy(host, scenario, name, sha):
                copied += 1
        for scenario, name in sorted(discarded):
            here = os.path.join(self.local, scenario, name)
            if os.path.isdir(here):
                away = os.path.join(self.local, "_discarded", scenario)
                os.makedirs(away, exist_ok=True)
                shutil.move(here, os.path.join(away, name))
                self.log(f"robot: {scenario}/{name} was set aside on the robot: set aside here too")
        return copied

    def _copy(self, host: str, scenario: str, name: str, sha: str) -> bool:
        incoming = os.path.join(self.local, ".incoming", scenario, name)
        os.makedirs(os.path.dirname(incoming), exist_ok=True)
        ssh = " ".join(self._ssh())
        code, out = self.run(["rsync", "-a", "--partial", "-e", ssh,
                              f"{self.user}@{host}:recordings/scenarios/{scenario}/{name}/", incoming + "/"], 3600)
        if code != 0:
            self.log(f"robot: copying {scenario}/{name} failed ({code}): {out.strip()[-300:]}; trying again next pass")
            return False
        try:
            verify_checksums(incoming)
            if checksum_id(incoming) != sha:
                raise TakeError("its files.sha256 changed while it was copied")
        except (TakeError, OSError) as e:
            self.log(f"robot: {scenario}/{name} did not check out after copying ({e}); copying it again next pass")
            shutil.rmtree(incoming, ignore_errors=True)
            return False
        os.makedirs(os.path.join(self.local, scenario), exist_ok=True)
        os.rename(incoming, os.path.join(self.local, scenario, name))
        self.log(f"robot: copied {scenario}/{name} from {host}")
        return True
