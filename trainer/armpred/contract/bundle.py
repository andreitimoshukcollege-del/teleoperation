"""The data bundle: checked sessions packed into one file for the GPU machine (docs/adr/0021).

``just pack-dataset`` writes it (through ``armpred pack-dataset``) and ``armpred finetune`` reads
it, so the format has one implementation. A ``.tar`` holding:

- ``sessions/<id>/...``: each session directory, its ``check.json`` included;
- ``robot-profile.json``: the robot profile the sessions were recorded with;
- ``index.json``: ``{"format": "jetrover-data", "version": 1, "repoSha": ..., "sessions": [...]}``,
  each session with its conditions, the sha256 of every file, its read gaps and check statistics.

A session whose ``check.json`` is missing or failed is refused at packing; a file whose sha256 does
not match the index is refused at reading.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import tarfile
from typing import Dict, List, Sequence

FORMAT = "jetrover-data"
VERSION = 1


class BundleError(ValueError):
    pass


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def write_bundle(sessions: Sequence[str], out: str, profile_path: str, repo_sha: str) -> Dict[str, object]:
    index: Dict[str, object] = {"format": FORMAT, "version": VERSION, "repoSha": repo_sha, "sessions": []}
    entries: List[dict] = index["sessions"]  # type: ignore[assignment]
    for directory in sessions:
        check = os.path.join(directory, "check.json")
        if not os.path.exists(check):
            raise BundleError(f"{directory}: no check.json -- run the session check first (just recording-check)")
        with open(check, "r", encoding="utf-8") as f:
            report = json.load(f)
        if not report.get("ok"):
            raise BundleError(f"{directory}: its check failed: {report.get('failures')}")
        with open(os.path.join(directory, "session.json"), "r", encoding="utf-8") as f:
            meta = json.load(f)
        files = {name: _sha256(os.path.join(directory, name)) for name in sorted(os.listdir(directory))
                 if os.path.isfile(os.path.join(directory, name))}
        entries.append({"id": meta["id"], "source": meta.get("source", "robot"), "conditions": meta.get("conditions", {}),
                        "files": files, "readGaps": report.get("read_gaps", []), "stats": report.get("stats", {})})
    ids = [e["id"] for e in entries]
    if len(ids) != len(set(ids)):
        raise BundleError("two sessions share an id")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with tarfile.open(out, "w") as tar:
        for directory, entry in zip(sessions, entries):
            for name in entry["files"]:
                tar.add(os.path.join(directory, name), arcname=f"sessions/{entry['id']}/{name}")
        tar.add(profile_path, arcname="robot-profile.json")
        data = json.dumps(index, indent=2).encode("utf-8")
        info = tarfile.TarInfo("index.json")
        info.size = len(data)
        tar.addfile(info, io.BytesIO(data))
    return index


def read_bundle(path: str, into: str) -> Dict[str, object]:
    """Unpacks (once) into ``into`` and verifies every file against the index."""
    if not os.path.exists(os.path.join(into, "index.json")):
        os.makedirs(into, exist_ok=True)
        root = os.path.realpath(into)
        with tarfile.open(path, "r") as tar:
            for member in tar.getmembers():
                target = os.path.realpath(os.path.join(into, member.name))
                if not (target == root or target.startswith(root + os.sep)) or member.issym() or member.islnk():
                    raise BundleError(f"{path}: refusing member {member.name!r}")
            tar.extractall(into)
    with open(os.path.join(into, "index.json"), "r", encoding="utf-8") as f:
        index = json.load(f)
    if index.get("format") != FORMAT or int(index.get("version", 0)) != VERSION:
        raise BundleError(f"{path}: not a version-{VERSION} {FORMAT} bundle")
    for entry in index["sessions"]:
        directory = os.path.join(into, "sessions", entry["id"])
        for name, sha in entry["files"].items():
            if _sha256(os.path.join(directory, name)) != sha:
                raise BundleError(f"{entry['id']}/{name}: does not match the bundle's index")
    return index
