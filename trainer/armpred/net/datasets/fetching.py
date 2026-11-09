"""Downloading a public network dataset, and its manifest (docs/adr/0035 decision 2).

**Downloads are untrusted data.** They are fetched over HTTPS only, from the hosts the dataset names, each into a
``.part`` file under a name chosen here (never one the server sent), capped in size, and parsed afterwards as data:
nothing downloaded is ever run, imported or unpacked by a tool that could write outside the dataset's folder.

``<root>/<dataset>/manifest.json`` records where every file came from, its size and sha256, when it was fetched, the
licence and terms of use, the citation, and what was kept. A file already present with the sha256 its manifest records
is not fetched again.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import os
import shutil
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

DEFAULT_ROOT = os.path.expanduser(os.environ.get("TELEOP_NETWORK_DATASETS", "~/datasets/network"))
USER_AGENT = "teleop-research-network-datasets/1 (docs/adr/0035; one download per file)"
CHUNK = 1 << 20


@dataclass
class Source:
    """What a dataset's manifest says about where it comes from and under what terms."""

    name: str
    title: str
    hosts: List[str]
    licence: str
    terms_url: str
    citation: str
    notes: str = ""


@dataclass
class Fetched:
    path: str  # relative to the dataset folder
    url: str
    bytes: int
    sha256: str
    fetched_utc: str


@dataclass
class Manifest:
    source: Source
    files: List[Fetched] = field(default_factory=list)
    kept: str = ""
    extra: Dict = field(default_factory=dict)

    def to_json(self) -> Dict:
        return {
            "name": self.source.name, "title": self.source.title, "hosts": self.source.hosts,
            "licence": self.source.licence, "termsUrl": self.source.terms_url, "citation": self.source.citation,
            "notes": self.source.notes, "kept": self.kept,
            "files": [f.__dict__ for f in self.files], **self.extra,
        }


def now_utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def sha256_of(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(CHUNK), b""):
            h.update(block)
    return h.hexdigest()


def dataset_dir(root: str, name: str) -> str:
    return os.path.join(root, name)


def load_manifest(root: str, name: str) -> Optional[Dict]:
    p = os.path.join(dataset_dir(root, name), "manifest.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def save_manifest(root: str, manifest: Manifest) -> str:
    d = dataset_dir(root, manifest.source.name)
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, "manifest.json")
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(manifest.to_json(), f, indent=2, sort_keys=False)
        f.write("\n")
    os.replace(p + ".tmp", p)
    return p


def safe_name(name: str) -> str:
    """A file name made here: letters, digits, '.', '-', '_' only, no leading dot."""
    if not name or name.startswith(".") or any(not (c.isalnum() or c in "._-") for c in name):
        raise ValueError(f"not a safe file name: {name!r}")
    return name


def check_url(url: str, hosts: Iterable[str]) -> None:
    u = urllib.parse.urlparse(url)
    if u.scheme != "https":
        raise ValueError(f"only HTTPS: {url}")
    if u.hostname not in set(hosts):
        raise ValueError(f"{u.hostname} is not one of this dataset's hosts {sorted(hosts)}")


def download(url: str, dest_dir: str, name: str, source: Source, max_bytes: int, known: Optional[Dict[str, str]] = None,
             timeout_s: float = 60.0, retries: int = 3) -> Fetched:
    """``url`` into ``dest_dir/name``, at most ``max_bytes``. Skipped when the file is there with the sha256 ``known``
    (path -> sha256, from the manifest) records for it."""
    check_url(url, source.hosts)
    os.makedirs(dest_dir, exist_ok=True)
    path = os.path.join(dest_dir, safe_name(name))
    rel = os.path.relpath(path, dataset_dir_of(dest_dir, source.name))
    if known and rel in known and os.path.exists(path) and sha256_of(path) == known[rel]:
        return Fetched(rel, url, os.path.getsize(path), known[rel], "(already here)")
    part = path + ".part"
    last: Optional[Exception] = None
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=timeout_s) as resp:
                final = resp.geturl()
                check_url(final, source.hosts)  # a redirect must stay on the dataset's hosts
                length = resp.headers.get("Content-Length")
                if length is not None and int(length) > max_bytes:
                    raise ValueError(f"{url}: {length} bytes is more than the {max_bytes} allowed")
                h = hashlib.sha256()
                n = 0
                with open(part, "wb") as f:
                    while True:
                        block = resp.read(CHUNK)
                        if not block:
                            break
                        n += len(block)
                        if n > max_bytes:
                            raise ValueError(f"{url}: more than the {max_bytes} bytes allowed")
                        h.update(block)
                        f.write(block)
            os.replace(part, path)
            return Fetched(rel, url, n, h.hexdigest(), now_utc())
        except ValueError:
            _remove(part)
            raise
        except Exception as e:  # noqa: BLE001 -- network errors: retry, then give up with the last one
            last = e
            _remove(part)
            time.sleep(2.0 * (attempt + 1))
    raise RuntimeError(f"{url}: {last}")


def dataset_dir_of(path: str, name: str) -> str:
    """The dataset folder that contains ``path`` (its ancestor named ``name``)."""
    p = os.path.abspath(path)
    while os.path.basename(p) != name:
        parent = os.path.dirname(p)
        if parent == p:
            raise ValueError(f"{path} is not inside a folder named {name}")
        p = parent
    return p


def _remove(p: str) -> None:
    try:
        os.remove(p)
    except FileNotFoundError:
        pass


def disk_free_gb(path: str) -> float:
    os.makedirs(path, exist_ok=True)
    return shutil.disk_usage(path).free / 1e9
