"""A URDF as Isaac's USD, converted once per content, for both bridges and the renderer (docs/adr/0031 decision 2,
docs/adr/0034 amendment 7).

The conversion's folder is named by :func:`urdf_digest`: the URDF and every mesh it names. The URDF alone is not enough:
re-exported meshes keep their names, and Isaac kept the JetRover's old jaws after they were swapped (amendment 7).
Isaac's importer is imported only when a conversion runs, so the digest is testable without Isaac.
"""

from __future__ import annotations

import hashlib
import os
import re


def urdf_digest(urdf: str) -> str:
    """The sha256 of the URDF and of every mesh file it names, in order: what a conversion is made of."""
    h = hashlib.sha256()
    with open(urdf, "rb") as f:
        text = f.read()
    h.update(text)
    base = os.path.dirname(os.path.abspath(urdf))
    for name in re.findall(rb'filename="([^"]+)"', text):
        path = os.path.join(base, name.decode())
        h.update(name)
        if os.path.isfile(path):
            with open(path, "rb") as m:
                h.update(hashlib.sha256(m.read()).digest())
    return h.hexdigest()[:12]


def convert_urdf(urdf: str, work: str, prefix: str) -> str:
    """The URDF's USD under ``work/<prefix>-<digest>``, converted by Isaac's importer the first time that content is seen."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    out = os.path.join(work, f"{prefix}-{urdf_digest(urdf)}")
    name = os.path.splitext(os.path.basename(urdf))[0]
    usd = os.path.join(out, name, f"{name}.usda")
    if not os.path.isfile(usd):
        os.makedirs(out, exist_ok=True)
        usd = URDFImporter(URDFImporterConfig(urdf_path=urdf, usd_path=out, fix_base=True,
                                              joint_drive_type="force", joint_target_type="position")).import_urdf()
    return usd
