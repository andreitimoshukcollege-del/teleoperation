from __future__ import annotations

from teleop_analysis.manifest import Manifest


def build_caption(manifest: Manifest, profile: str) -> str:
    """profile + seeds + git SHA -- analysis/CLAUDE.md: an uncaptioned figure is not reusable."""
    seeds = ",".join(str(s) for s in manifest.seeds) if manifest.seeds else "n/a"
    sha = manifest.git_sha[:12] if manifest.git_sha else "unknown"
    return f"profile={profile} | seeds=[{seeds}] | sha={sha}"
