from __future__ import annotations

from teleop_analysis.labels import friendly_profile_name
from teleop_analysis.manifest import Manifest


def _seeds_and_sha(manifest: Manifest) -> str:
    seeds = ",".join(str(s) for s in manifest.seeds) if manifest.seeds else "n/a"
    sha = manifest.git_sha[:12] if manifest.git_sha else "unknown"
    return f"seeds=[{seeds}] | sha={sha}"


def build_caption(manifest: Manifest, profile: str) -> str:
    """profile + seeds + git SHA -- analysis/CLAUDE.md: an uncaptioned figure is not reusable."""
    return f"profile={friendly_profile_name(profile)} | {_seeds_and_sha(manifest)}"


def build_caption_multi_profile(manifest: Manifest) -> str:
    """Same citability information as build_caption, for a figure spanning several network
    profiles at once rather than one -- there's no single "profile=X" to name in that case.
    """
    return _seeds_and_sha(manifest)
