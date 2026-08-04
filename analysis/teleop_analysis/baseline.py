from __future__ import annotations

import warnings
from typing import Optional

from teleop_analysis.manifest import Manifest, ResolvedStack


def find_baseline(manifest: Manifest) -> Optional[ResolvedStack]:
    """Find the (none predictor, snap reconciler, immediate playout, direct arbiter) stack.

    docs/metrics.md rule 1: always report the baseline, even when it obviously loses -- it is
    what makes every other number interpretable. Warn loudly rather than silently proceeding
    without one, matching this project's "no gate that fakes a pass" ethos.
    """
    candidates = [s for s in manifest.stacks if s.predictor == "none"]
    if not candidates:
        warnings.warn(
            f"no baseline stack (predictor='none') found in manifest for "
            f"{manifest.experiment_id!r} -- every comparison below is missing the reference "
            f"point that makes it interpretable"
        )
        return None

    exact = [
        s for s in candidates
        if s.playout_policy == "immediate" and s.arbiter == "direct"
    ]
    if exact:
        if len(exact) > 1:
            warnings.warn(
                f"multiple candidate baseline stacks found for {manifest.experiment_id!r} "
                f"({[s.name for s in exact]}); using {exact[0].name!r}"
            )
        return exact[0]

    warnings.warn(
        f"no stack with playoutPolicy='immediate' and arbiter='direct' found for "
        f"{manifest.experiment_id!r}; using {candidates[0].name!r} as baseline anyway, "
        f"but it is not a true no-mitigation-on-every-axis stack"
    )
    return candidates[0]
