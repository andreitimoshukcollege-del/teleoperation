from __future__ import annotations

from typing import Dict

from teleop_analysis.baseline import find_baseline
from teleop_analysis.manifest import Manifest


def classify_stacks(manifest: Manifest) -> Dict[str, str]:
    """Classify every stack relative to the baseline: 'baseline', 'single-axis', or 'combined'.

    A stack differing from baseline in exactly one of (predictor, reconciler, playoutPolicy,
    arbiter) is a "separately" study of that axis; differing in more than one is a "together"
    study of a named combination. Lets figures group "tested separately" vs "tested together"
    without any extra manifest field -- it's fully derived from the resolved stack list.
    """
    baseline = find_baseline(manifest)
    result: Dict[str, str] = {}
    for stack in manifest.stacks:
        if baseline is None:
            result[stack.name] = "unknown"
            continue
        if stack.name == baseline.name:
            result[stack.name] = "baseline"
            continue
        diffs = sum((
            stack.predictor != baseline.predictor,
            stack.reconciler != baseline.reconciler,
            stack.playout_policy != baseline.playout_policy,
            stack.arbiter != baseline.arbiter,
        ))
        result[stack.name] = "single-axis" if diffs <= 1 else "combined"
    return result
