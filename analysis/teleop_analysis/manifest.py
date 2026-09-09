from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List

# A manifest written before mitigation stacks existed only has a flat `predictors` list and a
# single `reconciler`. That shape is normalized into one synthetic stack per predictor, named after
# the predictor, holding the playout/arbiter axes at the values that reproduce the pre-Phase-6
# pipeline stand-in exactly.
#
# This value was the string "immediate" until docs/adr/0012-playout-policy-wiring.md. `immediate` is
# now a real registry key whose policy enforces capture-time ordering, which the inline stand-in did
# not -- so keeping the old label would make every legacy manifest claim its run used a policy that
# behaved differently and did not exist when it ran. Runs on either side of that ADR are not
# comparable on a profile where reordering occurs, and this label is what says so.
LEGACY_PLAYOUT_POLICY = "legacy-inline-playout"
LEGACY_ARBITER = "direct"

DEFAULT_SOURCE = "sweep"


@dataclass(frozen=True)
class ResolvedStack:
    name: str
    predictor: str
    reconciler: str
    playout_policy: str
    arbiter: str


@dataclass(frozen=True)
class Manifest:
    experiment_id: str
    git_sha: str
    seeds: List[int]
    stacks: List[ResolvedStack]
    network_profiles: List[str]
    trial_steps: int
    step_interval_ticks: int
    config_path: str
    machine: str
    command: str
    generated_at_utc: str
    source: str
    path: Path

    @staticmethod
    def load(run_dir: Path) -> "Manifest":
        manifest_path = Path(run_dir) / "manifest.json"
        if not manifest_path.exists():
            raise FileNotFoundError(
                f"no manifest.json under {run_dir} -- a result without a manifest is not "
                f"citable (results/CLAUDE.md)"
            )
        raw = json.loads(manifest_path.read_text())

        if "stacks" in raw:
            stacks = [
                ResolvedStack(
                    name=s["name"],
                    predictor=s["predictor"],
                    reconciler=s["reconciler"],
                    playout_policy=s["playoutPolicy"],
                    arbiter=s["arbiter"],
                )
                for s in raw["stacks"]
            ]
        else:
            reconciler = raw.get("reconciler", "")
            stacks = [
                ResolvedStack(
                    name=predictor,
                    predictor=predictor,
                    reconciler=reconciler,
                    playout_policy=LEGACY_PLAYOUT_POLICY,
                    arbiter=LEGACY_ARBITER,
                )
                for predictor in raw.get("predictors", [])
            ]

        return Manifest(
            experiment_id=raw["experimentId"],
            git_sha=raw.get("gitSha", ""),
            seeds=list(raw.get("seeds", [])),
            stacks=stacks,
            network_profiles=list(raw.get("networkProfiles", [])),
            trial_steps=raw.get("trialSteps", 0),
            step_interval_ticks=raw.get("stepIntervalTicks", 0),
            config_path=raw.get("configPath", ""),
            machine=raw.get("machine", ""),
            command=raw.get("command", ""),
            generated_at_utc=raw.get("generatedAtUtc", ""),
            source=raw.get("source", DEFAULT_SOURCE),
            path=manifest_path,
        )
