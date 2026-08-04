# 5. Isolated single-variable network-impairment profiles for sensitivity charts

## Status

Accepted.

## Context

The five profiles from ADR 0004 are deliberately realistic bundled presets — each one moves
delay, jitter, and loss together to represent a plausible real link. That's the right design for
"does this predictor survive a bad connection," but it makes them useless for a different,
narrower question `analysis/` now needs to answer: "as jitter alone increases, how does
correction cost change." Plotting bundled presets on a single-variable axis would silently
attribute delay's and loss's effects to whichever variable happens to be on the x-axis.

Only two of the four parametric profiles even have nonzero loss, and they have different burst
lengths (`ExpectedBurstLength` ≈1.005 for `150ms-20j-0.5loss` vs. ≈3.33 for
`300ms-60j-2loss-bursty`) — a loss-rate axis built from them would conflate rate with burst
shape, exactly the kind of two-variables-in-one-number result `Types/NetworkProfile.cs` already
argues against for reorder vs. jitter (its own doc comment: `ReorderProbability` is "kept as an
independent knob rather than left to emerge from jitter variance... conflating them into one
parameter would make the result uninterpretable").

## Decision

### Three new profile families, each isolating one variable

Each family holds the other two `NetworkProfile` parameters fixed so its axis is actually clean —
a genuine sensitivity sweep, not a relabeled preset comparison:

| Family | Varies | Held fixed | Points (ms or %) |
|---|---|---|---|
| `jitter-<N>ms` | jitter | delay=50ms, loss=0/0 | 0, 5, 10, 15, 20, 30, 40, 50, 60 |
| `delay-<N>ms` | delay | jitter=5ms, loss=0/0 | 0, 25, 50, 75, 100, 150, 200, 250, 300 |
| `loss-<N>pct` | loss (Bernoulli: `lossProbabilityAfterDelivered == lossProbabilityAfterLost`, so `ExpectedBurstLength` stays ≈1 across every point — no burst-shape confound) | delay=100ms, jitter=10ms | 0, 0.25, 0.5, 1, 1.5, 2, 3, 4, 5 |

27 profiles total. None carry reordering, matching every existing profile.

The jitter family's fixed companions (delay=50ms, loss=0/0) are the same as the existing
`50ms-5j` preset's non-jitter parameters. That overlap is harmless and not special-cased —
`jitter-5ms` and `50ms-5j` are numerically near-identical points from two different,
independently-purposed families.

### Resolved by pattern, not by 27 hand-written cases

`Sweep/NetworkProfileCatalog.cs` gets one new private resolver, `TryResolveIsolatedAxisProfile`,
matching `^jitter-(\d+(?:\.\d+)?)ms$` / `^delay-(\d+(?:\.\d+)?)ms$` / `^loss-(\d+(?:\.\d+)?)pct$`
and constructing the `NetworkProfile` from the parsed number plus that family's fixed companions —
tried before falling through to "unknown profile," alongside (not replacing) the existing named
switch. This avoids 27 copy-pasted `case` blocks, at the cost of the family's parameters living in
code as a formula instead of ADR-0004-style individual literals — acceptable here because the
*rule* (one family, one fixed companion set, evenly spaced points) is the citable decision, not
each individual value.

### `analysis/` axis lookups mirror this: hand-curated for legacy presets, parsed for the new families

`labels.py` keeps a small hand-curated jitter/delay table for the 4 legacy presets (unchanged
from before this ADR) and adds a regex parse of the new systematic names for the rest — same
reasoning as the C# side, and it means adding a 10th jitter point later never requires touching
`labels.py` by hand.

## Consequences

- This is a second, deliberately different way of generating network conditions from ADR 0004's:
  ADR 0004's profiles answer "does it survive a realistic bad link," this ADR's profiles answer
  "how sensitive is it to one specific variable." Reports should not mix the two families in a
  single "solutions vs. network profile" comparison without saying which kind of profile is being
  shown — they're not the same population.
- `experiments/exp-002-impairment-sensitivity.yaml` is the sweep that generates real, committed
  data for these 27 profiles. Its `results/` directory is what the jitter/delay/loss line charts
  in `analysis/teleop_analysis/figures/impairment_response.py` are actually citing when they show
  more than the 4 legacy points.
- Extending any family (a finer step, a wider range) is ordinary maintenance under this ADR's own
  rule, not a new ADR, as long as the fixed companions and family shape stay the same — only
  changing which variable is fixed, or reintroducing bursting on the loss axis, would need one.
