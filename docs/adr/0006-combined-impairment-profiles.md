# 6. Combined multi-axis network-impairment profiles

## Status

Accepted.

## Context

The catalog now has two families, and neither answers "what if jitter and delay and loss are all
bad at once, at values I choose":

- ADR 0004's 5 bundled presets (`lan`, `50ms-5j`, `150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`,
  `synthetic-burst`) already combine delay+jitter+loss, but only at 5 fixed, hand-picked points.
  There's no way to ask for, say, "100ms delay, 30ms jitter, 1% loss" if it isn't one of the five.
- ADR 0005's isolated families (`jitter-<N>ms`, `delay-<N>ms`, `loss-<N>pct`) exist specifically
  to vary *one* variable while holding the other two fixed — that's the whole point of the
  family, and it must stay that way for the sensitivity charts built from it
  (`impairment_response.py`) to mean anything. It cannot also serve "vary several at once."

A user studying interaction effects (does a predictor tuned for high jitter fall apart once loss
is added on top?) needs a third family: an arbitrary custom point in the same
delay/jitter/loss space, generated on demand rather than hand-picked.

## Decision

### A `combo__` name family, any subset of the three axes, at arbitrary values

`combo__delay-<N>ms__jitter-<N>ms__loss-<N>pct` — segments joined by `__`, each axis segment
optional and appearing at most once, in any order. E.g. `combo__delay-150ms__jitter-20ms` (loss
omitted) or `combo__jitter-30ms__loss-1pct` (delay omitted). The `combo__` prefix cannot collide
with the 5 bundled names, the 3 isolated-axis regexes (none start with `combo`), or the 3
reserved trace names.

**An axis segment that's absent resolves to 0**, not a nonzero baseline. This is the opposite of
ADR 0005's isolated families, and deliberately so: an isolated family holds companions fixed at a
*specific nonzero value* to keep one variable clean against a realistic backdrop; a combined
profile is one composite condition the caller explicitly chose the values for, so there is no
companion to hide — an axis not mentioned simply isn't impaired.

Resolution lives in `Sweep/NetworkProfileCatalog.cs`'s new `TryResolveCombinedProfile`, tried
after `TryResolveIsolatedAxisProfile` in the `default:` case, matching each `__`-split segment
against the same delay/jitter/loss token shapes the isolated resolver already uses. Loss is
Bernoulli (`lossProbabilityAfterDelivered == lossProbabilityAfterLost`) for the same reason as
ADR 0005's loss family — this grammar exposes rate, not burst shape; a bursty combined profile
would need a different, explicit mechanism, not an inferred one.

The resolver itself accepts any 1-or-more axes present — it's a grammar check, not a policy
check. The **generator** that actually produces these names,
`analysis/experiment_builder.py`'s `combined_points`, is what enforces "at least 2 axes," so a
1-axis `combo__` name is never actually produced (use the isolated family for that case instead).

### Generated as a cross-product of small explicit value lists, not min/max/step

Unlike ADR 0005's evenly-spaced ranges (up to 301 points on the delay axis alone),
`combined_points` takes an explicit list of values per axis and returns their Cartesian product.
A min/max/step control here would combine catastrophically (ADR 0005's own ranges multiplied
together are tens of millions of points) — the GUI surfaces this as short comma-separated value
lists ("0,20,60") per axis instead of range controls, and warns before launching a sweep whose
combined-profile count exceeds 200.

### No change to `analysis/teleop_analysis/labels.py`'s axis lookups

`axis_value`/`ordered_profiles_by_axis` (ADR 0005) correctly return `None` for every axis on a
`combo__` name, since it matches none of the three anchored single-axis regexes — a combined
profile isn't a clean single-axis point and must not appear on a jitter/delay/loss sensitivity
line chart. It appears automatically in the axis-agnostic bar charts (`error-cost`, `latency`,
`stack-comparison`), same as any other named profile. `friendly_profile_name` gains a parser for
`combo__` names (mirroring the Core-side grammar) purely for readable figure captions.

## Consequences

- Three profile families now exist, answering three different questions: ADR 0004 ("does it
  survive a realistic bad link"), ADR 0005 ("how sensitive is it to one variable, isolated"),
  this ADR ("what happens at this specific multi-variable combination"). Reports must not mix
  populations across families without saying which kind of profile is being shown.
- No `Registries.cs` entry, same reasoning as ADR 0004/0005 — this isn't a `Contracts/`
  interface implementation.
- No xUnit coverage exists for `NetworkProfileCatalog` (it lives in `Teleop.Eval`, which
  `Teleop.Core.Tests` doesn't reference) — verified instead by running a real sweep against
  hand-built `combo__` names and checking resolved ticks/probabilities by hand, the same way
  ADR 0005's resolver was verified.
