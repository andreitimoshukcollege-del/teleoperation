# 6. Combined multi-axis network-impairment profiles

## Status

Accepted. Revised before any real (committed) use: the generation strategy below was initially a
Cartesian product of per-axis ranges; changed to a lockstep/co-varying walk after review, because
a cross product answers a different (also useful, but not the intended) question and explodes
combinatorially. No citable result ever used the cross-product version, so this ADR was corrected
in place rather than superseded by a new one.

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

### Generated as a lockstep (co-varying) walk across per-axis min/max/step ranges, not a cross product

`combined_points(delay_ms, jitter_ms, loss_pct)` takes a list of values per axis and `zip`s them
— point *i* is `(delay_ms[i], jitter_ms[i], loss_pct[i])` for whichever axes are populated, so a
4-point delay range and a 4-point jitter range together produce 4 combined profiles, not 16. This
answers "how does the system degrade as the whole link gets simultaneously worse" — every checked
axis marching forward together at its own step size — not "every possible pairing of these
values," which is a different, much larger question this ADR does not attempt to answer. Requires
every populated axis to have the same number of points (raises otherwise — there's no principled
way to pair a 7-point axis with a 5-point one).

The GUI builds each axis's list with the same `axis_points(min, max, step)` function
`jitter_points`/`delay_points`/`loss_points` already use, so the "Combined impairments" section
looks and works exactly like the isolated-axis rows above it — the difference is entirely in how
`combined_points` combines the resulting lists (`zip`, not `itertools.product`). Because it's a
lockstep walk rather than a product, the profile count equals the (shared) per-axis point count,
not its square or cube — the GUI's confirmation-before-launch guard for a combined-profile count
over 200 is a much less frequent guard as a result, but stays in place as a backstop.

### No change to `analysis/teleop_analysis/labels.py`'s single-axis lookups

`axis_value`/`ordered_profiles_by_axis` (ADR 0005) correctly return `None` for every axis on a
`combo__` name, since it matches none of the three anchored single-axis regexes — a combined
profile isn't a clean single-axis point and must not appear on a jitter/delay/loss sensitivity
line chart. It still appears in the axis-agnostic per-profile bar charts (`error-cost`, `latency`,
`stack-comparison`), same as any other named profile, for inspecting one specific combined
condition in detail. `labels.py` gains `combined_profile_axes(name) -> Dict[str, float] | None`,
parsing every axis present in a `combo__` name — used both by `friendly_profile_name` for
captions and by the new figure family below for ordering and x-axis tick labels.

### One figure per metric across the whole combined sweep, not one per combined profile

`figures/combined_response.py` (`plot_correction_vs_combined`, `plot_prediction_error_vs_combined`)
plots the whole lockstep sweep as a single line chart per metric — one line per stack, x position
is step index, and the tick label at each position spells out every axis's value at that step
(e.g. `delay=20ms\njitter=10ms`). This is the reason the generation strategy above had to be
lockstep rather than a cross product: a single x-axis can only meaningfully order points that
vary together along one walk, not an unordered bag of every combination. Ordering uses whichever
axis is present in every combined profile in the run — the lockstep construction means every
populated axis increases together, so any one of them gives the correct order. Wired into
`teleop_analysis.cli` as the `combined-response` figure kind, grouped under the GUI's "Line
graphs" checkbox alongside `impairment-response`.

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
