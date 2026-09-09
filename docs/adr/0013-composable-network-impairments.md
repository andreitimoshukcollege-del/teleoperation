# 13. `EmulatedTransport` takes a set of impairments, not a profile struct

## Status

Accepted.

## Context

`Types/NetworkProfile.cs` is a readonly struct with six fields — base delay, jitter, two loss
probabilities, reorder probability, reorder delay — and `Transport/EmulatedTransport.cs` reads every
one of them directly. That shape has carried the project since Phase 3, and it is now the ceiling on
the whole axis.

Adding a genuinely new *kind* of impairment — duplication, bandwidth throttling, corruption,
correlated delay — is not an addition under this shape. It is a widening of the struct, a widening
of `ValidateProfile`, a new branch in `DrawDelayTicks`, and an edit to every construction site
including Unity's. The Unity authoring layer's own doc already names the ceiling verbatim:

> A genuinely new *kind* of disturbance — duplication, bandwidth throttling, corruption — is not a
> Unity change: `NetworkProfile` is a fixed six-field readonly struct and `EmulatedTransport` is
> what would have to grow the behaviour.

The second problem is that **naming and mechanism are fused.** `NetworkProfileCatalog` is the only
way to obtain a profile, so the frozen suite (ADRs 0004, 0005, 0006) is expressed as literals inside
a `switch` and three `Regex` families. There is no way to say "the 0004 numbers, but also
reordering" without inventing a name for it, and no way for a host that simply wants 200 ms of lag
to express that without going through a benchmark catalog it has no interest in.

That fusion is why two parallel vocabularies exist today. `Teleop.Eval` names a frozen profile;
`unity/TeleopVR/Assets/Teleop/Runtime/Bridge/` builds one from Inspector checkboxes through a
per-axis layer of its own. The impairment *engine* is shared — both end up in the same
`EmulatedTransport` — but the vocabulary for describing what to apply is duplicated, and neither
copy can grow an axis without the other.

Two facts about the current implementation shape the decision below.

**Impairment already happens at two distinct stages.** Loss is decided in `Send` (which returns
`false`, so the datagram never reaches the inner transport); delay, jitter and reorder are computed
much later in `DrainInner` → `DrawDelayTicks`. This is not stylistic. `Contracts/ITransport.cs`
promises "Returns false when the datagram will not be delivered — emulated loss, or a full send
queue", and a dropped packet that first occupied an in-flight slot would model a link that
transmitted bytes it was supposed to have lost.

**Common random numbers is currently achieved by hardcoding the draw shape.** `DrawDelayTicks` draws
for jitter and reorder unconditionally, even when those knobs are zero, and its own comment explains
why: "every datagram must consume exactly one draw here regardless of mode, or a trace run and a
parametric run sharing a seed would desynchronize their RNG streams". Exactly three draws per
datagram, in a fixed order, from one shared stream. Composition breaks that by construction — "do
not include a jitter impairment" removes a draw and shifts every subsequent one.

Finally, the cost of acting is at a minimum right now. `results/` contains **no `manifest.json` at
all**, and `docs/setup.md` already records Gate 5 as "closed once, not reproducible now". There are
zero citable results to invalidate.

## Decision

### 1. The emulator takes an ordered array of impairment objects

`Contracts/INetworkImpairment.cs` declares an axis: an `AxisName`, the stages it participates in,
`ApplyOnSend`/`ApplyOnDeliver`, `Bind`, and `Reset`. `EmulatedTransport` gains one constructor —
`(inner, INetworkImpairment[], ulong seed, int maxInFlight)` — and both existing constructors are
deleted.

It lives in `Contracts/` because impairment model is a research axis of this project: the entire
`results/` corpus is indexed by it, and `ls Contracts/` is supposed to answer "what are the
swappable parts of this system?". It is also the Core-declares/host-implements shape root
`CLAUDE.md` blesses, which is what lets Unity hold first-class impairment objects instead of folding
numbers into a struct it does not otherwise care about.

### 2. Two stages, because the transport genuinely has two

`ImpairmentStages` is a flags enum: `Send` (may drop) and `Deliver` (may delay). `EmulatedTransport`
partitions the array once at construction, so an impairment costs no virtual call at a stage it
does not participate in. This is why `Stages` is a declared property rather than "call both methods
and let one no-op".

### 3. Composition is order-independent, and that is enforced by two rules

Impairments mutate a `DatagramFate { long DelayTicks; bool Dropped; }` passed by `ref`. Delay is
**additive** (`+=`, never `=`) and `Dropped` is **monotone** (may be set true, never back to false).
Integer addition commutes and boolean OR commutes, so array order cannot change the outcome for any
impairment obeying those rules. Every impairment shipped here obeys them; a future order-sensitive
one makes order part of the configuration and must say so in its own doc.

### 4. Per-impairment RNG substreams replace the fixed draw shape

Each impairment owns a private `SeededRng`, seeded from `Derive(trialSeed,
FNV1a64(AxisName))`. The determinism contract changes from *the emulator draws a fixed shape* to:

> **Stream isolation.** Each impairment draws only from its own substream, and consumes a constant
> number of draws per datagram per stage, fixed by its type and independent of its parameter values.

This is strictly stronger than what it replaces, not merely different:

| Property | Before | After |
|---|---|---|
| changing an axis's *magnitude* leaves other axes' realizations identical | yes | yes |
| **adding or removing** an axis leaves others identical | **no** | yes |
| an axis at neutral parameters is identical to its **absence** | **no** | yes |
| permuting the array changes nothing | n/a | yes |

The third row is load-bearing: it is what allows the catalog to emit only non-zero axes while
reproducing ADR 0004's numbers exactly.

The hash is hand-rolled FNV-1a over UTF-16 code units, explicitly **not** `string.GetHashCode()`.
.NET Core randomises string hashing per process, so a `GetHashCode`-derived substream would be
non-reproducible across runs while every in-process determinism test still passed — a silent,
total loss of reproducibility. Same discipline as `Registries`' `StringComparer.Ordinal` and
`SeededRng`'s hand-rolled PRNG: pure integer arithmetic that every runtime executes identically.
`Types/SeededRng.cs` itself is not modified.

Stream identity comes from the axis name rather than a hand-assigned constant per type. A
hand-assigned constant has no collision risk from renames, but a copy-paste duplicate silently
aliases two streams — two axes drawing the same numbers, producing correlated loss and jitter, with
nothing wrong-looking in the source. Name-derived identity makes that impossible to hide, because
the constructor rejects duplicate `AxisName`s. The residual hazard — renaming an axis changes its
stream — is caught by a test asserting hardcoded stream-id literals.

### 5. The trace/parametric contradiction dissolves rather than being revalidated

Today the trace constructor throws when `BaseDelayTicks != 0 || JitterTicks != 0`, because synthetic
jitter layered on an already-recorded delay double-models the same variance. Under composition there
is nothing to validate: "a trace supersedes base delay and jitter" becomes "the array contains a
trace impairment and does not contain delay or jitter impairments". **The composition is the
configuration.**

This deletes `TraceMode_Constructor_RejectsNonZeroBaseDelayOrJitter`. Deleting a test needs
justifying: the condition it guarded was an artifact of one struct carrying two mutually exclusive
delay sources. With separate objects, including both is no longer a mistake to catch — it is an
explicit, legal configuration meaning "a recorded link plus an extra fixed hop", and it is covered
by a new test asserting the delays sum.

### 6. `NetworkProfile` is demoted, not deleted

It keeps every field, its constructor, `ExpectedBurstLength` and `ToString()`, and becomes **the
frozen catalog's literal data and the manifest's record of a named link**. Nothing in `Transport/`
consumes it on the hot path any more.

Deleting it was considered and rejected. ADR 0004 records the four frozen profiles' exact numbers
*against these field names*; `NetworkProfileCatalogTests` asserts on them; its doc comment is the
canonical prose definition of what jitter half-width, Gilbert-Elliott burst length and the reorder
knob mean in this project. Removing the type means relocating citable research data for no
mechanical benefit. Its closing line — "trace-driven replay is deliberately not expressible here" —
becomes more accurate, not less: replay is now expressible in the *pipeline*, just not in this
record.

### 7. The old constructors are deleted outright, not kept as a shim

Keeping `(inner, NetworkProfile, SeededRng, int)` would require building an impairment set
internally, which means **the same signature would return different numbers than before**. That is
the most dangerous kind of compatibility shim: a compile error is loud and immediate, while silently
changed results are exactly the failure this project's architecture exists to prevent. Deleting them
makes the compiler enumerate every call site.

The consequence is that this change is atomic across `core/` and `unity/` —
`unity/TeleopVR/Packages/manifest.json` links Core by relative path, so there is no window in which
Unity sits on an older Core. `just bridge-check` is the gate that makes this tractable: it compiles
`Bridge/` against the new Core headlessly, so the break is caught before merge rather than by
opening the editor.

### 8. No `Registries.cs` table for impairments

The five constructors have five different shapes; forcing them into one factory signature produces a
stringly-typed parameter bag. `Registry/CLAUDE.md` already refuses exactly this for
`EmulatedTransport` — "Forcing it into the same factory signature would be worse than not
registering it" — and the same argument applies unchanged.

The by-name selection this axis needs already exists and is richer than a dictionary could be:
`NetworkProfileCatalog` maps a name to a *set*, including three regex-driven families. The catalog
is this axis's registry.

Because the `audit` registry-completeness check reads a hardcoded list of `Contracts/` interfaces, a
new interface is not automatically policed. Rather than leave that silent, `audit` gains a scan
asserting every `INetworkImpairment` implementer's type name appears in
`Transport/NetworkProfileCatalog.cs`. That is the honest analogue of the question the registry check
asks — "can this be reached by name?" — and it doubles as an IL2CPP reachability check, since a type
nothing constructs is a type the stripper may remove. `Registries.cs` itself is untouched, which
also keeps a named cross-machine conflict file out of this change.

### 9. Unity keeps a serializable authoring surface

Unity's `Bridge/Impairments/*` classes are retargeted, not deleted: they become thin
`[Serializable]` shells whose only job is Inspector fields that *construct* Core impairments. This
is not a second pipe — the impairments that run are Core's.

It is forced by Unity serialization. Unity needs parameterless constructors and mutable public
fields; Core validates at construction and holds its parameters readonly. `[Range]`, `[Min]` and
`[Tooltip]` are `UnityEngine` and can never appear in Core. And `[SerializeReference]` in 2022.3
loses managed references when a type is renamed or moved, so binding scene data directly to Core
type identities would make any later Core refactor a silent data-loss event on the Windows box.
`RobotArmProfileData` is the existing precedent for this exact shape.

What does leave Bridge is every decision: clamping moves into Core's per-impairment constructors,
and `NetworkProfileDraft` — which exists only because `NetworkProfile` is a flat six-argument struct
— is deleted. That strengthens the "a Bridge file containing a coefficient is a bug" rule that
folder's own doc already states.

## Consequences

- **The frozen numbers are untouched; the frozen *stream* is not.** Every name in ADRs 0004, 0005
  and 0006 resolves to the same delay, jitter, loss and reorder parameters it always has, asserted
  by a test rather than by review. What changes is the order and source of the random draws that
  consume them. Two runs of `300ms-60j-2loss-bursty` at the same seed, one before and one after this
  change, produce different per-datagram outcomes. Same link, different realization of it.

- **ADRs 0004, 0005 and 0006 are untouched — not amended, not superseded.** They froze which links
  the benchmark suite contains and what numbers define them. This ADR changes only how those numbers
  reach the emulator. `Transport/CLAUDE.md`'s "do not edit or add to the standard set without an
  ADR" is not engaged, because the standard set is neither edited nor added to. ADR 0006's
  correct-in-place precedent is deliberately not used: nothing in those three documents is wrong.

- **No recorded result becomes non-comparable, because none exists.** `results/` holds no
  `manifest.json`; `docs/setup.md` records Gate 5 as not currently reproducible. This ADR spends a
  comparability break that is worth zero today — **and it is worth zero exactly once. After the
  first manifested run lands on `main`, a further change to the draw sequence needs its own ADR and
  a re-run of everything it invalidates.**

- **Distributions are unchanged; only realizations differ.** Identical parameters drive an identical
  model, so aggregate p50/p95/p99 tables remain statistically comparable across this change even
  though no individual datagram's fate matches. A report must not present pre- and post-change runs
  as the same trial, but it need not discard the older ones as measuring a different link.

- **`results/`'s on-disk layout and directory keys are unchanged.**
  `results/<experiment-id>/<UTC>/<stack>/<network-profile>/metrics.csv` still keys on the profile
  name, and `analysis/`'s name parsers keep working verbatim. Names staying frozen is what buys
  this, and it is the concrete reason the naming scheme is not also redesigned here.

- **`just core-check`'s determinism gate does not cover this code.** `verify` replays a golden
  `.tlog` through the recording codec and never constructs an `EmulatedTransport`. The impairment
  pipeline's determinism has always rested on two unit tests comparing two live instances, which
  cannot catch a change in the stream *across builds* — the exact thing this change makes possible.
  A frozen-literal stream-identity test is added for that reason, and the gap itself is now
  documented rather than assumed closed.

- **Adding an impairment kind stops being an ADR-scale change and becomes a `/new-impl`-scale one.**
  That is the point, and also the risk: the frozen suite's authority came partly from adding a knob
  being expensive. `Transport/CLAUDE.md`'s "do not add to the standard set" rule now has to carry
  alone what the struct's rigidity used to carry with it. That rule governs **named profiles in the
  suite**, not **impairment kinds available to compose** — after this ADR those are two separate
  questions and should not be conflated.

- **Duplication is left unimplemented, deliberately.** `ITransport`'s own doc promises datagrams
  "may be delayed, dropped, duplicated, or reordered", and duplication is the one of the four
  nothing implements. `DatagramFate` is one-in-one-out by construction, so duplication does not fit:
  it needs N in-flight slots, a per-copy fate, and a decision about whether a partial fan-out is
  legal under `maxInFlight` back-pressure. The recorded door is a third stage whose method returns a
  copy count, followed by a per-copy deliver loop — a change to the drain loop, not a new field on
  the fate struct.

- **This change is atomic across `core/` and `unity/`**, per decision 7. It cannot be sequenced as
  "Core first, Unity later" the way the two-machine boundary normally assumes, because Core is
  linked by relative path. `just bridge-check` must pass before the merge, and an editor session
  plus one IL2CPP build must follow it.
