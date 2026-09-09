# 2026-09-09 — decision record: network impairment becomes a composable pipeline in Core

**Organizer:** none — run directly in the main session on the Windows box, no delegation for the
implementation (two agents were used for design only). **Run type:** architecture change.
**HEAD at start:** `91fd615`. **ADR:** this decision changed the architecture, so it graduated to
[`docs/adr/0013-composable-network-impairments.md`](../adr/0013-composable-network-impairments.md),
which holds the argument and the alternatives. This record holds why the work was done at all and
what was decided along the way.

## The question

The user's framing, which reframed a question I had been asking badly:

> Why do we have to have a profile? The profile name should be generated only when we run a sweep
> using the testing harness. The impairment pipe should be separate from that. All it should do is
> import certain impairments, which would have their strength already assigned by Unity or the
> testing, and when you pass through data it should simulate those impairments.

I had been asking whether experiments should author impairment inline or by frozen name. That was
the wrong axis. The right split is that a *name* is a sweep-labelling concern and the *pipe* takes
impairments — which dissolves the tension, because the frozen catalog keeps naming links while
nothing else has to go through it.

## The decision

`EmulatedTransport` takes an ordered array of `INetworkImpairment`. `NetworkProfile` is demoted to
the frozen catalog's data and the manifest's record. Full argument in ADR 0013.

## Approaches considered and not pursued

- **Keeping the old `EmulatedTransport` constructors as a compatibility shim**, so `unity/` would
  keep compiling and the change could be sequenced Core-first, Unity-later. Rejected, and this was
  the one point where the two design agents disagreed. A shim keeping
  `(inner, NetworkProfile, SeededRng, int)` must build an impairment set internally, so **the same
  signature would return different numbers than before** — silently. A compile error is loud; a
  silently changed result is the exact failure this project's architecture exists to prevent. The
  cost of refusing is that Core and Unity become one atomic change, which is the whole reason
  `just bridge-check` had to exist first.
- **Deleting `NetworkProfile`.** Rejected. ADR 0004 records the frozen numbers *against its field
  names*, its doc comment is the canonical definition of what jitter half-width and burst length
  mean here, and it is referenced across ADRs, folder docs, research logs and Python. Removing it
  relocates citable research data for no mechanical benefit.
- **Deleting Unity's authoring layer** in favour of putting Core impairments straight in the
  Inspector. Rejected on a hard constraint: Unity serialization needs parameterless constructors and
  mutable public fields, Core validates at construction and holds parameters readonly, `[Range]` and
  friends are `UnityEngine`, and `[SerializeReference]` loses managed references on a type rename —
  which would make a future Core refactor a silent data-loss event on the Windows box. The layer
  survives as thin authoring shells; every *decision* left it.
- **One shared RNG stream with draws in set order.** Rejected. It preserves nothing useful — adding
  an axis still reshuffles everything downstream of it — and gives up the property that matters.
- **A `Registries.cs` table for impairments.** Rejected: five constructors, five shapes, and the
  catalog already does richer by-name resolution. Replaced with an `audit` reachability check so the
  omission is enforced rather than merely stated.
- **Letting experiment YAML author impairments inline.** Deferred, not rejected. Nothing needs it
  yet, the manifest would have to start recording the full expanded spec for provenance to survive,
  and spending that argument before an experiment needs it is premature.
- **Recording the resolved impairment set in the manifest.** Deferred to a follow-up. It is worth
  doing — and would close an *existing* hole, since `synthetic-burst` records nothing about the
  trace file it loaded, not even a hash — but it is separable and touches the results schema and
  Python.

## Course changes mid-run

- **I asked the user two badly-framed questions before understanding the goal.** Both were rejected
  with a request for clarification; the second time the user asked plainly what "ad-hoc impairment
  spec" meant. The jargon was mine and it obscured a simple choice. Worth recording because the
  reframing that followed produced a better design than either option I had offered.
- **The two-stage constraint was found by reading the hot path, not by design.** An early sketch had
  one `Apply` point. Loss is decided in `Send` and returns false so the datagram never reaches the
  wrapped transport — contractual, and physically right. A single apply-point could not express it.
- **The determinism contract turned out to be more specific than assumed.** `DrawDelayTicks` draws
  for jitter *even in trace mode where the result is discarded*, purely to keep seeds aligned. That
  is precisely what composition breaks, and finding it early is what made per-impairment substreams
  a designed decision rather than a discovered bug.

## Verdicts

- **Built and green:** the contract, five impairments, the substream derivation, the emulator
  rewrite, catalog resolution, the sweep call sites, the Unity retarget, and an `audit` check.
- **641 Core tests pass**, including every ported test. `Gate3_SyntheticOneWayDelayOf137Ms_...`
  passes **bit-identically**, which was the designated stop-condition: it is pure arithmetic on a
  drawless link, so if it had moved the refactor would have changed delay computation rather than
  its plumbing.
- **The frozen numbers are preserved**, asserted by a new equivalence test over ADRs 0004/0005/0006
  rather than by review.
- **Both new gates were verified to fail, not just to pass.** `bridge-check` exits 1 on the
  pre-change bridge; the `audit` reachability check exits 1 on a deliberately unreachable
  impairment. A gate nobody has watched fail is not known to work.

## What was left open

- **Unity has not compiled or run any of this.** `bridge-check` proves Bridge agrees with Core's
  API; it cannot see serialization, Inspector round-tripping of the nested axis objects, scene
  wiring, or IL2CPP. An editor session and one Quest build are still required.
- **Every recorded result is now non-reproducible bit-for-bit**, because substreams changed the draw
  sequence. This cost nothing today — `results/` holds no `manifest.json` at all and `docs/setup.md`
  already records Gate 5 as not reproducible — **and it will never be this cheap again.** ADR 0013
  states that after the first manifested run lands on `main`, a further change to the draw sequence
  needs its own ADR and a re-run of what it invalidates.
- **Duplication remains unimplemented.** `ITransport` promises datagrams "may be delayed, dropped,
  duplicated, or reordered" and it is the one of the four nothing implements. `DatagramFate` is
  one-in-one-out by construction; ADR 0013 records the door (a third stage returning a copy count).
- **The manifest still records only profile names.** See the deferred item above.
