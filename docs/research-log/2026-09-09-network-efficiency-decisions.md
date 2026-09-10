# 2026-09-09 — decision record: using the network well, second run

**Organizer:** research-organizer. **HEAD for the whole run:** `76b2a14` (`main`); all three
researcher worktrees branched from it and each confirmed the sha independently. Everything this run
produced is **uncommitted**, by the organizer's standing rule that work is left for a human to
review.

## The question

"Send packets more efficiently and manage network conditions better — use the internet well rather
than treating it as a dumb pipe." Explicitly not prediction.

This question was surveyed once before and all four of its candidates were rejected
(`2026-09-09-network-transport-survey-decisions.md`). That run's framing still binds: **nothing in
Core can reduce one-way delay**; what a candidate can change is the *impact* of loss and jitter. A
second run was worth doing only because two structural facts changed since — ADR 0013 made adding
an impairment kind a `/new-impl`-scale change rather than an ADR-scale one, and loss became
Gilbert-Elliott (genuinely bursty) rather than Bernoulli.

## How it was decomposed, and why

Three candidates, one researcher each, isolated worktrees, identical file scopes. The cut is by
**what the mechanism manipulates**, because that determines which contract it touches and therefore
what it costs here:

1. **Observability** — measure the link. `docs/metrics.md` §3 was defined and emitted by nothing,
   and the previous survey called that "the gating infrastructure for the whole Transport axis":
   every one of its four candidates had its benefit or its cost land in §3 and so could not be
   measured. Made a candidate in its own right rather than a prerequisite nobody owned.
2. **Sender-side behaviour against a bottleneck that can queue** — the one place the run's own
   framing might be wrong, because self-induced queuing delay is the one component of OWD a sender
   can remove. The previous survey named the missing queue model as the blocker and ADR 0013
   removed it.
3. **Path diversity, re-scoped to loss** — the door the previous run explicitly left open when it
   rejected multipath on the *jitter* analysis: "the real effect is on loss, and it is
   uninstrumented."

Each pair is separable by a measurement named in advance: 1 changes no behaviour at all; 2 shows an
effect only where a bottleneck queue exists and none on a loss-only profile; 3 the reverse.

Feasibility was front-loaded in every brief. For this axis the mechanisms are well understood
outside the repo, and the open question is always whether *these* contracts can express or observe
them.

## Approaches considered and not pursued

- **Payload redundancy / FEC, in full generality.** Closed by the previous run as a provable no-op.
  I verified the load-bearing fact myself rather than relaying it — `RigidBodyPlant.Command` rejects
  any frame whose `CaptureTicks` is at or below the last accepted, *entirely*, not partially. That
  generalizes further than its own log claimed: **any scheme that delivers an old frame no earlier
  than a newer frame arrives cannot move the plant.** So temporal spreading inside a datagram, and
  duplicate datagrams sent later, are both no-ops too, not just in-datagram redundancy. Recorded
  here because the generalization is what stops the idea coming back in a new costume.
  Path diversity escapes this argument, and only because a copy on a second path can arrive at a
  step where the first path's copy was lost — which is why candidate 3 was worth running.
- **Retransmission / ARQ, payload compression, `delta-quant`, trajectory/intent codecs,
  network-condition estimation.** All closed by the previous run; none had a changed fact behind it.
  Estimation in particular was redirected to `Buffering/`, and that door is closed because someone
  walked through it — `immediate`/`fixed`/`percentile` exist and are measured.
- **A command-side playout buffer at the robot.** The uplink has no jitter absorption at all, and
  this is probably the most valuable unexplored idea adjacent to this run. Not pursued because it is
  the Buffering axis, not Transport, and because buffering commands means actuating them later — a
  straight latency-for-smoothness trade the playout work has already characterized on the downlink.
  It deserves its own run on the right axis.
- **Transport-layer resequencing** to protect predictors from out-of-order input. Dropped during
  decomposition: holding a datagram until its predecessor arrives *is* a jitter buffer, so it
  collapses onto `Buffering/` rather than being a distinct mechanism. Candidate 1's findings have
  since made the underlying problem much better characterized, and it should be reconsidered there.
- **A codec-axis candidate.** The sweep hardcodes `RawPoseCodec` and the experiment schema has no
  codec axis, so nothing could be measured; and every planned codec is separately closed above.
- **Adding a bursty-loss profile, or any profile, to the frozen suite.** Needed by candidate 3 to
  have statistical power and by candidate 2 to be swept at all. Refused: the frozen suite is what
  every recorded result is measured against, and ADRs 0004/0005/0006 govern it. Both researchers
  were told to price it rather than do it. Note ADR 0013's distinction, which this run leaned on:
  a new *impairment kind* and a new *named profile* are now two separate questions.
- **A cross-candidate head-to-head sweep.** Not run, and this is a genuine gap rather than an
  oversight. The three candidates are not mutually exclusive alternatives competing for one slot,
  and more decisively, **the sweep cannot select a transport**: `Registries.Transports` is
  `Func<int, int, ITransport>`, which cannot express any decorator, and `ExperimentConfig` has no
  transport axis. Candidates 2 and 3 are therefore unrunnable in a sweep by construction. Ranking
  them against each other on numbers was never available, and pretending otherwise would have
  produced a ranking the measurement cannot support.
- **`judge-verdict`.** Not run, for the same reason and following the previous run's precedent.
  There is exactly one row in the only sweep this run could produce — the baseline — and no
  admitted candidate competing with another on numbers. A judge asked to rank one row against
  nothing emits noise that later readers mistake for evidence.

## Verdicts

- **Network observability — BUILT, admitted, and the run's substantive output.**
  `docs/metrics.md` §3 is now emitted: sent/dropped/received and a loss-burst-length *distribution*
  per direction, plus downlink jitter and reorder displacement. Both judges cleared it; the
  invariant auditor read the `OperatorEndpoint` diff specifically and confirmed it is genuinely
  additive, with `owd_*` untouched, and ran `bridge-check` itself. Its characterization of the
  frozen suite is worth more than the instrumentation: `synthetic-burst` has zero loss and five
  byte-identical seeds (so §8 rule 3 is unsatisfiable there, in four already-recorded experiments);
  the bursty and memoryless loss families really do differ in burst shape at similar rates; and on
  `300ms-60j-2loss-bursty` only 58.5% of submitted commands yield a complete latency trace, so every
  `owd_*` percentile recorded there describes the fastest ~69% of trips. Detail:
  `2026-09-09-network-observability.md`. Results: `results/exp-006-network-observability/`.
- **Sender-side rate control against a bottleneck — BUILT and admitted, with a split result that
  matters more than either half.** The framing that "nothing in Core can reduce OWD" is **wrong in
  one specific regime**: at a saturated bottleneck the standing queue is self-inflicted, and a
  sender that admits at the link's rate removes it at zero cost in delivered commands (the greedy
  and rate-limited senders deliver the same 50/s; p50 uplink delay 202 ms → 22 ms). The greedy
  sender's delay is linear in a buffer it does not own. **But the controller that could find that
  rate without being told it cannot**: loss-signalled AIMD recovers 9.7 ms of 180 ms, because a
  full buffer plus a matched rate produces no refusals and therefore no signal, and on a lossy link
  it is worse than useless. Rejected on measurement, by its own author. Detail:
  `2026-09-09-bottleneck-sender.md`. Its feasibility finding is the durable one: a queue is
  **not** expressible as an `INetworkImpairment` — `ApplyOnSend`/`ApplyOnDeliver` take no time and
  `DatagramFate` carries no length, and no widening fixes it because the interface is
  decide-and-emit-once per datagram with no way to hold a datagram across calls. It belongs at the
  `ITransport` seam, which needs no contract change at all.
- **Path diversity scoped to loss — EXCLUDED by admissibility.** Its duplicate suppression is
  payload byte-equality over a bounded ring, so it cannot uphold `TryReceive`'s unconditional "must
  never return the same datagram twice": copies whose arrival spread exceeds the ring depth leak
  through as fresh datagrams, and two legitimately identical payloads are swallowed as an
  unmodelled loss. The class's own doc concedes it is "wrong in general." Its measurements are
  therefore **not ranked and must not be cited as a win**. Its negatives survive exclusion and are
  the valuable part: two paths seeded identically are an exact no-op (measured, not argued — the
  per-axis substream derivation makes independence a seeding decision, not a given), and **the
  frozen suite cannot measure any loss-mitigation candidate at all**, because ±60 ms of jitter
  already causes the plant to reject 62% of arriving commands as stale, so switching 2% bursty loss
  off moves p95 by less than the seed spread. Detail: `2026-09-09-multipath-loss.md`.

## Course changes mid-run

- **One exclusion ground was my own doing, and I discounted it.** The admissibility judge cited the
  missing `Transport/CLAUDE.md` row as an invariant-9 breach for candidate 3. I had forbidden that
  file in every brief for anti-collision reasons. That omission is the organizer's, not the
  candidate's, and it would have been wrong to let a rule I imposed read as a defect in the work.
  The exclusion stands entirely on the independent contract failure above. All three candidates hit
  this, and all three drafted their axis-doc rows in their logs for a human to apply — which is the
  right outcome of the rule, but the rule should say so explicitly next time rather than leaving a
  judge to discover a self-inflicted gap.
- **I ran a follow-up experiment the panel asked for, because it was configuration rather than
  code.** Methodology review found candidate 1's headline — "reordering is 70.1% of arrivals on the
  worst profile, on profiles whose `reorderProbability` is 0.0" — to be confounded: much of it is
  the harness replying to a whole step's batch of commands at one shared send tick, not the link
  inverting packets. It named a decisive test costing two sweeps. I wrote both YAMLs and ran them
  (`results/exp-007-reorder-attribution-5ms/`, `-20ms/`), and the confound is confirmed: on
  `jitter-5ms` the measured reorder rate is 19.7% at a 5 ms step, 13.0% at 10 ms, and **exactly
  0.0% at 20 ms**. Reordering vanishes precisely when send spacing exceeds the jitter range.
  **The figure is a joint property of jitter width and the harness's poll schedule, not a property
  of the link, and it must never be quoted as a fact about networks.** The monotone ladder and the
  exact zero at `jitter-0ms` survive; the attribution does not. A related documentation defect is
  now outstanding: `docs/metrics.md` names this quantity *downlink* reordering while the emitter
  measures end-to-end inversion of an operator-assigned sequence, whose downlink component is
  provably zero at small jitter widths.
- **`dotnet test` is flaky on this machine and I did not touch it.** `just core-check` failed on
  the assembled tree with three `AllocationAssert` failures, all with *fractional* bytes/call, all
  in files no candidate touched. I ran the control myself rather than accept three agents' reports:
  with every candidate test excluded, the flake still fires in 2 of 4 runs. It is pre-existing and
  load-related. `verify`, `audit` and `bridge-check` all pass on the assembled tree. Nothing was
  weakened, and the gate remains unreliable — which is itself a finding, because a gate that cries
  wolf is one a future agent will be tempted to "fix" the wrong way.

## What is left open, and what is blocked on a human

- **Nothing from this run is citable.** Every manifest records `76b2a14`, which does not contain the
  code that produced the runs, because the work is deliberately uncommitted. Committing and
  re-running converts the findings into figures; that is a human's call and takes about a minute.
  Candidate 1's log also cites an earlier results directory that no longer exists — the numbers
  reproduce exactly in the committed-tree re-run, but the path must be corrected before citing.
- **The transport axis is unreachable from a sweep.** Three of the four `ITransport` implementations
  now on disk cannot be selected by name from an `experiments/*.yaml`, and `audit` structurally
  cannot notice, because `RegistryCompletenessAxes` excludes `Transports`. Until a registry shape or
  a small builder type exists, no transport candidate can produce a citable result. Priced at about
  a day by candidate 2, with a reproducibility ADR (per-path seeding) as the slow part.
- **Two harness defects that bias results already recorded**, neither owned by any axis:
  `InsertInFlight`'s delay-correlated ring eviction, now quantified at 31% of arriving replies on
  the worst profile; and end-of-trial truncation, which drops the last in-flight round trip of every
  trial on every profile, scaling with base delay. Both are cheap to fix and both change
  comparability with every existing `results/` directory, which is why no agent touched them.
- **The frozen suite cannot support loss research.** One bursty-loss profile, whose jitter swamps
  the loss it carries; `synthetic-burst`'s seeds are inert. Widening it needs an ADR.
- **ADR 0013's Consequences text over-claims**, naming bandwidth throttling as newly
  `/new-impl`-scale when the queue finding shows it is not.
- **`AllocationAssert` needs to be made robust under load** — serialized or quiesced, not loosened.
- **Axis `CLAUDE.md` rows for four new transports and the new metrics**, drafted verbatim in the
  three candidate logs, plus the first "Tried and rejected" row this axis has ever had
  (`backlog-backoff`, rejected on measurement).
