# 2026-09-10 — decision record: repairing the measurement substrate

**Organizer:** none — done directly in the main session, by instruction, after the
network-efficiency run. **Run type:** harness repair and re-baseline; no new algorithm.
**HEAD at start:** `e05becd`, stacked on the still-open PR #44.

## Why this instead of an algorithm

Three research runs in a row ended in the same place — not "the algorithm doesn't work" but "we
can't measure it". The network-efficiency run made it explicit: its most valuable output was a
*characterization of the harness*, not a candidate. On `300ms-60j-2loss-bursty` only 58.5% of
submitted commands completed a latency trace, so every `owd_*` percentile recorded there described
the fastest ~69% of round trips; a headline "70% reordering" was quoted as a property of the link
and is not one; and `just core-check` failed intermittently in files nobody had touched.

The user chose to fix the substrate and re-baseline now rather than preserve comparability. That
was the cheap moment to do it: recorded results already spanned five SHAs, ADR 0013's impairment
rewrite and the playout wiring, so almost nothing was cross-comparable anyway. The cost only grows.

## The three defects, and what each was actually doing

- **End-of-trial truncation.** The trial loop stopped with datagrams in flight; roughly
  `baseDelay / stepInterval` per direction were stranded, so the censoring was delay-correlated
  *across profiles* — the axis most figures compare. It broke §3's own definition: `sent − received`
  is specified as post-acceptance loss "which should therefore be zero — emitted so that 'should' is
  checked rather than assumed", and truncation made that check fail everywhere, by 5 datagrams on
  `lan` and 155 on `300ms-60j`. Now exactly zero in both directions on every profile.
- **In-flight ring eviction.** `InsertInFlight` never reused a slot freed by `TryTakeInFlight`, so
  the ring bounded "submissions in the last 64 calls" — a 640 ms window from capture — rather than
  "traces outstanding". A cliff, not a gradient: 31.4% of arriving replies on `300ms-60j`, zero
  everywhere else. Trace completion of submitted commands went 58.5% → **96.9%**, the residual
  matching real link loss.
- **Reordering reported as one number.** The observed sequence is the operator's, echoed by the
  robot, so an inversion could come from the uplink, from the robot batching replies, or from the
  downlink.

## Approaches considered and not pursued

- **Just making the ring bigger.** Rejected as a fix, taken as a supplement. A larger ring moves the
  cliff and stays silent when it fires; capacity did go 64 → 256, but because with free-slot reuse
  the bound becomes round trips outstanding, which is 72 on `300ms-60j` — above the old 64, so that
  profile would have kept displacing live traces even with the ring logic fixed.
- **Free-slot reuse without an age bound.** Rejected after the diagnosis showed it is a trap: a
  trace whose datagram the link dropped is never claimed, so on a lossy link the ring fills with
  traces for replies that are never coming. It converts a delay-correlated censor into a
  loss-correlated leak. Age-based reclamation is the half that makes reuse safe, and
  `inFlightMaxAgeTicks` makes the ring's real bound — always a time bound — explicit instead of
  faking it with a count that only equals a time window under a fixed cadence.
- **Loosening `AllocationAssert` with a byte tolerance.** Rejected on arithmetic, not on taste: a
  real 1-in-200 regression reports 0.160 bytes/call, inside the observed flake range of
  0.032–0.776, so any threshold that silences the flake silences the regression too.
- **Arguing the reordering mechanism instead of measuring it.** I had a hypothesis (robot reply
  batching) and the arithmetic to support it. Measuring was barely more expensive and produced an
  unambiguous answer where an argument would have produced a plausible one.
- **Making the harness stop *producing* the reordering confound.** A distinct per-reply send stamp
  needs the robot to read a clock per datagram, which invariant 2 forbids. That is an ADR against
  `RobotEndpoint`'s reply cadence, not a metric change, and it is not obviously worth doing —
  batching is what a real robot with one control loop does.
- **Making the transport axis sweepable.** Deferred, and the reasoning is worth keeping because it
  inverted my own recommendation. It would make the sender-pacing result (202 ms → 22 ms) citable —
  but that result is a unit test whose numbers are exact model arithmetic, predicted analytically
  before the code existed, in a regime requiring a link offering under 28 kbit/s to a 48 Hz stream.
  The transport survey had already concluded bandwidth is not scarce here, and the only congestion
  this project has *observed* is a 50 Hz single-threaded poll with depth-1 coalescing — the
  anti-bufferbloat queue. Meanwhile the axis is ADR-shaped: four decorators with four constructor
  shapes (the parameter-bag argument that killed the impairment table), a composition order with a
  live contradiction, and `TransportCapacity = 64` making the sweep measure the emulator's
  back-pressure rather than the link's queue.

## Course changes mid-run

- **I had the reordering mechanism wrong, in two committed research logs and a PR.** I wrote that
  reordering is "a joint property of jitter width and the harness's poll schedule", implying polling
  permutes delivery. `EmulatedTransport` delivers in synthetic-arrival order at any poll rate. The
  empirical claim was right and the mechanism was not; both logs now carry the correction.
- **My verification expectation for the truncation fix was also wrong**, in a way that mattered less
  but is worth recording. I predicted `1 − received/sent` would fall to the true loss rate. It falls
  to *zero*, because §3 counts a refused datagram in `dropped` and not in `sent`. Reading the
  definition rather than my own plan is what caught it.
- **I left six CPU spinners running** after a load test whose PID capture broke on a flattened
  heredoc, and only noticed because I checked. They would have poisoned every measurement after.
  Re-ran the load test from a script file with a trap.

## Verdicts

All four repairs landed and are measured, not asserted:

| | before | after |
|---|---|---|
| `sent − received`, `300ms-60j` (down/up) | 155 / 140 | **0 / 0** |
| trace completion of submissions, `300ms-60j` | 58.5% | **96.9%** |
| `latency_trace_evicted`, all profiles | (unmeasurable) | **0** |
| full-suite runs clean | ~2 in 4 | **10/10**, and 3/3 under load |

The reordering attribution answered its own question decisively. On `jitter-5ms` at a 10 ms step:
total 327, uplink **0**, downlink **0**, batched 620. Neither leg inverted anything; every
inversion is `RobotEndpoint` replying to a whole poll's worth of commands with one `nowTicks`. At a
20 ms step nothing shares a poll and all four counts are zero, which is why the rate looked
poll-dependent. At wider jitter the legs do contribute (`jitter-40ms` at 5 ms: 549 uplink, 1463
downlink of 1900).

## What the re-baseline shows

All ten experiments re-run at `b9c32dd`; every manifest now records a SHA that contains its code,
and carries `drainsInFlightAtTrialEnd`. **Every Buffering conclusion survives.** The largest single
movement is `percentile` on `300ms-60j` *improving* — 58.4 → 53.9 ms mean buffering at 8.44 → 7.30%
loss — because the drain lets its tail complete instead of truncating it. `synthetic-burst`, which
the 56-59% headline rests on, moves by less than 0.1 on every row.

## What is left open, and what is blocked on a human

- **Superseded results are still on disk and must not be cited.** `results/exp-003-playout-baselines/`
  and `results/exp-004-percentile-tracking/` are the pre-rename directories, replaced by
  `exp-008-*` and `exp-009-*`; the 17 `results/scratch-*` directories are replaced by nothing and
  were never citable. `results/` is append-only, so none were deleted. Anything predating
  `b9c32dd` was measured on the broken harness.
- **`net_downlink_jitter_ms` was not audited here.** The reordering half of the sequenced vantage
  was split three ways; the jitter half still reports one number from the same conflated stream and
  probably deserves the same treatment.
- **Goodput, uplink jitter and uplink reordering-at-the-robot** remain uninstrumented. §3 is now
  mostly emitted, not fully.
- **The batching confound is a harness property, not a link property**, and it is now measured
  rather than merely suspected. Whether `RobotEndpoint` should reply on a per-datagram stamp is an
  ADR nobody has written, and it is not obvious it should — a real robot with one control loop
  batches too.
- **`AllocationAssert`'s mechanism is inferred, not profiled.** Nobody has captured the allocating
  stack. If failures return, that inference is the first thing to doubt.
