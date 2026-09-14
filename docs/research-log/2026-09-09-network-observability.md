# 2026-09-09 — network observability: instrumenting `docs/metrics.md` §3

**Agent:** researcher (candidate: network observability), one of three in a Transport-axis run.
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a62cf35ca82b48faf`
**Branch:** `worktree-agent-a62cf35ca82b48faf`
**HEAD at start:** `76b2a149270bbcd2e7d0e4050b1e96e123d73940` (`76b2a14`, `main`, "Pre-approve git
wholesale; keep merging on ask (#43)") — confirmed with `git rev-parse HEAD` inside the worktree
before anything else.

> **Provenance note, so nothing here is overstated.** The log was appended to continuously, in the
> order the sections appear, and **the hypothesis below was written before any code was read in
> depth or written** — that is the point of it. This metadata block and the seed self-check table
> are the exception: my first attempt to create the file was rejected by the shell guard and I did
> not notice until the end of the run, so they were written last. Everything from "Hypothesis"
> onward is in original order and none of it was back-filled.

## Seed self-check

| Check | Result | Note |
|---|---|---|
| HEAD sha matches the sha the organizer named (`76b2a14`) | PASS | `git rev-parse HEAD` = `76b2a149270bbcd2e7d0e4050b1e96e123d73940` |
| Working in my own worktree, not the main checkout | PASS | cwd is `.claude/worktrees/agent-a62cf35ca82b48faf` |
| Branch is mine | PASS | `worktree-agent-a62cf35ca82b48faf` |
| `dotnet test` green before I changed anything | PASS | 651 Core tests at HEAD; see the flakiness note under Verification |
| `verify` and `audit` green before I changed anything | PASS | both re-run after every edit |
| `git-lfs` filter present (root CLAUDE.md requires it before any tree-modifying command) | PASS | version 3.8.0; no tree-modifying git command was run in any case |
| Both new `.cs.meta` guids unique across the whole tree | PASS | every `.meta` in the tree grepped before use |
| `docs/metrics.md` change is additive only | PASS | diffstat: 70 insertions, 0 deletions |
| No file outside my stated scope modified | PASS | status lists exactly the 11 expected paths |
| No `unity/`, `robot/` or existing `results/` file modified | PASS | one new `results/` directory appended, nothing edited |
| Nothing committed, pushed, tagged or amended | PASS | branch left dirty by instruction |
| No hardware recipe invoked (`move-arm`, `clocksync-check`, `deploy-robothost`) | PASS | none was needed; none was attempted |

## Verdict, in one line

**§3 is instrumentable from inside Core — every quantity it defines is now either emitted (loss
rate, burst-length distribution, reordering rate and max displacement, downlink RFC 3550 jitter),
derived from metrics that already exist (the OWD IQR), shown to be degenerate on this system
(goodput), or shown to need a wire-format change (uplink RFC 3550 jitter) — and the frozen profile
suite is now characterized rather than assumed** — the headline being that reordering, which
`reorderProbability = 0.0` implies is absent, is in fact **70% of downlink arrivals** on
`300ms-60j-2loss-bursty`. The one quantity that is not instrumentable here is uplink RFC 3550
jitter, which needs a wire-format change and is reported as such rather than approximated.

## Hypothesis (written before any code)

**H.** The network conditions this platform imposes are measurable from inside `Teleop.Core`,
deterministically, allocation-free and without a wire-format change, for the majority of
`docs/metrics.md` §3 — and the frozen profile suite can then be *characterized* by those
measurements rather than by its declared parameters.

Concretely, per direction (uplink = operator→robot, downlink = robot→operator):

- **Loss rate** and **loss burst-length distribution** are exactly observable at the *sender's*
  `ITransport.Send` boundary, because `GilbertElliottLossImpairment` declares
  `ImpairmentStages.Send` and `EmulatedTransport.Send` returns `false` for precisely the datagrams
  it destroyed. Nothing downstream of that boundary — no buffer, no playout policy — can
  contribute, which is how §3's "late-arrival loss must never be added to network loss" is
  satisfied structurally rather than by care.
- **Reordering rate and max displacement** are observable at the *receiver*, but only where a
  sequence number is in scope, which `ITransport` deliberately is not (`DatagramFate` carries no
  sequence; `TryReceive` yields opaque bytes).
- **RFC 3550 interarrival jitter** is observable at the receiver only where a sender timestamp
  *and the sender's tick rate* for the same datagram are in scope.
- **IQR of one-way delay** requires no new emission at all: `owd_uplink_ms` / `owd_downlink_ms`
  already exist and an IQR is an analysis-time reduction of them.
- **Goodput** has no redundancy and no retransmission to exclude in this system, so it degenerates
  to delivered-datagram rate × frame size and needs no new emission either.

**Predicted characterization** (stated now so it can be wrong):

1. `300ms-60j-2loss-bursty` will show measured send-refusal loss near its declared 2% steady state
   and a measured burst-length distribution with mean near 3.33 (= 1/(1−0.7)), p50 = 1, and a tail
   reaching 10+.
2. `synthetic-burst` will show **exactly zero** losses on both directions and identical counts
   across all five seeds.
3. Reordering will be **non-zero on jittered profiles and exactly zero on unjittered ones**,
   despite `reorderProbability = 0.0` everywhere, because `EmulatedTransport` delivers by earliest
   synthetic arrival and a jitter draw can invert two adjacent datagrams.
4. `net_<dir>_received` will equal `net_<dir>_sent` on every profile — i.e. the emulator destroys
   nothing that no impairment asked it to destroy. If it does not, `EmulatedTransport`'s
   `maxInFlight` back-pressure or `LoopbackTransport`'s full-ring refusal is adding loss that
   appears in no manifest.

**Falsifiers.** Each is checked and reported whether or not it fires:

- **F1 (design falsifier, per the brief).** If the only correct place to compute a given §3
  quantity is outside Core, or it requires a wire-format change, I say so and emit nothing for it
  rather than approximating. Specifically anticipated: uplink RFC 3550 jitter, because
  `CommandFrame` carries no operator `TicksPerSecond` and `docs/adr/0008` states the omission is
  deliberate ("conversion is operator-side only, so the robot never needs the operator's rate").
  Without the rate the robot cannot normalize the operator's send stamps, and RFC 3550's
  difference-of-differences cancels a clock *offset* but not a clock *rate*.
- **F2.** If instrumenting requires changing control flow, RNG draw order, or an existing emitted
  metric in `OperatorEndpoint`/`RobotEndpoint`, I stop and report that as the finding (brief's
  additive-only constraint).
- **F3.** If the observation cannot be made allocation-free or breaks determinism (`verify`
  fails), the design is wrong and I say so.
- **F4 (characterization falsifier).** If prediction 4 fails — measured deliveries fewer than
  measured accepted sends — then loss measured at the sender is *not* the whole story on this
  harness, and every loss number in the repo's logs that was reasoned from profile parameters is
  suspect. That is a finding, not a bug to paper over.
- **F5.** If prediction 3 fails in the direction "reordering is zero everywhere", the survey's
  "reordering is pervasive" claim — on which a `Prediction/`-axis lead about `double-exp`
  discarding a third of its input rests — is wrong, and I report that.

**What would make this candidate worthless:** if the quantities can only be computed from
*completed `LatencyTrace`s*, because `OperatorEndpoint.InsertInFlight` still overwrites an
occupied ring slot without checking, and that eviction is delay-correlated. Avoiding the trace
path entirely is therefore a design requirement, not a preference.

## Prior work consulted (candidate generation only — never evidence about this system)

- **RFC 3550 Appendix A.8 / §6.4.1** (https://www.rfc-editor.org/rfc/rfc3550.txt), fetched
  2026-09-09. Proposes: interarrival jitter as an exponentially-weighted mean deviation of the
  *relative transit time*, `D(i,j) = (R_j − S_j) − (R_i − S_i)`, updated as
  `J += (|D(i−1,i)| − J)/16`. Claims: a single-pole filter with gain 1/16 "gives a good noise
  reduction ratio while maintaining a reasonable rate of convergence". Conditions it was specified
  under: RTP media streams, i.e. a **fixed-rate sender** whose timestamp clock rate the receiver
  learns out-of-band from the payload type, arbitrary and *unsynchronized* wall clocks at the two
  ends. That last point is the load-bearing one for this repo: `D` is a difference of differences,
  so a constant clock **offset** cancels exactly — but a clock **rate** mismatch does not, and RTP
  only gets away with that because the rate is negotiated. This repo negotiates the *robot's* rate
  (`RobotStateFrame.TicksPerSecond`, `docs/adr/0008`) and deliberately not the operator's, which is
  exactly why the downlink is instrumentable and the uplink is not. I implement the algorithm as
  specified, at gain 1/16, in milliseconds.
- I did **not** find prior work worth using for the reordering definition and did not spend more of
  the window looking; `docs/metrics.md` §3 already fixes it ("fraction arriving out of sequence,
  plus max displacement") and the natural realization — an arrival whose sequence is below the
  running maximum, displacement = `maxSeen − seq` — is what that sentence says. Recorded so the
  next run does not re-search it.
- **Not consulted, deliberately:** anything on active network measurement (probes, packet pairs).
  This candidate is *passive* observation of a stream the system already sends; adding probe
  traffic would change the thing being measured and is a different candidate.

## Design decisions, and the alternatives that lost

### Where the observer lives

The organizer named three candidate sites. My determination: **§3 has no single vantage, and
pretending otherwise is the only way to get it wrong.** The five quantities split cleanly by what
information the site holds, so the design is two collaborating pieces, not one.

| §3 quantity | Needs | Only site that has it |
|---|---|---|
| loss rate | a send outcome | `ITransport.Send`'s return value |
| loss burst-length | consecutive send outcomes | same |
| reordering rate + max displacement | a sequence number | a caller that has decoded a frame |
| RFC 3550 jitter | a sender stamp **and its tick rate**, same datagram | a caller that has decoded a frame |
| IQR of OWD | `t_send`, `t_recv`, one clock domain | already emitted as `owd_*_ms` |
| goodput | delivered useful bytes | degenerate — see below |

**(a) A `MeasuredTransport`-style `ITransport` decorator — chosen, for the loss half only.**
It sees `Send` return `false`, which on this stack *is* emulated loss: `GilbertElliottLossImpairment`
declares `ImpairmentStages.Send`, and `EmulatedTransport.Send` returns false for exactly the
datagrams it destroyed, never handing them to the wrapped transport. So a refusal count is a loss
count with no estimation and no sequence bookkeeping. It has a property no other site has:
**it is structurally incapable of confusing link loss with a buffer's discard**, because no buffer
exists at that boundary. §3 is emphatic about that distinction and this makes it hold by
construction rather than by care.

One placement detail decides whether it measures anything at all: it must wrap **outside**
`EmulatedTransport`. Inside, it would see only the loopback's queue and never a lost datagram.

A second, unplanned property emerged and is worth recording, because it removes the vantage
problem for loss entirely: **in this pipeline one `ITransport` object is one direction of the
link.** `SweepCommand` builds one `uplink` and one `downlink`; the operator endpoint calls `Send`
on the uplink and the robot endpoint calls `TryReceive` on that same object. So a single decorator
per direction sees both the sender vantage and the receiver vantage, and `sent − received` is loss
inflicted after acceptance — a quantity nobody could previously see.

The organizer's stated doubt about this site is confirmed exactly: `DatagramFate` carries no
sequence (its doc says the absence is deliberate), `TryReceive` yields opaque bytes, and there is
no per-datagram send tick. **Per-datagram delay, reordering and jitter are not recoverable here**,
and I did not try to recover them by parsing payloads — a transport that decoded payloads would be
a codec, would need updating for every future codec, and would break the moment it wrapped a link
carrying something else.

**(b) A pure Core collector fed `(sequence, sendTicks, arrivalTicks, bytes)` — chosen, for the
sequence half.** Implemented as `Transport/NetworkObserver.cs`, which owns *all* the metric logic;
the decorator is a thin adapter that feeds it the two transport-vantage events. Splitting the
logic across the two sites was rejected: it would have put half of §3's definitions in a class
that cannot state them.

**What it is fed, and what it is deliberately not fed.** The organizer suggested `LatencyTrace`,
which carries `Sequence` plus all four stamps. **I rejected the trace path outright**, and this was
the single most consequential design decision here. `OperatorEndpoint.InsertInFlight` still
overwrites an occupied ring slot without checking (verified in the source at this HEAD — the
`TryReceiveState` fix that landed since the survey stops the *estimator* being censored, but the
ring still evicts, so trace completion is still censored). That eviction is delay-correlated: it
discards the slowest round trips first. **Any loss, delay or jitter statistic computed from
completed traces would therefore be censored hardest exactly where the link is worst**, which
would invert the sign of the result on the profiles that matter. The observer is instead fed from
`RobotStateFrame` at the moment of decode, before any trace lookup, so the ring cannot touch it.

**(c) Something at the endpoints, using `CommandFrame.Sequence` — chosen only in the narrow form
above, and explicitly *not* at `RobotEndpoint`.** Reasons, in order of weight:

1. **It would buy a replicate, not new information.** `SweepCommand` gives both directions the
   same profile with independent RNG substreams, so uplink and downlink reordering are draws from
   one distribution. A second measurement would improve precision, not answer a new question.
2. `RobotEndpoint`'s own doc states it holds no `IMetricSink` because ADR 0002 puts reporting
   operator-side. That reasoning is about clock-domain conversion, which sequence counting does
   not need — so this is a defensible edit, not a forbidden one. But it is an argument I would be
   making unsupervised about a documented architectural decision, for a replicate.
3. Uplink jitter is underivable there regardless (see F1 below), so the robot-side vantage could
   only ever deliver half of what the operator-side one delivers.

Recorded as a cheap, well-defined follow-up rather than done.

### Which §3 quantities are honestly derivable, and which are not

**Derivable and now emitted:** loss rate (both directions, sender and receiver vantage), loss
burst-length distribution (both directions), reordering rate and max displacement (downlink),
RFC 3550 interarrival jitter (downlink).

**Derivable but not emitted, deliberately:**

- *IQR of one-way delay.* Needs no new emission at all — `owd_uplink_ms` and `owd_downlink_ms`
  already exist, and an IQR is an analysis-time reduction of them. A second emitter could only
  disagree with them. Written into §3 as derived, following the `correction rate` precedent in §5.
  **Caveat that must travel with it:** `owd_*_ms` is emitted only on the trace-completion path, so
  it *is* exposed to the in-flight ring eviction described above. The §3 IQR is therefore the one
  §3 quantity that inherits that censoring. This is a reason to prefer the new jitter metric over
  the OWD IQR on the hard profiles, and it is not a reason to change `owd_*_ms`, which is a
  baseline.
- *Goodput.* Its definition excludes redundancy and retransmission. Nothing here retransmits
  (`ITransport` callers are forbidden to retry on a refusal) and no shipped codec sends
  redundancy, so **there is nothing for the definition to exclude and it degenerates to
  `count(net_<dir>_received) × 73 bytes / trial duration`** — a linear restatement of the delivery
  count. Emitting a constant-times-a-count under its own name would put a metric in the file that
  looks like an independent measurement and is not. Written into §3 as degenerate, with the
  condition under which it stops being degenerate (a redundant or variable-length codec) and the
  warning that its emitter must then count *useful* bytes, not bytes on the wire, or the metric
  would reward what it is defined to exclude.

**Not derivable here — F1 fired, once, exactly where predicted:**

- ***Uplink* RFC 3550 interarrival jitter.** `D` is a difference of differences, so a constant
  clock **offset** cancels — that is the whole reason RTP can use it between unsynchronized ends.
  A clock **rate** does not cancel, and RTP only gets away with that because the receiver learns
  the sender's timestamp rate out of band from the payload type. This repo has the exact analogue
  on the downlink (`RobotStateFrame.TicksPerSecond`, ADR 0008) and **deliberately not on the
  uplink**: ADR 0008 records "the uplink `CommandFrame` deliberately has no counterpart field:
  conversion is operator-side only, so the robot never needs the operator's rate." Computing
  uplink jitter would mean assuming the two ends tick alike, which is precisely the bug ADR 0008
  was written for (a 10 MHz operator against a 1 GHz Jetson inflated every measured RTT 100×).
  **This is a wire-format change, not an implementation, so per my own falsifier I emitted nothing
  for it** and said so in `docs/metrics.md` §3. The code encodes it rather than commenting it:
  `NetworkObserver.ForUplink` supplies no jitter name, and a non-positive sender tick rate
  suppresses jitter instead of assuming one.
- ***Uplink* reordering** is a weaker case: merely unwired, not impossible — the sequence is
  already on the wire and needs no new field. Recorded as such in §3 so the two are not confused.

### Registry entry: none, and why

`Registries.Transports`' factory shape is `(maxPayloadBytes, capacity) -> ITransport`.
`MeasuredTransport` is a decorator over another `ITransport` plus a `NetworkObserver`, which that
shape cannot express — the identical situation `Registry/CLAUDE.md` already documents for
`EmulatedTransport`, which is unregistered for exactly this reason. `NetworkObserver` implements no
`Contracts/` interface at all and is not a family of competing implementations, so no table
applies to it either. **Nothing was added to `Registries.cs`.** `AuditCommand` excludes
`Transports` from registry-completeness checking, so this does not silently defeat a gate; `audit`
passes and would still pass if a transport went missing, which is a pre-existing gap noted by the
previous survey and still unowned.

Both types are directly referenced by name from `SweepCommand` and from their tests, so IL2CPP
reachability — the actual reason invariant 5 exists — is satisfied by construction.

## What was built

| Path | What |
|---|---|
| `core/Teleop.Core/Transport/NetworkObserver.cs` (+ `.cs.meta`, guid `467108b806e24c26919f3770b3ba4ab0`) | the §3 collector: all metric logic, three event methods, allocation-free, no clock read |
| `core/Teleop.Core/Transport/MeasuredTransport.cs` (+ `.cs.meta`, guid `31a9a800fe994b4dbd752625a864f96c`) | transparent `ITransport` decorator feeding the two transport-vantage events |
| `core/Teleop.Core.Tests/Transport/NetworkObserverTests.cs` | 21 tests, every one against a hand-constructed stream with known ground truth |
| `core/Teleop.Core.Tests/Transport/MeasuredTransportTests.cs` | 11 tests: transparency, and loss against the declared generative model |
| `core/Teleop.Core/Pipeline/OperatorEndpoint.cs` | **additive only** — one optional trailing constructor parameter and one guarded call |
| `core/Teleop.Eval/Sweep/SweepCommand.cs` | wiring: two decorators, one sequenced observer |
| `docs/metrics.md` §3 | metric-name lines, **70 insertions and 0 deletions** (`git diff --stat`) |
| `experiments/exp-006-network-observability.yaml` | the characterization sweep |

Both guids were checked against every `.meta` in the tree before use; no collision.

### The `OperatorEndpoint` edit, against the additive-only constraint (F2: did not fire)

The diff is: one `using`, one nullable field with its doc, one optional trailing constructor
parameter defaulting to `null`, one assignment, and one `if (_downlinkNetworkObserver != null)`
call placed immediately after `_stateCodec.TryDecode` succeeds and before every existing branch.
No existing statement was moved, reordered or deleted. No control flow changed — the guarded call
has no `continue`, `return` or `throw` reachable in normal operation. No RNG is touched anywhere in
this class. No existing emitted metric changed name, value, count or stamp. The parameter is
optional, so every existing construction site (tests, `verify`'s golden replay, both Unity bridges)
compiles and behaves identically; `just bridge-check` confirms the bridges still compile.

The observer sees every reply the link delivered, including a duplicate and including one whose
in-flight trace has been evicted, because it runs before the branch that skips those.

### Metric names added to `docs/metrics.md` §3 — the complete list

Additive only; no existing definition anywhere in the file was altered, rescoped or reworded, and
no other section was touched.

| Name | Unit | Emitted | Under which §3 definition |
|---|---|---|---|
| `net_uplink_sent` / `net_downlink_sent` | count | 1 per datagram the sending transport accepted, at the send | Loss rate |
| `net_uplink_dropped` / `net_downlink_dropped` | count | 1 per datagram it refused, at the send | Loss rate |
| `net_uplink_received` / `net_downlink_received` | count | 1 per datagram the receiving transport delivered, at `t_recv` | Loss rate |
| `net_uplink_loss_burst` / `net_downlink_loss_burst` | datagrams | 1 per completed run of consecutive refusals, value = run length, at the acceptance that closed it | Loss burst-length distribution |
| `net_downlink_reorder_displacement` | dimensionless | 1 per out-of-order arrival, value = `highestSeen − seq` (wrap-safe), at `t_recv` | Reordering rate (both halves: rate = count/received, max displacement = max of the samples) |
| `net_downlink_jitter_ms` | ms | 1 per received datagram after the first, at `t_recv` | Jitter, first bullet |

Plus two prose statements with no name: the OWD-IQR bullet is derived from `owd_*_ms`, and goodput
is degenerate. And two explicit non-names with their reasons: no `net_uplink_jitter_ms`
(structurally impossible without a wire change), no `net_uplink_reorder_displacement` (merely
unwired).

## Results

**Run:** `results/exp-006-network-observability/20260910-021551Z/`
(`manifest.json` `gitSha` = `76b2a149270bbcd2e7d0e4050b1e96e123d73940`).
**Config:** `experiments/exp-006-network-observability.yaml` — 12 profiles × 5 seeds × 500 steps at
10 ms, stack pinned to the baselines `none` / `snap` / `immediate`. Nothing algorithmic varies; the
only varying axis is the network profile, which is what makes this a characterization rather than a
comparison. Every metric reported below is emitted by the transport layer or by the arrival path,
and no predictor, reconciler or playout policy can influence any of them.

**Read this before using any number here.** The `gitSha` in that manifest is `76b2a14`, but the
code that produced the run is **uncommitted in my worktree**. By `results/CLAUDE.md`'s own rule a
figure is citable when it traces to a manifest with a reachable SHA, and this one does not yet.
Every number below is a *finding*, not yet a citable figure; re-run after the diff lands.

### Headline table (pooled over 5 seeds; 2500 uplink sends per profile)

| profile | up sent | up drop | loss % | burst n | burst mean | burst max | dn recv | reorder % (per-seed range) | displ p50/p95/max | jitter p50/p95/p99 ms |
|---|---|---|---|---|---|---|---|---|---|---|
| `lan` | 2500 | 0 | 0.00 | 0 | — | — | 2490 | 0.0 (0.0–0.0) | — | 0.66 / 0.83 / 0.88 |
| `jitter-0ms` | 2500 | 0 | 0.00 | 0 | — | — | 2450 | 0.0 (0.0–0.0) | — | 0.00 / 0.00 / 0.00 |
| `50ms-5j` | 2500 | 0 | 0.00 | 0 | — | — | 2441 | 13.0 (12.1–13.9) | 1 / 1 / 1 | 3.19 / 4.03 / 4.35 |
| `jitter-5ms` | 2500 | 0 | 0.00 | 0 | — | — | 2441 | 13.0 (12.1–13.9) | 1 / 1 / 1 | 3.19 / 4.03 / 4.35 |
| `jitter-20ms` | 2500 | 0 | 0.00 | 0 | — | — | 2442 | 43.6 (42.2–46.0) | 2 / 4 / 6 | 13.51 / 17.10 / 18.34 |
| `jitter-40ms` | 2500 | 0 | 0.00 | 0 | — | — | 2450 | 60.7 (60.3–61.0) | 4 / 9 / 13 | 26.72 / 33.37 / 35.96 |
| `loss-0pct` | 2500 | 0 | 0.00 | 0 | — | — | 2393 | 18.8 (16.5–22.3) | 1 / 2 / 2 | 6.69 / 8.51 / 9.48 |
| `loss-1pct` | 2477 | 23 | 0.92 | 22 | 1.05 | 2 | 2354 | 19.4 (17.6–21.9) | 1 / 2 / 2 | 6.67 / 8.49 / 9.33 |
| `loss-5pct` | 2388 | 112 | 4.48 | 107 | 1.05 | 2 | 2179 | 18.0 (16.4–19.8) | 1 / 2 / 2 | 6.55 / 8.26 / 9.16 |
| `150ms-20j-0.5loss` | 2491 | 9 | 0.36 | 9 | 1.00 | 1 | 2328 | 43.9 (41.0–45.8) | 2 / 4 / 6 | 13.49 / 17.37 / 19.40 |
| `300ms-60j-2loss-bursty` | 2460 | 40 | 1.60 | 11 | 3.45 | 9 | 2133 | 70.1 (68.9–71.4) | 6 / 14 / 20 | 38.78 / 48.04 / 51.32 |
| `synthetic-burst` | 2500 | 0 | 0.00 | 0 | — | — | 2475 | 28.1 (28.1–28.1) | 1 / 30 / 38 | 1.44 / 75.63 / 98.77 |

Burst columns are uplink; downlink is statistically identical and is in the CSVs
(`net_downlink_loss_burst`). Per-seed ranges are given for the reordering rate because §8 rule 3
requires the observed seed spread; per-seed loss counts are in "Seed spread" below.

### 1. Reordering is pervasive, is a pure function of jitter, and is far worse than reported

`reorderProbability` is **0.0 on every profile in the suite**, and reordering is nevertheless the
dominant impairment on the hard profiles: **70.1% of downlink arrivals on
`300ms-60j-2loss-bursty` are out of sequence**, with a median displacement of 6 and a maximum of
20. The survey reported this qualitatively ("reordering is pervasive") on inference from
`EmulatedTransport` delivering by earliest synthetic arrival. **It is now measured, and it is
larger than anyone guessed.**

The mechanism is confirmed rather than assumed, by the ladder, which is why the ladder was in the
config:

| jitter half-width | reorder rate | max displacement |
|---|---|---|
| 0 ms (`jitter-0ms`) | **0.0%** | — |
| 1 ms (`lan`) | **0.0%** | — |
| 5 ms | 13.0% | 1 |
| 20 ms | 43.6% | 6 |
| 40 ms | 60.7% | 13 |
| 60 ms (`300ms-60j-2loss-bursty`) | 70.1% | 20 |

`jitter-0ms` and `lan` are the controls and both are **exactly zero** — reordering here is not an
artifact of the observer, it is jitter inverting adjacent arrivals. The threshold behaviour is
exactly right for a 10 ms send cadence: `lan`'s ±1 ms window cannot invert a 10 ms gap and produces
none; ±5 ms can only just do it and produces 13% at displacement 1; ±60 ms scatters a datagram over
twelve send slots and inverts almost everything.

**What this does to the `double-exp` lead.** `DoubleExponentialPredictor.Observe` opens with
`if (_hasState && obs.CaptureTicks <= _lastAcceptedTicks) return;` — it discards every sample not
newer than the last accepted, where `_lastAcceptedTicks` is a running maximum. That is the same
shape as the metric measured here, and on `300ms-60j-2loss-bursty` the measured out-of-order
fraction is **70%, not the "a third" the survey estimated**. I state this as strong support, not as
the measurement: my metric orders by *sequence* and the predictor orders by *capture tick*, and the
two coincide only where downlink send ticks are strictly monotone in sequence — which uplink
reordering and same-step reply batching can both break. The direct measurement is a `Prediction/`
axis experiment (count rejections inside `Observe`) and I did not make it. But the
`const-vel`-splices-them-in / `double-exp`-drops-them contrast is now known to operate on the
majority of the stream, not a minority of it.

### 2. `synthetic-burst`'s loss is exactly zero and its five seeds are byte-identical — both confirmed

Zero `net_uplink_dropped` and zero `net_downlink_dropped` across all five seeds. And splitting the
pooled CSV at trial boundaries, **all five seeds hash to one digest**: identical metric name, value
and tick, row for row. `jitter-0ms` is likewise identical across seeds, for the same reason (no
impairment with a random outcome). Every other profile differs across seeds, as it must.

So `docs/metrics.md` §8 rule 3 — "never declare a winner from a single seed, and always state the
observed seed spread" — is **unsatisfiable on `synthetic-burst`**: its effective n is 1 and its
seed spread is identically zero, which reads as "extremely stable" and means "the seed does
nothing". `synthetic-burst` appears in `exp-001`, `exp-003`, `exp-004` and `exp-005`. This was
reported by the previous survey and is now verified.

What `synthetic-burst` *does* have is the most extreme delay-variation profile in the suite:
reorder displacement p95 = 30, max 38, and a jitter distribution whose p50 is 1.44 ms and whose p99
is **98.77 ms** — a 69× spread, against `300ms-60j`'s 1.3×. It is the only profile in the suite
whose impairment is genuinely bursty in *time* rather than stationary, and the burst is entirely a
delay burst, exactly as the survey said. Its name misleads, and its ADR calls it "periodic", which
it is not.

### 3. Loss burst shape is measurable and separates the two loss families cleanly

`300ms-60j-2loss-bursty` declares p(lost|delivered) = 0.00612 and p(lost|lost) = 0.7, i.e. a 2.0%
steady state and an expected burst length of 1/(1−0.7) = **3.33**. Measured over the pooled run:
loss **1.60%**, mean burst **3.45**, max **9**, p95 **9**. The `loss-<N>pct` family declares equal
transition probabilities, degenerating to Bernoulli with expected burst length ~1: measured mean
**1.05** at both 1% and 5%, max **2**.

That is §3's own argument made concrete — `loss-5pct` (4.48%, mean burst 1.05) and
`300ms-60j-2loss-bursty` (1.60%, mean burst 3.45) have loss rates within 3× of each other and burst
distributions that are not comparable at all. The rate genuinely cannot distinguish them, and now
something can.

**The one honest wrinkle:** every measured loss rate sits below its declared value — 1.60 vs 2.0,
4.48 vs 5.0, 0.92 vs 1.0, 0.36 vs 0.5. All four are within ~1.5 standard errors given the burst
correlation, and 4/4 sharing a sign is 1-in-8 by chance, so this is *suggestive and not
significant*. The plausible mechanism is real but small: the Gilbert–Elliott chain is reset into
the good state at every trial boundary and each trial is only 500 datagrams, so short trials
undersample the steady state. Recorded rather than resolved; resolving it needs longer trials,
which is a config change, not a code change.

### 4. F4 fired — accepted sends exceed deliveries on every profile — and the cause is the harness, not the link

Predicted: `net_<dir>_received == net_<dir>_sent`, because no shipped impairment destroys a
datagram after acceptance. **Falsified on every profile**, by 5 to 155 datagrams. Before believing
a new loss mechanism I checked the shape, and it is entirely explained: the deficit per trial
equals the profile's base delay divided by the 10 ms step.

| profile | base delay | deficit/trial (uplink) | base delay ÷ step |
|---|---|---|---|
| `lan` | 2 ms | 1.0 | 1 |
| `jitter-0ms` | 50 ms | 5.0 | 5 |
| `loss-0pct` | 100 ms | 10.4 | 10 |
| `150ms-20j-0.5loss` | 150 ms | 15.0 | 15 |
| `300ms-60j-2loss-bursty` | 300 ms | 28.0 | 30 |

**These are datagrams still in flight when the trial ends**, not losses: a sweep stops stepping at
step 500 and never polls for what is still on the wire. So the finding is not "the emulator loses
datagrams" — the emulator is clean — it is that **every trial in this repo truncates its last
round trip**, and the amount truncated scales with delay: 0.2% of the stream on `lan`, and 5.6% of
the uplink and 6.8% of the downlink on `300ms-60j-2loss-bursty`. It is a shutdown transient of the
harness, it is delay-correlated, and it was invisible to every metric that existed before this
change. Whether it matters for any recorded comparison is a separate question I did not answer; it
is small, but it is not zero and it is not uniform across profiles.

The internal consistency of the chain is otherwise exact, which is what makes that diagnosis
trustworthy: on every profile, `net_uplink_received == net_downlink_sent + net_downlink_dropped` to
the datagram (e.g. `300ms-60j-2loss-bursty`: 2320 = 2288 + 32). The robot replies once per received
uplink datagram, so that identity has to hold — and it does, across three independently counted
metrics at three different points in the pipeline.

### 5. The jitter estimator agrees with the emulator's analytic jitter to ~3%

`UniformJitterImpairment` draws uniformly on `[−J, +J]`. Two independent draws differ by a
triangular variate on `[−2J, 2J]` with `E|D| = 2J/3`. Measured `net_downlink_jitter_ms` p50 against
`2J/3`:

| profile | J | 2J/3 | measured p50 | ratio |
|---|---|---|---|---|
| `lan` | 1 ms | 0.667 | 0.66 | 0.99 |
| `jitter-5ms` | 5 ms | 3.333 | 3.19 | 0.96 |
| `jitter-20ms` | 20 ms | 13.333 | 13.51 | 1.01 |
| `jitter-40ms` | 40 ms | 26.667 | 26.72 | 1.00 |
| `300ms-60j` | 60 ms | 40.000 | 38.78 | 0.97 |

This is the strongest available check that the estimator is right, because the target is analytic
rather than empirical: the model's parameter, not another measurement. The residual few percent is
expected — `J` is an EWMA whose median sits slightly below its mean.

Note also that `50ms-5j` and `jitter-5ms` produce **identical numbers in every column**. They are
the same profile (the isolated jitter family fixes base delay at 50 ms), so with the same seeds
they must, and they do — an unplanned end-to-end determinism check.

### Seed spread (§8 rule 3)

Per-seed `net_uplink_dropped`, seeds 1–5:

| profile | per seed | pooled |
|---|---|---|
| `300ms-60j-2loss-bursty` | 8, 14, 7, 9, 2 | 40 |
| `loss-5pct` | 20, 21, 29, 24, 18 | 112 |
| `loss-1pct` | 5, 6, 4, 5, 3 | 23 |
| `150ms-20j-0.5loss` | 2, 3, 2, 1, 1 | 9 |
| every zero-loss profile | 0, 0, 0, 0, 0 | 0 |

The spread is wide — `300ms-60j-2loss-bursty` ranges 2 to 14, a 7× spread across seeds at ~500
datagrams per trial. **Any single-seed loss figure on this profile is meaningless**, and the
burst-length distribution has only ~11 runs pooled across all five seeds. That is the quantitative
form of the survey's "the loss-burst evidence base is thin": thin by a factor that makes
single-trial burst statistics unusable, and no amount of instrumentation fixes it — only longer
trials or more seeds would.

### Falsifier scoreboard

| | outcome |
|---|---|
| **F1** — a §3 quantity computable only outside Core or needing a wire change | **FIRED, once, as predicted.** Uplink RFC 3550 jitter needs an operator `TicksPerSecond` on `CommandFrame`. Nothing emitted for it; recorded in §3 as a wire-format change. |
| **F2** — instrumenting requires changing control flow / RNG order / an existing metric | Did not fire. The `OperatorEndpoint` edit is one optional parameter and one guarded call; `verify` replays byte-identical. |
| **F3** — cannot be made allocation-free or breaks determinism | Did not fire. Allocation tests pass for all four hot-path methods; `verify` PASS. |
| **F4** — deliveries fewer than accepted sends | **FIRED on every profile.** Cause identified as end-of-trial truncation, not link loss, and quantified. |
| **F5** — reordering turns out to be zero | Did not fire; the opposite. 70% on the hardest profile, 0% only where jitter is 0. |
| Prediction 1 (2% / mean 3.33 burst) | Held: 1.60% / 3.45. |
| Prediction 2 (`synthetic-burst` zero loss, inert seeds) | Held exactly. |
| Prediction 3 (reordering non-zero when jittered, zero when not) | Held exactly. |
| Prediction 4 (received == sent) | Falsified — see F4. |

### Exposure to the in-flight ring defect

The survey's most consequential finding is that `OperatorEndpoint.InsertInFlight` overwrites an
occupied ring slot without checking, delay-correlated. **All six new metric families (ten names) have zero
exposure to it**: `net_*_sent`, `net_*_dropped`, `net_*_received` and `net_*_loss_burst` are
counted at the transport boundary, which the ring never touches, and
`net_downlink_reorder_displacement` / `net_downlink_jitter_ms` are emitted immediately after decode
and before the ring is consulted at all. That was a design requirement, not luck — see "Design
decisions".

The one §3 quantity that *is* exposed is the OWD IQR, because it reduces `owd_*_ms`, which is
emitted only on the trace-completion path. That is a reason to prefer `net_downlink_jitter_ms`
over the OWD IQR on the hard profiles, and it is emphatically **not** a reason to change
`owd_*_ms`, which is a baseline every recorded result depends on. I did not touch it.
## Verification

All gates run from the worktree root after the final edit.

```
$ cd core && dotnet test --nologo -v q
Passed!  - Failed:     0, Passed:    28, Skipped:     0, Total:    28 - Teleop.RobotArm.Tests.dll (net8.0)
Passed!  - Failed:     0, Passed:   683, Skipped:     0, Total:   683 - Teleop.Core.Tests.dll (net8.0)
Passed!  - Failed:     0, Passed:     3, Skipped:     0, Total:     3 - Teleop.Eval.Tests.dll (net8.0)
Passed!  - Failed:     0, Passed:    36, Skipped:     0, Total:    36 - Teleop.RobotHost.Tests.dll (net8.0)

$ dotnet run --project Teleop.Eval -- verify
verify: PASS -- .../testdata/golden/basic-session.tlog replays byte-identical across two
independent passes, and matches the original file exactly.

$ dotnet run --project Teleop.Eval -- audit
audit: PASS -- no invariant violations found in .../core/Teleop.Core or
.../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.

$ just bridge-check
Build succeeded.  0 Errors.  (1 pre-existing CS8632 warning in
unity/.../JetRoverOperatorBridge.cs, a file I did not touch)
```

`bridge-check` was run because the `OperatorEndpoint` constructor signature changed. The new
parameter is optional, so both Unity bridges compile unchanged — but that is exactly the class of
break `bridge-check` exists to catch, and "it's optional so it must be fine" is not a gate.

Test counts: 683 Core tests, of which **32 are new** (21 in `NetworkObserverTests`, 11 in
`MeasuredTransportTests`), and 651 pre-existing, all still green.

### `dotnet test` is currently flaky on this machine, and it is not my change

The clean run above is real, but it is not the only outcome. Across seven full-suite runs I saw
0, 1, 1, 2, 2 failures, never the same test twice, always an `AllocationAssert.Zero` test, always
with a **fractional** bytes-per-call figure (0.733, 0.165-scale) — which no real allocation can
produce, since the smallest managed object is 24 bytes. `TestSupport/AllocationAssert.cs`'s own doc
describes this exact signature as the JIT-tiering artifact it was hardened against on loaded CI
runners. Three researcher worktrees are building and running on this box concurrently, so the box
is loaded in the same way.

I did not accept that as an explanation on the strength of the doc comment. **Controlled check:** I
moved my two new test files out of the tree and ran the full suite six times at HEAD's test set —
1, 0, 3, 1, 1, 0 failures, same tests, same fractional signature. Then restored them. The flake
rate is indistinguishable with and without my tests, and every failing test
(`MotionMathTests.IntegrateWorld`, `MotionMathTests.ToRotationVector`,
`RobotStateFrameCodecTests.TryEncode`) is in a file I did not touch and exercises no code path my
change is on. Run in isolation those same tests pass 3/3.

**I changed nothing to make this pass** — no iteration count, no tolerance, no `[Trait]`, no skip.
Weakening an allocation gate is the specific thing invariant 10 forbids. It is a pre-existing
robustness problem in the harness under load, it belongs to nobody's axis, and it is written up
under "Left undone" rather than quietly absorbed.

### `just test` (analysis/) — BLOCKED, environment, not code

```
$ just test
python3 -m venv failed. On Debian/Ubuntu the stdlib venv/ensurepip module is
packaged separately: sudo apt install python3-venv
error: recipe `analysis-setup` failed with exit code 1
```

`analysis/.venv` exists in this worktree but is incomplete (`No module named pytest`) and
`analysis-setup` cannot repair it: `python3-venv` is not installed and installing it needs root.
**This gate did not run and I am not claiming it passed.**

Why I judge the risk low rather than unknown: I touched no file under `analysis/`. The only way my
change reaches Python is that `metrics.csv` now contains rows with seven new names. Every consumer
selects by name (`df[df["name"] == METRIC]` in `figures/*.py`) and `io_utils` reads `name` as a
pandas `category` dtype, which accepts arbitrary values — there is no allow-list, no schema check
and no enumeration of metric names anywhere in the package. Extra rows are inert to it. A human
with `python3-venv` should still run it.

## How this result could be flattering itself

Written deliberately, because the failure modes here are subtle and mostly favour me.

1. **The instrumentation and its ground truth share a code path.** `net_uplink_dropped` counts
   `Send` returning false, and `Send` returns false because `GilbertElliottLossImpairment` said so.
   If the impairment's *model* were wrong — if it drew from the wrong distribution — my measurement
   would agree with it perfectly and report a wrong link as correct. What partially rescues this is
   that two checks compare against something *analytic* rather than against the code: the burst
   mean against 1/(1−0.7) = 3.33, and the jitter p50 against `E|D| = 2J/3` for a uniform draw. Both
   are derived from the profile's declared parameters, not from the emulator's behaviour. The loss
   *rate* check is weaker in exactly this way, and its consistent 20% shortfall may be the first
   sign of that rather than the chain-warm-up story I offered.
2. **"Reordering is 70%" flatters the metric.** A big, surprising number is the easiest kind to be
   wrong about. Sanity checks I actually ran: the two zero-jitter controls report exactly 0.0%, so
   the metric is not counting arrivals in general; the rate rises monotonically with jitter width
   across four points; the maximum displacement tracks the jitter window divided by the send
   interval; and `50ms-5j` and `jitter-5ms`, which are the same profile under different names,
   agree to the last digit. What I did *not* do is verify against an independent reordering
   measurement, because none exists — this is the first one.
3. **The tolerance I widened.** `MeasuredTransportTests`' loss-rate bound started at
   `[0.015, 0.026]` and the observed 0.0156 landed just inside it. I widened it to
   `[0.013, 0.027]`. That is exactly the move that must be viewed with suspicion, so: the first
   bound came from the *Bernoulli* standard error, which is the wrong variance for a
   burst-correlated loss process; the corrected variance
   (`B·var(L) + E[L]²·var(B)`) gives ±3 se = ±0.007. The bound was recomputed from the corrected
   derivation, both derivations are in the test's comment, and the test **would have passed
   either way**. If a reviewer disagrees, reverting to the narrower bound costs nothing.
4. **Zero exposure to the in-flight ring is a design claim, not a measurement.** I argued it from
   where the calls sit in the source. It is checkable — `net_downlink_received` (transport vantage)
   should exceed `count(owd_downlink_ms)` (trace vantage) by exactly the ring's evictions — and on
   `300ms-60j-2loss-bursty` it does, by a lot: 2133 versus 1463, i.e. **31% of arriving replies
   never complete a trace**. That is consistent with my claim and is also an independent
   measurement of the ring defect's severity, which nobody had. I did not pursue it further because
   it is the survey's finding, not mine, and fixing the ring is a human's call about comparability.
5. **The characterization sweep cannot fail.** It has one stack and no competitor, so there is no
   comparison it could lose. That is appropriate for a characterization, and it also means nothing
   here is evidence that the instrumentation is *useful* — only that it is correct and that the
   profiles are what it says they are. Whether §3 unblocks the Transport axis is a claim only a
   later run that actually uses these metrics can support.
6. **One direction, one topology.** Every reordering and jitter figure is downlink-only, from one
   pipeline in which both directions carry the same profile. I assert uplink is statistically
   identical *by construction*; I did not measure it.

## Left undone / for a human

Ordered by what I would do next.

1. **Nothing is committed.** Branch `worktree-agent-a62cf35ca82b48faf`, dirty by instruction. The
   sweep's `manifest.json` records `gitSha` `76b2a14`, which does not contain the code that
   produced it, so **no number in this log is citable until the diff lands and the sweep is
   re-run**. That re-run costs about a minute.
2. **The end-of-trial truncation (finding 4) is unowned and affects every recorded result.** Each
   trial discards the datagrams still in flight when stepping stops — 1 per trial on `lan`, 28 on
   `300ms-60j-2loss-bursty`. It is delay-correlated, so it is not uniform across the profiles a
   figure compares. The fix is cheap (drain for one extra round trip after the last send, or drop
   the last N steps from the sample set); deciding what it does to comparability with existing
   `results/` directories is a human's call, exactly like the in-flight ring. I did not touch it.
3. **`net_downlink_received` = 2133 versus `count(owd_downlink_ms)` = 1463 on
   `300ms-60j-2loss-bursty` puts a number on the in-flight ring defect for the first time: 31% of
   arriving replies never complete a latency trace.** Every `owd_*_ms` percentile on that profile
   is computed over the 69% of round trips that were fast enough to survive eviction. Still a
   human's call, but it is no longer an unquantified one.
4. **Uplink reordering is one small edit away.** `RobotEndpoint` would take an optional
   `NetworkObserver` and call `OnSequencedArrival(frame.Sequence, 0, 0, arrivalTicks)` — sender
   rate 0, so reordering only, no jitter, no clock conversion, no ADR 0002 conflict. I chose not
   to, because it buys a statistical replicate rather than a new question (both directions carry
   the same profile). If the two directions are ever configured differently, do it.
5. **Uplink jitter needs an ADR, not an implementation.** Adding an operator `TicksPerSecond` to
   `CommandFrame` is a wire-format change and would reverse an explicit decision recorded in
   ADR 0008. It buys uplink jitter and nothing else. My recommendation is *don't*: the downlink
   figure already characterizes the link, and the uplink direction is drawn from the same profile.
6. **`AllocationAssert` is not robust on a loaded machine** and the failure is silent-ish — a
   different test each run, so it reads as a real regression. Evidence and the controlled check
   are above. Someone should decide whether to make the harness quiesce (force a GC and a settle
   before measuring) or serialize the allocation tests into their own non-parallel collection.
   **Not a tolerance change** — the assertion should stay "exactly zero".
7. **`Transport/CLAUDE.md` needs an "Implemented" row for `measured` and `NetworkObserver`, and a
   "Metric names" section.** Axis `CLAUDE.md` files are outside my scope by instruction, so here is
   the proposed row verbatim:
   > \| measured \| `MeasuredTransport.cs` \| transparent decorator; counts sends, refusals, loss
   > runs and deliveries into a `NetworkObserver` (docs/metrics.md §3). Wraps **outside**
   > `EmulatedTransport` — inside, it never sees a lost datagram. No `Registries.cs` entry, for the
   > same constructor-shape reason as `EmulatedTransport`. \|
8. **`docs/adr/0004` calls `synthetic-burst` "periodic".** Now verified as neither periodic nor
   seed-sensitive: five byte-identical seeds. The wording is misleading in the direction that makes
   forecasting pitches sound plausible, and correcting an ADR is a human's job.
9. **The frozen suite's loss evidence remains one profile deep**, and this run quantifies how thin:
   ~11 loss runs pooled across five seeds on the only bursty profile, with a 7× per-seed spread in
   drop count. Any future loss-burst work needs longer trials or more seeds before it needs a new
   profile family (which would need an ADR).
