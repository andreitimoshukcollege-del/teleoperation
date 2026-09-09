# Path diversity and send-time scheduling — feasibility (survey only, nothing built)

**Agent:** researcher 4 of 4, Transport-axis survey/feasibility run
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a6f62b0acaa9f962b`
**Branch:** `worktree-agent-a6f62b0acaa9f962b`
**HEAD at start:** `06c0ab5b6f570efa028002897c856c566041663b`
**Mandate:** survey and price. **No `.cs`, no test, no registry entry, no YAML, no sweep, no
`results/` output.** This file is the entire deliverable.

## Seed self-check

| Check | Expected | Observed | Verdict |
|---|---|---|---|
| `git rev-parse HEAD` | `06c0ab5b6f570efa028002897c856c566041663b` (authoritative value per the brief's correction) | `06c0ab5b6f570efa028002897c856c566041663b` | PASS |
| Worktree path | own worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-a6f62b0acaa9f962b` | PASS |
| Branch | own branch | `worktree-agent-a6f62b0acaa9f962b` | PASS |
| Gates (`just core-check`) | green, unchanged by me | 625 tests pass (558 Core + 36 RobotHost + 28 RobotArm + 3 Eval), `verify: PASS`, `audit: PASS` | PASS |

Note on the seed value: the brief first gave `...0286 7c...` and then corrected the authoritative
value to `...0289 7c...`. HEAD matches the corrected (authoritative) value exactly. Proceeding.

## Scope and handoffs

I own **where and when datagrams are put on the wire**: (a) path diversity — multipath,
duplication across paths, racing, path switching; (b) send-time scheduling on one path — pacing,
cadence, temporal spreading of related datagrams.

I do **not** own, and hand off explicitly:

- **What is inside a datagram** — N-frame redundancy, parity/FEC payloads. Candidate 1. I own the
  *placement in time and across paths* of a redundant copy; whether a redundant copy exists at all,
  and what it contains, is theirs. Concretely: "send a copy" is theirs, "send that copy k ticks
  later / on the other link" is mine, and the two are useless apart.
- **Trajectory/intent codecs** — candidate 2.
- **The path-quality estimator** — candidate 3. I own the *mechanism that acts on* a path estimate
  (the switch, the racer); they own the estimate. The handoff is one interface-shaped question:
  "given a path id, what is its current quality, now?" Neither side exists today.
- **Adaptive playout** — candidate 3.

## Framing I am required to be precise about

For a **single** path, nothing in Core reduces one-way delay. OWD belongs to the network.

My candidate is the one genuine partial exception in this run, and the precise statement is:

> With two paths whose delay distributions are `D1` and `D2`, and a datagram duplicated on both,
> the **delivered** delay is `min(D1, D2)`. That changes the delivered delay *distribution*. It
> does not reduce the delay of any path, and it cannot go below `min(support(D1) ∪ support(D2))` —
> the faster path's floor. It is a selection effect, not a speed-up.

Every other thing in my scope (pacing, spreading, switching without duplication) changes only the
*impact* of loss and jitter, never delivered delay — with the single exception that a path
*switch* to a genuinely faster path moves the whole distribution, which is again selection, not
reduction.

---

## Q1. What would multipath actually require here? (the crux)

**Determination: a duplicating / racing multipath transport is a pure decorator over two
`ITransport`s and needs no change to the `ITransport` contract. The contract as written already
permits it, and permits it *exactly*.** The new contract is only needed if you want deduplication
inside the transport layer (see Q2), and there is a way to avoid needing that too.

Evidence, clause by clause from `core/Teleop.Core/Contracts/ITransport.cs`:

| `ITransport` clause | Does a two-path decorator satisfy it? |
|---|---|
| "what is sent may be delayed, dropped, **duplicated**, or reordered" | Yes — duplication is named in the contract as an allowed channel behaviour. This is the single most important sentence for my candidate: a duplicating transport is not an abuse of the interface, it is a case the interface was written for. |
| `MaxPayloadBytes` | Take `min(a.MaxPayloadBytes, b.MaxPayloadBytes)`. Safe and honest — every datagram must fit on both paths if it is to be duplicated on both. No contract issue. |
| `Send(payload, nowTicks) -> bool` | Send on both, return `true` if **either** accepted. The doc says `false` means "will not be delivered", so returning true when one path took it is correct. Also: "callers must not retry on false" is preserved. |
| `TryReceive(nowTicks, dest, out byteCount, out arrivalTicks)` | Drain both sub-transports, buffer into one min-heap keyed by reported `arrivalTicks`, pop earliest due. This is structurally *identical* to what `EmulatedTransport` already does with its single inner transport — the only change is that `DrainInner` drains two instead of one. |
| "Datagrams are returned in arrival order, which is not send order — reordering is part of what is being studied" | Yes. Two paths guarantee reordering, and the contract explicitly tolerates it. |
| "an implementation must never return the same datagram twice" | **This is the one subtle clause.** Two copies of the same *application* frame are two distinct *datagrams* as far as the transport can tell — the transport sees opaque bytes and has no sequence field. Returning both is not "the same datagram twice" in the transport's own terms; it is two datagrams the channel duplicated, which the type doc's first paragraph explicitly allows. I read this clause as forbidding an implementation from re-delivering one datagram it already handed out, which a heap-pop implementation cannot do. **Assumption recorded:** that reading. The alternative reading (the transport must dedup) would force a contract change, and I flag it as the one place a human might disagree with me. |
| `Reset()` — "A decorator resets the transport it wraps" | Reset both. Exactly `EmulatedTransport.Reset()`'s pattern extended to two. |
| No wall clock, no I/O, no threads, no allocation after construction | All satisfiable — `EmulatedTransport` already does exactly this with a preallocated slot array + free-slot stack + min-heap, and the two-path version is the same data structure with one more drain source. |

So the shape is: `sealed class DuplicatingTransport : ITransport` taking `(ITransport pathA,
ITransport pathB, int maxInFlight)`, or a racing/selecting variant taking a path index it is told
to use. Roughly 250–350 lines, most of it lifted structurally from `EmulatedTransport`, plus the
usual doc comment density this repo maintains.

**Where the *composition* is already right.** `Teleop.Eval/Sweep/SweepCommand.cs` constructs one
`uplink` and one `downlink` `ITransport` and passes the *same instance* to both `OperatorEndpoint`
and `RobotEndpoint`. So a transport object is a directed pipe shared by both endpoints, and
substituting a two-path decorator for it is a one-expression change at the composition site.
`OperatorEndpoint`/`RobotEndpoint` need no modification at all for the *transport* part.

**What does NOT fit the decorator story:**

1. **A per-path header.** If the decorator wanted to stamp a path id or its own sequence into the
   bytes, `MaxPayloadBytes` would have to shrink by the header size, and `OperatorEndpoint`'s
   constructor throws when `commandCodec.MaxEncodedBytes > uplinkTransport.MaxPayloadBytes`. In the
   sweep the loopback is sized to *exactly* `RawPoseCodec.EncodedSize` (73), so a header would make
   the wiring throw until the sizing constant in `SweepCommand` changed. That is an
   `core/Teleop.Eval/` edit, not a Core one — cheap, but it is a real coupling and I could not make
   it in this run even if I were building.
2. **Anything with per-path *state* the pipeline must see** — a path-quality readout, a switch
   event. `ITransport` has no `Diagnostics` property (unlike `IPredictor`/`IReconciler`, which do).
   A path-switching transport that wanted to report *why* it switched would need one. That is a
   contract change, and it is the boundary at which this stops being free.

**Verdict on Q1:** **decorator, no contract change**, for duplication and for
told-which-path-to-use selection. **New contract** only for (i) transport-level diagnostics, or
(ii) a transport that must parse payloads. Neither is required by the minimal experiment.

## Q2. Where does the deduplication live?

**Answer: nowhere, and — surprisingly — the pipeline already tolerates duplicates end to end
today, but with one bandwidth artifact that would confound the experiment.** I traced both
endpoints.

**Uplink duplicate (two copies of one `CommandFrame` reach `RobotEndpoint`):**

- `RobotEndpoint.Step` decodes each datagram and calls `_plant.Command(frame)` for each.
- `IRobotPlant.Command`'s own contract: "Commands may arrive out of order or duplicated after
  transport, so an implementation compares `CommandFrame.CaptureTicks` against the setpoint it
  holds and **ignores a stale or repeated frame**".
- `Plant/RigidBodyPlant.cs:156` implements exactly that: `if (command.CaptureTicks <=
  _lastAcceptedCaptureTicks) return;`. **The plant is the dedup point, by contract and in fact.**
- **But** `RobotEndpoint` replies "once per **received** datagram, not once per **accepted**
  command" — this is `Pipeline/CLAUDE.md` requirement 3, deliberate, and documented at length in
  the method comment. So **a duplicated uplink datagram produces a duplicated downlink reply**,
  with the same `Sequence` and a later `RobotRecvTicks`.

**This is the finding that matters most for experiment design: duplicating the uplink silently
duplicates the downlink too.** A "uplink-only two-path" configuration does not exist without
changing `RobotEndpoint` — you get 2× uplink bytes *and* 2× downlink bytes. Any bandwidth
accounting that assumed 2× must actually say 2× on both directions, and any comparison against a
single-path baseline is comparing 2 datagrams/tick against 4 datagrams/tick, not 2 against 3.

**Downlink duplicate (two `RobotStateFrame`s with the same `Sequence` reach `OperatorEndpoint`):**

- `TryReceiveState` calls `TryTakeInFlight(stateFrame.Sequence, ...)`, which **clears the occupied
  flag** on match. The second copy finds no in-flight entry and hits `continue` — "A reply for an
  unknown or already-evicted sequence is an ordinary, silently skipped outcome."
- Consequence: `ClockSync.AddRoundTrip`, the OWD metric records, `_lastAckSequence`, and
  `ObserveRobotState` (which feeds the predictor and reconciler) **all fire exactly once per
  sequence, on the first copy to arrive**. No double-counting of anything.
- **And that "first copy to arrive" is precisely the min-over-paths semantics my candidate wants,
  for free, with no dedup code at all.** The take-once in-flight ring is an accidental but exact
  implementation of "take whichever arrives first."

**So: dedup lives in the plant (uplink, by contract) and in the take-once in-flight ring
(downlink, incidentally).** No new dedup component is needed in Core for a duplicating transport.
Three caveats:

1. The duplicate still costs work: an extra decode, an extra `_plant.Command` call, an extra 64-slot
   linear scan in `TryTakeInFlight`, and an extra reply encode+send. Allocation-free, so invariant 8
   holds, but not free.
2. `RobotEndpoint`'s `maxDatagramsPerStep` (64 in the sweep) and `LoopbackTransport`'s capacity (64)
   both halve in effective headroom. At 100 Hz with one frame per tick this is nowhere near
   binding, but it is a thing to check, not assume.
3. If a duplicating transport *did* want to suppress duplicates itself without a header, it could
   do byte-equality against a small preallocated ring of recently delivered payloads. That is exact
   for this workload because every `RawPoseCodec` frame carries a distinct `CaptureTicks`, so two
   byte-identical frames can only be duplicates. It is fragile in general and I would not build it
   first — the pipeline already tolerates duplicates, so the minimal experiment should not.

## Q3. Is this evaluable headlessly?

**Yes, today, with zero new profiles and zero new infrastructure beyond the decorator itself — and
the model is a *best case*, which is the honest caveat.**

`EmulatedTransport` is a decorator over any `ITransport`, carrying its own `NetworkProfile` and its
own owned `SeededRng`. So:

```
DuplicatingTransport(
    EmulatedTransport(LoopbackTransport(...), profileA, SeededRng(seed),   cap),
    EmulatedTransport(LoopbackTransport(...), profileB, SeededRng(seed+7), cap),
    cap)
```

is a two-path model available today. Two profiles, two independent RNG streams, one min-over-paths
delivery. Nothing in Core objects.

**What the model captures faithfully:** two different base delays; two different jitter widths; two
independent Gilbert-Elliott loss chains with different rates and burst lengths; independent
reordering; and, via `EmulatedTransport`'s trace-driven overload, two different *recorded* delay
traces. That is a real and non-trivial two-path model.

**What it cannot capture, and this list is the reason I would not trust a win from it:**

1. **Shared-bottleneck correlation.** Real "two paths" from a headset very often share the last
   mile, the home router, or the same cell tower's backhaul. When they share a bottleneck, their
   delays and their losses are *positively correlated*, and every benefit of a min-over-paths
   collapses toward the benefit of one path. **Independent paths are the best case, and an
   uncorrelated two-profile model overstates the benefit — possibly by a lot.** This is the single
   biggest fidelity gap and I want it stated flatly: a positive result from independent emulated
   profiles is an *upper bound*, not an estimate.
2. **Correlated loss across paths.** Same argument, sharper: the loss benefit (below) is entirely
   `p_A · p_B` under independence and degenerates to `min(p_A, p_B)` under perfect correlation.
   Two orders of magnitude of difference between those, on the same nominal profiles.
3. **Handover.** Wi-Fi→cellular switching costs a real, bursty, hundreds-of-milliseconds
   interruption. `EmulatedTransport` has no notion of a path going away and coming back; the GE
   chain's absorbing case (`LossProbabilityAfterLost == 1.0`) is a total-outage model but it never
   recovers before `Reset()`, so it cannot model an outage of finite duration.
4. **Path-switch cost.** In the emulator switching paths is free. In reality QUIC connection
   migration needs a path-validation round trip and resets the congestion controller.
5. **Bandwidth/congestion coupling.** Duplication doubles offered load. The emulator's loss and
   delay do not respond to load at all, so the model gives you the benefit of duplication and none
   of its cost. For a 58 kbit/s stream (below) that is probably fine; it is still a modelling
   assumption, not a fact.

**A correlated two-path profile does not exist and I am not permitted to add one (frozen suite,
and correctly so).** I record it as a **requirement**: any serious multipath result needs a
correlation knob — at minimum a shared-loss-event component and a shared-delay component between
two paths — and that is a `NetworkProfile`-shaped change, i.e. an ADR, because `NetworkProfile`'s
doc explicitly scopes it to "a **parametric** link", singular. Note the frozen combined family from
`docs/adr/0006` already lets a caller name any `(delay, jitter, loss)` triple, so **an
uncorrelated pair of arbitrary paths needs no new profile at all** — only the correlated case does.

## Q4. Does pacing mean anything for this workload?

**No. Family (b) is a non-candidate on this workload, and I want that said early and plainly.**

The arithmetic:

- `experiments/exp-001-predictor-baseline.yaml`: `stepIntervalTicks: 100000` at `TicksPerSecond =
  10_000_000` → **10 ms per step, 100 Hz**.
- Uplink: exactly one `RawPoseCodec` frame per step, fixed `EncodedSize = 73` bytes →
  7.3 kB/s ≈ **58 kbit/s**.
- Downlink: one `RobotStateFrameCodec` frame (57 bytes) per received uplink datagram →
  5.7 kB/s ≈ **46 kbit/s**.

Pacing exists to spread a **burst** so it does not overrun a bottleneck queue. There is no burst
here: the source is inelastic, one small fixed-size datagram per fixed tick, already perfectly
paced by construction. Anything a pacer could do to this stream is either a no-op or a delay
added on purpose. **Pacing is measurably meaningless for this workload and I would reject it
without an experiment.**

Send **cadence relative to render cadence** is a different and non-empty question — but it is a
question about *how often you sample the operator*, which changes the source, not its placement,
and it is entangled with the predictor (a higher command rate is a shorter extrapolation horizon).
It is also not what the brief scoped to me. I note it as a real, separate, unclaimed idea and
leave it.

**Temporal spreading** survives as the one part of (b) with content, and only as candidate 1's
partner. The analytic result, which needs no code:

> The Gilbert-Elliott chain's expected burst length is `L = 1/(1 − LossProbabilityAfterLost)`
> datagrams. For `300ms-60j-2loss-bursty` (`p_lost_after_lost = 0.70`), `L = 3.33` datagrams
> = **33 ms** at 100 Hz. A redundant copy offset by `k` ticks is only decorrelated from the
> original once `k > L`, i.e. **k ≥ 4 ticks = 40 ms**. Offsets of 1–3 ticks buy essentially
> nothing on this profile: the burst that took the original takes the copy too. And the copy
> arriving `k` ticks later means recovery is `k` ticks later, so the whole mechanism is a direct
> trade of `≥40 ms` of recovery latency for burst decorrelation.
> For `150ms-20j-0.5loss` (`p = p = 0.005`, degenerate Bernoulli), `L ≈ 1.005`, so **any** offset
> ≥1 tick already decorrelates and the trade is nearly free — but there is also almost nothing to
> gain, because isolated single drops are what an N-of-1 redundant copy on the *same* tick already
> covers.

That is the whole interleaving story on the frozen profile set, derived in a paragraph. It says the
interleaving parameter is only interesting on exactly one of the frozen profiles, and on that one
it costs 40 ms. **Handing this to candidate 1 is worth more than building it.**

---

## Prior art

Searched and read 2026-09-09. Rules I am holding to: a source generates candidates and settles
nothing here; for each I record what it claims *and the conditions it was measured under*; nothing
below is evidence about this system, and none of it may enter `results/`.

### The single most relevant source: multipath is *conditionally* good for latency

"Is multi-path transport suitable for latency sensitive traffic?" (Computer Networks, 2016;
<https://www.sciencedirect.com/science/article/abs/pii/S1389128616301396>, open version
<https://oatao.univ-toulouse.fr/28582/>). Measured with MPTCP over emulated and real paths.

- **Claims:** when paths are *symmetric* in capacity, delay and loss, latency is significantly
  reduced versus a single path. When paths are *asymmetric*, no improvement is observed. And —
  the line that matters most here — MPTCP "performs worse when there is not enough data to send."
- **Conditions:** MPTCP, i.e. a *reliable, in-order, elastic* transport with a congestion
  controller. Our workload is unreliable datagrams with no retransmission and no congestion
  control, so the "performs worse" mechanism (head-of-line blocking from a segment stranded on a
  slow path, plus a scheduler that cannot fill both paths) does **not** transfer directly.
- **Why I still weight it heavily:** its two conditions are exactly the two regimes my own
  arithmetic below identifies independently. Symmetric paths → the min actually does something.
  Asymmetric paths → the min degenerates to the faster path. Two different methods reaching the
  same partition is worth noting; it is still not evidence about this repo.

### Redundant multipath scheduling for a latency target

ReMDeLa, "Redundant Multipath-TCP Scheduling with Desired Packet Latency" (CHANTS'19,
<https://dl.acm.org/doi/abs/10.1145/3349625.3355440>). Claims to maximize the fraction of one-way
segment latencies meeting a target, beating existing redundant schedulers. Conditions: MPTCP,
asymmetric heterogeneous paths, segment-level latency. Objective is a **latency quantile**, not
throughput — which is the right objective for us and rare in this literature. The technique
(decide *per segment* whether to duplicate, based on a latency target) is the shape our
"predict which path will be good" question would take, and it is the clearest statement of the
handoff to candidate 3: the scheduler is worthless without the per-path latency prediction.

### Multipath QUIC in real-time applications, and where it loses

- "On the Latency of Multipath-QUIC in Real-time Applications" (IEEE, 2020,
  <https://ieeexplore.ieee.org/document/9253402/>).
- LoLa, "Low-Latency Realtime Video Conferencing over Multiple Cellular Carriers"
  (<https://arxiv.org/abs/2312.12642>). Technique: packet striping across carriers with dynamic
  per-link quality estimation plus codec adaptation. Conditions: **4 different cellular
  operators** in one metropolitan area, real-time video conferencing traffic, traces collected
  in the field. Claims throughput and delay gains over a state-of-the-art RTC solution. Note the
  path model: four *different carriers*, i.e. about as close to genuinely independent paths as
  the real world gets — the best case, and deliberately chosen as such. The abstract does not
  price redundancy bandwidth, and the mechanism is selection/striping rather than duplication.
- Search results consistently report that **vanilla MP-QUIC has a *worse* 99th-percentile
  completion time and higher rebuffering than single-path QUIC** — the gains all come from a
  scheduler tuned for the objective, never from having two paths per se. Directly applicable
  warning: "add a second path" is not the intervention; "add a scheduler that knows which path is
  good" is, and that is candidate 3's half.

### The min-of-two argument in its cleanest form, and why it does not transfer to our profiles

Dean & Barroso, "The Tail at Scale" (CACM 2013, <https://cacm.acm.org/research/the-tail-at-scale/>)
— hedged requests. Claim: on a 1000-key BigTable read across 100 servers, issuing a hedge after a
10 ms delay cut p99.9 from 1800 ms to 74 ms for ~2% extra requests. Conditions: datacenter RPCs,
**extremely heavy-tailed** service-time distribution (p99.9 is 24× the hedge threshold), and a
*deferred* hedge rather than unconditional duplication.

This is the canonical "duplication crushes the tail" result and it is the one most likely to be
invoked in favour of my candidate. **It does not transfer, and the reason is precise:** the entire
effect comes from the tail being long. This repo's parametric delay model is `Base + Uniform(−J,
+J)` — *bounded*, and uniform by explicit design (`Types/NetworkProfile.cs`: "a uniform integer
draw is the one distribution whose every outcome can be hand-computed and asserted exactly in a
deterministic test"). A bounded uniform has no tail to crush. The arithmetic in the next section
makes this quantitative and it inverts the expected result.

### Happy Eyeballs / connection racing — a scope correction

RFC 8305 (<https://www.rfc-editor.org/rfc/rfc8305.html>). Races connection *establishment* across
address families with a 50–250 ms bias delay, to avoid a broken path at setup. **It is a one-time
selection, not a per-datagram mechanism.** Against a stationary emulated `NetworkProfile` a
one-time race is worthless by construction: the profile does not change during a trial, so racing
would recover exactly the information the profile name already states. Happy-eyeballs-style
selection is only meaningful once a path can *change* mid-trial, which no frozen profile does.
**I am scoping connection racing out on that argument** — it is not falsifiable against the
current profile suite, and the profile suite is frozen.

### Search that found nothing useful

I looked for prior work modelling *correlated* multipath delay/loss for emulation purposes (the
thing I actually need to make the headless model faithful) and did not find a usable, concrete
parameterization in this window. "A relay-node-selection algorithm based on path correlation"
surfaced repeatedly as a topic but the search results gave no model I could hold up against
`NetworkProfile`'s six parameters. Recording that so the next run does not repeat it: **the gap
is not "does correlation matter" (everyone says yes), it is "what is a defensible two-path
correlated impairment model with few enough knobs to sweep."** That is unresolved and is the real
research obstacle to doing multipath honestly here.

---

## The arithmetic: min-of-two-draws against the frozen profiles

This section is the cheapest possible test of the candidate, and it did not require a line of
code. **It is a negative result.**

### Setup

Every parametric profile delivers `delay = BaseDelayTicks + U`, where `U` is a uniform integer
draw on `[−J, +J]` (`EmulatedTransport.DrawDelayTicks`). At 10 MHz ticks and `J` in the tens of
milliseconds the discrete draw is indistinguishable from continuous uniform, so I use the
continuous form.

For one path, the `q`-quantile is `Base + J(2q − 1)`.
For the min of two **independent, identically distributed** paths, `P(min ≤ x) = 1 − (1 − F(x))²`,
so the `q`-quantile of the min is the `(1 − √(1−q))`-quantile of one path:
**`Base + J(1 − 2√(1−q))`**.

### The improvement coefficient, in units of J

| Percentile | single path | min of two | **improvement** |
|---|---|---|---|
| p50 | `+0.000 J` | `−0.414 J` | **0.414 J** |
| p95 | `+0.900 J` | `+0.553 J` | **0.347 J** |
| p99 | `+0.980 J` | `+0.800 J` | **0.180 J** |
| p99.9 | `+0.998 J` | `+0.937 J` | **0.061 J** |

**The improvement shrinks monotonically as you go further into the tail.** Duplication over two
paths under this repo's delay model helps the **median more than the tail** — the exact opposite
of the multipath and hedged-request literature's headline claim, and entirely an artifact of the
bounded uniform jitter model. There is no tail here to win, because `NetworkProfile` deliberately
does not model one.

This is a genuinely useful finding and it cuts both ways: it kills the standard argument for the
candidate *on the current profile suite*, and it simultaneously says the current profile suite is
the wrong instrument for the question, which is a fact about the platform worth having.

### Absolute numbers, every parametric profile in the frozen suite

Delivered one-way delay in ms, `single → min-of-two (improvement)`, two i.i.d. paths of the
**same** profile:

| Profile | Base / J (ms) | p50 | p95 | p99 |
|---|---|---|---|---|
| `lan` | 2 / 1 | 2.00 → 1.59 (−0.41) | 2.90 → 2.55 (−0.35) | 2.98 → 2.80 (−0.18) |
| `50ms-5j` | 50 / 5 | 50.00 → 47.93 (−2.07) | 54.50 → 52.76 (−1.74) | 54.90 → 54.00 (−0.90) |
| `150ms-20j-0.5loss` | 150 / 20 | 150.00 → 141.72 (−8.28) | 168.00 → 161.06 (−6.94) | 169.60 → 166.00 (−3.60) |
| `300ms-60j-2loss-bursty` | 300 / 60 | 300.00 → 275.15 (−24.85) | 354.00 → 333.17 (−20.83) | 358.80 → 348.00 (−10.80) |
| `jitter-60ms` (ADR 0005, the largest J in any family) | 50 / 60 | 50.00 → 25.15 (−24.85) | 104.00 → 83.17 (−20.83) | 108.80 → 98.00 (−10.80) |

**Hard ceiling, stated as a bound:** duplication over `n` identical paths can never reduce
delivered delay below `Base − J`. The **entire** budget available to my mechanism on any frozen
profile is `J`, one-sided: at most **60 ms**, and only in the `n → ∞` limit. With two paths the
best case anywhere in the suite is **−24.9 ms at p50 and −20.8 ms at p95**, on
`300ms-60j-2loss-bursty` — about **6% of a 354 ms p95**. On `lan` and `50ms-5j` it is under 2 ms,
i.e. inside anyone's noise floor.

### Two *different* profiles: the min degenerates

The interesting-sounding case is two paths with different delay distributions. It is mostly not
interesting, and here is why in one line:

> If the supports are disjoint — `Base_A + J_A < Base_B − J_B` — then `min(D_A, D_B) = D_A`
> **exactly, always**. The slow path contributes nothing to delivered delay whatsoever. It is
> pure loss redundancy at 100% bandwidth overhead.

Among the frozen parametric five, **every pair has disjoint support**: `lan` [1,3],
`50ms-5j` [45,55], `150ms-20j` [130,170], `300ms-60j` [240,360]. So **no pair of frozen presets
produces any delivered-delay effect at all** — every such experiment reduces to "use the fast
path", which is trivially better and needs no mechanism.

The min only does something when the supports overlap, i.e. `|Base_A − Base_B| < J_A + J_B`. The
`combo__delay-<N>ms__jitter-<N>ms__loss-<N>pct` family from `docs/adr/0006` lets a caller name any
`(delay, jitter, loss)` triple, so **overlapping pairs are expressible today with no new
profile** — e.g. `combo__delay-150ms__jitter-40ms` against `combo__delay-170ms__jitter-40ms`.
Good news for cost; it does not change the ceiling, which is still `≤ J`.

### The loss side, which is where the real effect is — and it is uninstrumented

Gilbert-Elliott steady-state loss is `π = p / (p + (1 − r))` with `p =
LossProbabilityAfterDelivered`, `r = LossProbabilityAfterLost`.

| Profile | `p` / `r` | `π` (one path) | `π` (two independent paths, both must lose) | expected burst length, one path `1/(1−r)` | expected joint burst `≈ 1/(1 − r²)` |
|---|---|---|---|---|---|
| `150ms-20j-0.5loss` | 0.005 / 0.005 | 0.500% | **0.0025%** | 1.005 | 1.00 |
| `300ms-60j-2loss-bursty` | 0.00612 / 0.70 | 2.00% | **0.040%** | 3.33 | 1.96 |

**Duplication over two independent paths is a ~50× loss reduction on the worst frozen profile, and
it destroys the burst structure as well** (joint bursts require both chains bad at once). That is
by far the largest effect my mechanism produces — roughly two orders of magnitude, against a
delivered-delay effect of 6%.

**And this project cannot measure it.** `docs/metrics.md` §3 defines loss rate, burst-length
distribution, jitter, reordering rate and goodput — and I verified by grep over `core/` that the
**complete** set of metric names emitted anywhere is `owd_uplink_ms`, `owd_downlink_ms`,
`correction_magnitude_mm`, `correction_magnitude_deg`, `time_to_convergence_ms`, `jerk_mm_s3`,
`prediction_position_error_mm`, `prediction_orientation_error_deg` (plus host-only `m2p_ms`).
**Every single §3 network metric is defined and emitted by nothing.**

The irony is total and I want it on the record:

- The main benefit of my candidate is a **loss-rate** and **burst-length** improvement. Neither is
  emitted.
- The main cost of my candidate is **2× bytes on the wire**. **Goodput** — the metric that prices
  exactly that — is defined in §3 and is not emitted.
- The main side effect of my candidate is **guaranteed reordering** (two paths, different delays).
  **Reordering rate and max displacement** are defined in §3 and are not emitted.

**Three metrics, all already defined, all needed to evaluate this candidate, none of them
instrumented.** I may not add or redefine a metric, so I record this as the gating requirement:
*path diversity is not honestly measurable in this repo until §3's loss, goodput and reordering
metrics are emitted by something.* That instrumentation work is independent of my candidate,
smaller than it, and would benefit candidate 1 (payload redundancy) at least as much.

---

## Q-metrics: which existing metric moves, in which direction, on which profile

Named metrics only, and an explicit "the instrument does not exist" where that is the truth.

| Mechanism | Metric that would move | Direction | Profiles where anything is visible | Honest assessment |
|---|---|---|---|---|
| Duplication over 2 paths | `owd_uplink_ms`, `owd_downlink_ms` | **down**, p50 by `0.414·J`, p95 by `0.347·J` | only where the two paths' supports overlap; **no pair of frozen presets qualifies**. Needs ADR-0006 combined names. Max effect 24.9 ms p50 on a `J=60` pair | real but small, and the tail moves *less* than the median |
| Duplication over 2 paths | loss rate, burst-length distribution | **down** ~50× and burst 3.33 → 1.96 on `300ms-60j-2loss-bursty` | that profile and `loss-<N>pct` family | **instrument does not exist** (§3 defined, emitted by nothing) |
| Duplication over 2 paths | goodput (2× bytes) | **down** (worse) | all | **instrument does not exist** |
| Duplication over 2 paths | reordering rate, max displacement | **up** (worse) | all two-path configs by construction | **instrument does not exist** |
| Duplication over 2 paths | `prediction_position_error_mm` / `_deg` | **down** slightly — fewer lost commands means the plant tracks the operator better, so the predictor's target moves less erratically | `300ms-60j-2loss-bursty` mainly | this is an *indirect* readout of a loss improvement through a prediction metric. Measurable today, but it is a proxy and should be labelled one |
| Duplication over 2 paths | `correction_magnitude_mm/deg`, `jerk_mm_s3`, `time_to_convergence_ms` | **down** slightly (fewer gaps → fewer large corrections) | same | same caveat; and these are reconciler metrics, so a transport experiment moving them is easy to misattribute |
| Path switching (told which path) | as duplication for delay; no loss benefit; no bandwidth cost | mixed | needs a non-stationary profile, which does not exist | **not falsifiable against the frozen suite** |
| Connection racing (one-time) | none | — | none | **not falsifiable against a stationary profile**; scoped out |
| Pacing | none | — | none | **non-candidate**: inelastic 58 kbit/s, one 73-byte frame per 10 ms tick. No burst exists to pace |
| Temporal spreading of a redundant copy | belongs to candidate 1's metric story | — | `300ms-60j-2loss-bursty` only (`L = 3.33` datagrams = 33 ms; offset must be ≥ 4 ticks to decorrelate, costing ≥ 40 ms of recovery latency) | analytic result handed to candidate 1 |

**Displayed-pose accuracy** has no metric on this axis (the brief's known constraint 5). My
mechanism does not trade accuracy for anything directly, so this does not bite me — but note the
row above where prediction/correction metrics move as *indirect* readouts of a transport change.
That is the closest thing to an accuracy trade here and it is a measurement-attribution hazard,
not an accuracy loss.

---

## The OWD survivorship trap, which is load-bearing for exactly this candidate

`owd_uplink_ms` and `owd_downlink_ms` are emitted only from
`OperatorEndpoint.RecordOneWayDelayMetrics`, reached only after a downlink reply is matched to an
in-flight `LatencyTrace` by `Sequence`. **A lost packet emits nothing.** The OWD population is
therefore conditioned on round-trip survival, and *my candidate changes which packets survive*.
Pooled OWD percentiles across a one-path and a two-path configuration compare different
populations.

For this emulator specifically, the loss draw (at `Send`) and the delay draw (at drain) are
statistically independent, so single-path survival is *not* delay-biased **by the loss model**.
But there is a second, larger, and delay-**correlated** truncation that I do not think has been
written down before, and I found it by reading rather than measuring:

> `OperatorEndpoint`'s in-flight ring is a fixed 64 slots written round-robin
> (`InsertInFlight` overwrites `_inFlightNextIndex` unconditionally), and `SweepCommand` uses
> `InFlightCapacity = 64` with `stepIntervalTicks = 100000` at 10 MHz = **10 ms/step**. So a
> sequence's trace is **evicted 640 ms after submission**, matched or not.
>
> On `300ms-60j-2loss-bursty` the round trip is `U + D` with `U, D ~ Uniform[240, 360]` ms
> independently, plus up to ~20 ms of step quantization. `U + D` is triangular on [480, 720] with
> mean 600. `P(U + D > 620) = (720−620)² / (240·120) = 0.347`.
>
> **Roughly 22–35% of round trips on the worst frozen profile never produce an OWD sample at all,
> and they are systematically the slowest ones.** The measured `owd_*` distribution on that
> profile is truncated from above, so its p95 and p99 are biased low.

Two consequences, and the second is the one that would burn a multipath experiment:

1. This is a **pre-existing measurement artifact affecting every result already recorded on
   `300ms-60j-2loss-bursty`** (and any `combo__delay-` point past ~300 ms). It is not mine and I am
   not fixing it — but it should be checked empirically and, if confirmed, either the ring should
   be sized from the profile's delay or the affected profile's OWD numbers carry a caveat. I did
   **not** run a sweep to confirm this; it is arithmetic over the constants, and it needs
   verification before anyone acts on it.
2. **Duplication shifts the delay distribution down, which pulls previously-evicted slow round
   trips back inside the 640 ms window.** Those samples re-enter the population *at the slow end*.
   So a genuine delivered-delay improvement can show up as a **worse measured p95** — the
   improvement adds samples above the old truncation point faster than it moves the retained ones
   down. This is precisely the "sanity-check a surprising result" failure mode, running backwards:
   here it would manufacture a surprising *loss*.

**How a sweep would have to be structured to make a delivered-delay claim honestly:**

1. Choose a profile pair whose round trip is comfortably inside the ring window
   (`RTT_max + 20 ms << 640 ms`), i.e. base delay per path ≲ 200 ms. That rules out
   `300ms-60j-2loss-bursty`, which is the one profile where the effect is largest. This is the
   crux of the difficulty.
2. **Set loss to zero on both paths** for the delivered-delay claim, so the surviving population is
   the whole population and the two configurations are comparable by construction. Measure the
   loss benefit in a *separate* experiment with a *different* instrument (which does not exist).
   Do not try to read both effects off one pooled `owd_*` distribution.
3. Report the emitted **sample count** per configuration alongside every percentile, and refuse
   the comparison if they differ by more than a percent or two. Different `n` means different
   populations, and `docs/metrics.md`'s "the tail is what the operator perceives" is only true if
   the tail is in the sample.
4. Hold the predictor and reconciler fixed at the baseline (`none`, `snap`) so nothing downstream
   can absorb or amplify the transport change — the brief's coupled-axes rule applied to
   transport × prediction.
5. Multiple seeds, percentiles only, baseline row present. Note that the two paths need
   *different* seeds (`SweepCommand` already offsets the downlink RNG by `seed + 1`, so an
   established convention exists), and that two paths sharing one seed would be perfectly
   correlated — the worst-case rather than the best-case model, and arguably the more honest one.

---

## Results

Nothing was built and no `results/` directory was produced, per the brief. The findings are the
arithmetic above and the code-reading determinations below.

### Q1 — decorator or new contract?

**Decorator. No `ITransport` change.** A duplicating or told-which-path-to-use transport over two
`ITransport`s satisfies every clause of `Contracts/ITransport.cs` as written; the interface's own
doc names duplication as an allowed channel behaviour. `MaxPayloadBytes = min(a, b)`; `Send` on
both, true if either took it; `TryReceive` drains both into one min-heap by reported arrival tick,
structurally identical to `EmulatedTransport.DrainInner` with two sources; `Reset` resets both.
A **new contract is needed only** for transport-level diagnostics (`ITransport` has no
`Diagnostics` property, unlike `IPredictor`/`IReconciler`) or for a transport that must parse
payloads — neither is required by the minimal experiment.

### Q2 — where does dedup live?

**Nowhere new.** Uplink duplicates are absorbed by the plant, by contract
(`IRobotPlant.Command`: "ignores a stale or repeated frame") and in fact
(`RigidBodyPlant.cs:156`, `if (command.CaptureTicks <= _lastAcceptedCaptureTicks) return;`).
Downlink duplicates are absorbed by `OperatorEndpoint.TryTakeInFlight`, which clears the occupied
flag on match, so the second copy is silently skipped — meaning `ClockSync`, both OWD metrics,
`_lastAckSequence` and `ObserveRobotState` fire **exactly once per sequence, on the first copy to
arrive**. That take-once ring is an accidental but exact implementation of min-over-paths.

**The catch:** `RobotEndpoint` replies once per *received datagram*, not per *accepted command*
(`Pipeline/CLAUDE.md` requirement 3, deliberate). So **duplicating the uplink also duplicates the
downlink**. There is no uplink-only two-path configuration without changing `RobotEndpoint`, and
any bandwidth accounting must say 2× on both directions.

### Q3 — headless two-path emulation?

**Possible today, with no new profile and no new infrastructure beyond the decorator.** Two
`EmulatedTransport`s with two `NetworkProfile`s and two `SeededRng`s behind one decorator. Faithful
for: differing base delay, jitter width, loss rate, burst length, reordering, and recorded traces.
**Not faithful for: shared-bottleneck delay correlation, cross-path loss correlation, finite-
duration outages, handover, path-switch cost, and load-dependent congestion.** Independent paths
are the best case; **an uncorrelated two-profile model overstates the benefit, and the loss benefit
in particular is `π_A·π_B` under independence versus `min(π_A, π_B)` under perfect correlation —
two orders of magnitude apart on the same nominal profiles.** A correlated two-path profile does
not exist, I may not add one, and adding one is an ADR because `NetworkProfile`'s doc scopes it to
a single parametric link.

### Q4 — does pacing mean anything?

**No.** 100 Hz, one fixed 73-byte frame per tick, ~58 kbit/s uplink and ~46 kbit/s downlink,
inelastic. There is no burst to pace and the stream is already perfectly paced by construction.
**Family (b) is a non-candidate**, with the single exception of temporal spreading of a redundant
copy, which belongs to candidate 1 and whose entire answer is the analytic paragraph in Q4 above:
on `300ms-60j-2loss-bursty` the offset must be ≥ 4 ticks (40 ms) to beat the 3.33-datagram expected
burst, and on the other frozen profiles `L ≈ 1` so there is nothing to decorrelate.

### Q6 — is it distinguishable by measurement?

**From something already implemented:** yes in principle — no existing implementation changes
delivered delay, so a delay shift is unambiguous. But **the shift is small enough (≤ 6% of p95 in
the best frozen case, under 2 ms on `lan`/`50ms-5j`) that it is at real risk of being inside the
seed spread**, which `docs/metrics.md` §8.3 says is not a result. I would want the seed spread of
`owd_uplink_ms` p95 on the target profile *before* committing to build, and that number can be
read off an existing run in `results/` for free.

**From candidate 1's payload redundancy:** **largely no, and this is the sharpest strike against
my candidate.** Candidate 1's N-frame redundancy buys the same loss-tolerance benefit — the
dominant effect by two orders of magnitude — on **one** path, with **no** second link, **no** new
contract question, **no** reordering side-effect, and a bandwidth cost of `N×` the 73-byte payload
rather than `2×` everything on a whole second network interface. On a 58 kbit/s stream, bandwidth
is free, so the cheap mechanism wins outright. The only thing multipath buys that candidate 1
cannot is the delivered-delay shift, and that is the small effect. **If both were built and
measured on the current instruments, they would be hard to tell apart, and the one that is
indistinguishable-but-far-cheaper is not mine.**

### Q8 — how this assessment could flatter itself

Required section, and I have four:

1. **I chose the analytic route, and the analytic route is the one my candidate loses.** A closed
   form over a uniform distribution is exactly the setting where min-of-two looks worst. If the
   real question is "what happens on a *real* link", the frozen uniform profiles are a poor model
   of it and my ceiling of `J` is an artifact of the model, not a fact about multipath. I believe
   the negative conclusion *for this repo as it exists*, and I would not extend it to reality. The
   honest form of my result is: **"multipath cannot be shown to help against this profile suite,
   because this profile suite has no tail"** — which is as much a criticism of the instrument as
   of the idea, and I should not let the tidiness of the arithmetic disguise that.
2. **I may have made the decorator sound cheaper than it is by pricing only the happy path.** A
   `DuplicatingTransport` needs the full repo treatment: `Reset()` semantics tested against
   as-constructed state, an allocation-assertion test, doc comments at this repo's density, RNG
   determinism across two sub-transports with an interleaved poll schedule (and note
   `Transport/CLAUDE.md`'s warning that "a *different* poll schedule with the same seed is a
   different realization" — draining two inner transports changes the poll schedule of both, so
   **a two-path run and a one-path run at the same seed are different RNG realizations and are not
   common-random-number comparable**). I noticed that late and it is a real, non-obvious cost that
   weakens the cleanest form of the experiment.
3. **The in-flight-ring finding is arithmetic I did not verify empirically.** I was not permitted
   to run a sweep, and I am reporting a ~30% sample-loss figure derived from constants. If I am
   wrong about the timing (e.g. if replies are matched a step earlier than I traced), the number
   changes. It is stated as a prediction needing verification, and I have flagged it as such —
   but a reader skimming the table could easily take it as measured. It is not.
4. **I own the mechanism and not the estimator, which lets me push the hard half away.** Every
   version of this candidate that is actually interesting — path switching, per-packet redundancy
   decisions à la ReMDeLa, "predict which path will be good" — depends entirely on a path-quality
   estimator I have declared out of scope. It would be flattering to my assessment to say the
   mechanism is cheap and the estimator is someone else's problem. The truth is that **the cheap
   half is the useless half**: unconditional duplication (mechanism only, no estimator) is what my
   arithmetic just showed to be small and redundant with candidate 1, and the version with an
   estimator is the expensive one nobody has scoped.

---

## What it costs to build here

Split as the brief asks. "ADR" means `docs/adr/` first, per root `CLAUDE.md`'s "new question
entirely → new interface in `Contracts/`, new folder, `Pipeline/` learns to wire it. This is an
architecture change: write an ADR first."

| Piece | Cost | ADR? | Notes |
|---|---|---|---|
| **Headless two-path emulation** | **already possible**, zero new infrastructure | no | Two `EmulatedTransport`s + one decorator. No new profile: ADR-0006's `combo__` family already names any `(delay, jitter, loss)` triple |
| **Core-side multipath transport** (`DuplicatingTransport`) | **small: ~250–350 lines + tests**, structurally a two-source `EmulatedTransport` | **no** — fits `ITransport` unchanged | This is the surprise. The mechanism is the *cheapest* part of the whole candidate |
| **Registry / builder shape** | **medium** — this candidate is the **first** thing that genuinely needs `Transports` to carry more than one entry | **borderline; I would argue yes** | `Registry/CLAUDE.md` states the problem exactly: `EmulatedTransport` is unregistered because its ctor shape differs, and it "needs an entry (its own shape, or a small builder type) the moment a sweep needs to select a transport by name". A decorator over *two* transports plus two profiles plus two RNGs is a third shape again. Note `AuditCommand.cs:226` **deliberately excludes `Transports` from the registry-completeness check**, so the gate will *not* catch a missing entry — the discipline here is manual |
| **Sweep can vary the transport** | **medium, and entirely in `core/Teleop.Eval/`** (off-limits to me) | no | `ExperimentConfig` has no transport field; `SweepCommand` hardcodes `MakeTransport` and sizes the loopback by `RawPoseCodec.EncodedSize`. Needs: a config field, the axis threaded through `ResolvedStack`/output-directory naming, `ManifestWriter`, and `analysis/`'s label lookups. **Without this there is no way to run the experiment at all** — which is the real answer to known constraint 1's "work out what that costs you": it costs *everything*, because a Core decorator nobody can select from a YAML produces no result |
| **Correlated two-path profile** | **medium and unavoidable for a credible result** | **yes, ADR** | `NetworkProfile` is scoped to one parametric link by its own doc. Needs a shared-loss and shared-delay component between two paths. And per my search, **there is no off-the-shelf few-knob model to copy** |
| **§3 network metrics** (loss rate, burst-length distribution, goodput, reordering rate + max displacement) | **medium** — needs a counter somewhere with visibility of sends and receives, plus emission-cadence design | **probably yes** — where does a transport-level metric get emitted from, given `ITransport` has no `IMetricSink`? | **This is the actual blocker.** All four are already defined in `docs/metrics.md`; none is emitted. Without them the candidate's dominant effect (loss ~50×) and its entire cost (2× bytes) are both invisible. **This work is independent of my candidate, smaller than it, and helps candidate 1 more** |
| **Playout prerequisite** | **medium–large** | **yes, ADR** (`Pipeline/` learning a new contract) | see below |
| **Real-hardware multipath** | **cannot be done from this box** | ADR + human | Sockets, two NICs, Wi-Fi + cellular on the Quest — `Bridge/UdpTransport.cs`, `unity/`, Windows box. **Blocked on human review by rule.** I record it as blocked and do not price it further |

### On the playout prerequisite — I agree, with a qualification

The brief asks whether a reorder-tolerant playout buffer is a *prerequisite* for multipath, since
two paths guarantee reordering. **I agree for any configuration where the two paths' delay supports
overlap** — which is precisely the only configuration where the delivered-delay benefit exists.
Overlapping supports means frequent inversions; `OperatorEndpoint` hardcodes
`t_playout = t_operatorRecv`, so an inverted arrival is consumed immediately and out of order, and
the reconciler sees a *backwards* authoritative sample. `ObserveRobotState` feeds
`_robotStatePredictor.Observe(sample)` with a `captureTicks` that has gone backwards, which is a
predictor input the predictors were not designed for.

The qualification: for the **disjoint-support** case (fast path + slow redundant path), inversions
are rare or impossible and no buffer is needed — but that case has no delay benefit either. So:
**the version of multipath worth building requires playout; the version that does not require
playout is not worth building.** That is a clean statement of the dependency and it says playout
comes first.

`Buffering/` has contracts and config types and zero implementations. Wiring `IPlayoutPolicy` into
`Pipeline/` is an ADR by root `CLAUDE.md`'s rule, and `Pipeline/CLAUDE.md` says the hardcoded line
should be *replaced*, not added around.

---

## The falsifier — cheapest single experiment that could kill it

**The cheapest falsifier is the arithmetic above, and it already ran.** That is the most valuable
outcome available here and it cost no code:

> **Falsifier (analytic):** if the min of two draws from the repo's delay model cannot move an
> emitted percentile by more than the seed-to-seed spread of that percentile, the candidate is
> dead on the current instruments.
>
> **Result:** the entire available budget is `J`, one-sided, and two paths capture
> `0.414·J` at p50 and `0.347·J` at p95. Best case anywhere in the frozen suite: **−20.8 ms on a
> 354 ms p95**, ~6%, on the one profile where the OWD instrument is *also* truncated by in-flight
> ring eviction. On `lan` and `50ms-5j` the effect is **under 2 ms**. And the improvement gets
> *smaller* deeper into the tail, inverting the literature's central claim, because the model's
> jitter is bounded uniform by design.

If someone wants an empirical falsifier anyway, the cheapest one needs **no new code at all**:

> **Zero-code empirical falsifier:** take the seed spread of `owd_uplink_ms` p95 from an existing
> `results/` run on the profile you would target. If that spread is comparable to `0.347·J` for
> that profile, no two-path experiment on that profile can produce a result, and you have learned
> that for the cost of one `analysis/` query. `docs/metrics.md` §8.3: "A difference inside
> run-to-run variance is not a result."

I did not run that query — I was told not to produce `results/` output and I read the constraint
conservatively as also not launching sweeps. **Running that one query is the single highest-value
next action for whoever picks this up, and it is minutes of work.**

---

## Bottom line

**Do not build it next.** It is the most expensive and least ready of the four directions: the
delivered-delay benefit is capped at ~6% of p95 on the best frozen profile and shrinks into the
tail; the large benefit (~50× loss reduction) is real but **duplicates what candidate 1 buys more
cheaply on one path**, and is **unmeasurable today because §3's loss, goodput and reordering
metrics are defined and emitted by nothing**. The mechanism itself is cheap — a decorator over two
`ITransport`s with no contract change, which is the one genuinely good news here — but everything
around it (transport axis in the sweep, a registry shape, a correlated two-path profile, the §3
instrumentation, and a playout buffer to absorb the reordering it creates) is three ADRs and a
`Teleop.Eval` refactor, in service of the smallest of the effects.

**Build it after:** (1) §3's network metrics are emitted by *something*, (2) `Buffering/` has a
real `IPlayoutPolicy` wired into `Pipeline/`, and (3) somebody has an honest correlated two-path
impairment model. Any earlier and the result is an upper bound measured on the wrong instrument.

---

## Verification

I built nothing, so there is nothing of mine to verify. I ran `just core-check` once, on the
unmodified tree, to confirm the code I read was green:

```
dotnet test  -> Failed: 0, Passed: 558 (Teleop.Core.Tests)
                Failed: 0, Passed:  36 (Teleop.RobotHost.Tests)
                Failed: 0, Passed:  28 (Teleop.RobotArm.Tests)
                Failed: 0, Passed:   3 (Teleop.Eval.Tests)
verify: PASS -- basic-session.tlog replays byte-identical across two passes
audit:  PASS -- no invariant violations
```

The only file I created is this log. `git status` should show exactly one untracked file in
`docs/research-log/`. Nothing committed, per the brief.

## Left undone / for a human

1. **The single highest-value follow-up, minutes of work:** read the seed spread of
   `owd_uplink_ms` p95 out of an existing `results/` run on `150ms-20j-0.5loss` and
   `300ms-60j-2loss-bursty`, and compare it against `0.347·J` (6.9 ms and 20.8 ms respectively).
   That settles whether *any* two-path delivered-delay experiment on this suite could ever produce
   a result.
2. **Verify the in-flight-ring eviction arithmetic empirically.** My prediction: on
   `300ms-60j-2loss-bursty`, 22–35% of submitted commands produce no `owd_*` sample, and the loss
   is delay-correlated (the slowest round trips), biasing that profile's measured p95/p99 *low* in
   every result already recorded. `InFlightCapacity = 64` × 10 ms/step = a 640 ms window against a
   480–720 ms round trip. **I did not measure this. If it holds it is a bug in the measurement, not
   in any algorithm, and it is not mine to fix.**
3. **Handoff to candidate 1 (payload redundancy):** the interleaving offset on
   `300ms-60j-2loss-bursty` must be **≥ 4 ticks (40 ms)** to beat the 3.33-datagram expected burst
   length; smaller offsets buy nothing because the burst that takes the original takes the copy.
   On every other frozen profile `L ≈ 1.005`, so there is nothing to decorrelate and the offset
   should be zero. That is the whole temporal-spreading design space on the frozen suite.
4. **Handoff to candidate 3 (estimator):** the mechanism half of path switching is a small
   decorator; the estimator half does not exist and is where all the difficulty is. Also: path
   switching is **not falsifiable against the frozen profile suite at all**, because every profile
   is stationary within a trial. A non-stationary profile is an ADR and a change to a frozen set.
5. **Blocked on human — real-hardware multipath.** Two NICs, Wi-Fi + cellular, sockets,
   `Bridge/UdpTransport.cs`, `unity/`, Windows box. Off-limits from this box by rule. Recorded as
   blocked, not attempted.
6. **Argued-for ADRs I did not write** (the brief forbids it): (a) a correlated two-path
   `NetworkProfile` extension; (b) emission of `docs/metrics.md` §3's network metrics, including
   where a transport-level metric gets its `IMetricSink` from; (c) `IPlayoutPolicy` wired into
   `Pipeline/`. (b) is the one I would write first, and it is not really about multipath.
7. **One `ITransport` doc-comment ambiguity a human should rule on:** "an implementation must
   never return the same datagram twice" — I read this as forbidding re-delivery of one datagram,
   not as requiring cross-path dedup, and the type doc's own "may be ... duplicated" supports that
   reading. My entire decorator-not-contract determination rests on it. If a human reads it the
   other way, multipath becomes a contract change and the price goes up materially.

