# Payload redundancy / forward error correction on the uplink — feasibility survey

**Agent:** Claude Opus 5 (parallel researcher, Transport survey run — candidate 1 of 4)
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-af039c6d7f3f1d29f`
**Branch:** `worktree-agent-af039c6d7f3f1d29f`
**HEAD at start:** `06c0ab5b6f570efa028002897c856c566041663b`
**Date:** 2026-09-09

**This is a survey and feasibility assessment. No code was written, no test was added, no
experiment was run, no `results/` directory was produced.** The only file this run creates is
this log. That is the brief. A well-argued "do not build" is the intended shape of a successful
outcome here.

## Seed self-check

| Check | Expected | Observed | Verdict |
|---|---|---|---|
| `git rev-parse HEAD` | `06c0ab5b6f570efa028002897c856c566041663b` | `06c0ab5b6f570efa028002897c856c566041663b` | PASS |
| Worktree path | own worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-af039c6d7f3f1d29f` | PASS |
| Branch | own branch | `worktree-agent-af039c6d7f3f1d29f` | PASS |
| `dotnet test` | green | 558 passed, 0 failed, 0 skipped | PASS |
| `Teleop.Eval -- verify` | PASS | `basic-session.tlog` replays byte-identical across two passes | PASS |
| `Teleop.Eval -- audit` | PASS | no invariant violations in `Teleop.Core` or its built assembly | PASS |

Tree was green on arrival and nothing in this run changed it. The gates were run once, read-only,
to establish that baseline.

## Scope

**Mine:** what goes inside a datagram on a single path under a fixed (non-adaptive) policy —
N-frame repetition of past `CommandFrame`s, XOR/parity packets, Reed–Solomon / block FEC,
fountain codes, and interleaving of redundancy across time.

**Not mine, owned by other researchers in this run:** sending *future intended motion* (the
`trajectory` codec, candidate 2); making redundancy depth *adapt* to a link estimate
(candidate 3); multipath / racing / pacing (candidate 4). Handoffs to 2 and 3 are marked below.

**Framing I am held to:** a codec cannot reduce one-way delay. OWD belongs to the network. What
redundancy changes is the *impact* of loss — fewer gaps the receiver must coast through. Every
claim below names an existing metric from `docs/metrics.md` and a direction, or says explicitly
that no such metric exists.

---

## Hypothesis (written before any analysis of the pipeline internals)

**H1.** Repeating the last N `CommandFrame`s in every uplink datagram reduces the number and
length of the gaps the robot side must coast through under burst loss, and therefore reduces
`prediction_position_error_mm` p95/p99 and `correction_magnitude_mm` p95/p99 on the profiles with
nonzero loss (`150ms-20j-0.5loss`, `300ms-60j-2loss-bursty`, `loss-<N>pct` for N ≥ 0.5), at a
bandwidth cost proportional to N. It changes nothing on `lan`, `50ms-5j`, `jitter-<N>ms`,
`delay-<N>ms`, `loss-0pct` — those have zero loss and there is nothing to recover.

**Falsifier for H1 (stated before the investigation):** if the robot-side consumer discards
recovered past frames — or if applying them produces a state trajectory identical to not applying
them — then N-frame redundancy is a no-op and no metric can distinguish it from `raw`, on any
profile.

**Result: H1 is falsified, by that exact falsifier, on a code-reading argument, for every
mechanism in my scope.** The detail is in "The structural finding" below. I did not need to run
anything to establish it, and the argument is cheaper and more decisive than any percentile would
have been.

---

## 1. The mechanism, its sub-variants, and what prior art claims

### 1.1 The family in one paragraph

Uplink payload redundancy accepts a bandwidth increase in exchange for the receiver being able to
reconstruct the content of datagrams that never arrived. On a hard-latency-budget control link,
retransmission is not available (an ARQ repair costs a forward + backward + forward delay, which
at this project's 150–300 ms profiles is 450–900 ms and therefore useless), so the redundancy has
to be *proactive*: it travels with, or immediately after, the data it protects. The design space
splits on **where the redundancy sits** (inside the same datagram as fresh data, or in a separate
parity datagram) and on **how deep in time it reaches** (how many consecutive losses it survives).
Those two choices set the bandwidth multiplier and, critically, the **recovery latency** — how
long after the loss the receiver can reconstruct the missing content.

### 1.2 Sub-variants, and what prior work claims about each

I record for each what the source proposes, what it claims, and under what conditions it was
measured, because the operating point matters more than the headline. **None of this is evidence
about this repo.** All URLs accessed 2026-09-09.

**(a) N-frame repetition (game-netcode input redundancy).** Every datagram carries the newest
frame plus verbatim copies of the previous N−1. Recovery is instantaneous — the copy arrives in a
datagram that is itself fresh — and it tolerates a run of N−1 consecutive datagram losses with
zero residual frame loss. Prior art: Quake 3's networking model
(<https://fabiensanglard.net/quake3/network.php>, <https://www.jfedor.org/quake3/>) sends
unreliable UDP with sequence numbers and never independently acknowledges commands; GGPO-style
rollback netcode explicitly bundles redundant past inputs in every packet
(<https://www.ggpo.net/>, <https://www.snapnet.dev/blog/netcode-architectures-part-2-rollback/>,
<https://github.com/gregorik/Rollback-Core>). **Measurement conditions:** these are engineering
practice descriptions, not measured studies — no loss model, no latency budget, no packet size is
stated anywhere I found. What matters far more than the absent numbers is the *reason* the
technique exists there: a rollback simulation is **deterministic and path-dependent**, so every
input in the sequence must eventually be applied, and a late-arriving old input is used to
re-simulate a mispredicted past. Their payloads are a few bytes of button bitmask at 60 Hz, so
N = 8 or more is nearly free. Neither of those properties holds here (see §2).

**(b) Opus in-band FEC / LBRR.** The encoder re-encodes the *previous* frame at low bitrate and
tucks the small copy inside the *next* packet; if packet N−1 is lost but N arrives, the decoder
plays the degraded backup rather than a concealment guess. Prior art:
<https://blog.mozilla.org/webrtc/audio-fec-experiments/>,
<https://wiki.hydrogenaudio.org/index.php?title=Opus>. **Measurement conditions:** 20 ms audio
frames, tens of bytes per frame, VoIP bitrates; Mozilla's telemetry-driven analysis notes LBRR is
only enabled above roughly 1% observed loss and that at Firefox's then-default 40 kbps two-channel
setting it never engages at all — i.e. the technique is claimed useful only in a specific
loss-and-bitrate corner. Note that this is depth-1 (N = 2) repetition with a *lossy* copy: the
redundancy is cheap because the backup is allowed to be worse than the original. That degradation
trick has an analogue here (delta-code or quantize the older copies) and is worth remembering if
anyone ever does build this — but it overlaps candidate 2's and the `delta-quant` row's territory.
The reason it works for audio is the same reason it fails here: **audio is a sequence in which
every sample must be played**, so recovering an old frame recovers real, irreplaceable content.

**(c) XOR / parity packets (RFC 5109).** A parity datagram over a group of k media datagrams lets
the receiver recover any single loss within the group. Prior art:
<https://www.rfc-editor.org/rfc/rfc5109>. **Measurement conditions:** the RFC is a payload-format
specification and reports no measurements at all; it states the tradeoff qualitatively ("the more
FEC packets as a fraction of source packets, the stronger the protection and the greater the
bandwidth"). Structurally: half the bandwidth of 2× repetition for equal single-loss protection,
but recovery requires the *whole group minus one* to have arrived, so recovery latency is up to
k−1 packet intervals, and **it cannot recover two losses in a group** — which makes it close to
worthless against the burst channel this repo actually models.

**(d) Block FEC / Reed–Solomon (k-of-n MDS).** Any k of n datagrams reconstruct all k source
datagrams. Best bandwidth efficiency per unit of recovery depth of anything in this list.
Structurally: recovery latency is bounded below by the time to receive k of n, i.e. up to n−1
packet intervals. **Measurement conditions:** the delay-constrained-coding literature is the right
place to look here — Fong/Khisti et al.'s streaming codes work
(<https://arxiv.org/pdf/1801.04241>, <https://arxiv.org/abs/1909.06709>) proposes codes that
correct both arbitrary and burst erasures under a *fixed decoding delay*, and states explicitly
that FEC rather than ARQ is the right family for low-latency links because retransmission injects
redundancy at a highly non-uniform rate and costs a 3-way delay per repair. The 2019 adaptive
paper's evaluation is simulation over statistical losses and real-world packet-loss traces, with
the motivating application being **audio** (it names crackling and jitter as the artifacts), and
it claims "significantly higher performance" over uncoded UDP and non-adaptive FEC — no absolute
numbers in the abstract, and the packet size / cadence / RTT are not stated there. Its adaptive
half is candidate 3's problem, not mine; its non-adaptive half is directly the "fixed policy block
FEC" row of my scope, and its key structural message for me is that **the achievable
loss-tolerance is a function of the delay budget you are willing to spend on decoding**, which for
a 10 ms-cadence control stream is close to zero.

**(e) Fountain / rateless codes (RaptorQ).** Designed for large objects over erasure channels with
unknown loss, where the receiver collects slightly more than k symbols in any order. Fundamentally
mismatched to a 73-byte-per-10-ms stream where each unit is independently urgent: the coding gain
comes from the block being large, and here the block cannot be large without becoming stale. Heavy
implementation (and Core has zero NuGet dependencies, invariant 7, so it would be hand-written).
Recorded here so nobody proposes it again.

**(f) Interleaving / temporal spreading.** Not a mechanism, a modifier on (c)/(d): spread a block's
symbols across a longer window so a burst hits at most one symbol per block. It buys burst
tolerance by **spending recovery latency**, exactly the wrong currency here.

### 1.3 A search that found nothing useful, recorded so it is not repeated

I looked for measured work applying erasure coding or payload redundancy to *robot teleoperation
command streams* specifically (search: "bilateral teleoperation UDP packet loss redundant
transmission command packets robot experiment evaluation"). The teleoperation literature that came
back treats packet loss as a **control-stability / passivity** problem — wave variables, delay-
robust controllers, transparency analysis (e.g. <https://arxiv.org/pdf/1711.03605>,
<https://arxiv.org/pdf/2008.06685>) — not as an erasure-coding problem. I found no measured study
of FEC on a pose-command uplink. That absence is itself informative and consistent with §2: for a
setpoint-tracking receiver, recovering old setpoints is not the interesting move; making the
receiver behave better through a gap is. Do not re-run this search.

---

## 2. The structural finding: past-frame redundancy is a provable no-op on this uplink

This is the core of the assessment and it kills the whole family in my scope, not just one variant.

### 2.1 The chain of code

1. **`ICommandCodec.TryDecode` returns only the newest frame in a datagram.** Its own doc
   (`core/Teleop.Core/Contracts/ICommandCodec.cs`): "A codec carrying redundant copies may find
   several frames in one datagram; it returns the newest here and exposes the rest through its own
   surface, since recovering older frames is codec-specific and not part of this contract."

2. **`RobotEndpoint.Step` calls only `TryDecode`.** It has no knowledge of any codec-specific
   surface and applies exactly one `CommandFrame` per received datagram
   (`core/Teleop.Core/Pipeline/RobotEndpoint.cs`). So today the recovered copies are decoded into
   nothing and reach no consumer. Redundancy is inert before it even gets to the plant.

3. **Even if `RobotEndpoint` were taught to apply them, the plant discards them.**
   `RigidBodyPlant.Command` opens with:

   ```csharp
   if (command.CaptureTicks <= _lastAcceptedCaptureTicks)
   {
       return;
   }
   ```

   Every recovered copy is, by construction, older than the newest frame in the same datagram.
   Whichever order they are applied in, the plant ends in the same state.

4. **The plant is path-independent, so ordering cannot rescue it either.** `Command` is a pure
   assignment — position, rotation, both velocities and gripper are overwritten wholesale, with no
   accumulation. Applying k−2, then k−1, then k leaves exactly the state that applying k alone
   leaves. There is no integral, no rollback buffer, no re-simulation. This is the property that
   makes the GGPO analogy fail: rollback netcode needs old inputs *because its simulation is
   path-dependent and re-runs history*; this plant's history is unrecoverable and irrelevant.

5. **Redundancy does not change which datagrams survive.** It adds bytes, not packets.
   `EmulatedTransport.Send` draws loss from the Gilbert–Elliott state and the profile
   probabilities with **no reference to `payload.Length`** (verified: the draw at
   `Transport/EmulatedTransport.cs:247-263` is size-independent). So the set of delivered datagrams
   is bit-for-bit the same realization for a given seed, and the newest frame in each delivered
   datagram is the same frame it would have been under `raw`.

**Conclusion:** with a correctly-resized transport, an N-frame redundant codec produces a
**bit-identical plant trajectory, a bit-identical downlink reply stream, and therefore bit-identical
values for every metric this system emits**, on every network profile, at every N. Not "a small
effect". Zero.

### 2.2 This generalizes to every mechanism in my scope

Steps 3–5 above never mention *how* the old content was recovered. XOR parity, Reed–Solomon and
fountain codes all reconstruct **past** datagrams, and they do so *later* than repetition does.
They are therefore strictly worse than repetition here: same zero benefit, plus recovery latency,
plus implementation cost. Interleaving adds still more latency for the same zero.

The only mechanism in the broader family that escapes this argument is one that sends content the
receiver does **not** already have a newer substitute for — i.e. *future* intent rather than past
frames. That is precisely the `trajectory` codec, and it is candidate 2's, not mine. **The clean
statement of the boundary between us: for a latest-value-wins receiver, redundancy in the past
direction is worthless and redundancy in the future direction is the whole idea.** That is the most
useful thing my scope can tell candidate 2.

### 2.3 Where redundancy would actually pay in this system — and it is not the uplink

The uplink's consumer (`RigidBodyPlant`) is latest-value-wins. The **downlink's** consumer is not:
`OperatorEndpoint.ObserveRobotState` feeds each `RobotStateFrame` into
`IPredictor<Pose>.Observe(Stamped<Pose>)`, and the predictors are **history-based** — they
difference a pair of samples to estimate velocity. Recovering a lost past robot-state sample
genuinely densifies that history, which is real new information the receiver does not already
have.

Two caveats that keep even that from being an easy win, and which whoever picks it up must handle:

- `ConstantVelocityPredictor` deliberately restricts its rate estimate to the **newest pair**,
  specifically so that "reinserting one in the middle of the window must not retroactively change
  what the predictor is currently extrapolating" (its own type doc). A recovered sample only
  changes anything if it *becomes* the newest pair — which depends on whether the codec surfaces
  the recovered older sample before or after the newest one within the same datagram. That
  intra-datagram ordering would be a load-bearing design decision, and getting it wrong silently
  produces "no effect" that looks like a failed idea rather than a wiring bug.
- The downlink wire format is `RobotStateFrame` / `RobotStateFrameCodec`, which `Pipeline/CLAUDE.md`
  states is deliberately a plain class and not a `Contracts/` interface because "exactly one
  downlink shape exists". Adding redundancy there is a wire-format change (v2 → v3) and an
  architecture change. **That needs an ADR**, and it is out of my scope in two directions at once
  (wrong leg, and an architecture change).

I am recording this as a **handoff, not a recommendation**: "downlink state redundancy" is a
genuinely different candidate from mine, it is the one place the family is not structurally dead,
and nobody in this run owns it.

---

## 3. Which existing metric moves, in which direction, on which profiles

**Answer: none, in any direction, on any profile.** Given §2, that is a structural statement, not a
measurement claim. But the metric situation is independently bad enough to be worth writing down,
because it would block this candidate even if §2 had gone the other way.

### 3.1 The complete emitted set, verified

I verified the emitted-metric inventory rather than taking it on trust
(`grep -rn "\.Record(" --include="*.cs" core/`, excluding tests):

| Metric | Emitted by |
|---|---|
| `owd_uplink_ms`, `owd_downlink_ms` | `Pipeline/OperatorEndpoint.cs:227,232` |
| `correction_magnitude_mm`, `correction_magnitude_deg`, `time_to_convergence_ms`, `jerk_mm_s3` | the seven `Reconciliation/` implementations |
| `prediction_position_error_mm`, `prediction_orientation_error_deg` | `Teleop.Eval/Sweep/SweepCommand.cs:304,305` |
| `m2p_ms` | host-only, `Bridge/` — not reachable headlessly |

**Everything in `docs/metrics.md` §3 — loss rate, loss burst-length distribution, jitter (both
definitions), reordering rate, goodput — is defined and emitted by nothing.** Confirmed: no source
file outside tests contains any of those names.

### 3.2 The instrumentation gap, stated precisely

**This candidate's entire benefit is measured in residual loss and its entire cost is measured in
bandwidth, and neither has an instrument today.** Worse, the two instruments that §3 *does* define
are not quite the right ones:

- **`docs/metrics.md` §3 defines loss rate as "fraction of sent datagrams never received".** That
  is *datagram*-level. My mechanism, by construction, **cannot change datagram loss rate at all** —
  it changes how many `CommandFrame`s reach a consumer per delivered datagram. The quantity that
  would score it is *frame-level residual loss*: "fraction of emitted `CommandFrame`s never
  delivered to the plant". **That metric does not exist, and I may not create it** — `docs/metrics.md`
  is a closed list and adding to it is a human's call. So the honest position is: **the metric that
  would make this candidate scorable at all does not exist and cannot be added by an agent.**

- **Goodput is defined as "application-useful bytes/s, excluding redundancy and retransmission" —
  i.e. the definition already anticipates this candidate and already scores it as pure overhead.**
  It is the exactly-right cost instrument and it is emitted by nothing. To emit it you need
  (i) a byte counter at the `ITransport.Send` call site in `Pipeline/OperatorEndpoint.cs`, and
  (ii) a way for the codec to report how many of the bytes it just wrote were *useful* versus
  redundant. `ICommandCodec` has no such member. So goodput instrumentation implies a **contract
  change** (`ICommandCodec` gains something like a per-encode useful-bytes out-parameter, or a
  `Diagnostics` struct in the style the Core style guide already mandates), which is an
  architecture change and an ADR.

- **Uplink loss rate is not measurable at all in the current architecture, and this is a real
  finding.** The natural observer is `RobotEndpoint`, which already decodes `frame.Sequence` and
  could count gaps — but `RobotEndpoint` deliberately has **no `IMetricSink`**, because ADR 0002
  fixes that "clock-domain conversion and latency reporting happen exactly once, operator-side".
  Operator-side inference cannot substitute: a sequence that never returns was lost on the uplink
  *or* the downlink and the operator cannot tell which. Closing that would mean either amending
  ADR 0002 to let the robot emit metrics, or adding a robot-observed gap counter to
  `RobotStateFrame` (wire v3). **Either way it is an ADR, not a codec.**

**The specific, minimum answer to "what would have to be emitted, by which component":** a
frame-level residual-loss metric (new definition, human-owned) emitted by `RobotEndpoint` (which
requires ADR 0002 to be amended to give it a sink), reported together with goodput emitted by
`OperatorEndpoint` (which requires `ICommandCodec` to gain a useful-bytes surface). That is two
ADRs and a new metric definition before the first line of codec code is worth writing. **That is
the real price of this axis, and it is much higher than the codec itself.**

### 3.3 The OWD population trap, applied to this candidate

`owd_uplink_ms`/`owd_downlink_ms` are emitted only inside `RecordOneWayDelayMetrics`, reached only
when a `RobotStateFrame` comes back whose `Sequence` matches a live in-flight `LatencyTrace`
(`OperatorEndpoint.TryTakeInFlight`, 64-entry ring). **A lost packet emits nothing.** The OWD
sample population is conditioned on survival of *both* legs.

For my candidate this trap is worse than "read the percentiles carefully":

- As designed (recovered frames inert), the population is unchanged, so pooled OWD comparison
  against `raw` is safe — but only because the mechanism does nothing.
- **Any implementation that makes the mechanism do something breaks the metric.** If
  `RobotEndpoint` were extended to reply per *recovered* frame in order to make redundancy visible,
  each recovered frame's `robotRecvTicks` would be the arrival time of a **later** datagram. Its
  `owd_uplink_ms` would be inflated by (N−1) × the packet interval — 10–30 ms of pure artifact —
  and the redundant codec would appear to have made the network *worse* while also enlarging the
  sample population with a systematically-biased subgroup. That is a confident wrong answer of
  exactly the kind this platform is set up to avoid.

**How a sweep would have to be read:** never pool OWD percentiles across codecs. Compare only on
the intersection of sequences that produced a sample under *both* codecs with the same seed
(common random numbers make this well-defined: `EmulatedTransport` consumes exactly one draw at
send and two at drain per datagram regardless of profile values, per `Transport/CLAUDE.md`, so two
codecs at one seed see the same loss realization). And report the sample count per cell alongside
every percentile, so a changed population is visible rather than silently averaged.

### 3.4 Which profiles could distinguish it, if it worked

- **`lan` shows literally nothing.** Loss is 0/0 (ADR 0004). So do `50ms-5j`, every `jitter-<N>ms`,
  every `delay-<N>ms`, `loss-0pct`, and `synthetic-burst` (trace-driven delay only; the ADR 0004
  trace profiles carry no loss). That is the large majority of the catalog.
- **`loss-<N>pct` for N ≥ 0.25** is Bernoulli by ADR 0005's explicit design
  (`after-delivered == after-lost`, `ExpectedBurstLength ≈ 1`). Isolated single losses are the
  *easiest* case for redundancy: N = 2 covers 100% of them. This family would flatter the
  mechanism, and it is the least realistic of the loss conditions.
- **`150ms-20j-0.5loss`** is also effectively Bernoulli (0.5%/0.5%, `ExpectedBurstLength ≈ 1.005`).
- **`300ms-60j-2loss-bursty` is the only profile in the frozen set with real burst shape**
  (0.612%/70%, `ExpectedBurstLength ≈ 3.33`). It is therefore the only existing profile that can
  distinguish repetition depth from repetition-at-all — and there is exactly one of it.
- **Reintroducing burst shape on the loss axis needs a new ADR** (ADR 0005 says so in its own
  Consequences). So a proper depth-vs-burst-length study — the only study that would make this
  family interesting — is blocked on an ADR before it is blocked on anything else.

**Statistical power, which nobody has costed:** every experiment in `experiments/` runs
`trialSteps: 500` at `stepIntervalTicks: 100000` on a 10 MHz clock — 10 ms per step, 100 Hz, **5
seconds and 500 uplink datagrams per trial**, 5 seeds, so 2500 datagrams per cell. On
`300ms-60j-2loss-bursty` that is ~50 lost datagrams per cell in roughly **15 burst events**.
Fifteen events is not enough to resolve p95 of anything, let alone to distinguish N = 3 from N = 4.
Any real attempt at this axis needs longer trials or more seeds *before* it needs a codec.

---

## 4. What it costs to build here

### 4.1 What fits unchanged

More than I expected, and this part is genuinely cheap:

- `ICommandCodec` accommodates a redundant codec **as written**. `MaxEncodedBytes` is documented as
  an upper bound; `TryEncode`'s contract already says a failed encode "must not consume the delta
  baseline or a redundancy slot"; `TryDecode` already anticipates several frames in one datagram;
  `Reset` already names "no redundancy history". The interface was designed for this row.
- `ITransport` is message-oriented with a per-datagram `byteCount`, so **variable-length datagrams
  already work**. I checked `EmulatedTransport`: it allocates `_innerMaxPayloadBytes * maxInFlight`
  once and copies `length` bytes per datagram. `RobotEndpoint` sizes `_recvBuffer` from
  `uplinkTransport.MaxPayloadBytes`. Nothing assumes a fixed size *except* the one sweep line
  below.
- Invariant 8 (no hot-path allocation) is satisfiable: an N-slot ring of `CommandFrame` structs
  sized in the constructor.

### 4.2 The sweep constraint, priced precisely

The brief flagged this as the constraint most directly in my path. Having read
`core/Teleop.Eval/Sweep/SweepCommand.cs` and `ExperimentConfig.cs` read-only, **it is smaller than
advertised, and the fixed-size worry is not the expensive part:**

1. `ExperimentConfig` has no codec field (confirmed: `Predictors`, `Reconciler`/`Reconcilers`,
   `NetworkProfiles`, `Seeds`, `TrialSteps`, `StepIntervalTicks`, three reconciler knobs — no
   codec). Adding `Codecs: List<string>` plus validation against `Registries.Codecs` mirrors the
   existing `Predictors` handling almost line for line. **There is direct precedent**: the same
   gap existed for the reconciler and was closed by the Python `stacks` schema, so the pattern for
   adding a swept dimension is already established and does not need reinventing.
2. Three `new RawPoseCodec()` sites in `RunTrial` (lines 258 and 261 — two codec instances, plus
   line 250's sizing use) become `Registries.Codecs[codecName]()`. **Two separate instances is
   correct and must stay that way**: a stateful codec's encoder and decoder are distinct, and
   `ICommandCodec.Reset`'s doc requires both ends reset together.
3. `new LoopbackTransport(RawPoseCodec.EncodedSize, TransportCapacity)` (line 250) sizes the inner
   transport by a *fixed* constant. This becomes `codec.MaxEncodedBytes`. **The cost of "larger
   fixed size" and the cost of "variable length" are the same one-token change**, because the
   transport layer already carries per-datagram lengths (see 4.1). The brief's worry that these are
   different costs does not survive reading the transports — I am recording the disagreement rather
   than quietly agreeing.
4. **The registry is the awkward part.** `Registries.Codecs` is typed `Func<ICommandCodec>` —
   parameterless, unlike `Predictors` (`Func<PredictorConfig, ITimeAuthority, …>`) and
   `Reconcilers` (`Func<ReconcilerConfig, IMetricSink, ITimeAuthority, …>`). **So N cannot be swept
   through a config object.** Two options: register `redundant-2` / `redundant-3` / `redundant-4`
   as distinct hand-written names (cheap, in the spirit of the `Codecs` table's existing shape, and
   my recommendation), or widen the factory signature — which touches `Registries.cs`, named in the
   root `CLAUDE.md` as one of the four real cross-machine conflict surfaces. Do not widen the
   signature for this.
5. `EmulatedTransport` still is not registered in `Transports`, but this candidate does not need it
   to be: `SweepCommand.MakeTransport` wires it directly. Constraint 4 from the brief does not bind
   here.

**Estimate: 60–120 lines across two or three `Teleop.Eval` files plus tests — call it half a day of
human work.** That is not the blocker. The blocker is §3.2.

### 4.3 Does it need an ADR?

- The codec itself: **no.** New file in `Transport/`, hand-written registry line, unit test,
  benchmark row. Exactly the `/new-impl` shape.
- Making it have any effect at all: **yes.** Either `RobotEndpoint` learns a codec-specific
  recovered-frames surface (Pipeline learning a new capability — root `CLAUDE.md` calls that an
  architecture change), or the plant becomes path-dependent (which would change the Phase-4
  zero-mitigation baseline — expressly forbidden, `RigidBodyPlant`'s own doc explains that a
  smoothing plant "would suppress exactly the correction cost the baseline exists to measure").
- Measuring it: **yes, twice** — the new frame-level residual-loss metric definition, and either an
  ADR-0002 amendment or a `RobotStateFrame` v3 field. Plus an `ICommandCodec` surface for goodput.
- Studying depth against burst length properly: **yes** — a bursty loss family is a new ADR under
  ADR 0005's own rule.

### 4.4 One unit of work or several?

**Several, and they are ordered.** In dependency order: (1) new metric definition + robot-side
emission [human + ADR]; (2) goodput instrumentation + `ICommandCodec` surface [ADR]; (3) a bursty
loss profile family [ADR]; (4) longer trials for statistical power [config]; (5) the sweep codec
dimension [half a day]; (6) the codec itself [one unit]. **The codec — the only part that looks
like the candidate — is last and smallest.** Anyone who starts at (6) will produce a green test
suite, a registry entry, a benchmark row that is numerically identical to `raw`, and no knowledge.

---

## 5. The falsifier — the cheapest single experiment that could kill it

**The cheapest falsifier is not an experiment, and I have already run it: the code-reading argument
in §2.** For a latest-value-wins plant with a monotonic staleness filter, past-frame redundancy
cannot change the state trajectory. That is a proof, not a percentile, and it costs nothing.

For completeness, the cheapest *measured* falsifier, if the organizer wants one on record:

- **Build:** the smallest possible `NFrameRedundantCodec` whose newest-frame field layout is
  byte-identical to `RawPoseCodec`'s, with N−1 verbatim copies appended. Do **not** touch
  `RobotEndpoint`. Size the inner loopback by `codec.MaxEncodedBytes`.
- **Compare:** `raw` versus `redundant-3` versus `redundant-8`, holding predictor (`const-vel`),
  reconciler (`snap`), plant, cadence and seeds fixed — one axis varying, per the coupled-axis rule.
- **Profiles:** `lan` (must show nothing — the sanity control), `loss-5pct` (Bernoulli, the case
  most favourable to redundancy), `300ms-60j-2loss-bursty` (the only real burst profile).
- **Seeds:** the standard `[1,2,3,4,5]`, and report seed spread.
- **The assertion, and this is the point:** the comparison is not a percentile test, it is an
  **equality** test. The plant's pose trajectory and every emitted metric sample should be
  *bit-identical* between `raw` and `redundant-N` at matched seed. Bit-identical is the predicted
  outcome and it is unambiguous — no statistics, no power calculation, no sample-population
  argument.
- **What "not worth building" looks like:** bit-identical output. **What would rescue the
  candidate:** any non-identical output at all. If that happened, the first hypothesis should be a
  *bug* (transport sizing, RNG draw ordering, buffer reuse), not a win — see §7.

A one-line variant that is cheaper still and needs no codec: add a temporary assertion to a
throwaway harness that `RigidBodyPlant.Command` is idempotent under replay of any prefix of the
accepted command sequence. If that holds — and §2.1 step 4 says it does by inspection — the whole
family is dead without writing a codec at all.

---

## 6. Is it distinguishable by measurement from something already implemented?

**No — and not even on profiles with loss > 0.** This is a stronger negative than the brief
anticipated, so I want to be exact about it.

- `Registries.Codecs` contains exactly one entry, `raw`. So `raw` is the only comparison.
- On zero-loss profiles (`lan`, `50ms-5j`, all `jitter-<N>ms`, all `delay-<N>ms`, `loss-0pct`,
  `synthetic-burst`) the two are trivially identical: there is nothing to recover.
- On loss > 0 profiles they are **also** identical, for the structural reason in §2 — the recovered
  frames reach no consumer, and would be discarded if they did.
- The single instrument that *could* ever separate them is **goodput**, which is defined precisely
  as "excluding redundancy" — so the only measurement that can see this candidate would show it as
  N× pure overhead for zero benefit. And goodput is emitted by nothing.

That last sentence is the whole assessment in one line.

---

## 7. Bottom line

**Do not build it.** Past-frame redundancy on the uplink is structurally a no-op against a
latest-value-wins plant with a monotonic staleness filter, so it cannot move any emitted metric on
any profile; the metric that would score it (frame-level residual loss) does not exist and the
metric that would price it (goodput) is emitted by nothing and needs an `ICommandCodec` contract
change to emit.

Ranked within my own scope, **for the record and so the ranking is not re-derived**:

| Rank | Mechanism | Benefit here | Cost | Verdict |
|---|---|---|---|---|
| 1 | N-frame verbatim repetition | zero (§2) | N× payload; zero recovery latency; ~1 day to build | the only one worth building *if the receiver semantics ever change*; build nothing now |
| 2 | XOR parity over a window (RFC 5109 style) | zero | ~1/k payload; ≥1 packet-interval recovery latency; **cannot recover 2 losses**, so useless against bursts | no |
| 3 | Reed–Solomon / block FEC | zero | best bytes-per-depth; up to (n−1) packet-intervals recovery latency, fatal at 10 ms cadence; GF(256) hand-written under invariant 7 | no |
| 4 | Interleaving | zero | a latency-for-burst-tolerance modifier on 2/3; wrong currency | no |
| 5 | Fountain / RaptorQ | zero | designed for large blocks; worst fit and heaviest implementation | never |

**Build it after X, for the one live descendant:** *downlink* state redundancy (§2.3) is not
structurally dead, because the operator-side predictor is a history-based consumer. It should be
assessed as its own candidate, after (a) a frame-level residual-loss metric exists and (b) the
`RobotStateFrame` v3 ADR is written. It is not mine and nobody in this run owns it.

**Handoffs.**
- *To candidate 2 (`trajectory`):* the sharp result from my scope is that redundancy in the **past**
  direction is worthless against this plant while redundancy in the **future** direction is the
  entire idea. `RigidBodyPlant` coasts on `CommandFrame.LinearVelocity`/`AngularVelocity` through a
  gap, and `SweepCommand` passes `Vector3.Zero` for both — so today the plant **holds** through a
  gap rather than coasting, and the intent channel the trajectory codec depends on has never been
  exercised. That is candidate 2's first problem and it is in `Teleop.Eval`, not Core.
- *To candidate 3 (adaptive depth):* an adaptive redundancy controller would need, as input, a
  receiver-observed estimate of frame-level residual loss and burst length. Neither is emitted, and
  §3.2 shows the *uplink* one is not observable operator-side at all under ADR 0002. **An adaptive
  redundancy controller is blocked on the same missing instrument as the fixed one, plus a feedback
  channel.** I make no assessment of the estimator itself.

**Recommended `Transport/CLAUDE.md` change, which I did not make** (shared file; organizer's call):
move the `redundant` row from the codec table into a new "Tried and rejected" section with a
one-paragraph verdict — "structurally a no-op against a latest-value-wins plant; see
`docs/research-log/2026-09-09-fec-redundancy-feasibility.md`" — so it is not proposed again. Every
"Tried and rejected" section in this repo currently reads *(none yet)*; this is a candidate for the
first entry, and it is a rejection on argument rather than on measurement, which the table should
say plainly.

---

## 8. How this assessment could flatter itself

Required section, and I have five real ones.

1. **A negative verdict is cheap to defend, and I found it early.** I identified the plant's
   staleness filter within the first twenty minutes and everything after that was elaboration of a
   conclusion I had already reached. I did not spend equal effort hunting for the configuration in
   which the candidate wins. A survey agent on a clock is structurally biased toward "do not build",
   because "do not build" requires no benchmark. Treat §2 as strong and §1's ranking as weaker.
2. **I assessed "the candidate as it fits the pipeline unchanged", and called that "the
   candidate".** The no-op proof depends on `RobotEndpoint` calling only `TryDecode` and on
   `RigidBodyPlant.Command`'s staleness filter. Someone building this in earnest would plausibly
   change one of those — and if they did, my argument evaporates and the real question (does a
   path-dependent robot-side consumer help?) is one I did not answer. I chose the narrow reading
   because widening it is an architecture change requiring an ADR, but that is a scoping choice, not
   a fact about the idea.
3. **I ran no experiment, so every number in §3.4 and §4.2 is arithmetic, not measurement.** The
   burst-event count (~15 per cell), the bandwidth multipliers, the recovery-latency figures — all
   derived from reading constants, none observed. `TicksPerSecond = 10_000_000` and
   `stepIntervalTicks = 100000` give 100 Hz; if any future experiment changes the cadence, the
   power argument changes with it.
4. **Prior art did the explanatory work and none of it is evidence here.** Opus/LBRR and GGPO are
   the two analogies I leaned on to explain *why* past-frame redundancy pays elsewhere and not here,
   and both are qualitative engineering descriptions measured (where measured at all) on 20 ms
   audio frames or 60 Hz button bitmasks — nothing like 73-byte pose frames over a Gilbert–Elliott
   burst channel at 300 ms base delay. An analogy that explains a conclusion I reached
   independently is a comfortable thing to have, and comfortable is a warning sign.
5. **The measurement harness understates loss-mitigation benefit, and I should not be allowed to
   use that as cover in either direction.** `SweepCommand.SyntheticOperatorMotion` is an analytic
   sinusoid (`x = sin(t)·0.5`, `z = 1 + cos(0.7t)·0.3`, identity rotation), which is C^∞ and
   trivially extrapolable: a 30 ms gap costs the predictor far less here than it would on real hand
   motion with abrupt reversals, so **any** loss-mitigation result measured in this harness is
   biased *downward*, probably substantially. Additionally, `maxObservationGapTicks` is set to
   `TicksPerSecond` (1 second) against a 10 ms cadence, so `const-vel`'s gap-collapse safety —
   exactly the failure mode redundancy exists to prevent — requires 100 consecutive losses to trip
   and is never exercised by any existing profile. Both facts mean the harness is a weak instrument
   for this whole axis. **But neither rescues this candidate:** an understated multiple of zero is
   still zero, and I would be fooling myself if I softened the verdict on the grounds that the
   measurement is unfair. The right conclusion is that the harness bias is a finding about the
   *platform* — it will understate candidates 2, 3 and 4 too — and it should be recorded against
   the run, not against this candidate.

---

## Verification

Gates run once, read-only, before any analysis, to establish that the tree was green on arrival and
that nothing in this run changed it:

| Gate | Result |
|---|---|
| `dotnet test core/Teleop.Core.Tests` | `Passed! - Failed: 0, Passed: 558, Skipped: 0` |
| `dotnet run --project core/Teleop.Eval -- verify` | `verify: PASS -- basic-session.tlog replays byte-identical across two independent passes` |
| `dotnet run --project core/Teleop.Eval -- audit` | `audit: PASS -- no invariant violations found` |

No source file was modified. `git status` should show exactly one untracked file: this log.

## Left undone / for a human

- **The `Transport/CLAUDE.md` "Tried and rejected" row** proposed in §7. I did not edit the axis
  doc — it is a shared file and the brief forbids it. The organizer should carry it.
- **Two ADRs and one metric definition** are the real prerequisites of this whole axis, not of this
  candidate specifically (§3.2): a frame-level residual-loss metric; robot-side metric emission or
  a `RobotStateFrame` v3 gap counter; and an `ICommandCodec` useful-bytes surface so goodput can be
  emitted. All three are human calls. **Until at least the first exists, no Transport-codec
  candidate in this run can be scored on the thing it actually changes.**
- **A bursty loss profile family** (`loss-<N>pct-burst<M>` or similar) is needed before redundancy
  depth can be studied against burst length at all, and ADR 0005 explicitly requires a new ADR for
  it. Blocked on a human.
- **Trial length / seed count.** 500 steps × 5 seeds yields ~15 burst events per cell on the only
  bursty profile. Any loss-focused result from this platform needs that raised first. This affects
  every candidate in this run, not just mine.
- **Downlink state redundancy** (§2.3) is unowned, is not structurally dead, and is the only live
  descendant of this family. Somebody should pick it up as its own candidate.
- **Not attempted, deliberately:** anything touching `unity/`, `robot/`, real hardware, `results/`,
  or any shared file. No `just move-arm`, `clocksync-check` or `deploy-robothost` was invoked or
  needed.
