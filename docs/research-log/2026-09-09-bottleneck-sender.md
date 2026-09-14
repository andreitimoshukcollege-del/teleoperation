# Bottleneck queue impairment + sender-side send-rate behaviour

**Agent:** Claude Opus 5 (candidate 3 of 3, Transport axis run, 2026-09-09)
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-ab391cc8fe77034c9`
**Branch:** `worktree-agent-ab391cc8fe77034c9`
**HEAD at start:** `76b2a149270bbcd2e7d0e4050b1e96e123d73940` (confirmed via `git rev-parse HEAD`)

## The question I was given

Two questions, strictly in order:

1. **Is a bottleneck/queue impairment expressible under the existing `INetworkImpairment`
   contract at all?** Feasibility assessment written *before* any implementation. A well-argued
   "this needs an ADR" is a successful outcome.
2. **Given a queue exists, is there anything a Core-side sender can actually do?** This is where
   the run's framing ("nothing in Core can reduce one-way delay") gets tested: self-induced
   queuing delay is in principle the one OWD component a sender can reduce.

The previous survey (`2026-09-09-network-transport-survey-decisions.md`) *assumed* the negative
and recorded it as a suspicion. My job is to test it, not inherit it.

## Log discipline

Written continuously. Sections appear in the order they were produced, so an interruption leaves
a truthful partial record rather than a tidy retrospective.

## Seed self-check

| Check | Expected | Observed | Verdict |
|---|---|---|---|
| `git rev-parse HEAD` | `76b2a14` | `76b2a149270bbcd2e7d0e4050b1e96e123d73940` | PASS |
| Worktree is my own, not the main checkout | not `~/Projects/teleoperation` | `.claude/worktrees/agent-ab391cc8fe77034c9` | PASS |
| Branch is not `main` | — | `worktree-agent-ab391cc8fe77034c9` | PASS |
| Working tree clean at start | clean | clean (`git status --short` empty) | PASS |
| Gates green *before* I touch anything | 3/3 | `dotnet test` 644+36+28+3 passed / 0 failed; `verify: PASS`; `audit: PASS` | PASS |

## Prior work consulted (candidates and vocabulary only — never evidence about this system)

- **Nichols & Jacobson, "Controlling Queue Delay", ACM Queue 10(5), 2012**
  (<https://queue.acm.org/detail.cfm?id=2209336>). Proposes CoDel; the part I actually used is its
  framing of the *standing queue*: a persistent backlog at a bottleneck that adds delay to every
  packet and buys nothing, distinguished from the transient "good" queue that absorbs bursts.
  Claims CoDel holds queue delay near a 5 ms target with no tuning. **Measured under:** elastic TCP
  flows, ns-2 and Linux, bottlenecks in the Mbit/s range, RTTs 30–500 ms. It is an *AQM* — it lives
  in the bottleneck, which in this architecture is the network, not Core. I use the standing-queue
  concept only, not the algorithm.
- **"Sender-side Buffers and the Case for Multimedia Adaptation", ACM Queue, 2012**
  (<https://queue.acm.org/detail.cfm?id=2381998>). The directly relevant one: it argues the
  *sender's own* buffer is a first-class latency source and the fix is to write the minimum the
  link can carry rather than everything the application produces. **Measured under:** TCP-based
  video over residential broadband, i.e. an elastic encoder — the opposite sender class from ours.
  The transferable idea, and the only thing I took, is "offering more than the link can carry
  converts throughput you do not get into delay you do pay."
- **RFC 8083, "Multimedia Congestion Control: Circuit Breakers for Unicast RTP Sessions"**
  (<https://www.rfc-editor.org/rfc/rfc8083.html>). Read to check what the standards world says an
  *inelastic* sender should do under congestion. Its answer is deliberately minimal: not a rate
  controller, only conditions under which a sender must **stop**. Our sender is inelastic in
  exactly its sense (fixed cadence, fixed 73-byte payload), so the literature's own answer for this
  sender class is admission/cessation rather than rate shaping. That is what pointed me at an
  *admission* mechanism instead of a rate-adaptation one.

Negative search result, recorded so it is not repeated: I found **no** prior work on send-side
admission control for a fixed-rate *pose/command* stream as distinct from media. Everything in the
RTC congestion literature assumes an elastic encoder whose bitrate is the actuator. That absence
matters — the design choice below (which packet to sacrifice when the link cannot carry them all)
is not inherited from anywhere and has to be argued and measured here.

## Feasibility assessment (Question 1) — written before any implementation

### 1a. Can a bottleneck/queue be an `INetworkImpairment`? **No, not as the contract stands.**

I re-verified the brief's three facts against the source rather than trusting them.
`Contracts/INetworkImpairment.cs` declares exactly `ApplyOnSend(ref DatagramFate)` and
`ApplyOnDeliver(ref DatagramFate)` — confirmed, **neither takes a time parameter**.
`Types/DatagramFate.cs` carries exactly `long DelayTicks` and `bool Dropped`, and its
"Deliberately absent: send tick, sequence number, payload length" is verbatim.

The minimal FIFO bottleneck is one recurrence:

```
departure[i]  = max(arrival[i], departure[i-1]) + serialization(length[i])
queueDelay[i] = departure[i] - arrival[i]
```

Three inputs. `departure[i-1]` is impairment-private state and is fine. `arrival[i]` and
`length[i]` are **both** on the deliberately-absent list, and neither is recoverable: Core forbids
clock reads (invariant 2), so an impairment cannot ask the time, and the payload never passes
through `INetworkImpairment` at all — `EmulatedTransport.Send` receives the `ReadOnlySpan<byte>`
and hands the impairments only a `DatagramFate`.

This is not an accident of which fields happened to be chosen. **Every impairment shipped today is
memoryless in time**: fixed delay is a constant; jitter is one draw; Gilbert-Elliott is a draw
conditioned on a Markov bit (state, but not *elapsed time*); reorder is one draw; trace-delay
advances a cursor **per datagram**, not per tick. A queue is the first impairment whose output
depends on how much wall time passed since the previous datagram. The contract has no channel for
that, by construction.

**Finding against ADR 0013.** Its Context names "duplication, **bandwidth throttling**, corruption,
correlated delay" as the kinds of impairment the new shape unlocks, and its Consequences say
"adding an impairment kind stops being an ADR-scale change and becomes a `/new-impl`-scale one."
For bandwidth throttling that is **not true as written**: a rate is bytes per unit *time* and the
fate struct carries neither bytes nor time. Duplication is blocked for a different reason (a fate
can drop a datagram, but cannot emit two). The ADR is correct for delay/jitter/loss-shaped axes and
over-claims for the rest. Amending its text is a human's call; I have not touched `docs/adr/`.

### 1b. Could it be made expressible? Yes, and it is pre-blessed — and I still rejected it.

`DatagramFate`'s own doc says adding a field "later is source-compatible for every impairment that
ignores it", and my brief pre-blesses that. Adding `long StageTicks` (send tick at the send stage,
inner arrival tick at the deliver stage) plus `int PayloadLength`, populated by `EmulatedTransport`,
would be strictly additive and provably baseline-neutral: no shipped impairment reads them, so
every existing realization stays bit-identical, and no RNG draw moves, so `ImpairmentStreamsTests`'
frozen literals are untouched. I convinced myself it would work. I did not do it, for three reasons
that survive the pre-blessing:

1. **Order-independence would genuinely break.** The two `DatagramFate` rules make composition
   commutative *because each impairment's output depends only on its own parameters*. A queue's
   output depends on the arrival **stream**, which at the deliver stage is whatever the send stage
   let through — so a loss impairment sitting before it changes the backlog. That is legal (the
   docs permit an order-sensitive impairment that documents itself as such) but it makes array
   order part of the configuration, which every profile name in ADRs 0004/0005/0006 currently
   assumes it is not. That cost is paid by every future reader, not by me.
2. **Tail drop would land at the wrong stage.** A finite buffer must drop on overflow. At the
   deliver stage `EmulatedTransport` honours `Dropped` but its own comment says it "does NOT
   unsend: the wrapped transport already carried the bytes" — precisely the error ADR 0013 forbids
   ("a dropped packet that first occupied an in-flight slot would model a link that transmitted
   bytes it was supposed to have lost"). At the send stage there is no arrival tick to key the
   queue on without a second field and a second, differently shaped code path.
3. **There is a route that needs no contract change at all.**

### 1c. The route that works today: a bottleneck is an `ITransport`, not an impairment

`Contracts/ITransport.cs` already hands over everything a queue model needs, legitimately rather
than by widening anything:

| Queue model needs | `INetworkImpairment` | `ITransport` |
|---|---|---|
| current time at enqueue | absent | `Send(payload, **nowTicks**)` |
| payload length | absent | `payload.Length` |
| current time at dequeue | absent | `TryReceive(**nowTicks**, ...)` |
| somewhere to report the departure instant | `DelayTicks`, additive only | `out arrivalTicks` |
| a contractual way to refuse on overflow | `Dropped`, at the wrong stage | `Send` returns **false** |

The last row decides it. `ITransport.Send`'s own doc says it "returns false when the datagram will
not be delivered — emulated loss, **or a full send queue**." A finite bottleneck buffer refusing a
datagram is *literally that documented case*, not an approximation of it. And because the refusal
happens inside `Send` before anything is forwarded, no byte the model says was dropped is ever
carried — the exact property ADR 0013 protects.

So: **`Transport/BottleneckTransport.cs`, a decorator over any `ITransport`**, composed underneath
the emulator:

```
OperatorEndpoint -> EmulatedTransport(delay/jitter/loss) -> BottleneckTransport -> LoopbackTransport
```

which is also the physically correct order — the access link serializes and queues, *then* the rest
of the path adds propagation delay, jitter and loss. `EmulatedTransport`'s doc already promises
those composition semantics ("impairment is always additive: the wrapped transport's own transit
delay and its own losses stand"), so that class needs no change and I am not making one.

**Consequences of the transport route, stated so a reader can disagree with me:**

- The bottleneck is **not addressable by a profile name.** `NetworkProfileCatalog` maps names to
  impairment *sets*, and this is not an impairment. A named profile carrying a bottleneck would
  need the catalog to return a transport factory as well — a real change, needing an ADR, because
  profile names are frozen benchmark identity. I therefore added **no** profile name and touched
  **neither** copy of `NetworkProfileCatalog`. Per my brief a new impairment kind and a new named
  profile are separate questions; this is a third thing, a new transport.
- **No `Registries.Transports` entry.** That table is `Func<int, int, ITransport>` —
  `(maxPayloadBytes, capacity)`. A decorator taking `(inner, capacityBytesPerSecond, bufferBytes,
  ticksPerSecond)` does not fit, which is the identical objection `Registry/CLAUDE.md` records for
  `EmulatedTransport`, and `AuditCommand.RegistryCompletenessAxes` excludes `Transports` for that
  reason. Forcing it in would produce a stringly-typed parameter bag. **My registry line is: none,
  and this paragraph is the "say why" the brief asked for.**
- It cannot be swept today; `SweepCommand` hardcodes its transports and belongs to another
  researcher this run. Priced under "Left undone".

### 1d. Is `EmulatedTransport`'s back-pressure already a crude queue? **Yes, and it never engages**

`DrainInner` stops pulling when no in-flight slot is free, and `LoopbackTransport.Send` returns
false on a full ring. That is a two-stage finite buffer with tail drop, so the mechanism exists.
Whether it engages:

- `SweepCommand` uses `TransportCapacity = 64` for **both** the loopback ring and `maxInFlight`;
  `stepIntervalTicks` is 10 ms in every experiment YAML in the repo.
- The uplink is drained every step (`RobotEndpoint.Step` loops `TryReceive` up to
  `MaxDatagramsPerStep = 64`), and `DrainInner` moves everything the loopback holds into the heap
  on the first `TryReceive` of a step.
- Heap occupancy is therefore about `oneWayDelay / stepInterval`. Slots exhaust at
  `64 x 10 ms = 640 ms` of one-way delay.
- The worst frozen profile, `300ms-60j-2loss-bursty`, tops out at **360 ms**.

So the crude queue **engages in no recorded result**, and my baseline is unaffected by it. I verify
that with a test rather than leaving it as arithmetic. Two things follow. It is not a usable
research bottleneck — its capacity is one datagram per drain per step, a *slot* limit with no
bytes-per-second parameter to sweep. And, more important, **if anything ever pushes one-way delay
past 640 ms the emulator starts refusing sends for a reason no manifest records**, so my tests must
be able to say which of the two mechanisms did the dropping.

### Question 1 answer, one line

**Yes — expressible today as an `ITransport` decorator with zero contract change, and *not*
expressible as an `INetworkImpairment`, which carries neither time nor length and would break
order-independence if widened to. No ADR is needed for the fixture; an ADR *is* needed to give it a
profile name or to sweep it, and I did neither.**

## Feasibility assessment (Question 2) — what a Core-side sender could do, written before any code

### 2a. Enumerating the actuators, honestly

The brief lists three and invites more. Against this sender (`SweepCommand` submits one command per
fixed `stepIntervalTicks`; `RawPoseCodec.EncodedSize` is a fixed 73 bytes, so offered load is
`73 B / 10 ms = 7300 B/s = 58.4 kbit/s`, constant):

| Actuator | Verdict |
|---|---|
| Sub-step pacing (spread a burst across the interval) | **Meaningless.** A step is atomic and Core has no threads. There is also no burst — exactly one datagram per step. |
| Adaptive payload size | **Blocked, and not mine.** `RawPoseCodec` is fixed-size; `SweepCommand` sizes the transport from `RawPoseCodec.EncodedSize`, so a variable-size codec breaks the wiring. Codec work is explicitly reserved by the user. |
| Prioritisation among streams | **No such thing here.** There is one uplink stream and one downlink stream, on separate transports. Nothing to prioritise against. |
| **Adaptive cadence / admission** — skip sends when backlog builds | **The only live one.** Assessed below. |
| *(mine, not in the brief)* **Head-drop instead of tail-drop in the sender's own queue** | Real in principle — for a command stream the oldest queued pose is the worthless one — but this sender holds no queue. It hands each datagram straight to `Send`. Recorded as a variant a human might want, not pursued. |

### 2b. Where a send policy would live — and the surprise

The brief expects this to be the blocker: `OperatorEndpoint.SubmitCommand` is called once per step
by the host and "do not send this one" has no home; there is no `Contracts/` interface for a send
policy. I verified `SubmitCommand` and it does exactly what the brief says — encode, then
`_uplinkTransport.Send(...)` with the return value **discarded**.

But that last detail is the answer, not the problem. **"Do not send this one" already has a home:
it is `ITransport.Send` returning `false`.** The contract's own words are "returns false when the
datagram will not be delivered — emulated loss, or a full send queue... callers must not retry on
it". A decorator that decides not to forward a datagram and returns false is *indistinguishable, at
the contract level, from a link that lost it*, and `OperatorEndpoint` already handles that case
correctly by ignoring the return and letting the trace age out of the ring exactly as it does for
network loss.

So a sender-side admission controller is **a new file in `Transport/` with no new interface, no
`Pipeline/` change and no ADR.** That directly contradicts the previous run's determination
("adaptive send rate / pacing — effectively out of scope for Core... `ITransport.Send` has no
pacing concept, and Core may not own a loop"). That reasoning is right about *cadence* — Core may
not own the loop — and wrong about *admission*, which is a per-call decision the transport contract
already models. This is the single most useful correction I have to make to the prior record, and
it is available only because the fixture in 1c gives it something to act on.

### 2c. The regime, and the arithmetic that makes it interesting

Take a bottleneck of capacity `C` bytes/s with a buffer of `B` bytes, fed by our 7300 B/s sender.
For `C >= 7300` the queue is empty and there is nothing to do. For `C < 7300` the queue saturates
and two things become true at once:

- **Delivered command rate is pinned at `C / 73` per second and is completely independent of the
  offered rate.** Sending faster delivers nothing extra; the excess is tail-dropped.
- **Every delivered command waits the full buffer, `B / C` seconds.** That is the standing queue.

Worked example I will measure: `C = 3650 B/s` (half the offered load, so 20 ms per datagram) and
`B = 730 B` (10 datagrams). Greedy sender: delivers 50 commands/s, each having waited
`730 / 3650 = 200 ms`. A sender admitting only 50 commands/s: delivers **the same 50 commands/s**,
each having waited **20 ms**. Same rate. 180 ms less delay. The 180 ms is pure self-inflicted
delay, and it is a component of `owd_uplink_ms` — a metric in `docs/metrics.md` §2.

If that holds, the run's framing is wrong in a narrow but real way: *the network owns propagation
delay, but the sender owns the queuing delay it creates itself.*

### 2d. The part I expect to kill the obvious controller

The only signal a transport-layer decorator can see is `Send` returning false. It cannot see
`owd_uplink_ms` — that is computed in `OperatorEndpoint` from the returned `RobotStateFrame` and is
one round trip stale. So the natural controller is loss-based, AIMD on the send gap.

**I expect loss-based control to recover the rate and *not* the delay,** for the textbook
bufferbloat reason: once the buffer is full, a sender transmitting at exactly `C` experiences *no
refusals at all* (a slot frees precisely as each new datagram arrives), so the controller sees a
clean link and stops backing off — while the queue stays pinned full. Loss-based control's
equilibrium is "rate = C, buffer = full", which is the same delay as greedy. Draining the queue
requires deliberately undershooting `C`, which a loss signal never asks for.

If that is right, the honest verdict is layered rather than yes/no, and I want it on the record in
that shape.

## Hypothesis (written before any code)

Metric and direction, named first: **`owd_uplink_ms`** (`docs/metrics.md` §2, `t_recv − t_send` on
the operator→robot command), **direction: decrease**, reported at p50/p95/p99, never as a mean.
The counterweight reported alongside it every time is **delivered command count** (the §3 "loss
rate" quantity — *defined* in `docs/metrics.md` and *emitted by nothing*, so I compute it in my own
harness and flag the dependency).

**H1 (the fixture works, and the regime is real).** With a bottleneck of capacity `C` and buffer
`B` in front of the link, at `C < 7300 B/s` the steady-state uplink delay converges to `B / C`
independent of `C`'s distance below the offered load, and delivered command rate converges to
`C / 73` per second independent of the offered rate.
*Falsifier:* delay does not exceed one serialization time, **or** delivered rate varies with
offered rate while `C < 7300`. Either kills the fixture, and with it everything below.

**H2 (the sender owns its self-inflicted delay).** A sender that admits commands at a rate at or
just below `C / 73` achieves **the same** delivered command count as the greedy sender (within a
few percent) at **materially lower** `owd_uplink_ms` — I predict roughly `B/C` down to one
serialization time, i.e. 200 ms → 20 ms in the worked example.
*Falsifier:* delivered count drops materially (say >10%), **or** p95 `owd_uplink_ms` falls by less
than the buffer-drain time. Either means the delay is not self-inflicted and the run's framing
survives intact.

**H3 (the controller that could actually run cannot get it).** A closed-loop controller using the
only signal available to a transport decorator — `Send` returning false — converges to the right
*rate* but leaves `owd_uplink_ms` statistically indistinguishable from greedy, because a full
buffer plus rate `= C` produces no refusals to learn from.
*Falsifier:* the loss-based controller's p50 `owd_uplink_ms` lands materially below greedy's. That
would be the genuinely surprising outcome and I would then have to explain the mechanism before
believing it.

**Falsifier for the whole candidate:** if `C >= 7300 B/s` (an uncongested link, which is every
condition this repo currently benchmarks) the admission controller must be *harmless*. If it drops
commands or adds delay when there is no queue, the mechanism is a net negative for this platform
regardless of what it does in the congested regime, and I will say so.

### What I am explicitly NOT claiming

Not that this reduces propagation delay — it cannot, and nothing here tries. Not that the frozen
profiles are affected — none of them has a bottleneck, and I have added no profile. Not that this
is measurable by `sweep` — it is not, today.

## What I built

Three new files under `core/Teleop.Core/Transport/`, each with a freshly generated `.cs.meta` guid
(checked against every guid in `core/` and `unity/` for collisions — none):

| File | What it is |
|---|---|
| `BottleneckTransport.cs` | The **fixture**. `ITransport` decorator: finite-rate link (`capacityBytesPerSecond`) with a finite FIFO queue (`bufferDatagrams`), FIFO service, tail drop at `Send`. Deterministic, no RNG, no seed. |
| `RateLimitedTransport.cs` | The **oracle mechanism**. `ITransport` decorator: token bucket admission at a configured rate, credit denominated in ticks so refill is exact. Open-loop — it is told the link rate. |
| `BacklogBackoffTransport.cs` | The **realizable mechanism**. `ITransport` decorator: AIMD on the admission gap, driven by the only signal a decorator can see, `Send` returning false. |

Four new test files under `core/Teleop.Core.Tests/Transport/` (51 tests):
`BottleneckTransportTests.cs`, `RateLimitedTransportTests.cs`, `BacklogBackoffTransportTests.cs`,
and `SenderAgainstBottleneckExperimentTests.cs` (the measurement harness).

**Registry entries: none, deliberately.** `Registries.Transports` is `Func<int, int, ITransport>` —
`(maxPayloadBytes, capacity)` — and all three of these are decorators taking an inner transport plus
their own parameters. That is the identical objection `Registry/CLAUDE.md` records against
registering `EmulatedTransport`, and `AuditCommand.RegistryCompletenessAxes` excludes `Transports`
for exactly that reason, so `audit` neither requires nor flags an entry. Forcing them in would
produce a stringly-typed parameter bag.

**Files in my exclusive scope that I did NOT touch:** `Types/DatagramFate.cs`,
`Transport/EmulatedTransport.cs`, `Transport/NetworkProfileCatalog.cs`,
`Teleop.Eval/Sweep/NetworkProfileCatalog.cs`. Question 1 concluded no change to any of them is
forced, so none was made. No experiment YAML either — see "Where the numbers live" below.

## Where the numbers live, and why they are not citable

`SweepCommand` hardcodes `LoopbackTransport` + `EmulatedTransport` and cannot select a transport by
name, so a bottleneck cannot appear in a sweep and no `manifest.json` can describe one. It is also
owned by another researcher this run. Rather than leave the candidate unmeasured I put the harness
in `SenderAgainstBottleneckExperimentTests.cs`, deterministic and seeded, asserting the structural
claims. **These numbers are reproducible from that file but are not citable in this repo's sense —
there is no `results/` directory and no manifest SHA behind them, and I did not create one.**
Reproduce with:

```
cd core && dotnet test Teleop.Core.Tests \
  --filter "FullyQualifiedName~SenderAgainstBottleneck" --logger "console;verbosity=detailed"
```

Common to every table: 73-byte payload every 10 ms (7300 B/s = 58.4 kbit/s offered, matching
`RawPoseCodec.EncodedSize` and the sweep's cadence), 3000 steps, first 1000 discarded as the
queue-fill transient, percentiles by nearest rank. `accept/s` is `Send` returning true; `deliv/s`
is datagrams actually received; `refused` is what the *sender* withheld, never counting what the
link dropped. Stack: `sender-policy -> EmulatedTransport(profile) -> BottleneckTransport -> Loopback`.

## Results

### H1 — CONFIRMED. Below capacity the delivered rate is pinned and the delay is the whole buffer

Greedy sender, buffer 10 datagrams, profile `lan`, seed 1:

| C / offered | accept/s | deliv/s | p50 ms | p95 ms | p99 ms |
|---|---|---|---|---|---|
| 2.00 | 100.0 | 100.0 | 7.0 | 7.9 | 8.0 |
| 1.00 | 100.0 | 99.9 | 12.0 | 12.9 | 13.0 |
| 0.75 | 75.0 | 74.5 | 132.0 | 136.0 | 136.2 |
| 0.50 | 50.0 | 49.5 | 202.0 | 202.9 | 203.0 |
| 0.25 | 25.0 | 24.5 | 402.0 | 402.9 | 403.0 |

Delivered rate is `C / 73` per second at every point below capacity, against a constant 100/s
offered — the sender gets nothing at all for the extra 50–75 datagrams per second it sends. The
delay is `bufferDatagrams x serializationTicks` plus the profile's own 2 ms, exactly.
`BottleneckTransport.InnerRefusedCount` is asserted zero in every row, so every drop measured is
this model's tail drop and not something the wrapped transport quietly refused.

Neither falsifier fired.

### The shape result: recoverable delay is the network's buffer depth, and the greedy sender's delay is unbounded in it

C = 0.5x offered, profile `lan`, seed 1:

| buffer (datagrams) | greedy p50 ms | rate-limited p50 ms |
|---|---|---|
| 2 | 42.0 | 22.0 |
| 5 | 102.0 | 22.0 |
| 10 | 202.0 | 22.0 |
| 20 | 402.0 | 22.0 |
| 40 | 802.0 | 22.0 |

This is the finding I would keep if I could keep only one. **The greedy sender's uplink delay is
linear in a buffer it does not own and cannot see; the admission-controlled sender's is flat at one
serialization time.** It is not a percentage improvement, it is a different dependence: the deeper
the network's buffer, the more the greedy sender loses, without bound. That is the bufferbloat
result, reproduced on this platform.

### H2 — CONFIRMED, and the cost side is zero, not small

C = 0.5x offered, buffer 10, profile `lan`, seed 1:

| sender | accept/s | deliv/s | sender-refused | max delivery gap ms | p50 ms | p95 ms | p99 ms |
|---|---|---|---|---|---|---|---|
| greedy (baseline) | 50.0 | 49.5 | 0 | 21.9 | 202.0 | 202.9 | 203.0 |
| rate-limited @1.00C | 50.0 | 50.0 | 1500 | 21.9 | 22.0 | 22.9 | 23.0 |
| rate-limited @0.95C, 1-datagram bucket | 33.3 | 33.3 | 2000 | 31.9 | 22.0 | 22.9 | 23.0 |
| rate-limited @0.95C, 2-datagram bucket | 47.5 | 47.4 | 1574 | 31.7 | 22.0 | 22.9 | 23.0 |

**p50 uplink one-way delay 202.0 ms → 22.0 ms, at an identical accepted command rate (50.0/s both)
and a slightly higher delivered rate.** 180 ms of `owd_uplink_ms` removed by a Core-side sender.

Across five seeds on `150ms-20j-0.5loss` (jitter and loss present, so the realization varies):
greedy p50 349.6–351.0 ms, rate-limited p50 169.7–171.1 ms; deliv/s 49.1–49.2 vs 49.1–49.4. The
180 ms is recovered on every seed and the delivered rate is never worse.

On an uncongested link (C = 2x offered, `50ms-5j`) the falsifier for the whole candidate does not
fire: greedy, rate-limited and backlog-backoff produce **byte-identical** delivered counts and
percentiles (99.7/s, p50 55.0 ms, p95 59.5 ms) with zero sender refusals. The mechanism is inert
where there is no queue.

### H3 — CONFIRMED. The controller that could actually run recovers the rate and not the delay

C = 0.5x offered, buffer 10, profile `lan`, seed 1:

| sender | deliv/s | p50 ms | delay recovered |
|---|---|---|---|
| greedy | 49.5 | 202.0 | — |
| backlog-backoff (loss-signalled) | 49.5 | 192.3 | **9.7 ms of the 180.0 ms available** |
| rate-limited @1.00C (oracle) | 50.0 | 22.0 | 180.0 ms |

The closed-loop controller finds the *rate* precisely (49.5/s, identical to greedy) and recovers
5% of the delay. The mechanism is the textbook one and the trace confirms it: once the buffer is
full and the admitted rate equals `C`, a slot frees exactly as each new datagram arrives, so the
link stops refusing, so the controller stops backing off — and the queue stays pinned full. A loss
signal never asks a sender to *undershoot*, and undershooting is the only thing that drains a
standing queue.

On the lossy profile it is worse than that: at seed 2 the controller's delivered rate drops to
47.2/s (from greedy's 49.2) and its max delivery gap doubles to 107.7 ms, because **loss and
queue-full are the same bit** through `ITransport.Send` and it backs off for losses that had
nothing to do with congestion. So the realizable controller pays a real cost on a lossy link and
buys almost nothing on a congested one.

### The metric that would be needed, and does not exist

`docs/metrics.md` §3's **loss rate** ("fraction of sent datagrams never received") is the exact
quantity that is this mechanism's cost side, and it is *defined and emitted by nothing*. I computed
it in-harness as raw accepted/delivered counts. **That dependency is a finding, not a workaround:
without §3 instrumentation, a sweep of this mechanism would show the delay win and hide the
command-rate cost entirely** — the same shape of invisible-cost failure `docs/metrics.md` warns
about for playout delay. I did not add a metric; another researcher owns that file this run.

I also computed **max inter-delivery gap** in-harness (not a defined metric, not emitted) precisely
because delivered-per-second alone cannot detect a mechanism that wins by clumping deliveries. It
is identical between greedy and rate-limited on `lan` (21.9 ms both), which is what closes that
loophole. It is *not* identical for the loss-signalled controller on a lossy link (107.7 vs
55.1 ms), which is how that mechanism's real cost became visible.

## How this result could flatter itself

Written before I was comfortable with it, because a 202 ms → 22 ms improvement is exactly the shape
of a broken measurement. Everything below I went looking for; the first two are real and I quantify
them, the rest I checked and cleared.

1. **The delivered-count comparison is biased *in my mechanism's favour*, slightly.** Greedy shows
   49.5 deliv/s against rate-limited's 50.0, which reads as the mechanism delivering *more*. It does
   not. The `accept/s` column is 50.0 for both. The difference is end-of-trial truncation: greedy
   finishes the run with a full 10-datagram queue that never drains before step 3000, so ~10
   datagrams over the 20 s window (0.5/s, 1%) are accepted and never counted. That is why I added
   the `accept/s` column rather than reporting delivered alone. **The honest statement is that the
   two deliver the same number of commands, not that admission control delivers more.**
2. **The `@0.95C` row at 33.3/s is an artifact of my own bucket, not a property of the mechanism**,
   and I nearly reported it as a cost. With a one-datagram bucket the cap and the per-datagram cost
   are the same quantity, so fractional credit cannot carry over and the achievable admitted rate
   quantizes to `offered / k`. A two-datagram bucket delivers 47.5/s at the same 22 ms. Both rows
   are in the table, and the test asserts the difference, so nobody re-derives it. **Consequence
   for anyone building on this: aiming just below capacity requires a bucket of at least two
   datagrams.**
3. **The percentiles compare different sample sets, unavoidably.** `owd_uplink_ms` exists only for
   delivered datagrams, so greedy's 1500-ish samples and the limiter's 1500-ish samples are
   different datagrams. This is the population-versus-algorithm trap my brief names. Three things
   make it survivable here and none of them is "it is probably fine": the sample *counts* are equal
   to within 1% (from `accept/s`), the delivered *spacing* is identical (max delivery gap 21.9 ms
   both), and the effect size is 9x, not a few percent. I would not make this comparison at a 5%
   effect.
4. **A bottleneck below 7300 B/s is a modelling choice I made, and it is contestable.** Nothing in
   this system models cross traffic, so a link that cannot carry 58 kbit/s only makes sense as the
   *residual* share of a contended link. I documented that reading in the constructor. If a reader
   rejects it, the whole regime is unreachable and the candidate is a negative — which is why it is
   stated in the type's own doc and not buried here.
5. **Deep-buffer rows exceed the in-flight ring window that censors the real pipeline.** At buffer
   40 the greedy sender's uplink delay is 802 ms. `OperatorEndpoint.InsertInFlight` overwrites an
   occupied slot without checking, and at `InFlightCapacity = 64` with 10 ms steps that is a 640 ms
   window against the *round* trip — so in the real pipeline those trips would be censored, and
   censored delay-correlated, exactly as the survey's finding describes. **My harness is not exposed
   to it** (it measures at the transport, with `maxInFlight` 1024 and a 4096-slot loopback), which
   is why the numbers are clean — but it means the buffer-40 row could not be reproduced end-to-end
   today without hitting that defect first. I did not fix it; that is a human's call about
   comparability with existing results. Anyone taking this to the full pipeline must deal with it.
6. **Checked and cleared:** `InnerRefusedCount == 0` in every H1 row, so no drop is attributable to
   the loopback filling. The transient discard is 1000 steps against a worst-case fill time of
   40 datagrams x 20 ms = 800 ms, so the measured window is 25x past the transient. Percentiles are
   nearest-rank on the raw sample list, not on a smoothed series. The uncongested control (H2b)
   returns byte-identical numbers for all three senders, which is the strongest single check that
   the harness is not systematically favouring the decorated stacks.
7. **The oracle is an oracle.** `rate-limited @1.00C` is *told* the link rate. It is the ceiling on
   what any sender-side controller could achieve, not a deployable algorithm, and the gap between
   it and `backlog-backoff` is the honest measure of the mechanism's practical value today: 180 ms
   available, 9.7 ms reachable.

## Verification

All four gates run at the end of the work, from
`/home/andrei/Projects/teleoperation/.claude/worktrees/agent-ab391cc8fe77034c9`.

**`dotnet test`** — my 51 new tests pass 100% of the time, in isolation and in the full suite.
The full suite is **intermittently red on this box, from a pre-existing flake I did not cause and
did not touch.** Five consecutive full runs:

```
run 1: Core 695/695 PASS   RobotArm 28 PASS  Eval 3 PASS  RobotHost 36 PASS
run 2: Core 695/695 PASS   RobotArm 28 PASS  Eval 3 PASS  RobotHost 36 PASS
run 3: Core 695/695 PASS   RobotArm 28 PASS  Eval 3 PASS  RobotHost 36 PASS
run 4: Core 695/695 PASS   RobotArm 28 PASS  Eval 3 PASS  RobotHost 36 PASS
run 5: Core 693/695 FAIL   RobotArm 28 PASS  Eval 3 PASS  RobotHost 36 PASS
```

The failures are always `AllocationAssert.Zero` assertions in tests I did not write or modify —
`DoubleExponentialPredictorTests.Predict_Allocates_Zero_Bytes`,
`SnapReconcilerTests.Reconcile_WithNothingPending_Allocates_Zero_Bytes`,
`ImmediatePlayoutTests.Diagnostics_ReadRepeatedly_Allocates_Zero_Bytes` — with **fractional
bytes-per-call** (0.032, 0.108, 0.165), a different subset each run. `AllocationAssert`'s own doc
describes this exact signature and history ("failures of 0.165 to 0.678 bytes/call — fractions of a
byte, which no real allocation can be, since the smallest object is twenty-four — and a *different*
test failed on each run"), and commit `28f82a2` was a previous attempt at it.

I established it is not mine with two controls rather than asserting it:

```
# my test classes excluded from execution entirely -- still flaky
$ dotnet test Teleop.Core.Tests --filter "FullyQualifiedName!~Bottleneck&FullyQualifiedName!~RateLimitedTransport&FullyQualifiedName!~BacklogBackoff"
  Failed: 2 / 644     Failed: 2 / 644     Failed: 2 / 644     Failed: 1 / 644

# allocation tests alone -- never flaky
$ dotnet test Teleop.Core.Tests --filter "FullyQualifiedName~Allocates_Zero_Bytes"
  Passed: 77/77       Passed: 77/77       Passed: 77/77
```

So it is contention between xUnit's parallel collections and .NET's background tiering JIT on this
6-core box, present with none of my code running, and absent when the allocation tests do not
compete. **I did not weaken, skip, or retry any assertion to make this pass.** It is a real,
pre-existing gate reliability problem and it belongs to a human — see "Left undone".

**`dotnet run --project Teleop.Eval -- verify`** — exit 0:

```
verify: PASS -- .../testdata/golden/basic-session.tlog replays byte-identical across two
independent passes, and matches the original file exactly.
```

**`dotnet run --project Teleop.Eval -- audit`** — exit 0:

```
audit: PASS -- no invariant violations found in .../core/Teleop.Core or
.../build/Teleop.Core/bin/Debug/netstandard2.1/Teleop.Core.dll.
```

Note `audit`'s impairment-reachability check is unaffected: I added no `INetworkImpairment`
implementation, so nothing new needs constructing in `NetworkProfileCatalog.cs`. And its
registry-completeness check excludes `Transports` by design, so it neither demands nor flags an
entry for my three decorators.

**`just bridge-check`** (`cd unity/BridgeCheck && dotnet build`) — 0 errors. Run even though I
changed no Core signature, because my brief required it and because a new public Core type can in
principle collide. The single warning (`CS8632` in `JetRoverOperatorBridge.cs`) is pre-existing in a
file I did not touch.

**Baseline before any of my edits**, for comparison: `dotnet test` 644 + 36 + 28 + 3 passed / 0
failed (a lucky green — see above), `verify: PASS`, `audit: PASS`.

## Answers to the two questions, restated plainly

**Question 1 — is a bottleneck/queue impairment expressible under the existing contract?**
**Yes, but not as an `INetworkImpairment`.** That contract carries neither time nor payload length
and both are structurally required by any queue; widening `DatagramFate` is pre-blessed and would
work, but would make array order part of the configuration and would put tail drop at a stage where
`EmulatedTransport` cannot un-send. The route that needs **zero** contract change is an `ITransport`
decorator, because `ITransport` already hands over `nowTicks`, `payload.Length`, an `arrivalTicks`
out-parameter, and a contractual `false` for "full send queue". No ADR is needed for the fixture. An
ADR *is* needed to give a bottleneck a profile name or to make it sweepable, and I did neither.

Side finding: **ADR 0013 over-claims.** It names "bandwidth throttling" as an example of what its
new shape makes a `/new-impl`-scale change; it is not, for the reason above. Duplication is blocked
too (a fate can drop a datagram but cannot emit two). The ADR is right for delay/jitter/loss-shaped
axes.

Second side finding: `EmulatedTransport`'s existing back-pressure **is** a crude queue but is
**unreachable** — it engages only past `maxInFlight x stepInterval = 640 ms` of one-way delay, and
the worst frozen profile reaches 360 ms. My baseline is therefore uncontaminated by it, which I
verified rather than assumed.

Third side finding, small but sharp: **`ITransport.Send`'s `false` is overloaded.** It means both
"I lost this" and "I have no room right now", and a queueing decorator cannot tell them apart. The
contract resolves it ("callers must not retry"), so `BottleneckTransport` discards and counts, but
this is also precisely why `BacklogBackoffTransport` backs off for losses that are not congestion.

**Question 2 — given a queue, is there anything a Core-side sender can do?**
**Yes, and more than the framing allows — but not with the signal it can currently reach.**

- The mechanism is real and large: at a saturated bottleneck, 180 ms of `owd_uplink_ms` is *self-
  inflicted* and a Core-side sender can remove all of it at **zero** cost in delivered command rate,
  because the commands it withholds were being tail-dropped anyway. **The run's framing — "nothing
  in Core can reduce one-way delay" — is wrong in this regime.** Propagation delay belongs to the
  network; queuing delay the sender caused belongs to the sender.
- The mechanism has a home that costs nothing architecturally: `ITransport.Send` returning false
  already *is* "do not send this one". No new `Contracts/` interface, no `Pipeline/` change, no ADR.
  This corrects the previous run's "adaptive send rate is effectively out of scope for Core", which
  was right about cadence and wrong about admission.
- **But the controller that can actually be built recovers 5% of it.** A transport decorator sees
  one bit per datagram, and loss-based control's equilibrium is a *full* buffer. Getting the rest
  needs a delay signal — `owd_uplink_ms`, which lives in `OperatorEndpoint` one round trip late.
  That controller is a new `Contracts/` interface plus `Pipeline/` wiring: ADR-scale, and I have
  not written one.

So the honest verdict is layered: **the physics says yes and is worth a lot; the plumbing says the
cheap version of it is nearly worthless, and the valuable version is an ADR away.**

## Left undone / for a human

### Blocked on a human, in priority order

1. **`docs/metrics.md` §3 loss rate is this mechanism's entire cost side and is emitted by nothing.**
   Another researcher owns that file this run, so I did not touch it. Until it exists, any sweep of
   admission control shows a large `owd_uplink_ms` win with its command-rate cost invisible. Note
   the requirement precisely: the analyst needs *offered*, *sender-withheld* and *delivered* counts
   as three separate quantities, because folding "the sender chose not to send it" into "the link
   lost it" is what lets an admission mechanism hide its bill. `RateLimitedTransport.RefusedCount`
   and `BacklogBackoffTransport.RefusedCount` already expose the middle one and deliberately
   exclude the link's own refusals.
2. **The sweep cannot select a transport, so none of this is citable.** `SweepCommand` hardcodes
   `LoopbackTransport` + `EmulatedTransport`; `ExperimentConfig` has no transport field;
   `Registries.Transports` has a factory shape (`(maxPayloadBytes, capacity)`) that no decorator
   fits. Priced, not done — that file belongs to another researcher this run. Making a bottleneck
   sweepable is the prerequisite for a `results/` directory with a manifest.
3. **The pre-existing `AllocationAssert` flake makes `dotnet test` unreliable on a 6-core box**
   (~20–50% of full-suite runs red, always fractional bytes-per-call, always in tests unrelated to
   the change under test). Evidence in `## Verification`. This is a gate that cries wolf, and a gate
   that cries wolf is on its way to being ignored. Likely direction, for whoever picks it up:
   xUnit's default parallelism lets allocation-measuring tests run concurrently with each other and
   with background tiering compilation; `[Collection]`-serializing the allocation tests would
   address the contention without weakening a single assertion. **I did not do it** — it touches
   tests across four axes owned by nobody in this run.
4. **`ADR 0013`'s Consequences over-claim.** "Adding an impairment kind stops being an ADR-scale
   change" is false for bandwidth throttling and for duplication, for structural reasons given
   above. Correcting an accepted ADR's text is a human's job; I may not edit `docs/adr/`.
5. **The in-flight ring defect gates any end-to-end reproduction of the deep-buffer rows.** Uplink
   delays past ~640 ms of round trip are censored by `OperatorEndpoint.InsertInFlight` overwriting
   occupied slots. My transport-level harness is not exposed to it; a pipeline-level one would be,
   and would be censored delay-correlated, which is the worst case. Not mine to fix.

### The ADR that would be needed next, in substance (NOT written as a file — an unratified ADR in the tree is worse than none)

**Title:** A send policy that can see one-way delay.

**Context.** Admission control at the transport seam works and is cheap (this log), but the only
signal at that seam is `Send` returning false, and loss-based control's equilibrium is a full
buffer — measured here at 9.7 ms recovered out of 180 ms available. The signal that would work,
`owd_uplink_ms`, already exists: `OperatorEndpoint.TryReceiveState` computes it from the returned
`RobotStateFrame` via `ClockSync`. It is one round trip stale, which is fine — a standing queue is
a persistent condition, not a transient, so a delay-based controller of the LEDBAT/Copa shape
(target a small queue above the *minimum* observed delay, undershoot to drain) does not need a
fresh signal, it needs a *baseline* and a *trend*.

**Decision the ADR must make.** Where the policy lives and how it is fed. Two shapes, and I would
argue for the second:

- (a) A new `Contracts/ISendPolicy` consulted by `OperatorEndpoint.SubmitCommand` before encoding,
  with `OperatorEndpoint` feeding it each completed `LatencyTrace`. Clean, but it puts an algorithm
  inside `Pipeline/`, which that folder's doc says it must not hold.
- (b) Keep the mechanism where it already works — an `ITransport` decorator — and give it the
  signal, by having `OperatorEndpoint` push completed uplink OWD samples to an object it shares
  with the decorator. This preserves "Pipeline holds no algorithm", keeps the sweep's transport
  stack composable, and means the three classes in this log need no rewrite. The cost is a new
  Core-declared interface for the sample push, and a decision about where it is constructed.

**Also to decide in the same ADR**, because they are the same wiring question: whether
`ExperimentConfig`/`SweepCommand` gain transport selection (item 2 above), and whether
`Registries.Transports`' factory shape changes or whether decorators stay unregistered as they are
now.

**What the ADR should NOT do:** add a bottleneck to any named profile. A new *impairment kind*, a
new *named profile*, and a new *transport* are three separate questions, and this is the third.
Adding a bottleneck to the frozen suite changes benchmark identity and needs its own ADR against
`docs/adr/0004`.

### Rows a human should add to `Transport/CLAUDE.md` (out of my scope; I did not edit it)

To **Implemented**:

| `bottleneck` | `BottleneckTransport.cs` | finite-rate link + finite FIFO queue, tail drop at `Send`; the fixture that makes send-rate control measurable |
| `rate-limited` | `RateLimitedTransport.cs` | sender-side token-bucket admission; open-loop oracle |
| `backlog-backoff` | `BacklogBackoffTransport.cs` | sender-side AIMD admission on the `Send`-false signal |

All three deliberately unregistered, same reasoning as `EmulatedTransport` (that section already
states it).

To **Tried and rejected** — this axis's section currently reads *(none yet)*:

> **Loss-signalled sender-side congestion control (`backlog-backoff`).** Rejected **on
> measurement**, not on argument. It converges on the correct send rate but recovers only 9.7 ms of
> the 180 ms of self-inflicted queuing delay available at a saturated bottleneck, because a full
> buffer plus a matched rate produces no refusals to learn from — the classic bufferbloat
> equilibrium. On a lossy profile it is actively harmful: loss and queue-full are the same bit
> through `ITransport.Send`, so it backs off for losses that are not congestion, costing ~4% of
> delivered commands and doubling the worst inter-delivery gap. The code is kept as the measured
> negative and as the honest ceiling of what the transport seam's signal supports. See
> `docs/research-log/2026-09-09-bottleneck-sender.md`.

The `Network profiles` section should also gain a sentence that **no profile has a bottleneck**, and
that adding one is an ADR against 0004 rather than a `/new-impl`.

### What I would do next, in order

1. Instrument §3 loss rate (blocked on the metrics owner). Without it nothing here is sweepable
   *honestly*, even once it is sweepable *mechanically*.
2. Write the ADR above and build the delay-signalled controller. The oracle says 180 ms is on the
   table and the realizable controller gets 9.7; that gap is the largest single number this
   candidate produced and it is the whole remaining question.
3. Only then consider a named bottleneck profile — and note that if step 2 fails, the right
   conclusion is that this whole regime is a *fixture* for studying other axes (a jitter buffer or
   predictor under a deep bottleneck is an interesting condition nobody can currently create) rather
   than a mitigation in its own right.

### Not done, on purpose

No `experiments/exp-006-bottleneck-sender.yaml`: an experiment YAML that the sweep cannot execute
would be a file claiming a capability the tool does not have. No `results/` directory: no manifest
SHA is reachable from an uncommitted worktree, so anything written there would be uncitable by
construction. No edits to `Types/DatagramFate.cs`, `Transport/EmulatedTransport.cs` or either
`NetworkProfileCatalog.cs` — Question 1 concluded none is forced, and the pre-blessing to widen
`DatagramFate` is not a reason to spend it. No `docs/adr/`, no `docs/metrics.md`, no axis
`CLAUDE.md`, no `unity/TeleopVR/`, no `robot/`, no hardware, no commits.
