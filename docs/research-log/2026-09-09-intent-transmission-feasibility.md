# 2026-09-09 — `trajectory` (intent transmission) feasibility assessment

**Agent:** feasibility/survey run on the Transport axis, one of four parallel researchers.
**Worktree:** `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-abed484aae1a81134`
**Branch:** `worktree-agent-abed484aae1a81134`
**HEAD at start:** `06c0ab5b6f570efa028002897c856c566041663b`

**This is an assessment, not an implementation.** No `.cs` file, test, `.meta`, registry entry,
experiment YAML, sweep, or `results/` output was produced, and none is claimed. The only file
this run creates or modifies is this log.

Candidate: the `trajectory` row in `core/Teleop.Core/Transport/CLAUDE.md`'s codec table —
"~200 ms of *intended future motion* per frame", which that file calls "the interesting one".

Out of scope by assignment (each owned by another researcher this run): frame repetition /
parity / FEC (candidate 1), adapting the horizon to a link estimate (candidate 3), multipath /
racing / pacing (candidate 4).

## Seed self-check

| Check | Result |
|---|---|
| Worktree | `/home/andrei/Projects/teleoperation/.claude/worktrees/agent-abed484aae1a81134` |
| Branch | `worktree-agent-abed484aae1a81134` |
| `git rev-parse HEAD` | `06c0ab5b6f570efa028002897c856c566041663b` — **matches** the required `06c0ab5` |
| `dotnet test` | **PASS** — 558 `Teleop.Core.Tests` + 36 `Teleop.RobotHost.Tests` + 28 `Teleop.RobotArm.Tests` + 3 `Teleop.Eval.Tests`, 0 failed |
| `Teleop.Eval -- verify` | **PASS** — golden log replays byte-identical across two passes |
| `Teleop.Eval -- audit` | **PASS** — no invariant violations |
| Tree changed by this run | only this log file |

Run once, at the start, to confirm the tree was green before I touched it. Nothing in this run
changes any of the three.

## Bottom line (stated up front; the argument follows)

**Do not build `trajectory` as a codec.** Under `ICommandCodec` as written, a trajectory codec
is *provably indistinguishable from `raw` at the decoder's output*, because `TryDecode` returns
one `CommandFrame` and `CommandFrame` has exactly one order of intent (`LinearVelocity` /
`AngularVelocity`) — which `raw` already transmits in full. Everything a plan carries beyond
first order has nowhere to be written. This is the same shape as the `ekf` rejection
(`docs/research-log/2026-09-09-ekf-feasibility.md`): the distinguishing output is unobservable
by every existing consumer.

The honest unit of work underneath this candidate is not a codec at all. It is, in order:
(1) make the harness emit real velocity intent — today it is identically `Vector3.Zero`;
(2) add an instrument for *command-tracking* error, which does not exist and which no existing
metric substitutes for; and only then (3) an ADR for a plan-carrying wire shape and a
plan-evaluating consumer. Steps 1 and 2 are cheap, are not this candidate, and would settle the
first-order question — "does transmitting intent help at all?" — without writing a codec.

---

## 1. Does a plan have a consumer today?

Asked and answered first, per the brief. **Partly — at exactly first order, and by accident of
plant policy. Beyond first order, no, and the gap is an ADR.**

### 1.1 The end-to-end trace

Uplink path, read in full, `06c0ab5`:

1. `Pipeline/OperatorEndpoint.SubmitCommand(pose, linearVelocity, angularVelocity, gripper,
   nowTicks)` builds one `CommandFrame` and calls `_commandCodec.TryEncode(frame, _sendBuffer,
   out bytesWritten)`, then `_uplinkTransport.Send(...)`. The send buffer is
   `new byte[commandCodec.MaxEncodedBytes]`, allocated once in the constructor.
2. `Transport/EmulatedTransport` draws loss/jitter/reorder **per datagram, independent of
   payload length** (`Send` takes a `ReadOnlySpan<byte>` and the Gilbert-Elliott draw never
   looks at `payload.Length`). There is no bandwidth, serialization-delay, or MTU-fragmentation
   model anywhere in Core's transports.
3. `Pipeline/RobotEndpoint.Step(nowTicks)` drains the uplink in a `while` loop. For each
   datagram: `_commandCodec.TryDecode(span, out CommandFrame frame)` → `_plant.Command(frame)`.
   Then `_plant.Step(nowTicks)` **unconditionally**, then one state reply per received datagram.
4. `Plant/RigidBodyPlant.Command` rejects any frame whose `CaptureTicks <= _lastAcceptedCaptureTicks`
   **entirely**; otherwise it snaps `_position`/`_rotation` to the commanded pose and stores
   `_linearVelocity`/`_angularVelocity`/`_gripper`.
5. `RigidBodyPlant.Step` dead-reckons: `_position += _linearVelocity * dt`, rotation via
   `MotionMath.IntegrateWorld`. **Gap policy is coast-indefinitely on the last commanded
   velocity** — documented in both `Plant/CLAUDE.md` requirement 1 and the type doc, explicitly
   because "intent is what survives a lost packet".

### 1.2 The consumer that exists

Step 5 *is* a plan follower. `RigidBodyPlant` holds a degree-1, unbounded-horizon plan
(`pose₀ + v·dt`) and executes it through arbitrarily long gaps with no timeout and no ramp.
`CommandFrame`'s velocity fields are the plan; `Step` is the follower. So the claim "nothing can
consume a plan" is **false at first order** — the consumer has been there since `RigidBodyPlant`
was written, and its doc says so in as many words.

### 1.3 The consumer that does not exist, and why the contract blocks it

`ICommandCodec.TryDecode(ReadOnlySpan<byte> source, out CommandFrame frame)` has two properties
that together close the door:

- **It returns exactly one `CommandFrame`.** `CommandFrame`'s fields are `Sequence`,
  `AckSequence`, `CaptureTicks`, `Pose`, `LinearVelocity`, `AngularVelocity`, `Gripper`. There
  is no acceleration field, no jerk field, no knot array, no horizon, no coefficient block. A
  spline, a cubic, or a 20-knot setpoint sequence has **nowhere to be written on the way out**.
- **It takes no time parameter.** It is a pure function of bytes. It therefore *cannot* evaluate
  a plan at "now" even if it held one, because it does not know what "now" is. (Nor could it
  legally read a clock — invariant 2.)

Consequence, and it is the whole finding: a `TrajectorySplineCodec` under this contract can
encode whatever it likes on the wire, but the only thing it can hand the pipeline is
`(pose, v, ω, gripper)` at one instant. That is bit-for-bit the information content `raw`
already delivers. **A trajectory codec and `raw` are the same decoder, by construction.**

The `redundant` codec's escape hatch does not help. `ICommandCodec.TryDecode`'s own doc
anticipates multi-frame payloads — "a codec carrying redundant copies may find several frames in
one datagram; it returns the newest here and exposes the rest through its own surface". That
works for redundancy because redundant frames are **past** frames: real `CommandFrame`s with
real, already-elapsed `CaptureTicks`, which `RigidBodyPlant`'s staleness rule handles correctly
by ignoring them. It does not work for a forward plan, because a forward plan's knots have
`CaptureTicks` **in the future**.

### 1.4 A concrete, load-bearing failure mode if anyone tries the shortcut

Suppose someone "solves" 1.3 by having a trajectory codec return the plan's *last* knot, or by
having `RobotEndpoint` call `_plant.Command` once per knot. `RigidBodyPlant.Command` accepts any
frame strictly newer than its baseline, so a knot stamped 200 ms in the future would:

1. **teleport the plant** immediately to the operator's *intended future* pose (a 200 ms
   position jump, not a smoothing), and
2. set `_lastAcceptedCaptureTicks` 200 ms ahead, so **every genuine command for the next 200 ms
   is silently rejected as stale**.

The plant would be permanently running 200 ms ahead of the operator and deaf for one horizon at
a time. Steady-state, at the harness's 100 Hz cadence, it would accept roughly one command in
twenty. Nothing in the pipeline would report an error; `owd_uplink_ms` would keep flowing
normally. This is not a hypothetical style objection — it is what the smallest-looking
implementation actually does, and it is invisible to every metric the project emits.

### 1.5 The smallest honest change, and whether it is an ADR

Four candidates, in increasing cost:

| # | Change | Blocks? | ADR? |
|---|---|---|---|
| A | Nothing in Core. Make the harness send real `LinearVelocity`/`AngularVelocity` and measure `raw`-with-intent vs `raw`-with-zeros | `core/Teleop.Eval/` is human-owned; changing what the existing path sends alters the `none`/`snap` baseline | **No** — but it is a baseline change, so a human must own it (see §5.2) |
| B | Widen `CommandFrame` with acceleration fields | Changes `RawPoseCodec.EncodedSize` (73 → 97), i.e. the wire format *and* the baseline codec | **Yes** — wire formats are explicitly ADR-binding |
| C | New `Contracts/` interface for a plan (`TryDecodePlan` → a plan value) + a plan-evaluating `IRobotPlant` + `RobotEndpoint` learns to wire it | Root `CLAUDE.md`: "New question entirely → new interface in `Contracts/`, new folder, `Pipeline/` learns to wire it. This is an architecture change: write an ADR in `docs/adr/` first." | **Yes**, unambiguously |
| D | Give the plan to a *plant* implementation that decodes it itself | Layering violation — the plant would have to know the wire format, and `RobotEndpoint` owns the codec | n/a, rejected |

**The smallest honest change that gives a plan a consumer is C, and C is an ADR.** A is the
smallest change that produces *information*, but A does not give a plan a consumer — it exercises
the first-order consumer that already exists.

One judgement call, recorded as an assumption: I treat B as strictly worse than C even though it
is less code, because B changes the baseline codec's byte layout and therefore makes every
`results/` run recorded against wire v1 non-comparable, while C leaves `raw` untouched and adds a
second, parallel path. If a human disagrees, the argument to overturn is exactly that
comparability tradeoff.

### 1.6 The consumer disappears entirely on real hardware

`RigidBodyPlant`'s coast is the *only* reason a first-order plan has a consumer.
`docs/adr/0007-jetrover-plant-and-robot-host.md` states that `JetRoverPlant`'s gap policy is
**hold**, not coast — deliberately, because the arm's bus servos already hold with no command and
a coasting real arm is a strain risk. So on the machine this project actually targets,
`CommandFrame.LinearVelocity` is dropped on the floor and **there is no plan consumer at any
order**. Anything measured against `RigidBodyPlant` about intent transmission is a statement
about the Core simulation's gap policy, not about the robot.

### 1.7 Answer

- **First order (velocity intent): yes, a consumer exists** — `RigidBodyPlant`'s coast — but only
  in simulation, and it is currently fed zeros (§3.1).
- **Second order and above, and any time-parameterized plan: no consumer, and the contract makes
  one impossible without a new `Contracts/` type.** That is an **ADR**.
- **Same shape as the `ekf` rejection.** `docs/research-log/2026-09-09-ekf-feasibility.md` rejected
  `ekf` as a unit of work because "its one distinguishing output is provably unobservable by every
  existing consumer". `trajectory` is the same failure, one level worse: for `ekf` the
  unobservable thing was a `PredictorDiagnostics` field that a future consumer could read; here
  the return **type** of `TryDecode` has no field to put it in at all.

---

## 2. The mechanism, its sub-variants, and what prior work claims

### 2.1 One paragraph

Instead of sending "the operator's hand is here, now", send "the operator's hand is here now and
is going *there* over the next H milliseconds". The receiver holds the plan and evaluates it at
its own step time, so a datagram lost at time *t* costs nothing as long as some earlier
datagram's plan still covers *t*. This does not reduce one-way delay — OWD belongs to the network
and no wire format touches it — it reduces the number and length of intervals during which the
receiver has no valid setpoint, and therefore the amplitude of the receiver's own extrapolation
error. Its cost is bandwidth (a plan is O(H·rate) times a point, or O(order) times a point as
coefficients) and, more importantly, *commitment*: a receiver executing a 200 ms plan is
confidently wrong for up to 200 ms whenever the operator changes their mind inside the horizon.

### 2.2 Sub-variants, in increasing cost and increasing exposure to the commitment problem

| Variant | Wire content | Consumer needed | Fits today? |
|---|---|---|---|
| **V0 — first-order intent** | pose + `v`, `ω` (already in `CommandFrame`) | plant coast policy | **yes** — this is literally `raw` |
| **V1 — intent shaping** | pose + `αv`, `αω`, α a swept knob | plant coast policy | **yes**, `ICommandCodec` unchanged, no ADR (§4.4) |
| **V2 — second order** | + acceleration | plant must integrate accel | no — no `CommandFrame` field; ADR |
| **V3 — polynomial / spline coefficients** | cubic or quintic over H | plan evaluator | no; ADR |
| **V4 — explicit knot sequence** | N time-stamped setpoints over H | plan buffer + evaluator | no; ADR |
| **V5 — overlapping V4 segments** | as V4, consecutive datagrams overlapping by H−Δ | as V4 | no; ADR. **A redundancy scheme** (§6.3) |

### 2.3 Prior work

All accessed 2026-09-09. These generate candidates and settle nothing here.

**IEEE 1278.1 DIS dead-reckoning models** —
<https://github.com/open-dis/dis-tutorial/wiki/Dead-Reckoning>.
*Proposes:* nine DRM codes indexed by (fixed/rotating orientation, position-rate vs
velocity-rate, world vs body frame). Senders transmit an Entity State PDU only when true state
diverges from the receiver's dead-reckoned estimate beyond a threshold, with a heartbeat floor.
*Claims:* large reductions in PDU rate at bounded error. *Conditions:* military vehicle and
missile motion; heartbeat floors quoted at one PDU per five seconds — update intervals **two to
three orders of magnitude longer** than this project's 10 ms cadence, on ballistic motion with
smooth, physically constrained accelerations. This is the canonical form of the mechanism, but
its operating point is not ours: at 100 Hz with a 200 ms horizon, DIS's central tradeoff
(send-rate reduction) is not even the goal.

**Dead Reckoning: Latency Hiding for Networked Games** (Aronson, 1997) —
<https://www.gamedeveloper.com/programming/dead-reckoning-latency-hiding-for-networked-games>.
*Proposes:* the three-tier ladder — static, pose+velocity, pose+velocity+acceleration. *Claims:*
conceptual only; the article gives **no comparative measurement** between the three orders, and
notes only that large thresholds cause visible jerkiness without quantifying either.
*Conditions:* quotes 150–200 ms end-to-end latency as typical for the networks of the day.
Useful for vocabulary, worthless as evidence.

**Adaptive / multi-level-threshold DIS dead reckoning** — search-result summaries around
<https://www.researchgate.net/publication/237344987_Adaptive_dead_reckoning_algorithms_for_Distributed_Interactive_Simulation>
and
<https://www.researchgate.net/publication/328418593_Performance_of_Dead_Reckoning_Algorithms_Across_Technology_Eras>.
*The claim that matters most here:* "the second order algorithm using acceleration and velocity
is far more efficient than the first order algorithm that used only velocity, while third order
algorithms that use rate of change of acceleration offer only marginal improvement".
*Conditions:* DIS entity motion again — vehicles, area-of-interest-scoped, threshold-triggered. I
could not retrieve the underlying measurement setups from the abstracts, so treat "second order
beats first order" as a hypothesis imported from vehicle motion, not a result. §5.2's arithmetic
shows it is *false on this project's current harness motion by two orders of magnitude*, which is
a statement about the harness, not about the source.

**Packetized predictive control over erasure channels** — Nagahara, Quevedo, Nešić, "Sparse
Packetized Predictive Control for Networked Control over Erasure Channels", arXiv:1307.8242,
<https://arxiv.org/abs/1307.8242>. *Proposes:* exactly V4 — at every sampling instant the
controller transmits a packet containing the whole finite-horizon predicted **input sequence**
minimizing a finite-horizon cost; the actuator side keeps a **buffer** and consumes the *k*-th
element of the most recently received sequence, falling back through older sequences during
dropouts. Sparsity-promoting ℓ₁-ℓ₂ / ℓ₂-constrained-ℓ₀ optimizations shrink the packet.
*Claims:* practical stability **provided the number of consecutive dropouts is bounded**.
*Conditions:* I could retrieve only the abstract; it does not state plant, sampling rate, horizon
in samples, or dropout rate, and it explicitly assumes *bounded* consecutive dropouts rather than
a Markov chain. That mismatch is not cosmetic — this project's Gilbert-Elliott burst length is
geometric and therefore **unbounded**, so the entire stability guarantee is conditioned on an
assumption our profiles violate by construction. The follow-on "Robust stability of packetized
predictive control ... with Markovian packet losses"
(<https://www.sciencedirect.com/science/article/abs/pii/S0005109812002191>) removes the bounded
assumption and is the version that would actually apply; I did not retrieve it. **This is the
closest prior art to the candidate as written, and it is the source a V4 ADR should start from.**

**Model-mediated teleoperation** —
<https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.611251/full>
and <https://mediatum.ub.tum.de/doc/1326017/1326017.pdf>. *Proposes:* transmit *model parameters*
of the remote environment rather than sampled signals; the master reconstructs a local model and
computes non-delayed force feedback from it. *Claims:* stability and transparency robust to
arbitrary delay and to packet loss. *Conditions:* haptic/force loops at ~1 kHz with contact
dynamics — a different loop, a different rate, and a different failure mode (energy/passivity)
than a 100 Hz visual-setpoint loop with no force channel. Structurally the strongest form of
"send a model, not a sample", and probably why the axis file calls this row interesting — but
nothing in this project has a haptic loop or a contact model, so the transferable part is the
framing, not the technique.

**ROS 2 `trajectory_msgs` / `joint_trajectory_controller`** —
<https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html>.
*Proposes:* the industry-standard shape of V4 — a coordinated sequence of joint configurations
with `time_from_start` per point, plus a fire-and-forget topic interface. *Claims:* nothing about
network loss; the docs address tolerances and clock discontinuities, not dropouts. Its value here
is as an **existence proof of the consumer contract**: a time-parameterized setpoint buffer with a
per-point time offset is the shape any V4 ADR should converge on, and `robot/` already speaks
ROS 2, so a plan type mirroring `JointTrajectoryPoint` costs nothing extra at the hardware
boundary.

**Overwatch's input-buffer / rollback netcode** (GDC 2017, Ford), secondhand via
<https://edgegap.com/blog/game-backend-deep-dive-overwatch-2016-netcode-architecture-rollback>.
*Proposes:* the *opposite* trade — the client varies its own tick rate to keep the server's input
buffer full so the server never has to extrapolate, and handles misprediction by rollback against
an authoritative server. *Conditions:* deterministic simulation, authoritative server, sub-100 ms
links. Relevant as a counterexample: the game industry's answer to "the receiver has no input for
this tick" was to **over-supply real inputs and pace the sender**, not to extrapolate intent —
and pacing is candidate 4's territory, not mine.

**Hand-motion predictability in VR — the operating point that decides this candidate.** Search
summaries around <https://arxiv.org/pdf/2507.13179> and
<https://cs.brown.edu/people/jlaviola/pubs/kfvsexp_final_laviola.pdf> report (a) that LaViola's
double-exponential predictor matches EKF accuracy at ~135× the speed on head/hand pose — that is
already `double-exp` in `Prediction/` — and (b) that **"after 200 ms the RMSE increases
exponentially for all classical models of hand motion prediction"**, with hand motion showing the
largest variance of any tracked body part; remote-rendering work typically caps horizons below
100 ms for this reason. **200 ms is precisely the horizon at which classical extrapolation of
hand motion is documented to fall apart** — and 200 ms is the horizon the `trajectory` row
proposes. I could not retrieve "So Predictable! Continuous 3D Hand Trajectory Prediction in
Virtual Reality" (<https://dl.acm.org/doi/fullHtml/10.1145/3472749.3474753>) for its per-horizon
numbers; ACM returned HTTP 403. Recorded so the next run does not retry that URL unauthenticated.

**Searches that found nothing useful,** recorded so they are not repeated: "Quake 3 / Source
engine redundant `usercmd` per packet" returned only protocol overviews, with no quantified
comparison of past-input redundancy against forward extrapolation; and I found **no source at all
measuring forward-plan transmission on hand-held-controller motion**, as opposed to vehicles,
robot joints, or head pose. The single most relevant experiment for this candidate does not
appear to exist in the literature I could reach — which is a reason to run it, and also a reason
not to expect the DIS/NCS numbers to transfer.

---

## 3. Which existing metric moves, in which direction, on which profiles

### 3.1 The instrument does not exist. This is the headline of this section.

I verified the brief's claim by grepping every `IMetricSink.Record` call site in `core/`
excluding tests. The complete set of metric names emitted anywhere is:

| Name | Emitted by |
|---|---|
| `owd_uplink_ms`, `owd_downlink_ms` | `Pipeline/OperatorEndpoint.cs:227,232` |
| `prediction_position_error_mm`, `prediction_orientation_error_deg` | `Teleop.Eval/Sweep/SweepCommand.cs:304,305` |
| `correction_magnitude_mm`, `correction_magnitude_deg`, `time_to_convergence_ms`, `jerk_mm_s3` | the reconcilers (per `docs/metrics.md` §5) |
| `m2p_ms` | `Bridge/`, host-only, not available headless |

Confirmed: **everything in `docs/metrics.md` §3 — loss rate, burst-length distribution, jitter,
reordering rate, goodput — is defined and emitted by nothing.**

The metric this candidate needs is **command-tracking error**: the distance between the pose the
operator commanded at time *t* and the plant's actual pose at time *t*. It is not in
`docs/metrics.md`. Nothing substitutes for it:

- `prediction_position_error_mm` is `predictor-estimate-of-robot` vs `robot-truth`. Different
  pair, different question. **It must not be re-scoped to mean tracking error** — that would
  silently change the ruler and invalidate every prediction result in the repo. Hard stop.
- `docs/metrics.md` §2's **C2A** is the nearest defined concept and it is a *time*
  (`t_actuation − t_capture`), not a distance, and is emitted by nothing.
- §6's path efficiency would capture it, but requires the frozen benchmark tasks, which have no
  headless implementation.

So: **the candidate's entire benefit is currently unmeasurable, and making it measurable requires
adding a metric definition to `docs/metrics.md`, which this run is forbidden to do and which a
human must own.**

### 3.2 The harness supplies no input signal either

`SweepCommand.cs:269` — `operatorEndpoint.SubmitCommand(operatorPose, Vector3.Zero, Vector3.Zero,
gripper: 0f, now)`. Confirmed. `CommandFrame.LinearVelocity`/`AngularVelocity` — the fields whose
own doc comment says "intent is what survives a lost packet" and "the trajectory codec
extrapolates from them" — are **identically zero in every sweep this project has ever run**. The
`RigidBodyPlant` coast that §1.2 identifies as the one existing plan consumer therefore always
coasts at zero, i.e. it *holds*. The documented gap policy of the Core plant has never once been
exercised.

`SyntheticOperatorMotion` (`SweepCommand.cs:330-335`) is
`x = sin(t)·0.5`, `z = 1 + cos(0.7t)·0.3`, `y = 0`, rotation `Quaternion.Identity`. Two further
consequences:

- **`prediction_orientation_error_deg` is identically zero in every run.** The commanded rotation
  is always identity, `AngularVelocity` is always zero, so the plant's rotation never leaves
  identity and the predictor's estimate of it never does either. Any rotational aspect of a
  trajectory codec is untestable, and the orientation half of `correction_magnitude_deg` is a
  dead channel.
- The motion is analytic and smooth, with no intent-change events beyond three slow sinusoidal
  turning points in a 5 s trial.

**Whose job is it to make the harness produce real intent?** `SweepCommand.cs` is in
`core/Teleop.Eval/`, which is out of scope for me and, per the brief, human-owned. It is a small
change (the analytic derivative of the existing sinusoid is four lines) but it is **not** a small
decision: it changes what the `none`/`snap` baseline does through every gap, so every existing
`results/` run becomes non-comparable to every run after it. It must be added as a *new*
configuration dimension, not as an edit to the existing path. **That is a baseline change and it
is a human's to make.** Flagged as the single highest-value cheap action out of this whole
assessment (§9).

### 3.3 Cadence and burst arithmetic, against the actual profiles

`SweepCommand`: `TicksPerSecond = 10_000_000`; every committed experiment uses
`stepIntervalTicks: 100000` and `trialSteps: 500`. That is **10 ms per step, 100 Hz, a 5 s
trial, 500 uplink datagrams per direction per trial.** A 200 ms plan horizon therefore spans
**20 command frames**.

| Profile | Loss (deliv/lost) | E[burst] pkts | E[burst] ms | Bursts per 500-pkt trial | Can it distinguish this candidate? |
|---|---|---|---|---|---|
| `lan` | 0 / 0 | — | 0 | 0 | **No. Literally nothing.** |
| `50ms-5j` | 0 / 0 | — | 0 | 0 | No (jitter ±5 ms ⇒ gaps ≤ 20 ms) |
| `150ms-20j-0.5loss` | 0.005 / 0.005 | 1.005 | 10 | ~2.5 singletons | Barely — ~25 ms of gap in 5000 ms |
| `300ms-60j-2loss-bursty` | 0.00612 / 0.70 | 3.33 | 33 | ~3 bursts | **Yes — the only one in the frozen set** |
| `synthetic-burst` (trace) | **0 / 0** | — | 0 | 0 | Only via *delay* bursts, not loss (see below) |
| `jitter-<N>ms` (ADR 0005) | 0 / 0 | — | 0 | 0 | Only via jitter-induced gaps (≤ 10+2N ms) |
| `delay-<N>ms` (ADR 0005) | 0 / 0 | — | 0 | 0 | No |
| `loss-<N>pct` (ADR 0005) | Bernoulli by design | ≈1 | 10 | up to 25 singletons at 5% | Intent-vs-no-intent only; **cannot distinguish horizon** |
| `combo__…loss-<N>pct` (ADR 0006) | Bernoulli by design | ≈1 | 10 | as above | Same limitation |

Three findings fall out of this table.

1. **Exactly one resolvable profile in the entire catalog has bursty loss**:
   `300ms-60j-2loss-bursty`. ADR 0005 §"loss family" and ADR 0006 §"Loss is Bernoulli" both state
   they removed burst shape from the loss axis *on purpose*, to avoid a rate/burst-shape
   confound. That is the right call for those families and it means **neither extension family
   can test this candidate's actual mechanism**, which is surviving a *burst*.
2. **`synthetic-burst` has zero loss.** Verified in
   `core/Teleop.Eval/Sweep/NetworkProfileCatalog.cs:79-82`: it constructs
   `new NetworkProfile(baseDelayTicks: 0, jitterTicks: 0, lossProbabilityAfterDelivered: 0.0,
   lossProbabilityAfterLost: 0.0, reorderProbability: 0.0, reorderDelayTicks: 0)` and supplies
   the trace only as a *delay* sequence. Its "burst" is a congestion-delay burst. It still
   produces a **setpoint gap** — the plant receives nothing for the burst duration, then a run of
   arrivals — so it is arguably a *good* profile for this candidate, but anyone reading the name
   as "burst loss" would be wrong.
3. **`300ms-60j-2loss-bursty` yields only ~3 burst events per trial.** Geometric burst length
   with continue-probability 0.7: P(L ≥ k) = 0.7^(k−1), so the ~p99 burst is L ≈ 14 packets
   = 140 ms, and a 200 ms plan covers roughly p99.7 of bursts. But you will not *see* a p99 burst
   in a single 500-step trial that contains three of them. Any experiment here needs many seeds
   or much longer trials, and **p99 of a metric emitted 500 times per trial is being computed
   over the top five samples**. That is a fragile place to claim a percentile win.

### 3.4 Direction of movement on the metrics that do exist

Assuming §3.2 is fixed so the harness sends real intent, and holding predictor/reconciler fixed:

| Metric | Moves? | Direction | Why, and is it evidence? |
|---|---|---|---|
| `owd_uplink_ms` / `owd_downlink_ms` | **No** | — | A codec cannot change one-way delay. `EmulatedTransport` draws loss per datagram **independent of payload length** (`Send` never reads `payload.Length`), so a bigger datagram is not more likely to be lost, and the survival-conditioned OWD population is unchanged. **Use this as a sanity check: if OWD moves, something is broken or the emission cadence changed.** |
| `prediction_position_error_mm` | Yes | **down** | Confounded. It compares the *operator-side* predictor against `plant.State.Value`. An uplink intent codec changes the *ground truth* — the plant coasts smoothly instead of freezing — and a smoothly-moving plant is easier for `const-vel`/`double-exp` to track. Lower error here means "the robot moved more smoothly", **not** "the robot went where the operator wanted". |
| `prediction_orientation_error_deg` | **No** | — | Identically zero today (§3.2). |
| `correction_magnitude_mm` | Yes | **down** | Same confound: less predictor disagreement because the plant is smoother. |
| `correction_magnitude_deg` | **No** | — | Dead channel (§3.2). |
| `jerk_mm_s3` | Yes | **down** | Same confound, and the sharpest form of it: a plan followed through a gap is **smooth by construction whether or not it is right**. |
| `time_to_convergence_ms` | Marginal | down | `snap` reports 0 always; only smoothed reconcilers would show anything. |
| goodput / loss rate / burst distribution | — | — | Defined in `docs/metrics.md` §3, **emitted by nothing**. Which means the candidate's one real cost — bandwidth — is invisible. |

**The shape of the problem, stated plainly:** every metric that would move, moves in the
candidate's favour, and every one of them moves for a reason that is not the candidate's claimed
benefit. The one metric that would falsify it does not exist, and the one metric that would price
it (goodput) does not exist either. `EmulatedTransport` has no bandwidth, serialization-delay, or
MTU model at all, so a 20-knot plan costs literally zero in simulation. Reporting a `trajectory`
win on this harness would be the exact analogue of reporting prediction error without correction
cost — and the repo already has a rule against that (`docs/metrics.md` §8.2).

---

## 4. What it costs to build here

### 4.1 Contracts it fits unchanged

- `ICommandCodec` — fits, but see §1.3: the fit is vacuous, because the decoder cannot express a
  plan.
- `ITransport` — fits. `MaxEncodedBytes` is already documented as an upper bound, so a
  variable-length codec is contractually fine.

### 4.2 Missing infrastructure, priced

| Item | Where | Size | Owner |
|---|---|---|---|
| Harness emits real velocity intent | `Teleop.Eval/Sweep/SweepCommand.cs:269` + `SyntheticOperatorMotion` | ~5 lines, but it is a **baseline change** | human |
| A command-tracking metric | `docs/metrics.md` + `SweepCommand` | new definition + ~4 lines | human (metric definitions) |
| Sweep can select a codec | `ExperimentConfig` has no codec field; `SweepCommand` hardcodes `new RawPoseCodec()` at lines 258 and 261, and sizes the uplink with the **fixed** `RawPoseCodec.EncodedSize` at line 250 | 4 sites + config type + `analysis/`'s `stacks` schema | human, `Teleop.Eval` |
| Variable-length uplink | line 250 must become `codec.MaxEncodedBytes`; note `OperatorEndpoint`'s constructor **throws** `ArgumentException` if `commandCodec.MaxEncodedBytes > uplinkTransport.MaxPayloadBytes`, so a fat codec against a `RawPoseCodec.EncodedSize`-sized transport fails loudly at wiring rather than silently — good, but it means the two must be changed together | as above | human |
| A plan type + plan consumer | new `Contracts/` interface, new `Plant/` implementation, `RobotEndpoint` wiring | **ADR first** | human decision, then Core |
| Motion with intent changes | `SyntheticOperatorMotion` | new motion generator | human, `Teleop.Eval` |

A note on the plan type under invariant 6 and 8, because it constrains the ADR: Core is C# 9 /
`netstandard2.1` with no `AllowUnsafeBlocks` in `Teleop.Core.csproj`. There is no `InlineArray`
(C# 12) and no `fixed` buffer, and a `ref struct` holding a `Span<byte>` cannot be stored in a
plant field. So a plan value type is either a struct with N hand-written knot fields (ugly, N
fixed at compile time) or a **class with a preallocated knot array**, constructed once and
refilled in place. The latter is the only clean option and it is compatible with invariant 8
(allocate in the constructor). Worth deciding in the ADR rather than discovering during
implementation.

### 4.3 One unit of work or several?

**Several — at least four, and only the last is this candidate.**

1. Harness emits intent (baseline change, human).
2. Command-tracking metric defined and emitted (metric change, human).
3. A motion profile with intent-change events (harness, human).
4. ADR for a plan wire shape and a plan-evaluating plant; then the codec, the plan type, the
   plant, the registry entries, the tests, the experiment YAML.

Steps 1–3 are prerequisites for *any* honest measurement on this axis, and they are prerequisites
for candidates 1, 3 and 4 as well. Step 4 without steps 1–3 produces a number that cannot be
interpreted.

### 4.4 The one version that fits today, and why it is still blocked

**V1, intent shaping.** A codec with the same 73-byte layout as `raw` that writes `α·v` instead
of `v`, α a swept parameter in [0, 1]. α = 0 is "plant holds through gaps"; α = 1 is `raw`;
α > 1 is aggressive. It fits `ICommandCodec` unchanged, needs no ADR, no wire-format change, no
new contract, and it is a genuine one-parameter tradeoff — coast accuracy through a gap against
overshoot at an intent reversal — which is exactly the shape of finding this project values.

It is nonetheless blocked on the *same* two prerequisites: with `Vector3.Zero` on the wire every
α scores identically, and with no tracking metric the sweep cannot tell an α that tracks well from
one that merely looks smooth. So even the ADR-free version of this candidate cannot be measured
today. That is the strongest single argument that the prerequisites, not the codec, are the work.

---

## 5. The falsifier

### 5.1 Hypothesis, stated so it can fail

*Transmitting the operator's near-future intended motion, rather than an instantaneous pose,
reduces command-tracking error through uplink gaps on burst-loss and burst-delay profiles,
without a compensating increase in overshoot at intent-change events.*

Falsified if any of: (a) tracking error p95 does not improve on the one bursty-loss profile;
(b) it improves on `lan`, which has no gaps at all and where the only honest answer is "no
change"; (c) `jerk_mm_s3` and `correction_magnitude_mm` improve while tracking error does not
— which would confirm the mechanism is buying smoothness, not correctness; or (d) the
second-order/plan variants beat first order by less than the reconciler's own 5 mm convergence
tolerance, in which case the extra bytes buy nothing observable.

### 5.2 The arithmetic that predicts (d) analytically, before running anything

The current synthetic motion is `x = 0.5·sin t`, `z = 1 + 0.3·cos(0.7t)`. Bounds:
`|v| ≤ 0.542 m/s`, `|a| ≤ 0.521 m/s²`, `|j| ≤ 0.511 m/s³`. Extrapolation error over a gap Δ, by
Taylor order, in millimetres:

| Gap Δ | order 0 (hold — **what the harness does today**) `\|v\|Δ` | order 1 (`raw` with real intent) `½\|a\|Δ²` | order 2 (a trajectory codec) `⅙\|j\|Δ³` |
|---|---|---|---|
| 10 ms (single-packet loss, the `loss-<N>pct` family) | 5.4 | 0.026 | ~0 |
| 33 ms (mean burst on `300ms-60j-2loss-bursty`) | 17.9 | 0.28 | 0.003 |
| 140 ms (~p99 burst, ~1 in 100 trials at 3 bursts/trial) | 75.9 | 5.1 | 0.23 |
| 200 ms (the proposed horizon; ~p99.7 burst) | 108.5 | 10.4 | 0.68 |

Read the columns, not the rows. **Fixing the harness (order 0 → order 1) is worth 18 mm at the
mean burst and 108 mm at the horizon. Building the candidate on top of that (order 1 → order 2)
is worth 0.28 mm at the mean burst and 9.7 mm at a burst that occurs roughly once per hundred
trials.** The reconciler's own convergence tolerance is 5 mm. So on the harness as it stands, the
candidate's entire incremental value sits **below the tolerance at every burst length except the
p99.7 tail**, and one to two orders of magnitude below the harness defect it is hiding behind.

Inverting it: for second order to be worth 10 mm at the *mean* 33 ms burst you need
`|a| ≈ 18 m/s²`; at a 140 ms burst you need `|a| ≈ 1.0 m/s²`. The current motion has 0.52 m/s².
So the candidate needs roughly 2× the current acceleration to matter at p99 bursts, and ~35×
to matter at typical ones.

**This is the single most decisive finding in the assessment and it required no code.** It also
means the DIS "second order far more efficient than first order" claim (§2.3) is simply not
transferable: it was measured on vehicles under threshold-triggered updates seconds apart, where
`|a|Δ²` is large. Here Δ is 33 ms.

### 5.3 The cheapest experiment that could kill it — and it does not require building it

**E1: intent-vs-no-intent, `raw` only.** Vary exactly one thing — whether `SubmitCommand`
receives the analytic derivative of `SyntheticOperatorMotion` or `Vector3.Zero`. Hold codec
= `raw`, predictor = `none`, reconciler = `snap` (the mandated baseline row). Profiles: `lan`
(control, must show no difference), `loss-5pct` (25 singleton gaps), `300ms-60j-2loss-bursty`
(the only bursty-loss profile), `synthetic-burst` (delay gaps, zero loss). ≥ 8 seeds, and report
the seed spread — with ~3 burst events per trial, run-to-run variance will be large. Report
p50/p95/p99 of the new tracking metric **together with** `correction_magnitude_mm` and
`jerk_mm_s3`, per `docs/metrics.md` §8.2.

Why this kills the *candidate* and not just its prerequisite: E1 measures the total headroom
available to intent transmission at first order. Per §5.2, first order captures ≥ 98% of that
headroom at every burst length below the p99 tail. **If E1's improvement is small, there is no
room left for a trajectory codec to win, and the row closes without anyone writing it. If E1's
improvement is large, it was the harness all along, and the correct next step is still not a
codec (§6.2).** Either outcome closes this candidate.

### 5.4 The motion profile it would need to be honest

The current sinusoid is *perfectly* extrapolable: infinitely differentiable, three slow turning
points in 5 s, no rotation at all. It would make any intent-transmitting scheme look excellent
for a reason that has nothing to do with real operators. **Yes — the current harness could
produce a misleadingly positive result for this candidate, and I would not believe one produced
on it.**

What is needed instead, and it is already in the project's vocabulary: **min-jerk reciprocal
reaching (Fitts tapping)**, which `docs/metrics.md` §6 already names as a frozen benchmark task.
A 0.3 m reach in 0.5 s has peak acceleration `5.77·D/T² ≈ 6.9 m/s²` — an order above the current
motion — and, critically, a **discrete intent-change event at every tap**. That is where a 200 ms
plan is wrong for 200 ms, where `const-accel`'s own planned-row note ("overshoots on direction
reversal") bites hardest, and where the hand-motion literature's "RMSE increases exponentially
past 200 ms" (§2.3) should show up. Non-zero commanded rotation is also required, or the
orientation half of every metric stays dead.

My honest expectation, recorded before anyone runs it: **on reversal-rich motion a longer or
higher-order plan loses**, because commitment cost grows with horizon while coverage benefit
saturates at the burst length. That is the interesting shape — a horizon with an optimum, not a
monotone win — and it is exactly what candidate 3 (adaptive horizon) would need in order to have
anything to adapt to. Which is an argument for running E1 *before* candidate 3 as well.

---

## 6. Is it distinguishable by measurement from something already implemented?

### 6.1 From `const-vel` / `double-exp` operator-side prediction — **not on today's harness**

They are different mechanisms on different halves of the loop: the predictors extrapolate
*downlink robot state* at the operator; a trajectory codec fills *uplink command gaps* at the
robot. They are not in competition and they compose.

But **every metric that exists scores the operator/downlink side**. `prediction_position_error_mm`
compares the predictor's estimate to `plant.State.Value`; `correction_magnitude_mm` and
`jerk_mm_s3` come off the operator-side reconciler. An uplink codec reaches those metrics only by
changing the plant's motion — i.e. by changing the *ground truth* the predictor is scored against.
So an uplink trajectory codec and a better operator-side predictor **both show up as "prediction
error and jerk went down", for entirely different reasons, and nothing in the harness separates
them.** That is a finding, and I state it as the brief requires: **on the harness as it stands, a
trajectory codec and a good predictor are behaviourally indistinguishable in the metrics, and a
sweep that varied both would produce numbers nobody could attribute.**

Corollary for whoever runs the axis: codec and predictor must be swept one at a time
(root-`CLAUDE.md`'s coupled-axis rule applies here as much as to playout/prediction), and
`Transport/CLAUDE.md`'s own instruction to "benchmark the pair" cannot be honoured until there is
a metric that separates them.

### 6.2 From the thing that is strictly cheaper and does the same job — **robot-side intent prediction**

This is the finding I did not expect and it changes the recommendation.

`RigidBodyPlant`'s coast is a robot-side constant-velocity predictor of operator intent,
implemented inside the plant, whose velocity input happens to arrive over the wire. But the robot
**already has the information to estimate that velocity itself** — it receives a stream of
`CommandFrame.Pose` values with `CaptureTicks`, and differencing them gives `v` without any wire
change at all. `Prediction/CLAUDE.md` names this as a distinct, first-class problem — "Robot-side:
the commands you have are stale; predict what the operator wants *now*" — and `Pipeline/CLAUDE.md`
confirms it is unwired: "Only operator-side prediction is wired ... `RobotEndpoint` is untouched".

So there is an alternative that obtains the same first-order benefit with:

- **no wire-format change** (`raw` unchanged, `RawPoseCodec.EncodedSize` unchanged, every existing
  `results/` run stays comparable);
- **no new contract** — it reuses `IPredictor<Pose>` and the existing `const-vel` / `double-exp`
  registry entries verbatim;
- **no ADR for the contract**, and by the Phase-5 precedent (`OperatorEndpoint` gained an injected
  predictor/reconciler pair as required constructor dependencies) probably no ADR for the wiring
  either — though injecting a predictor into `RobotEndpoint` is a `Pipeline/` composition change
  and a human should confirm that read;
- **no dependence on the harness sending intent**, because it derives intent from the pose stream
  the harness already sends. It is the one option in this whole assessment that is *not* blocked
  on §3.2.

It is also strictly more general than the codec at first order: differencing works on any pose
stream, including from a `raw` sender that never populated the velocity fields, and it degrades
gracefully because the predictor sees the actual arrival pattern rather than a plan authored
before the loss occurred. Its disadvantage is real and worth measuring: the sender knows its own
intent exactly, while the receiver's differenced estimate is noisy and one sample stale. **That
difference — transmitted intent vs. differenced intent — is the genuinely interesting experiment
on this axis**, and it is a Prediction-axis experiment, not a Transport-axis one.

Recording this as a judgement call: I am recommending against my own assigned candidate in favour
of a neighbouring axis's unbuilt row. The reason is §1.3 (the codec cannot express what it wants
to send) plus §5.2 (what it *could* express beyond first order is worth sub-millimetre here), not
a preference about axes.

### 6.3 From candidate 1 (redundancy / FEC) — and how a reader tells them apart

V5 (overlapping trajectory segments) *is* a redundancy scheme: consecutive 200 ms plans at 100 Hz
overlap by 190 ms, so each interval is covered by twenty datagrams. Anyone comparing V5 against
`redundant` at matched bytes-on-wire and matched coverage would be comparing "twenty extrapolated
futures" against "twenty recorded pasts".

The distinction is not just bookkeeping, and it cuts sharply in the candidate's favour on the
uplink:

> **For a setpoint stream, past-frame redundancy is nearly useless against the gaps that matter.**
> The plant only ever wants the newest setpoint. A gap in the *middle* of the stream is
> self-healing — if packet *k* is lost but *k+1* arrives, *k+1* supersedes it, and the redundant
> copy of *k* riding in *k+1* is worth nothing. The gap that actually hurts is the **trailing**
> one: the newest *m* packets are all lost and the plant has nothing current. Past-frame
> redundancy cannot help there by construction, because the only copies of the missing packets
> live in packets that were themselves lost. A forward plan *can* help there, because the last
> successfully received packet carries coverage extending into the gap.

So the clean discriminating experiment, and the one I would hand to candidate 1's owner as a
cross-candidate check: **on burst loss, plot tracking error against burst length for `redundant`
and for a forward-plan scheme at matched bytes.** `redundant` should be flat — no improvement at
any burst length, because every gap in a monotonic setpoint stream is trailing. A forward plan
should improve up to its horizon and then fall off a cliff. If `redundant` shows a benefit
anyway, the benefit is coming from reordering tolerance, not loss recovery, and both candidates
are mismeasured.

(This argument is specific to a *setpoint* stream where only the newest value is consumed. It
does not apply to the downlink if a playout buffer is ever built, because a buffer does consume
old samples — which is one more reason `Buffering/` changes the answer here.)

---

## 7. Bottom line

**Do not build `trajectory`. Build the prerequisites, then build robot-side intent prediction
instead.**

The reason, in one sentence: `ICommandCodec.TryDecode` returns a `CommandFrame`, `CommandFrame`
carries exactly one order of intent, and `raw` already fills it — so a trajectory codec is
unobservable at the decoder without an ADR, and the arithmetic says what it would express beyond
first order is worth 0.3 mm at the mean burst on this project's motion.

Restated as the three verdicts a reader wants:

- **`trajectory` as an `ICommandCodec`: do not build.** Unobservable by construction (§1.3),
  incremental value below tolerance (§5.2), no instrument to score it (§3.1), no input signal to
  feed it (§3.2), and one profile in the catalog that could show anything (§3.3).
- **`trajectory` as a plan contract (V3/V4): not now, and it is an ADR.** Revisit only after E1
  has been run on reversal-rich motion and has shown residual headroom that first order does not
  capture. If E1 shows no residual, this row should be closed permanently.
- **The work that should happen on this question:** harness intent → tracking metric →
  reversal-rich motion → robot-side intent prediction in `RobotEndpoint` (§6.2). The first three
  unblock candidates 1, 3 and 4 as well.

---

## 8. How this assessment could flatter itself

Required section, and I have four.

1. **My arithmetic is analytic and could be too kind to *my own conclusion*.** §5.2 treats plant
   error as equal to the Taylor remainder over the gap. In reality `RigidBodyPlant.Command` snaps
   on every accepted frame, so error does not accumulate across gaps — which makes the candidate
   look *even smaller*, not larger. The bias here runs against the candidate but in favour of my
   verdict, so I should say plainly what would overturn it: a plant with real dynamics, actuator
   lag, or a hold policy would accumulate error differently and could make higher-order intent
   matter more. `Plant/CLAUDE.md` explicitly invites such a plant as a second implementation.
   **My verdict is conditional on `RigidBodyPlant`.**

2. **I may be over-reading `ICommandCodec` as a closed door.** A codec *could* return a
   `CommandFrame` whose `LinearVelocity` is not the instantaneous tangent but something
   plan-derived — that is V1 in §4.4, and it is real. I have not shown that no clever use of the
   existing fields captures a useful fraction of the plan's value; I have only shown that the
   fields cannot carry *more than first order*. If someone demonstrates that a shaped
   first-order intent captures most of the benefit, that is a win for V1 and a partial reversal
   of my "unobservable" framing. It would still not be `trajectory` as the axis table describes
   it.

3. **The `ekf` parallel is rhetorically convenient and I reached for it because the brief pointed
   at it.** The two are not identical: `ekf`'s output was unobservable because no *consumer*
   read the field; `trajectory`'s is unobservable because no *field exists*. I think that makes
   the case stronger, but a reader should notice I was primed to find that shape and check the
   §1.3 trace themselves rather than take the analogy on trust. The trace is the evidence; the
   analogy is decoration.

4. **I never ran anything.** Every number in §5.2 is a closed-form bound on a motion I read out
   of `SweepCommand.cs`, not a measurement, and it carries no manifest and no SHA — it is not a
   citation in this repo's sense and must not be reported as one. The bounds are non-simultaneous
   maxima of the two axes, so they are conservative (real errors will be smaller), which again
   runs against the candidate. If any of it is wrong, the way to find out is E1, which is cheap.

One more, about scope rather than reasoning: I assessed the *uplink*. A "robot sends its plan"
variant on the downlink is a different idea with a different consumer story — `RobotStateFrame`
carries a `Pose` and **no velocity at all**, so the downlink transmits strictly less intent than
the uplink does, and the operator-side predictor reconstructs velocity by differencing. Adding
velocity to `RobotStateFrame` would be a wire v3 bump and an ADR. I did not price it, and it may
be the better half of the loop to attack, because that is the half every existing metric already
watches. Recorded as unexamined.

---

## 9. Left undone / for a human

Nothing was built. In priority order, what a human should decide or do:

1. **Make the harness able to send real velocity intent** — `SweepCommand.cs:269` plus the
   analytic derivative of `SyntheticOperatorMotion`. Small code, real decision: it changes the
   `none`/`snap` baseline's behaviour through every gap and must therefore be a **new
   configuration dimension, not an edit to the existing path**, or every prior `results/` run
   becomes non-comparable. This unblocks candidates 1, 2, 3 and 4 and is the highest-value cheap
   action out of this assessment.
2. **Decide whether to add a command-tracking metric to `docs/metrics.md`** — "operator commanded
   pose at *t*" vs "plant pose at *t*", mm, p50/p95/p99. Without it the Transport axis has no
   correctness instrument at all and can only report smoothness. **This must not be done by
   re-scoping `prediction_position_error_mm`**, which measures a different pair.
3. **Decide whether the frozen profile set needs a second bursty-loss point.** Exactly one
   resolvable profile has bursty loss (§3.3), and ADRs 0005/0006 removed burst shape from both
   extension families on purpose. Any loss-burst research on this platform is currently a
   single-point study. That is an ADR-level question about the benchmark suite and it affects
   candidate 1 at least as much as candidate 2.
4. **Rename or annotate `synthetic-burst`,** or at least note in `Transport/CLAUDE.md` that its
   burst is a *delay* burst and its loss is zero. I did not edit that file (out of scope). The
   name misleads.
5. **Consider `RobotEndpoint` gaining an injected `IPredictor<Pose>`** for operator-intent
   prediction (§6.2) — reusing existing contracts and registry entries, probably no ADR by the
   Phase-5 precedent, and not blocked on item 1. This is my recommended next unit of work on the
   underlying question, and it belongs to the Prediction axis, not Transport.
6. **`Transport/CLAUDE.md`'s "Tried and rejected" section is `(none yet)` and should gain a
   `trajectory` row** pointing here — but rejected *on argument*, not on measurement, and the
   distinction must be written down. This assessment produced no `results/` directory, so the row
   should link this log and say so. I did not edit that file; the organizer carries it.

## Verification

The three gates were run once at the start of this run, before anything was written, to confirm
the tree was green — and nothing in this run changes code, so they are unchanged by it.

| Gate | Result |
|---|---|
| `dotnet test` (`core/Teleop.sln`) | **PASS** — 558 + 36 + 28 + 3 tests, 0 failed |
| `Teleop.Eval -- verify` | **PASS** — golden log replays byte-identical across two passes and matches the original file |
| `Teleop.Eval -- audit` | **PASS** — no invariant violations |

No `.cs` file, test, `.meta`, registry entry, experiment YAML, sweep, or `results/` directory was
created or modified. No shared file (`docs/metrics.md`, any `CLAUDE.md`, any ADR,
`Registry/Registries.cs`, anything under `core/Teleop.Eval/`) was edited. No git command that
writes was run. The working tree's only change is this file.
