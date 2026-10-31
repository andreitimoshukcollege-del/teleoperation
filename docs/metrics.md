# Metrics

Every metric this project reports is defined here. **If it isn't defined here, it isn't a
metric** — add the definition in the same PR that emits it. Undefined metrics are how a
research project ends up with numbers nobody can interpret six months later.

Conventions used throughout:

- Time in **milliseconds**, on the synced clock (`Time/ClockSync.cs`). Never a local clock.
  Cross-clock figures from real cross-machine sessions recorded before 2026-10-31 may be a few
  milliseconds off: the clock-offset estimate was stuck on a coarse grid (docs/adr/0008
  amendment). The run's `manifest.json` SHA dates it.
- Distances in **millimetres**, angles in **degrees**, both in ROS convention (right-handed,
  Z-up, X-forward).
- Every distribution reported as **p50 / p95 / p99**, never a mean alone. These distributions
  are heavy-tailed and the tail is what the operator perceives.
- Every reported figure carries the network profile, seed, and git SHA from the run manifest.

## 1. Timestamps

Every sample carries these stamps. All later metrics are differences between them.

| Stamp | Meaning |
|---|---|
| `t_capture` | sensor or input device sampled the value |
| `t_send` | serialized and handed to the transport |
| `t_recv` | arrived at the far end (stamped on the network thread, not the main thread) |
| `t_playout` | consumed by the playout policy for use |
| `t_render` | frame containing it was submitted to the compositor |
| `t_photon` | `t_render + DisplayOffset`; estimated light emission |

Stamping `t_recv` on the main thread instead of the network thread folds frame time into
measured network delay. This has been a real source of wrong numbers — check it if one-way
delay looks suspiciously frame-quantized.

## 2. Latency

**One-way delay (OWD)** — `t_recv − t_send`, per direction, reported separately. Uplink and
downlink are frequently asymmetric and averaging them hides that.

Metric names emitted via `IMetricSink.Record`: `owd_uplink_ms` (`t_recv − t_send` for the
operator→robot command) and `owd_downlink_ms` (`t_recv − t_send` for the robot→operator state
reply), both in milliseconds, both already converted into the operator's canonical clock domain
via `ClockSync` before the subtraction (`docs/adr/0002-latency-trace.md`). First emitted by
`Pipeline/OperatorEndpoint.cs`.

**Latency-trace eviction** — the denominator correction for everything above. `OperatorEndpoint`
holds an open `LatencyTrace` per submitted command in a fixed-capacity ring; if the ring is too
small for the round trip in flight, a trace is displaced before its reply arrives and that round
trip contributes to no `owd_*` sample at all. Because the displaced ones are the slowest, the
censoring is **delay-correlated** — it removes the tail of the distribution being reported.

Metric name: `latency_trace_evicted`, one sample of value 1 per displaced trace, stamped at the
displacing submission. Emitted by `Pipeline/OperatorEndpoint.cs`. **A run with any of these has
understated `owd_*` percentiles and the shortfall is not random**; the fix is a larger ring or a
longer `inFlightMaxAgeTicks`, not a caveat in the writeup. Expiry of a trace whose reply the link
lost is *not* counted here — that slot is reclaimed rather than displaced, and no completing round
trip is lost.

**Motion-to-photon (M2P)** — `t_photon(displayed) − t_capture(operator motion)`. The headline
number. Validate the software estimate against a physical rig at least once — LED plus
photodiode, or a high-speed camera on a spinning marker — then trust the software estimate and
re-validate whenever the render path changes.

Metric name emitted via `IMetricSink.Record`: `m2p_ms`, milliseconds, already in the operator's
canonical clock domain (both `t_photon` and `t_capture` are operator-domain natively per
`docs/adr/0002-latency-trace.md`, so no `ClockSync` correction applies here the way it does for
OWD). Host-only, for the same reason `t_render`/`t_photon` themselves are host-only
(`Types/LatencyTrace.cs`): Core has no compositor to produce either stamp. First emitted by
`Bridge/TeleopOperatorBridge.cs`. See `docs/adr/0003-display-offset-calibration.md` for where
`DisplayOffset` (folded into `t_photon` before this subtraction) comes from.

**Command-to-actuation (C2A)** — `t_actuation − t_capture` on the operator input that caused
it. The other headline number; separate from M2P because they can be improved independently.

**Stage breakdown** — capture, encode, transit, buffer, decode, render, display. Must sum to
M2P within tolerance; if it doesn't, a stage is unaccounted for. Report as a stacked
contribution, because the point of the breakdown is deciding where optimization is worth doing.

**Playout delay** — `t_playout − t_recv`: the `buffer` stage of that breakdown, and the price a
jitter buffer charges every sample whether or not that sample needed the wait. Separate from OWD
on purpose: `owd_downlink_ms` ends at `t_recv`, so before this metric existed a playout policy
could halve its late-arrival rate and the added latency appeared nowhere at all — a buffering win
read as free.

Metric name: `playout_delay_ms`, milliseconds, one sample per released sample, stamped at
`t_playout`. Zero for `immediate` by construction. First emitted by `Buffering/PlayoutMetrics.cs`.

**Playout budget** — the delay budget the policy is currently targeting between a sample's capture
and its playout: the x-axis of a latency/loss plot. Constant for `immediate` (always zero) and
`fixed`; for an adaptive policy this moving is the entire mechanism.

Metric name: `playout_budget_ms`, milliseconds, one sample per released sample. Report it
alongside `playout_delay_ms`, never instead of it — the budget is what the policy *intends*, the
delay is what it *charged*, and they differ whenever a sample arrives after its own due instant.

**Playout occupancy** — buffered samples as a fraction of the policy's capacity, in [0, 1].
Reported as a fraction rather than a count so it is comparable across policies configured with
different capacities. Metric name: `playout_occupancy`, dimensionless, one sample per released
sample. Occupancy pinned at 1.0 means the capacity, not the policy, is deciding the late rate.

## 3. Network

**Jitter** — report both, they answer different questions:
- RFC 3550 interarrival jitter (smoothed; comparable to the streaming literature)
- IQR of one-way delay (distribution-level; what the playout policy actually has to absorb)

Metric name for the first: `net_downlink_jitter_ms`, milliseconds, one sample per received
datagram after the first, stamped at that datagram's `t_recv`. RFC 3550 A.8 exactly —
`D(i−1,i) = (R_i − R_{i−1}) − (S_i − S_{i−1})`, `J += (|D| − J)/16` — computed from the sender's
**raw** stamp and the sender's own tick rate, never from a `ClockSync`-corrected stamp: `D` is a
difference of differences, so a constant clock offset cancels and feeding a corrected stamp would
report the movement of the offset estimate as network jitter. First emitted by
`Transport/NetworkObserver.cs`, fed from `Pipeline/OperatorEndpoint.cs`.

There is deliberately **no `net_uplink_jitter_ms`.** The only vantage holding an uplink datagram's
sender stamp is the robot, and `D`'s difference-of-differences cancels a clock *offset* but not a
clock *rate*; `CommandFrame` carries no operator `TicksPerSecond`, and
`docs/adr/0008-clocksync-cross-rate-normalization.md` records that omission as deliberate
("conversion is operator-side only, so the robot never needs the operator's rate"). Computing it
would mean assuming the two ends tick alike, which is the exact bug that ADR was written for.
Emitting it therefore needs a wire-format change, not an implementation.

The second is **derived at analysis time from `owd_uplink_ms` / `owd_downlink_ms`** and is not
separately emitted — it is a reduction of samples that already exist, and a second emitter for it
could only disagree with them.

**Loss rate** — fraction of sent datagrams never received.

Metric names: `net_uplink_sent` / `net_downlink_sent`, one sample of value 1 per datagram the
sending transport accepted, and `net_uplink_dropped` / `net_downlink_dropped`, one sample of value
1 per datagram it refused, both stamped at the send. Counts rather than a rate, following
`playout_late` below: a direction that loses nothing emits nothing rather than a stream of zeroes.
The rate is `count(dropped) / (count(sent) + count(dropped))`, computed by the analyst.

Also `net_uplink_received` / `net_downlink_received`, one sample of value 1 per datagram the
receiving transport delivered, stamped at its `t_recv`. Measured at the transport boundary, so it
counts what the link delivered including a datagram a codec later fails to decode.
`count(sent) − count(received)` is loss inflicted *after* acceptance, which no impairment in
`Transport/Impairments/` models and which should therefore be zero — it is emitted so that "should
be zero" can be checked rather than assumed.

All four are observed at `ITransport.Send`/`TryReceive`, which is what keeps this quantity
structurally incapable of absorbing the buffer's discards: no buffer exists at that boundary. See
the sharp warning under **Late-arrival (induced) loss** below. First emitted by
`Transport/NetworkObserver.cs`, fed from `Transport/MeasuredTransport.cs`.

**Loss burst-length distribution** — histogram of consecutive-loss run lengths. Report this
alongside the rate always. A 2% loss rate in bursts of 20 and a 2% rate of isolated drops
break a jitter buffer in completely different ways, and the rate alone cannot distinguish them.

Metric names: `net_uplink_loss_burst` / `net_downlink_loss_burst`, one sample per completed run of
consecutive refused sends, value = the run's length in datagrams, stamped at the acceptance that
ended the run. The histogram §3 asks for is the distribution of these samples, so its percentiles
and its maximum are read the same way as every other metric here, and
`sum(loss_burst) == count(dropped)` is an identity worth asserting in analysis. A run still open
when the observer is reset is discarded rather than emitted — the reset carries no tick to stamp
it at — so the distribution is censored by at most one run per trial, on count and not on shape.
First emitted by `Transport/NetworkObserver.cs`.

**Reordering rate** — fraction arriving out of sequence, plus max displacement.

**Measured as a total plus a three-way attribution, because a single number here was actively
misleading.** The observed sequence is the operator's, echoed back by the robot, so an inversion
seen at the operator can have been caused by any of three things. Reported under one
downlink-sounding name, it invited exactly the wrong conclusion: a headline "70% reordering on
`300ms-60j-2loss-bursty`" was quoted as a property of the link, and it is not one. On `jitter-5ms`
at a 10 ms step, distinct downlink send instants are ≥10 ms apart against a ±5 ms jitter range, so
the downlink *provably cannot* invert anything — yet 13.0% was measured.

The total: `net_roundtrip_reorder_displacement`, dimensionless, one sample per out-of-order
arrival, value = `highestSequenceSeen − thisSequence` (wrap-safe), stamped at that datagram's
`t_recv`. One metric answers both halves of the definition: the rate is
`count(net_roundtrip_reorder_displacement) / count(net_downlink_received)` and the max displacement
is the maximum of the same samples, so the two can never disagree about what counted as a
reordering. A sequence *gap* is not a reordering and is not counted, or this would just re-report
the loss rate. This is the quantity previously called `net_downlink_reorder_displacement`; only
the name changed, to stop it claiming a leg it never measured.

The attribution, all emitted alongside it and all sharing its displacement value:

- `net_uplink_reorder_displacement` — the datagram reached the **robot** after a later-sent command
  did. Sequence order is submit order by construction, so a `RobotRecvTicks` running the other way
  is the uplink inverting them. This is the metric that "needs a vantage which has decoded a
  `CommandFrame`" — it turns out not to: the robot echoes its own receive stamp on every reply, so
  the operator can order two of the robot's stamps without any clock correction.
- `net_downlink_reorder_displacement` — the robot emitted it **strictly** before a frame that had
  already arrived, so the inversion happened after the robot let go of it. Strictly, because frames
  sharing a send stamp left together and the downlink cannot be blamed for their order.
- `net_robot_reply_batched` — value 1 per frame sharing a send stamp with the arrival before it.
  Not a transit effect at all: `RobotEndpoint` replies to every command drained in one `Step` with
  the same `nowTicks`, so a batch leaves with identical stamps and independent downlink jitter
  shuffles it. Undercounts when the downlink separates a batch's members, which is itself
  informative — it means the batch mostly survived in order.

The three need not sum to the total: a frame can be inverted on both legs, and one inverted purely
by batching is attributed by the third rather than the first two.

**A residual limit no instrumentation removes.** A sequence-based reorder rate is defined relative
to send spacing, so it is never a cadence-free constant: halving the step doubles a fixed jitter
window's exposure. Measured directly — same ±5 ms jitter, 19.7% / 13.0% / 0.0% at 5 / 10 / 20 ms
steps. Always state the step interval alongside a reordering figure, the way §8 rule 4 requires the
network profile.

First emitted by `Transport/NetworkObserver.cs`, fed from `Pipeline/OperatorEndpoint.cs`.

**Late-arrival (induced) loss** — samples the network delivered but the playout policy discarded
as too late to play in capture order. **This is not network loss and must never be added to it:**
network loss is what the link destroyed, this is what the buffer chose to drop, and the whole
latency/loss tradeoff is the second trading against `playout_delay_ms`. A policy reporting one
without the other is not evaluable (`Buffering/CLAUDE.md` requirement 1).

Metric name: `playout_late`, one sample of value 1 per discarded sample, stamped at its arrival.
A count rather than a rate, so a policy that discards nothing emits nothing rather than a stream
of zeroes; the rate is
`count(playout_late) / (count(playout_late) + count(playout_delay_ms))`, computed by the analyst.

**Playout underrun** — a drain that released nothing and had nothing to release: the pipeline asked
the policy for a sample and there was none to have. Distinct from late loss, which is a sample
arriving too late to use; an underrun is no sample at all. Both halves of that definition matter —
see `docs/adr/0012-playout-policy-wiring.md` §5 for the two simpler definitions that score one
baseline or the other at 100%.

Metric name: `playout_underrun`, one sample of value 1 per starved drain. Never silently zero on a
lossy trace: `IPlayoutPolicy` clause 3 requires this pushed to `IMetricSink`, because graceful
degradation is what the caller does with an underrun, not a reason to stop counting it.

**Goodput** — application-useful bytes/s, excluding redundancy and retransmission.

No metric name, and that is a statement about the system rather than an omission: nothing here
retransmits (`ITransport` callers are forbidden to retry on a refusal) and no shipped codec sends
redundancy, so there is nothing for the definition to exclude and goodput degenerates to
`count(net_<dir>_received) × <codec frame size> / <trial duration>`, derivable at analysis time
from metrics that already exist. It stops being degenerate the moment a redundant or
variable-length codec exists, and that change is where a real emitter belongs — measured as
delivered *useful* bytes, not as bytes on the wire, or the redundancy would count as goodput and
the metric would reward exactly what it is defined to exclude.

## 4. Prediction quality

Scored **counterfactually and offline**: at time *t* the predictor was asked to estimate state
at *t+Δ*; when ground truth for *t+Δ* appears in the recording, log the error. This is why any
predictor can be scored against a committed `.tlog` without a robot, a headset, or a network —
and why no scoring path may require live hardware.

**Position error** — Euclidean, mm, at horizons Δ ∈ {50, 100, 200, 400} ms.

**Orientation error** — geodesic angle between quaternions, degrees, same horizons.

**Velocity error** — mm/s. Included because a predictor can be positionally accurate while
badly wrong about direction of travel, which the reconciler then has to absorb.

**Failure rate** — fraction of predictions exceeding a stated gross-error threshold. Tail
behavior matters more than average accuracy; a predictor with a lower p50 and a worse p99 is
usually the worse choice.

**`prediction_position_error_mm`** / **`prediction_orientation_error_deg`** — metric names
emitted via `IMetricSink.Record`, mm and degrees respectively, first emitted by
`Teleop.Eval/Sweep/SweepCommand.cs`. `PoseMath.PositionErrorMeters`/`OrientationErrorRadians`
between the predictor's live estimate and the plant's simultaneous ground truth, stamped at that
instant. **This is a simplified online proxy, not the counterfactual, horizon-binned
methodology above** — it compares "now" to "now," not "a stale prediction for *t+Δ*" to "truth
that arrived at *t+Δ*," and reports no horizon breakdown. It exists because that fuller
mechanism needs an offline `.tlog` replay scorer that does not exist yet; this metric is what
makes Gate 5's "prediction error and correction cost reported together" requirement literally
true in the meantime, honestly labeled as a stand-in rather than silently presented as the real
thing. Replace it with the counterfactual scorer's output, don't add around it, once that scorer
exists.

**`joint_prediction_error_deg`** (docs/adr/0018): joint-space prediction error, in degrees,
for an `ICommandAwarePredictor`.
- **Definition:** the absolute difference between the predicted value of an arm joint for time
  *t* and that joint's value at *t*, pooled over the arm joints. Grippers are excluded, because
  they are fractions and open-loop.
- **Reporting:** percentiles (p50/p95/p99/max), never a mean.
- **In the benchmark:** `Teleop.Eval joint-predict-bench` computes it against a simulated servo
  arm's ground truth, at every 90 Hz display frame after a 1 s warm-up. That makes it a
  benchmark of the predictor under a stated simulation, not a measurement of the robot.
- **On real sessions:** the truth at *t* is the measured joint whose sensed time is *t*,
  interpolated between frames. It is computed offline from the `jetrover-joints-*.csv`
  recordings, never on the headset.

**`joint_correction_deg`** (docs/adr/0018): the joint-space correction cost of an
`ICommandAwarePredictor`, in degrees: how far the predicted arm jumps when a measurement arrives.
Report it alongside `joint_prediction_error_deg`, always (§8 rule 2).
- **Definition:** at each `ObserveState` that brings a new measurement, at the instant *t* it is
  observed, the absolute difference per arm joint between `TryPredict(t)` immediately before
  and immediately after the observation, pooled over the arm joints. Grippers are excluded, as
  above. "Brings a new measurement" means the frame's sensed stamp is later than that of every
  frame observed before it. Frames that repeat an earlier stamp are excluded, and so are
  out-of-order frames, which the predictor rejects. Both would only add zeros. A reading that
  repeats the previous values under a new stamp still counts, because the predictor re-anchors
  on it.
- **Reporting:** percentiles (p50/p95/p99/max), never a mean. It is a per-event magnitude, not a
  rate: the measured arm's update rate sets how often it happens.
- **Not the same as `correction_magnitude_*`** (§5): there is no reconciler in the joint-space
  path, so this is the jump the operator would see, undamped. It is the joint-space counterpart
  of predicted-at-capture minus authoritative, taken at the arrival instant rather than at the
  capture time.
- **In the benchmark:** `Teleop.Eval joint-predict-bench` reports it next to the error, after the
  same 1 s warm-up, under the same stated simulation.
- **On real sessions:** not yet computed. It needs both predictions at each arrival, so either a
  replay of the recorded commands and frames through the predictor, offline, or a headset log
  of both values.

**Amendment (2026-10-21, docs/adr/0021):** on recorded sessions (session v1, `trainer/contract/`) the
truth for `joint_prediction_error_deg` is each successful driver read, at the midpoint of its bus
transaction, and errors are scored only at those read instants; `joint_correction_deg` is computed by
replaying the session's view (the headset's calls) through the predictor, offline. The trainer's
evaluation (`trainer/armpred/evaluate.py`) computes every metric below this way. The trainer's numbers
are not results (ADR 0021 decision 8); they decide whether the C# consumer is built.

**Live variants on the headset (docs/adr/0023, 2026-10-26).** `Core/Metrics/LiveJointScorer` computes
`joint_prediction_error_deg`, `joint_correction_deg` and `joint_display_jitter_deg` on the headset, for
every enabled predictor at once, over a rolling 30 s window, and the console shows their p50/p95. The
definitions are the ones in this section; the read instant is a fresh frame's sensed stamp converted
by the link's `ClockSync`, so the truth is the frame itself rather than a driver read, and percentiles
are nearest-rank. They are diagnostics, **never results**: they are not recorded with a manifest, and
their clock-sync error is in them.

**`joint_display_jitter_deg`** (docs/adr/0021): how much the predicted arm shakes, frame to frame.
- **Definition:** per arm joint and 90 Hz display tick k, |p_k − 2 p_(k−1) + p_(k−2)| in degrees (the
  second difference of the prediction), pooled over the arm joints, over a stretch of display ticks.
- **Reporting:** percentiles. Compared against a baseline, never alone: a predictor that tracks a
  moving arm has some by construction.

**`joint_event_error_deg`** (docs/adr/0021): `joint_prediction_error_deg` aligned on an event.
- **Events:** a downlink switch (`network`), a slider change, a hand on or off, a payload, an uplink
  change (`uplink`), a mark. An `uplink-change` is an `uplink` event across which the uplink's one-way
  p50 delay moved by at least 50 ms (0.5 s after against 0.5 s before, from the view's true
  conditions).
- **t_info:** the first moment the headset's data could show the event. A downlink switch shows at
  once. For anything the robot side does, t_info is when the first frame sensed after it reaches the
  predictor.
- **Windows:** [t_e − 2 s, t_e) before the event; the information-limited stretch [t_e, t_info); then
  0–100, 100–250, 250–500, 500–1000 and 1000–2000 ms after t_info.
- **Forecast gate window (ADR 0021 decision 14):** [t_e, t_info + 1 s) after each `uplink-change`, pooled.
- **Reporting:** percentiles per window. Only predictions whose target time falls in the window count.

**`joint_recovery_time_ms`** (docs/adr/0021): from t_info until the 100 ms trailing median of the
error stays below 1.5° for 250 ms. Capped at 3000 ms, and a never-recovered event is reported at the
cap. Reporting: percentiles over events of one kind.

**`joint_prediction_lead_ms`** (docs/adr/0021 decision 14): how far ahead of the arm the prediction
runs. It measures directly the symptom that prompted that decision, a predicted arm that "goes
straight to the target".
- **Definition:** take the 90 Hz display ticks of a view, and the truth between driver reads,
  linear between them. A *motion segment* is a run of ticks where one arm joint's true speed exceeds
  0.3 rad/s for at least 0.5 s. For each segment, the lead is the shift τ in [−300, +300] ms, in
  5 ms steps, that minimises the median over the segment of |prediction(t) − truth(t + τ)|. Shifts
  that leave the truth's range for more than 20% of the segment are not considered.
- **Sign:** positive means the prediction shows where the arm will be (it runs ahead); negative means
  where it was (it lags).
- **Reporting:** p5, p50 and p95 over segments, and the share of segments at the ±300 ms edge (the
  search hit its bound, so the true lead may be larger).

**`joint_predictor_fallback`** (docs/adr/0021): for a learned joint predictor, the share of calls in
which it fell back to its prior: no fresh reading yet, the newest reading more than 1 s old, an output
that is not finite, or the backend failing. Reported as a count and a rate.

**`joint_inference_us`** (docs/adr/0021): a learned joint predictor's inference time per call, in
microseconds, with the device, the runtime and the thread count stated. Percentiles; the gate uses p99.

## 5. Correction cost

The counterweight to prediction accuracy, and the reason accuracy alone is never a result.

**Correction magnitude** — positional and angular distance between the predicted state and the
reconciled state at the moment truth arrives.

**Correction rate** — corrections per second exceeding a stated perceptual threshold.

**Peak jerk** — third derivative of displayed position, mm/s³. The nausea proxy. A system with
excellent prediction error and constant micro-snapping is worse for the operator than a
smoother, less accurate one.

**Time-to-convergence** — ms from correction onset until displayed state is within tolerance of
authoritative state. Bounded convergence is a requirement of `IReconciler`, not a nice-to-have.

Metric names emitted via `IMetricSink.Record`, first emitted by
`Reconciliation/SnapReconciler.cs`:

| Name | Unit | Emitted | Definition |
|---|---|---|---|
| `correction_magnitude_mm` | mm | on each authoritative sample that disagrees beyond tolerance, stamped at that sample's `t_capture` | `PoseMath.PositionErrorMeters(predictedAtCapture, authoritative)` × 1000 |
| `correction_magnitude_deg` | degrees | as above | `PoseMath.OrientationErrorRadians(predictedAtCapture, authoritative)` in degrees |
| `time_to_convergence_ms` | ms | on the frame a correction completes, stamped at that frame | onset frame to the frame the displayed state is within tolerance; `snap` always reports 0 |
| `jerk_mm_s3` | mm/s³ | on **every** frame that advances the displayed state, stamped at that frame | magnitude of the third derivative of displayed position, from a cascade of central differences over the four most recent displayed positions; not emitted until four exist. Every frame, not every correction: jerk is a property of the displayed trajectory, and emitting it per correction event gave a one-frame reconciler (`snap`) one sample where a smoothed one gave ~100, so pooled percentiles compared different populations instead of different reconcilers. All reconcilers share one estimator (`Reconciliation/DisplayedJerkEstimator.cs`) |

**Correction rate** is derived at analysis time by counting `correction_magnitude_mm` samples
above the stated perceptual threshold per second — it is not a separately emitted metric, so
that "corrections per second" and "correction magnitude" can never disagree about what counted
as a correction.

## 6. Task performance

Measured on the frozen benchmark tasks (Fitts reciprocal tapping, peg-in-hole, pick-and-place,
moving-target tracking).

**Completion time** — s, per trial.

**Path efficiency** — actual path length ÷ straight-line distance.

**Error events** — collisions, drops, missed targets; counted, not scored into a composite.
Composites hide which failure mode changed.

**Fitts throughput** — bits/s, `ID / MT` where `ID = log₂(2A/W)`. One scalar, comparable across
conditions and directly comparable to the human-factors literature.

## 7. Subjective

Required for any human-facing claim; a latency improvement that nobody perceives is not a
usability result.

- **NASA-TLX** — workload, six subscales, administered per condition.
- **Simulator Sickness Questionnaire (SSQ)** — administered before and after each condition per
  the standard protocol. Nausea, oculomotor, and disorientation subscales reported separately.
- **Presence** — a single validated instrument, chosen once and kept.

Condition order must be counterbalanced, and SSQ requires a washout between conditions.
Deviating from the standard protocol makes the scores unpublishable, so decide the protocol
before collecting anything.

## 8. Reporting rules

1. Always report the baseline (`none` predictor, `snap` reconciler) in every comparison, even
   when it obviously loses. It is what makes the other numbers interpretable.
2. Always report prediction error and correction cost **together**.
3. Never declare a winner from a single seed, and always state the observed seed spread. A
   difference inside run-to-run variance is not a result.
4. State the network profile in every figure caption. A result without its profile is
   meaningless.
5. Report negative and inconclusive results. They are what stop an idea from being retried
   indefinitely — record them in the relevant folder's `CLAUDE.md` "Tried and rejected"
   section with a link to the results directory.

## 9. Camera

The robot's camera frames (docs/adr/0014-camera-frame-downlink.md). Emitted by
`Camera/CameraLatencyRecorder.cs`, once per frame that reaches the screen, every sample stamped at
that frame's `t_render` (operator domain). The host supplies the stamps and the pose path's
`ClockSync`; the recorder never updates the estimate, because a one-way video stream has no round
trips of its own (ADR 0014 §5).

Per-frame stamps, in order:

| Stamp | Clock | Meaning |
|---|---|---|
| `t_capture` | robot | the camera driver's buffer timestamp, CLOCK_MONOTONIC, taken at **start of exposure** on the JetRover's camera (ADR 0014 §4). Unset when the driver gave none, never substituted |
| `t_send` | robot | the frame's first chunk handed to the socket; every chunk carries the same value |
| `t_recv_first`, `t_recv` | operator | first and last chunk dequeued from the socket, on the network thread (§1) |
| `t_decode_start`, `t_decode_end` | operator | the host's JPEG decode |
| `t_render` | operator | host-defined, as for M2P: in Unity, `Application.onBeforeRender` of the first frame drawn after the decoded texture was uploaded |

| Metric | Unit | Definition | Needs `ClockSync` |
|---|---|---|---|
| `camera_capture_to_send_ms` | ms | `t_send − t_capture`, both robot domain. The camera's exposure, readout and USB transfer, plus the sender's own handling. Not emitted when `t_capture` is unset | no |
| `camera_owd_ms` | ms | `t_recv − t_send`, `t_send` converted to the operator's clock. The whole frame's transfer, first chunk sent to last chunk received | yes |
| `camera_reassembly_ms` | ms | `t_recv − t_recv_first`. **Part of `camera_owd_ms`, not added to it**: the spread of one frame's chunks on arrival | no |
| `camera_handoff_ms` | ms | `t_decode_start − t_recv`: waiting for the display thread | no |
| `camera_decode_ms` | ms | `t_decode_end − t_decode_start`. In Unity this is `Texture2D.LoadImage` on the main thread, and it competes with the 11.1 ms frame budget | no |
| `camera_decode_to_render_ms` | ms | `t_render − t_decode_end` | no |
| `camera_capture_to_render_ms` | ms | `t_render − t_capture`, `t_capture` converted to the operator's clock. **The headline camera figure**, the video counterpart of M2P without `DisplayOffset` | yes, and `t_capture` set |
| `camera_frame_dropped` | count | one sample of value 1 per frame that completed nowhere on screen: given up as incomplete, superseded before it was taken, or replaced before the display thread decoded it. As with `latency_trace_evicted`, the denominator correction for everything above | no |

Rules specific to these:

- **The stages sum exactly.** `capture_to_send + owd + handoff + decode + decode_to_render =
  capture_to_render`, to within one operator tick of conversion rounding, because each stage is the
  difference of adjacent stamps. This is the §2 stage-breakdown requirement met by construction;
  a frame whose stamps are out of order records nothing at all rather than a negative stage.
- **Cross-clock metrics exist only while `ClockSyncDiagnostics.IsSynced`.** A session that only
  watches the camera, with no pose traffic, has `camera_owd_ms` and `camera_capture_to_render_ms`
  missing by design, not by fault. Their accuracy is the sync estimate's: report its uncertainty
  alongside them.
- **`camera_capture_to_render_ms` gets one physical validation before it is cited** (ADR 0014 §9):
  the camera films a millisecond counter drawn by the operator's display, and the photographed
  difference is compared with the software figure, as M2P is validated against a photodiode
  (ADR 0003).
- **The capture stamp is start of exposure.** On the JetRover's camera `camera_capture_to_send_ms`
  is about one frame period (32 ms at 30 fps, 68 ms at 15 fps, measured by
  `just camera-timestamp-spike` on 2026-10-01) before the network is involved at all.
