# Metrics

Every metric this project reports is defined here. **If it isn't defined here, it isn't a
metric** — add the definition in the same PR that emits it. Undefined metrics are how a
research project ends up with numbers nobody can interpret six months later.

Conventions used throughout:

- Time in **milliseconds**, on the synced clock (`Time/ClockSync.cs`). Never a local clock.
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

**Motion-to-photon (M2P)** — `t_photon(displayed) − t_capture(operator motion)`. The headline
number. Validate the software estimate against a physical rig at least once — LED plus
photodiode, or a high-speed camera on a spinning marker — then trust the software estimate and
re-validate whenever the render path changes.

**Command-to-actuation (C2A)** — `t_actuation − t_capture` on the operator input that caused
it. The other headline number; separate from M2P because they can be improved independently.

**Stage breakdown** — capture, encode, transit, buffer, decode, render, display. Must sum to
M2P within tolerance; if it doesn't, a stage is unaccounted for. Report as a stacked
contribution, because the point of the breakdown is deciding where optimization is worth doing.

## 3. Network

**Jitter** — report both, they answer different questions:
- RFC 3550 interarrival jitter (smoothed; comparable to the streaming literature)
- IQR of one-way delay (distribution-level; what the playout policy actually has to absorb)

**Loss rate** — fraction of sent datagrams never received.

**Loss burst-length distribution** — histogram of consecutive-loss run lengths. Report this
alongside the rate always. A 2% loss rate in bursts of 20 and a 2% rate of isolated drops
break a jitter buffer in completely different ways, and the rate alone cannot distinguish them.

**Reordering rate** — fraction arriving out of sequence, plus max displacement.

**Goodput** — application-useful bytes/s, excluding redundancy and retransmission.

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
