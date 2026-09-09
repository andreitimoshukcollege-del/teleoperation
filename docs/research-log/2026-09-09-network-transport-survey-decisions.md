# 2026-09-09 — decision record: treating the network as smarter than a dumb pipe

**Organizer:** research-organizer. **Run type:** survey and feasibility assessment, by explicit
instruction. Nothing was built: no implementation, no test, no registry entry, no experiment YAML,
no sweep, no `results/` directory. **HEAD for the whole run:** `06c0ab5` (`main`); all four
researcher worktrees branched from it and each reported the sha back independently.

## The question

"How do we improve effective latency by treating the network as something smarter than a dumb
pipe?" The user named two directions — the way packets are sent, and predicting network conditions
rather than absorbing them — and imposed the framing that decided the run: **nothing in Core can
reduce one-way delay.** OWD is the network's. What these approaches can change is the *impact* of
loss and jitter. Any candidate whose pitch was "reduces latency" without naming an existing metric
and a direction was not a candidate.

The deliverable was a ranked set of candidate directions with what each would actually cost here,
so that a later run builds the winner rather than infrastructure for the wrong axis.

## How it was decomposed, and why

Four candidates, one researcher each, isolated worktrees, all four given an identical file scope
(their own log and nothing else) and the same set of known constraints and known artifacts.

The cut is along **what a mechanism actually manipulates**, because that is what determines which
contract it touches and therefore what it costs:

1. **Payload redundancy / FEC** — what goes inside a datagram, fixed policy, one path.
2. **Intent transmission** — the *semantics* of the payload: send a plan, not a point.
3. **Network-condition estimation and adaptation** — measure or forecast the link and change
   behaviour in response.
4. **Path diversity and send-time scheduling** — how and where datagrams are put on the wire.

Each pair is distinguishable by a measurement named in advance: 1 versus 2 by whether coverage of
a gap is backward-looking or forward-looking; 1 versus 3 by whether the policy is fixed or
adapts, which shows up as bandwidth cost under time-varying loss; 4 versus everything by whether a
second path exists. Explicit scope handoffs were written into each brief to stop 1 and 4 from both
assessing packet duplication, and 1 and 3 from both assessing redundancy depth.

I front-loaded the feasibility question over the mechanism question in every brief, because for
this axis the mechanisms are all well understood outside the repo and the open question was always
whether this codebase can express or observe them.

## Approaches considered and not pursued

- **A jitter-buffer / playout candidate in its own right.** The obvious fifth. Not given its own
  researcher because it is the Buffering axis, not Transport, and because it is the *consumer* half
  of candidate 3 — assessing it separately would have produced two agents reading the same
  contract and reaching the same desk. Folding it into candidate 3 as "the consumer" was the
  cheaper equivalent, and it was the right call: candidate 3's central finding is precisely that
  the candidate collapses onto this consumer.
- **Retransmission / ARQ.** Deliberately excluded before decomposition. Under a hard per-frame
  latency budget at 100 Hz, a retransmission arrives after the sample it replaces is worthless, and
  a newer sample supersedes it in any case. It is the textbook wrong answer for real-time control
  and did not need a window spent on it.
- **Application-layer compression of the pose payload.** 73 bytes at 100 Hz is ~58 kbit/s.
  Bandwidth is not scarce anywhere in this system, so compression buys nothing measurable and
  costs CPU on the hot path. Recorded so it is not proposed as a "network improvement" later.
- **Congestion control proper (GCC/BBR-style rate adaptation).** Considered as a candidate of its
  own and folded into candidate 3 instead, on the suspicion that an inelastic fixed-rate sender has
  no rate to adapt. Candidate 3 confirmed this and went further than suspected — see its section 3.
- **Changing `EmulatedTransport` to model a queue** so that send-rate control would have something
  to act on. Rejected as out of scope and as a baseline change: it would alter every recorded
  result's meaning. It is a real requirement if anyone ever wants sender-side congestion work, and
  is recorded as such rather than done.
- **Splitting candidate 4 into "multipath" and "pacing".** Rejected during decomposition: pacing
  was expected to be a one-paragraph negative for a fixed-cadence sender, which is not a
  researcher's worth of work. Bundling them and instructing the researcher to state the negative
  crisply and early worked — that is exactly what came back.
- **Running the judge panel.** Rejected, and worth recording because it is the pipeline's normal
  next step. `judge-admissibility` judges an implementation against its contract,
  `judge-methodology` judges a sweep, and `judge-verdict` ranks candidates on numbers. There is no
  implementation and no sweep in this run, so all three would have been asked to score nothing. A
  judge with nothing to judge emits noise that later readers mistake for evidence. The user
  instructed this and I agree with the reasoning.
- **Fixing the sweep's inability to vary the codec.** Explicitly reserved by the user as their own
  work once an axis wins. Every researcher was told to price it and forbidden to do it. Correctly
  observed by all four.

## Verdicts

Nothing was built, by design. All four were asked for an argument; three delivered complete ones
and one was truncated by an agent failure.

- **Payload redundancy / FEC — rejected on argument, and the rejection is structural.** Not "small
  benefit" but **provable no-op**: `ICommandCodec.TryDecode` returns only the newest frame,
  `RigidBodyPlant.Command` rejects any frame whose `CaptureTicks` is not newer than the last
  accepted, and `Command` is a pure assignment, so replaying older recovered frames cannot change
  the plant's trajectory. I verified all three against the source rather than take them on report.
  A `redundant-N` codec would produce bit-identical output to `raw` at every N on every profile.
  Detail: `2026-09-09-fec-redundancy-feasibility.md`.
- **Intent transmission — rejected on argument, with a redirect.** First-order intent already has a
  consumer by accident (`RigidBodyPlant` coasts on commanded velocity through gaps); beyond first
  order `TryDecode` returns a single `CommandFrame` with no acceleration, knots or horizon and
  takes no time parameter, so a trajectory codec and `raw` are the same decoder by construction —
  the `ekf` failure shape, one level worse. The researcher's own arithmetic put the candidate's
  incremental value at ~0.28 mm at the mean burst against a 5 mm convergence tolerance, one to two
  orders below the harness defect it would hide behind. Its redirect — robot-side differencing of
  the received pose stream, which needs no wire change and belongs to the Prediction axis — is the
  more interesting experiment. Detail: `2026-09-09-intent-transmission-feasibility.md`.
- **Path diversity and send-time scheduling — rejected on argument; most expensive, least ready.**
  The pleasant surprise is that the mechanism is cheap: a multipath decorator over two
  `ITransport`s satisfies the existing contract unchanged, and headless two-path emulation is
  possible today with no new profile. The killer is analytic. Jitter is bounded uniform by design,
  so min-of-two-paths improves p50 by 0.414·J and p99 by only 0.180·J — the benefit *shrinks* into
  the tail, the opposite of the hedged-request literature, because there is no tail to crush. Every
  pair of frozen presets has disjoint delay support, degenerating to "use the faster path". The
  real effect is on loss, and it is uninstrumented. Detail:
  `2026-09-09-path-diversity-feasibility.md`.
- **Network-condition estimation — complete, and the verdict is "yes, but not here and not yet".**
  Two independent assessments exist for this one candidate (see "Course changes"): a complete one at
  `2026-09-09-network-condition-estimation-feasibility-first-run.md` and a truncated one at
  `2026-09-09-network-condition-estimation-feasibility.md`. They were produced without knowledge of
  each other and agree on both determinations that matter.** All four falsifiers the researcher stated up front are reported fired. Its contract
  determination is unambiguous and is the most useful output of the run: this is an implementation
  of the *existing* `IPlayoutPolicy`, not a new `Contracts/` interface — the estimator is a private
  collaborator of a policy, and `PlayoutPolicyConfig`/`PlayoutPolicyDiagnostics` already carry its
  parameters and its output surface. The ADR that is needed is about *wiring*
  (`OperatorEndpoint` gaining a two-phase receive), not about a new abstraction. It also answered
  the inelastic-sender challenge with a clean yes: only the receive side has a knob, and a
  send-rate controller would be a no-op *by construction* because the emulator has no queue model.
  Bottom line from both: **not yet, and not in `Transport/` — on this system "predict the network"
  reduces to "schedule playout better", which makes it a `Buffering/` candidate.** Neither
  recommends building an estimator: delay on every parametric profile is bounded uniform, so the
  optimal budget is the known constant `Base + J` and there is nothing to estimate; the only
  persistent-structure profile yields two events per trial at n = 1; the only advertised periodic
  one does not exist. Four ADR decisions are named, and a next step cheaper than any of them.

No ranking is claimed on measurement, because nothing was measured. The ordering of work below is
an argument, not a result.

## Course changes mid-run

- **I twice misjudged a live researcher as dead, and removed both their worktrees while they were
  still running. This is the run's worst process failure and it was mine.** Candidate 3's first
  attempt went 90 minutes with a silent transcript and only a scratch file on disk, so I concluded
  it had stalled and relaunched it. The relaunch stopped growing its log mid-structure and went
  silent for 18 minutes, so I concluded that had stalled too, copied what it had written, and ran
  end-of-run cleanup. **Both agents were alive.** Both then reported that their worktree had been
  deleted out from under them. Diagnosis error: I treated transcript staleness as death, when these
  agents were simply slow — one ran nearly eight hours. Staleness is not a liveness signal, and I
  had no reliable one.

  **Nothing was ultimately lost, by luck rather than judgement.** The first agent's entire
  assessment came back in its return message and I reconstructed its log from that; the second had
  rewritten its log in full into the leftover directory and the file was recoverable there. Had
  either detail gone differently, a complete assessment would have been destroyed permanently,
  because worktree contents are gitignored. The standing rule — *copy logs out before removing
  anything* — is not sufficient on its own; the missing rule is **do not remove a worktree until
  its agent has actually reported completion**, regardless of how dead it looks.

- **The relaunch produced a second, independent assessment of the same candidate, and I kept both.**
  They were written without knowledge of each other and agree on the contract determination, the
  inelastic-sender answer, and the forecasting-versus-percentile-tracking negative — reached from
  different literature. That agreement is worth more than a single assessment would have been, so
  reconciling them into one file would have destroyed evidence. They disagree on whether the
  in-flight ring's 64-deep window is actually exceeded in practice; that disagreement is preserved
  and flagged rather than settled, because settling it needs a measurement.

- **The lesson that does generalize:** briefing a long-running researcher to *create its log
  immediately and append continuously* worked exactly as intended. The relaunch had a substantial
  log on disk long before it finished, which is the only reason I had anything at all during the
  window I believed it was dead. Keep that instruction in every future brief.
- **I verified researcher claims against the source rather than relaying them.** Specifically the
  plant's staleness rejection and payload-length-independent loss draw (candidate 1), the plant's
  coast-on-velocity gap policy (candidate 2), `synthetic-burst`'s zero loss and the Bernoulli shape
  of the whole `loss-<N>pct` family (candidate 2), and the in-flight ring overwrite plus
  `leo-satellite`'s non-existence (candidate 3). All held. This was not scepticism about any
  particular agent; the run's whole recommendation rests on a handful of structural facts and a
  wrong one would have inverted it.

## What was left open, and what is blocked on a human

- **`synthetic-burst` is not periodic, and its seeds are inert.** Its bursts are triggered by a
  memoryless Bernoulli draw (`SyntheticTraceBuilder.BurstStartProbabilityPerSample = 0.01`), so
  onset is unforecastable by construction — yet both `docs/adr/0004` and the generator's own XML doc
  call the result "periodic congestion bursts". That word is load-bearing in the wrong direction:
  it is what makes the forecasting pitch sound plausible. Separately, the trace profile zeroes base
  delay, jitter, loss and reorder, so the RNG has no outcome to affect and all five seeds produce
  byte-identical trials; `docs/metrics.md` section 8 rule 3's seed-spread requirement is
  unsatisfiable there, and a 500-step trial sees only two burst episodes. Correcting the wording is
  a human's job (an ADR edit); deciding whether to authorise a genuinely forecastable trace family
  is a new ADR.
- **The playout operating point cannot be reported.** `PlayoutPolicyDiagnostics` is a struct
  property and never reaches `IMetricSink`, and none of its fields are defined in `docs/metrics.md`.
  `IPlayoutPolicy`'s own doc says a policy that does not report its operating point "is not
  evaluable" — which is currently true of any policy anyone could write. Worse, `owd_*` is
  `t_recv - t_send` and a buffer acts after `t_recv`, so **a buffer's entire latency cost is
  invisible to every currently emitted metric.** A buffering win would read as free. Metric
  definitions are a human's call.
- **`Registries.PlayoutPolicies`' factory lacks an `ITimeAuthority`** that
  `PlayoutPolicyConfig.MaxAdaptationRatePerSecond` (specified per second of wall time) requires.
  `Predictors` takes `(config, clock)` and `Reconcilers` `(config, sink, clock)`, so precedent
  favours adding it — but that edits a named shared-conflict file and should ride with the ADR.
- **A lead, not a result: `double-exp` may be discarding a third of its input.** Reordering is
  pervasive even though `reorderProbability` is 0.0 everywhere, because the emulator delivers by
  earliest synthetic arrival; `DoubleExponentialPredictor.Observe` rejects every out-of-order
  sample outright while `const-vel` splices them in. The researcher reports a matching signature in
  existing data (best p50, catastrophic p95) but flags its own analysis as resting on reconstructed
  capture ticks and a different sha. Worth confirming, and it belongs to the Prediction axis.

- **A harness defect that affects results already recorded.** `OperatorEndpoint.InsertInFlight`
  overwrites an occupied ring slot without checking, and a state frame whose trace has been evicted
  is dropped *before* `ObserveRobotState`, so it never reaches the predictor or reconciler. At
  `InFlightCapacity = 64` and 10 ms steps that is a 640 ms window against a 480-720 ms round trip on
  `300ms-60j-2loss-bursty`. This is not merely a metrics artifact — it changes what the algorithms
  are fed, on the hardest profile, and it is delay-correlated so it censors the slowest trips. The
  fix is cheap; deciding what it does to comparability with every existing `results/` directory is
  a human's call, not an agent's. **This is the single most consequential finding of the run and it
  belongs to no candidate's axis.**
- **`docs/metrics.md` §3 is entirely uninstrumented.** Loss rate, burst-length distribution, jitter,
  reordering rate and goodput are all defined and emitted by nothing. Every candidate in this run
  had its benefit or its cost, and usually both, land in §3. This is the gating infrastructure for
  the whole Transport axis.
- **No metric for displayed-pose accuracy or command-tracking error.** Three of four candidates can
  improve smoothness while degrading correctness, invisibly. Adding one is a `docs/metrics.md`
  change and should not re-scope `prediction_position_error_mm`.
- **The harness has never exercised intent.** `SweepCommand` passes `Vector3.Zero` for both velocity
  fields, holding `RigidBodyPlant` in exactly the mode its own doc comment says "would understate
  what packet loss looks like to an operator". Motion is a smooth analytic sinusoid with identity
  rotation, so `prediction_orientation_error_deg` is identically zero in every run recorded.
  Fixing this is a baseline change and must be a new dimension, not an edit.
- **One bursty-loss profile in the whole suite.** `300ms-60j-2loss-bursty` is the only one;
  `150ms-20j-0.5loss` and the entire `loss-<N>pct` and `combo__` families are memoryless, and
  `synthetic-burst` has zero loss (its burst is a *delay* burst — the name misleads and the axis
  doc should say so). Any loss-burst research here is a single-point study at ~3 events per trial.
  Widening the suite needs an ADR.
- **Axis `CLAUDE.md` rows that should move.** `Transport/CLAUDE.md`'s "Tried and rejected" section
  is `(none yet)` and should gain `redundant` and `trajectory` rows linking the two logs, marked
  **rejected on argument, not on measurement** — a distinction that matters here because no
  `results/` directory exists to point at. I did not make these edits: shared files are outside
  researcher scope by design, and the user owns the axis doc.
- **`AuditCommand` excludes `Transports` from registry-completeness checking**, so a missing
  transport entry would not be caught by the gate. Noted by candidate 4, unowned.
