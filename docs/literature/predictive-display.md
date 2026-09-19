# Predictive display and classical delay compensation

Field survey: classical and control-theoretic predictive display, and the delay-compensation
families that grew up around it — predictor displays, phantom-robot graphics, model-mediated
teleoperation, wave variables and scattering, time-domain passivity, Smith predictors, and the
passivity-versus-transparency argument.

Written 2026-09-15. Every source below was fetched at the URL given on that date; the handful that
could not be fetched are in §9 with what happened, and are not used as evidence anywhere.

**Revised 2026-09-16 by a second, bounded pass whose only job was to close the retrieval gaps in
§9.** Nothing was rewritten or removed; entries were added and status records were filled in.
Sources carrying `(accessed 2026-09-16)` were fetched on that date. What the second pass recovered:
the 1990 phantom-robot paper and the 1993 JPL-to-Goddard demonstration in §6.2, both from NASA
rather than IEEE; Azuma & Bishop's SIGGRAPH '95 analysis in §8, from the author's own site; the
model-jump chapter and four further model-mediation sources in §6.4; and the CHI 2026 telerobotics
visualisation study in §6.2. What it did not: Hokayem & Spong's historical survey, and Mitra, Gentry
& Niemeyer's 2007 user study of how to render a model jump — which is, on everything else this file
now knows, the most on-axis missing source in the field. Routes are in §9.1. Per
`docs/literature/CLAUDE.md` this file may say *prior work reports X under conditions Y* and may not
say *therefore our Z is wrong*; nothing here settles a comparison in this repo, and no number here
belongs in `results/`.

Two scope notes. **This project has no haptic loop and no force feedback** — §5 is the explicit
accounting of which of these families survive that fact. And **the field boundary is soft on one
side**: distributed-simulation dead reckoning and game netcode (§8) are another file's territory,
but they turn out to own the vocabulary this project's Reconciliation axis has been missing, so
five sources from there are included and flagged as out-of-field.

---

## 1. The question, answered directly

**Is "operator sees a locally extrapolated proxy of remote state, corrections blended in when truth
arrives" already a named thing?**

**Yes for the display architecture, no for the blend.** The architecture has a name, a 60-year
lineage and a canonical demo. The blending step — the part this repo calls reconciliation and has
five implementations of — is *not* named in the classical predictive-display literature, and the
reason it is not named is structural rather than accidental (§2). It is named twice elsewhere:
"model jump" in model-mediated teleoperation, and "convergence"/"smoothing" in distributed
interactive simulation.

**The name is predictive display** (equivalently *predictor display*; *phantom robot* for the
robot-arm form; *predictive graphics simulation* in the space-robotics dialect). The lineage:
Ferrell's 1962-65 delay experiments established that an operator under transmission delay degenerates
into move-and-wait (<https://ntrs.nasa.gov/citations/19650052768>); Ferrell and Sheridan's response
was supervisory control, and Sheridan's 1992 book is the standing reference for that whole programme
(<https://archive.org/details/teleroboticsauto0000sher>). The predictive-display form was made
concrete by Bejczy, Kim and Venema at JPL as the **phantom robot**: a real-time graphics model of the
arm, driven by the operator's commands with no delay, overlaid on the delayed camera view, so the
phantom "predicts" and the real image catches up later
(<https://ntrs.nasa.gov/citations/19920000396>). DLR flew the same idea on ROTEX in 1993 against a
5-7 second round trip (<https://www.dlr.de/en/rm/research/robotic-systems/hands/rotex-1988-1993>).
Modern instances are everywhere: stereo AR overlays on the da Vinci
(<https://arxiv.org/abs/1809.08627>), vehicle teleoperation
(<https://arxiv.org/abs/2211.11918>), digital-twin telesurgery
(<https://pmc.ncbi.nlm.nih.gov/articles/PMC12864800/>), and a 2026 benchmark that applies the same
term to generative video models (<https://arxiv.org/abs/2605.09670>).

**So this repo did not invent an architecture. It did, however, build a variant that the classical
line does not contain**, and the difference is worth stating precisely because it explains the
missing vocabulary:

| | Classical predictive display | This repo |
|---|---|---|
| What drives the local proxy | the **operator's own commands**, pushed through a forward model of the remote robot | **extrapolation of received remote telemetry** to "now" (`IPredictor.Predict(targetTicks)`) |
| Where truth appears | a **second, separate visual object** — the delayed camera image, shown alongside the graphics overlay | the **same** displayed pose; there is one thing on screen |
| Who reconciles | the **operator**, perceptually, by watching the real arm converge onto where the phantom was | the **system**, by a reconciler policy (`snap`, `spring`, `budget-blend`, ...) |
| What is measured | overlay/registration accuracy in mm, task time, workload | prediction error *and* correction cost (magnitude, jerk, time-to-convergence) |

That third row is the whole finding. In the phantom-robot architecture there is nothing to blend,
because the prediction and the truth are rendered as two different things and the human does the
data fusion. The repo's choice to display **one** pose is what creates the reconciliation problem —
and it is the choice that VR forces, because you cannot show an operator a ghost arm and a real arm
in a head-mounted display without paying for it in exactly the currency (comfort) the project is
trying to buy. The nearest classical relative of the repo's *mechanism* is therefore not the phantom
robot at all; it is dead reckoning with a convergence algorithm (§8), which is a distributed-
simulation technique, wearing a predictive-display architecture.

**Where this leaves the repo's naming:** "predictive display" appears zero times in this repository
(verified 2026-09-15 across docs, ADRs, logs, sources and comments). The `Prediction/` axis is doing
predictive display and the `Reconciliation/` axis is doing what MMT calls model updating; neither
says so. §10 argues for what to do about that, since this file cannot change other files.

---

## 2. Does the classical literature name — and measure — the reconciliation step?

**Name: not in the predictive-display line.** I searched for it directly (§7 records the queries).
What the predictive-display corpus discusses instead is *registration* — how well the graphics model
sits on top of the camera image — which is the same quantity evaluated **statically**, not the
dynamics of absorbing a discrepancy. Kim and Bejczy's 1993 JPL-to-Goddard demonstration is the
canonical measured example of that (**retrieved in full on the second pass, 2026-09-16; §6.2**). The modern digital-twin telesurgery
paper reports the same class of number: maximum spatial registration error 7.83 mm over the full
workspace, under 2 mm within a 4 cm radius of the working point
(<https://pmc.ncbi.nlm.nih.gov/articles/PMC12864800/>). That is the analogue of
`correction_magnitude_mm` — an error magnitude — and there is **no** analogue of `jerk_mm_s3` or
`time_to_convergence_ms` anywhere in the retrieved predictive-display corpus.

**Update, 2026-09-16, after recovering the three primaries.** Two things change and one does not.
*Changed, on the measurement question:* Kim and Bejczy's overlay accuracy is now readable at the
source and it is a full error budget, not a single figure — the headline "+/-5 mm" holds, at a
zoomed-in camera setting, against a +/-12 mm task margin, and the underlying registration residuals
run 0.5-1.6% of the image plane depending on field of view, which is 0.2 cm at 1 m zoomed in and
2.5 cm against a wide-angle side camera (§6.2). It is still a *static* quantity. *Changed, on the
naming question:* the reconciliation step **is** named, measured and parameterised in the
model-mediated line, more concretely than the first pass could establish — Willaert, Van Brussel and
Niemeyer implement a gradual constant-time model jump at t_move = 500 ms in and t_fade = 300 ms out,
behind a 4 mm trigger deadband, and Mitra, Gentry and Niemeyer ran a human-subjects comparison of
jump-rendering methods in 2007 (§6.4). *Unchanged:* none of them reports a **distribution**. There
is still no percentile, no jerk statistic and no time-to-convergence anywhere in this corpus, and
the 2007 preference study — the one source that compared correction policies against each other —
could not be retrieved (§9).

**Name: yes, twice, outside that line.**

- **Model-mediated teleoperation calls it the "model jump".** When the local environment model is
  replaced by a newly estimated one, the abrupt parameter change produces a discontinuity in what the
  operator feels and how the system behaves. The remedies described in that literature are recognisably
  the same family as this repo's reconcilers: *gradual update* over a fixed rate or a fixed time
  period, passivity-constrained updating, and constant-time introduction/removal of objects
  (<https://scientiairanica.sharif.edu/article_20007.html>,
  <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.611251/full>).
  A "fixed time period" gradual update is `budget-blend`. A passivity-constrained one is the thing
  `spring` approximates without the energy argument.
- **Time-domain passivity calls the accumulated version "position drift"**, and there is explicit work
  on making the correction *smoother*: Coelho et al. criticise existing drift compensators for either
  over-constraining feedback or adding "high impulse-like force signals", and propose compensation
  "in a smoother way" (<https://arxiv.org/abs/2002.02296>). That is the same objection this repo's
  axis makes against `snap`, in a force channel instead of a visual one.
- **Distributed interactive simulation calls it convergence/smoothing** (§8), and is the only body of
  work retrieved that states the tradeoff between how hard you extrapolate and how visible the
  correction is, in as many words.

**Measure: essentially no, not as a distribution.** MMT and TDPA measure force amplitude, energy and
passivity violation; DIS measures update rate against threshold; predictive-display studies measure
task time, error rate, workload and registration accuracy. Nothing retrieved reports percentiles of a
corrected display's smoothness. On the retrieved evidence, the repo's insistence on reporting
correction cost as a percentile distribution alongside prediction error is **not** a standard
instrument in this field — which makes it a contribution rather than a reinvention, and also means
there is no external number to check the implementations against.

---

## 3. "Predicting further necessarily costs more correction" — established, refuted, or unaddressed?

Split it in two, because the two halves have very different support.

**Half one — error grows with horizon: well established, including in this system's own regime.**
The 2026 generative predictive-display benchmark evaluates "temporal error evolution across the
prediction horizon" and reports that no tested model achieves non-divergent per-step error, with some
models' error increasing sharply and others drifting steadily
(<https://arxiv.org/abs/2605.09670>). The head-tracking literature on which VR prediction rests says
the same about the operating point: prediction requires the lag to be "small and consistent"
(<https://userpages.cs.umbc.edu/olano/papers/latency/>), and Kalman head-pose prediction work chooses
60 ms look-ahead as the design point (<https://arxiv.org/abs/2007.14084>). Azuma and Bishop's
frequency-domain analysis is the canonical statement that error grows rapidly with prediction
interval; the first pass could not retrieve it, **the second pass did — from the first author's own
site, in full (§8)** — so its numbers can now be quoted, with the domain gap stated. They are:
error growth "roughly the square of the prediction interval and the frequency" for a second-order
polynomial predictor with perfect noise-free inputs; prediction interval and frequency entering the
transfer function only as the product, so that "bandwidth times the prediction interval yields a
constant performance level"; and, measured on a recorded HMD motion sequence through a simulated
6-D Kalman predictor, average screen-space errors **2.3x larger at 100 ms than at 50 ms, and 9x
larger at 200 ms than at 50 ms**. Head motion in a 1995 HMD, not a robot arm over a network — but
it is a measured error-versus-horizon curve landing squarely on this project's horizon values, and
its shape is superlinear with the knee between 100 and 200 ms.

**Half two — the *coupling* to correction cost: not established here, and only qualitatively stated
anywhere I found.** The single clearest statement is out-of-field and 29 years old: "Large dead
reckoning thresholds can result in noticeable jerkiness of motion when new PDUs are received. On the
other hand, small dead reckoning thresholds force more PDUs to be sent."
(<https://www.gamedeveloper.com/programming/dead-reckoning-latency-hiding-for-networked-games>).
Note carefully what that is and is not: it is a **threshold**-versus-visible-correction tradeoff, not
a **horizon**-versus-correction tradeoff. Extrapolating further ahead and tolerating more error before
publishing an update are different knobs that happen to push the same quantity.

I found **no** source that reports prediction error and correction cost jointly, as a curve over
horizon, for a visual-only teleoperated system. The queries are in §7. So the project's founding
premise is, against this corpus: *plausible, partly supported on the error half, and unmeasured on
the coupling that actually matters.* That is a good position to be in — the experiment that would
settle it is cheap (sweep horizon at 50/100/200/400 ms on a fixed reconciler, report
`correction_magnitude_mm` and `jerk_mm_s3` percentiles against prediction error), it is already
inside the Prediction axis's stated benchmark requirement 6, and nobody retrieved here has run it.

**Update, 2026-09-16.** The second pass moved half two by one notch, from "only qualitatively
stated anywhere, and only out-of-field" to "qualitatively stated *in* field, by a primary". Willaert,
Van Brussel and Niemeyer write that "longer lags between master and slave will enlarge the size of
necessary model jumps and increase the need for explicit handling and containment" (§6.4). That is
delay-versus-correction-magnitude asserted inside delayed teleoperation rather than borrowed from
game netcode, and it is about **lag**, which is the knob this project sweeps, rather than about a
publication **threshold**, which is what the dead-reckoning statement in §8 is actually about. It
is still an assertion: no curve, no distribution, no units. And it is about the *size* of the jump,
not its *cost* — nobody has yet said what a larger jump costs the operator, which is the whole
content of `jerk_mm_s3`. **The founding premise remains unmeasured on the coupling, by anyone, and
the experiment above is still the one to run.** Two caveats it now inherits from the recovered
sources: Azuma's growth is superlinear, so a horizon sweep spaced 50/100/200/400 ms will show most
of its action in the last two rows; and Willaert et al.'s own blend budgets (500 ms in, 300 ms out,
4 mm deadband) are a sane place to start a reconciler arm of the sweep, at 1 DoF and 150 ms, which
is not this system.

One further prior-art datum that bears on the premise from the other side, and is a warning about
assuming prediction is free: a simulated-teledriving study with 29 participants found a
constant-velocity predictive display at 150 ms of added delay "neither improved performance nor
reduced workload", with operators tolerating 150 ms about as well as 50 ms
(<https://pmc.ncbi.nlm.nih.gov/articles/PMC12788196/>). Conditions: driving, screen-based, not VR,
150 ms — well below this project's impaired profiles, and a vehicle rather than an arm. It does not
transfer, but it is the kind of null result that makes "always predict harder" a hypothesis rather
than a default.

---

## 4. What the operating points look like across this corpus

Worth having in one place, because it decides what transfers. Delay figures below are as reported by
the source, with the caveat that most are round-trip and some do not say.

- **Sub-100 ms**: head-pose prediction design points (60 ms look-ahead,
  <https://arxiv.org/abs/2007.14084>); ROKVISS/Kontur-2 space force feedback at 10-30 ms
  (<https://pmc.ncbi.nlm.nih.gov/articles/PMC8232524/>).
- **100-300 ms**: teledriving study at 150 ms (<https://pmc.ncbi.nlm.nih.gov/articles/PMC12788196/>);
  ground-vehicle trajectory guidance targeting >200 ms (<https://arxiv.org/abs/2212.02706>);
  bimanual avatar telemanipulation handling delays around 120 ms with a passivity observer
  (<https://arxiv.org/abs/2301.00764>).
- **300 ms - 1 s**: telesurgery digital twin at 300/600/900 ms
  (<https://pmc.ncbi.nlm.nih.gov/articles/PMC12864800/>); remote ultrasound MMT to 1000 ms round trip
  (<https://arxiv.org/abs/2502.07922>); TDPA drift compensation to 500 ms
  (<https://arxiv.org/abs/2002.02296>); Analog-1 orbit-to-ground at ~850 ms and Lunar Gateway
  scenarios (<https://pmc.ncbi.nlm.nih.gov/articles/PMC8232524/>).
- **Seconds**: prescient humanoid teleoperation with stochastic delays to 2 s
  (<https://arxiv.org/abs/2107.01281>); ROTEX at 5-7 s
  (<https://www.dlr.de/en/rm/research/robotic-systems/hands/rotex-1988-1993>); space intervention
  interfaces at 4-5 s round trip
  (<https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.747917/full>).

This project's profiles sit in the second and third bands. That is the band where predictive display
is reported to help and where the phantom/truth divergence is large enough that how you absorb it
starts to matter — i.e. exactly where the reconciliation axis lives, and where the classical corpus
stops having anything to say because it split the two objects visually.

---

## 5. How much of this literature is load-bearing on a force loop — and what survives without one

This is the question that decides whether most of the field is relevant to this project at all, so it
gets an explicit accounting rather than a hedge.

**Load-bearing on haptics; does not transfer.**

- **Wave variables and scattering theory.** The construction exists to make a delayed communication
  channel *passive* by transmitting scattering variables instead of the power-conjugate pair
  (force, velocity), so that the delayed channel cannot generate energy and destabilise the loop.
  With no force channel there is no power-conjugate pair, no energy exchanged through the display,
  and nothing to passivate. The known side effects that this literature spends its time on — wave
  reflection and position drift — are artefacts of the transform itself, not problems this project
  has (<https://dspace.mit.edu/handle/1721.1/10622>,
  <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.578805/full>).
- **Time-domain passivity control (passivity observer / passivity controller).** Measures energy flow
  in the loop and dissipates the excess. Same reasoning: the observed quantity does not exist here
  (<https://arxiv.org/abs/2108.07658>, <https://arxiv.org/abs/2301.00764>).
- **Bilateral stability proofs and the transparency-versus-passivity tradeoff.** "Transparency" is
  defined on the impedance the operator feels; the whole optimisation is over a two-port network
  (<https://arxiv.org/abs/2106.12470>, <https://arxiv.org/abs/1711.03605>). The *shape* of the
  tradeoff is suggestive here (see "The tradeoff framing" below) but the machinery is not applicable.
- **MMT's force-rendering half.** Local contact models rendered at ~1 kHz to produce non-delayed
  force. Different loop, different rate, different failure mode.

The 2026-09-09 intent-transmission log's sentence — *"nothing in this project has a haptic loop or a
contact model, so the transferable part is the framing, not the technique"* — is **correct about all
four of those**, and I found nothing to overturn it on those terms.

**Not load-bearing on haptics; transfers directly.**

- **The predictive-display architecture itself.** Nothing about a phantom robot, a commanded display
  or an augmented-virtuality overlay needs force. The space-teleoperation work that actually shipped
  (ROTEX, the JPL demonstrations) used *predictive graphics instead of* force feedback, precisely
  because the delay made force feedback unusable
  (<https://pmc.ncbi.nlm.nih.gov/articles/PMC8232524/>). This is the strongest single argument
  against treating the delay-compensation corpus as a haptics corpus: its flagship results are
  visual.
- **The model-update / model-jump problem and its remedies.** The *mechanism* by which a model jump
  hurts in MMT is haptic, but the *problem shape* is axis-identical to this repo's
  `IReconciler`: an authoritative update contradicts what the local model was showing; applying it
  directly produces a discontinuity; the remedies are gradual updates over a fixed rate or a fixed
  time budget, with a stated bound on the transition
  (<https://scientiairanica.sharif.edu/article_20007.html>). Strip the passivity argument and what is
  left is a design space the Reconciliation axis has independently rediscovered.
- **The Smith-predictor structure.** A model run without the delay in an inner loop, corrected by the
  delayed measurement in an outer loop, with model mismatch as the dominant failure mode
  (<https://en.wikipedia.org/wiki/Smith_predictor>). That is `IPredictor` + `IReconciler` drawn as a
  block diagram, and its documented weakness is the one this project should expect.
- **Supervisory control, time clutch, position clutch.** Decoupling operator time from robot time
  (<https://patents.google.com/patent/US5046022A>) is the `Autonomy/` axis's oldest prior art and has
  no force content whatsoever.
- **The tradeoff *framing*.** Transparency-versus-robust-stability is structurally the same shape as
  prediction-aggressiveness-versus-correction-cost: a fidelity objective traded against a
  well-behavedness objective, where optimising either alone produces a system that is unusable in the
  other. Borrow the shape, not the theorems.

**One consequence worth stating plainly.** In a force-reflecting system, "instability" is a property
of the plant-controller-channel loop and is provable. Here the loop closes through the operator's
eyes and vestibular system, so the analogous failure — over-correction, oscillation, nausea — is a
**human-factors** quantity with no Lyapunov argument available. That is why `jerk_mm_s3` exists and
why no source in this corpus has an equivalent: they could prove what this project has to measure.

**Verdict on the 2026-09-09 dismissal.** As written, scoped to *"does a wire format carrying a model
beat one carrying a point?"*, the dismissal was right and remains right — MMT's wire-format claim is
inseparable from its contact model. Reframed as predictive display, it is too strong in one specific
place: MMT's **model-update discontinuity** is the closest named prior art to the Reconciliation
axis that exists anywhere, and it was filed away as "framing, not technique" when at least the
gradual-update-with-a-deadline technique is directly comparable to `budget-blend`. Prior work reports
that naive model replacement produces discontinuities and that fixed-duration gradual updates fix
them, under 1 kHz haptic conditions this project does not share; that is a candidate generator for
the reconciler axis, not a result about it.

---

## 6. Approach families

### 6.1 Delay and the human operator — the substrate everything else sits on

Before any compensation scheme, the empirical finding: an operator under transmission delay stops
tracking continuously and starts issuing discrete open-loop moves, waiting to see the result. Every
mitigation in this file is ultimately an attempt to give the operator back a closed loop — either by
faking one locally (predictive display), by raising the level of command (supervisory control), or by
making the remote side locally competent (shared autonomy).

> **Ferrell, "Remote manipulation with transmission delay"**, IEEE Transactions on Human Factors in
> Electronics HFE-6 (1965). <https://ntrs.nasa.gov/citations/19650052768> (accessed 2026-09-15).
> *Proposes:* the founding experiment — insert transmission delay between master and slave of a
> remote manipulator and measure what the operator does.
> *Claims:* performance of simple and complex tasks is degraded by the inserted delay; the operator
> adopts move-and-wait.
> *Conditions:* **NTRS record page only — no full text is available for this record** ("There are no
> available downloads"). The abstract line is the one-sentence summary quoted above; task,
> delay magnitudes and subject count are not visible from this page. I did not retrieve the paper.

> **Sheridan, "Telerobotics, Automation, and Human Supervisory Control"**, MIT Press (1992).
> <https://archive.org/details/teleroboticsauto0000sher> (accessed 2026-09-15).
> *Proposes:* the supervisory-control framework — the operator programmes and monitors a locally
> competent remote system rather than closing a continuous manual loop through the delay.
> *Claims:* the standing synthesis of ~30 years of MIT Man-Machine Systems Laboratory work.
> *Conditions:* **catalogue record only; the Internet Archive item is access-restricted and I read no
> content from it.** Listed so the reference is traceable, not as a source for any claim.

> **Farajiparvar, Ying & Pandya, "A Brief Survey of Telerobotic Time Delay Mitigation"**, Frontiers
> in Robotics and AI (2020).
> <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.578805/full>
> (accessed 2026-09-15).
> *Proposes:* a taxonomy splitting mitigation into pre-2014 control-theoretic and interface
> approaches versus post-2014 learned time-series prediction.
> *Claims:* move-and-wait "works; however, it takes a longer time and has smoothness implications";
> wave variables are an extension of passivity theory whose barriers are "wave reflection and drift";
> predictive displays use "phantom robot models to predict real robot motion".
> *Conditions:* a narrative survey, not a measurement — it reports no experiment of its own, and the
> full text as fetched contains no quantitative comparison and nothing on operator comfort or on how
> a display's prediction error is corrected. Useful as a map and a set of pointers only.

### 6.2 Predictive display proper — phantom robots and their descendants

The core trick is unchanged since 1990: render a model of the remote system driven by something you
have locally (the operator's commands, or a forward-simulated state), show it immediately, and let the
delayed truth arrive behind it. What varies across thirty-five years is what gets rendered (wireframe
arm, calibrated overlay, warped video, generated video) and what the prediction is conditioned on.

> **Bejczy, Kim & Venema, "Predictive Display For Teleoperation With Delay"**, NASA Tech Briefs 16(7)
> (1992), JPL/Caltech. <https://ntrs.nasa.gov/citations/19920000396> (accessed 2026-09-15).
> *Proposes:* the phantom robot — a high-fidelity real-time graphics simulation of the manipulator,
> superimposed on the delayed "real" monitoring image, responding to control signals immediately so
> that "motion predicts that of robot. After delay, real image follows motion of phantom."
> *Claims:* the operator can control and monitor through the delay; depth, perspective and lighting
> cues improve control.
> *Conditions:* **a Tech Briefs announcement, not a measured study** — no delay magnitude, no task, no
> performance number, and NTRS offers no downloadable full text. This is the traceable statement of the
> architecture, not of its performance. The underlying 1990 ICRA paper could not be retrieved (§9).

> **Bejczy, Kim & Venema, "The phantom robot: predictive displays for teleoperation with time
> delay"**, IEEE ICRA 1990. NASA record: <https://ntrs.nasa.gov/citations/19910050529> (accessed
> 2026-09-16). **Recovered on the second pass** — IEEE Xplore still refuses this (§9), but NTRS
> carries the record and its full abstract under a different accession number than the Tech Briefs
> item above.
> *Proposes:* the phantom robot proper — "a high-fidelity graphics phantom robot that is being
> controlled in real time (without time delay) against the static task image", so that "the motion
> of the phantom robot image on the monitor predicts the motion of the real robot" and the real
> robot follows it by the communication delay. Real-time graphics of a PUMA arm overlaid on the
> actual camera view, registered by "a simple camera calibration technique".
> *Claims:* from "a preliminary experiment ... by using a very simple tapping task", predictive
> display "enhances the human operator's telemanipulation task performance significantly during
> free motion when there is a long time delay"; and — the caveat that matters for a head-mounted
> system — "either two-view or stereoscopic predictive displays are necessary for general
> three-dimensional tasks".
> *Conditions:* **NTRS record page with full abstract only; "There are no available downloads for
> this record", so I read the abstract and not the paper.** The abstract gives no delay magnitude,
> no subject count and no numbers; the task is a tapping task in free motion, i.e. no contact.
> Two things this settles that the Tech Briefs item could not: the architecture primary is a
> *measured* study, however preliminary, and its own authors flagged single-view 2-D overlay as
> insufficient for 3-D tasks in 1990.

> **Kim & Bejczy, "Demonstration of a High-Fidelity Predictive/Preview Display Technique for
> Telerobotic Servicing in Space"**, IEEE Transactions on Robotics and Automation 9(5), October
> 1993, pp. 698-702. <https://ntrs.nasa.gov/citations/19970020676> (accessed 2026-09-16); full
> text at <https://ntrs.nasa.gov/api/citations/19970020676/downloads/19970020676.pdf>.
> **Recovered on the second pass, in full, and it is the most useful single source in this file
> for §2.** IEEE Xplore refuses it; NASA does not, because it is a JPL work.
> *Proposes:* calibrated high-fidelity overlay of 3-D graphics arm and object models on 2-D TV
> camera images, plus *preview* — the operator drives the graphics to the end point of a planned
> motion and inspects it before committing — plus synthetic camera views of events hidden from any
> real camera.
> *Claims:* verbatim from the abstract, "The positioning accuracy achieved by this technique for a
> zoomed-in camera setting was about +/-5 mm, well within the allowable +/-12 mm error margin at
> the insertion of a 45 cm long tool in the servicing task." **The +/-5 mm figure flagged as
> unverified in §9 is therefore confirmed, at the source, with its margin and its task.**
> *Conditions:* full text read. May 1993, JPL as simulated ground control, Goddard Space Flight
> Center 2500 miles away as the simulated satellite servicing site; the task was exchange of a
> roughly four-foot Orbital Replacement Unit on a satellite mock-up. Video went down the NASA
> Select Satellite TV channel at 30 frames/s; command and status data went over the Internet as
> data blocks. The paper states the expected round trip with the then-existing NASA facilities as
> "four to eight seconds" and the demonstration as "several seconds communication time delay"; it
> does not report the delay it actually measured on the day.
> **The error budget, which is what §2 wanted.** Calibration used 15-30 operator-entered data
> points from 3 or 4 arm poses per camera. Reported residuals, all as a fraction of the image
> plane: side-view and oblique-view cameras (fovy 32 degrees, arm ~3 m away) 0.5-0.7% average and
> 1.6-2.0% maximum, with 0.6% average corresponding to 1 cm of displacement at 3 m; overhead
> front-view camera ~1 m away, wide-angle (fovy 38 degrees) 0.7-0.9% average and 2.0-2.8% maximum,
> 0.8% average corresponding to 0.5 cm at 1 m; the same camera zoomed in (fovy 8 degrees) 1.2-1.6%
> average and 3.2-4.2% maximum, 1.4% average corresponding to 0.2 cm at 1 m. Object localization
> errors were 0.9-1.9% average and 3.0-7.0% maximum, giving 0.2 cm against the zoomed-in overhead
> camera and 2.5 cm against the wide-field side or oblique cameras.
> **Read that budget before treating +/-5 mm as a figure of merit for anything.** It is a *static*
> registration accuracy at one favourable camera setting, it degrades by an order of magnitude at
> wide field of view, and it is the accuracy of where the overlay *sits*, not of how it moves — so
> it remains, exactly as §2 says, the analogue of `correction_magnitude_mm` and not of
> `jerk_mm_s3` or `time_to_convergence_ms`. Note also what the paper lists as future work:
> "interactive model building and intermittent model matching updates using model-based image
> processing" — the reconciliation step, named as future work in 1993 and not measured here either.

> **Schenker, Bejczy & Kim, "Advanced Teleoperation: Technology Innovations and Applications"**,
> NASA/JPL, NTRS 19940027947.
> <https://ntrs.nasa.gov/api/citations/19940027947/downloads/19940027947.pdf?attachment=true>
> (accessed 2026-09-16). Previously listed in §9 as a PDF with no extractable text layer; the text
> layer is there and was read on the second pass.
> *Proposes:* a programme overview of JPL advanced teleoperation, including the calibrated
> predictive graphics display as one component among force reflection and shared control.
> *Claims:* the development of "this predictive graphics display (with a calibrated virtual
> reality) has enabled us to preserve the operational features of teleoperation, and reliably
> operate with intermittent time delays up to 5-10 seconds", naming the JPL-Goddard demonstration
> as the instance.
> *Conditions:* **a programme paper, not an experiment** — the 5-10 s figure is an operational
> claim about the JPL system, with no task, trial count or error number attached to it in this
> document. Useful only as the JPL-side statement of the delay band the architecture was built for,
> which brackets the Kim & Bejczy demonstration above.

> **Cardinaels, Ramakers, Veuskens, Pietrzak, Rovelo Ruiz & Luyten, "Every Move You Make:
> Visualizing Near-Future Motion Under Delay for Telerobotics"**, CHI 2026.
> <https://driescardinaels.be/papers/every-move-you-make/> (accessed 2026-09-16), author-hosted,
> with the PDF at <https://driescardinaels.be/papers/every-move-you-make/paper.pdf>. **Recovered on
> the second pass** — the ACM DL copy (<https://doi.org/10.1145/3772318.3791452>) is still 403, and
> the author's own page was found by searching the exact title.
> *Proposes:* splitting operator uncertainty under delay into three facets — communication (when
> will my command take effect), trajectory (how does my input map to motion) and environmental (what
> will the world do to it) — and externalising each as a separate visualisation. **Network** shows
> four per-key timelines of pending commands travelling toward execution; **Path** projects the
> robot's expected trajectory from its kinematics as left-wheel, right-wheel and centreline lines;
> **Envelope** widens Path into a cone covering worst-case deviation under modelled disturbance.
> *Claims:* Path significantly shortened task time (median 135.2 s versus 209.9 s for the delayed-
> video baseline), gave the lowest cognitive load (NASA-TLX mean 7.29/20 against about 13 for
> baseline and Network) and reduced the frequency of reactive move-and-wait pauses; Envelope lowered
> cognitive load (9.25) but did not improve performance or reduce reactive behaviour; Network had no
> measurable effect.
> *Conditions:* 24 novices (mean age 26.1; 14 male, 10 female), within-subjects and counterbalanced,
> simulated lunar-terrain ground robot, **fixed 2.56 s round-trip delay** chosen as an Earth-Moon
> baseline, 300 s time limit per trial, baseline = delayed video only. Simulation, not hardware; a
> ground vehicle, not an arm; and a delay an order of magnitude beyond this project's profiles.
> **This is substantially a human-factors result and `docs/literature/human-factors.md` is the file
> that should own it** — it is entered here because §9 recorded it as lost from this file and
> because of what it says about *what to draw*. Note the shape: the visualisation that won is the
> one that shows the **trajectory the robot will take**, not the one that shows the **network
> state**, and the uncertainty cone bought comfort without buying performance. That is a directly
> testable claim about display content, and it is orthogonal to how hard a predictor extrapolates.

> **Dybvik, Løland, Gerstenberg, Slåttsveen & Steinert, "A low-cost predictive display for
> teleoperation: Investigating effects on human performance and workload"**, International Journal
> of Human-Computer Studies 145 (2021), article 102536.
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1016/j.ijhcs.2020.102536?fields=title,abstract,year,venue,authors,externalIds>
> (accessed 2026-09-16).
> *Proposes:* a deliberately cheap predictive display — positional and scale transformations applied
> to the video feed, no model of the vehicle or the scene.
> *Claims:* "a statistically significant increase of 20% in human performance with the aid of the
> predictive display"; subjective workload differences were **not** statistically significant,
> though subjective performance and game performance both rose; the gain "almost doubled for
> participants defining themselves as regular gamers".
> *Conditions:* **abstract only, and read through the Semantic Scholar record rather than the
> publisher — ScienceDirect returns 403 on both the article and its CC-BY PDF (§9).** N = 57
> participants; a peg-in-hole navigation task with a ground ROV; three conditions, C1 latency, C2
> latency plus predictive display, C3 baseline with no added latency. **The delay magnitudes are not
> stated in the abstract.** So: the "57 participants, 20% improvement" half of the secondhand figure
> recorded in §9 is confirmed at the source's own abstract, and the "at 700 ms versus a 250 ms
> baseline" half is **still unverified and must not be repeated** — the abstract's baseline is "no
> added latency", which is not the same claim. The workload half of the secondhand summary was also
> wrong in direction: workload did not reach significance.

> **DLR, "ROTEX (1988-1993)"** — project record for the first remotely controlled robot in space.
> <https://www.dlr.de/en/rm/research/robotic-systems/hands/rotex-1988-1993> (accessed 2026-09-15).
> *Proposes:* telerobotic ground control of an on-orbit arm using "the predictive graphics simulation
> concept", plus local sensory feedback and shared autonomy on board.
> *Claims:* the predictive graphics concept compensated a **5-7 second** communication delay; the arm
> closed/opened connector plugs, assembled structures and captured a free-floating object.
> *Conditions:* an institutional project page summarising a 1993 Spacelab D2 flight, not a paper.
> No per-task numbers, no comparison against a non-predictive baseline. Its value here is as
> verification that the architecture flew and at what delay.

> **Kazanzides et al., "Teleoperation and Visualization Interfaces for Remote Intervention in
> Space"**, Frontiers in Robotics and AI (2021).
> <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.747917/full>
> (accessed 2026-09-15).
> *Proposes:* a vocabulary distinction this repo lacks — **commanded display** (a virtual overlay
> placed at the current *commanded* pose, with no delay) versus predictive display proper; plus
> augmented virtuality, which projects the real delayed video onto a 3D model rather than predicting
> it away.
> *Claims:* predictive approaches "are not feasible when the robot must contact the environment
> because current models cannot accurately predict the future state of the system"; trained operators
> preferred a conventional interface with good visualisation over an immersive 3D console.
> *Conditions:* scenarios with latency "on the order of seconds or tens of seconds"; experiments at
> 4-5 s round trip, one model-mediated study at 4 s. Space intervention tasks, expert operators,
> small n. The contact-prediction caveat is stated as an argument, not measured here.

> **Richter, Zhang, Zhi, Orosco & Yip, "Augmented Reality Predictive Displays to Help Mitigate the
> Effects of Delayed Telesurgery"**, ICRA 2019 (arXiv:1809.08627).
> <https://arxiv.org/abs/1809.08627> (accessed 2026-09-15).
> *Proposes:* a Stereoscopic AR Predictive Display (SARPD) overlaying predicted tool motion on the
> surgical view, with real-time tool tracking to keep the overlay registered.
> *Claims:* decreased task completion time with no effect on error rate when operating under delay.
> *Conditions:* **abstract only readable at this URL.** Ten participants on a da Vinci Surgical
> System; the abstract does not state the delay magnitudes tested, the tasks, or the metric
> definitions. Do not carry the headline forward without the delays.

> **Prakash, Vignati, Vignarca, Sabbioni & Cheli, "Predictive Display with Perspective Projection of
> Surroundings in Vehicle Teleoperation to Account Time-delays"** (arXiv:2211.11918, 2022).
> <https://arxiv.org/abs/2211.11918> (accessed 2026-09-15).
> *Proposes:* forecasting the video stream by perspective-projecting the current surroundings into the
> predicted future viewpoint, with a Smith-predictor correction on the vehicle state.
> *Claims:* reduced path deviation with perspective projection versus without.
> *Conditions:* **abstract only.** Online vehicle teleoperation over 4G with variable delay; no delay
> values, no absolute numbers in the abstract. Street "edge case" manoeuvres.

> **Prakash, Vignati & Sabbioni, "SRPT vs Smith Predictor for Vehicle Teleoperation"**
> (arXiv:2305.00911, 2023). <https://arxiv.org/abs/2305.00911> (accessed 2026-09-15).
> *Proposes:* Successive Reference Pose Tracking — transmit a stream of *reference poses* rather than
> steering commands, so the remote side tracks a trajectory instead of replaying stale inputs.
> *Claims:* "significantly improves stability and reference tracking performance, with negligible
> effect of network delays on path tracking", compared against Smith-predictor configurations with
> Lookahead and Stanley drivers.
> *Conditions:* **Simulink simulation only**, variable network delays (values not given in the
> abstract), several vehicle speeds, tight corners, slalom, low adhesion, crosswind. No human subjects
> in the abstract. Structurally this is the same move as sending a time-parameterised plan rather than
> a point — the repo's intent-transmission V4 candidate — in a vehicle domain.

> **"Predicted Trajectory Guidance Control Framework of Teleoperated Ground Vehicles Compensating for
> Delays"** (arXiv:2212.02706, 2022). <https://arxiv.org/abs/2212.02706> (accessed 2026-09-15).
> *Proposes:* predict the operator's *intended trajectory* from delayed historical commands plus LiDAR
> point cloud, and guide the vehicle along it — removing the operator from the direct loop so long as
> "the prediction horizon exceeds the delays".
> *Claims:* significant improvement above 200 ms on completion time, centreline deviation and steering
> effort; **limited effectiveness at small delays, where operators adapt on their own.**
> *Conditions:* five delay levels with emphasis on >200 ms; ground vehicle; metrics as listed. The
> "helps only past a threshold" shape is the interesting part and is the kind of claim this project's
> sweeps can test directly.

> **Chakraborty et al., "Towards Real-Time Generation of Delay-Compensated Video Feeds for Outdoor
> Mobile Robot Teleoperation"** (arXiv:2409.09921, 2024/2025).
> <https://arxiv.org/abs/2409.09921> (accessed 2026-09-15).
> *Proposes:* a modular pipeline — monocular metric depth, a kinematics model to predict future poses
> from user actions, sphere-based rendering, learned inpainting — to synthesise a delay-compensated
> image.
> *Claims:* real-time delay-compensated images in outdoor agricultural field environments.
> *Conditions:* **abstract/landing page only**; no delay values, hardware, frame rates or quantitative
> metrics visible. Recorded for the pipeline shape, not for any number.

> **Khalil & Kwon, "Towards Generative Predictive Display for Vision-Based Teleoperation: A Zero-Shot
> Benchmark of Off-the-Shelf Video Models"** (arXiv:2605.09670, May 2026).
> <https://arxiv.org/abs/2605.09670> (accessed 2026-09-15).
> *Proposes:* benchmark five released video models (transformer and diffusion) zero-shot as
> predictive displays, framed as rollout-based future-frame prediction.
> *Claims:* **no tested model simultaneously achieves low rollout error, non-divergent per-step error
> behaviour, and real-time inference at the source frame rate**; error growth across the horizon is
> reported explicitly as "temporal error evolution across the prediction horizon".
> *Conditions:* **abstract only.** Metrics: mean absolute difference, per-rollout latency, peak GPU
> memory. No hardware or target frame rate stated in the abstract; no human subjects. Relevant here
> mainly as a contemporary statement that error-versus-horizon divergence is the binding constraint —
> in an image domain, which is not this project's domain.

> **Musicant, Kuperman & Barachman, "Safety, Efficiency, and Mental Workload of Predictive Display in
> Simulated Teledriving"**, Sensors (2025). <https://pmc.ncbi.nlm.nih.gov/articles/PMC12788196/>
> (accessed 2026-09-15).
> *Proposes:* a minimal predictive display — constant-velocity extrapolation 150 ms ahead, drawn as a
> black extension of the ego vehicle.
> *Claims:* **a null result.** The predictive display "neither improved performance nor reduced
> workload"; operators showed tolerance to typical 4G/5G delays, with no significant differences
> between 50 ms and 150 ms on most measures.
> *Conditions:* 29 university students, simulated teledriving, three conditions (50 ms baseline,
> 150 ms without PD, 150 ms with PD); safety measures (crashes, steering/braking intensity),
> efficiency (completion time, navigation errors), NASA-TLX, PSSUQ. Screen-based, not VR; vehicle, not
> arm; and 150 ms is at the *bottom* of this project's impaired range. The paper also notes a
> mechanism-level artefact worth knowing: at zero speed the overlay vanishes because the prediction
> term (delay x speed) goes to zero.

> **Yuan et al., "Enhancing telesurgical safety with predictive digital twin synchronization: a
> framework for latency compensation in robotic surgery"**, npj Digital Medicine (2026).
> <https://pmc.ncbi.nlm.nih.gov/articles/PMC12864800/> (accessed 2026-09-15).
> *Proposes:* Digital Twin Visual Assistance — simulate the surgeon's intended action in a digital
> twin from their inputs and render it immediately as a "visuomotor delay bridge", ahead of the
> physical robot's response.
> *Claims:* at 900 ms added latency, 13.6% lower completion time, 27.2% lower workload, 42.9% less
> tissue tearing; three clinical radical nephrectomies completed at 300 ms over 209.2 km.
> *Conditions:* communication latencies 5/300/600/900 ms; system intrinsic latency 20.86 +/- 2.09 ms;
> spatial registration error max 7.83 mm across the workspace and <=2 mm within a 4 cm radius. This is
> the **only** retrieved source that reports both a task-performance benefit and a spatial error
> budget for the predicted overlay, which makes it the closest thing in the corpus to reporting
> prediction quality and its cost together — though the "cost" reported is static registration error,
> not the dynamics of a correction.

> **Penco, Mouret & Ivaldi, "Prescient teleoperation of humanoid robots"** (arXiv:2107.01281,
> 2021/2022). <https://arxiv.org/abs/2107.01281> (accessed 2026-09-15).
> *Proposes:* robot-side prediction of the operator's *future commands* from past trajectories,
> conditioned on the last received commands, so the robot "execute[s] commands before it actually
> receives them" and the returning visual feedback appears synchronised.
> *Claims:* works with stochastic delays up to 2 s on reaching, bottle picking and box placing.
> *Conditions:* **abstract only**; 32-DoF humanoid, operator in a wearable motion-capture suit. No
> per-horizon error breakdown in the abstract. This is precisely the repo's *robot-side* prediction
> problem (predict what the operator wants now), and it is the strongest evidence retrieved that the
> two-sided split in `Prediction/CLAUDE.md` matches how the field actually divides the problem.

> **Schwarz & Behnke, "Low-Latency Immersive 6D Televisualization with Spherical Rendering"**
> (arXiv:2109.11373, 2021). <https://arxiv.org/abs/2109.11373> (accessed 2026-09-15).
> *Proposes:* render captured stereo images as spheres so head-pose changes can be re-rendered
> locally and immediately, decoupling head motion latency from the network.
> *Claims:* outperforms other visualisation methods in lab experiments and a user study.
> *Conditions:* **abstract only** — no latency values, frame rates or hardware detail visible. The
> mechanism (local reprojection against the newest received data) is the view-synthesis analogue of
> what `Prediction/` does for pose, and belongs to a different axis of this project.

> **"Toward a Predictive eXtended Reality Teleoperation System with Duo-Virtual Spaces"**
> (arXiv:2409.15464, 2024). <https://arxiv.org/abs/2409.15464> (accessed 2026-09-15).
> *Proposes:* two virtual spaces — one user-side, one agent-side — with the user-side space localising
> agent and objects locally and being "calibrat[ed] with periodic ground-truth poses from the
> agent-side virtual space".
> *Claims:* addresses end-to-end XR teleoperation latency; a position paper rather than a result.
> *Conditions:* **three-page position paper; no delays, hardware, metrics or reconciliation mechanism
> are given.** Included because the architecture it sketches — local predicted space periodically
> corrected by authoritative poses — is this repo's architecture, stated in XR terms and left
> unmeasured. The gap it leaves open ("what happens at the periodic calibration?") is the
> Reconciliation axis.

> **Zhao, Allison, Vinnikov & Jennings, "The Effects of Visual and Control Latency on Piloting a
> Quadcopter using a Head-Mounted Display"** (arXiv:1807.11123, 2018/2020).
> <https://arxiv.org/abs/1807.11123> (accessed 2026-09-15).
> *Proposes:* a VR paradigm for separating *visual* latency from *control* latency and measuring each
> against performance and simulator sickness.
> *Claims:* not extractable from the landing page beyond the framing.
> *Conditions:* **abstract only** — latency values, participant count, task and results are not
> visible. Recorded because the visual/control latency split is a distinction this project's metrics
> make (`docs/metrics.md` §2) and because it is the one retrieved source that treats simulator
> sickness as a dependent variable under latency.

> **Richter, Orosco & Yip, "Motion Scaling Solutions for Improved Performance in High Delay Surgical
> Teleoperation"** (arXiv:1902.03290, 2019). <https://arxiv.org/abs/1902.03290> (accessed
> 2026-09-15).
> *Proposes:* scale operator motion down under delay instead of predicting — an alternative in the
> same design space that changes the operator's behaviour rather than the display.
> *Claims:* reduces error rate under high delay, with minimal change to the teleoperation
> architecture.
> *Conditions:* **abstract only;** 17 participants, delays and tasks not stated in the abstract.
> Recorded as a non-predictive control condition the field uses, which this project does not have.

> **Du, Vann, Zhou, Ye & Zhu, "Sensory Manipulation as a Countermeasure to Robot Teleoperation
> Delays"** (arXiv:2310.08788, 2023). <https://arxiv.org/abs/2310.08788> (accessed 2026-09-15).
> *Proposes:* alter haptic cues (from a physics engine and robot sensors) so delay is perceived as
> less objectionable, rather than reducing delay.
> *Claims:* reduced completion time, improved perception of visual delays, lower mental strain.
> *Conditions:* **abstract only;** 41 participants; delay values not stated. Haptic-dependent and
> therefore not transferable here, but it is the clearest example in the corpus of the
> perception-side framing this project's nausea proxy also assumes.

### 6.3 Time clutch and position clutch — decoupling operator time from robot time

A separate answer to the same problem, and the one that most resembles the `Autonomy/` axis: instead
of predicting the remote state, let the operator work *ahead* in a forward simulator and let the
remote system catch up on its own schedule.

> **Conway, Volz & Walker, "Tele-autonomous system and method employing time/position
> synchrony/desynchrony"**, US Patent 5,046,022, University of Michigan, filed 1988, granted 1991.
> <https://patents.google.com/patent/US5046022A> (accessed 2026-09-15).
> *Proposes:* a forward simulator (wireframe robot) responding immediately to the operator alongside
> the real delayed robot, plus a **time clutch** (disengage to let the operator specify a path faster
> than the robot can execute it, "saving up time") and a **position clutch** (disengage to reposition
> the simulated end-effector without committing the intermediate poses to the command buffer).
> *Claims:* on re-engagement the remote controller interpolates between archived position samples and
> "seamlessly continues from its actual location toward newly planned positions".
> *Conditions:* **a patent, not an evaluation** — no experiment, no delay figures, no operator study.
> Note the re-synchronisation sentence: even in 1988 the resynchronisation-on-re-engagement step is
> described as interpolation and given one clause. That is the reconciliation step, unnamed and
> unmeasured, thirty-eight years ago.

### 6.4 Model-mediated teleoperation — send a model, update it, and pay at the update

MMT replaces the delayed signal with a locally reconstructed *model* of the remote environment that
the operator interacts with at full rate; the delayed stream is used to correct the model's
parameters. Its central unsolved problem is what happens at those corrections.

> **Hulin et al., "Model-Augmented Haptic Telemanipulation: Concept, Retrospective Overview, and
> Current Use Cases"**, Frontiers in Robotics and AI (2021).
> <https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.611251/full>
> (mirror: <https://pmc.ncbi.nlm.nih.gov/articles/PMC8232524/>) (accessed 2026-09-15).
> *Proposes:* MATM — a generalisation of MMT using two models, a remote one for shared autonomy and a
> local one for augmented haptic feedback; presented with a retrospective of DLR's delay-compensation
> missions.
> *Claims:* stability with delays of several seconds via passivity-based design; explicitly poses the
> model-update problem as "how can stability be established despite the fact that the updating process
> is highly nonlinear, especially in case of time delay, jitter, and packet loss?"; states that
> ROTEX/ETS-VII at ~7 s **relied on predictive graphics instead of haptic feedback**.
> *Conditions:* full text readable. Delay figures spanning the programme: 10-30 ms (ROKVISS/Kontur-2),
> 270 ms (LEO), ~850 ms (Analog-1, Lunar Gateway scenarios), up to 3 s terrestrial; 1 kHz haptic
> rendering; DLR hardware. It is a retrospective, so its numbers are pointers into other papers, not
> measurements made here. **This is the source that best describes Mitra and Niemeyer's original MMT
> for a reader who cannot retrieve it** — I could not retrieve the 2008 IJRR paper itself (§9).

> **Mitra & Niemeyer, "Model-mediated Telemanipulation"**, International Journal of Robotics
> Research 27(2), February 2008, pp. 253-262, DOI 10.1177/0278364907084590.
> <https://api.crossref.org/works/10.1177/0278364907084590> (accessed 2026-09-16).
> *Proposes:* the MMT primary. Rather than transmitting raw sensory data across the delay, the
> method "abstracts the data to form a very simple model of the environment"; the model is sent to
> the master and haptically rendered without lag, while the slave executes only commands consistent
> with that model.
> *Claims:* stable and transparent bilateral telemanipulation across large communication delays.
> *Conditions:* **abstract only, and read from the Crossref record, not the paper — SAGE returns
> 403 and Semantic Scholar reports the full text CLOSED (§9).** The one condition the abstract does
> give is the one that matters most and was missing from this file: the demonstration was **a single
> degree-of-freedom robot at a four-second round-trip delay**. So the named prior art for this
> repo's reconciler was established on 1 DoF, in force, at 4 s. Treat every transfer accordingly.

> **Willaert, Van Brussel & Niemeyer, "Stability of Model-Mediated Teleoperation: Discussion and
> Experiments"**, in *Haptics: Perception, Devices, Mobility, and Communication* (EuroHaptics 2012),
> Springer LNCS. <https://lirias.kuleuven.be/retrieve/1ad073f5-f797-4e9a-bb01-529b8d0126d3>
> (accessed 2026-09-16). Previously in §9's no-text-layer list; the text layer is present and was
> read on the second pass. **This is the single most directly relevant source in the file for the
> Reconciliation axis, and the first pass missed it because the PDF looked like binary.**
> *Proposes:* a first systematic stability treatment of MMT that separates *continuous* model
> adjustment from *discrete model jumps*, and treats the jump as a subsystem-crossing design
> problem with three parts: triggering (when to jump), rendering (how to show the jump to the
> operator) and task execution under a jump (how the controller avoids its own discontinuity).
> *Claims, and the two that matter here.* First, the jump is information and must not be filtered
> away: temporal model discontinuities indicate "a radical change in the expected environment and
> should not be filtered or removed from the user's perception" — but they must be "isolated and
> contained from propagation", or an imperfect model estimator answers one jump with another and
> the system escalates "into a limit cycle or other stability problems". Second, and this is the
> coupling §3 says nobody states: **"longer lags between master and slave will enlarge the size of
> necessary model jumps and increase the need for explicit handling and containment."** That is
> delay-versus-correction-magnitude, asserted directly, by a primary, in a delayed-teleoperation
> system. It is still an assertion and not a measured curve — no distribution, no percentile, no
> smoothness statistic — so §3's broader negative survives intact.
> *The rendering policy, with its numbers.* They select "a gradual, constant-time introduction or
> removal of the object", parameterised as **t_move = 500 ms** to move a newly detected object from
> the master position to its true location, and **t_fade = 300 ms** to retract a disappeared one;
> object detection and removal are gated by a position deadband **Δx0 = 0.004 m**; and on the task
> side a position offset is added at the moment of the jump and "gradually removed over a period of
> time t_fade" so the controller does not step. **That is `budget-blend`, with a published budget,
> a deadband on when to trigger at all, and a separate budget for the growing and the shrinking
> direction.** The repo's reconcilers use one budget for both directions and no trigger deadband.
> *Conditions:* **1 DoF, round-trip delay 150 ms, constant.** Master and slave closed-loop
> bandwidths of 2.7 Hz and 0.5 Hz, a 10 Hz filter, stiffness gains of 2000-4000 N/m, a 7 kg
> displayed mass, a sliding-friction object at about 6 N. Haptic, single-axis, one operator, and an
> order of magnitude faster than this project's worst profiles — so the 500/300 ms budgets are a
> candidate starting point and emphatically not a transferable setting.

> **Mitra, Gentry & Niemeyer, "User preferences and performance in model mediated
> telemanipulation"**, World Haptics Conference 2007, pp. 268-273, DOI 10.1109/WHC.2007.122.
> <https://api.crossref.org/works/10.1109/WHC.2007.122> (accessed 2026-09-16).
> *Proposes:* the only study found anywhere in this corpus that compares *methods of applying the
> correction* against each other and asks the operator which one they prefer — i.e. a reconciler
> comparison, run as a human-subjects study, in 2007.
> *Claims:* reported secondhand by two full-text sources read on the second pass, which agree.
> Willaert et al. above describe it as "a previous user study [that] has compared several methods
> of rendering model jumps" and adopt its gradual constant-time introduction as their own
> implementation. Park's dissertation (below) states the outcome: "users generally prefer slightly
> active methods, where the user is gently pushed back as a new environment model is introduced."
> *Conditions:* **the paper itself was not retrieved — IEEE Xplore returns an empty body, Crossref
> carries metadata but no abstract, and Semantic Scholar reports the abstract elided by the
> publisher (§9).** Crossref confirms authors, venue, pages and date only. Everything above is
> secondhand from two sources that read it, and the study's delay, participant count, task and
> measures are unknown to me. **Do not carry the preference finding forward as a result.** What it
> legitimately does is generate a candidate the Reconciliation axis has not tried: the family of
> *active* corrections that move the operator, rather than the passive blends the repo implements.
> A human with IEEE access should read this one; of everything still missing, it is the most
> directly on-axis.

> **Park, "Improving Teleoperation with Models and Tasks"**, PhD dissertation, Department of
> Mechanical Engineering, Stanford University, December 2009 (adviser: Günter Niemeyer).
> <https://stacks.stanford.edu/file/druid:ph465qm8083/dissertation-augmented.pdf> (accessed
> 2026-09-16); record at <http://purl.stanford.edu/ph465qm8083>. CC BY-NC.
> *Proposes:* models *and* tasks as the two things worth transmitting across a delay — the model
> carries the environment to the master, the task carries operator intent to the slave, and both
> are chosen to change slowly so that the link can be slow.
> *Claims:* the clearest full-text statement retrieved of why model updating is the hard part:
> when model error grows enough to need an update, "the update process effectively connects the
> user to the remote environment, nullifying the efforts to weaken the connection", so the system
> becomes delay-sensitive again precisely at the update. Hence MMT's founding assumption, stated
> as a requirement rather than a convenience — the environment "and hence its model description,
> change slowly" — and the corollary that sparser updates buy delay tolerance.
> *Conditions:* full text read; a dissertation, from Niemeyer's own lab, so it is a well-placed
> secondary on Mitra & Niemeyer 2008 and a primary on its own contribution. Haptic, force-reflecting
> throughout. Recorded here mainly for that one sentence, which is the general form of the tradeoff
> the Reconciliation axis is measuring: **the correction is where the delay gets back in.** Written
> for a force loop, it holds unchanged for a visual one.

> **Xu, Cizmeci, Schuwerk & Steinbach, "Model-Mediated Teleoperation: Toward Stable and Transparent
> Teleoperation Systems"**, IEEE Access 4 (2016), pp. 425-449, DOI 10.1109/ACCESS.2016.2517926,
> gold open access, CC BY-NC-ND.
> <https://api.semanticscholar.org/graph/v1/paper/DOI:10.1109/ACCESS.2016.2517926?fields=title,abstract,year,venue,authors,externalIds>
> (accessed 2026-09-16); publisher record via <https://doi.org/10.1109/ACCESS.2016.2517926>.
> *Proposes:* a survey of MMT in two parts — its history from the late 1980s to 2016, and the main
> challenges in designing a reliable MMT system — with the survey's own experiments run to "compare
> the performance between the existing techniques and to supply data that were missing in the
> previous studies".
> *Claims:* that MMT "has been developed to guarantee both system stability and transparency in the
> presence of arbitrary communication delays".
> *Conditions:* **abstract only, read from the Semantic Scholar record; the IEEE Access full text
> was not retrieved (the DOI redirects to ieeexplore.ieee.org, which returns an empty body).** No
> delay values, hardware or metrics are visible at abstract level.
> **Housekeeping, which is why this entry exists.** `docs/research-log/2026-09-09-intent-
> transmission-feasibility.md` records two URLs for model-mediated teleoperation, of which
> <https://mediatum.ub.tum.de/doc/1326017/1326017.pdf> no longer resolves — TUM's repository now
> answers "Access Denied" behind an anti-bot layer for both the PDF and its landing page (§9). This
> IEEE Access survey is TUM/LMT work by that repository's authors and its abstract matches, almost
> phrase for phrase, the claim that log attributes to the dead URL ("stability and transparency
> robust to arbitrary delay"). **The identification is inferred from authorship and matching claim,
> not confirmed** — mediatum would not serve the record that would confirm it. Offered as the
> working replacement URL with that caveat attached; I have not edited the research log.

> **Yazdankhoo & Beigzadeh, "Increasing stability in model-mediated teleoperation approach by
> reducing model jump effect"**, Scientia Iranica 26 (2019), pp. 3-14.
> <https://scientiairanica.sharif.edu/article_20007.html> (accessed 2026-09-15).
> *Proposes:* name and attack the **model jump** — the destabilising transient when the local virtual
> environment model is replaced because the delayed data finally identified a different environment.
> Their remedy is to halt master-slave communication during the transition and control both sides
> independently with sliding-mode controllers until stability returns.
> *Claims:* stability under large delays for both rigid and compliant environments.
> *Conditions:* **abstract and metadata only were readable; the full PDF is behind a download.** The
> abstract gives no numerical delay values, no hardware and no quantified discontinuity measure. The
> value of this entry is the *name* and the problem statement, not its solution — and note that its
> remedy (suspend the coupling during the transition) is a strategy the Reconciliation axis has not
> tried.

> **Black, Tirindelli, Salcudean, Wein & Esposito, "Visual-Haptic Model Mediated Teleoperation for
> Remote Ultrasound"** (arXiv:2502.07922, 2025). <https://arxiv.org/abs/2502.07922> (accessed
> 2026-09-15).
> *Proposes:* extend MMT with a *visual* model — re-slice and render a pre-acquired ultrasound sweep
> in real time to preview what the delayed image will look like.
> *Claims:* fully compensates delays up to 1000 ms round trip in operator effort and completion time;
> outperforms both conventional teleoperation at long delay and standard (haptic-only) MMT.
> *Conditions:* **abstract only;** 15 volunteer operators; delays to 1000 ms round trip; hardware not
> stated in the abstract. Notable as evidence that the *visual* half of MMT carries real benefit
> independently of the force half — the most directly relevant MMT result for a visual-only system.

> **Beik-Mohammadi et al., "Model Mediated Teleoperation with a Hand-Arm Exoskeleton in Long Time
> Delays Using Reinforcement Learning"** (arXiv:2107.00359, 2021).
> <https://arxiv.org/abs/2107.00359> (accessed 2026-09-15).
> *Proposes:* a two-layer system combining Dynamic Movement Primitives with RL so the remote model
> adapts to changed conditions under long delay.
> *Claims:* RL finds alternative solutions when object position changes; DMPs adapt without model
> uncertainty.
> *Conditions:* **abstract only;** DLR Exodex Adam hand-arm exoskeleton; "long time delays" with no
> values given; RL algorithm unnamed in the abstract. Recorded for completeness of the MMT family.

### 6.5 Wave variables, scattering, and time-domain passivity

These two families exist to answer one question — *how do you keep a force-reflecting loop stable when
the channel delays it?* — with two different instruments: an algebraic transform that makes the
channel passive by construction, and an online energy accountant that dissipates whatever the channel
manufactures. §5 argues neither transfers to a visual-only system; they are here because they are the
bulk of the corpus and because the *drift* problem they both develop is the closest control-theoretic
relative of reconciliation.

> **Niemeyer, "Using wave variables in time delayed force reflecting teleoperation"**, PhD thesis,
> MIT Department of Aeronautics and Astronautics (1996), advisor Slotine.
> <https://dspace.mit.edu/handle/1721.1/10622> (accessed 2026-09-15).
> *Proposes:* the wave-variable formulation of delayed force-reflecting teleoperation — transmit
> scattering variables so that the delayed channel is passive regardless of the delay.
> *Claims:* per the surrounding literature, energy-conservation and stability guarantees under
> constant delay.
> *Conditions:* **DSpace record page verified; no abstract is shown on the record and I did not read
> the 14.4 MB PDF.** Listed as a pointer to the primary source, not as evidence of any claim. The
> secondary description of what wave variables do and what they cost (reflection, drift) comes from
> the Farajiparvar survey above, which I did read.

> **Niemeyer & Slotine, "Telemanipulation with Time Delays"**, International Journal of Robotics
> Research 23(9), September 2004, pp. 873-890, DOI 10.1177/0278364904045563.
> <https://api.crossref.org/works/10.1177/0278364904045563> (accessed 2026-09-16). **Recovered to
> abstract level on the second pass** — SAGE is still 403 (§9), Crossref carries the abstract.
> *Proposes:* the canonical review of the wave-variable concept and of wave-based teleoperation,
> with a design methodology that "aims to create a virtual tool which accounts for the implicit
> limitations imposed by the delay".
> *Claims:* passive transmission guarantees stability, "but wave reflections and spurious dynamics
> may interfere with normal operation"; with the right design choices a system with "consistent and
> predictable behavior" can be built; and the same development extends to "wave-based prediction"
> and to variable delays "such as those inherent to Internet-based telemanipulation".
> *Conditions:* **abstract only.** The stated delay range is unusually explicit for an abstract and
> is the reason to record it: unknown but **constant** transmission delays "ranging from periods
> less than the human reaction time to several seconds". Force-reflecting throughout, so §5's
> accounting applies unchanged — what this confirms is that the flagship review of the family
> itself names reflection and spurious dynamics, not comfort or correction, as the costs it pays.

> **Franken, Stramigioli, Misra, Secchi & Macchelli, "Bilateral Telemanipulation With Time Delays:
> A Two-Layer Approach Combining Passivity and Transparency"**, IEEE Transactions on Robotics 27(4),
> August 2011, pp. 741-756. <https://surgicalroboticslab.nl/wp-content/uploads/2015/01/franken11-tro.pdf>
> (accessed 2026-09-16). Previously in §9's no-text-layer list; text layer present and read on the
> second pass.
> *Proposes:* split the controller into two layers — a top *transparency* layer free to implement
> any bilateral strategy, and a lower *passivity* layer that guarantees no virtual energy is
> generated — with separate communication channels so that energy bookkeeping never mixes with
> desired behaviour.
> *Claims:* any bilateral controller can be made passive this way; changing the transparency layer
> (position-force to impedance-reflecting) requires no change at all to the passivity layer, and
> neither does introducing the delay.
> *Conditions:* two identical 1-DOF direct-drive devices, 1 kHz control loop, a ~1500 N/m mechanical
> spring as the environment, and **an artificial channel delay of 1 s, constant, with no packet
> loss**; user grasps varied hard/relaxed/soft. Recorded because the architectural move — isolate
> the thing that must be guaranteed from the thing you want to optimise, and let them talk over
> separate channels — is the cleanest statement in this corpus of a separation this project makes
> implicitly between `IPredictor` and `IReconciler`. The guarantee itself is energy, so §5 applies:
> the shape transfers, the theorem does not.

> **Bakhshi, Talebi, Suratgar & Abdeetedal, "Stability and Transparency Analysis of a Bilateral
> Teleoperation in Presence of Data Loss"** (arXiv:1711.03605, 2017).
> <https://arxiv.org/abs/1711.03605> (accessed 2026-09-15).
> *Proposes:* model data loss as periodic continuous pulses with a finite series representation, and
> analyse stability and transparency of a wave-variable bilateral scheme under it.
> *Claims:* passivity of the overall system is maintained under the proposed loss model.
> *Conditions:* **abstract only;** the abstract states neither delay ranges, loss rates, nor whether
> results are analytic, simulated or hardware. Relevant only as an illustration that when this
> literature meets packet loss, it reaches for a passivity argument rather than an erasure-coding or
> percentile-loss one — the same observation the repo's FEC log made (§10).

> **Wang, Li & Jiang, "Bilateral Control of Teleoperators with Closed Architecture and Time-Varying
> Delay"** (arXiv:2106.12470, 2021). <https://arxiv.org/abs/2106.12470> (accessed 2026-09-15).
> *Proposes:* kinematic and adaptive dynamic bilateral controllers that need no force/torque
> measurement, robust to arbitrary bounded time-varying delay.
> *Claims:* robustness without force measurement, removing the usual linear-stability assumptions.
> *Conditions:* **abstract only;** validated on a Phantom Omni paired with a UR10. No delay magnitudes
> in the abstract. The "closed architecture, no force sensing" framing is the closest this family gets
> to a system shaped like this one, and it is still a bilateral impedance problem.

> **Risiglione et al., "Passivity-based control for haptic teleoperation of a legged manipulator in
> presence of time-delays"**, IROS 2021 (arXiv:2108.07658).
> <https://arxiv.org/abs/2108.07658> (accessed 2026-09-15).
> *Proposes:* discrete-time energy modulation at the master, passivity constraints inside an
> optimisation-based controller at the slave.
> *Claims:* stability under time delays and master/slave frequency mismatch, across stance and
> locomotion modes.
> *Conditions:* **abstract only;** quadrupedal robot with artificial network delay; delay magnitudes
> and control rates not stated in the abstract. Included as the modern statement of the PO/PC
> approach, and as the clearest example of the thing §5 says does not transfer: the quantity being
> observed is energy.

> **Lenz & Behnke, "Bimanual Telemanipulation with Force and Haptic Feedback through an
> Anthropomorphic Avatar System"** (arXiv:2301.00764, 2023). <https://arxiv.org/abs/2301.00764>
> (accessed 2026-09-15).
> *Proposes:* an avatar telemanipulation system with force feedback and — the part relevant here — "a
> predictive avatar model for limit avoidance which runs on the operator side, ensuring low latency".
> *Claims:* evaluated at the ANA Avatar XPRIZE semifinals plus lab experiments and a small user study
> with mostly untrained operators.
> *Conditions:* **abstract only;** delays, control rates and the passivity-observer details are not
> visible at the landing page. Recorded as an example of operator-side local prediction used for a
> *constraint* (joint limits) rather than for display — a use this project has not considered.

> **Coelho, Singh, Muskardin, Balachandran & Kondak, "Smoother Position-Drift Compensation for Time
> Domain Passivity Approach based Teleoperation"** (arXiv:2002.02296, 2020).
> <https://arxiv.org/abs/2002.02296> (accessed 2026-09-15).
> *Proposes:* compensate the master-slave position drift that TDPA's energy dissipation accumulates,
> "in a smoother way, which keeps the forces within the normal range of the teleoperation task".
> *Claims:* good position tracking with regular-amplitude forces, against prior compensators that
> either over-conservatively constrain feedback or "add high impulse-like force signals".
> *Conditions:* **abstract only;** round-trip delays up to 500 ms, constant and variable; hard-wall
> contacts; no hardware or sample rate in the abstract, and **no quantitative smoothness metric is
> named**. This is the single closest control-theoretic analogue of this repo's Reconciliation axis:
> an accumulated discrepancy, a correction that must be applied, an explicit preference for smooth
> over impulsive, and — exactly as in the classical predictive-display line — **no measured definition
> of "smoother".**

### 6.6 Smith predictors

The oldest structural answer to dead time, and the block diagram this project's pipeline reproduces
without saying so: run a model of the plant without the delay to close a fast inner loop, and use the
delayed measurement only to correct the model in a slow outer loop.

> **"Smith predictor"**, encyclopedia entry. <https://en.wikipedia.org/wiki/Smith_predictor>
> (accessed 2026-09-15).
> *Proposes:* O. J. M. Smith's 1957 dead-time compensator: an inner loop on an undelayed plant model,
> an outer loop correcting with the delayed measurement.
> *Claims:* removes the delay from the closed-loop characteristic behaviour when the model is exact.
> *Conditions:* **this is an encyclopedia article, not a source of measurement**, and is used here only
> to fix the structure and its stated failure mode — "it is impossible for the model to perfectly
> match the plant", so model mismatch bounds everything. Smith's 1957 original was not retrievable in
> any form I could fetch. Applications to teleoperation are in §6.2 (both Prakash sources use a Smith
> predictor as the comparison baseline, which is how the field treats it: the thing you must beat).

---

## 7. Searches that found nothing useful, with the exact queries

Recorded so the next run does not repeat them. Each was run 2026-09-15.

1. `predictive display prediction error grows with horizon divergence model mismatch teleoperation
   measured` — returned only the 2026 generative-video benchmark (§6.2) and unrelated MPC work.
   **No classical or control-theoretic predictive-display study reporting an error-versus-horizon
   curve surfaced.** This is the single most load-bearing absence in this file: the project's
   founding premise has no measured counterpart here.
2. `"predictive display" mismatch correction "when the real image arrives" discontinuity operator
   perception jump telerobotics` — returned patents and an unrelated camera-frame misalignment paper.
   No source describing, let alone measuring, the transition when truth overtakes a prediction on a
   display.
3. `"predictive display" evaluation robot manipulator "prediction error" reported millimeters overlay
   registration delay experiment open access` — returned registration-accuracy results only
   (static mm error), confirming §2: this literature measures where the overlay *sits*, not how it
   *moves* when corrected.
4. `Nuno Basanez Ortega "Passivity-based control for bilateral teleoperation: a tutorial" Automatica
   2011 upcommons pdf` — no open copy located; every hit was a paywall or ResearchGate.
5. `Sheridan "Space teleoperation through time delay: review and prognosis" 1993 open access pdf
   review` — no fetchable copy of the canonical review. Its content is only available secondhand.
6. `supervisory control Sheridan teleoperation retrospective open access chapter "supervisory control"
   time delay history IntechOpen` — three rounds, nothing fetchable and on-topic; the IntechOpen hits
   were about supervisory control of industrial processes, a different field with the same name.
7. `time domain passivity approach teleoperation DLR elib open access Ryu Artigas passivity observer
   controller` — the DLR/elib open-access copies were not surfaced; only publisher pages.

**One broader negative worth stating as a result:** across all of the above plus the family searches
in §6, I found **no source that reports prediction error and correction cost together**, and **no
source that reports any smoothness statistic (jerk, or anything equivalent) of a corrected display**
in teleoperation. The closest is Coelho et al. (§6.5), which argues for smoothness qualitatively and
names no metric. If the repo's `jerk_mm_s3` percentile reporting has prior art, it is not in this
field and I did not find it.

**The second pass, 2026-09-16, did not overturn that** — and it is worth being precise about what it
did and did not look for. It ran targeted title-and-author searches for the specific sources named
in §9 and nothing else; it did not re-run any of the seven queries above, and it did not search for
new topics. Two of its recoveries bear on the negative without changing it: Willaert et al. name the
*size* of a correction as growing with lag but measure no distribution (§6.4), and Mitra, Gentry &
Niemeyer compared correction-rendering policies on human subjects in 2007 but could not be retrieved
(§9.1), so what they measured is unknown. One procedural note for the next run: this pass exhausted
its web-search budget partway through and completed the remainder through direct fetches of NTRS,
Crossref and Semantic Scholar endpoints, which is why §9.1 is organised by route rather than by
query. Targeted API lookups are cheaper than searches and should be tried first when the target is
already named.

---

## 8. Adjacent fields that own the vocabulary this one lacks

Out of scope for this file's remit and probably covered by another topic file — included in short
form because the naming question in §1 cannot be answered honestly without them.

> **Aronson, "Dead Reckoning: Latency Hiding for Networked Games"**, Game Developer, 19 September
> 1997. <https://www.gamedeveloper.com/programming/dead-reckoning-latency-hiding-for-networked-games>
> (accessed 2026-09-15).
> *Proposes:* agreed extrapolation algorithms on every node, with entity updates published only when
> the extrapolation error exceeds a threshold; plus smoothing of the visible transition on update.
> *Claims:* the tradeoff, in as many words — "Large dead reckoning thresholds can result in noticeable
> jerkiness of motion when new PDUs are received. On the other hand, small dead reckoning thresholds
> force more PDUs to be sent"; and "DIS simulations also typically use smoothing algorithms to lessen
> the apparent jerkiness as entities are updated from dead reckoned positions to new updated true
> positions".
> *Conditions:* **a practitioner article, not a study** — no experiment, no numbers, vehicle-scale
> entity motion in DIS-style simulations, 1997 network conditions. It asserts the
> extrapolate-harder-costs-more-visible-correction tradeoff; it does not measure it.

> **Hakiri, Berthou & Gayraud, "Survey study of the QoS Management in Distributed Interactive
> Simulation Through Dead Reckoning Algorithms"** (arXiv:1008.3758, 2010).
> <https://arxiv.org/abs/1008.3758> (accessed 2026-09-15).
> *Proposes:* survey of bandwidth-reduction via dead reckoning, plus an ANFIS-based extension.
> *Claims:* conventional dead reckoning "sacrific[es] remote predictive accuracy in favor of low
> computational complexity" by ignoring contextual information.
> *Conditions:* **abstract only;** no thresholds, convergence mechanisms or smoothing detail visible
> at the landing page. Recorded as the survey entry point into the DIS threshold literature, where
> terms like *convergence algorithm* and the failure modes *hops, warps, wobbles, shimmies* live.

> **Gambetta, "Client-Side Prediction and Server Reconciliation"**, part II of *Fast-Paced
> Multiplayer*. <https://www.gabrielgambetta.com/client-side-prediction-server-reconciliation.html>
> (accessed 2026-09-15).
> *Proposes:* the game-netcode formulation — the client renders its own inputs immediately, keeps the
> unacknowledged inputs, and on receiving authoritative state replays them from that state.
> *Claims:* removes perceived input latency while keeping the server authoritative.
> *Conditions:* **a tutorial, not a study** — no measurements, no network conditions, deterministic
> simulation with replayable inputs assumed. Two things matter for this project: (a) the *word*
> "reconciliation" means, in that field, *replay inputs from authoritative state* — which is
> `rollback`, not the repo's blend-the-displayed-pose family; and (b) the tutorial explicitly
> describes the visible outcome of a misprediction as the character jumping, and **does not discuss
> smoothing the correction at all**. The repo's five reconcilers occupy a design space this tutorial
> skips in one sentence.

> **Olano, Cohen, Mine & Bishop, "Combatting Rendering Latency"**, ACM SIGGRAPH Symposium on
> Interactive 3D Graphics course/paper material (1995).
> <https://userpages.cs.umbc.edu/olano/papers/latency/> (accessed 2026-09-15).
> *Proposes:* three HMD latency remedies — predictive tracking, just-in-time pixels (render each pixel
> for its own display time), and post-render image repositioning against the newest tracker data.
> *Claims:* "for prediction to work effectively, the lag must be small and consistent."
> *Conditions:* full text readable; a survey of techniques with no quantitative comparison of its own,
> at 1995 HMD latencies. The third technique — correct the already-rendered image with fresher truth
> instead of predicting further — is the display-side analogue of favouring reconciliation over
> prediction aggressiveness, which is a framing this project could use.

> **Azuma & Bishop, "A Frequency-Domain Analysis of Head-Motion Prediction"**, SIGGRAPH '95,
> pp. 401-408. <https://www.ronaldazuma.com/papers/s95paper.pdf> (accessed 2026-09-16).
> **Recovered on the second pass, in full.** The ACM DL copy and the UNC author copy
> (<https://www.cs.unc.edu/~azuma/s95paper.pdf>) both return 403; the first author's own site serves
> the same PDF without complaint. `docs/literature/learned-predictive-display.md` already covers
> Azuma's 1997 course notes, which restate the transfer-function analysis — **see that file for the
> analysis itself; what follows is what this paper adds beyond it.**
> *Proposes:* characterise head-motion predictors analytically rather than empirically, by deriving
> frequency-domain transfer functions for two families — second-order polynomial extrapolation and
> Kalman-based prediction — so that predictors can be compared without re-running everyone's traces.
> *Claims, and these are the numbers §3 wanted.* (a) "even with perfect, noise-free inputs, the
> error in predicted position grows rapidly with increasing prediction intervals and input signal
> frequencies", and for the second-order polynomial "the rate of growth is roughly the square of the
> prediction interval and the frequency". (b) The prediction interval p and the angular frequency ω
> "always occur together as ω p", from which: "Halving the prediction interval means that the signal
> can double in frequency while maintaining the same prediction performance. That is, bandwidth
> times the prediction interval yields a constant performance level." (c) Empirically, on recorded
> head motion through a simulated 6-D Kalman predictor, "the average errors at the 100 ms prediction
> interval are 2.3 times as large as the errors at the 50 ms prediction interval, but the factor
> jumps to 9 when comparing 200 ms against 50 ms". (d) Acceleration sensing is worth more to a
> predictor than velocity sensing.
> *Conditions:* full text read. Head motion only — 6 DoF of a head-mounted display, recorded from "a
> first-time user in our HMD system", one motion sequence for the scatterplots, a 30 degree
> field-of-view HMD at 512x512 for the screen-space error, prediction intervals of 50/100/200 ms.
> The transfer-function half assumes **linear, separable, temporally discrete** predictors and
> noise-free inputs; the paper says plainly that nonlinear and adaptive predictors are outside it.
> No robot, no network, no operator task, no comfort measure.
> **Why it is worth having despite all of that.** It is the only source retrieved anywhere in this
> file that reports a *measured error-versus-horizon curve at this project's horizon values*, and
> its shape is the interesting part: the growth is superlinear and the knee is between 100 and
> 200 ms — 2.3x for the first doubling, 9x by the second. §3's half one is therefore better
> supported than the first pass could show. §3's half two is untouched: this paper measures error
> and says nothing whatsoever about what correcting it costs, because in an HMD the correction is
> the next frame. And note the domain gap before borrowing the numbers — head motion at 1 m of
> screen projection is not a robot arm's end-effector, and 1995 head motion is not 2026 head motion.

> **Gül, Bosse, Podborski, Schierl & Hellge, "Kalman Filter-based Head Motion Prediction for
> Cloud-based Mixed Reality"**, ACM Multimedia 2020 (arXiv:2007.14084).
> <https://arxiv.org/abs/2007.14084> (accessed 2026-09-15).
> *Proposes:* Kalman-filter head-pose prediction to hide cloud rendering latency.
> *Claims:* 0.5 degrees better orientation prediction than an autoregression baseline at a 60 ms
> look-ahead.
> *Conditions:* **abstract only;** recorded head-motion traces, baseline = autoregression, look-ahead
> 60 ms; no hardware, no trace provenance, no subject count visible. Included because the *chosen
> operating point* (60 ms) is a data point about where practitioners believe head/hand prediction
> stops paying — relevant to this project's 200/400 ms horizon rows, and consistent with the ~200 ms
> hand-motion breakdown already recorded in `docs/research-log/2026-09-09-intent-transmission-
> feasibility.md`.

---

## 9. Sources I could not retrieve, and what happened

Listed with the failure, per `docs/literature/CLAUDE.md`. None of these is used as evidence above.

**This section was revised on 2026-09-16 by a second, gap-filling pass.** The original records are
kept; items that were recovered are annotated in place with where they went, and §9.1 lists every
route the second pass tried, including the ones that failed. Nothing was deleted.

**Whole platforms that refused every attempt** (do not retry unauthenticated):

- **IEEE Xplore** — returned an empty body on every attempt. Lost: Bejczy, Kim & Venema, "The phantom
  robot: predictive displays for teleoperation with time delay", ICRA 1990
  (<https://ieeexplore.ieee.org/document/126037/>) — the primary phantom-robot paper; and Kim &
  Bejczy, "Demonstration of a high-fidelity predictive/preview display technique for telerobotic
  servicing in space", IEEE T-RA 9 (1993) (<https://ieeexplore.ieee.org/document/258061/>) — the
  JPL-to-Goddard demonstration, and the one classical source that reportedly quantifies overlay
  positioning accuracy. Search-engine summaries state a ~+/-5 mm figure against a +/-12 mm allowable
  margin; **I could not verify that and it must not be repeated as fact.**
  **Both recovered 2026-09-16 via NTRS, which is the right route for JPL work and was not tried on
  the first pass** (IEEE Xplore is still an empty body). The 1990 ICRA paper is at
  <https://ntrs.nasa.gov/citations/19910050529> — record page with full abstract, no downloadable
  full text — and is now entered in §6.2. The 1993 T-RA paper is at
  <https://ntrs.nasa.gov/citations/19970020676> with a complete, text-extractable PDF at
  <https://ntrs.nasa.gov/api/citations/19970020676/downloads/19970020676.pdf>, and is entered in
  §6.2 with its full error budget. **The +/-5 mm figure is verified at the source, with its +/-12 mm
  margin and its 45 cm tool-insertion task, and may now be repeated with those conditions attached.**
  Note that NTRS also holds a second record for the same 1993 paper, 20210004575.
- **Elsevier ScienceDirect** — HTTP 403. Lost: Hokayem & Spong, "Bilateral teleoperation: An
  historical survey", Automatica 42 (2006) (also 403 via
  <https://dl.acm.org/doi/10.1016/j.automatica.2006.06.027>); Dybvik, Løland, Gerstenberg, Slåttsveen
  & Steinert, "A low-cost predictive display for teleoperation", Int. J. Human-Computer Studies
  (2021) (<https://www.sciencedirect.com/science/article/pii/S1071581920301385>) — reported
  secondhand as 57 participants and a 20% performance improvement at 700 ms versus 250 ms baseline;
  **unverified, do not carry forward.**
  **Partially resolved 2026-09-16.** ScienceDirect is still 403 on both the article page and the
  CC-BY PDF at `.../S1071581920301385/pdf`, but the abstract is readable through the Semantic
  Scholar graph API and the entry is now in §6.2. Verified: N = 57, "a statistically significant
  increase of 20% in human performance". **Not verified, and still not to be repeated: the "700 ms
  versus 250 ms" part** — the abstract names three conditions as latency, latency-plus-predictive-
  display and a no-added-latency baseline, and gives no millisecond values at all. Also worth
  correcting: the secondhand summary implied a workload benefit, and the abstract says subjective
  workload differences were *not* statistically significant. Hokayem & Spong remains lost; see §9.1.
- **SAGE** — HTTP 403. Lost: Niemeyer & Slotine, "Telemanipulation with Time Delays", IJRR 23 (2004)
  (<https://journals.sagepub.com/doi/10.1177/0278364904045563>) — the canonical wave-variable review.
  Mitra & Niemeyer, "Model-mediated telemanipulation", IJRR 27 (2008), is on the same platform and was
  not retrieved either; §6.4's Hulin retrospective is the fetchable substitute.
  **Both recovered to abstract level 2026-09-16 through Crossref** (`api.crossref.org/works/<DOI>`),
  which carries publisher-deposited abstracts for both and is a route the first pass did not use.
  SAGE itself is still 403 and neither full text was read. Niemeyer & Slotine is now in §6.5 and
  Mitra & Niemeyer in §6.4 — the latter supplying the condition the file most needed, that the MMT
  primary was demonstrated on **1 DoF at a 4 s round trip**. Semantic Scholar reports the 2008 full
  text CLOSED, so there is no open copy to find.
- **Springer / SpringerLink** — HTTP 303 into an IdP authorize endpoint. Lost: the ROBOMECH Journal
  "Intention-reflected predictive display" paper (2023) and the Willaert et al. "Stability of
  Model-Mediated Teleoperation" chapter. **The Willaert chapter was recovered 2026-09-16** — not
  from Springer, but from the KU Leuven Lirias copy already listed below as a no-text-layer PDF,
  which does have a text layer. It is now in §6.4 and is the most on-axis source in the file. The
  ROBOMECH paper was not retried.
- **MDPI** — HTTP 403 on both <https://www.mdpi.com/2079-9292/14/13/2595> (wave-variable compensators)
  and <https://www.mdpi.com/1424-8220/22/23/9119> (Smith predictor versus SRPT with human subjects).
  Unexpected for an open-access publisher; recorded so the next run does not assume MDPI is fetchable.
- **ACM Digital Library** — HTTP 403, including the PDF path
  (<https://dl.acm.org/doi/pdf/10.1145/215585.215650>). Lost: Azuma & Bishop, "A frequency-domain
  analysis of head-motion prediction", SIGGRAPH 1995 (<https://dl.acm.org/doi/10.1145/218380.218496>)
  — **the primary source for error-versus-prediction-interval growth**; also the author-hosted copy at
  <https://www.cs.unc.edu/~azuma/s95paper.pdf> returned 403. Secondhand summaries report that error
  grows rapidly with prediction interval and that prediction is most effective below ~80 ms;
  **unverified.** Also lost: CHI 2026, "Every Move You Make: Visualizing Near-Future Motion Under
  Delay for Telerobotics" (<https://doi.org/10.1145/3772318.3791452>, which redirects to dl.acm.org and
  then returns 403), which from its title is the most directly relevant recent human-factors work in
  this whole field.
  **Both recovered 2026-09-16, both from author-hosted copies, both found by searching the exact
  title in quotes.** Azuma & Bishop 1995 is served without complaint from the first author's own
  site at <https://www.ronaldazuma.com/papers/s95paper.pdf> — note that this is a *different host*
  from the UNC copy, which is still 403 — and is entered in §8.
  The CHI 2026 paper is at <https://driescardinaels.be/papers/every-move-you-make/> with its PDF
  alongside, and is entered briefly in §6.2; it is substantially human-factors and
  `docs/literature/human-factors.md` is the file that should own it.
  **The ~80 ms figure is now verified and means something narrower than the secondhand summary.**
  It is not in the 1995 paper at all. It is in Azuma's 1997 course notes, where it describes a
  *different, earlier* system: "For system delays under 80 ms, prediction reduced average
  registration errors by a factor of 2-3 without the use of inertial sensors and a factor of 5-10
  with the use of inertial sensors." That is the delay regime in which a 1994 result was
  demonstrated — **not a stated cut-off past which prediction stops working.** The nearest thing to
  a cut-off in that document is a separate, blunter sentence: "Simply attaching a predictor to a
  virtual environment system with 250 ms of delay is not going to yield satisfactory results."
  Both were read at <https://www.ronaldazuma.com/papers/sig97pred.pdf> on 2026-09-16. The 1997
  course notes belong to `docs/literature/learned-predictive-display.md`, which already covers them;
  the verification is recorded here because the unverified flag was recorded here.
- **Semantic Scholar** — returned an empty body for both search and paper pages. **Still true of
  the HTML site on 2026-09-16, and irrelevant: the graph API at
  `api.semanticscholar.org/graph/v1/paper/DOI:<doi>?fields=...` returns clean JSON including
  abstracts and open-access status.** That, plus `api.crossref.org/works/<doi>` and its
  `works?query.bibliographic=` form, is how three of this pass's recoveries were made. Both are
  metadata services: an entry resting only on them has read an abstract, not a paper, and says so.
- **NASA ADS** — HTTP 405 on <https://ui.adsabs.harvard.edu/abs/1991IJOE...16..152N/abstract>
  (Niemeyer & Slotine, "Stable adaptive teleoperation", IEEE J. Oceanic Eng. 1991).
- **MIT Press (direct.mit.edu)** — HTTP 403. Lost: Kim & Bejczy, "Virtual Reality Calibration and
  Preview/Predictive Displays for Telerobotics", Presence 5(2) 1996; "A Photorealistic Predictive
  Display", Presence 13(1) 2004; Artigas et al., "Time Domain Passivity Control for Position-Position
  Teleoperation Architectures", Presence 19(5) 2010.
- **mediatum.ub.tum.de** — "Access Denied" (Anubis anti-bot). This matters beyond this file:
  <https://mediatum.ub.tum.de/doc/1326017/1326017.pdf> is one of the two MMT URLs recorded in
  `docs/research-log/2026-09-09-intent-transmission-feasibility.md`, and it **no longer resolves**.
  **Retried 2026-09-16 on the landing page too — <https://mediatum.ub.tum.de/doc/1326017/> returns
  the same Anubis "Access Denied" page, with no metadata, so the record cannot even be identified
  from its own repository.** The working replacement offered in §6.4 is Xu, Cizmeci, Schuwerk &
  Steinbach's gold-open-access IEEE Access 2016 MMT survey, which is TUM/LMT work whose abstract
  matches that log's claim sentence closely. **That identification is inferred, not confirmed.**
  I have not edited the research log.
- **deepblue.lib.umich.edu** and **scholar.afit.edu** — HTTP 403.

**PDFs that fetched but had no extractable text layer** (the URL is live; the fetch tool returned raw
binary, so I read nothing and they are not used as sources) — **all five resolved on 2026-09-16; see
§9.1 for how, and note that four of them are now used as sources:** Franken et al., "Bilateral
Telemanipulation With Time Delays: A Two-Layer Approach", IEEE T-RO 2011
(<https://surgicalroboticslab.nl/wp-content/uploads/2015/01/franken11-tro.pdf>); Bejczy, "Advanced
Teleoperation: Technology Innovations and Applications", NTRS 19940027947
(<https://ntrs.nasa.gov/api/citations/19940027947/downloads/19940027947.pdf?attachment=true>);
InTech, "Contact Task by Force Feedback Teleoperation Under Communication Time Delay"
(<https://cdn.intechopen.com/pdfs/594/InTech-Contact_task_by_force_feedback_teleoperation_under_communication_time_delay.pdf>);
the KU Leuven Lirias copy of "Stability of Model-Mediated Teleoperation"
(<https://lirias.kuleuven.be/retrieve/1ad073f5-f797-4e9a-bb01-529b8d0126d3>); Azuma, "Correcting for
Dynamic Error", SIGGRAPH 97 course notes
(<https://www.ronaldazuma.com/papers/sig97pred.pdf>).

**Net effect on this file's coverage.** The three most load-bearing primary sources in the classical
line — the 1990 phantom robot paper, the 1993 JPL/Goddard demonstration, and Mitra & Niemeyer's 2008
MMT paper — were all unreachable, and are represented here by fetchable NASA, DLR and DLR-retrospective
substitutes that describe them without their numbers. A human with institutional access should read
those three directly before any of this file's §1-§3 conclusions are treated as settled.

### 9.1 Second retrieval pass, 2026-09-16 — routes tried and what each returned

A bounded gap-filling pass against the §9 list above. No new topics were surveyed. Recorded so that
the next run knows which doors open and which are shut for good.

**Routes that worked, in descending order of yield.**

1. **NTRS (`ntrs.nasa.gov`) for anything JPL or NASA.** Three recoveries, including two of the
   three primaries the first pass called load-bearing. The search endpoint
   `https://ntrs.nasa.gov/api/citations/search?q=<terms>&page.size=25` returns readable JSON and
   surfaces accession numbers that a plain web search does not — the 1990 phantom-robot paper is
   filed under 19910050529, which is *not* the 19920000396 Tech Briefs record the first pass found,
   and the 1993 T-RA paper is filed twice, under 19970020676 and 20210004575. US-government works,
   fully open, no paywall anywhere in the path.
2. **Searching the exact title in quotes to find an author-hosted copy.** Two recoveries: Azuma &
   Bishop 1995 from `ronaldazuma.com` after `cs.unc.edu` refused the identical file, and the CHI
   2026 paper from the first author's personal site after the ACM DL refused it. The lesson is the
   narrow one: a 403 identifies a *host*, not a document, and the first pass generalised from host
   to document in both cases.
3. **`api.crossref.org/works/<DOI>`** — publisher-deposited abstracts for Mitra & Niemeyer 2008 and
   Niemeyer & Slotine 2004, both behind SAGE 403. `works?query.bibliographic=<title+author>` also
   works as a bibliographic lookup and is how Dybvik et al.'s DOI was confirmed.
4. **`api.semanticscholar.org/graph/v1/paper/DOI:<doi>?fields=title,abstract,...`** — abstracts and
   open-access status where Crossref has none. Recovered Dybvik et al. and Xu et al. Note the
   failure mode: for IEEE conference records it returns `abstract: null` with an explicit note that
   the publisher elided it, which is how Mitra, Gentry & Niemeyer 2007 stayed lost.
5. **Extracting the text layer locally from a PDF the fetch tool reported as raw binary.** All five
   PDFs in the list above have perfectly good text layers; the fetch tool simply does not run a PDF
   text extractor. This retired the whole category. Two of the five turned out to matter: the Lirias
   copy of Willaert et al. is the most on-axis source in the file, and the NTRS Schenker/Bejczy/Kim
   report gives JPL's own 5-10 s operating claim. Franken et al. 2011 is now in §6.5. Azuma's 1997
   course notes are covered by `learned-predictive-display.md` and were read here only to settle the
   ~80 ms flag. The fifth, Nohmi & Bock's InTech chapter "Contact Task by Force Feedback
   Teleoperation Under Communication Time Delay", read as a force-feedback space-teleoperation
   chapter with no predictive-display or reconciliation content; it is **retrieved but not entered**,
   because §5 rules its family out and this pass was not authorised to broaden the file.

**Routes that did not work.**

- **`web.archive.org`** — unavailable to the fetch tool in this environment ("unable to fetch from
  web.archive.org"), so the archive route suggested for dead URLs could not be tried at all. Anyone
  retrying the mediatum URL or the UNC Azuma copy should start there, since it is untested rather
  than exhausted.
- **IEEE Xplore** — still an empty body, direct and via `doi.org` redirect. This blocked the one
  remaining on-axis primary, Mitra, Gentry & Niemeyer's 2007 World Haptics user study of model-jump
  rendering methods (§6.4).
- **Elsevier ScienceDirect** — still 403, including on a **CC BY** PDF path that Semantic Scholar
  lists as open (`/science/article/pii/S1071581920301385/pdf`). An open licence does not imply a
  fetchable host.
- **mediatum.ub.tum.de** — Anubis "Access Denied" on both the PDF and the landing page.
- **Hokayem & Spong, "Bilateral teleoperation: An historical survey", Automatica 42 (2006) — still
  lost, and now lost through four routes rather than two.** ScienceDirect 403; the ACM DL mirror
  403; `api.crossref.org/works/10.1016/j.automatica.2006.06.027` returns metadata (Automatica 42(12),
  pp. 2035-2057, December 2006) but **no abstract**; `api.semanticscholar.org` returns the record
  with open-access status CLOSED and the abstract elided by the publisher. No open copy was located.
  Of the §9 targets this pass was asked to close, this is the one that closed on nothing at all.
- **`semanticscholar.org` HTML search** — still an empty body; use the graph API instead.

**Net effect on this file's coverage, revised.** Of the three primaries called load-bearing above,
**two are now read at the source** — Bejczy, Kim & Venema 1990 to abstract depth and Kim & Bejczy
1993 in full, both in §6.2 — and the third, Mitra & Niemeyer 2008, is read to abstract depth with
its operating point recovered, backed by two openly-licensed full texts from the same lab (§6.4).
Azuma & Bishop 1995 is read in full (§8). **§1 and §2 can reasonably be treated as settled** on the
architecture and naming questions: the phantom-robot architecture is confirmed as a measured study
whose own authors flagged single-view overlay as inadequate for 3-D tasks, and the reconciliation
step is confirmed as named, parameterised and studied on humans inside the model-mediated line,
which is a stronger version of what §2 already said. **§3 cannot.** Its half one is now supported by
a measured error-versus-horizon curve, but its half two — the coupling from horizon to correction
cost — rests on one qualitative sentence from Willaert et al. and nothing else, and the single
source that compared correction policies against each other is still behind IEEE Xplore. A human
with institutional access should read Mitra, Gentry & Niemeyer 2007 and Hokayem & Spong 2006 before
§3 is treated as anything but open.

---

## 10. Notes for the repository

Arguments only. This run may write no file but this one, so these are recorded here rather than
applied, per the instruction that gave me the file.

1. **`docs/adr/` or the vocabulary section of the root `CLAUDE.md` should learn the phrase "predictive
   display".** It currently appears zero times in the repository (verified across docs, ADRs, logs,
   sources and comments on 2026-09-15) while the `Prediction/` axis implements it and the
   `Reconciliation/` axis solves the problem it creates. The cost of the gap is not stylistic: it is
   that a reader looking for prior art searches the repo's vocabulary, finds nothing, and re-derives.
   Suggested minimal change, for a human to apply: add to the root `CLAUDE.md` vocabulary list a
   **predictive display** entry — *"the architecture where the operator is shown a locally computed
   estimate of remote state instead of the delayed truth; `Prediction/` + `Reconciliation/` together
   implement one"* — with a pointer to this file. I have not made the edit.
2. **The standing instruction in `docs/research-log/2026-09-09-fec-redundancy-feasibility.md`
   (~line 166), "Do not re-run this search", was defensible in its scope and wrong as a standing
   rule.** Defensible: the query was about erasure coding on a command uplink, the corpus answered in
   passivity and stability terms, and that mismatch is real — §6.5's data-loss paper reaches for a
   passivity argument exactly as that log predicted. Wrong as a standing rule for two reasons. First,
   the instruction is attached to a *corpus* ("delay-compensated teleoperation") but was earned by a
   *query* ("FEC on a pose-command uplink"); fencing off a corpus because one narrow question got no
   answer from it is how a project stops discovering that the corpus names its central architecture.
   Second, the thing it fenced off contains this file's §1, §2 and §5 — the name of what the project
   built, the only two named prior formulations of its reconciliation step, and the reason most of
   the family does not apply. The narrow FEC finding should stay; the corpus-wide "do not re-run"
   should not be honoured by future runs, and this file is the reason. A human should consider
   striking that one sentence and replacing it with "no FEC-on-command-uplink study found; this says
   nothing about the rest of the delay-compensation corpus — see `docs/literature/predictive-
   display.md`."
3. **The MMT dismissal in `docs/research-log/2026-09-09-intent-transmission-feasibility.md` holds as
   written and is too strong when re-framed.** See §5's verdict. The specific salvage is one row for
   `Reconciliation/`'s "Planned, not yet implemented" table, which a human should decide on: **suspend
   the correction coupling during a transition** (Yazdankhoo & Beigzadeh's model-jump remedy, §6.4)
   is structurally different from all five existing reconcilers, which all apply a correction
   continuously. I am not adding the row.
4. **Two metric observations, both explicitly *not* proposals to change `docs/metrics.md`.** (a) Prior
   work in this field reports the predicted-versus-true *spatial* discrepancy as a static registration
   accuracy in mm (§2, §6.2) — which is the closest external analogue to `correction_magnitude_mm`,
   and suggests that the missing "displayed pose versus ground truth" measurement flagged at the
   bottom of `Reconciliation/CLAUDE.md` is the quantity the rest of the field actually reports. (b) No
   source retrieved reports any smoothness statistic of a corrected display, so `jerk_mm_s3` has no
   external calibration point; that is worth knowing before anyone treats a jerk percentile as
   comparable to a published figure. Neither observation licenses redefining anything.
5. **The cheapest experiment this file implies** — and the reason §3 matters — is a horizon sweep on a
   *fixed* reconciler reporting prediction error and correction cost together at 50/100/200/400 ms.
   It is already required by `Prediction/CLAUDE.md` requirement 6, it would test the project's
   founding premise directly, and no retrieved source has run its equivalent. The coupled-axis trap
   applies: vary horizon only, hold the reconciler fixed, then swap.
6. **Two pieces of cross-file housekeeping the second pass created, for a human to place.** (a) The
   CHI 2026 paper in §6.2 (Cardinaels et al., "Every Move You Make") is substantially a
   human-factors result — 24 novices, NASA-TLX, reactive-pause coding — and
   `docs/literature/human-factors.md` is the file that should own it. It is entered here only
   because §9 recorded it as lost *from here*; a human should decide whether it moves or is
   cross-referenced. I did not write to that file. (b) `docs/research-log/2026-09-09-intent-
   transmission-feasibility.md` carries a dead URL (`mediatum.ub.tum.de/doc/1326017/1326017.pdf`).
   §6.4 offers a working, gold-open-access replacement with its identification flagged as inferred.
   I did not edit that log.
7. **Item 3's proposed Reconciliation row now has a published parameterisation, which changes what
   the row should say.** The first pass proposed "suspend the correction coupling during a
   transition" from Yazdankhoo & Beigzadeh. The second pass found the same family stated more
   usefully by Willaert, Van Brussel and Niemeyer (§6.4): a gradual constant-time model jump with
   **separate budgets for the two directions** (500 ms to introduce, 300 ms to retract) and a
   **trigger deadband** (4 mm) below which no correction is applied at all. Against the repo's five
   reconcilers, the deadband is the genuinely new knob — every existing reconciler corrects
   continuously and none has a dead zone. A human should decide whether that is one planned row or
   two. It is a candidate generated by prior work, measured at 1 DoF and 150 ms in a force loop, and
   it settles nothing here until it is swept.
