# literature — index and reading list

Five fields this project sits next to, mapped 2026-09-15/16 by one survey run. **210 sources, every
one fetched at the URL given.** Conventions, the per-source format and the rule that none of this is
evidence about this system are in `CLAUDE.md` in this directory — read that first if you intend to
add anything.

---

## The finding that prompted the run, and it holds

**This project independently rederived predictive display.**

"Predictive display" appeared **zero times** in this repository before this run — not in a doc, an
ADR, a log, a source file or a comment. `Prediction/` composed with `Reconciliation/` — operator
sees a locally extrapolated proxy of the remote state, corrections blended in when truth arrives —
is that field's core architecture, named by Ferrell in the 1960s, developed through Sheridan's
supervisory control, flown by JPL and DLR against multi-second round trips, and continuously worked
on since. It has a name, a lineage and decades of results.

The identification is exact for the architecture and **not** exact for the blend, which is the
interesting part:

- The classical line renders the prediction and the delayed truth as **two separate visual objects**
  and lets the operator fuse them perceptually. It therefore never needed a blending step, never
  named one, and never measured one. **This project's decision to display a single pose — which VR
  forces — is what creates the reconciliation problem.** That is a real difference, not a gap in
  anyone's reading.
- The step *is* named twice outside that line. Model-mediated teleoperation calls it the **"model
  jump"** and treats gradual fixed-duration updates as the remedy, which is `budget-blend` under
  another name. Distributed interactive simulation calls it **"convergence"** or **"smoothing"**.
- Game netcode's "reconciliation" means input replay from an authoritative state — that is
  `rollback`, already assessed and closed on this axis, and **not** what the other four reconcilers
  here do. The word collides; the technique does not.

**What this changes:** `budget-blend` has a published parameterisation nobody here knew about —
**separate in and out budgets (500 ms moving in, 300 ms fading out) behind a 4 mm trigger
deadband**, from Willaert, Van Brussel & Niemeyer. All five reconcilers in this repo use a single
symmetric budget and no deadband. That is a concrete candidate the axis did not have, and it is the
clearest return on the whole run.

A standing instruction in `docs/research-log/2026-09-09-fec-redundancy-feasibility.md` — **"Do not
re-run this search"**, aimed at the delay-compensated-teleoperation corpus — was defensible for the
narrow FEC question that earned it and wrong as a standing rule, because the corpus it fenced off
contains the name of what this project built. `predictive-display.md` §10.2 argues for replacing
that sentence. It is one line in an existing log and this run did not touch it.

---

## What each file covers

| File | Field | Sources |
|---|---|---|
| `predictive-display.md` | Classical and control-theoretic: Ferrell/Sheridan, JPL phantom robot, ROTEX, model-mediated teleoperation, wave variables, time-domain passivity, Smith predictors | 50 |
| `learned-predictive-display.md` | The learned version: RNN/LSTM/Transformer hand and head prediction, intent prediction, generative video as a display, cloud-gaming latency hiding | 26 |
| `world-models.md` | World models and learned dynamics for manipulation: Dreamer/PlaNet lineage, LaDi-WM, GE-Act, AHEAD, the 2026 surveys — organised around measured inference cost | 51 |
| `human-factors.md` | What operators actually perceive: latency JNDs, motion-to-photon, cybersickness and SSQ, sensory conflict, Fitts under delay **(incomplete — see below)** | 30 |
| `networking.md` | Adaptive playout and delay-based control: Ramjee, Moon/Kurose/Towsley, NetEQ as shipped, GARCH playout, LEDBAT/Copa/BBR/GCC, the delay-vs-loss curve | 53 |

`human-factors.md` is **marked "Status: in progress" by its own author and is genuinely
unfinished.** It ends inside question (c); its fourth question — whether a tail statistic or a
median better predicts operator discomfort — is unanswered, and it has no
searches-that-found-nothing or could-not-retrieve sections. Its first three questions are answered
and substantial. Treat the file as a strong partial, not a completed survey.

---

## Read these first

1. **Hulin et al., model-mediated teleoperation retrospective** (`predictive-display.md` §2) — the
   best fetchable map of the field, and the source for the fact that ROTEX and ETS-VII used
   predictive graphics *instead of* force feedback. That single fact is why this corpus is not the
   haptics-only corpus it was previously dismissed as.
2. **Willaert, Van Brussel & Niemeyer, model jumps** (`predictive-display.md` §3) — the reconciler
   axis's nearest published relative: triggering, rendering, asymmetric budgets, a deadband, and the
   one in-field statement of the coupling this project was founded on.
3. **Gamage et al., "So Predictable!", UIST '21** (`learned-predictive-display.md` §4) — the paper a
   previous run quoted without reading. Per-horizon numbers, and the finding that each added
   derivative helps at short horizons and *hurts* at long ones.
4. **Azuma & Bishop / Azuma's 1997 course notes** (`predictive-display.md` §3,
   `learned-predictive-display.md` §2) — the analytic form of error against prediction interval, and
   the still-decisive observation that a predictor's own runtime adds directly to the delay it
   exists to hide.
5. **Khalil & Kwon, generative predictive display benchmark** (`world-models.md`, `arXiv:2605.09670`)
   — measures exactly this problem with off-the-shelf video models and reports that none achieves
   low rollout error, non-divergent per-step error and real-time inference simultaneously.
6. **WebRTC's `reorder_optimizer.cc` / `underrun_optimizer.cc`** (`networking.md` §6) — the only
   fully specified, allocation-free, integer-only playout recipe in the field, and the only scheme
   anywhere with a *separate* reordering estimator. It is source code; there is no paper.
7. **The motion-sickness dose literature** (`human-factors.md` §5) — read for what it does *not*
   support. See below.

---

## Where prior work bears on something this project believes on its own reasoning

None of this settles anything. A published number never enters `results/`, and every item here is a
sweep someone has not run. Ordered by how weakly founded the belief currently is.

**Peak jerk as "the nausea proxy" has no support in the literature, for this case.** The validated
dose model for physical motion sickness is **band-weighted acceleration accumulated over time**, not
a derivative order, and its provocative band is 0.1–0.5 Hz peaking near 0.2 Hz. A correction spread
over a 100 ms budget puts essentially all its energy one to two orders of magnitude above that band.
There is one controlled test of jerk against discomfort in the physical domain and its sign came out
**negative** — at matched peak acceleration, longer pulses were rated *less* comfortable, so "slower
is gentler" is not a safe prior even where the intuition originated. On the jerk of a *displayed*
object as a sickness predictor, prior work supplies **nothing**. Unstudied, not contradicted — but
this is the sole differentiating metric on the Reconciliation axis, and every reconciler verdict
this project holds is a p99-jerk ranking.

**The founding premise splits in two.** "Error grows with the horizon" is well supported and has an
analytic form. "Predicting harder necessarily costs more correction" is supported by exactly one
qualitative sentence in the entire retrieved corpus, and prior work reports the two relationships
separately: monotonic in the horizon at fixed predictor form, but **not** monotonic across predictor
forms at fixed horizon — one source reports a predictor that was simultaneously 79% more accurate
*and* lower on every side-effect measure than its comparator. The project's one sentence conflates
these. The falsifying sweep uses only metrics that already exist.

**The ~200 ms extrapolation collapse is real, at ~160–180 ms, and was quoted inaccurately.** The
paper behind that claim was never read here — ACM returned 403 — and is openly available as an
author copy. The quoted phrase "after 200 ms the RMSE increases significantly" does not appear in
it; the text says degradation past **t = 0.18 s**, first significant separation at **t > 160 ms**,
and it is a knee in an exponential, not a cliff. Consequence for `docs/metrics.md` §4's Δ ∈ {50,
100, 200, 400} ms grid: **Δ = 400 ms is not noise, it is a regime change** where an unconditioned
predictor should be expected to lose to `none` — worth demonstrating, not dropping — and Δ = 200 ms
is the most informative row, not the least.

**The 5 mm tolerance and 100 ms budget have no perceptual basis in prior work.** Not refuted —
unstudied for this stimulus. The structural point is sharper than the number: measured detection
thresholds for a displayed offset depend on the observer's concurrent motion and on the *sign* of
the offset, which a single scalar tolerance cannot express.

**A measurement trap in the field this project borrowed its buffering ideas from.** The founding
playout papers define average playout delay **over the played-out packets only** — so the delay axis
of those classical curves is computed over a sample set that moves as you slide along the curve.
That is delay-correlated censoring, the exact failure `docs/metrics.md` already warns about,
unremarked in the founding definitions of that literature.

**Playout operating points are chosen differently than here.** From roughly 2000 onward the playout
literature picks a target late-loss *rate* and reports the resulting average delay, sweeping the
algorithm's own control parameter to produce a curve — no delay percentile anywhere. The convention
is absent at the origin (Ramjee 1994 parameterises by buffer size) and the neighbouring
real-time-congestion literature uses the *opposite* convention, reporting delay percentiles with
loss as an outcome. Same authors, same venues, same decade, no shared convention. What that implies
for reading verdicts off p99 here is a sweep, not a reading.

**A world model does not currently fit this frame budget, and nobody is using one for this
problem.** Human-in-the-loop delay compensation predicts the *operator's command*, never the world.
The only reported figures under 11.1 ms get there by not generating pixels, and both are amortised,
open-loop, chunked numbers with no closed-loop re-prediction rate reported — which is precisely
where that argument would break. Pixel-generating models run three orders of magnitude slower.
Beware a keyword trap: papers whose titles contain "teleoperation" in this field almost always mean
*data collection*, not delay.

---

## What is missing, and what a human with institutional access could close in an hour

Retrieval was the binding constraint, not search. Each file records its failures with routes tried;
these are the ones worth someone's login:

- **Mitra, Gentry & Niemeyer's World Haptics 2007 user study** of how to render a model jump. The
  only source anywhere that compared correction policies against each other on human subjects — the
  Reconciliation axis's question, asked and answered nineteen years ago. IEEE Xplore, abstract
  elided by the publisher, absent from Crossref. **The most on-axis missing source in the field.**
- **Nancel et al., UIST '16** — names seven side-effect categories of prediction and correlates them
  against predictor choice. Green OA deposited only in HAL, which is bot-walled; fifteen routes
  failed. The correlation values are the part that matters and they remain unknown.
- **Atzori & Lobina's 2006 playout survey** — the only work that would let the playout families be
  compared on a common footing. The authors' own repository holds metadata and no file.
- **Moon, Kurose & Towsley 1998** — the field's founding paper; no open-access copy exists anywhere,
  per two independent metadata services. Currently reconstructed secondhand from two papers that
  used it.

Two environment facts worth recording so nobody re-derives them: **`web.archive.org` and all
Internet Archive infrastructure are unreachable** from this tooling, which combined with CiteSeerX's
retirement kills most routes into pre-2010 literature; and a PDF that appears to have "no extractable
text layer" usually does have one — that is a property of the fetch tool, and `pdftotext` reads the
same file fine. The second observation converted seven recorded failures into full reads in a single
pass and should be the first thing any continuation tries.
