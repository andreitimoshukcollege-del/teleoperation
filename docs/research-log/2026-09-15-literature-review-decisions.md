# Literature review — decision record

**Run:** 2026-09-15 to 2026-09-16. Organizer: research-organizer. Branch `literature-review`, HEAD
`aafa443` at start. **Survey run** — no implementation, no assembly, no sweep, no judge panel.
**Nothing committed.**

**Deliverable:** `docs/literature/` — five topic files plus `README.md`. 210 sources, every one
fetched at the URL given.

---

## The question

Produce a broad map of five fields this project sits next to, as a reading list a human will
actually open. The run was commissioned against a specific suspicion: that `Prediction/` composed
with `Reconciliation/` is **predictive display**, a named field with decades of results, and that
this project rederived it without reading anything. 27 research logs, 13 ADRs, six built axes, and
"predictive display" appeared zero times in the repository.

**The suspicion was correct**, with one real qualification that turned out to be the most useful
part of the answer. See `docs/literature/README.md`; it is not repeated here.

## How it was decomposed, and why

Five researchers, one per field, one file each. The two-to-four-candidate rule was deliberately not
applied: it exists because candidates must be distinguishable by a measurement, and survey areas are
partitioned by subject rather than competing. These five collide only if two are given the same
file, so each was given exactly one and explicitly forbidden the other four.

**Worktrees were skipped.** They exist to isolate builds; prose researchers writing to distinct
files do not need one, and skipping saved ~375 MB and the entire teardown risk. Report step 1 —
copying logs out of worktrees — therefore had nothing to do, and no log was ever at risk.

**No judges were run.** `judge-admissibility` needs an implementation and `judge-verdict` needs a
sweep with a baseline row; neither exists. `judge-methodology` needs a `manifest.json` to work
against. Running one anyway would have emitted noise that a later reader mistakes for evidence.

Each brief carried the verification standard first and at length, because the dominant failure mode
of a machine-assisted review is confidently invented references. That emphasis was vindicated: three
researchers independently caught themselves constructing an identifier rather than retrieving one,
and one such guess resolved to a real but completely unrelated paper. All three recorded it as a
process note. None of the fabrications reached a file.

## Approaches considered and not pursued

- **A sixth and seventh field — shared/traded autonomy under delay, and VR locomotion.** Autonomy is
  a built axis here and the literature is large. Dropped to keep five researchers at full depth
  rather than seven at partial; the classical file covers supervisory control, which is the part
  that bears on delay. Autonomy under delay is the obvious next survey and nothing in this run
  covers it.
- **Autonomous-driving world models (GAIA/DriveDreamer class).** Deliberately excluded by the
  world-models researcher, with my agreement, to spend coverage on manipulation. Recorded in that
  file. Someone will propose it; it was weighed.
- **Letting researchers read each other's files to cross-reference.** Rejected. Five agents running
  concurrently against a shared directory is a collision pile, and the value is low — I can
  cross-link in the index, which is what happened. Boundary markers in each file did the job.
- **Having a researcher fix the `Do not re-run this search` line in
  `2026-09-09-fec-redundancy-feasibility.md`.** Rejected: out of scope, and a researcher editing
  another run's log is exactly the shared-file edit the anti-collision rule forbids. The argument
  for replacing that sentence is written in `predictive-display.md` §10.2 instead. **The line is
  still there and a human should decide.**
- **Answering the unfinished human-factors question myself.** Rejected on two grounds, either
  sufficient: writing research content is a researcher's job and never the organizer's, and the file
  has a live owner.
- **A structured bibliography or citation-key scheme.** Rejected — `docs/literature/CLAUDE.md`
  explicitly specifies inline URLs and no bibliography. Following the existing convention beat
  improving it.

## Course changes mid-run

**Four continuation passes were dispatched after the first five reported.** Each of the first four
researchers identified a specific high-value gap in its own final report, and one of them had
discovered a technique the others lacked: a paper ACM serves as HTTP 403 was openly available as an
author-hosted copy at a corporate research lab, found by searching its exact title. That transfers,
and it did — the continuations recovered the two load-bearing JPL primaries, the founding RTC
congestion-control papers, and Azuma & Bishop.

There is no channel to resume a completed agent's session, so the continuations were fresh agents
scoped to exactly one already-static file, additive-only, forbidden from deleting or weakening any
existing verified entry. That constraint held: no entry was lost, and one file's 48-row table came
back with its column consistency programmatically re-verified.

The continuations also produced the run's only **retraction**: the networking file had claimed, from
an unread source, that a 2016 paper moved Google Congestion Control to a sender-side trendline
estimator. Reading it showed neither that paper nor its journal extension contains the words
"trendline" or "linear regression" — both describe the Kalman filter. The claim was withdrawn and
the trendline estimator libwebrtc actually ships is now recorded as a gap with no peer-reviewed
evaluation anywhere. This is the clearest demonstration in the run of why an unread source is worse
than an absent one.

## Verdicts

| Field | File | Outcome |
|---|---|---|
| Classical predictive display | `docs/literature/predictive-display.md` | **Complete.** 50 sources. Answered the naming question directly and refused to over-claim on the half its evidence did not reach |
| Learned predictive display | `docs/literature/learned-predictive-display.md` | **Complete.** 26 sources. Recovered the paper a previous run quoted without reading, and corrected the quote |
| World models | `docs/literature/world-models.md` | **Complete.** 51 sources. Negative result on the bridge question, and a compute-cost table built only from reported numbers with named hardware |
| Human factors | `docs/literature/human-factors.md` | **Incomplete — see below.** 30 sources. Three of four questions answered, and question (a) is the run's most consequential finding |
| Networking | `docs/literature/networking.md` | **Complete.** 53 sources. Forced one retraction; found a measurement trap in the field's founding metric definitions |

## Left open

**The human-factors survey is unfinished and its agent never reported.** The file is self-labelled
"Status: in progress" and ends inside question (c). Question (d) — whether a tail statistic or a
median better predicts operator discomfort, which is the assumption behind reading verdicts off p99
— is unanswered, and the file has no searches-that-found-nothing or could-not-retrieve sections.

I did not launch a continuation against it, and that was the most deliberate decision of the run.
An agent that has not reported is not a dead agent; a silent transcript is not a liveness signal,
and a second agent with the same file scope produces two live writers competing over one file. The
incomplete file that can be resumed beats a tidy one that cannot. **Whoever picks this up should
confirm the original agent is gone before writing to that file.** Its first three answers are
substantial and stand on their own.

**Four sources are blocked on institutional access**, listed with their failed routes in
`docs/literature/README.md`. One matters more than the rest: Mitra, Gentry & Niemeyer's World
Haptics 2007 user study of how to render a model jump is the only work anywhere that compared
correction policies against each other on human subjects — the Reconciliation axis's own question,
asked nineteen years ago, and this project cannot read it.

**Nothing here changed any metric, tolerance, baseline or axis document, and nothing should until a
sweep says so.** Every finding above is prior work reporting X under conditions Y. The candidates
this run generated — an asymmetric in/out correction budget with a trigger deadband, and the two
falsifying sweeps for the founding premise, both of which use only metrics that already exist — are
research runs someone should commission, not edits anyone should make. The one genuine repository
edit this run recommends is deleting a single sentence from a 2026-09-09 log, and that is a human's
call.
