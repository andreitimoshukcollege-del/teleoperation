# research-log

What a research run leaves behind after its worktree is deleted and its `results/` output is gone.
`results/` is gitignored and agent worktrees are disposable, so **these files are the only durable
record of why anything was done.** The code that worked is in git; the reasoning that produced it,
and the four approaches that did not work, are only here.

Two artifacts per run, and they answer different questions.

## `<YYYY-MM-DD>-<slug>.md` — the candidate log

One per candidate, written by the agent that built it, continuously rather than at the end. This is
where **specifics** go: the hypothesis and its falsifier stated before any code, the design
decisions and their alternatives, what each test proves, the numbers, the gate output, and an honest
section on how the result could flatter itself.

Established structure, worth matching: a `**Agent:** / **Worktree:** / **Branch:** / **HEAD at
start:**` metadata block, a `## Seed self-check` PASS/FAIL table, `## Hypothesis (written before any
code)`, `## Results`, `## Verification`, `## Left undone / for a human`.

Length is not a virtue here but it is not a vice either — these run to a few hundred lines and that
is fine, because nobody reads them for orientation. They are read when someone needs the detail.

## `<YYYY-MM-DD>-<slug>-decisions.md` — the decision record

**One per run, not per candidate.** Written by whoever organized the run. This is the file someone
reads six months later to understand what happened, so it holds **major decisions only**:

- The question, and how it was decomposed — and *why that decomposition*.
- **Approaches considered and not pursued, each with the reason.** This is the most valuable section
  and the one most likely to be skipped. An approach silently dropped gets proposed again next
  quarter by someone who has no way to know it was already weighed.
- Verdicts: one line per candidate — built, rejected on argument, rejected on measurement, or
  excluded — each pointing at its candidate log for the detail.
- **Course changes mid-run.** Something implemented and then reverted is a decision, and the reason
  it was reverted is usually worth more than the thing itself.
- What was left open, and what is blocked on a human.

**What does not belong here:** numbers tables, code, derivations, test inventories, tool output.
Every one of those lives in a candidate log or in `results/`. If this file needs scrolling to read,
it has drifted into specifics and should be cut back — a decision record that is as long as the work
it describes will not be read, which defeats the only reason it exists.

Write it even when a run produces nothing. "We looked at this and stopped, because" is a result.

## How this relates to the other places decisions are recorded

- **`docs/adr/`** — architecture decisions that *bind future code*: a new `Contracts/` interface,
  a wire format, a change to how `Pipeline/` composes things. A research decision that would change
  the architecture does not belong in a decision record; it graduates to an ADR, and the decision
  record says so and links it.
- **An axis folder's "Tried and rejected" table** — the durable, per-axis index of closed ideas, so
  they stop being retried. It carries the one-paragraph verdict and links here for the argument.
  A decision record is chronological and per-run; that table is thematic and per-axis. Both are
  needed: the table answers "has anyone tried this?", this directory answers "what happened when
  they did?".
- **`results/`** — the numbers, with a `manifest.json` and a SHA. Never duplicate a metric table
  into a log; link the run directory instead, because a number copied by hand loses its provenance
  and cannot be re-derived.

## Discipline

Copy a log out of an agent worktree **before** the worktree is removed. Worktrees are gitignored and
routinely deleted; a log left in one is simply lost, and no amount of care afterwards recovers it.
