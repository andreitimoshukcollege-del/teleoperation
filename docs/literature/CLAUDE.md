# literature

A map of the outside work this project sits next to. One file per field, plus `README.md` as the
index. This directory exists because the project spent a year producing results without
systematically reading anything — sources appeared inline in a handful of candidate logs, always
subordinate to something being built, never free-standing.

## The rule that governs everything here

Quoted from `.claude/agents/deep-researcher.md`, because this directory is where it will be
strained hardest:

> **Literature generates candidates and prevents reinvention. It is never evidence about this
> system.** A published number never enters `results/` and never settles a comparison here. The
> paper was measured on a different system, under conditions you do not control and probably cannot
> reconstruct.

So a file in here may say *"prior work reports X under conditions Y"*. It may not say *"therefore
our Z is wrong"* — that requires a sweep. What prior work legitimately does is tell you which
experiment to run, and stop you building something that already exists.

**Do not write "cite" about a source.** That word is reserved for a number traceable to a
`manifest.json` with a reachable SHA (`results/CLAUDE.md`). Write "source" or "prior work", so the
two never blur.

## Per-source format

Established across ~40 sources already in `docs/research-log/`. Bold name, inline URL, accessed
date, then three fields:

> **Zhang, Fay, Kilmartin & Moore, "A Garch-based adaptive playout delay algorithm for VoIP"**,
> Computer Networks 54 (2010). <https://example.org/paper.pdf> (accessed 2026-09-15).
> *Proposes:* ARMA(1,0)+GARCH(1,1) forecasting of the delay distribution, playout set from the
> forecast quantile.
> *Claims:* lower late-loss at equal mean delay than a percentile tracker.
> *Conditions:* three real VoIP traces captured 2007, G.729B, 20 ms packets, 80-byte payloads.

**`Conditions:` is the field that matters and the one that gets skipped.** A technique that wins at
20 ms RTT may lose at 300 ms with burst loss, and this project's profiles go there. A source with
no recorded conditions is barely worth having — the headline number transfers to nothing.

There is no bibliography and no citation key format. URLs go inline where the source is used.

## What else belongs in a file here

- **Searches that found nothing**, with the query, so the next person does not repeat them.
- **Sources that could not be retrieved**, with what happened — "ACM returned HTTP 403" is a useful
  thing to have written down, and there is precedent for it.
- **Where only an abstract was readable**, said plainly in `Conditions:`. An abstract's headline
  number without its setup is the exact thing this directory exists to stop being repeated as fact.

## Verification standard

Every source must resolve at the URL given. A source nobody fetched does not go in — the dominant
failure mode of a machine-assisted literature review is confidently invented references, and a
plausible-looking fake is worse than a gap because it will be believed.

## How this relates to the other places things are recorded

- **`docs/research-log/`** is chronological and per-run: what a research run left behind. This
  directory is thematic and durable — organised by field, not by when someone looked.
- **An axis folder's "Tried and rejected" table** answers "has anyone here tried this?". A file here
  answers "has anyone *anywhere* tried this, and what happened?".
- **`docs/adr/`** binds future code. Nothing here binds anything. If a source changes what this
  project should build, that graduates to an ADR or a research run — the source is the reason, not
  the authority.
