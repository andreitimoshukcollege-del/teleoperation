# testdata

Committed fixtures the build depends on.

## `golden/`

`basic-session.tlog` is the golden log `Teleop.Eval -- verify` replays (Gate 3: replay twice,
byte-identical). Frozen once committed — do not hand-edit it. Regenerate only via
`dotnet run --project Teleop.Eval -- gen-golden` (run from `core/`), inspect the diff, and
commit the result. A legitimate reason to regenerate is an intentional `RecordFormat` version
bump; that is ordinary maintenance, not something that needs an ADR.

`.gitattributes` already declares `*.tlog text eol=lf` — this is a plain, diffable text file,
not a binary blob, and is not LFS-tracked.

## `traces/`

`synthetic-burst.trace` is the one committed trace fixture, authorized by
`docs/adr/0004-network-profile-suite.md`. Same discipline as `golden/`: never hand-edit it.
Regenerate only via `dotnet run --project Teleop.Eval -- gen-trace` (run from `core/`), inspect
the diff, and commit the result.

`cellular-congested`, `leo-satellite`, and `long-haul` — the three frozen-set names implying a
real network capture — are still intentionally absent; adding a real capture here does not need
a new ADR (0004 already reserves the names), but does need an actual measurement, which no agent
can produce.
