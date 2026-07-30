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

Intentionally does not exist yet. Network-profile captures for `EmulatedTransport`'s
trace-driven mode are Phase 5 scope (`docs/setup.md`), and `Transport/CLAUDE.md` requires an
ADR before that frozen set is created or edited.
