# Recording

The versioned `.tlog` format: `RecordFormat` (the spec — tags, delimiter, unset token, numeric
formatting, FNV-1a checksum fold), `SessionWriter`/`SessionReader` (line-at-a-time encode/decode),
`SessionOpenResult` (header-decode outcome).

**`.tlog` is a text format, not binary.** `.gitattributes` already declares
`*.tlog text eol=lf`, grouped with `*.csv` under "Research data is text — diff it," and is
absent from the LFS binary list — first-party evidence this is meant to be line-oriented and
human-diffable. One record per line, `\n`-terminated (the newline is sufficient framing on its
own), fields separated by `|`. An unset `LatencyTrace` tick field is written as `_`
(`RecordFormat.UnsetToken`, mapped directly to `LatencyTrace.Unset` — the same numeric constant —
so decoding needs no special-casing) rather than an opaque sentinel integer, so a diff shows the
word "unset" as plain text.

## Hard constraint (repeated here because it's easy to violate locally)

**No file I/O in this folder, ever.** `SessionWriter`/`SessionReader` encode into a
caller-owned `Span<byte>` and decode from a caller-supplied `ReadOnlySpan<byte>` only, mirroring
`Contracts/ICommandCodec.cs`'s pattern exactly — same "false + required-length on too-small
buffer, no partial state on failure" contract. The actual `.tlog` file handle is opened by the
host: `core/Teleop.Eval/Recording/TlogFileWriter.cs` and `TlogFileReader.cs` own the
`FileStream`/`StreamWriter`/`StreamReader` and call into this folder per line.

## Checksum accounting

`SessionWriter` auto-accumulates an FNV-1a checksum over every line it successfully writes
(it is never asked to write something it doesn't understand, so there's no reason not to).
`SessionReader` requires an explicit `AccumulateChecksum(lineBytes)` call from its caller for
every line, in session order — including lines whose tag the caller doesn't recognize and
chooses to skip for forward compatibility — because the checksum must cover the whole byte
stream to detect truncation, not just the lines this reader happened to successfully decode.

## Golden logs

`core/testdata/golden/*.tlog` fixtures are generated via `dotnet run --project Teleop.Eval --
gen-golden`, never hand-authored. See `core/testdata/CLAUDE.md`.
