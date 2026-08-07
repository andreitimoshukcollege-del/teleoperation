# Teleop.Eval

The headless CLI: `verify`, `audit`, `sweep`, `replay`, `compare`, plus two undocumented
`gen-*` tools. A **host** — unlike `core/Teleop.Core/`, this project is allowed to touch a
wall clock, do real file I/O, and take NuGet dependencies (`YamlDotNet`, so far).

## Layout

| Folder | Holds |
|---|---|
| `Time/` | `MonotonicClock` — the `Stopwatch`-backed `ITimeAuthority` Core is forbidden from having |
| `Metrics/` | `CsvMetricSink` — the file-writing `IMetricSink` Core is forbidden from having |
| `Recording/` | `TlogFileWriter`/`TlogFileReader` — own the actual `.tlog` file handle `Recording/RecordFormat.cs` (Core) is forbidden from touching |
| `Verification/` | `VerifyCommand`, `AuditCommand` — Gate 3's two real checks |
| `Sweep/` | `ExperimentConfig`, `NetworkProfileCatalog`, `TraceFile`, `SweepCommand`, `ManifestWriter` — Gate 5's sweep |
| `Tooling/` | `GoldenSessionBuilder`, `SyntheticTraceBuilder` — deterministic generators for committed fixtures, never hand-authored data |
| `Net/` | `UdpTransport` — a real, socket-backed `ITransport` for this host's side of a cross-machine link. A deliberate byte-for-byte duplicate of `Teleop.RobotHost`'s own copy, same precedent as `Time/MonotonicClock` |
| `ClockSyncCheck/` | `ClockSyncCheckCommand`, `ClockSyncCheckArgs` — Phase 3's real cross-machine `ClockSync` diagnostic |

## Subcommands

| Command | Status |
|---|---|
| `verify` | real — replays the golden `.tlog` twice, asserts byte-identical |
| `audit` | real — invariant + registry-completeness check over the built assembly |
| `sweep` | real — runs an `experiments/*.yaml`, writes `results/<id>/<timestamp>/` |
| `gen-golden` | real, not one of the five documented subcommands — regenerates the golden `.tlog` |
| `gen-trace` | real, not one of the five documented subcommands — regenerates a network trace |
| `clocksync-check` | real, not one of the five documented subcommands — Phase 3 of the JetRover integration (`docs/adr/0007-jetrover-plant-and-robot-host.md`); a real cross-machine `ClockSync` diagnostic against an already-running `Teleop.RobotHost`, see `ClockSyncCheck/ClockSyncCheckCommand.cs`'s own doc comment |
| `replay` | stub, exits 70 (`EX_SOFTWARE`) — not built yet |
| `compare` | stub, exits 70 — not built yet |

`replay`/`compare` staying stubbed is root `CLAUDE.md` invariant 10 in action: an unimplemented
check must exit non-zero, never fake a pass.

## `sweep`'s scope, precisely

Runs the (predictor × network profile × seed) matrix an experiment config describes through the
same loopback pipeline `Teleop.Core.Tests`' `LoopbackPipelineIntegrationTests` already proves
correct, and writes two things: raw `name,value,ticks` rows via `CsvMetricSink`, and
`manifest.json` (schema in `results/CLAUDE.md`). It does **not** compute a percentile table —
per `.claude/commands/run-sweep.md`'s own step split, that aggregation happens after `sweep`
runs, from the raw CSV, not inside this tool.

It also emits `prediction_position_error_mm`/`prediction_orientation_error_deg`: an **online**
proxy (predictor's live estimate vs. the plant's simultaneous ground truth), not
`docs/metrics.md` §4's full counterfactual, horizon-binned methodology. That fuller mechanism
needs an offline `.tlog` replay scorer that does not exist yet — see the metric definitions in
`docs/metrics.md` for the exact distinction. Gate 5 needs "prediction error and correction cost
reported together"; this metric is what makes that literally true for this pass, honestly
labeled as a simplification rather than silently standing in for the real thing.
