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
| `Net/` | `UdpTransport` — a real, socket-backed `ITransport` for this host's side of a cross-machine link. A deliberate byte-for-byte duplicate of `Teleop.RobotHost`'s own copy, same precedent as `Time/MonotonicClock`. `UdpRelayCommand` — `udp-relay` |
| `ClockSyncCheck/` | `ClockSyncCheckCommand`, `ClockSyncCheckArgs` — Phase 3's real cross-machine `ClockSync` diagnostic |
| `MoveArm/` | `MoveArmCommand`, `MoveArmArgs` — general-purpose "move the real JetRover arm to a Cartesian target" operator tool, `just move-arm x y z` |
| `ImportUrdf/` | `ImportUrdfCommand`, `UrdfModel`, `SerialChainProfileJson` — `import-urdf`: a URDF to a serial-chain robot profile (docs/adr/0026), `just import-fr3-profile` |
| `IkStress/` | `IkStressRunner`, `OperatorScript`, `ReachOracle`, `IkStressCommand` — `chain-ik-stress`: long synthetic teleoperation sessions of the FR3 Duo through the headset's mapping and IK, scored against a slow multi-start reachability check (docs/adr/0029 amendment 8), `just fr3-ik-stress` |
| `JointDrive/` | `JointDriveCommand`, `DrivePlan`, `DriveWatchdog`, `OperatorLog`, `ChainDriveCommand`, `ChainTeleopProbeCommand` — `joint-drive`: the arm drives itself through scripted joint moves for recorded sessions (docs/adr/0021), `just record-drive` / `just pilot` |

## Subcommands

| Command | Status |
|---|---|
| `verify` | real — replays the golden `.tlog` twice, asserts byte-identical |
| `audit` | real — invariant + registry-completeness check over the built assembly |
| `sweep` | real — runs an `experiments/*.yaml`, writes `results/<id>/<timestamp>/` |
| `gen-golden` | real, not one of the five documented subcommands — regenerates the golden `.tlog` |
| `gen-trace` | real, not one of the five documented subcommands — regenerates a network trace |
| `clocksync-check` | real, not one of the five documented subcommands — Phase 3 of the JetRover integration (`docs/adr/0007-jetrover-plant-and-robot-host.md`); a real cross-machine `ClockSync` diagnostic against an already-running `Teleop.RobotHost`, see `ClockSyncCheck/ClockSyncCheckCommand.cs`'s own doc comment |
| `move-arm` | real, not one of the five documented subcommands — sends a fixed `CommandFrame` to an already-running `Teleop.RobotHost`, repeated until the robot's own reported position converges within `--position-tolerance-meters` of `--x --y --z` (or `--timeout-seconds` elapses, exit non-zero) — never claims success on a fixed duration alone; see `MoveArm/MoveArmCommand.cs`'s own doc comment |
| `joint-drive` | real, not one of the five documented subcommands — drives the real JetRover through a planned session of joint moves (docs/adr/0021): every pose checked against the collision model first, the IK's ranges less a margin, a stall and stream watchdog, Enter to stop; writes `operator.jlog`. `--dry-run` plans and sends nothing. See `JointDrive/JointDriveCommand.cs`'s own doc comment |
| `chain-drive` | real, not one of the five documented subcommands — drives a serial-chain RobotHost (the simulated FR3 Duo, docs/adr/0027) through seeded joint-space waypoints at 48 Hz, writes `operator.jlog`, Enter stops it back at its start pose; refuses to run without `--simulated-robot` until the serial-chain collision model exists. `just fr3-sim-drive`, `just fr3-sim-record` |
| `chain-teleop-probe` | real, not one of the five documented subcommands — the headset's two-hand FR3 operator (docs/adr/0029) with scripted hands, over a real socket to a running serial-chain RobotHost: `SerialChainJointLink` and `SerialChainTeleop` exactly as the Galaxy XR runs them. Passes when each measured tool reaches its clutch target, the gripper closes, nothing is blocked and the twin is within 1° p95 of each reading at its instant; exits 2 when no robot answers. Simulated robots only. `just fr3-teleop-probe` |
| `chain-ik-stress` | real, not one of the five documented subcommands — long synthetic FR3 Duo teleop sessions, headless (docs/adr/0029 amendment 8): a scripted operator through `BodyAnchoredMapping` and `SerialChainTeleop` at 90 Hz; reports off-target time on reachable targets, settle times after the hand stops, the recovery's work, and stuck / wrong-way episodes, and exits 1 on any. Touches no robot. `just fr3-ik-stress` |
| `import-urdf` | real, not one of the five documented subcommands — writes a serial-chain robot profile from a URDF (docs/adr/0026 decision 3); `just import-fr3-profile` regenerates `core/RobotProfiles/fr3-duo.json`, and `ImportUrdfCommandTests` requires the committed bytes. Touches no robot |
| `discover-robot` | real, not one of the five documented subcommands — finds the robot as the headset does at start (docs/adr/0022), with the same `RobotAddressResolver`; `just robot-discover`. Sends only discovery queries |
| `udp-relay` | real, not one of the five documented subcommands — forwards a headset's UDP (the joint, Cartesian and camera ports) from this machine to a host it can reach and the headset cannot: Isaac's RobotHost on the lab's GPU machine over this PC's VPN, for a headset on this PC's Wi-Fi. Bytes pass unchanged; a new sender on a port takes it over; Enter stops it. Adds this PC's hop to every delay. `just isaac-relay` |
| `headset-replay` | real, not one of the five documented subcommands — a real JetRover headset session replayed offline through the headset's own predictors with and without the network forecast (docs/adr/0035 N3): RobotHost's `robothost.jlog` (frames sent, commands) joined by sequence with the headset's `.ltrace` (arrivals); ClockSync and the forecaster as the link runs them; every default preset scored over the session by `LiveJointScorer`. Prints; one session is not a result. `just headset-replay` |
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
