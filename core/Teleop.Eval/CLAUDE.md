# Teleop.Eval

The headless CLI: the gates `verify` and `audit`, `sweep`, the stubs `replay` and `compare`, and
the fixture generators, robot tools and benches in the tables below. A **host** — unlike
`core/Teleop.Core/`, this project is allowed to touch a wall clock, do real file I/O, and take
NuGet dependencies (`YamlDotNet`, so far).

## Layout

| Folder | Holds |
|---|---|
| `Time/` | `MonotonicClock` — the `Stopwatch`-backed `ITimeAuthority` Core is forbidden from having |
| `Metrics/` | `CsvMetricSink` — the file-writing `IMetricSink` Core is forbidden from having |
| `Recording/` | `TlogFileWriter`/`TlogFileReader` — own the actual `.tlog` file handle `Recording/RecordFormat.cs` (Core) is forbidden from touching |
| `Verification/` | `VerifyCommand`, `AuditCommand` — Gate 3's two real checks |
| `Sweep/` | `ExperimentConfig`, `NetworkProfileCatalog`, `TraceFile`, `SweepCommand`, `ManifestWriter`, `ResolvedStack` — Gate 5's sweep |
| `Tooling/` | `GoldenSessionBuilder`, `SyntheticTraceBuilder`, `TrainerGoldenBuilder` — deterministic generators for committed fixtures, never hand-authored data (`gen-golden`, `gen-trace`, `gen-trainer-golden`). `CatalogCommand` — `catalog`: every registry entry and network profile a sweep can name, as JSON |
| `Net/` | `UdpTransport` — a real, socket-backed `ITransport` for this host's side of a cross-machine link. Its own copy rather than a reference to `Teleop.RobotHost`, same precedent as `Time/MonotonicClock`. It began as a copy of RobotHost's and keeps only its fixed-peer mode; RobotHost's has since added reply-to-sender (#57). `UdpRelayCommand` — `udp-relay` |
| `ClockSyncCheck/` | `ClockSyncCheckCommand`, `ClockSyncCheckArgs` — Phase 3's real cross-machine `ClockSync` diagnostic |
| `MoveArm/` | `MoveArmCommand`, `MoveArmArgs` — general-purpose "move the real JetRover arm to a Cartesian target" operator tool, `just move-arm x y z`. `DiscoverRobotCommand` — `discover-robot` |
| `ImportUrdf/` | `ImportUrdfCommand`, `ImportUrdfArgs`, `UrdfModel`, `SerialChainProfileJson` — `import-urdf`: a URDF to a serial-chain robot profile (docs/adr/0026), `just import-fr3-profile`. `FitCapsulesCommand` — `fit-capsules`: the profile's collision capsules fitted to the URDF's collision geometry (docs/adr/0026 amendment) |
| `IkStress/` | `IkStressRunner`, `OperatorScript`, `ReachOracle`, `IkStressCommand` — `chain-ik-stress`: long synthetic teleoperation sessions of the FR3 Duo through the headset's mapping and IK, scored against a slow multi-start reachability check (docs/adr/0029 amendment 8), `just fr3-ik-stress` |
| `JointDrive/` | `JointDriveCommand`, `DrivePlan`, `DriveWatchdog`, `OperatorLog`, `CourseScript`, `TakeClient`, `ChainDriveCommand`, `ChainTeleopProbeCommand` — `joint-drive`: the arm drives itself through scripted joint moves for recorded sessions (docs/adr/0021), `just record-drive` / `just pilot`; `TakeClient` records a drive as a take (`--take`, docs/adr/0033, docs/adr/0036) |
| `ArmPower/` | `ArmPowerCommand` — `arm-power`: ROBOT OFF / ROBOT ON from the PC (docs/adr/0023 decision 5), `just robot-off` / `just robot-on` |
| `BuildProfile/` | `BuildProfileCommand`, `ProfileBuilderWizard`, `RobotArmProfileJson` — `build-profile`: an interactive wizard that writes a robot arm profile (docs/adr/0011), `just build-profile` |
| `JointBench/` | `JointPredictBenchCommand` — `joint-predict-bench`: the joint-space predictors' benchmark row against a simulated servo arm (docs/adr/0018). `HeadsetReplayCommand` — `headset-replay` (docs/adr/0035 N3) |
| `JointProbe/` | `JointProbeCommand` — `joint-probe`: what a RobotHost's joint-state stream carries, `just joint-probe`. `WorldProbeCommand` — `world-probe`: checks the world a serial-chain RobotHost streams (docs/adr/0031 amendment 2), `just fr3-world-probe` |

## Subcommands

| Command | Status |
|---|---|
| `verify` | real — replays the golden `.tlog` twice, asserts byte-identical |
| `audit` | real — invariant + registry-completeness check over the built assembly |
| `sweep` | real — runs an `experiments/*.yaml`, writes `results/<id>/<timestamp>/` |
| `gen-golden` | real, not one of the five documented subcommands — regenerates the golden `.tlog` |
| `gen-trace` | real, not one of the five documented subcommands — regenerates a network trace |
| `gen-trainer-golden` | real — writes the golden files that pin the trainer's Python copies of Core (docs/adr/0021 decision 5) to `trainer/contract/golden/`. `just gen-trainer-golden` |
| `catalog` | real — prints every registry entry and network profile a sweep can be configured with, as JSON, read live from `Registries` and `NetworkProfileCatalog`; the experiment GUI reads it |
| `build-profile` | real — interactive wizard that writes a robot arm profile to `RobotProfiles/<name>.json` (docs/adr/0011). `just build-profile` |
| `fit-capsules` | real — fits a serial-chain robot's collision capsules to its URDF's collision geometry, byte-stable (docs/adr/0026 amendment); `FitCapsulesCommandTests` requires the committed file. Touches no robot |
| `joint-predict-bench` | real — scores every registered `ICommandAwarePredictor` against a simulated servo arm and network, deterministic: `joint_prediction_error_deg` and `joint_correction_deg` (docs/metrics.md §4) |
| `joint-probe` | real — listens to a RobotHost's joint-state stream for a few seconds and reports frame rate, round trip, and which joints are sensed. Sends only the keepalive. `just joint-probe` |
| `world-probe` | real — checks the world a serial-chain RobotHost streams: status names the scene file, every object posed within 2 s, frames at 50 ± 5 Hz; optional layout and move checks. Sends no joint command. `just fr3-world-probe` |
| `clocksync-check` | real, not one of the five documented subcommands — Phase 3 of the JetRover integration (`docs/adr/0007-jetrover-plant-and-robot-host.md`); a real cross-machine `ClockSync` diagnostic against an already-running `Teleop.RobotHost`, see `ClockSyncCheck/ClockSyncCheckCommand.cs`'s own doc comment |
| `move-arm` | real, not one of the five documented subcommands — sends a fixed `CommandFrame` to an already-running `Teleop.RobotHost`, repeated until the robot's own reported position converges within `--position-tolerance-meters` of `--x --y --z` (or `--timeout-seconds` elapses, exit non-zero) — never claims success on a fixed duration alone; see `MoveArm/MoveArmCommand.cs`'s own doc comment |
| `joint-drive` | real, not one of the five documented subcommands — drives the real JetRover through a planned session of joint moves (docs/adr/0021): every pose checked against the collision model first, the IK's ranges less a margin, a stall and stream watchdog, Enter to stop; writes `operator.jlog`. `--dry-run` plans and sends nothing. Before moving it switches torque on (`arm-power` hold), and from an arm resting on its support it frees it with the smallest single-joint move first (`DrivePlan.TryApproach`). Scripts: `waypoints`, `steps`, `course` (the headset's training course, `CourseScript`). See `JointDrive/JointDriveCommand.cs`'s own doc comment |
| `arm-power` | real, not one of the five documented subcommands -- the headset's ROBOT OFF / ROBOT ON from the PC (docs/adr/0023 decision 5): `--request relax` parks the arm at RobotHost's rest pose and switches torque off, `--request hold` switches it back on; repeats the operator message (as a tool) until RobotHost's status answers, exits non-zero on a refusal or a timeout. `just robot-off`, `just robot-on`. **Relax moves the arm** |
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
