# 9. Compute JetRover inverse kinematics operator-side, not on the Jetson

## Status

Accepted.

## Context

A new feature lets a VR operator drag a target handle in Unity and have the real JetRover arm
follow it interactively, in addition to the existing occasional CLI use (`move-arm`,
`clocksync-check`). Today, `JetRoverPlant.Command` (`core/Teleop.RobotHost/Plant/JetRoverPlant.cs`)
runs `FourDofArmKinematics.TryInverse` on every accepted `CommandFrame`, on the Jetson Nano's
ARM64 processor. That processor is weak, and an interactive VR loop commands the arm at a far
higher effective rate than an occasional `move-arm` call — the operator/host machine (Windows,
running Unity) is far better positioned to spend CPU on a trig solve than the embedded robot host
is. This is a "new question entirely" per root `CLAUDE.md`'s own rule for architecture changes.

`FourDofArmKinematics`/`ArmLinkLengths` (`core/Teleop.RobotHost/Kinematics/`) lives only in
`Teleop.RobotHost`, a separate `.NET` 8 console project Unity cannot reference. Once IK needs to
run for real on the operator side, Unity needs the *actual* computation, not an approximation —
unlike the visualization-only duplicate first considered for this feature (rejected once the
Unity-computed angles became the authoritative source of what's actually sent to the robot: a
repo whose own stated value is "an algorithm that cannot be evaluated headlessly does not count"
should not carry two independently-maintained copies of the one calculation that decides real
arm motion).

## Decision

**Prediction/reconciliation keeps running on the Cartesian drag-target pose, unmodified. IK runs
once, last, immediately before a command crosses the wire.** `OperatorEndpoint`/
`IPredictor<Pose>`/`IReconciler<Pose>`/`ClockSync` are untouched — this is what keeps "test
different predictors/network profiles" a live capability for this feature, not just for `sweep`.
Only the final, reconciled Cartesian target is converted to joint angles, operator-side.

**`FourDofArmKinematics`/`ArmLinkLengths` move into a new shared package, `core/Teleop.JetRover/`,
structured exactly like `Teleop.Core` itself** (`package.json` + `.asmdef` for Unity, `.csproj`
for `dotnet`, one copy compiled two ways). Moved, not copied — `Teleop.RobotHost` references this
package instead of owning its own copy. Deliberately **not** part of `Teleop.Core`
(`core/Teleop.Core/Plant/CLAUDE.md` is explicit that hardware-specific geometry for one particular
robot does not belong in Core's "family of competing research techniques," and this file has no
`Contracts/` interface or registry entry). Deliberately **not** duplicated into Unity's `Bridge/`
either, for the reason above: it is no longer just a visualization aid.

`TryInverse` gains a `wasClamped` output (computed from whether the pre-clamp reach fell outside
`[minReach, maxReach]`), safe to add now that there is exactly one authoritative copy: a VR
operator dragging past the arm's reach should see that visibly, rather than repeating the same
silent-clamping confusion a real CLI operator hit earlier this session (`robot/README.md`).

**A new wire message, alongside — not replacing — the existing one.** `CommandFrame`/
`RawPoseCodec`/`IRobotPlant`/`RobotEndpoint` keep their exact current meaning: a Cartesian target,
robot-side IK. `RigidBodyPlant`, `move-arm`, and `clocksync-check` all depend on that meaning and
none of them change. `Teleop.JetRover` adds `JointCommandFrame` (Sequence, AckSequence,
CaptureTicks, BaseYaw/LowerPitch/MiddlePitch/UpperPitch in radians, Gripper) and a matching
`JointCommandCodec` — fixed-size binary, versioned, same style as `RawPoseCodec`, but *not* an
`ICommandCodec`: that interface's `TryDecode` is pinned to producing a `CommandFrame`, and forcing
joint angles through a Cartesian-shaped output would mean reconstructing a fake pose robot-side
and re-deriving the angles via forward-then-inverse kinematics — defeating the entire point of
this change.

**`Teleop.RobotHost` runs both paths side by side, on two separate ports, sharing one
`JetRoverPlant` instance.** `JetRoverPlant.Command(in CommandFrame)` is unchanged, for
`move-arm`/`clocksync-check`. It gains `CommandJointAngles(baseYaw, lowerPitch, middlePitch,
upperPitch, gripper, captureTicks)`, which skips `FourDofArmKinematics.TryInverse` entirely and
goes straight to the existing clamp/belief/relay-send tail — refactored out of `Command` into a
private helper both entry points call, so that logic exists in exactly one place. `Program.cs`
keeps its existing `RobotEndpoint`+`UdpTransport`+`RawPoseCodec` pipeline on its existing port
unmodified, and adds a second, small, JetRover-specific receive loop (not `RobotEndpoint`, which
is hardwired to one `ICommandCodec`/`CommandFrame` shape) on a new optional `--joint-local-port`
that decodes `JointCommandFrame`s and calls `plant.CommandJointAngles(...)`. This channel is
uplink-only and sends no reply: a VR operator gets robot state feedback through its own separate
Cartesian `OperatorEndpoint` connection (used for prediction/reconciliation/`ClockSync` regardless
of how the actual command was shaped), so the joint channel doesn't need its own downlink. Two
independent transports were chosen over one port with a frame-kind discriminator byte specifically
to avoid touching `RobotEndpoint`'s existing, already-proven generic dispatch loop at all.

**Also moved, as a byproduct of the same session's related work**: the parametric portion of
`Teleop.Eval/Sweep/NetworkProfileCatalog.cs`'s name-to-`NetworkProfile` resolution (the 4 named
profiles plus the isolated/combined regex families — everything except `synthetic-burst`'s
trace-file loading) into `core/Teleop.Core/Transport/NetworkProfileCatalog.cs`, since the new VR
feature needs the same by-name resolution `sweep` already has, and that logic has zero
`YamlDotNet`/file-I/O dependency — pure string parsing and arithmetic, already anticipated as a
gap by `Registry/CLAUDE.md`. One correctness fix made during the move: the original
`RegexOptions.Compiled` flag is dropped, since `Regex`'s compiled mode uses runtime code
generation that does not work under IL2CPP full-AOT (root `CLAUDE.md` invariant 5) — this class
now gets compiled into Unity for the first time, so a flag that was harmless in `Teleop.Eval`
(`dotnet`-only, never IL2CPP) needed to change to remain correct in its new home.

## Consequences

- **`JetRoverPlant`'s two entry points track staleness independently** (`_lastAcceptedCartesianCaptureTicks`
  for `Command`, `_lastAcceptedJointCaptureTicks` for `CommandJointAngles`). A single shared
  tracker was the original design and was found to be a real bug, not just a theoretical one, via
  real-hardware testing (2026-08-12): `JetRoverOperatorBridge` stamps both its Cartesian
  `SubmitCommand` and its joint-angle send with the *same* `now` in one `Update()` tick, and a
  shared tracker treated the second of the two as a stale duplicate of the first — silently
  dropping roughly half of every joint command even with no other caller involved at all, not
  just under the "two tools running at once" scenario originally anticipated. Running
  `move-arm`/`clocksync-check` and the VR feature at the same time against the same robot process
  remains something to avoid (driving one physical arm from two independent sources at once is
  unsafe regardless of staleness bookkeeping), but it's no longer a source of silent command loss
  on its own.
- `Teleop.RobotHost.csproj` gains a project reference to `Teleop.JetRover.csproj` in place of its
  own `Kinematics/` folder; `unity/TeleopVR/Packages/manifest.json` gains a second local package
  reference (`com.teleop.jetrover`) alongside the existing `com.teleop.core`.
- `Teleop.Eval/Sweep/NetworkProfileCatalog.TryResolve`'s public behavior is unchanged — this is a
  pure relocation of the parametric cases into a function it now delegates to, verified by its
  existing tests staying green and by re-running an existing multi-profile experiment through
  `sweep` before and after.
- **Deliberately unchanged**: `Teleop.Core`'s `CommandFrame`/`RawPoseCodec`/`IRobotPlant`/
  `RobotEndpoint` contracts; `move-arm`/`clocksync-check`; the Phase-4 loopback scene;
  `RobotStateFrame`/downlink shape.
