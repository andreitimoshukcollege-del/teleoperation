# Teleop.Bridge — the shared Unity package (`com.teleop.bridge`)

The only place Unity and Core meet. Shared by `unity/TeleopVR` (Quest, Unity 2022.3, built-in
pipeline) and `unity/TeleopXR` (Galaxy XR, Unity 6.3, URP) per docs/adr/0016, so **every change
here must compile and behave under both editors and both pipelines**. See `unity/CLAUDE.md` for
how that is checked. Changes here want human review: this is where real I/O lives.

Layout: `package.json`, and `Runtime/` (the `Teleop.Bridge` asmdef, every `.cs`, and `Resources/`
with the default `jetrover_connection.json` and `display_calibration.json`, which are overridden
per device from `Application.persistentDataPath`). The `.meta` files are tracked. Their GUIDs are what
the projects' scenes reference, so never regenerate or delete them.

## Bridge/ contains exactly two kinds of file

**Adapters** — drive Core from Unity callbacks: `TeleopOperatorBridge`, `TeleopRobotBridge`,
`CoordConversion`, `ConfigLoader`, `XrDisplayTimeProvider`, `LatencyHud` (display only, reads the
metric sink and writes nothing back), `CameraFeedBridge` (the robot camera on a world-locked panel,
docs/adr/0014; every reassembly decision is Core's `CameraFrameReassembler`).
`DisplayCalibrationConfig` is the plain data type `ConfigLoader` loads, not an adapter itself.

**Implementations of Core interfaces** — the direction inverts here. Core declares, Unity
provides: `UnityRobotPlant : IRobotPlant` (not yet built — Phase 4 reuses Core's own
`RigidBodyPlant` directly instead, see `Plant/CLAUDE.md`), `UdpTransport : ITransport` (not yet
built — Phase 4 is in-process only), `UnityMetricSink : IMetricSink` (built),
`UnityMonotonicClock : ITimeAuthority` (built; every Core component Bridge constructs needs one,
the same reasoning `Time/CLAUDE.md` gives for why `Teleop.Eval` has its own `MonotonicClock`),
and eventually an `IInferenceBackend` (see the Sentis note in `unity/TeleopVR/Assets/Teleop/CLAUDE.md`).

Bridge should stay small: roughly a dozen files, mostly under 100 lines. Growth means logic is
leaking out of Core, and every leaked line is a line the headless sweeps can no longer test.

## Network impairment: the checkbox panel

Lets an operator switch lag, jitter, loss and reordering on and off during Play mode, so "what does
200ms feel like?" can be answered in a headset instead of argued about.

- `NetworkImpairmentSettings` — the sliders and checkboxes, and a `CreateImpairments` that builds
  the Core impairment objects from them. Same category as `RobotArmProfileData`: a Unity-serializable
  surface feeding types Unity cannot serialize itself.
- `NetworkProfilePresets` — loads a frozen profile's values into the axes (see below).
- `DelayTraceLoader` — reads a recorded `.trace`, since Core cannot do I/O.
- `SwappableTransport : ITransport` — installs and removes an `EmulatedTransport` over its inner
  transport while running. Needed because the endpoints take their transports once, in their
  constructors, so without it every settings change would rebuild the endpoint stack and throw away
  `ClockSync`'s convergence and the recording with it.
- `NetworkImpairmentController` — the MonoBehaviour holding the checkboxes, wired to
  `TeleopOperatorBridge`'s two transports.

**Four files in a folder whose rule above is that growth is a warning sign, so the justification
matters.** None of them impair anything. Every delay, drop and reorder decision is
made by Core's `EmulatedTransport` from a Core `NetworkProfile`; what these add is a serializable
surface for it and one level of indirection so it can be swapped at runtime. The count is spread
across small single-purpose files precisely so each stays a description rather than a computation.
If a coefficient, a distribution, or a drop decision ever appears in any of them, that is the leak
this section's rule is about and it belongs back in Core.

### Where the impairments actually live

**In Core**, one file each: `core/Teleop.Core/Transport/Impairments/`. That is where a new kind of
disturbance is added, and it is the only place any impairment model exists.

Nothing under `Bridge/` mirrors that structure, deliberately. Unity uses Core directly all over —
`EmulatedTransport`, `OperatorEndpoint`, `ClockSync`, and the Core impairments themselves are all
constructed here. **There is exactly one thing Unity cannot do with a Core object: draw it in the
Inspector and save it into a scene.** Unity's serializer only persists public mutable fields, and
Core's impairments keep their parameters private and readonly so they can be validated once at
construction and never be wrong afterwards.

So `NetworkImpairmentSettings` is a flat block of sliders and checkboxes — the boxes an operator
types into — with one `CreateImpairments` that builds the Core objects. That is the whole Unity
side. There was briefly a per-axis class hierarchy under `Bridge/Impairments/` mirroring Core's
layout; it was removed, because a plain number does not need its own class or file, and the
duplicated shape read as though the impairments had never moved to Core at all.

**Adding an axis:** a new file in Core implementing `Contracts/INetworkImpairment.cs`, plus a field
and one `if` in `NetworkImpairmentSettings.CreateImpairments`.

### Presets, and how this relates to the sweeps

The impairment runs through the identical `EmulatedTransport` the sweeps use — same Gilbert-Elliott
loss chain, same uniform jitter draw, both directions, decorrelated seeds. `SweepCommand` and
`TeleopOperatorBridge` build structurally identical stacks. **The mechanism was never the
difference; only how the numbers were authored was.**

`NetworkProfilePresets` closes that in the safe direction. A preset dropdown loads a frozen
profile's exact values — read from `NetworkProfileCatalog`, never duplicated here — into the
per-axis fields. So "feel what `300ms-60j-2loss-bursty` is like" uses the numbers the sweep citing
that name used. Every axis then stays independently editable, because "start from that profile and
halve the jitter" is a real question a preset list can't enumerate.

The moment any axis differs from what the preset loaded, the HUD/log says `<name> (modified)`. The
settings genuinely are no longer that profile, and a summary that kept the bare name would make a
recording look comparable to a sweep it isn't comparable to.

**The frozen suite is never authored from here.** ADR 0004 records its numbers so a manifest and
the documentation can't drift; a preset reads that suite, it does not write it.

`synthetic-burst` is reachable too, via the `DelayTrace` axis — its bursts live in recorded samples and
are not expressible as base+jitter, which is exactly why it matters (the Buffering result rests on
it). `DelayTraceLoader` reads the file, since Core cannot do I/O. Two things about it:

- **No copy of the trace is committed under `unity/`.** `core/testdata/traces/` is the one source;
  `just install-traces` copies it to `Application.persistentDataPath` for the Editor, and `adb
  push` does the same on device. A tracked duplicate would drift silently.
- **The loader rescales samples between tick rates, and that is not cosmetic.** The header records
  the writing machine's rate (10,000,000 on Windows, 1,000,000,000 on Linux ARM64). Replaying a
  Windows-written trace on a Quest unrescaled would inflate every delay 100× while each individual
  number still looked plausible — the exact failure already recorded in `robot/README.md`'s
  ClockSync finding.

In trace mode the delay and jitter axes are switched off rather than left ticked and ignored:
`EmulatedTransport`'s trace constructor rejects a profile carrying either, on the grounds that
synthetic jitter on an already-recorded delay double-models the same variance.

Two things that are deliberate rather than incidental:

- **Only the loopback path is wired.** `TeleopOperatorBridge` owns both directions in-process, so
  impairing them is honest. `JetRoverOperatorBridge` is deliberately left out: `EmulatedTransport`
  impairs on the *receiving* side, so impairing what the real robot receives means wrapping
  `Teleop.RobotHost`'s transport on the Jetson. Wiring it there anyway would give a checkbox that
  moves the HUD's numbers while the physical arm behaves identically — worse than no checkbox.
- **Changing conditions mid-session invalidates the recording.** The impairment state is not
  recoverable from a `.tlog`, so every change is logged with its tick. This control is for feeling
  out the parameter space and for demos. Citable numbers come from `Teleop.Eval` sweeps against the
  frozen, *named* profile suite (`docs/adr/0004`–`0006`) — which is also why this composes an
  ad-hoc profile instead of making that suite mutable.

## Camera feed (`CameraFeedBridge`, docs/adr/0014)

Shows the JetRover's camera on a world-locked panel. How each project wires it is in that project's
CLAUDE.md. These rules hold for both:

- **The panel's material is a material asset**, never a `Shader.Find`. A shader nothing references
  is stripped from IL2CPP builds, and the panel then renders pink on device only. Use
  `Unlit/Texture` in the built-in pipeline and `Universal Render Pipeline/Unlit` in URP.
  `CameraFeedBridge` assigns `material.mainTexture`, which reaches either.
- **Never parent the panel to the camera** (ADR 0014 §7). With **Arm Rig** set, the panel is moved to
  **Offset From Arm Base** at start; leave it empty when the scene places the panel itself.
- **Robot address:** the same `jetrover_connection` config as `JetRoverOperatorBridge`. The robot
  must be running the sender: `teleop-camerahost.service` (`just deploy-camerahost`,
  `just install-camerahost-service`) or `just camera-serve` for a one-off.
- **Logging:** every 10 s the Console logs `[camera] ...: received N frames, shown N, ... last decode X ms`.
  **On a headset, watch `last decode`.** `Texture2D.LoadImage` decodes on the main thread, and it is
  the one cost here that can take frame time from the render loop. If it is a meaningful share of
  the frame budget, lower **Max Frames Per Second** (the sender then thins frames by capture time)
  before anything else.
- **Windows Firewall does not get in the Editor's way**, even though it could seem to. On SINRG
  WIFI (classed as Public) the Unity Editor's rule *blocks* inbound, but that rule only stops
  traffic Unity has not asked for. The frames come back from exactly the address and port the
  keepalive went to (`robot:6003` → local 6004, every second), and the firewall lets those in as
  replies. The robot's state replies come back the same way (`robot:6000` → local 6001). Confirmed
  on 2026-10-02 with the block rule in place. It would break only if a sender answered from a
  different port, or if the keepalive stopped for long enough that the firewall forgot it.

No `camera_*` metrics are recorded yet. They are defined in `docs/metrics.md` §9 and computed by
Core's `CameraLatencyRecorder`; wiring them needs the pose path's `ClockSync` and a `t_render`
stamp, and is a follow-up.
