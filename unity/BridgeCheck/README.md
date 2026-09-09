# BridgeCheck

`just bridge-check` compiles the real `Bridge/` sources against the real Core assemblies,
headlessly. It exists because `unity/` silently stopped compiling against `main` twice on
2026-09-09 — Core changed an interface, nothing failed, and the breakage was only discoverable by
opening the Unity editor.

This folder is **not** part of the Unity project. Unity only opens `unity/TeleopVR/`, so a sibling
directory here is invisible to it and cannot affect an editor session or a Quest build.

## What it proves

That `Bridge/` and Core still agree on their API, under Unity's own compilation rules —
`netstandard2.1` at `LangVersion 9.0`, matching root `CLAUDE.md` invariant 6. It therefore catches:

- a Core constructor that gained a parameter (this is the real case: ADR 0012 added a required
  `IPlayoutPolicy<Pose>` to `OperatorEndpoint`, and both bridges kept calling the old signature)
- a renamed or removed Core method, property, or type
- a Bridge file using a C# 10+ feature invariant 6 bans — a file-scoped namespace, `required`,
  a collection expression — which would compile under a modern `dotnet` and break the Quest build

It uses `ProjectReference`, not a `HintPath` to build output, specifically so it cannot pass
against a stale Core.

## What it does NOT prove

**This is not a substitute for opening the editor, and a green run must never be reported as
one.** It cannot see:

- anything about Unity's actual behavior: serialization, Inspector drawers, scene wiring, prefabs,
  `[SerializeField]` fields that reference the wrong object
- IL2CPP/AOT failures, which appear on device only (invariant 5)
- XR, rendering, TextMeshPro, and the packages under `Assets/` generally
- whether the *stubs* match Unity's real API — see below

## The stubs are the cost

`UnityStubs.cs` is hand-written. That is the price of the gate, and it cuts both ways:

- **False positive:** a Bridge file starts using a Unity member no stub has, and the build fails on
  the stub rather than on a real problem. Expected, cheap — add the member.
- **False negative, and the one that matters:** a stub whose signature has *drifted* from the real
  UnityEngine API lets code compile here that Unity would reject. This is why `UnityStubs.cs`'s
  header insists signatures be copied from the real API rather than invented until the error goes
  away. A gate that manufactures confidence is worse than no gate (invariant 10).

## Covered files

The compiled set is listed explicitly in `Teleop.BridgeCheck.csproj` — deliberately not a glob,
so one unrelated new Bridge file cannot turn a clean run into a wall of stub errors.

Covered today: `TeleopOperatorBridge`, `TeleopRobotBridge`, `JetRoverOperatorBridge`,
`JetRoverArmConfig`, `RobotArmProfileData`, `NetworkImpairmentSettings`, `SwappableTransport`,
`NetworkImpairmentController` — i.e. every Bridge file that constructs or calls into Core, which is
where this class of breakage lands.

Not covered: `LatencyHud` and `JetRoverConnectionHud` (TextMeshPro, display-only, no Core
constructors), `JetRoverArmRig` (Unity Transform maths, no Core API), and the adapters stubbed in
`UnityStubs.cs` (`UnityMetricSink`, `UnityMonotonicClock`, `ConfigLoader`, `CoordConversion`,
`XrDisplayTimeProvider`, `UdpTransport`). Add a file to the covered set whenever it starts touching
a Core API; that is the trigger, not file count.

## Verifying the gate itself

A gate nobody has seen fail is not known to work. This one was checked by swapping in the
pre-`dd0cee1` `TeleopOperatorBridge.cs` and confirming it exits non-zero with
`CS7036: no argument given for required parameter 'playoutPolicy'`. Worth repeating after any
significant change to this project:

```bash
F=unity/TeleopVR/Assets/Teleop/Runtime/Bridge/TeleopOperatorBridge.cs
git show f7cf9b7:$F > "$F.tmp" && mv "$F.tmp" "$F"
just bridge-check          # must FAIL, non-zero
git checkout -- "$F"
just bridge-check          # must pass again
```
