# Teleop.Core

Every algorithm in the project lives here. This assembly is compiled by **two** build systems
from these same `.cs` files: `dotnet` (via `Teleop.Core.csproj`, for tests and eval) and Unity
(via `package.json` + `Teleop.Core.asmdef`, for the Quest build). There is one copy. It cannot
drift. That property is the reason the project is laid out this way — protect it.

## Layout

- `Contracts/` — **interfaces only.** One file per research axis. `ls Contracts/` answers
  "what are the swappable parts of this system?"
- `Prediction/`, `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/` — implementations
  of exactly one contract each. `ls` answers "what have we tried on this axis?"
- `Types/` — value types crossing the wire. Immutable, `System.Numerics`, ROS convention.
- `Time/` — `ITimeAuthority` and its implementations. Nothing else reads a clock.
- `Pipeline/` — composition. The wiring diagram, expressed in code.
- `Registry/Registries.cs` — static `string -> factory` tables. Hand-maintained.
- `Recording/` — versioned `.tlog` format, reader and writer.
- `Metrics/` — trackers and sinks. Definitions live in `docs/metrics.md`.

## Hard constraints (repeated here because they are easy to violate locally)

- No `UnityEngine`. No `System.IO`. No sockets. No threads. No `DateTime`. No `new Random()`.
- No NuGet packages. Declare an interface instead and let the host implement it.
- `netstandard2.1`, `LangVersion 9.0` (Unity 2022.3 is C# 9). Do not modernize the csproj.
  Block-scoped namespaces only — `namespace X;` is C# 10 and breaks the Unity build.
- Reflection-free construction. Add to `Registries.cs` by hand.

## Style

- `sealed` by default. Interfaces over inheritance.
- Constructor-injected dependencies; no service locators, no statics with mutable state
  (Unity wipes statics on domain reload, and statics break sweep determinism).
- Struct + `in`/`ref readonly` for hot-path types. Fixed-size buffers over `List<T>`.
- `Reset()` on every stateful component, and a test proving `Reset()` returns it to the
  as-constructed state. Sweeps reuse instances across trials.
- Diagnostics via a `Diagnostics` property returning a struct — never a log call.

## Adding an implementation

Use `/new-impl <contract> <name>`. It scaffolds the file, the test, the registry entry, and
the benchmark row together, which is the only combination that counts as complete.
