# Registry

`Registries.cs` — static `string -> factory` tables, one per research axis, hand-maintained.
Reflection-free by design (root CLAUDE.md invariant 5): IL2CPP's stripper removes any type
nothing references directly, and there is no runtime codegen on Quest, so `Activator.CreateInstance`
is not an option. Every entry here is a plain dictionary literal calling a real constructor.

## Adding an entry

When `/new-impl` scaffolds a new implementation, it adds one line to the matching dictionary
here by hand. Never generate this file, never populate it via reflection over a folder's
contents — the whole point is that this is the one place a human states "this name means this
type," so the mapping survives whatever IL2CPP strips.

## Current entries

| Property | Axis | Entries |
|---|---|---|
| `Predictors` | `Prediction/` | `none`, `const-vel`, `double-exp` |
| `Reconcilers` | `Reconciliation/` | `snap` |
| `Codecs` | `Transport/` (`ICommandCodec`) | `raw` |
| `Transports` | `Transport/` (`ITransport`) | `loopback` |
| `PlayoutPolicies` | `Buffering/` | *(none yet)* |
| `Arbiters` | `Autonomy/` | *(none yet)* |

`PlayoutPolicies` and `Arbiters` are declared and correctly typed despite being empty — adding
the first `Buffering/`/`Autonomy/` implementation is a one-line addition to an existing table,
not a new table to design.

`EmulatedTransport` is deliberately not registered in `Transports` yet: it is a decorator over
another `ITransport` plus a `NetworkProfile` and a `SeededRng`, a materially different
constructor shape than `LoopbackTransport`'s `(maxPayloadBytes, capacity)`. Forcing it into the
same factory signature would be worse than not registering it; give it its own entry shape (or a
small builder type) once a sweep actually needs to select a transport by name.

## Requirements

1. `StringComparer.Ordinal` on every dictionary — a registry lookup must not depend on
   locale/culture behavior differing between `dotnet test` (CoreCLR) and the Quest build
   (IL2CPP/Mono), the same reasoning `Types/SeededRng.cs` gives for hand-rolling its PRNG instead
   of trusting `System.Random` to agree across runtimes.
2. No entry may exist for a type that doesn't actually exist on disk, and no implementation may
   exist without an entry here — `Teleop.Eval -- audit`'s registry-completeness check enforces
   both directions.
3. Non-generic. Every contract's own doc says `TState` is "typically `Pose`," and nothing in
   this project instantiates one against another state type. A second `TState` is a second
   table, not a generic rewrite.
