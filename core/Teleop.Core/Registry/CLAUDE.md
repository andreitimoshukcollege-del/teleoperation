# Registry

`Registries.cs` — static `string -> factory` tables, one per research axis, hand-maintained.
Reflection-free by design (root CLAUDE.md invariant 5): IL2CPP's stripper removes any type
nothing references directly, and there is no runtime codegen on the headset, so `Activator.CreateInstance`
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
| `JointPredictors` | `Prediction/` (`ICommandAwarePredictor`, docs/adr/0018) | `hold-last`, `servo-replay`, `relay-replay`, `relay-replay-servo`, `franka-replay` (docs/adr/0027), `learned` (docs/adr/0025) |
| `Reconcilers` | `Reconciliation/` | `snap`, `spring`, `budget-blend`, `velocity-match`, `exp-smooth`, `exp-smooth-c1`, `exp-smooth-track` |
| `Codecs` | `Transport/` (`ICommandCodec`) | `raw` |
| `Transports` | `Transport/` (`ITransport`) | `loopback` |
| `PlayoutPolicies` | `Buffering/` | `immediate`, `fixed`, `percentile` |
| `Arbiters` | `Autonomy/` | *(none yet)* |
| `NetworkForecasters` | `Forecast/` (`INetworkForecaster`, docs/adr/0035 N3) | `persistence`, `rtt-half` |

`franka-replay` throws without `config.Shaper` and `learned` without `config.Model`: the host
supplies both.

`Arbiters` is declared and correctly typed despite being empty — adding the first `Autonomy/`
implementation is a one-line addition to an existing table, not a new table to design.

Only `loopback` is registered in `Transports`. `EmulatedTransport`, `MeasuredTransport`,
`BottleneckTransport`, `RateLimitedTransport` and `BacklogBackoffTransport` are decorators with
constructor shapes unlike `LoopbackTransport`'s `(maxPayloadBytes, capacity)` (`EmulatedTransport`
takes another `ITransport`, a `NetworkProfile` and a `SeededRng`). Forcing them into the same
factory signature would be worse than not registering them; give them their own entry shape (or a
small builder type) once a sweep actually needs to select a transport by name. `Transport/CLAUDE.md`
lists them, since `audit` does not check `Transports`.

## Requirements

1. `StringComparer.Ordinal` on every dictionary — a registry lookup must not depend on
   locale/culture behavior differing between `dotnet test` (CoreCLR) and the headset build
   (IL2CPP), the same reasoning `Types/SeededRng.cs` gives for hand-rolling its PRNG instead
   of trusting `System.Random` to agree across runtimes.
2. No entry may exist for a type that doesn't actually exist on disk, and no implementation may
   exist without an entry here — `Teleop.Eval -- audit`'s registry-completeness check enforces
   both directions.
3. Non-generic. Every contract's own doc says `TState` is "typically `Pose`," and nothing in
   this project instantiates one against another state type. A second `TState` is a second
   table, not a generic rewrite.

## Impairments have no table here, on purpose

`Contracts/INetworkImpairment.cs` is a research axis but gets no `Registries.cs` entry, and
`audit`'s `RegistryCompletenessAxes` list deliberately does not include it. Two reasons, and they
are the same ones that keep `EmulatedTransport` out:

- The six constructors take five different shapes — `(long)` twice, `(double, double)`,
  `(double, long)`, `(long[])`, `(long[], bool[], long)`. One factory signature covering them all is a stringly-typed
  parameter bag, which is worse than not registering them.
- The by-name selection this axis needs already exists and is richer than a dictionary could be:
  `Transport/NetworkProfileCatalog.cs` maps a name to a whole *set*, including three regex-driven
  families. The catalog is this axis's registry.

Rather than leave the gap silent, `audit` gains its own check: every `INetworkImpairment`
implementation must be constructed by name in `NetworkProfileCatalog.cs`. That asks the honest
analogue of the registry question — "can this be reached by name?" — and doubles as an IL2CPP
reachability check, since a type nothing constructs is a type full AOT may strip. See
`docs/adr/0013-composable-network-impairments.md` decision 8.
