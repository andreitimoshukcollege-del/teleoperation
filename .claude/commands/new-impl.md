---
description: Scaffold a new algorithm implementation in Teleop.Core with test, registry entry, and benchmark row
argument-hint: <contract> <name> — e.g. predictor ukf | reconciler velocity-match | codec trajectory
allowed-tools: Read, Write, Edit, Glob, Grep, Bash(dotnet:*), Bash(ls:*), Bash(find:*)
model: opus
---

Implement a new `$1` named `$2` in `core/Teleop.Core`.

## Routing

| `$1` | Interface | Folder | Registry table |
|---|---|---|---|
| `predictor` | `IPredictor<TState>` | `Prediction/` | `Predictors` |
| `reconciler` | `IReconciler<TState>` | `Reconciliation/` | `Reconcilers` |
| `playout` | `IPlayoutPolicy` | `Buffering/` | `PlayoutPolicies` |
| `codec` | `ICommandCodec` | `Transport/Codecs/` | `Codecs` |
| `arbiter` | `IAutonomyArbiter` | `Autonomy/` | `Arbiters` |
| `transport` | `ITransport` | `Transport/` | `Transports` |

If `$1` is not in this table, stop and ask — a genuinely new axis needs a new `Contracts/`
interface and an ADR, which is a different and larger task.

## Steps

1. Read the root `CLAUDE.md`, then the `CLAUDE.md` in the target folder. Read the interface in
   `Contracts/`. Read one existing implementation in the folder to match its conventions.
2. Check whether `$2` (or something equivalent under another name) already exists, including
   in the folder's "Tried and rejected" section. If it does, say so and stop.
3. Write the implementation: `sealed`, config-driven, no magic numbers, allocation-free on the
   hot path, deterministic. No `UnityEngine`, no clock reads, no I/O, no unseeded randomness,
   nothing past C# 9.
4. Add the registry entry by hand. Never reflection.
5. Write tests covering, at minimum: determinism under repeated identical input; `Reset()`
   restoring as-constructed state; out-of-order observations; duplicate observations; a gap of
   several hundred ms; zero allocation on the hot path.
6. Add the benchmark row so it appears in the standard sweep.
7. Update the folder `CLAUDE.md` "Implemented" table.
8. Run all three and paste the output:
   `dotnet test` · `dotnet run --project Teleop.Eval -- verify` · `... -- audit`

## Report

The registry key, the files touched, the test names, the verification output, and any modeling
assumption you made — noise model, motion model, coordinate convention, parameter defaults.
State those explicitly; unstated assumptions are what silently invalidate results later.
