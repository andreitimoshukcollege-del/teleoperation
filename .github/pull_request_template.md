## What and why

<!-- One paragraph. If this changes an algorithm, name the research axis it belongs to. -->

## Research axis

<!-- prediction / reconciliation / buffering / transport / autonomy / view-synthesis /
     infrastructure / analysis -->

## Core invariants

- [ ] New implementation registered in `Registry/Registries.cs` (static table, not reflection)
- [ ] Unit test + benchmark row added
- [ ] No `UnityEngine`, no wall-clock read, no I/O, no unseeded RNG in Core
- [ ] Nothing past C# 9; `Teleop.Core.csproj` still `netstandard2.1`
- [ ] No new NuGet dependency in Core
- [ ] No allocation on the per-frame hot path (allocation test passes)
- [ ] Folder `CLAUDE.md` "Implemented" table updated

## Verification

<!-- Paste actual output. A successful build is not verification. -->

```
dotnet test                                 ->
dotnet run --project Teleop.Eval -- verify  ->
dotnet run --project Teleop.Eval -- audit   ->
```

## Unity changes

- [ ] No files under `unity/` touched
- [ ] Touches `unity/` — **requires human review**; describe what to check in the headset:

## Results

<!-- If this PR is backed by a sweep: link the results/ directory, and confirm the SHA it ran
     against is on main or tagged (a squash-merged branch makes the manifest SHA
     unresolvable). If it changes an algorithm without a sweep, say why not yet. -->

## Metrics

- [ ] No new metric
- [ ] New metric — defined in `docs/metrics.md` in this PR

## Assumptions

<!-- Modeling assumptions introduced: noise model, motion model, coordinate convention,
     parameter defaults. These are what silently invalidate results later. -->
