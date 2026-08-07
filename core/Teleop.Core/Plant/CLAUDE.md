# Plant

The Core-side `IRobotPlant` — the robot the pipeline commands, headless and deterministic.

## Implemented

| Name | File | Idea |
|---|---|---|
| — | `RigidBodyPlant.cs` | kinematic dead-reckoner: setpoint snap on `Command`, coast on last commanded velocity between commands, semi-implicit Euler in `Step` |

`Registry/Registries.cs` exists now (see `Registry/CLAUDE.md`), but has no `Plants` table:
plants aren't selected by name the way predictors/reconcilers are -- the host always knows which
plant it wants (Core's for a sweep, `unity/`'s `Bridge/` for Unity physics, `Teleop.RobotHost/`
for real hardware) and constructs it directly. A registry table would still be one line away the
day a sweep actually needs to name a plant, but nothing today does.

**This folder exists late on purpose, and its absence was an accident.** Each of the five
research axes (`Prediction/`, `Reconciliation/`, `Buffering/`, `Transport/`, `Autonomy/`) got a
folder from the start, because each is a family of competing implementations. `IRobotPlant` has
been in `Contracts/` since early on with nowhere in Core to implement it — the two implementations
anyone had in mind, Unity physics and real hardware. Unity physics lives in `unity/`'s `Bridge/`;
real hardware lives in `core/Teleop.RobotHost/` (docs/adr/0007-jetrover-plant-and-robot-host.md)
-- **not** `Bridge/`, since routing a real robot's commands through Unity would carry them outside
Core's own `ITransport`/`ClockSync` instrumentation, silently opting the real robot out of every
latency figure this project exists to produce. But "the plant sweeps actually run against" is
Core's, by the same rule that puts `ManualClock` in `Time/`: an algorithm that cannot be evaluated
headlessly does not count, and that requires a plant that does not need a headset.

## Kinematic, not physics

`RigidBodyPlant` is a dead-reckoner, not a simulation. No mass, no forces, no inertia, no
contact, no joint limits, no actuator dynamics. `Command` snaps the pose one-to-one onto the
commanded setpoint; `Step` integrates position and orientation forward on the last commanded
velocity.

That is a deliberate choice, not a simplification to be fixed later. Phase 4 (docs/setup.md) is
the explicit **zero-mitigation baseline**, and any damped or spring/PD tracking scheme is itself
a smoothing behaviour: it would absorb part of every correction the reconciler is being measured
on, so the baseline would already be mitigating and every later comparison against it would be
scored short. If a plant with real dynamics is wanted, it belongs beside this one as a second
implementation with its own registry key — not as an "improvement" to this one.

## Requirements

1. **Gap policy documented, because it is what packet loss looks like to the operator.**
   `RigidBodyPlant` coasts indefinitely on the last commanded velocity: no timeout, no ramp to a
   stop. That is the policy that exercises `CommandFrame`'s velocity fields for their stated
   purpose — intent is what survives a lost packet — and it is the right choice *for this
   plant specifically*, because nothing here has mass or motors. It is not house style to be
   copied by every plant: a real-hardware plant should pick whichever of hold/coast/ramp-to-stop
   is actually safe for that hardware and document why, the same way
   docs/adr/0007-jetrover-plant-and-robot-host.md does for `JetRoverPlant` choosing hold instead.
2. **Stale and duplicate commands rejected whole**, by `CommandFrame.CaptureTicks`, never
   partially applied. Transport reorders and duplicates; a plant that jerks backwards to an old
   setpoint produces prediction-error plots that look like a predictor bug.
3. **Two tick domains, never compared.** `CommandFrame.CaptureTicks` is the operator's timebase;
   `Step`'s `nowTicks` is the host's. They are both `long` and they are not interchangeable.
   Relating them is `Time/ClockSync.cs`'s job.
4. **`Command` never advances simulation time.** Only `Step` moves `State.CaptureTicks`.
5. Allocation-free `Command` and `Step`, and a `Reset()` that restores as-constructed state —
   *including* the staleness baseline, so a reused instance accepts the next trial's first
   command whatever its stamp.
