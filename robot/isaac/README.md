# robot/isaac: the FR3 Duo in Isaac Sim

The Isaac bridge (docs/adr/0031) is the FR3 Duo with physics. It runs in Isaac Sim on the GPU machine,
behind a RobotHost there. The headset and the Eval verbs drive it exactly as they drive the simulated
host or, later, the real robot (docs/adr/0030). Its sessions say `"source": "isaac"`, and they are
never the robot's.

```
headset ──UDP joint port──> RobotHost (--backend arm-server) ──Unix datagram socket──> bridge ──> Isaac Sim (PhysX 1 kHz)
```

## Running it

The GPU machine needs our own Isaac Sim, installed once: `just isaac-setup`, then `just isaac-check`.
Push the commit you want to run, then:

```bash
just isaac-host                       # Isaac and RobotHost on the GPU machine; Enter stops both
just isaac-stop                       # stop them from any terminal (RobotHost closes its recording first)
just fr3-teleop-probe 0.08 3 franka-replay false <GPU machine>   # drive it headless from here
just push-galaxy-fr3-config <GPU machine>                        # point the headset at it
```

`isaac-host` refuses while a training run of ours, or another Isaac host of ours, is going. Other
people's GPU jobs no longer stop it (2026-11-02): it says how much of the GPU they hold, since sharing it
can make physics steps late. The bridge's log is `~/isaacsim/bridge.log` on that machine.

## The render check

`just isaac-render-check` (`render_check.py`, docs/adr/0032 phase P0) measures what the cameras' design
rests on, and prints one `RESULT` line:
1. **The bridge alone** for 20 s.
2. **A separate renderer with the bridge beside it,** the two pinned to separate cores. The renderer is a
   visual-only copy of the robot, posed from joint angles about its URDF axes, with the table, the
   backdrop surfaces and the head and third-person cameras.
3. **The head camera rendered inside the bridge's physics loop.**

It exits non-zero if any number is missing, or if the separate renderer misses a criterion:
- the bridge's late steps stay at most 2× its rate alone, or 1/s;
- at least 29.5 fps;
- a render lag that is stable in at least 99.5% of frames;
- depth p99 under 3 mm;
- projection p99 under 3 px.

The bridge runs from the pushed commit. Sample frames stay on that machine, in
`~/isaacsim/render-check/latest`. ADR 0032 holds the 2026-11-02 numbers.

## What is pinned, and how

| Copy | Of | Pinned by |
|---|---|---|
| `teleop_isaac/wire.py` | RobotHost's `ArmServer/ArmServerWire.cs` | `golden/arm-server-wire-v1.json`, written by the C# tests (`just gen-arm-server-golden`) |
| `teleop_isaac/shaper.py` | Core's `JointTrajectoryShaper` (via the trainer's copy, as text) | `golden/joint-shaper-v1.json`, the trainer's golden; an Eval test holds the two equal (`just gen-trainer-golden` refreshes both) |

`just isaac-bridge-test` runs those checks. They are plain Python and need no Isaac. Change a copy
and its original together, or the tests fail.

## What stands in for the robot

These are all bridge arguments, and the Welcome's description records them in each session's log:
- **Position drives** of 16000 Nm/rad and 250 Nm·s/rad on the arm joints (about 20 Hz tracking),
  and 2000 N/m and 100 N·s/m on the fingers. They play Franka's internal controller.
- **No gravity on the arms' own links.** The real FR3 compensates its own weight. Objects will feel
  gravity (phase 2).
- **PhysX at 1 kHz on the CPU,** with 8 solver iterations and no results written back to USD. That
  setting is what makes 1 kHz affordable (docs/adr/0031 amendment).
