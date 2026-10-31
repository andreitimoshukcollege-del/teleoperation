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
just fr3-teleop-probe 0.08 3 franka-replay false <GPU machine>   # drive it headless from here
just push-galaxy-fr3-config <GPU machine>                        # point the headset at it
```

`isaac-host` refuses while the GPU is busy with someone else's job, or with a training run of ours.
The bridge's log is `~/isaacsim/bridge.log` on that machine.

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
