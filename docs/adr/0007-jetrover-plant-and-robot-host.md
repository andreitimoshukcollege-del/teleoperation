# 7. JetRoverPlant and a new Teleop.RobotHost process

## Status

Accepted. Phase 0 (feasibility) and the ROS-side groundwork are done; `Teleop.RobotHost` and
`JetRoverPlant` themselves are not yet built (tracked separately).

## Context

The project has a real Hiwonder JetRover (Jetson Nano, ROS 2 Foxy, reachable over Tailscale) and
`IRobotPlant` has been documented since early on as covering "a Core rigid-body approximation...
Unity physics... or real hardware," but no real-hardware implementation exists. Two materially
different architectures were considered for where that implementation, and the transport carrying
commands to it, should live:

**Unity as the ROS 2 client.** Add Unity's `ROS-TCP-Connector` package, implement
`UnityRobotPlant : IRobotPlant` in `Bridge/` (as `Plant/CLAUDE.md` previously assumed real-hardware
plants would), and have it publish/subscribe to the JetRover's ROS topics directly over
ROS-TCP-Connector's own TCP socket. Keeps everything in one Unity process, exactly like today's
Phase-4 loopback baseline.

**A new, non-Unity `.NET` host is the actual `RobotEndpoint`-side of a real `ITransport`.** The
robot side runs a separate process (not Unity) implementing `IRobotPlant` and using a real
`ITransport` over UDP, wired through Core's existing, unmodified `RobotEndpoint`.

The first option was rejected. `Pipeline/RobotEndpoint.cs` and `OperatorEndpoint.cs` already
assume all latency accounting (`ClockSync`, `owd_uplink_ms`/`owd_downlink_ms`, `docs/metrics.md`)
happens by feeding real timestamps from a real (or emulated) `ITransport` into `ClockSync`'s
Cristian's-algorithm-style offset estimator — this machinery is already fully built and simply
unexercised by a second real clock domain so far. Routing the real Tailscale hop through
ROS-TCP-Connector's socket instead would carry `CommandFrame`/state data over a channel Core's own
instrumentation never touches, silently opting the real robot out of every latency figure this
project exists to produce. `TeleopOperatorBridge.cs`'s own comment already anticipated a real
transport showing up independent of anything robot-side ("a real `UdpTransport` would reintroduce
that [network-thread] row; this is a deliberate simplification, not an oversight") — a second,
independent signal that the intended shape was always "swap `LoopbackTransport` for a real one,"
not "let Unity talk ROS directly."

## Decision

### `Teleop.RobotHost` is a new sibling `.NET` project under `core/`, not a `Teleop.Eval` verb, not `Bridge/`

Referenced only `Teleop.Core` (the same one-hop dependency shape as `Teleop.Eval`). Not a
`Teleop.Eval` verb: Eval is a CLI that runs once and exits (root `CLAUDE.md` invariant 10's
exit-code contract); this host runs unattended, indefinitely, on a different machine, and
shouldn't inherit Eval's sweep/replay/YAML dependency graph. Not `Bridge/`: Unity is not in this
path at all under this architecture — `Plant/CLAUDE.md`'s previous claim that real-hardware
plants live in `Bridge/` is corrected by this ADR.

### `JetRoverPlant`'s gap policy is hold, not `RigidBodyPlant`'s "coast indefinitely"

`RigidBodyPlant`'s own doc is explicit that indefinite coast is the *zero-mitigation research
baseline*, not a safety-appropriate default. A real rover has mass and motors; on a
command-timeout, `JetRoverPlant` holds (the arm's bus servos already hold position with no
repeated command — zero relay traffic needed to do so). This is an intentional divergence from
the only existing plant's convention, not an oversight to be "fixed" back into consistency later.

### `IRobotPlant.Reset()` does not command hardware motion on `JetRoverPlant`

The interface's doc describes returning to "as-constructed state" for sweep reuse — free and
instantaneous for a kinematic plant, but neither free nor safe to interpret literally as "teleport
the physical arm back" on real hardware. `JetRoverPlant.Reset()` clears only its own bookkeeping
(setpoint, staleness baseline, gap timer) and must say so loudly in its own doc comment.

### The ROS 2 code lives in a new, separate repo, not this repo's `robot/` and not `industryxr-robot`

[`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros) is a
new repo containing `jetrover_arm_control`, ported and trimmed from
`SINRG-Lab/industryxr-robot`'s `sinrg_robot_sdk` (camera/perception code, `ros_tcp_endpoint`, and
other unrelated pieces left behind). This repo's `robot/` stays documentation-only — a pointer to
that repo plus the local relay protocol spec once it exists — keeping root `CLAUDE.md`'s literal
claim that `robot/` "does not interact with the above builds" true. `industryxr-robot`'s local
clone on the Jetson was removed after confirming nothing was uncommitted/unpushed to its GitHub
remote, which is untouched.

### Local channel between `Teleop.RobotHost` and the ROS relay node: a Unix domain socket, not UDP-loopback

Planned for the phase that adds `JetRoverPlant`/the relay node: a tiny fixed-size struct over a
Unix domain datagram socket, no sequence/staleness/coast logic at that hop (`JetRoverPlant`/
`RobotEndpoint` already own that). A UDS path is categorically unreachable from Tailscale/LAN,
which matters because this hop will drive real motors with zero authentication — a UDP-loopback
socket is one accidental non-loopback bind away from being reachable off-host.

## Consequences

- `Plant/CLAUDE.md` needs its "real hardware lives in `Bridge/`" line corrected.
- A fourth host location now exists in root `CLAUDE.md`'s one-law diagram
  (`Teleop.RobotHost`, alongside `Teleop.Eval` and `unity/TeleopVR`), each a leaf depending only on
  `Teleop.Core`.
- Mecanum base motion is out of scope until (if ever) a later ADR revisits it — no wheel odometry
  exists anywhere in the board SDK, so a base plant could only ever be dead-reckoned from
  commanded velocity, never sensed, and that limitation needs its own explicit decision rather
  than arriving as a side effect of this one.
