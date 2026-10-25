# 22. The headset finds the robot itself: a discovery query over Tailscale and the LAN

## Status

Accepted (2026-10-25). The user asked for the headset to work as the laptop's `just robot-ip` does:
ask through Tailscale for the robot's LAN address, use the LAN when the two share a network, and
Tailscale when they do not. The headset does not run Tailscale yet (the user was not sure), so a LAN
broadcast covers the same network without it.

## Context

- **The robot is on DHCP and its address moves** (memory and `robot/README.md`: 10.188.57.244 on
  2026-10-20, 10.188.58.137 on 2026-10-22). A DHCP reservation is the real fix, and is the lab's to
  make.
- **The headset uses one fixed address.** `JetRoverArmConfig.RemoteHost` is parsed with
  `IPAddress.Parse` by `JetRoverOperatorBridge` (pose, joint commands, joint state) and by
  `CameraFeedBridge`. Its built-in default is the robot's Tailscale address, and `just
  push-galaxy-config` overwrites it with whatever LAN address `just robot-ip` found that day. When the
  robot's lease changes, the headset finds nothing until someone pushes the config again.
- **The laptop already copes** (`just _robot-host`): mDNS first, then `tailscale ping`, which reports
  a direct LAN path when there is one. Neither is available to a Unity app on Android without a
  plugin: Android filters multicast without a `WifiManager.MulticastLock`, and there is no Tailscale
  CLI.

## Decision

1. **The robot answers a discovery query** on UDP **6005** (`--discovery-port`, which `just
   deploy-robothost` passes). RobotHost replies to the query's sender with its hostname and its IPv4
   addresses, read from its interfaces at each query, so a new lease is reported at once. It leaves out
   loopback, link-local (169.254/16), Tailscale's 100.64/10, and Docker's bridges (`docker*`,
   `br-*`, `veth*`, 172.17/16). Answering moves nothing and changes no subscriber: the port is its
   own, not the joint port, where any sender takes the joint-state stream.
2. **The wire format** is `RobotDiscoveryCodec` (`core/Teleop.RobotArm/Wire/`), little-endian:
   - query, 8 bytes: magic `0x5244` ("RD"), version 1, type 1, a nonce (`u32`);
   - reply: magic, version, type 2, the query's nonce, an address count (`u8`, at most 8), 4 bytes per
     address, a name length (`u8`, at most 32) and the ASCII hostname.
   A reply whose nonce is not the query's is ignored.
3. **The headset resolves the address once, at start** (`RobotAddressResolver`, next to the codec:
   a pure state machine, tested headlessly; `Bridge/RobotAddressDiscovery.cs` does the I/O).
   Each query is repeated every 150 ms within its step:
   1. **Ask through Tailscale** (`RobotTailscaleHost`, default 100.112.90.72), for up to 1.5 s. If the
      robot answers, ask each LAN address it reported, directly, for up to 0.5 s. The first to answer
      is the robot on this network: **use the LAN**. If none answers, the two are on different
      networks: **use Tailscale**.
   2. **If Tailscale does not answer** (no Tailscale on the headset, or it is off), broadcast the query
      to 255.255.255.255 and ask the configured `RemoteHost`, for up to 0.6 s. Any answer comes from
      the robot's current LAN address: **use it**.
   3. **If nothing answers, use `RemoteHost`** as before, and log that discovery failed.
   The worst case, nothing answering, costs about 2.1 s once at start. Both Bridge components share
   one result for the life of the app (a static cache), so they always talk to the same address.
4. **`DiscoverRobot` (default true) turns it off**, leaving `RemoteHost` as the only address, exactly
   as before. `RemoteHost` remains what `just push-galaxy-config` writes, so the last known LAN
   address is still a fallback.
5. **`just robot-discover` runs the same resolver from the laptop** (`Teleop.Eval discover-robot`), so
   the robot's side can be checked without a headset.

## Consequences

- **The headset follows a DHCP change without a config push**, on the same network. Across networks it
  needs Tailscale installed and signed in on the headset; until then only the LAN steps can work.
- **An address change during a session is not followed.** Discovery runs once, at start. Restart the
  app.
- **A broadcast can be filtered.** Some Wi-Fi networks drop broadcasts between clients. The lab Wi-Fi
  passes mDNS, which suggests it passes broadcasts too; this is unverified until a headset tries it.
- **The Bridge change wants human review** (`unity/CLAUDE.md`): it opens a socket at start and delays
  the first connection by up to about 2 s. `just bridge-check` compiles it; only an IL2CPP build on the
  headset tests it.
- **RobotHost gains a socket**, so `just deploy-robothost` (which restarts it) is needed once, with
  someone at the robot.

## Alternatives considered

- **mDNS from the headset** (`jetrover-sinrg.local`). Android's resolver does not reliably resolve
  `.local` names for an app, and receiving multicast needs a Java `MulticastLock`. Asking our own
  responder by unicast and broadcast needs neither.
- **Answering on the joint port (6002).** RobotHost streams joint state to whoever last sent on that
  port, so a discovery query from a second device would take the stream. A port of its own cannot.
- **A DHCP reservation.** The real fix, but not ours to make; discovery also covers a robot moved to
  another network.
