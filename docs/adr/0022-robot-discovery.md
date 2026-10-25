# 22. The headset finds the robot itself: Tailscale first, then mDNS

## Status

Accepted (2026-10-25). **Supersedes ADR 0015's Unity plan** (§1–§5: mDNS first, the Tailscale
address as the fallback), which was accepted on 2026-10-02 and never built in Unity. It keeps 0015's
mDNS query as its LAN step, and 0015's recipe-side lookup (`just robot-ip`) is unchanged.

The user's choices:
- the order the laptop's lookup already follows in spirit: ask through Tailscale for the robot's LAN
  address, use the LAN when the two share a network, and Tailscale when they do not;
- when Tailscale is not available on the headset (it is not installed there yet), ADR 0015's mDNS
  query, which was measured to work on this LAN, rather than a LAN broadcast, which was not.

## Context

- **The robot is on DHCP and its address moves**: 10.188.57.244 on 2026-10-20, 10.188.58.137 on
  2026-10-22, 10.188.62.69 on 2026-10-25. A DHCP reservation is the real fix, and is the lab's to make.
- **The headset used one fixed address.** `JetRoverArmConfig.RemoteHost` was parsed with
  `IPAddress.Parse` by `JetRoverOperatorBridge` (pose, joint commands, joint state) and by
  `CameraFeedBridge`. `just push-galaxy-config` writes whatever LAN address `just robot-ip` found that
  day. When the lease changes, the headset finds nothing until someone pushes the config again.
- **ADR 0015's facts still hold**: the robot's own mDNS responder answers a legacy-unicast query by
  unicast to the asking port, so Android needs no `MulticastLock`; the query must be sent from each
  interface, because from 0.0.0.0 Windows routes it out of a virtual adapter.
- **Measured on 2026-10-25** from the PC with `just robot-discover`, against the robot at 10.188.62.69:
  - through Tailscale: the robot answered and reported 10.188.62.69, which answered directly. LAN,
    in 0.08 s;
  - without Tailscale, by mDNS from each interface: 10.188.62.69, in 0.02 s;
  - a broadcast (this ADR's first draft, the same day): no answer in 0.6 s. Whether Windows' adapter
    routing or the network dropped it was not settled; it was replaced by mDNS.

## Decision

1. **The robot answers a discovery query** on UDP **6005** (`--discovery-port`, which `just
   deploy-robothost` passes). RobotHost replies to the query's sender with its hostname and its IPv4
   addresses, read from its interfaces at each query, so a new lease is reported at once. It leaves out
   loopback, link-local (169.254/16), Tailscale's 100.64/10, and Docker's bridges (`docker*`,
   `br-*`, `veth*`, 172.17/16). Answering moves nothing and changes no subscriber: the port is its
   own, not the joint port, where any sender takes the joint-state stream.
2. **The wire formats** are in `core/Teleop.RobotArm/Wire/`, next to the joint codecs (not in a
   `Core/Discovery/` folder as 0015 planned: they are wire formats between the robot and its operators,
   which is what `Teleop.RobotArm/Wire` holds):
   - `RobotDiscoveryCodec`, little-endian. Query, 8 bytes: magic `0x5244` ("RD"), version 1, type 1,
     a nonce (`u32`). Reply: magic, version, type 2, the query's nonce, an address count (`u8`, at most
     8), 4 bytes per address, a name length (`u8`, at most 32) and the ASCII hostname. A reply whose
     nonce is not the query's is ignored;
   - `MdnsQueryCodec`, ADR 0015 §2's one-question client: an A query to 224.0.0.251:5353 from an
     ephemeral port, and a parser that rejects pointer loops, truncation, counts that disagree with the
     bytes, and answers for another name.
3. **The headset resolves the address once, at start.** `RobotAddressResolver`, next to the codecs, is
   a pure state machine, tested headlessly; `Bridge/RobotAddressDiscovery.cs` owns the sockets. Each
   query is repeated every 150 ms within its step:
   1. **Ask through Tailscale** (`RobotTailscaleHost`, default 100.112.90.72), for up to 1.5 s. If the
      robot answers, ask each LAN address it reported, directly, for up to 0.5 s. The first to answer
      is the robot on this network: **use the LAN**. If none answers, the two are on different
      networks: **use Tailscale**.
   2. **If Tailscale does not answer** (no Tailscale on the headset, or it is off), ask for
      `RemoteHostName` (default `jetrover-sinrg.local`) over mDNS, from a socket bound to each IPv4
      interface (0015 §3; one socket on 0.0.0.0 where interfaces cannot be listed), and ask the
      configured `RemoteHost` with a discovery query, for up to 1.0 s. An mDNS A record that another
      device could reach (not loopback, link-local, Docker or Tailscale), or the configured address
      answering, **is the LAN address**.
   3. **If nothing answers, use `RemoteHost`** as before, and log that discovery failed.
   Discovery queries go from a socket on 0.0.0.0, so the one through Tailscale takes Tailscale's route.
   The worst case, nothing answering, costs about 2.5 s once at start. Both Bridge components share one
   result for the life of the app (a static cache), so they always talk to the same address.
4. **`DiscoverRobot` (default true) turns it off**, leaving `RemoteHost` as the only address, exactly
   as before. An empty `RemoteHostName` skips mDNS. `RemoteHost` remains what `just
   push-galaxy-config` writes, so the last known LAN address is still a fallback. A headset override
   written before these fields existed gets their defaults.
5. **`just robot-discover` runs the same resolver from the PC** (`Teleop.Eval discover-robot`, with the
   same socket handling), so both the robot's side and the network can be checked without a headset.
   `just robot-discover none` skips Tailscale, as a headset without it does.

## Consequences

- **The headset follows a DHCP change without a config push**, on the same network, with or without
  Tailscale. Across networks it needs Tailscale installed and signed in on the headset.
- **An address change during a session is not followed.** Discovery runs once, at start; restart the
  app. ADR 0015 §4's re-resolve when the robot goes quiet, with the transports switched in place, is
  not built. It is the natural next step if mid-session changes turn out to happen.
- **The order is Tailscale first, so a headset with Tailscale pays a round trip through it** before
  probing the LAN: 0.08 s measured from the PC on a direct Tailscale path. Over a relayed path it is
  longer, at most 1.5 s.
- **mDNS is unauthenticated**, as 0015 says: a device on the LAN could answer for the name. The same
  holds for the discovery reply. Neither can command the robot; either could fake one. This is the trust
  level the project already accepts on this network.
- **The Bridge change wants human review** (`unity/CLAUDE.md`): it opens sockets at start and delays
  the first connection by up to about 2.5 s. `just bridge-check` compiles it; only an IL2CPP build on
  the headset tests it, including whether Android lists its interfaces (if not, mDNS goes out on the
  default one, which on a headset is the Wi-Fi).
- **RobotHost gains a socket**, so `just deploy-robothost` (which restarts it) is needed once, with
  someone at the robot.

## Alternatives considered

- **ADR 0015 as it stood: mDNS first, Tailscale fallback.** It needs no robot change and works on the
  LAN, but it cannot tell "on another network" from "robot off", and its fallback is a fixed Tailscale
  address rather than the robot telling us where it is. The user asked for Tailscale first.
- **A LAN broadcast to the robot's responder** (this ADR's first draft). It got no answer from the PC
  on 2026-10-25, while mDNS answered in 0.02 s. 0015 had rejected it for the same worry.
- **The OS resolver for `jetrover-sinrg.local`.** Unreliable on Android (0015).
- **Answering on the joint port (6002).** RobotHost streams joint state to whoever last sent on that
  port, so a query from a second device would take the stream.
- **A DHCP reservation.** The real fix, but not ours to make; discovery also covers a robot moved to
  another network.
