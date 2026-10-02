# 15. The operator finds the robot by name over mDNS, at connect time and again whenever it goes quiet

## Status

Proposed.

## Context

Every operator host reaches the robot through a literal IPv4 address:

- **Unity:** `JetRoverOperatorBridge` reads `RemoteHost` from the `jetrover_connection` config and
  calls `IPAddress.Parse` on it once. `CameraFeedBridge` reads the same field. The committed
  default is the robot's Tailscale address, `100.112.90.72`. A per-machine override in
  `Application.persistentDataPath` replaces the whole config, not one field.
- **Command line:** the `just` recipes take `JETROVER_HOST` and pass it to `Teleop.Eval` and
  `scp`/`ssh`.

That address has moved under us twice, and each time the symptom was "Unity won't connect" with
nothing saying why:

- **2026-10-01:** the PC dropped from SINRG WIFI onto eduroam, so the LAN address was unreachable
  and Tailscale fell back to a relay ("iad").
- **2026-10-02:** the hand-set static `10.188.57.2` (`just robot-static-ip`) stopped working about
  a day after it was set. The gateway, internet and Tailscale were all unreachable while
  NetworkManager still said "connected". Switching back to DHCP fixed it at once, and DHCP handed
  back the *same* address. That points to the switches enforcing DHCP leases. The same day,
  Tailscale on the robot had logged itself out. With the LAN address dead and Tailscale offline,
  Unity's committed default could not have worked either.

The robot is on DHCP now, with no reservation. Its address can change at any lease renewal, and
nothing on the operator side would notice.

Facts measured on 2026-10-02 that shape the decision:

- **The campus DNS does not know the robot.** `nslookup nano` against `dns1.neu.edu` returns
  NXDOMAIN, and there is no reverse entry for `10.188.57.2`.
- **The robot already advertises itself over mDNS.** `ping nano.local` from the Windows PC resolves
  to `10.188.57.2`. The Jetson's mDNS responder needs nothing installed or configured, and SINRG
  passes link-local multicast between the PC's WiFi and the robot's wired port.
- **The robot answers a query straight back to the asking port.** A hand-built query from an
  ephemeral port to `224.0.0.251:5353` came back as a unicast reply to that port, from
  `10.188.57.2:5353`, in 7 ms. RFC 6762 §6.7 calls this a "legacy unicast" query. The reply had ID
  echoed, flags `0x8400`, one A record, TTL 10 s, value `10.188.57.2`. Its 44 bytes are the test
  fixture below.
- **Which interface sends the query matters.** The same query from a socket bound to `0.0.0.0` got
  no reply: Windows routed the multicast out of a virtual adapter (WSL's `vEthernet` or Tailscale).
  Bound to the WiFi address, it worked.
- **`.local` resolution through the OS is uneven.** Windows' resolver handles it, so
  `Dns.GetHostAddresses("nano.local")` would work in the Editor. Android's stub resolver does not
  reliably, and the Quest is Android. Receiving *multicast* on Android also needs a
  `WifiManager.MulticastLock`, which a unicast reply does not.

## Decision

### 1. The robot is configured by name. The literal address becomes the fallback.

`JetRoverArmConfig` gains `RemoteHostName`, default `"nano.local"`. `RemoteHost` stays, keeps its
meaning as a literal IPv4, and becomes the fallback used when the name does not resolve. Its
committed default stays the Tailscale address, which works from off the LAN.

An empty `RemoteHostName` turns discovery off and restores today's behaviour exactly. That is the
escape hatch for a network that blocks multicast.

### 2. Resolution is a single mDNS legacy-unicast query that the project builds itself, not the OS resolver

The host sends one standard DNS query for `<name>` (type A, class IN, non-zero ID) to
`224.0.0.251:5353` from an ephemeral port, then waits for the unicast reply. It accepts the first
response that echoes the query's ID, answers the question, and carries at least one A record for
that name.

Using our own query rather than `Dns.GetHostAddresses` gives:

- **One code path for the Editor, the Quest and the command line.** It does not depend on how a
  given OS feels about `.local`.
- **No `MulticastLock`.** The reply is unicast to our port, so nothing on Android has to be asked
  to stop filtering multicast.
- **A diagnosable failure.** "No reply in 1 s on any interface" is a specific, loggable fact. "The
  resolver threw" is not.

There are no retransmission backoffs and no caching beyond what §4 needs. This is a one-question
client, not an mDNS implementation.

### 3. The query is sent once per IPv4 interface address

The resolver binds one socket to each up, non-loopback IPv4 address the host has, sends the query
from each, and takes the first valid reply. This is what made the measured query work on a Windows
PC with Tailscale and WSL adapters.

Where interface enumeration is unavailable or throws (a known weak spot of
`NetworkInterface.GetAllNetworkInterfaces` under IL2CPP on Android), it falls back to a single
socket bound to `0.0.0.0`. On the Quest that is the WiFi interface, because there is no other.

### 4. Resolve at connect, then again whenever the robot goes quiet; switch the address in place

- **At start,** the transports are built immediately against the fallback `RemoteHost`. Resolution
  runs on a background thread, so the first frame never waits on the network. When it answers, the
  transports switch to the resolved address.
- **Afterwards,** whenever `JetRoverOperatorBridge.Status` is `Stale`, or still `NeverConnected`,
  the name is resolved again, at most once every 5 s. If the answer differs from the current
  address, the cartesian, joint and camera transports switch to it.
- **Switching is in place.** The `UdpTransport`s gain a settable remote endpoint instead of being
  rebuilt. `ClockSync`, the predictor history and the recording carry on: the robot's clock did not
  change, only its address. This mirrors why `SwappableTransport` exists (Unity `CLAUDE.md`).
- **Every switch is logged** with its tick and both addresses, e.g.
  `[net] nano.local -> 10.188.57.9 (was 10.188.57.2)`. The HUD shows which address and which path
  is in use (`LAN via nano.local`, or `fallback 100.112.90.72`).

The robot needs no change. `Teleop.RobotHost` replies to the source of the latest command (#57),
and `Teleop.CameraHost` streams to the source of the latest keepalive, so both follow the operator
wherever the operator's packets come from.

### 5. The pure half lives in Core: `core/Teleop.Core/Discovery/`

`MdnsQueryCodec` encodes the query for a name and ID into a caller's buffer. It parses a response
into "an A record for that name: yes/no, which IPv4, which TTL", with no allocation, no I/O and no
clock. Its parser has to handle DNS name compression (the measured reply uses a `0xC00C` pointer),
so it is written to reject:

- pointer loops;
- labels running past the end of the buffer;
- counts that disagree with the bytes present;
- answers whose name is not the one asked about.

Malformed input is everyday input on a shared multicast group, not an exception. Unit tests use the
44-byte reply captured above as a fixture, plus hand-built malformed variants.

Sockets, interface enumeration, the background thread and the re-resolve policy belong to the hosts:
`Bridge/RobotAddressResolver.cs` in Unity, and later `Teleop.Eval`.

There is **no `Contracts/` interface and no registry entry**, for the same reason ADR 0014 §6 gives
for `Camera/`. There is one way to find the robot, not a family of competing implementations, and
nothing here is a research axis or appears in a benchmark. A new folder is still an architecture
change under root `CLAUDE.md`, which is why this ADR exists.

### 6. The command line follows later, as its own step

`Teleop.Eval`'s hardware verbs and the `just` recipes would benefit from the same lookup, so that
`JETROVER_HOST` could default to `nano.local`. They are not part of the first implementation. Unity
is where the failure bit, and `ssh`/`scp` in the recipes already resolve `.local` through the OS on
Windows.

## Consequences

- **Unity reconnects by itself after a DHCP renumbering**, in about 5 s plus one round trip, with a
  log line saying what happened. Today it stays silently disconnected.
- **A run can move between paths mid-session** (LAN to Tailscale fallback, or the reverse), and the
  paths have different latency. The switch is logged and shown on the HUD, but it is not yet in the
  `.tlog` (open question 2). Until it is, a citable run must be checked for switches by hand.
- **mDNS is unauthenticated,** like the command and camera protocols themselves. A device on the
  same LAN could answer for `nano.local` and draw the operator's commands away from the robot. It
  cannot command the robot that way, but it could fake one. This is the same trust level the
  project already accepts on this network, and it is written down here so it is a known limit rather
  than a surprise.
- **The name has to be unique on the LAN.** If a second device claims `nano`, mDNS conflict
  resolution renames one of them (`nano-2.local`), and which one keeps `nano` depends on boot
  order. "nano" is a plausible default for other Jetsons in a robotics lab (open question 1).
- **A DHCP reservation is still worth having.** Discovery makes a changed address survivable. It
  does not make it stop changing, and the `just` recipes still take a literal until §6 is done.
  `robot-static-ip` should be retired whatever happens here: on SINRG it breaks the network a day
  after it is applied.

## Alternatives considered

- **Static IP on the robot.** Tried on 2026-10-01; broke on 2026-10-02 (Context).
- **DHCP reservation only.** Still recommended, but it needs IT, and it does not help the day the
  robot moves to another network or the PC joins a different one.
- **Tailscale only (MagicDNS name or the 100.x address).** It works from anywhere, but it logs out
  on key expiry (which happened), and it can take a relay path whose latency is not the LAN's. It
  stays as the fallback, not the primary path.
- **The OS resolver (`Dns.GetHostAddresses("nano.local")`).** Simplest in the Editor, but unreliable
  on the Quest (Context). It also hides which interface asked, and on this PC that is what decides
  whether an answer comes back at all.
- **A robot-side UDP broadcast responder.** It would need a new datagram type in `Teleop.RobotHost`
  and a protocol change, and it would rely on broadcast crossing WiFi to wired. Multicast already
  crosses, and the responder already exists.

## Open questions

1. **Rename the robot?** A unique hostname (e.g. `jetrover-sinrg`) would remove the "second
   `nano`" risk at the cost of one robot-side change, which needs the user's go-ahead (`robot/` is
   ask-first).
2. **Record path changes in the `.tlog`?** A new tag carrying the tick, the old and new address and
   the source (`mdns` or `fallback`) would make mid-session path switches visible to `analysis/`.
   Readers skip unknown tags (`Recording/CLAUDE.md`), so this is forward-compatible. It is
   left for the implementation PR to decide with the recording owner.

## Verification

- **Core:** codec unit tests (the captured fixture, compression, every malformed shape in §5),
  plus `just core-check`.
- **Bridge:** `just bridge-check` compiles the resolver and the endpoint change against Core.
- **Editor, by hand:**
  - set the override's `RemoteHost` to a wrong address and confirm Unity still connects through
    `nano.local`;
  - with `RemoteHostName` empty, confirm today's behaviour;
  - unplug and replug the robot and confirm the `[net]` line and the reconnect.
- **Quest:** confirm the lookup gets an answer on the headset (the log line), since that is where
  interface enumeration and the absence of a `MulticastLock` actually get tested.
