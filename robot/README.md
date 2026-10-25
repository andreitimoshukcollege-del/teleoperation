# robot/

Everything that runs on the JetRover's Jetson that is not .NET:

| Path | What |
|---|---|
| `ros/` | the ROS 2 (Foxy) colcon workspace: `jetrover_arm_control` (the Hiwonder board's serial protocol, the servo bus, the battery) and `teleop_relay` (the bridge to RobotHost) |
| `systemd/` | the Jetson's four services, and how to install them |

`ros/` came in from the separate
[`jetrover-teleop-ros`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros) repository
on 2026-10-15, with its history (docs/adr/0019). It was imported from the Jetson's own checkout,
because that was the copy that ran: the ROS side of ADR 0011 and later settings had never been
pushed. The old repository is left as it was. This directory is now the source of truth.

The workspace was ported and trimmed from `SINRG-Lab/industryxr-robot`'s `sinrg_robot_sdk`; that
repository's README says what was kept and what was left behind. Every repository involved is
public and MIT-licensed, including the Hiwonder board library inside `jetrover_arm_control`.

Nothing in `core/` or `unity/` builds or references `ros/`. They meet only at the relay's wire
format.

## Deploying

```bash
just deploy-robot                # everything: gates, this workspace, RobotHost, CameraHost, checks
just deploy-ros false            # this workspace only: copy to the Jetson and build there; nothing running changes
just deploy-ros                  # ...then switch the two ROS services to it and restart them
```

Each deploy builds into a new `~/teleop_ros_deploy_<time>/`, and `~/teleop_ros_current` points at
the active one. The services source `~/teleop_ros_current/install/setup.bash`. Rolling back is
repointing that link and restarting `jetrover-arm-control` and `jetrover-relay`.

**Restarting the servo driver interrupts the arm** (the shoulder has sagged during a restart), so
someone must be at the robot. Before the first `deploy-ros`, the Jetson ran the old checkout at
`~/projects/jetrover-teleop-ros`, which stays there untouched as a fallback.

Read-only checks:
- `just robot-status`: the services;
- `just ros-check`: is the board answering the driver;
- `just robot-feedback-ids`: which servos report position;
- `just robot-battery`: battery voltage;
- `just robot-board-commands`: what the board accepts.

## Recording sessions for the trainer (docs/adr/0021)

The learned joint predictor (`trainer/`) learns from what the robot itself recorded: every servo read
and write (the driver's `driver.jlog`) and every command, relay message and frame (RobotHost's
`robothost.jlog`), all on the Jetson's CLOCK_MONOTONIC. The format is
`trainer/contract/SESSION_FORMAT.md`. Recording moves nothing and restarts nothing: both processes
watch `~/recordings/.active` and write while it names a session directory.

Once, with someone at the robot (the ROS deploy restarts the servo driver):

```bash
just deploy-ros                   # the driver's session log
just deploy-robothost             # RobotHost with --record-control and --uplink-control
just jetson-clock-check           # both logs really are on one clock
```

Each session, with someone watching the arm as for any teleop:

```bash
just record-start teleop-2p5-none 2.5                  # label, slider (rad/s or varied); load=none
just record-start payload-100g 2.5 load=payload payload_g=100
just record-start hand-1 3.8 load=hand
just record-start live-150 2.5 network=150ms-20j-0.5loss   # the uplink impaired on the robot, for real
just record-start live-sched 2.5 "network=schedule:lan@0;150ms-20j-0.5loss@45;lan@90;50ms-5j@135"  # and changing
just record-mark "payload on"                          # a moment worth aligning on, on the Jetson's clock
just record-stop
just robot-recordings                                  # what is on the robot, and whether each log closed
```

`record-start`'s options are `key=value` after the label and the slider, in any order (just itself has
no keyword arguments; before 2026-10-22 these examples bound positionally and failed).

**The arm can drive itself** (`joint-drive`, docs/adr/0021's data-collection tool): scripted joint
moves sent the way the headset sends them, with no headset. `waypoints` is smooth, teleop-like motion
with pauses; `steps` moves one joint at a time by 0.15–0.8 rad and holds, which is what `armpred
sysid` needs to measure the servo's speed limit and lag. The limits are the IK's joint ranges (the
collision model, docs/adr/0020) less 15° at each end, and every pose is checked against the collision
model before anything moves. That model knows the robot's own body and the floor, nothing else, so
clear the space around the arm. It starts by moving slowly from wherever the arm is to the model's
home pose, and refuses if that path is not clear (`just move-arm 0.15 0 0.08` first). **Press Enter to
stop it**: the arm holds where it is. Ctrl+C does not reach the Windows `dotnet` under WSL. It also
stops by itself on a stall (a joint over 0.25 rad from its command and not moving for 1 s), and when
the joint-state stream has been missing or below 30 Hz for 3 s of any 10 s: the robot is gone, or the
headset app (or `joint-probe`) is on the joint port too. Close the headset app first. A shorter gap,
such as a Wi-Fi stall, only pauses it: the arm holds, and the script resumes from the same pose.

```bash
just joint-drive-preview steps 6           # sends nothing: the envelope and what it would do
just record-drive steps-6 steps 6 seconds=300                 # one recorded session (motion=joint-drive:steps)
just record-drive wp-payload waypoints 2.5 load=payload payload_g=100
just pilot                                 # robot/pilot/pilot-v1.txt: the whole pilot hour, session by session
just pilot from=drive-wp-5                 # resume there
```

`just pilot` reads a plan (`robot/pilot/pilot-v1.txt`, one session per line), says before each
session what a person has to do (headset on, a slider value, a payload, close the app), waits for
Enter, and records it: `record-start`/`record-stop` around a timed teleop session, or `record-drive`.
After each it reads the battery from that session's log and stops below 11.0 V. Nothing reports servo
temperature, so feel the servos between sessions. At the end it runs `just fetch-recordings`.
**Without the laptop** (someone still watches the arm the whole time): `just deploy-joint-drive` puts
`joint-drive` on the Jetson, and `just pilot-on-robot` starts `robot/pilot/onboard.py` there, detached,
which records every `drive:` session of the plan back to back, talking to RobotHost over loopback (those
sessions have no Wi-Fi in them, and their notes say so). It rests the servos between sessions
(`rest_s=60`), stops below 11.0 V, and starts no further session after one that ended early. Stop it
with `just pilot-stop` from any machine (joint-drive holds the arm at once), or with the power switch
(the arm goes limp). Watch it with `just pilot-status`; collect with `just fetch-recordings`. The
teleop sessions still need the laptop: `just pilot only=teleop`. `onboard.py` writes session.json and
closes sessions itself, as `record-start`/`record-stop` do, and must be kept the same as them.

```bash
just deploy-joint-drive          # once, and after joint-drive changes; moves nothing
just pilot only=teleop           # the headset sessions, from the laptop
just pilot-on-robot              # the drive sessions, on the robot; the laptop can close
just pilot-status                # from any machine with the repo
just pilot-stop
```

`record-drive` copies `joint-drive`'s `operator.jlog` (every command it sent, on the laptop's clock)
into the session before `record-stop` checksums it, and closes the session even when the drive stops
early.

Then, back at the repo: `just fetch-recordings` (copies the new sessions into the gitignored
`recordings/jetrover/` and checks them), and `just pack-dataset recordings/jetrover/<id> ...` for the
one file the GPU machine needs (`dist/jetrover-data-<UTC>.tar`). `just recording-check` fails a
session with dropped records, a missing trailer, logs that do not share a clock, or a read gap over
500 ms (`allow_gaps=true` keeps the gaps, annotated). Archive the sessions to a GitHub Release; they
are not committed.

- **`network=<profile>`** makes the arm really receive late, jittery, lossy commands (Core's
  `EmulatedTransport` on RobotHost's receive side; frames to the headset are untouched).
  `just uplink-impair none` turns it off; `record-stop` does so too.
- **`network=schedule:<profile>@<s>;...`** (`just uplink-schedule`) changes the uplink mid-session,
  switched by RobotHost on time and logged as `UP` at each switch (docs/adr/0021 decision 13). These
  are the sessions where a network forecast could matter: the arm executes commands that arrive later
  than the headset assumed. At a scheduled switch, what the previous profile held still arrives; a
  hand switch (`uplink-impair`) drops it.
- **Never run `just joint-probe` during a session.** Anything sending on UDP 6002 takes the
  joint-state stream, and the check fails the session for its second subscriber.
- **Hand resistance stays light:** a stall of at most 2 s, contact for at most 20% of a session. Stop
  at a servo above 60 °C or a pack below 11.0 V. If the board goes silent, run `just arm-logs` and
  `just robot-board-diagnose` before the power switch.

## How it connects to the rest of the repo

`IRobotPlant` (`core/Teleop.Core/Contracts/IRobotPlant.cs`) is Core's contract for "the robot
being commanded." A real hardware implementation, `GenericArmPlant` (originally `JetRoverPlant`,
generalized to a configurable `RobotArmProfile` by docs/adr/0011-generic-robot-arm-profiles.md —
the "Status" log below still says `JetRoverPlant` where that was the correct name at the time),
lives in `core/Teleop.RobotHost/`, **not** in `unity/`'s `Bridge/` and **not** in this directory.
See `docs/adr/0007-jetrover-plant-and-robot-host.md` for the original architecture and why, and
`docs/adr/0011` for the later generalization.

```
Teleop.RobotHost (on the Jetson)                    robot/ros (on the Jetson)
  UdpTransport : ITransport   <-- UDP from the operator (LAN, or Tailscale)
  RobotEndpoint (Core, unmodified), joint-command and joint-state ports
  GenericArmPlant : IRobotPlant
      |
      | Unix domain socket, local only (Relay/RelayProtocol.cs here, and
      | ros/src/teleop_relay/teleop_relay/relay_protocol.py -- must match exactly; change both in one PR)
      v
  teleop_relay (ROS 2 node) ──────────────────────────────────────>  jetrover_arm_control
                                                                      (per-motor targets and feedback:
                                                                       base, shoulder, elbow, wrist,
                                                                       wrist roll, gripper)
```

`teleop_relay` is a thin node with no staleness, sequencing or gap-policy logic of its own. It
translates the local wire protocol above into calls against `jetrover_arm_control`'s topics.

## Status

- **Phase 1 (arm base servo only) confirmed working fully end-to-end on the physical robot**: a
  real `CommandFrame`, sent over genuine UDP through Tailscale from a dev-machine test harness,
  decoded by `Teleop.RobotHost`'s `UdpTransport`, applied by Core's unmodified `RobotEndpoint` to
  a real `JetRoverPlant`, forwarded over a Unix domain socket to `teleop_relay`, published to
  `jetrover_arm_control`'s `/arm/servo/base` topic, and physically moving the real servo --
  visually confirmed. Feedback round-trips correctly back through the same chain into the
  `RobotStateFrame` reply.
- **Phase 2 (real inverse kinematics, all four position-affecting joints, gripper) also confirmed
  working on the physical robot.** `JetRoverPlant.Command` now runs
  `Kinematics/FourDofArmKinematics.cs` against `CommandFrame.Pose` instead of Phase 1's stand-in,
  and denormalizes `CommandFrame.Gripper`. A commanded Cartesian target moved the arm to
  approximately the right position -- X/Y matched the intended target closely; Z was
  off by the amount explained below (not a sign or math error -- confirmed by hand-deriving the
  IK for the test target and cross-checking against a passing round-trip test suite).
- **Real, hardware-level finding: this JetRover's middle-arm servo (ID 3) never responds to any
  read command, full stop -- not specific to position.** Originally found (and under-described)
  as "never responds to position-read requests"; re-investigated on 2026-08-13 after the
  operator, feeling real resistance/torque at the joint, correctly pushed back on that servo being
  described as "confirmed physically dead" (a mischaracterization that had crept into
  `unity/.../JetRoverArmRig.cs`'s doc comment, since corrected -- it is not in this file, which
  was always accurate). A standalone script calling the board SDK directly (ROS stopped, exclusive
  serial access, `jetrover-teleop-ros`'s own pattern for isolating hardware-layer findings) probed
  servo ID 3 with `read_id`/`read_position`/`read_temp`/`read_vin`/`read_offset`, 5 attempts each:
  **zero responses across all 25 attempts, every command type**, while the same probe repeated
  against servo IDs 1 (base), 2 (lower), and 4 (upper) -- all on the identical shared bus --
  succeeded on 24/25 or better, every command type, for every one of those three. ID 4 (upper) is
  physically further down the daisy chain than ID 3's own connector, so its clean reads prove the
  bus wire itself is electrically intact end-to-end through and past the middle servo's
  connector -- ruling out a loose/broken connector or wiring fault as the cause. Writes to ID 3
  still work fine (it visibly moves, and the operator can feel it holding torque under a write).
  Taken alone, a servo that receives and acts on every write but never drives a response onto an
  otherwise-healthy bus for *any* read command looked like an isolated fault in that one unit's
  own transmit/response path (e.g. a failed bus-driver output stage), with the rest of the
  mechanism assumed fine.

  **That narrower theory is contradicted by a second finding, the same day.** With the robot
  fully powered off, the operator found the middle servo has significantly more resistance to
  manual rotation than the other (also powered-off) servos, and confirmed the resistance is
  **uniform through the servo's full rotation range**, not localized to one spot. Uniform
  drag rather than a localized catch/grind rules out a stripped gear tooth or debris in the mesh
  (which would only show up at the rotation angle where the damaged tooth engages) and instead
  matches, specifically, a **shorted motor winding**: the classic and specific behavior of a
  shorted small DC motor is cogging/braking via induced current at every shaft angle, not a
  positional catch. This is not something a communications-only fault would produce, and it is
  not confirmed by opening the case, but the behavioral signature is now quite specific. The
  honest current state is real internal motor damage, not an isolated response-driver fault with
  an otherwise-healthy mechanism. **Do not describe this servo as "dead"** (it demonstrably still
  moves and holds torque under command) **and do not assume the mechanism is otherwise fine** (it
  is not). A shorted winding is not something to keep driving: repeatedly energizing a shorted
  winding draws excess current through the drive circuit and risks heat damage to the servo's own
  driver stage or, over enough repetitions, the board's own motor-driver channel for that servo.

  **Resolved 2026-08-13: the servo was physically replaced, and the replacement is confirmed
  fully working.** The same protocol-level probe (`read_id`/`read_position`/`read_temp`/
  `read_vin`/`read_offset`, 5 attempts each) now succeeds 25/25 for the middle joint, matching
  base/lower/upper. One real complication during replacement, worth recording: the new unit
  shipped with its bus ID still at its factory default (1), colliding with the existing base
  servo (also ID 1) once installed -- symptomatic as intermittent, seemingly-random read failures
  on *both* base and the new middle servo, which briefly looked like the replacement unit was
  itself defective (or the wiring still bad) until traced to the ID collision. Fixed via a
  three-step temporary-ID swap, since the physical bus topology (single chain, board connector
  inaccessible without full teardown) makes it impossible to isolate the new servo alone on the
  bus directly: with only the base servo connected (the one accessible isolation point), its ID
  was moved 1 -> 9 (a scratch ID); with everything reconnected, ID 1 now uniquely belonged to the
  new servo, which was moved 1 -> 3 (its correct address); base was then moved back 9 -> 1. Each
  step was verified by reading back the new ID before proceeding, and by confirming the old ID no
  longer responded. General lesson for any future bus-servo replacement on this hardware: check
  (or reset) a replacement unit's ID *before* installing it, since a default-ID collision with an
  existing servo is easy to mistake for a hardware fault in the new part.
  This is not a software bug to fix -- `JetRoverPlant.State` already degrades honestly for exactly
  this case (`IsFullySensed` reports false; the affected joint's contribution falls back to this
  plant's last-commanded target rather than a stale sensed value or a fixed default). It is the
  reason Phase 2's real-hardware Z result didn't exactly match the commanded target, and it also
  means this joint's belief can never be corrected by real feedback (see
  `SeedBeliefsFromSensedIfNeeded` in `JetRoverPlant.cs`) -- a real, still-open contributor to
  reports of the arm "moving but not quite correctly" at this joint, distinct from the
  rate-dependent command loss below.
- Three real, pre-existing bugs in the ported `jetrover_arm_control` SDK were found and fixed
  along the way (all in `jetrover-teleop-ros`, not here): an uncaught `queue.Empty` on a
  board-read timeout that crashed the whole ROS node; a single-slot response queue with no
  request/response correlation that silently dropped fresh servo-position responses whenever a
  previous request's response had arrived late; and (found during Phase 2) the same
  ROS-side node's `pulseToDeg` uses an assumed 180-degree servo range where Hiwonder's own
  published docs confirm the real range is 240 degrees -- `JetRoverPlantConfig` accounts for this
  explicitly (`PulsePerRadian` vs `PulsePerDegreeAssumed180`) rather than treating it as fixed.
- A real bug was also found and fixed in `JetRoverPlant` itself during Phase 2's hardware testing:
  when a single command's required delta exceeded the per-call direction clamp, the plant
  credited its own belief with the full, unclamped target instead of the amount actually applied
  -- silently stalling large moves forever instead of closing the remaining distance over
  repeated commands. See `JetRoverPlant`'s own doc comment and its regression test.
- Mecanum base motion is explicitly out of scope for now — no wheel odometry exists anywhere in
  the board SDK, so a base plant could only ever be dead-reckoned from commanded velocity, never
  sensed.
- **Phase 3 (real cross-machine `ClockSync` validation) is done, with a real finding.** A new
  `Teleop.Eval` verb, `clocksync-check` (`core/Teleop.Eval/ClockSyncCheck/`), builds a real
  `OperatorEndpoint` over a real UDP socket and talks to an already-running `Teleop.RobotHost`
  over Tailscale. Run against the physical Jetson: `IsSynced` converged, round trips were
  accepted with zero rejections, and measured RTT (~63-110ms depending on network conditions)
  was plausible for a real Tailscale link. The arm moved correctly end-to-end, confirmed by a
  byte-for-byte match between what `JetRoverPlant.Command()` computed and what the relay
  received and decoded.
  - **Real finding, since fixed: `ClockSync`'s offset/OWD numbers were not trustworthy across
    this specific pair of machines.** The Jetson (`.NET` on Linux ARM64) reports
    `TicksPerSecond=1,000,000,000`; this project's Windows dev machine reports `10,000,000` -- a
    100x mismatch. `ClockSync.AddRoundTrip` (`core/Teleop.Core/Time/ClockSync.cs`) added and
    subtracted operator-domain and robot-domain ticks directly, which is only numerically valid
    if both sides' rates agree -- true by construction on every loopback/sweep use (one process,
    one clock), never checked before because Phase 1-2's own hardware tests never depended on the
    diagnostic numbers being right, only on the arm moving correctly. The symptom was unmistakable
    once looked for: `owd_uplink_ms` and `owd_downlink_ms` came out enormous and of opposite sign
    (order 10,000-40,000ms) while still summing back to the real RTT, because the scale error was
    identical and opposite in the two terms. **Fixed** in
    `docs/adr/0008-clocksync-cross-rate-normalization.md`: `RobotStateFrame` now carries the
    robot's own `TicksPerSecond` on every reply (wire v2), and `ClockSync.AddRoundTrip`/
    `ToOperatorTicks` rescale robot-domain stamps into operator-tick-equivalent units before any
    cross-domain arithmetic. **Re-verified against the real Jetson: fixed.** `clocksync-check`
    now reports `IsSynced: True`, `LastRttMs`/`MinRttMs` in the 61-77ms range (matching the real
    Tailscale RTT observed independently, not the previous ~100x-inflated figures),
    `OffsetUncertaintyMs` tracking `LastRttMs/2` exactly as the algorithm intends, zero rejected
    samples, and `owd_uplink_ms + owd_downlink_ms` tracking `MinRttMs` closely, per the tool's own
    sanity check. The reported clock *offset* itself is still a very large number (both machines'
    ticks are uptime-based with an arbitrary per-machine epoch, not wall-clock-synced, so a large
    offset is expected and not a bug — only the RTT/OWD figures, which depend solely on tick
    *differences* within a domain, are the numbers this fix was actually about).
  - **Real finding, found while re-verifying the fix above, unrelated to it and since fixed: at
    the arm's normal 20 Hz command rate, `JetRoverPlant`'s first few (large, correct) corrective
    commands could be silently lost before the physical servo ever executed them.** Confirmed by
    layered diagnosis: a temporary print added to `relay_node.py`'s decode path (reverted, never
    committed) proved the *relay* faithfully received and republished the real, correct
    direction values `JetRoverPlant` computes (e.g. `lower=-3.022 middle=5.000 upper=-5.000` on
    the very first real command toward the validated target) — the same values a hand-crafted
    raw datagram sent directly to the relay's socket reliably moved the arm with. Yet at 20 Hz
    the arm did not move at all across several repeated attempts. Slowing `clocksync-check`'s own
    send rate to 1 Hz (`--rate-hz 1`) made the exact same target move the arm reliably. Root cause
    (confirmed by direct code inspection, not this session's earlier hypotheses): `jetrover_arm_control`'s
    `robot_controller_node.py` subscribed to the servo topics with depth-1 QoS on a
    single-threaded ROS executor, and `ServoController.setPos` did a blocking serial *read*
    (up to a 1s timeout) before every write, plus a hard 0.3s `sleep` after each move — all on the
    executor's only thread. At 20 Hz that ~1.3s worst case meant later commands silently
    overwrote earlier unprocessed ones in the depth-1 queue before their callback ever ran; at
    1 Hz there was no contention. **Fixed** in
    [`jetrover-teleop-ros#1`](https://github.com/andreitimoshukcollege-del/jetrover-teleop-ros/pull/1):
    `ServoController` now tracks a local position belief (mirroring `JetRoverPlant`'s own
    optimistic/sensed pattern) instead of re-reading hardware before every write, replaced the
    blocking sleep with a non-blocking cooldown, and bumped QoS depth to 3 as defense-in-depth.
    Re-verified on the real robot at both 20 Hz and 1 Hz: the arm now moves to the target and
    holds at the real teleop rate. Entirely unrelated to the `ClockSync` wire-format change above
    (confirmed unrelated: `ClockSync` affects only the downlink diagnostic numbers, never the
    uplink command content or its cadence).
  - Getting a real hardware-motion signal required isolating the failure by layer (direct ROS
    topic publish, then a raw datagram straight to the relay's Unix socket, then the full
    `Teleop.RobotHost` pipeline) after an initial false alarm: a diagnostic `pyserial` probe
    opened during debugging toggled the board's DTR line and reset it, which looked identical
    to "nothing moves" until traced back and fixed with a clean ROS-stack restart.
  - Also real, and unrelated to any of the above: the dev machine's `dotnet` process (a native
    Windows process, per this repo's WSL-to-Windows `dotnet` wrapper) and a plain WSL-native
    process take genuinely different network paths to Tailscale -- WSL's own NAT rewrites the
    source port for WSL-native traffic but not for the Windows-native `dotnet` process's own
    traffic. A NAT-discovery step aimed at the wrong process's traffic silently pointed
    `Teleop.RobotHost`'s reply target at a port nothing was listening on.
- **`move-arm` (`core/Teleop.Eval/MoveArm/`) is a general-purpose "move the real arm to x,y,z and
  wait for real convergence" operator tool** (`just move-arm x y z`), built after Phase 3 to make
  ad hoc real-hardware testing easier than hand-rolling a harness each time. Building and using it
  surfaced two more real findings:
  - **A real collision, and two software fixes it drove.** Testing `move-arm` against the arm's
    normal working target drove the lower arm into the robot's own base plate, straining the
    servo against the obstruction until manually corrected (twice, across this session -- see
    below). Root cause: after a `Teleop.RobotHost` restart mid-session, `JetRoverPlant`'s belief
    starts at `ZeroPulse` regardless of the arm's real physical position, so its first *relative*
    step is sized for the wrong starting point and the real servo applies it on top of its own
    true position -- overshooting by the belief/reality gap. Fixed with two changes to
    `JetRoverPlant.cs`: a one-time seed of each joint's belief from real sensed feedback the first
    time it's used (not on every command -- that would reintroduce stale-feedback overshoot for a
    different reason), and a new, safe-by-default, configurable `LowerArmMinPulse` hard limit
    (`--lower-arm-min-pulse`, `Teleop.RobotHost/RobotHostArgs.cs`) preventing the lower arm from
    ever being commanded below a calibrated boundary, regardless of what any IK target asks for.
  - **Calibrating that boundary against the real robot, with a human confirming clearance at
    every step, surfaced a second real bug: a servo command-rate race that had silently
    invalidated the first calibration pass.** `ServoController.setPos` (ROS side) enforces a real
    ~300ms cooldown between physical moves. `move-arm`'s original default send rate was 5Hz
    (200ms) -- faster than that cooldown -- so a final, smallest, most important correction could
    arrive inside the cooldown window and be silently dropped, while `JetRoverPlant`'s belief
    still credited itself with having sent it (no ack exists for "did this specific direction
    actually land"). The first calibration pass converged on 25 pulse as "safe," confirmed by
    eye -- but re-testing at a rate below the cooldown (2Hz) revealed the arm had never actually
    reached 25; it reached a much higher (further from the plate) real pulse the whole time, and
    reducing `--rate-hz` to something the arm could keep up with immediately drove it into a real
    strain at a target the first pass had called safe. Recalibrated at 2Hz with a human confirming
    clearance at every step; the real boundary is 50 pulse, now `RobotHostArgs.DefaultLowerArmMinPulse`.
    `move-arm`'s own default rate is now 2Hz for the same reason.
  - **A third real bug in this same limit: the clamp direction was backwards.** The field was
    originally implemented as an upper bound (`Math.Clamp(value, MinPulse, LowerArmMaxPulse)`),
    which does nothing to stop the lower-arm target from going *below* the calibrated value --
    exactly the dangerous direction on this hardware, since a lower pulse is what drives the
    lower arm toward the plate. Invisible throughout every calibration test above because they
    all retested against the same target, whose unclamped pulse happened to always be *above*
    the limit; surfaced by the operator hitting `move-arm 0.1 0.1 0.1` and `move-arm 0 0 0`,
    whose unclamped pulses are naturally low, and finding the arm went below the calibrated 50
    with nothing stopping it. Fixed by renaming the field to `LowerArmMinPulse` and flipping the
    clamp to `Math.Clamp(value, LowerArmMinPulse, MaxPulse)` -- a floor, not a ceiling. Retested
    against the exact targets that exposed it.
  - **Also real: the ROS-side write-gate itself had two bugs, fixed in the same investigation**
    (`jetrover-teleop-ros#2`). The original gate (`abs(nextPos - currentPos) >= 50`) silently
    dropped any smaller real correction with no error, which is what made the calibration race
    above hard to see -- a fix that removed the gate entirely was tried first and immediately
    caused a real regression on hardware (floating-point noise in `JetRoverPlant`'s asymptotic
    convergence, truncated to whole pulses, drove the arm measurably past the calibrated limit
    over a run of otherwise-idle commands). Landed on a small (5 pulse) noise floor instead of
    zero: large enough to filter truncation noise, small enough that a real 25-pulse correction
    still goes through.
  - Three real hardware strain incidents happened across this session's `move-arm` work, each
    resolved with a manual corrective command (`ros2 topic pub` directly to `/arm/servo/joint/lower`)
    confirmed by the operator watching the hardware before continuing. No damage occurred.

### 2026-10 (Galaxy XR sessions)

- **A rebooted headset froze the arm, silently.** RobotHost dropped every joint command whose
  capture stamp was not newer than the last one it had accepted. The stamps are the operator's own
  clock, which restarts near zero at a reboot. So after the Galaxy XR rebooted, every command was
  "stale" until RobotHost restarted. The camera and the joint-state stream kept working, which made
  it look like a connection problem. Fixed: a new sender address restarts the check
  (`GenericArmPlant.RestartJointCommandOrdering`).
- **A low battery looks like a software fault.** At 9.25 V (3-cell pack, full is 12.6 V):
  - the servo board stopped answering, so the arm ignored commands and the joint values froze;
  - no battery readings came back;
  - the camera kept dropping off USB, 31 reopens;
  - the board's alarm beeped, even on the charger, because the robot running draws more than the
    charger supplies.

  Charge it switched off. The voltage also sagged steadily under use, so the pack may be worn.
- **`/robot/buzzer` crashes the servo driver.** Its handler, `robot_val.py`'s `setBuzzerCbk(self)`,
  takes no message argument, so any message raises a `TypeError` and `jetrover-arm-control` exits
  (systemd restarts it 2 s later). Even when called, it only plays one short beep: there is no
  setting that mutes the low-battery alarm, which is the board firmware's. Fixed in `ros/` on import.
- **The hand-tuned shoulder floor is gone (2026-10-19).** `minAngleRadians -1.885` on motor 2 ("the
  plate", `LowerArmMinPulse` 50) was replaced by a collision model generated from the robot's CAD
  (docs/adr/0020). The headset's IK and RobotHost's Cartesian path never ask for a pose that hits
  the body, and RobotHost refuses one on the joint path, logging `[plant] refused N joint
  command(s)` with the pair that would collide.
  - **Open question:** the CAD puts first contact between the lower arm and the turntable at −33°
    (the model allows −30°). The robot has gone lower before: the validated `move-arm` target used
    −36° with the old IK. Measure where the lower arm really touches before trusting either
    number near the base.
- **Feedback came about once a second, and commands stalled behind it (fixed 2026-10-20).**
  - Each bus-servo position read waited up to 1 s for an answer, and the gripper (motor 10), which
    never answers, was on the feedback list. Every feedback cycle stalled a full second on it.
  - The node handles commands on the same thread, so commands queued behind the stall too: the
    real arm moved in jerks.
  - The relay marks a reading stale after about 0.2 s, so the measured arm kept flickering.
  - Fixed in `robot/ros`: motor 10 is off the list.
  - **Don't shorten the read timeout below 1 s.** Cutting it to 50 ms at the same time stopped all
    servo feedback and the arm, so this board's answers can take longer than that. Reverted.
- **Motor ids:** motor 5 is the wrist roll and motor 10 the gripper. Motors 1–5 report position.
  Motor 10 did not answer position reads until 2026-10-25 (next entry).
- **The gripper answers after all, and is read again (2026-10-25).** `just robot-servo-probe` asked
  motor 10 for its id, position, voltage and temperature: each answered within 3–9 ms. The driver
  reads it again, and now skips any servo for 2 s after a read it does not answer, so a silent
  servo can no longer stall every cycle (docs/adr/0018 §8). The same probe found the gripper at
  **72 °C** and the base at 66 °C, over this README's 60 °C limit. The open gripper sat at pulse 176
  against a target of 167, very likely straining against its open stop: its open and closed pulses
  have never been calibrated. Measure them with `just robot-measure-gripper`.
- **The servo board goes silent, and only the power switch brings it back (open, 2026-10-20).**
  - **What it looks like:** the camera streams, but the arm ignores commands. `just joint-probe`
    shows every motor `sensed NO`, and there is no battery reading.
  - **What `just robot-board-diagnose` showed while it was happening** (read-only, and it never
    opens the port):
    - the driver read **0 bytes** from the board and wrote one position request a second (each
      read waits its 1 s timeout);
    - its receive thread was alive;
    - the USB serial chip stayed connected, and the kernel logged no USB errors and no
      undervoltage.

    So the board's controller stops replying while the chip in front of it stays up.
  - **Ruled out:**
    - a fresh driver process (a redeploy) does not revive it;
    - a DTR or RTS pulse on the port does not either (`just robot-board-reset`).
  - **Not the trigger on their own:** three driver restarts with the headset idle, one with the
    headset streaming, three deploys, and a build on the Jetson (`just deploy-ros false`). The
    board kept answering through all of them.
  - **Earlier cases:**
    - after the read-timeout deploy above, which is probably why "50 ms stopped all feedback" was
      blamed on the timeout;
    - after the redeploy of 2026-10-20 01:51;
    - during the low-battery incident. Each followed a driver restart, but the restart alone did
      not reproduce it.
  - **Evidence now kept:** every 5 s the driver logs its read statistics and the last battery
    voltage (`just arm-logs`), so the journal shows when the reads started failing and the
    voltage just before. Since 2026-10-21 the journal survives reboots (`just robot-journal-persist`,
    capped at 200 MB; the card had 1.7 GB free), so a power-cycle no longer wipes it. Still run
    `just robot-board-diagnose` **before** switching off: it needs the fault present. After a reboot,
    the previous boot's lines are on the robot under `journalctl -b -1 -u jetrover-arm-control`.
    Entries before the clock syncs at boot carry a 2023 date: the Jetson has no clock battery.
- **A measurement trap: `just joint-probe` while the headset streams.** RobotHost sends the
  joint-state stream to whoever last sent on the joint port. A headset sending joint commands takes
  it back 48 times a second, so the probe gets scraps. On 2026-10-20 that read as "feedback falls to
  1.3 Hz under motion" while the headset got everything. The probe now warns when it sees far fewer
  than 50 frames a second. Measure with motion off, or read the driver's and relay's statistics
  lines in `just arm-logs`.
- **The board's torque codes are swapped (found 2026-10-21).** Hiwonder's SDK sends 0x0B for
  torque on and 0x0C for off. On this board, 0x0C leaves a servo stiff (its torque-state read, 0x0D,
  stays 1), and 0x0B makes it limp (reads 0). The driver's `/arm/servo/set/torque` topic was
  therefore always inverted. `robot_controller_sdk.py` is corrected, and goes live with the next
  `just deploy-ros`. `just robot-measure-joint-limits` sends 0x0B itself, so it does not depend on
  the deployed version, and checks the read-back before telling anyone to move the arm.
