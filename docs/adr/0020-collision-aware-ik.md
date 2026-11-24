# 20. Collision-aware IK from the robot's CAD; the robot enforces the same model

## Status

Accepted (2026-10-19). The user's choices:
- the robot keeps a backstop, and it is the **same** collision model the IK uses, rather than
  only the servo range;
- when the exact target is blocked, the solver first **tilts the wrist pitch or switches branch**,
  and only then **stops at the closest collision-free pose**.

Amends ADR 0011 (per-joint angle limits) and ADR 0017 (the clutch's workspace projection).

## Context

The arm's IK (`ArmKinematics.TryInverse`) is closed-form, always picks one elbow branch, and
clamps reach only. It knows neither the joints' ranges nor the robot's body. Two things follow:

- **The commanded ("ghost") arm in the headset passes through the chassis, the screen, the
  turntable and its own links.** It also asks for more than the servos' ±120°.
- **The real robot silently ends up somewhere else.** RobotHost clamps every joint to the servo
  range, plus one hand-tuned floor on the shoulder (`minAngleRadians -1.885` on motor 2, "the
  plate", robot/README.md). The display then disagrees with the robot, and the operator learns
  where the body is by hitting it.

The robot's own CAD (ADR 0019's twin, `unity/TeleopXR/Assets/Teleop/Lab/RobotModel/`) describes
the body exactly. What was missing is a model of it that Core can evaluate.

## Decision

1. **A collision model in `Teleop.RobotArm` (`Collision/`), plain data:**
   - **Moving parts:** oriented boxes attached to link frames. The frames are the turntable (yaw),
     the proximal link, the distal link, the wrist (pitch) and the gripper (wrist plus roll).
     Boxes, not capsules: the links are flat brackets about 5 cm wide and 2 cm thick, and a round
     cross-section sized to the width overhangs the thickness by centimetres. That made the arm
     collide with itself at rest.
   - **The body:** axis-aligned boxes in the arm base frame (Core's convention: origin at the yaw
     pivot, Z up, X forward), covering the chassis, the screen, the lidar and the wheels.
   - **A floor plane**, and a clearance margin (1 cm).
   - **Box tests:** the separating-axis test (exact), with the margin added to one side.
   - **Adjacent links are never box-tested** (the gripper counts as adjacent to the distal link,
     since it hangs off the wrist joint). Parts meeting at a joint nest millimetres apart by
     design, finer than boxes and a margin can resolve. Their contact is each joint's **range**
     instead.

   It is Core: deterministic, allocation-free per call, no Unity.
2. **The model is generated from the CAD, never hand-tuned.** `just robot-collision-model` runs
   `RobotCollisionExtractor` in the TeleopXR editor:
   - **Sampling and fitting:** it samples every part's surface every 2.5 mm, in its own link frame.
     It fits 1 to 6 oriented boxes per link and up to 20 boxes to the body (k-means clusters, the
     smaller of a principal-axis box and a link-aligned one).
   - **Joint ranges:** each joint is swept 1° at a time against the parent link's surface samples,
     to first contact within 3.5 mm, then backed off 2°. Samples within 1.5 cm of the joint axis
     are left out, where the servo horn and bracket touch by construction. The wrist is swept with
     the gripper at five rolls.
   - **Skip list:** pairs in contact in at least 98% of 3,000 random poses are skipped, and so is
     the proximal link's piece at the shoulder against the body (it sits millimetres from the
     housing it turns in). The outer pieces, which are what reach the plate, stay checked.
   - **Validation** fails the run, writing nothing:
     - every surface sample must lie inside its own part's boxes, so a real contact between two
       checked parts always lies inside both parts' boxes and is caught;
     - Core's link frames must match Unity's rig within 1 mm;
     - the home pose must be valid.
   - **Output:** `Collision/JetRoverCollisionModel.g.cs`, stamped with the FBX hashes, plus a
     report and preview images.
   - The editor assembly references `Teleop.RobotArm` for this. That is editor tooling, not the
     runtime boundary unity/CLAUDE.md guards.
3. **`ArmSolver` replaces direct `TryInverse` calls in the operator bridge and RobotHost's Cartesian
   path.** Its search is deterministic:
   1. The target is pulled into reach.
   2. Of every valid pose at the target, it takes the one closest to the current pose. The
      candidates are four closed-form branches (elbow either way; base facing the target or half
      round, reaching over the top), each with the pitch tilted by 0, ±5°, … ±90°. Closest means
      the least joint motion, counting each radian of tilt as one radian of motion, so an exact
      pitch wins unless keeping it costs a bigger swing.
   3. No joint moves more than **30° per solve**. A goal further than that is approached along the
      straight joint path, 30° per solve, each step checked. A target held still is reached over a
      few frames. This avoids the dead ends a straight line in space hits, for example passing
      over the shoulder inside the reach the elbow's range allows.
   4. A goal more than **90° away in any joint** is not approached on its own. Reaching it would
      mean swinging the base half round or flipping the elbow; on the headset (2026-10-20) that
      looked like the arm doing a full turn when the hand passed the base's ±120°. The arm stops
      instead.
   5. If no valid pose at the target can be reached that way, the arm stops at the furthest point
      toward it: bisection along the segment from the last pose reached.

   - **Status:** `Exact`, `PitchAdjusted`, `Approaching` (on the way, not compromised) or `Blocked`.
   - **A joint jump beyond 8°** also requires the straight joint path to it to be collision-free,
     unless the arm starts in a pose the model already calls colliding (for example folded at
     rest), when the next pose is judged on its own.
   - **At most 600 collision checks per solve.** Past that, the arm holds its best valid answer. A
     count rather than a time, so the result is the same on every machine.
   - **`TryInverse` stays** as the closed-form inner step.
4. **The robot enforces the same model.** `GenericArmPlant` runs it on every joint command and
   refuses a colliding pose whole (holds still, logs the pair once per episode). Its Cartesian path
   solves with `ArmSolver`. The servo-range pulse clamp stays, since it is physical. **The
   hand-tuned shoulder floor is removed** from `jetrover.json` and `JetRoverMeasuredDefault`. The
   generic `Min/MaxAngleRadians` mechanism stays, unused by the shipped profile.
5. **While joint commands arrive, they alone move the arm.** The headset sends both its IK's joint
   angles and, for ClockSync and latency, the Cartesian target. RobotHost used to apply both, which
   was harmless while both ends ran the same stateless closed form. With a stateful solver on each
   end, the two can choose different poses, and the robot alternated between them, doing things
   the ghost did not show (2026-10-20). `GenericArmPlant` therefore ignores a Cartesian command for
   motion within half a second of a joint command, while still replying to it. `move-arm`, which
   sends Cartesian commands only, is unaffected.
6. **The headset shows the result truthfully.**
   - The wrist LED is red when blocked or out of reach, and amber when the pitch was adjusted.
   - With `snapTargetToReachable` (on in TeleopXR, off in the parked TeleopVR), the drag target
     moves to the pose the arm actually reaches, except while approaching, when the arm is still
     on its way. The blue marker then sits on the ghost's wrist, and
     re-engaging the clutch starts from there.

### Amendment (2026-10-21): a jumpy roll no longer stops the arm

The headset reported reachable targets as blocked, mostly with the gripper pointing down. A headless
replay of the solver along hand paths found three roll faults, none of them the collision model's:

1. **A target past straight down read back with its roll half a turn round.** The clutch's pitch is
   relative (the target's at engage, plus the hand's change since), so it passes −90° easily. The
   bridge reads pitch and roll back from the drag target's rotation, and past vertical the plain
   reading is the same orientation at pitch within ±90° with the roll turned 180° and the heading
   reversed. The IK ignores heading, so to it that was a different pose. Re-engaging the clutch read
   its roll back the same way. Both now use `ArmKinematics.ExtractPitchRollNear`, which takes the
   reading nearer the previous frame's.
2. **Within 0.6° of vertical the roll read 0.** `ExtractRollRadians` now measures it from heading 0
   there, which is exact for the clutch's pitch · roll target.
3. **The gripper hand's roll was a difference of two absolute rolls,** which swings wildly when the
   hand points straight down (heading and roll are the same turn there). It is now the hand's twist
   about its own pointing axis since the anchor (`WristRollMapping.TwistRadians`).

**The solver also stops letting the roll block the arm.** The roll counted against the 30° per-solve
and 90° approach caps like an arm joint, so one frame of a jumpy roll froze the arm. It now turns
toward the request at up to `MaxRollPerSolve` (30°) per solve and is otherwise just part of the pose
that is checked.

Over random hand paths in clearly free space (targets at least 15 cm up), with the gripper pointing
down through vertical: before, 424 of 2,109 frames were blocked (raw orientation). After, in the
clutch's flow, 2 of 1,998 frames were blocked with the target swinging past vertical, and 46 of
1,760 hovering at straight down with a 60° roll. What remains is real: pointing straight down
from near the body needs the wrist past its −103° limit, or the gripper's tip passes through the
deck between two tilts. A replay in a scratch script, not a benchmark row.

### Amendment (2026-10-21): joint stops measured on the robot

The CAD's sweep gives each joint's range, but where the robot was measured going further, the
measurement now wins. `just robot-measure-joint-limits` switches off the torque on chosen servos;
someone moves each joint by hand until it touches, and the furthest angles reached are printed and
saved on the robot. The generator's `MeasuredStops` table records each stop with its date. That end
of the range becomes the stop less 2°, within the servo's range. Everything else is still the CAD.

| Joint | CAD sweep | Measured (2026-10-21) | Now |
|---|---|---|---|
| Wrist down | −103° (contact −106°) | −115.2°, −114.7° | −112.7° |
| Wrist up | +115° (contact +118°) | +119.0° | +117° |
| Shoulder down | −30° (contact −33°) | not reached | −30° |

The shoulder stopped near −14° both times, but that was the gripper meeting the deck, which the
boxes model. It was not the lower arm meeting the turntable, which is what the joint's range covers.
So the shoulder's −30° is still the CAD's, and still unverified. Measure it with the elbow straight
and the gripper clear of the deck.

On this board the torque codes are the other way round from Hiwonder's SDK: 0x0B switches the
torque off and 0x0C on (read back with 0x0D). The SDK in `robot/ros` is corrected.

### Amendment (2026-11-24): a refused joint command is clamped, not dropped

Decision 4's backstop refused a failing command whole and the arm held. Since the 2026-11 bug hunt (C-05) it also judges the
servo path from the arm's **measured** pose, and the arm trails its commands by about 0.2 s, so commands that are themselves
clear were refused while the arm lagged: 1.0% of all twin commands, in runs of up to 3 s that ended in a jump of about 25°,
and 2.0% of the real course sessions (docs/adr/0038 "M, motion"). The world-models side asked for the arm to keep moving
toward the command instead, and the user approved it. Built on branch `work/v2-clamp6`; **not yet run on a twin or the robot.**

- **The rule** (`GenericArmPlant.CommandJointAngles`, `LineClamp`). Passing, the command is sent as before (`JC` `A`). Failing,
  the plant sends the farthest point toward it on the joint-space line s + α(c − s), α in [0, 1], from the measured pose s
  (put into the model's joint ranges, the start the servo-path check itself uses) that passes **the same check**, the end pose
  and the servo path from s (`JC` `C`, its `RO`, and `JK t seq alpha`). Only the joints the check covers move along the line:
  base yaw, proximal, distal, wrist, wrist roll. The gripper's opening (servo 10) and every speed are the command's. Nothing
  passing, the command is refused as before (`JC` `R`, no `RO`), and the arm holds.
- **The search.** The line's start first: failing, nothing passes. Then 8 rounds of bisection, to 1/256 of the line. Bisection
  alone cannot see a second passing stretch beyond its failing bound, and the check is not monotone along the line: the servo
  path to a nearer point is not part of the path to a farther one, and paths under 4° are not sampled at all. So the grid
  points k/16 beyond the failing bound are asked from the end down, stopping at the first that passes. One passing means the
  line is not monotone, and that largest passing grid point is the answer. A failure below the passing bound is not looked
  for: it cannot give a larger α, and every candidate is judged on its own from s. At most 24 checks, deterministic and
  allocation-free.
- **The invariant.** The targets about to be sent are judged once more, exactly as a received command is, from the same
  measured pose; failing, they are refused and held, and `ClampInvariantViolations` counts it. So the clamp never sends what
  the check refuses. Tests over 300 refused random commands, and the replay of 664 twin takes below, found it 0 every time.
- **Order.** Check, clamp, then the command gate (docs/adr/0038 N), which judges the clamped command, then the relay. The park
  (docs/adr/0023) never clamps: its path is planned and checked first, so a refused waypoint is a bug, and the park holds.
- **Records** (trainer/contract/SESSION_FORMAT.md): `JC` outcome `C`; `RO` for every command sent (`A` and `C`), with the
  targets sent; `JK|t|seq|alpha` per `JC` `C`; and `HK|t|measured|clamp|1` (or `refuse`) at the start of every session. A
  session without `HK` refused.
- **One flag back:** `--host-check-onfail refuse` restores the old behaviour exactly. The default is `clamp`, on the twin and
  the real robot alike (the same RobotHost). RobotHost prints which at start, and a throttled line of clamps and refusals.
- **Offline replay.** `Teleop.RobotHost --replay-commands [--host-check-onfail clamp|refuse] <robothost.jlog>...` runs a
  take's `JC` through the plant with its `RI` where they were recorded; it exits 1 when refuse mode does not reproduce every
  recorded outcome or a clamp fails its re-check. Open loop: after a clamp the recorded arm still holds, so each command's
  outcome is right for its measured pose, while a live clamp would shorten the runs (the arm follows). At ad61965, Release,
  on the GPU machine:

| | The 12 mixed takes of 2026-11-24 18:44-18:49 | All 664 JetRover twin takes (both twins) | Real `free-play` (2 takes) |
|---|---|---|---|
| Commands replayed | 10,396 | 515,479 | 2,682 |
| Refuse mode against the recording | 0 differ | 0 differ | 0 differ |
| Refused before | 322 (3.1%, 10 takes) | 5,629 (1.1%, 281 takes) | 0 |
| Clamped now / still held | 322 / 0 | 5,617 / 12 (3 with the measured pose itself failing) | – |
| From the grid (non-monotone) | 2 | 73 (1.3% of clamps) | – |
| α p10 / p25 / p50 / p75 / p90 | 0.023 / 0.21 / 0.41 / 0.69 / 0.91 | 0.10 / 0.22 / 0.45 / 0.75 / 0.90 | – |
| Checks per search, p50 / max | 18 / 24 | 17 / 24 | – |
| Plant time, `A` command, p50 / p99 | 4.1 / 14 µs | 4.5 / 23 µs | 7.5 / 26 µs |
| Plant time, `C` command, p50 / p99 / max | 42 / 101 / 856 µs | 48 / 167 / 1,118 µs | – |

  α spreads across (0, 1): over all twin takes about 10% per tenth, 551 under 0.1. Small α means the arm barely moves
  toward the command; in a live run it would then be measured nearer and the next command's α larger.
- **Cost.** A stress test of 300 commands drawn anywhere in the ranges (largest joint change p50 165°, far beyond any lag):
  p50 380 µs, p99 817 µs, max 892 µs per clamped command on the GPU machine (`HostCheckClampTests`, logged). The Jetson's
  cores are slower, by an unmeasured factor (perhaps 3-5x); even so the replay's worst clamp would be a few ms against a
  command every 20 ms. A command's check cost grows with its distance from the measured pose (one sample per 2° of the
  largest joint change), and the clamp asks at most 26 of them.
- **Not checked:** the six real headset sessions of 2026-10-25 and 27 (2.0% refused in the two course sessions) are on the
  laptop and were not replayed here; nothing ran on the twin or the robot. A twin check should show `HK` in every take, `JC`
  `C` with a `JK` and an `RO` each, far fewer `R`, and no `CLAMP RESULT(S) FAILED THE RE-CHECK` line. The real-robot test,
  supervised, should confirm the start line, then drive the course with the arm lagging (fast moves near the deck and the
  body) and look for: no contact, the arm creeping toward the command where it used to freeze and jump, no oscillation at the
  boundary, and `JC` `C` in the session with no re-check failures.

## Consequences

- **The ghost and the robot agree by construction:** one model, one solver, both sides.
- **The model is conservative,** boxes with a margin, so the arm stops slightly before real
  contact.
- **The CAD limits the shoulder to −30° downward,** where the lower arm's underside meets the
  turntable's top. The robot has been driven lower than that before: the validated `move-arm`
  target used −36° with the old IK, and the old floor sat near −115°. Either the CAD differs from
  the real turntable or bracket there, or that pose was closer to contact than anyone saw. Until
  it is measured on the robot, the CAD's answer stands. It costs reach close to the base, never a
  collision.
- **The model covers only the robot, not its surroundings.** The bench is the floor plane;
  nothing else in the room is modelled.
- **The model follows the CAD.** A CAD change means rerunning `just robot-collision-model`, and its
  validation, before trusting it.
- **Branch switches are rarer than before but larger.** They are where the joint-space path check
  matters most.
- **A blocked solve costs more.** Measured with the generated model on this PC (5,000 solves, two
  thirds of them blocked): mean 0.27 ms, 99th percentile 1.2 ms, worst 2.6 ms. The test suite logs
  it. The headset's CPU is slower, which is what the per-solve check budget is for.
