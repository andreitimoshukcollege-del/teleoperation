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
   path.** Its search order is deterministic:
   1. reach projection;
   2. pitch offsets ordered by size (0, ±5°, … ±90°), within the previous solution's branch;
   3. the other branches: elbow mirror, and base yaw ±π reaching over the top within the yaw range;
   4. if nothing is valid, bisection along the segment from the last valid pose toward the target.

   - **Status:** `Exact`, `PitchAdjusted` or `Blocked`.
   - **A large joint jump** (a branch switch) also requires the straight joint-space path to it to
     be collision-free, unless the arm starts in a pose the model already calls colliding (for
     example folded at rest), when the next pose is judged on its own.
   - **At most 600 collision checks per solve.** Past that, the arm holds its best valid answer. A
     count rather than a time, so the result is the same on every machine.
   - **`TryInverse` stays** as the closed-form inner step.
4. **The robot enforces the same model.** `GenericArmPlant` runs it on every joint command and
   refuses a colliding pose whole (holds still, logs the pair once per episode). Its Cartesian path
   solves with `ArmSolver`. The servo-range pulse clamp stays, since it is physical. **The
   hand-tuned shoulder floor is removed** from `jetrover.json` and `JetRoverMeasuredDefault`. The
   generic `Min/MaxAngleRadians` mechanism stays, unused by the shipped profile.
5. **The headset shows the result truthfully.**
   - The wrist LED is red when blocked or out of reach, and amber when the pitch was adjusted.
   - With `snapTargetToReachable` (on in TeleopXR, off in the parked TeleopVR), the drag target
     moves to the pose the arm actually reaches. The blue marker then sits on the ghost's wrist, and
     re-engaging the clutch starts from there.

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
