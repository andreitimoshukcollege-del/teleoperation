using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.JointDrive
{
    /// <summary>What a safety script may be told from the command line (joint-drive's options).</summary>
    public sealed class SafetyOptions
    {
        /// <summary>
        /// collide_and_recover's push past the collision model's first touch of the object's box, metres (joint-drive
        /// --touch-depth-mm). The model's boxes for the arm are larger than Isaac's link shapes, so its first touch is short of
        /// the real one; the default is <see cref="SafetyScript.TouchDepthMeters"/>, measured on the twin.
        /// </summary>
        public float TouchDepthMeters { get; set; } = SafetyScript.TouchDepthMeters;
    }

    /// <summary>One scene object as a safety script plans around it: its axis-aligned bounds in the arm base frame (metres).</summary>
    public readonly struct SafetyObject
    {
        public readonly byte Id;
        public readonly Vector3 Min;
        public readonly Vector3 Max;

        public SafetyObject(byte id, Vector3 min, Vector3 max)
        {
            Id = id;
            Min = min;
            Max = max;
        }

        public Vector3 Center => (Min + Max) / 2f;

        public CollisionBox Box(float grow) =>
            new CollisionBox($"object {Id}", Min - new Vector3(grow), Max + new Vector3(grow));
    }

    /// <summary>
    /// joint-drive's four safety scripts (docs/adr/0036 amendment 1): each take one episode of one of the lab's categories, the
    /// arm driven relative to where the objects are now. A frame's label is the physics' contact, not the script's intent; the
    /// scripts make the contacts happen, or not, and the take is checked against its category before it is kept.
    /// <list type="bullet">
    /// <item><b>reach_no_obstacle:</b> waypoints, every link kept <see cref="ClearanceMeters"/> (beyond the model's margin) from
    /// every object's box.</item>
    /// <item><b>stop_before_collision:</b> free motion, then the wrist along a straight line towards an object, stopping where
    /// the arm is <see cref="StopGapMinMeters"/>-<see cref="StopGapMaxMeters"/> from its box; a hold, and back out.</item>
    /// <item><b>collide_and_recover:</b> the same line, slowly, to <see cref="TouchDepthMeters"/> past the first touch of the
    /// box; a short hold, and back out before it tips.</item>
    /// <item><b>collide_with_obstacle:</b> the same line, fast, through the object to <see cref="ThroughPastMeters"/> past its
    /// centre, then the wrist lifts away.</item>
    /// </list>
    /// <para>An object is its axis-aligned box, so a rotated object is a little larger than it is: a stop stops a little early,
    /// and a touch may fall short by a few millimetres, which the take check catches. Every pose stays in the envelope and passes
    /// the collision model (floor, body, the arm's own links), as every script's do; only the object is allowed to be hit, by
    /// the two collide scripts. The line's poses come from the headset's IK (<see cref="ArmSolver"/>), solved every
    /// <see cref="LineStepMeters"/>.</para>
    /// </summary>
    public static class SafetyScript
    {
        public const float ClearanceMeters = 0.05f;
        public const float ApproachClearanceMeters = 0.02f;
        public const float StopGapMinMeters = 0.01f;
        public const float StopGapMaxMeters = 0.03f;
        // collide_and_recover goes this deep into the object's box (capped by the box's size). Measured on the twin 2026-11-15:
        // the collision model's arm boxes are larger than Isaac's link shapes (about 15 mm), so a touch must go well into the
        // box to touch in Isaac; pushes measured along the sweep (8 mm reached nothing, 15-25 mm touched, 35 mm tipped a post or
        // grazed) put it near 20 mm. Measured into the box since then: a grazing sweep goes along a box, not into it.
        public const float TouchDepthMeters = 0.020f;
        public const float ThroughPastMeters = 0.04f;

        /// <summary>collide_with_obstacle goes at least this deep into the object's box (or as deep as the box allows): through it, not past its edge.</summary>
        public const float ThroughDepthMeters = 0.012f;
        public const float SweepRadians = 0.6f;
        public const float MinTravelMeters = 0.03f;
        public const float LineStepMeters = 0.005f;
        public const float LiftMeters = 0.08f;
        public const double PreRollSeconds = 3.0;

        /// <summary>The wrist's peak speed along the sweep, m/s: (min, max) per script.</summary>
        public static readonly (float Min, float Max) StopSpeed = (0.08f, 0.16f), TouchSpeed = (0.04f, 0.08f), ThroughSpeed = (0.25f, 0.40f);

        // The wrist is the IK's target and sits well above the table (the JetRover's wrist barely goes below its shoulder), so the
        // gripper comes down beside the object, pointing down, its tip 1-6 cm below the object's top.
        private static readonly float[] Pitches = { -1.5f, -1.2f, -0.9f };
        private static readonly float[] HeightsAboveTop = { 0.10f, 0.12f, 0.08f, 0.14f, 0.06f, 0.16f };
        private static readonly float[] RadiusInside = { 0.0f, 0.02f, 0.04f, 0.06f, 0.08f, -0.02f, 0.10f };

        public static bool IsSafety(DriveScript script) => script == DriveScript.ReachNoObstacle || script == DriveScript.StopBeforeCollision
            || script == DriveScript.CollideAndRecover || script == DriveScript.CollideWithObstacle;

        /// <summary>The lab's category name (the take's scenario and the episode's task string) for a safety script.</summary>
        public static string Category(DriveScript script) => script switch
        {
            DriveScript.ReachNoObstacle => "reach_no_obstacle",
            DriveScript.StopBeforeCollision => "stop_before_collision",
            DriveScript.CollideAndRecover => "collide_and_recover",
            DriveScript.CollideWithObstacle => "collide_with_obstacle",
            _ => throw new ArgumentOutOfRangeException(nameof(script)),
        };

        /// <summary><paramref name="model"/> with every object's box, grown by <paramref name="grow"/>, among its fixed boxes; <paramref name="margin"/> replaces its margin when given.</summary>
        public static ArmCollisionModel WithObjects(ArmCollisionModel model, IReadOnlyList<SafetyObject> objects, float grow, float? margin = null)
        {
            int n = model.Bodies.Length, m = model.Boxes.Length;
            var boxes = new CollisionBox[m + objects.Count];
            Array.Copy(model.Boxes, boxes, m);
            for (int i = 0; i < objects.Count; i++) boxes[m + i] = objects[i].Box(grow);
            var pairs = new List<(int, int)>();
            var bodyBoxes = new List<(int, int)>();
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++) if (model.IgnoresBodyPair(i, j)) pairs.Add((i, j));
                for (int k = 0; k < m; k++) if (model.IgnoresBodyBox(i, k)) bodyBoxes.Add((i, k));
            }

            return new ArmCollisionModel(model.Name + "+objects", model.Bodies, boxes, model.FloorZ, margin ?? model.Margin, model.BaseYaw,
                model.Proximal, model.Distal, model.Wrist, model.Roll, model.Home, pairs.ToArray(), bodyBoxes.ToArray(), model.Source);
        }

        /// <summary>
        /// Appends the script's episode from <paramref name="t"/>, moving <paramref name="current"/> along: until
        /// <paramref name="end"/> for reach_no_obstacle, one approach (with free motion before it) for the others. False, with the
        /// reason, when no object can be approached as the script needs (or, for reach_no_obstacle, no free waypoint exists).
        /// </summary>
        public static bool Append(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, float speed, ref ArmPose current, ref double t, double end, ref SeededRng rng,
            List<DriveLeg> legs, out string? error, SafetyOptions? options = null)
        {
            error = null;
            options ??= new SafetyOptions();
            ArmCollisionModel clear = WithObjects(model, objects, ClearanceMeters);
            if (script == DriveScript.ReachNoObstacle)
            {
                return DragFree(profile, clear, envelope, ref current, ref t, end, ref rng, legs, out error)
                       && DragHome(profile, clear, envelope, ref current, ref t, ref rng, legs);
            }

            if (objects.Count == 0)
            {
                error = "the scene has no objects to approach";
                return false;
            }

            if (!DragFree(profile, clear, envelope, ref current, ref t, t + PreRollSeconds, ref rng, legs, out error)) return false;

            // Objects in a random order; for each, the approach heights and pitches in turn: the first line that works.
            var order = new int[objects.Count];
            for (int i = 0; i < order.Length; i++) order[i] = i;
            for (int i = order.Length - 1; i > 0; i--)
            {
                int j = (int)(rng.NextDouble() * (i + 1)) % (i + 1);
                (order[i], order[j]) = (order[j], order[i]);
            }

            foreach (int oi in order)
            {
                foreach (Sweep sweep in Sweeps(objects[oi], ref rng))
                {
                    // From home first, a cheap test of the sweep; TryEvent solves it again from where the drag in arrives.
                    if (TryLine(profile, model, envelope, objects, oi, sweep, model.Home, out Line line) &&
                        TryEvent(script, profile, model, envelope, objects, oi, sweep, line, ref current, ref t, ref rng, legs, options))
                    {
                        // Home clear of the objects unless the episode knocked one over (then it is anywhere: the arm's own model).
                        ArmCollisionModel homeward = script == DriveScript.CollideWithObstacle ? model : WithObjects(model, objects, ApproachClearanceMeters);
                        return DragHome(profile, homeward, envelope, ref current, ref t, ref rng, legs);
                    }
                }
            }

            error = $"no object could be approached for {Category(script)} from {DrivePlan.Describe(current)}";
            return false;
        }

        /// <summary>
        /// Free movement as an operator's: drag strokes from where the wrist is to reachable points (now and then through one more
        /// on the way), each at a hand's speed, a third of them followed by a pause (the clutch let go), every tick's pose
        /// checked against <paramref name="model"/> (the objects' boxes grown, so it keeps clear of them).
        /// </summary>
        private static bool DragFree(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            ref ArmPose current, ref double t, double end, ref SeededRng rng, List<DriveLeg> legs, out string? error)
        {
            error = null;
            if (!ArmCollision.IsValid(profile, model, current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {ClearanceMeters * 100:0} cm of an object";
                return false;
            }

            var sampler = new TrainingTargetSampler(profile, CourseScript.MinHeightMeters);
            int failed = 0;
            while (t < end)
            {
                var points = new List<Vector3> { DragPath.Wrist(profile, current) };
                int stops = 1 + (rng.NextDouble() < 0.4 ? 1 : 0);
                bool drawn = true;
                for (int k = 0; k < stops && drawn; k++)
                {
                    drawn = sampler.TrySample(ref rng, out Vector3 p, points[points.Count - 1], 0.04f, 60);
                    if (drawn) points.Add(p);
                }

                float peak = DragPath.MinStrokeSpeed + (float)rng.NextDouble() * (DragPath.MaxStrokeSpeed - DragPath.MinStrokeSpeed);
                float pitch = (float)(-0.6 + 0.9 * rng.NextDouble());
                ulong tremor = rng.NextUInt64() | 1;
                if (drawn && DragPath.Follow(profile, model, envelope, current,
                        DragPath.Stroke(points, peak, DragPath.Profile.MinimumJerk, tremor), pitch, out List<ArmPose> poses))
                {
                    DragPath.AddTicks(legs, ref t, current, poses);
                    current = poses[poses.Count - 1];
                    if (rng.NextDouble() < 0.33) DragPath.AddHold(legs, ref t, current, 0.3 + 0.9 * rng.NextDouble());
                    failed = 0;
                    continue;
                }

                if (++failed > 80)
                {
                    error = $"no drag stroke from {DrivePlan.Describe(current)} keeps clear of the objects in 80 tries";
                    return false;
                }
            }

            return true;
        }

        /// <summary>The episode's end as an operator's: a drag stroke back to the home pose's wrist (DrivePlan then settles any
        /// small remainder). Skipped, leaving DrivePlan's own return, when no such stroke is free.</summary>
        private static bool DragHome(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            ref ArmPose current, ref double t, ref SeededRng rng, List<DriveLeg> legs)
        {
            Vector3 from = DragPath.Wrist(profile, current), home = DragPath.Wrist(profile, model.Home);
            if (Vector3.Distance(from, home) < 0.01f) return true;
            float peak = DragPath.MinStrokeSpeed + (float)rng.NextDouble() * (DragPath.MaxStrokeSpeed - DragPath.MinStrokeSpeed);
            if (DragPath.Follow(profile, model, envelope, current,
                    DragPath.Stroke(new[] { from, home }, peak, DragPath.Profile.MinimumJerk, rng.NextUInt64() | 1), -0.3f, out List<ArmPose> poses))
            {
                DragPath.AddTicks(legs, ref t, current, poses);
                current = poses[poses.Count - 1];
            }

            return true;
        }

        /// <summary>The wrist's line towards one object, and its IK poses; the indices where the arm first comes within a gap of the object's box.</summary>
        private readonly struct Line
        {
            public readonly Vector3[] Points;
            public readonly ArmPose[] Poses;
            public readonly int Usable; // poses 0..Usable-1 are reachable, in the envelope and free of everything but the object
            public readonly int Contact; // the first pose touching the object's box (gap 0), or -1
            public readonly float Pitch; // the wrist pitch it was solved at

            public Line(Vector3[] points, ArmPose[] poses, int usable, int contact, float pitch)
            {
                Pitch = pitch;
                Points = points;
                Poses = poses;
                Usable = usable;
                Contact = contact;
            }
        }

        private static bool TryLine(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, int oi, Sweep sweep, in ArmPose from, out Line line)
        {
            // The wrist swings sideways on an arc round the base, at a fixed radius, height and pitch, from SweepRadians to one
            // side of the object's bearing to past it: the arm's reach is a thin shell (measured 2026-11-14: the wrist 14-24 cm
            // out, its gripper pointing down), so an object is met from the side, as a swing knocks it over. The path starts at
            // its first reachable pose clear of every object, and the first touch of the object's box is wherever the collision
            // model finds it, whichever link it is.
            line = default;
            SafetyObject o = objects[oi];
            Vector3 c = o.Center;
            float bearing = MathF.Atan2(c.Y, c.X);
            float halfWidth = MathF.Max(o.Max.X - o.Min.X, o.Max.Y - o.Min.Y) / 2f;
            float past = (halfWidth + ThroughPastMeters) / sweep.Radius;
            float a0 = bearing + sweep.Side * SweepRadians, a1 = bearing - sweep.Side * past;
            int n = (int)MathF.Ceiling(MathF.Abs(a1 - a0) * sweep.Radius / LineStepMeters) + 1;
            ArmCollisionModel touch = WithObjects(model, new[] { o }, 0f, 0f);
            ArmCollisionModel clearOfAll = WithObjects(model, objects, ApproachClearanceMeters);
            var points = new List<Vector3>(n);
            var poses = new List<ArmPose>(n);
            ArmPose at = from;
            int contact = -1;
            for (int i = 0; i < n; i++)
            {
                float a = a0 + (a1 - a0) * i / (n - 1);
                var p = new Vector3(MathF.Cos(a) * sweep.Radius, MathF.Sin(a) * sweep.Radius, sweep.Z);
                bool ok = TrySolve(profile, at, p, sweep.Pitch, out ArmPose pose) && envelope.Contains(pose) && ArmCollision.IsValid(profile, model, pose)
                          && (poses.Count == 0 || poses[poses.Count - 1].MaxJointDelta(pose) <= 0.15f); // no IK branch flip
                if (poses.Count == 0)
                {
                    if (ok && ArmCollision.IsValid(profile, clearOfAll, pose))
                    {
                        points.Add(p);
                        poses.Add(pose);
                        at = pose;
                    }

                    continue;
                }

                if (!ok) break;
                points.Add(p);
                poses.Add(pose);
                at = pose;
                if (contact < 0 && !ArmCollision.IsValid(profile, touch, pose)) contact = poses.Count - 1;
            }

            if (contact < (int)(MinTravelMeters / LineStepMeters)) return false;
            line = new Line(points.ToArray(), poses.ToArray(), poses.Count, contact, sweep.Pitch);
            return true;
        }

        /// <summary>One sideways sweep's geometry: the wrist's radius from the base axis and height (arm base frame), its pitch, and which side it comes from (+1: from the object's left, counter-clockwise of it).</summary>
        private readonly struct Sweep
        {
            public readonly float Radius, Z, Pitch;
            public readonly int Side;

            public Sweep(float radius, float z, float pitch, int side)
            {
                Radius = radius;
                Z = z;
                Pitch = pitch;
                Side = side;
            }
        }

        /// <summary>The sweeps to try for one object, in a seeded order: radii a little inside its own, heights above its top, pitches, both sides.</summary>
        private static List<Sweep> Sweeps(in SafetyObject o, ref SeededRng rng)
        {
            Vector3 c = o.Center;
            float reach = new Vector2(c.X, c.Y).Length();
            var all = new List<Sweep>();
            foreach (float inside in RadiusInside)
            foreach (float above in HeightsAboveTop)
            foreach (float pitch in Pitches)
            foreach (int side in new[] { 1, -1 })
            {
                float r = reach - inside;
                if (r > 0.06f) all.Add(new Sweep(r, o.Max.Z + above, pitch, side));
            }

            for (int i = all.Count - 1; i > 0; i--)
            {
                int j = (int)(rng.NextDouble() * (i + 1)) % (i + 1);
                (all[i], all[j]) = (all[j], all[i]);
            }

            return all;
        }

        private static bool TryEvent(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, int oi, Sweep sweep, Line line, ref ArmPose current, ref double t, ref SeededRng rng, List<DriveLeg> legs,
            SafetyOptions options)
        {
            // Dragged into the sweep's start on a path clear of every object: straight there (through a point a little above the
            // middle), or by way of home's wrist. Then the sweep solved again from where the drag arrived, as the headset's IK
            // would go on from there; its contact and stop or touch points are found on that.
            ArmCollisionModel approachModel = WithObjects(model, objects, ApproachClearanceMeters);
            Vector3 here = DragPath.Wrist(profile, current), start = line.Points[0];
            float peak = DragPath.MinStrokeSpeed + (float)rng.NextDouble() * (DragPath.MaxStrokeSpeed - DragPath.MinStrokeSpeed);
            ulong tremor = rng.NextUInt64() | 1;
            Vector3 above = Vector3.Lerp(here, start, 0.5f) + new Vector3(0, 0, 0.02f + 0.02f * (float)rng.NextDouble());
            if (!DragPath.Follow(profile, approachModel, envelope, current, DragPath.Stroke(new[] { here, above, start }, peak, DragPath.Profile.MinimumJerk, tremor),
                    line.Pitch, out List<ArmPose> inbound) &&
                !DragPath.Follow(profile, approachModel, envelope, current,
                    DragPath.Stroke(new[] { here, DragPath.Wrist(profile, model.Home), start }, peak, DragPath.Profile.MinimumJerk, tremor), line.Pitch, out inbound))
            {
                return false;
            }

            ArmPose arrived = inbound[inbound.Count - 1];
            if (!TryLine(profile, model, envelope, objects, oi, sweep, arrived, out line) || Vector3.Distance(line.Points[0], start) > 0.01f) return false;

            int target;
            double touchAt = 0;
            (float Min, float Max) speeds;
            if (script == DriveScript.StopBeforeCollision)
            {
                float gap = StopGapMinMeters + (float)rng.NextDouble() * (StopGapMaxMeters - StopGapMinMeters);
                ArmCollisionModel near = WithObjects(model, new[] { objects[oi] }, gap, 0f);
                target = -1;
                for (int i = 0; i < line.Contact; i++)
                {
                    if (!ArmCollision.IsValid(profile, near, line.Poses[i])) break;
                    target = i;
                }

                if (target < 2) return false;
                speeds = StopSpeed;
            }
            else if (script == DriveScript.CollideAndRecover)
            {
                // Into the object's box by TouchDepthMeters, not that far along the sweep: a sweep that grazes the box goes along
                // it, not into it (2026-11-15: 20 mm along reached 1-3 mm in, and Isaac's links are ~15 mm inside the model's
                // boxes, so it never touched). Capped at how deep this object's box can be entered at all.
                float want = TouchTarget(objects[oi], options.TouchDepthMeters);
                target = -1;
                float d0 = 0f, d1 = 0f;
                for (int i = line.Contact; i < line.Usable; i++)
                {
                    d1 = Depth(profile, model, objects[oi], line.Poses[i]);
                    if (d1 >= want)
                    {
                        target = i;
                        break;
                    }

                    d0 = Math.Max(d1, 0f);
                }

                if (target < 0) return false;
                // Stop at the depth itself, not at the next 5 mm step past it.
                touchAt = target - 1 + (d1 > d0 ? Math.Clamp((want - d0) / (d1 - d0), 0f, 1f) : 1f);
                speeds = TouchSpeed;
            }
            else
            {
                target = line.Usable - 1;
                if (target < line.Contact + (int)MathF.Ceiling(0.03f / LineStepMeters)) return false; // not far enough through
                // Through the object, not past its edge: some pose well inside its box.
                float through = Math.Min(ThroughDepthMeters, MaxDepth(objects[oi]));
                bool deep = false;
                for (int i = line.Contact; i <= target && !deep; i++) deep = Depth(profile, model, objects[oi], line.Poses[i]) >= through - 0.0005f;
                if (!deep) return false;
                speeds = ThroughSpeed;
            }

            float v = speeds.Min + (float)rng.NextDouble() * (speeds.Max - speeds.Min);
            var newLegs = new List<DriveLeg>();
            double tt = t;
            DragPath.AddTicks(newLegs, ref tt, current, inbound);
            if (arrived.MaxJointDelta(line.Poses[0]) > 0.005f) DragPath.AddTicks(newLegs, ref tt, arrived, Along(line, -1, 0, 0.05f, DragPath.Profile.MinimumJerk, arrived));
            DragPath.AddHold(newLegs, ref tt, line.Poses[0], 0.2 + 0.4 * rng.NextDouble());
            ArmPose last;
            if (script == DriveScript.StopBeforeCollision)
            {
                // Slowing to a stop at the gap, a pause, then dragged back.
                DragPath.AddTicks(newLegs, ref tt, line.Poses[0], Along(line, 0, target, v, DragPath.Profile.MinimumJerk));
                DragPath.AddHold(newLegs, ref tt, line.Poses[target], 1.0 + rng.NextDouble());
                DragPath.AddTicks(newLegs, ref tt, line.Poses[target], Along(line, target, 0, v, DragPath.Profile.MinimumJerk));
                last = line.Poses[0];
            }
            else if (script == DriveScript.CollideAndRecover)
            {
                // Easing into the touch, a moment held, then pulled back before it tips.
                ArmPose touch = PoseAt(line, touchAt);
                DragPath.AddTicks(newLegs, ref tt, line.Poses[0], Along(line, 0, touchAt, v, DragPath.Profile.MinimumJerk));
                DragPath.AddHold(newLegs, ref tt, touch, 0.2 + 0.2 * rng.NextDouble());
                float back = StopSpeed.Min + (float)rng.NextDouble() * (StopSpeed.Max - StopSpeed.Min);
                DragPath.AddTicks(newLegs, ref tt, touch, Along(line, touchAt, 0, back, DragPath.Profile.MinimumJerk));
                last = line.Poses[0];
            }
            else
            {
                // Speeding up and keeping its speed through the object, then lifted away.
                DragPath.AddTicks(newLegs, ref tt, line.Poses[0], Along(line, 0, target, v, DragPath.Profile.EaseIn));
                ArmPose through = line.Poses[target];
                Vector3 p = line.Points[target];
                if (!DragPath.Follow(profile, model, envelope, through,
                        DragPath.Stroke(new[] { p, p + new Vector3(0, 0, LiftMeters) }, 0.08f, DragPath.Profile.MinimumJerk), line.Pitch, out List<ArmPose> lift))
                {
                    return false;
                }

                DragPath.AddTicks(newLegs, ref tt, through, lift);
                last = lift[lift.Count - 1];
            }

            // DrivePlan returns home from here on a path the model calls free: an event that leaves no such path would refuse the
            // whole take (2026-11-15: "no free path back to home"), so another sweep is tried instead.
            if (!DrivePlan.PathValid(profile, model, null, last, model.Home)) return false;
            legs.AddRange(newLegs);
            t = tt;
            current = last;
            return true;
        }

        /// <summary>
        /// How deep <paramref name="pose"/>'s arm is inside object <paramref name="o"/>'s box, metres, to the millimetre: the
        /// most the box can shrink on every side and still be touched (each axis no further than 0.5 mm short of its middle);
        /// below 0 when it does not touch it at all.
        /// </summary>
        private static float Depth(in RobotArmProfile profile, ArmCollisionModel model, in SafetyObject o, in ArmPose pose)
        {
            float best = -1f, cap = MaxDepth(o);
            for (float d = 0f; d <= cap + 1e-6f; d += 0.001f)
            {
                Vector3 half = (o.Max - o.Min) / 2f, s = Vector3.Min(new Vector3(d), half - new Vector3(0.0005f));
                var inner = new SafetyObject(o.Id, o.Min + s, o.Max - s);
                if (ArmCollision.IsValid(profile, WithObjects(model, new[] { inner }, 0f, 0f), pose)) break;
                best = d;
            }

            return best;
        }

        /// <summary>The deepest <see cref="Depth"/> can measure in <paramref name="o"/>: its smallest half-size, less 0.5 mm.</summary>
        public static float MaxDepth(in SafetyObject o)
        {
            Vector3 half = (o.Max - o.Min) / 2f;
            return MathF.Max(0f, MathF.Min(half.X, MathF.Min(half.Y, half.Z)) - 0.0005f);
        }

        /// <summary>
        /// The sweep's poses every tick from index <paramref name="from"/> to <paramref name="to"/> (either way), timed by
        /// <paramref name="profile"/> at a peak of <paramref name="peak"/> m/s along the wrist's arc: its solved poses
        /// interpolated 5 mm apart, as dense as the headset's targets at these speeds. <paramref name="from"/> -1 joins
        /// <paramref name="joinFrom"/> to pose 0 over 5 mm's worth of time.
        /// </summary>
        private static List<ArmPose> Along(in Line line, double from, double to, float peak, DragPath.Profile profile, ArmPose joinFrom = default)
        {
            var poses = new List<ArmPose>();
            double meters = Math.Max(1.0, Math.Abs(to - from)) * LineStepMeters;
            int ticks = Math.Max(1, (int)Math.Ceiling(DragPath.Seconds(profile, meters, peak) / DragPath.TickSeconds));
            for (int k = 1; k <= ticks; k++)
            {
                double s = DragPath.Shape(profile, k / (double)ticks);
                if (from < 0)
                {
                    poses.Add(ArmPose.Lerp(joinFrom, line.Poses[0], (float)s));
                    continue;
                }

                poses.Add(PoseAt(line, from + (to - from) * s));
            }

            return poses;
        }

        /// <summary>The sweep's pose at a fractional index: its solved poses, 5 mm apart, interpolated.</summary>
        private static ArmPose PoseAt(in Line line, double at)
        {
            int i0 = Math.Clamp((int)Math.Floor(at), 0, line.Usable - 1), i1 = Math.Min(i0 + 1, line.Usable - 1);
            return ArmPose.Lerp(line.Poses[i0], line.Poses[i1], (float)Math.Clamp(at - i0, 0, 1));
        }

        /// <summary>How deep collide_and_recover goes into <paramref name="o"/>'s box: <paramref name="depth"/>, but never within 3 mm of its middle (a touch, not a push through).</summary>
        public static float TouchTarget(in SafetyObject o, float depth) => MathF.Max(0.001f, MathF.Min(depth, MaxDepth(o) - 0.003f));

        /// <summary>The headset's IK from <paramref name="from"/> to the wrist <paramref name="target"/>, roll 0, until it gets there.</summary>
        private static bool TrySolve(in RobotArmProfile profile, in ArmPose from, Vector3 target, float pitch, out ArmPose pose)
        {
            var solver = new ArmSolver();
            solver.Reset(profile, from);
            pose = from;
            for (int i = 0; i < TrainingTargetSampler.MaxSolvesPerCheck; i++)
            {
                ArmSolution s = solver.Solve(profile, target, pitch, 0f);
                if (s.Status == ArmSolveStatus.Approaching) continue;
                pose = s.Pose;
                return (s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted) && !s.ReachClamped &&
                       Vector3.Distance(s.Position, target) <= TrainingTargetSampler.ReachToleranceMeters;
            }

            return false;
        }
    }
}
