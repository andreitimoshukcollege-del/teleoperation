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
        public const float TouchDepthMeters = 0.008f;
        public const float ThroughPastMeters = 0.04f;
        public const float SweepRadians = 0.6f;
        public const float MinTravelMeters = 0.03f;
        public const float LineStepMeters = 0.005f;
        public const float LiftMeters = 0.08f;
        public const double PreRollSeconds = 3.0;

        /// <summary>The wrist's speed along the line, m/s: (min, max) per script.</summary>
        public static readonly (float Min, float Max) StopSpeed = (0.08f, 0.16f), TouchSpeed = (0.03f, 0.06f), ThroughSpeed = (0.25f, 0.40f);

        // The wrist is the IK's target and sits well above the table (the JetRover's wrist barely goes below its shoulder), so the
        // gripper comes down beside the object, pointing down, its tip 1-6 cm below the object's top.
        private static readonly float[] Pitches = { -1.5f, -1.2f, -0.9f };
        private static readonly float[] HeightsAboveTop = { 0.10f, 0.12f, 0.08f, 0.14f };
        private static readonly float[] RadiusInside = { 0.0f, 0.02f, 0.04f, 0.06f, 0.08f };

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
            List<DriveLeg> legs, out string? error)
        {
            error = null;
            ArmCollisionModel clear = WithObjects(model, objects, ClearanceMeters);
            if (script == DriveScript.ReachNoObstacle)
            {
                return Waypoints(profile, clear, envelope, speed, ref current, ref t, end, ref rng, legs, out error);
            }

            if (objects.Count == 0)
            {
                error = "the scene has no objects to approach";
                return false;
            }

            if (!Waypoints(profile, clear, envelope, speed, ref current, ref t, t + PreRollSeconds, ref rng, legs, out error)) return false;

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
                    if (TryLine(profile, model, envelope, objects, oi, sweep, model.Home, out Line line) &&
                        TryEvent(script, profile, model, envelope, objects, oi, line, ref current, ref t, ref rng, legs))
                    {
                        return true;
                    }
                }
            }

            error = $"no object could be approached for {Category(script)} from {DrivePlan.Describe(current)}";
            return false;
        }

        private static bool Waypoints(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, float speed,
            ref ArmPose current, ref double t, double end, ref SeededRng rng, List<DriveLeg> legs, out string? error)
        {
            error = null;
            if (!ArmCollision.IsValid(profile, model, current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {ClearanceMeters * 100:0} cm of an object";
                return false;
            }

            while (t < end)
            {
                string? why = DrivePlan.NextWaypoint(profile, model, envelope, current, speed, t, ref rng, out DriveLeg leg);
                if (why != null)
                {
                    error = why;
                    return false;
                }

                legs.Add(leg);
                t = leg.End;
                current = leg.To;
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

            public Line(Vector3[] points, ArmPose[] poses, int usable, int contact)
            {
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
            line = new Line(points.ToArray(), poses.ToArray(), poses.Count, contact);
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
            IReadOnlyList<SafetyObject> objects, int oi, in Line line, ref ArmPose current, ref double t, ref SeededRng rng, List<DriveLeg> legs)
        {
            int target;
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
                // The first touch lies between the last free pose and the contact pose: TouchDepthMeters on from the free one,
                // so the push is at most that, plus nothing.
                target = line.Contact - 1 + (int)MathF.Ceiling(TouchDepthMeters / LineStepMeters);
                if (target >= line.Usable) return false;
                speeds = TouchSpeed;
            }
            else
            {
                target = line.Usable - 1;
                if (target < line.Contact + (int)MathF.Ceiling(0.03f / LineStepMeters)) return false; // not far enough through
                speeds = ThroughSpeed;
            }

            // Into the line's start on a path clear of every object: straight there, or by way of home.
            ArmCollisionModel approachModel = WithObjects(model, objects, ApproachClearanceMeters);
            bool direct = DrivePlan.PathValid(profile, approachModel, envelope, current, line.Poses[0]);
            if (!direct && !(DrivePlan.PathValid(profile, approachModel, envelope, current, model.Home)
                             && DrivePlan.PathValid(profile, approachModel, envelope, model.Home, line.Poses[0]))) return false;
            ArmPose lifted = default;
            bool lift = script == DriveScript.CollideWithObstacle;
            if (lift && !TryLift(profile, model, envelope, line, target, out lifted)) return false;

            float v = speeds.Min + (float)rng.NextDouble() * (speeds.Max - speeds.Min);
            var newLegs = new List<DriveLeg>();
            double tt = t;
            ArmPose via = current;
            if (!direct)
            {
                double dh = DrivePlan.SmoothDuration(current.MaxJointDelta(model.Home), 1.5f);
                newLegs.Add(new DriveLeg(tt, dh, 0.2, current, model.Home, LegPhase.Script));
                tt = newLegs[newLegs.Count - 1].End;
                via = model.Home;
            }

            double d0 = DrivePlan.SmoothDuration(via.MaxJointDelta(line.Poses[0]), 1.5f);
            newLegs.Add(new DriveLeg(tt, d0, 0.3 + 0.4 * rng.NextDouble(), via, line.Poses[0], LegPhase.Script));
            tt = newLegs[newLegs.Count - 1].End;
            AddLine(line, 0, target, v, ref tt, newLegs);
            ArmPose last = line.Poses[target];
            if (script == DriveScript.StopBeforeCollision)
            {
                newLegs.Add(new DriveLeg(tt, 0, 1.0 + rng.NextDouble(), last, last, LegPhase.Script));
                tt += newLegs[newLegs.Count - 1].Hold;
                AddLine(line, target, 0, v, ref tt, newLegs);
                last = line.Poses[0];
            }
            else if (script == DriveScript.CollideAndRecover)
            {
                newLegs.Add(new DriveLeg(tt, 0, 0.2 + 0.2 * rng.NextDouble(), last, last, LegPhase.Script));
                tt += newLegs[newLegs.Count - 1].Hold;
                AddLine(line, target, 0, StopSpeed.Min, ref tt, newLegs);
                last = line.Poses[0];
            }
            else
            {
                newLegs.Add(new DriveLeg(tt, 0, 0.3, last, last, LegPhase.Script));
                tt += 0.3;
                double dl = DrivePlan.SmoothDuration(last.MaxJointDelta(lifted), 1.0f);
                newLegs.Add(new DriveLeg(tt, dl, 0.5, last, lifted, LegPhase.Script));
                tt = newLegs[newLegs.Count - 1].End;
                last = lifted;
            }

            legs.AddRange(newLegs);
            t = tt;
            current = last;
            return true;
        }

        /// <summary>Linear legs along the line's poses from index <paramref name="from"/> to <paramref name="to"/> (either way) at <paramref name="speed"/> m/s.</summary>
        private static void AddLine(in Line line, int from, int to, float speed, ref double t, List<DriveLeg> legs)
        {
            int step = to >= from ? 1 : -1;
            for (int i = from; i != to; i += step)
            {
                double d = Math.Max(0.01, Vector3.Distance(line.Points[i], line.Points[i + step]) / speed);
                legs.Add(new DriveLeg(t, d, 0, line.Poses[i], line.Poses[i + step], LegPhase.Script, linear: true));
                t += d;
            }
        }

        /// <summary>The wrist <see cref="LiftMeters"/> above the line's pose <paramref name="at"/>, on a path free of everything but the object.</summary>
        private static bool TryLift(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, in Line line, int at, out ArmPose lifted)
        {
            Vector3 up = line.Points[at] + new Vector3(0, 0, LiftMeters);
            foreach (float pitch in Pitches)
            {
                if (TrySolve(profile, line.Poses[at], up, pitch, out lifted) && DrivePlan.PathValid(profile, model, envelope, line.Poses[at], lifted))
                {
                    return true;
                }
            }

            lifted = default;
            return false;
        }

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
