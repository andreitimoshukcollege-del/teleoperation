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

        /// <summary>
        /// docs/adr/0037 P4: the heaviest movable object, at its size, kg, that the arm knocks over (the scene's knockMaxMass,
        /// measured on the twin). Above 0, roles apply: collide_with_obstacle aims only at <see cref="Knockable"/> objects, and
        /// collide_and_recover only at ones a touch does not tip (<see cref="Steady"/>). 0: collide_with_obstacle aims at any
        /// movable object and collide_and_recover at any object.
        /// </summary>
        public double KnockMaxKg { get; set; }

        /// <summary>Whether <paramref name="o"/> is one the arm knocks over: movable, and within <see cref="KnockMaxKg"/> when it is set.</summary>
        public bool Knockable(in SafetyObject o) => o.Movable && (KnockMaxKg <= 0 || o.Mass <= KnockMaxKg);

        /// <summary>
        /// Whether a touch leaves <paramref name="o"/> standing: a wall, one too heavy to knock over, or a squat one (pushed, a
        /// squat object slides before it tips; see <see cref="SafetyObject.Tippy"/>).
        /// </summary>
        public bool Steady(in SafetyObject o) => !o.Movable || (KnockMaxKg > 0 && o.Mass > KnockMaxKg) || !o.Tippy;
    }

    /// <summary>One scene object as a safety script plans around it: its axis-aligned bounds in the arm base frame (metres).</summary>
    public readonly struct SafetyObject
    {
        public readonly byte Id;
        public readonly Vector3 Min;
        public readonly Vector3 Max;

        /// <summary>Whether physics can move it (a scene's movable object): only such an object can be knocked over.</summary>
        public readonly bool Movable;

        /// <summary>Its mass at its size, kg (0 for a fixed object): what decides whether the arm can knock it over (docs/adr/0037 P4).</summary>
        public readonly double Mass;

        /// <summary>
        /// Whether a push tips it before it slides (docs/adr/0037 P4): taller than <see cref="TippyAspect"/> times its narrower
        /// side. Pushed at mid-height, an object tips first when its height exceeds its width over its friction (about 0.8).
        /// </summary>
        public readonly bool Tippy;

        /// <summary>Height over narrower side above which a pushed object tips before it slides.</summary>
        public const double TippyAspect = 1.25;

        public SafetyObject(byte id, Vector3 min, Vector3 max, bool movable = true, double mass = 0, bool tippy = false)
        {
            Id = id;
            Min = min;
            Max = max;
            Movable = movable;
            Mass = mass;
            Tippy = tippy;
        }

        public Vector3 Center => (Min + Max) / 2f;

        public CollisionBox Box(float grow) =>
            new CollisionBox($"object {Id}", Min - new Vector3(grow), Max + new Vector3(grow));

        /// <summary>This object's box grown by <paramref name="grow"/> on every side (shrunk when negative, never past its middle).</summary>
        public SafetyObject Grown(float grow)
        {
            if (grow >= 0) return new SafetyObject(Id, Min - new Vector3(grow), Max + new Vector3(grow), Movable, Mass, Tippy);
            Vector3 s = Vector3.Min(new Vector3(-grow), (Max - Min) / 2f - new Vector3(0.0005f));
            return new SafetyObject(Id, Min + s, Max - s, Movable, Mass, Tippy);
        }
    }

    /// <summary>
    /// joint-drive's four safety scripts (docs/adr/0036 amendments 1 and 2): each take one episode of one of the lab's
    /// categories, the arm driven relative to where the objects are now, by a person as <see cref="HumanOperator"/> models
    /// one (overlapping submovements, the clutch let go and taken again on the move, the hand's jitter, the roll and gripper
    /// in use). A frame's label is the physics' contact, not the script's intent; the scripts make the contacts happen, or
    /// not, and the take is checked against its category before it is kept.
    /// <list type="bullet">
    /// <item><b>reach_no_obstacle:</b> free movement, every link kept <see cref="ClearanceMeters"/> (beyond the model's margin)
    /// from every object's box.</item>
    /// <item><b>stop_before_collision:</b> free movement, then a move to a point beside an object and a slowing final approach
    /// toward it that stops with the arm <see cref="StopGapMinMeters"/>-<see cref="StopGapMaxMeters"/> from its box; a hold,
    /// and back out.</item>
    /// <item><b>collide_and_recover:</b> the same approach, slowly, to <see cref="TouchDepthMeters"/> into the box; a moment
    /// in contact, and back out before it tips.</item>
    /// <item><b>collide_with_obstacle:</b> a fast swipe through the object, then lifted away.</item>
    /// </list>
    /// <para>Where an approach goes is found on a sideways sweep round the base past the object (the arm's reach is a thin
    /// shell, so an object is met from the side; <see cref="Sweeps"/>). How far the final approach goes is one number,
    /// searched with every random draw held fixed until the closest approach (or depth) the collision model measures over
    /// every tick is the category's. An object is its axis-aligned box, so a rotated object is a little larger than it is: a
    /// stop stops a little early, and a touch may fall short by a few millimetres, which the take check catches. Every pose
    /// stays in the envelope and passes the collision model (floor, body, the arm's own links), as every script's do; only the
    /// object is allowed to be hit, by the two collide scripts. Every tick is solved by the headset's IK
    /// (<see cref="ArmSolver"/>).</para>
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
        /// <summary>Free movement before an approach, seconds (uniform between the two).</summary>
        public const double PreRollMinSeconds = 3.0, PreRollMaxSeconds = 8.0;

        /// <summary>The gripper during an approach: open, as collide_and_recover's touch depth was measured (2026-11-15).</summary>
        public const float ApproachGripper = 0f;

        /// <summary>The final approach starts this far (metres, uniform between the two) before where it aims, along the sweep.</summary>
        public const float FinalApproachMinMeters = 0.05f, FinalApproachMaxMeters = 0.10f;

        /// <summary>A touch is at most this far (metres) from its depth, and in contact this long (seconds), by the collision model.</summary>
        public const float TouchToleranceMeters = 0.0015f;
        public const double TouchMinSeconds = 0.12, TouchMaxSeconds = 1.2;

        /// <summary>How many approaches (object, sweep) are built before the script gives up.</summary>
        public const int MaxBuilds = 40;


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
        /// <paramref name="end"/> for reach_no_obstacle, one approach (with free movement before it) for the others. False, with
        /// the reason, when no object can be approached as the script needs (or, for reach_no_obstacle, no free movement exists).
        /// </summary>
        public static bool Append(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, float speed, ref ArmPose current, ref double t, double end, ref SeededRng rng,
            List<DriveLeg> legs, out string? error, SafetyOptions? options = null)
        {
            error = null;
            options ??= new SafetyOptions();
            ArmCollisionModel clear = WithObjects(model, objects, ClearanceMeters);
            if (!ArmCollision.IsValid(profile, clear, current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {ClearanceMeters * 100:0} cm of an object";
                return false;
            }

            var hand = new HumanOperator(profile, envelope, current, rng.NextUInt64());
            Vector3 home = HandModel.Wrist(profile, model.Home);
            float homePitch = HandModel.PitchOf(model.Home);
            if (script == DriveScript.ReachNoObstacle)
            {
                if (!hand.FreeUntil(end - t, clear))
                {
                    error = $"no free movement from {DrivePlan.Describe(current)} keeps clear of the objects";
                    return false;
                }

                // Back toward home as an operator brings it (DrivePlan then settles any small remainder); without a free way
                // there, DrivePlan's own return.
                hand.MoveTo(home, homePitch, 0f, clear, 0.004);
                hand.AppendLegs(legs, ref t, ref current);
                return true;
            }

            if (objects.Count == 0)
            {
                error = "the scene has no objects to approach";
                return false;
            }

            if (!hand.FreeUntil(PreRollMinSeconds + rng.NextDouble() * (PreRollMaxSeconds - PreRollMinSeconds), clear))
            {
                error = $"no free movement from {DrivePlan.Describe(current)} keeps clear of the objects";
                return false;
            }

            HandSnapshot rolled = hand.Save();

            // Objects in a random order; for each, its sweeps in turn: the first approach that works.
            var order = new int[objects.Count];
            for (int i = 0; i < order.Length; i++) order[i] = i;
            for (int i = order.Length - 1; i > 0; i--)
            {
                int j = (int)(rng.NextDouble() * (i + 1)) % (i + 1);
                (order[i], order[j]) = (order[j], order[i]);
            }

            int builds = 0;
            foreach (int oi in order)
            {
                // Roles (docs/adr/0037 P4): knocking over needs a body the arm can tip or shove (a fixed wall, or one too heavy, is
                // an obstacle, never the target); touching and backing off needs one that does not tip.
                if (script == DriveScript.CollideWithObstacle && !options.Knockable(objects[oi])) continue;
                if (script == DriveScript.CollideAndRecover && options.KnockMaxKg > 0 && !options.Steady(objects[oi])) continue;
                foreach (Sweep sweep in Sweeps(objects[oi], ref rng))
                {
                    if (!TryLine(profile, model, envelope, objects, oi, sweep, model.Home, out Line line)) continue;
                    if (++builds > MaxBuilds) break;
                    hand.Restore(rolled);
                    if (TryEvent(script, profile, model, objects, oi, line, hand, ref rng, options, home, homePitch))
                    {
                        hand.AppendLegs(legs, ref t, ref current);
                        return true;
                    }
                }

                if (builds > MaxBuilds) break;
            }

            error = $"no object could be approached for {Category(script)} from {DrivePlan.Describe(current)}";
            return false;
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

        /// <summary>What one realisation of the final approach did, by the collision model.</summary>
        private readonly struct Outcome
        {
            public readonly bool Solved, OthersClear;
            public readonly float Closest; // the smallest signed gap to the object over the measured ticks (below 0: that deep inside)
            public readonly double ContactSeconds, LastContact, EventEnd;

            public Outcome(bool solved, bool othersClear, float closest, double contactSeconds, double lastContact, double eventEnd)
            {
                Solved = solved;
                OthersClear = othersClear;
                Closest = closest;
                ContactSeconds = contactSeconds;
                LastContact = lastContact;
                EventEnd = eventEnd;
            }
        }

        private static bool TryEvent(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, IReadOnlyList<SafetyObject> objects, int oi,
            Line line, HumanOperator hand, ref SeededRng rng, SafetyOptions options, Vector3 home, float homePitch)
        {
            SafetyObject o = objects[oi];
            RobotArmProfile arm = profile; // a copy the local functions below can use
            var ladder = new GapLadder(profile, model, o);
            var gap = new float[line.Usable];
            for (int i = 0; i < line.Usable; i++) gap[i] = ladder.SignedGap(line.Poses[i]);

            // Where the final approach aims, on the sweep, and what it must reach there.
            HandEvent kind;
            int aim = -1;
            float target = 0f;
            if (script == DriveScript.StopBeforeCollision)
            {
                kind = HandEvent.Stop;
                target = StopGapMinMeters + 0.003f + (float)rng.NextDouble() * (StopGapMaxMeters - StopGapMinMeters - 0.006f);
                for (int i = 0; i < line.Contact && aim < 0; i++) if (gap[i] <= target) aim = i;
            }
            else if (script == DriveScript.CollideAndRecover)
            {
                // Into the object's box by the touch depth, not that far along the sweep: a sweep that grazes the box goes along
                // it, not into it (2026-11-15: 20 mm along reached 1-3 mm in, and Isaac's links are ~15 mm inside the model's
                // boxes, so it never touched). Capped at how deep this object's box can be entered at all.
                kind = HandEvent.Touch;
                target = TouchTarget(o, options.TouchDepthMeters);
                for (int i = line.Contact; i < line.Usable && aim < 0; i++) if (-gap[i] >= target) aim = i;
            }
            else
            {
                kind = HandEvent.Through;
                aim = line.Usable - 1;
                if (aim < line.Contact + (int)MathF.Ceiling(0.03f / LineStepMeters)) return false; // not far enough through
                // Through the object, not past its edge: some pose well inside its box.
                target = Math.Min(ThroughDepthMeters, MaxDepth(o));
                bool deep = false;
                for (int i = line.Contact; i <= aim && !deep; i++) deep = -gap[i] >= target - 0.0005f;
                if (!deep) return false;
            }

            if (aim < 0) return false;

            // The final approach starts a few centimetres back along the sweep, where the arm is still clear of the object.
            int back = (int)MathF.Round((FinalApproachMinMeters + (float)rng.NextDouble() * (FinalApproachMaxMeters - FinalApproachMinMeters)) / LineStepMeters);
            int stage = Math.Max(0, (kind == HandEvent.Through ? line.Contact : aim) - back);
            while (stage > 0 && gap[stage] < ApproachClearanceMeters) stage--;
            if ((aim - stage) * LineStepMeters < MinTravelMeters || gap[stage] < 0.008f) return false;

            // To the start of the final approach on a path clear of every object, beyond the model's margin (this one by what the
            // start leaves it). The arm's own boxes are the solver's to keep clear, at the model's margin.
            var grown = new SafetyObject[objects.Count];
            for (int i = 0; i < objects.Count; i++)
            {
                grown[i] = objects[i].Grown(i == oi ? MathF.Min(ApproachClearanceMeters + model.Margin, gap[stage] - 0.005f) : ApproachClearanceMeters + model.Margin);
            }

            ArmCollisionModel staging = WithObjects(model, grown, 0f, 0f);
            // With the gripper opened on the way (as the touch depth was measured) and a roll the approach then keeps.
            float roll = (float)((rng.NextDouble() * 2 - 1) * 15 * Math.PI / 180);
            if (!hand.MoveTo(line.Points[stage], line.Pitch, roll, staging, 0.004, overTheTop: true, pauseAfter: true, gripperGoal: ApproachGripper)) return false;
            HandSnapshot staged = hand.Save();

            var others = new List<SafetyObject>(objects.Count);
            for (int i = 0; i < objects.Count; i++) if (i != oi) others.Add(objects[i]);
            ArmCollisionModel othersClear = WithObjects(model, others, ApproachClearanceMeters);
            ulong suffix = rng.NextUInt64();
            Vector3[] points = line.Points;
            int usable = line.Usable;

            // The sweep from the start of the final approach to <lambda> of the way to its aim (past it above 1).
            List<Vector3> PathTo(float lambda)
            {
                double f = Math.Clamp(stage + lambda * (aim - stage), stage + 1, usable - 1);
                int i0 = (int)Math.Floor(f);
                var path = new List<Vector3>(i0 - stage + 2);
                for (int i = stage; i <= i0; i++) path.Add(points[i]);
                if (f > i0 && i0 + 1 < usable) path.Add(Vector3.Lerp(points[i0], points[i0 + 1], (float)(f - i0)));
                return path;
            }

            Outcome Realise(float lambda)
            {
                hand.Restore(staged, suffix);
                if (!hand.Event(kind, PathTo(lambda), line.Pitch, home, homePitch, out double finalStart, out double eventEnd))
                {
                    return new Outcome(false, false, 0f, 0, 0, eventEnd);
                }

                int k0 = (int)Math.Floor(finalStart / HandModel.TickSeconds);
                int kEvent = (int)Math.Ceiling(eventEnd / HandModel.TickSeconds);
                bool clearOfOthers = true;
                float closest = float.MaxValue;
                int touching = 0, last = -1;
                for (int k = k0; k < hand.Poses.Count; k++)
                {
                    // After a swipe the object is anywhere (knocked over): only the event is judged, and the other objects only
                    // until the swipe meets its own (they stand close on the bench; what the swipe sends flying is physics').
                    if (kind == HandEvent.Through && k > kEvent) break;
                    ArmPose p = hand.Poses[k];
                    if (clearOfOthers && (kind != HandEvent.Through || touching == 0) && !ArmCollision.IsValid(arm, othersClear, p)) clearOfOthers = false;
                    float g = ladder.SignedGap(p);
                    closest = MathF.Min(closest, g);
                    if (g < 0)
                    {
                        touching++;
                        last = k;
                    }
                }

                return new Outcome(true, clearOfOthers, closest, touching * HandModel.TickSeconds, last * HandModel.TickSeconds, eventEnd);
            }

            bool Good(in Outcome r)
            {
                if (!r.Solved || !r.OthersClear) return false;
                return kind switch
                {
                    HandEvent.Stop => r.Closest >= StopGapMinMeters && r.Closest <= StopGapMaxMeters,
                    HandEvent.Touch => MathF.Abs(-r.Closest - target) <= TouchToleranceMeters && r.ContactSeconds >= TouchMinSeconds
                                       && r.ContactSeconds <= TouchMaxSeconds && r.LastContact <= r.EventEnd + 0.6,
                    _ => -r.Closest >= target - 0.0005f,
                };
            }

            float chosen = float.NaN;
            if (kind == HandEvent.Through)
            {
                foreach (float lambda in new[] { 1.0f, 0.85f })
                {
                    if (Good(Realise(lambda)))
                    {
                        chosen = lambda;
                        break;
                    }
                }
            }
            else
            {
                // How far the final approach goes, as a scale about its start: bisected with every draw fixed (the same suffix
                // seed), so only that one number moves; closer or deeper as it grows.
                float lo = 0.6f, hi = 1.4f, best = float.NaN, bestMiss = float.MaxValue;
                for (int it = 0; it < 12; it++)
                {
                    float lambda = it == 0 ? 1f : 0.5f * (lo + hi);
                    Outcome r = Realise(lambda);
                    if (!r.Solved)
                    {
                        // Out of reach or blocked this far: come back toward the start.
                        if (lambda >= 1f) hi = lambda;
                        else lo = lambda;
                        continue;
                    }

                    float reached = kind == HandEvent.Stop ? r.Closest : -r.Closest;
                    float miss = MathF.Abs(reached - target);
                    if (Good(r) && miss < bestMiss)
                    {
                        best = lambda;
                        bestMiss = miss;
                    }

                    if (miss < 0.0007f) break;
                    bool tooFar = kind == HandEvent.Stop ? reached < target : reached > target;
                    if (tooFar) hi = lambda;
                    else lo = lambda;
                }

                chosen = best;
            }

            if (float.IsNaN(chosen)) return false;
            Outcome final = Realise(chosen);
            if (!Good(final)) return false;

            // DrivePlan returns home from here on a path the model calls free: an event that leaves no such path would refuse the
            // whole take (2026-11-15: "no free path back to home"), so another approach is tried instead.
            return DrivePlan.PathValid(profile, model, null, hand.Poses[hand.Poses.Count - 1], model.Home);
        }

        /// <summary>The deepest a pose can be measured inside <paramref name="o"/>: its smallest half-size, less 0.5 mm.</summary>
        public static float MaxDepth(in SafetyObject o)
        {
            Vector3 half = (o.Max - o.Min) / 2f;
            return MathF.Max(0f, MathF.Min(half.X, MathF.Min(half.Y, half.Z)) - 0.0005f);
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

    /// <summary>
    /// How close a pose's arm comes to one object's box, by the collision model, to <see cref="Step"/>: the boxes grown and
    /// shrunk in steps, built once, and searched. Clear of the box: the most it can grow and stay clear (up to
    /// <see cref="MaxGap"/>). Touching it: minus the most it can shrink on every side (each axis no further than 0.5 mm short of
    /// its middle) and still be touched, a hair below zero at a first touch.
    /// </summary>
    public sealed class GapLadder
    {
        public const float Step = 0.0005f, MaxGap = 0.04f;
        private readonly RobotArmProfile _profile;
        private readonly ArmCollisionModel[] _grown; // [i]: grown by (i + 1) * Step
        private readonly ArmCollisionModel[] _shrunk; // [i]: shrunk by i * Step ([0]: the box itself)

        public GapLadder(in RobotArmProfile profile, ArmCollisionModel model, in SafetyObject o)
        {
            _profile = profile;
            _grown = new ArmCollisionModel[(int)MathF.Round(MaxGap / Step)];
            for (int i = 0; i < _grown.Length; i++) _grown[i] = SafetyScript.WithObjects(model, new[] { o.Grown((i + 1) * Step) }, 0f, 0f);
            _shrunk = new ArmCollisionModel[(int)MathF.Floor(SafetyScript.MaxDepth(o) / Step + 1e-3f) + 1];
            for (int i = 0; i < _shrunk.Length; i++) _shrunk[i] = SafetyScript.WithObjects(model, new[] { o.Grown(-i * Step) }, 0f, 0f);
        }

        public float SignedGap(in ArmPose pose)
        {
            if (ArmCollision.IsValid(_profile, _shrunk[0], pose))
            {
                int lo = -1, hi = _grown.Length; // clear at lo (-1: the box itself), touching at hi
                while (hi - lo > 1)
                {
                    int mid = (lo + hi) / 2;
                    if (ArmCollision.IsValid(_profile, _grown[mid], pose)) lo = mid;
                    else hi = mid;
                }

                return (lo + 1) * Step;
            }

            int a = 0, b = _shrunk.Length; // touching at a, clear at b
            while (b - a > 1)
            {
                int mid = (a + b) / 2;
                if (!ArmCollision.IsValid(_profile, _shrunk[mid], pose)) a = mid;
                else b = mid;
            }

            return -a * Step - 1e-5f;
        }
    }
}
