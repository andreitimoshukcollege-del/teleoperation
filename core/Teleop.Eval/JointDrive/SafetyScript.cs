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
        /// collide_and_recover's push into an object's box, metres (joint-drive --touch-depth-mm), for an object without a shape
        /// (<see cref="SafetyObject.Shape"/>): its box is all the planner knows of it. The default is
        /// <see cref="SafetyScript.TouchDepthMeters"/>, measured on the twin.
        /// </summary>
        public float TouchDepthMeters { get; set; } = SafetyScript.TouchDepthMeters;

        /// <summary>
        /// collide_and_recover's push into an object's own shape, metres, for an object that has one (docs/adr/0037 amendment 5,
        /// third revision): <see cref="SafetyScript.ShapeTouchDepthMeters"/> by default, kept 3 mm short of the inner radius of
        /// the slab it meets.
        /// </summary>
        public float ShapeTouchDepthMeters { get; set; } = SafetyScript.ShapeTouchDepthMeters;

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

        /// <summary>
        /// The planner as it was until 2026-11-21 (docs/adr/0036 amendments 2 and 3): 3-8 s of free movement after a reaction
        /// time, one event, and the take ending during the move after it. Only to plan again what was recorded then
        /// (<c>TouchMissesTests</c>): every draw is the same as it was.
        /// </summary>
        public bool OneEventAtTheEnd { get; set; }

        /// <summary>
        /// How many events a take of stop_before_collision, collide_and_recover or collide_with_obstacle has: drawn uniformly
        /// from <see cref="MinEvents"/> to <see cref="MaxEvents"/> (docs/adr/0036 amendment 5; joint-drive --events). Fewer may
        /// be planned: a later event that finds no approach, or would make the take longer than <see cref="MaxTakeSeconds"/>,
        /// is left out, and after a knock the next is planned only once the objects have settled.
        /// </summary>
        public int MinEvents { get; set; } = 1;

        /// <summary>See <see cref="MinEvents"/>.</summary>
        public int MaxEvents { get; set; } = 3;

        /// <summary>Free movement before the first event, seconds (uniform between the two).</summary>
        public double LeadMinSeconds { get; set; } = 0.5;

        /// <summary>See <see cref="LeadMinSeconds"/>.</summary>
        public double LeadMaxSeconds { get; set; } = 4.0;

        /// <summary>Free movement between one event and the approach to the next, seconds (uniform between the two); after a knock, at least until the objects have settled and the next is planned.</summary>
        public double BetweenMinSeconds { get; set; } = 1.5;

        /// <summary>See <see cref="BetweenMinSeconds"/>.</summary>
        public double BetweenMaxSeconds { get; set; } = 5.0;

        /// <summary>Free movement after the last event before the take ends, seconds (uniform between the two), longer when the take would otherwise be shorter than <see cref="MinTakeSeconds"/>.</summary>
        public double TailMinSeconds { get; set; } = 1.0;

        /// <summary>See <see cref="TailMinSeconds"/>.</summary>
        public double TailMaxSeconds { get; set; } = 4.0;

        /// <summary>The shortest take of an event category, seconds: the tail grows to it.</summary>
        public double MinTakeSeconds { get; set; } = 10.0;

        /// <summary>The longest take of an event category, seconds: an event that would end the take later is left out.</summary>
        public double MaxTakeSeconds { get; set; } = 30.0;

        /// <summary>Throws when the event and timing options make no sense together.</summary>
        public void Validate()
        {
            if (MinEvents < 1 || MaxEvents < MinEvents || MaxEvents > 10) throw new ArgumentException($"events must be 1 <= min <= max <= 10, not {MinEvents}-{MaxEvents}");
            foreach ((double lo, double hi, string name) in new[] { (LeadMinSeconds, LeadMaxSeconds, "lead"), (BetweenMinSeconds, BetweenMaxSeconds, "between"),
                         (TailMinSeconds, TailMaxSeconds, "tail") })
            {
                if (!(lo >= 0 && hi >= lo)) throw new ArgumentException($"{name} seconds must be 0 <= min <= max, not {lo}-{hi}");
            }

            if (!(MinTakeSeconds >= 0 && MaxTakeSeconds > MinTakeSeconds)) throw new ArgumentException($"take seconds must be 0 <= min < max, not {MinTakeSeconds}-{MaxTakeSeconds}");
        }
    }

    /// <summary>
    /// One scene object as a safety script plans around it, in the arm base frame (metres): its box as it is turned
    /// (<see cref="Center"/>, <see cref="Rotation"/>, <see cref="HalfExtents"/>), and that box's axis-aligned bounds
    /// (<see cref="Min"/>, <see cref="Max"/>), which place the sweeps; and, for a scanned object, its own shape in that box
    /// (<see cref="Shape"/>), which only collide_and_recover's touch is planned against.
    /// </summary>
    public readonly struct SafetyObject
    {
        public readonly byte Id;
        public readonly Vector3 Min;
        public readonly Vector3 Max;
        public readonly Vector3 Center;
        public readonly Quaternion Rotation;
        public readonly Vector3 HalfExtents;

        /// <summary>Whether physics can move it (a scene's movable object): only such an object can be knocked over.</summary>
        public readonly bool Movable;

        /// <summary>Its mass at its size, kg (0 for a fixed object): what decides whether the arm can knock it over (docs/adr/0037 P4).</summary>
        public readonly double Mass;

        /// <summary>
        /// Whether a push tips it before it slides (docs/adr/0037 P4): taller than <see cref="TippyAspect"/> times its narrower
        /// side. Pushed at mid-height, an object tips first when its height exceeds its width over its friction (about 0.8).
        /// Lying on a round side it rolls instead, which a touch's check counts the same (amendment 6; <see cref="WorldView.IsTippy"/>).
        /// </summary>
        public readonly bool Tippy;

        /// <summary>Height over narrower side above which a pushed object tips before it slides.</summary>
        public const double TippyAspect = 1.25;

        /// <summary>
        /// The object as it really is, in its box's frame (docs/adr/0037 amendment 5, third revision): a scanned object is far
        /// smaller than its box when it is round or irregular. Null: the box is all there is to it. Only collide_and_recover's
        /// touch is planned against it; every clearance stays on the box.
        /// </summary>
        public readonly ObjectShape? Shape;

        /// <summary>An axis-aligned object.</summary>
        public SafetyObject(byte id, Vector3 min, Vector3 max, bool movable = true, double mass = 0, bool tippy = false, ObjectShape? shape = null)
            : this(id, (min + max) / 2f, Quaternion.Identity, (max - min) / 2f, movable, mass, tippy, shape)
        {
        }

        /// <summary>A turned object: its box centred at <paramref name="center"/>, turned by <paramref name="rotation"/>.</summary>
        public SafetyObject(byte id, Vector3 center, Quaternion rotation, Vector3 halfExtents, bool movable = true, double mass = 0, bool tippy = false,
            ObjectShape? shape = null)
        {
            Id = id;
            Center = center;
            Rotation = rotation;
            HalfExtents = halfExtents;
            var box = new CollisionBox("", center, rotation, halfExtents);
            Min = box.Min;
            Max = box.Max;
            Movable = movable;
            Mass = mass;
            Tippy = tippy;
            Shape = shape;
        }

        public CollisionBox Box(float grow) => new CollisionBox($"object {Id}", Center, Rotation, HalfExtents + new Vector3(grow));

        /// <summary>This object's box grown by <paramref name="grow"/> on every side (shrunk when negative, never past its middle); its shape as it is.</summary>
        public SafetyObject Grown(float grow)
        {
            Vector3 half = grow >= 0
                ? HalfExtents + new Vector3(grow)
                : HalfExtents - Vector3.Min(new Vector3(-grow), HalfExtents - new Vector3(0.0005f));
            return new SafetyObject(Id, Center, Rotation, half, Movable, Mass, Tippy, Shape);
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
    /// <item><b>collide_and_recover:</b> the same approach, slowly, to <see cref="ShapeTouchDepthMeters"/> into the object's
    /// own shape (<see cref="TouchDepthMeters"/> into the box of one without a shape); a moment in contact, and back out before
    /// it tips.</item>
    /// <item><b>collide_with_obstacle:</b> a fast swipe through the object, then lifted away.</item>
    /// </list>
    /// <para>Where an approach goes is found on a sideways sweep round the base past the object (the arm's reach is a thin
    /// shell, so an object is met from the side; <see cref="Sweeps"/>). How far the final approach goes is one number,
    /// searched with every random draw held fixed until the closest approach (or depth) the collision model measures over
    /// every tick is the category's. An object is its box, turned as it stands; a touch is planned against the object's own
    /// shape when it has one (docs/adr/0037 amendment 5, third revision: a round or irregular scan is far smaller than its box,
    /// and touches planned into the box stopped 2-10 cm short of it), with the arm as the twin's links are
    /// (<see cref="ShapeGap.TouchBodies"/>). Every pose
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
        // collide_and_recover goes this deep into the box of an object without a shape (capped by the box's size). Measured on
        // the twin 2026-11-15: pushes along the sweep (8 mm reached nothing, 15-25 mm touched, 35 mm tipped a post or grazed) put
        // it near 20 mm. That was put down to the model's arm boxes being about 15 mm larger than Isaac's links; they are Isaac's
        // links but for the fingers, and what fell short was the round objects' own shapes (docs/adr/0037 amendment 5, third
        // revision: ShapeTouchDepthMeters). Measured into the box since then: a grazing sweep goes along a box, not into it.
        public const float TouchDepthMeters = 0.020f;

        /// <summary>
        /// collide_and_recover goes this deep into an object's own shape (docs/adr/0037 amendment 5, third revision), measured
        /// with the arm as the twin's links are: Isaac counts a touch only within 1 mm (or with an impulse), its mesh colliders lie
        /// a little outside the render mesh the shape is surveyed from, and a slab's prism stands off a sloping mesh by up to
        /// the survey's excess (a few millimetres); 9 mm clears all three. Kept <see cref="ShapeInnerMarginMeters"/> short of
        /// the inner radius of the slab it meets (about 6 mm into the large marker).
        /// </summary>
        public const float ShapeTouchDepthMeters = 0.009f;

        /// <summary>A touch into a shape stays this far short of the inner radius of the slab it meets: a touch, not a push through.</summary>
        public const float ShapeInnerMarginMeters = 0.003f;

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

        /// <summary>The share of collide_and_recover takes that try a fixed object (a fixture) before the movable ones.</summary>
        public const double FixtureFirstShare = 0.5;


        // The wrist is the IK's target and sits well above the table (the JetRover's wrist barely goes below its shoulder), so the
        // gripper comes down beside the object, pointing down, its tip 1-6 cm below the object's top.
        private static readonly float[] Pitches = { -1.5f, -1.2f, -0.9f };
        private static readonly float[] HeightsAboveTop = { 0.10f, 0.12f, 0.08f, 0.14f, 0.06f, 0.16f };
        private static readonly float[] RadiusInside = { 0.0f, 0.02f, 0.04f, 0.06f, 0.08f, -0.02f, 0.10f };

        // A tall object (a fixture bigger than the robot, docs/adr/0037 amendment 5): its top is out of the arm's reach, so it
        // is also swept along its side, the hand near level, so the gripper reaching out past the wrist meets it. The wrist
        // goes only where the IK reaches it (mapped 2026-11-16 in the arm base frame: 16-24 cm out, 2 cm below to 10 cm above
        // the base frame's origin, within 90 degrees of straight ahead); the sweeps above the top alone never approached a fixture.
        public const float TallObjectMeters = 0.15f;
        private static readonly float[] SideWristZ = { 0.02f, 0.0f, 0.04f, 0.06f, -0.02f };
        private static readonly float[] SideRadii = { 0.20f, 0.18f, 0.22f, 0.16f, 0.24f };
        private static readonly float[] SidePitches = { 0.0f, -0.3f, -0.6f };
        /// <summary>A side sweep's wrist stays at least this far below a tall object's top, so the hand meets its side.</summary>
        public const float SideBelowTopMeters = 0.04f;

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
        /// <para>The planner as it was until 2026-11-21 (<see cref="SafetyOptions.OneEventAtTheEnd"/>): one event, at the end.
        /// Takes are planned by <see cref="SafetyTake"/> now, with several events at any point (docs/adr/0036 amendment 5).</para>
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
            if (script == DriveScript.ReachNoObstacle)
            {
                if (!hand.FreeUntil(end - t, clear))
                {
                    error = $"no free movement from {DrivePlan.Describe(current)} keeps clear of the objects";
                    return false;
                }

                // No way home (docs/adr/0036 amendment 3): the take ends partway through its last move, the arm still moving.
                hand.AppendLegs(legs, ref t, ref current);
                EndMoving(legs, ref t, ref current, ref rng);
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

            if (TryAnyEvent(script, profile, model, envelope, objects, hand, ref rng, options, null, null, false, out _))
            {
                double t0 = t; // the hand's timeline starts here in the plan
                hand.AppendLegs(legs, ref t, ref current);
                EndMoving(legs, ref t, ref current, ref rng, t0 + hand.OnwardStart, t0 + hand.OnwardEnd);
                return true;
            }

            error = $"no object could be approached for {Category(script)} from {DrivePlan.Describe(current)}";
            return false;
        }

        /// <summary>What <see cref="TryAnyEvent"/> planned: the object, the event's kind and times on the hand's timeline, and the way its final approach went.</summary>
        internal readonly struct PlannedEvent
        {
            public readonly int Object; // its index in the objects planned around
            public readonly HandEvent Kind;
            public readonly double ApproachStart, FinalStart, Contact, End, Clear;
            public readonly Vector3 Way; // the final approach's direction at its aim (a swipe's: where the object is sent)

            public PlannedEvent(int obj, HandEvent kind, double approachStart, double finalStart, double contact, double end, double clear, Vector3 way)
            {
                Object = obj;
                Kind = kind;
                ApproachStart = approachStart;
                FinalStart = finalStart;
                Contact = contact;
                End = end;
                Clear = clear;
                Way = way;
            }
        }

        /// <summary>
        /// One event of <paramref name="script"/> from where <paramref name="hand"/> is: objects in a random order, by their
        /// role's rank, and for each its sweeps in turn: the first approach that works (<see cref="TryEvent"/>) is planned onto
        /// the hand. False, with the hand as it was, when none works within <see cref="MaxBuilds"/> approaches.
        /// <para>With <paramref name="used"/> (several events a take, docs/adr/0036 amendment 5), objects already aimed at are
        /// tried after every other, and the share of collide_and_recover events that try a fixture first is drawn per event;
        /// with <paramref name="after"/>, the arm must be clear of it once the event is over, where free movement takes over.
        /// With neither, every draw is the one-event planner's (<see cref="SafetyOptions.OneEventAtTheEnd"/>).</para>
        /// </summary>
        internal static bool TryAnyEvent(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, HumanOperator hand, ref SeededRng rng, SafetyOptions options, ICollection<byte>? used,
            Func<int, ArmCollisionModel>? after, bool multi, out PlannedEvent planned)
        {
            planned = default;
            Vector3 home = HandModel.Wrist(profile, model.Home);
            float homePitch = HandModel.PitchOf(model.Home);
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
            // A touch is planned against the object's own shape when it has one (docs/adr/0037 amendment 5, third revision).
            var shapes = new ShapeGap?[objects.Count];
            RobotArmProfile arm = profile; // a copy the local function can use
            ShapeGap? ShapeOf(int i)
            {
                if (script != DriveScript.CollideAndRecover || objects[i].Shape == null) return null;
                return shapes[i] ??= new ShapeGap(arm, model, objects[i]);
            }

            // Roles (docs/adr/0037 P4): knocking over needs a body the arm can tip or shove (a fixed wall, or one too heavy, is an
            // obstacle, never the target). Touching and backing off prefers one that does not tip; when none of those can be
            // approached, the others are tried (2026-11-16: with fixtures beside the robot and many low objects, steady ones
            // alone left 10 of 16 layouts unplannable; a tipped object sets the take aside at its check instead). A fixture is
            // always steady and, swept along its side, nearly always in reach, so it would be nearly every touch (34 of 40
            // layouts): it is tried first in FixtureFirstShare of the takes (of the events, with several a take), and after
            // every movable object in the rest. With several events, an object already aimed at comes after all the others.
            bool roles = options.KnockMaxKg > 0;
            bool fixtureFirst = script == DriveScript.CollideAndRecover && roles && rng.NextDouble() < FixtureFirstShare;
            int Rank(in SafetyObject o)
            {
                int again = used != null && used.Contains(o.Id) ? 3 : 0;
                if (script != DriveScript.CollideAndRecover || !roles) return again;
                if (!o.Movable) return again + (fixtureFirst ? 0 : 2);
                return again + (options.Steady(o) ? 0 : 1);
            }

            int passes = used == null ? 3 : 6;
            for (int pass = 0; pass < passes; pass++)
            {
                foreach (int oi in order)
                {
                    if (script == DriveScript.CollideWithObstacle && !options.Knockable(objects[oi])) continue;
                    if (Rank(objects[oi]) != pass) continue;
                    foreach (Sweep sweep in Sweeps(objects[oi], ref rng))
                    {
                        ShapeGap? shape = ShapeOf(oi);
                        if (!TryLine(profile, model, envelope, objects, oi, sweep, model.Home, shape, out Line line)) continue;
                        if (++builds > MaxBuilds) break;
                        hand.Restore(rolled);
                        if (TryEvent(script, profile, model, objects, oi, line, hand, ref rng, options, home, homePitch, shape, after?.Invoke(oi), multi,
                                out EventTimes times))
                        {
                            planned = new PlannedEvent(oi, times.Kind, times.ApproachStart, times.FinalStart, times.Contact, times.End, hand.Free, times.Way);
                            return true;
                        }
                    }

                    if (builds > MaxBuilds) break;
                }

                if (builds > MaxBuilds) break;
            }

            hand.Restore(rolled);
            return false;
        }

        /// <summary>
        /// Ends the take partway (50-80%) through a final move, dropping what follows, so it stops while the arm still moves
        /// (docs/adr/0036 amendment 3): the next take carries on from wherever this one left the arm. The move is the plan's
        /// [<paramref name="windowFrom"/>, <paramref name="windowTo"/>] when given (an event's move onward: never the event
        /// itself), else its last run of moving ticks.
        /// </summary>
        internal static void EndMoving(List<DriveLeg> legs, ref double t, ref ArmPose current, ref SeededRng rng,
            double windowFrom = double.NaN, double windowTo = double.NaN)
        {
            int last = legs.Count - 1;
            while (last >= 0 && legs[last].From.MaxJointDelta(legs[last].To) < 1e-5f) last--;
            if (last < 0) return;
            int first = last;
            while (first > 0 && legs[first - 1].From.MaxJointDelta(legs[first - 1].To) >= 1e-5f && legs[first - 1].Hold < 1e-6) first--;
            double from = legs[first].Start, to = legs[last].Start + legs[last].Duration;
            if (!double.IsNaN(windowFrom) && windowTo > windowFrom)
            {
                from = windowFrom;
                to = windowTo;
            }
            double cut = from + (0.5 + 0.3 * rng.NextDouble()) * (to - from);
            CutAt(legs, ref t, ref current, cut, moving: false);
        }

        /// <summary>
        /// Ends the plan with the leg in force at <paramref name="cut"/> (plan seconds), dropping what follows; with
        /// <paramref name="moving"/>, at the first leg from there on that moves the arm, so the take ends with the arm moving.
        /// </summary>
        internal static void CutAt(List<DriveLeg> legs, ref double t, ref ArmPose current, double cut, bool moving = true)
        {
            int keep = legs.FindLastIndex(l => l.Start <= cut);
            if (keep < 0) return;
            while (moving && keep < legs.Count - 1 && (legs[keep].Duration <= 0 || legs[keep].From.MaxJointDelta(legs[keep].To) < 1e-5f)) keep++;
            legs.RemoveRange(keep + 1, legs.Count - keep - 1);
            DriveLeg k = legs[keep];
            t = k.Start + k.Duration + k.Hold;
            current = k.To;
        }

        /// <summary>The wrist's line towards one object, and its IK poses; the indices where the arm first touches the object's box, and its shape.</summary>
        private readonly struct Line
        {
            public readonly Vector3[] Points;
            public readonly ArmPose[] Poses;
            public readonly int Usable; // poses 0..Usable-1 are reachable, in the envelope and free of everything but the object
            public readonly int Contact; // the first pose touching the object's box (gap 0), or -1
            public readonly int ShapeContact; // the first pose touching its shape (the touch's arm, ShapeGap), or -1 (or no shape)
            public readonly float Pitch; // the wrist pitch it was solved at

            public Line(Vector3[] points, ArmPose[] poses, int usable, int contact, int shapeContact, float pitch)
            {
                Pitch = pitch;
                Points = points;
                Poses = poses;
                Usable = usable;
                Contact = contact;
                ShapeContact = shapeContact;
            }
        }

        private static bool TryLine(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, int oi, Sweep sweep, in ArmPose from, ShapeGap? shape, out Line line)
        {
            // The wrist swings sideways on an arc round the base, at a fixed radius, height and pitch, from SweepRadians to one
            // side of the object's bearing to past it: the arm's reach is a thin shell (measured 2026-11-14: the wrist 14-24 cm
            // out, its gripper pointing down), so an object is met from the side, as a swing knocks it over. The path starts at
            // its first reachable pose clear of every object, and the first touch of the object's box is wherever the collision
            // model finds it, whichever link it is. A touch planned against the object's shape needs the line to reach that too.
            line = default;
            SafetyObject o = objects[oi];
            Vector3 c = o.Center;
            float bearing = MathF.Atan2(c.Y, c.X);
            float halfWidth = MathF.Max(o.Max.X - o.Min.X, o.Max.Y - o.Min.Y) / 2f;
            float past = (halfWidth + ThroughPastMeters) / sweep.Radius;
            float lead = SweepRadians;
            if (o.Max.Z - o.Min.Z >= TallObjectMeters)
            {
                // A fixture beside the robot is wide and near: seen from the base it spans 20-30 degrees each side, and a level
                // hand reaches its corner from well off its middle (2026-11-16: every arc began touching it). So the arc begins
                // SweepRadians beyond its edge, and goes as far past its far edge.
                float span = AngularHalfSpan(o, bearing);
                lead = span + SweepRadians;
                past = span + ThroughPastMeters / sweep.Radius;
            }

            float a0 = bearing + sweep.Side * lead, a1 = bearing - sweep.Side * past;
            int n = (int)MathF.Ceiling(MathF.Abs(a1 - a0) * sweep.Radius / LineStepMeters) + 1;
            ArmCollisionModel touch = WithObjects(model, new[] { o }, 0f, 0f);
            ArmCollisionModel clearOfAll = WithObjects(model, objects, ApproachClearanceMeters);
            var points = new List<Vector3>(n);
            var poses = new List<ArmPose>(n);
            ArmPose at = from;
            int contact = -1, shapeContact = -1;
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
                if (shape != null && shapeContact < 0 && shape.Touches(pose)) shapeContact = poses.Count - 1;
            }

            if (contact < (int)(MinTravelMeters / LineStepMeters)) return false;
            if (shape != null && shapeContact < (int)(MinTravelMeters / LineStepMeters)) return false; // the line misses the object itself
            line = new Line(points.ToArray(), poses.ToArray(), poses.Count, contact, shapeContact, sweep.Pitch);
            return true;
        }

        /// <summary>How far, in radians either side of <paramref name="bearing"/>, an object's footprint reaches as seen from the base axis.</summary>
        private static float AngularHalfSpan(in SafetyObject o, float bearing)
        {
            float span = 0f;
            foreach (float x in new[] { o.Min.X, o.Max.X })
            foreach (float y in new[] { o.Min.Y, o.Max.Y })
            {
                float d = MathF.Atan2(y, x) - bearing;
                while (d > MathF.PI) d -= 2f * MathF.PI;
                while (d < -MathF.PI) d += 2f * MathF.PI;
                span = MathF.Max(span, MathF.Abs(d));
            }

            return span;
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

        /// <summary>The sweeps to try for one object, in a seeded order: radii a little inside its own, heights above its top, pitches, both sides; and a tall object also along its side.</summary>
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

            if (o.Max.Z - o.Min.Z >= TallObjectMeters)
            {
                foreach (float r in SideRadii)
                foreach (float z in SideWristZ)
                foreach (float pitch in SidePitches)
                foreach (int side in new[] { 1, -1 })
                {
                    if (z <= o.Max.Z - SideBelowTopMeters) all.Add(new Sweep(r, z, pitch, side));
                }
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
            public readonly double FinalStart, FirstContact, ClosestAt; // seconds on the hand's timeline (FirstContact NaN: none)
            public readonly bool AfterClear; // clear of the free movement's model once the event is over (or none was given)

            public Outcome(bool solved, bool othersClear, float closest, double contactSeconds, double lastContact, double eventEnd,
                double finalStart = double.NaN, double firstContact = double.NaN, double closestAt = double.NaN, bool afterClear = true)
            {
                Solved = solved;
                OthersClear = othersClear;
                Closest = closest;
                ContactSeconds = contactSeconds;
                LastContact = lastContact;
                EventEnd = eventEnd;
                FinalStart = finalStart;
                FirstContact = firstContact;
                ClosestAt = closestAt;
                AfterClear = afterClear;
            }
        }

        /// <summary>When an event planned by <see cref="TryEvent"/> happens on the hand's timeline, seconds, and the way its final approach went.</summary>
        private readonly struct EventTimes
        {
            public readonly HandEvent Kind;
            public readonly double ApproachStart; // the move to where the final approach starts (the hand leaves free movement)
            public readonly double FinalStart; // the final approach
            public readonly double Contact; // the first planned contact (touch, swipe), or the closest approach (stop)
            public readonly double End; // the stop's stillness or the touch's moment in contact ends; the swipe's lift away ends
            public readonly Vector3 Way;

            public EventTimes(HandEvent kind, double approachStart, double finalStart, double contact, double end, Vector3 way)
            {
                Kind = kind;
                ApproachStart = approachStart;
                FinalStart = finalStart;
                Contact = contact;
                End = end;
                Way = way;
            }
        }

        private static bool TryEvent(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, IReadOnlyList<SafetyObject> objects, int oi,
            Line line, HumanOperator hand, ref SeededRng rng, SafetyOptions options, Vector3 home, float homePitch, ShapeGap? shape,
            ArmCollisionModel? after, bool multi, out EventTimes times)
        {
            times = default;
            double approachStart = hand.Free;
            SafetyObject o = objects[oi];
            RobotArmProfile arm = profile; // a copy the local functions below can use
            var ladder = new GapLadder(profile, model, o);
            var gap = new float[line.Usable];
            for (int i = 0; i < line.Usable; i++) gap[i] = ladder.SignedGap(line.Poses[i]);
            // A touch of an object with a shape is measured against the shape, with the arm as the twin's links are (the line's
            // poses with the gripper open, as the approach sets it; the final approach's with the gripper each tick commands);
            // every clearance (where the final approach starts, the path there, the other objects) stays on the boxes.
            ShapeGap? touchShape = script == DriveScript.CollideAndRecover ? shape : null;

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
            else if (script == DriveScript.CollideAndRecover && touchShape != null)
            {
                // Into the object's own shape (docs/adr/0037 amendment 5, third revision), kept short of the inner radius of the
                // slab the line first meets: a thin object (the large marker) is touched, not pushed through.
                kind = HandEvent.Touch;
                target = ShapeTouchTarget(touchShape.Shape, options.ShapeTouchDepthMeters, touchShape.TouchedInnerRadius(line.Poses[line.ShapeContact]));
                for (int i = line.ShapeContact; i < line.Usable && aim < 0; i++) if (-touchShape.SignedGap(line.Poses[i]) >= target) aim = i;
            }
            else if (script == DriveScript.CollideAndRecover)
            {
                // Into the object's box by the touch depth, not that far along the sweep: a sweep that grazes the box goes along
                // it, not into it (2026-11-15: 20 mm along reached 1-3 mm in, and never touched). Capped at how deep this object's
                // box can be entered at all.
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
                float closest = float.MaxValue, closestInEvent = float.MaxValue;
                int touching = 0, last = -1, first = -1, nearest = -1;
                for (int k = k0; k < hand.Poses.Count; k++)
                {
                    // After a swipe the object is anywhere (knocked over): only the event is judged, and the other objects only
                    // until the swipe meets its own (they stand close on the bench; what the swipe sends flying is physics').
                    // With more to come in the take (multi), the other objects are judged again once the swipe and lift are done.
                    bool past = kind == HandEvent.Through && k > kEvent;
                    if (past && !multi) break;
                    ArmPose p = hand.Poses[k];
                    if (clearOfOthers && (kind != HandEvent.Through || touching == 0 || past) && !ArmCollision.IsValid(arm, othersClear, p)) clearOfOthers = false;
                    if (past) continue;
                    float g = touchShape != null ? touchShape.SignedGap(p, hand.Commands[k].Gripper) : ladder.SignedGap(p); // the pads where the gripper holds them
                    closest = MathF.Min(closest, g);
                    if (k * HandModel.TickSeconds <= eventEnd && g < closestInEvent)
                    {
                        closestInEvent = g; // where a stop is: its closest approach before it backs off
                        nearest = k;
                    }

                    if (g < 0)
                    {
                        touching++;
                        last = k;
                        if (first < 0) first = k;
                    }
                }

                // Where free movement takes over (the next part may start), the arm must be clear of its model.
                bool afterClear = true;
                if (after != null)
                {
                    for (int k = Math.Max(0, (int)Math.Floor(hand.Free / HandModel.TickSeconds)); k < hand.Poses.Count && afterClear; k++)
                    {
                        afterClear = ArmCollision.IsValid(arm, after, hand.Poses[k]);
                    }
                }

                return new Outcome(true, clearOfOthers, closest, touching * HandModel.TickSeconds, last * HandModel.TickSeconds, eventEnd, finalStart,
                    first < 0 ? double.NaN : first * HandModel.TickSeconds, nearest * HandModel.TickSeconds, afterClear);
            }

            bool Good(in Outcome r)
            {
                if (!r.Solved || !r.OthersClear || !r.AfterClear) return false;
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
            if (!DrivePlan.PathValid(profile, model, null, hand.Poses[hand.Poses.Count - 1], model.Home)) return false;
            Vector3 way = points[aim] - points[Math.Max(0, aim - 1)];
            times = new EventTimes(kind, approachStart, final.FinalStart, kind == HandEvent.Stop ? final.ClosestAt : final.FirstContact, final.EventEnd,
                way.Length() > 1e-6f ? Vector3.Normalize(way) : Vector3.UnitY);
            return true;
        }

        /// <summary>The deepest a pose can be measured inside <paramref name="o"/>: its smallest half-size, less 0.5 mm.</summary>
        public static float MaxDepth(in SafetyObject o)
        {
            Vector3 half = o.HalfExtents;
            return MathF.Max(0f, MathF.Min(half.X, MathF.Min(half.Y, half.Z)) - 0.0005f);
        }

        /// <summary>How deep collide_and_recover goes into <paramref name="o"/>'s box: <paramref name="depth"/>, but never within 3 mm of its middle (a touch, not a push through).</summary>
        public static float TouchTarget(in SafetyObject o, float depth) => MathF.Max(0.001f, MathF.Min(depth, MaxDepth(o) - 0.003f));

        /// <summary>
        /// How deep collide_and_recover goes into an object's own shape: <paramref name="depth"/>, but
        /// <see cref="ShapeInnerMarginMeters"/> short of <paramref name="slabInnerRadius"/> (the inner radius of the slab the touch
        /// meets) and of the deepest the shape can be measured, and at least 1 mm.
        /// </summary>
        public static float ShapeTouchTarget(ObjectShape shape, float depth, float slabInnerRadius) =>
            MathF.Max(0.001f, MathF.Min(depth, MathF.Min(slabInnerRadius, shape.MaxDepth) - ShapeInnerMarginMeters));

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
