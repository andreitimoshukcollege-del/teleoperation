using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>What the mixed script may be told (joint-drive's options; docs/adr/0038 decision 1).</summary>
    public sealed class MixedOptions
    {
        /// <summary>The share of moves aimed at an object (an aim or a pass), when one can be; the rest are layout-blind.</summary>
        public double ObjectShare { get; set; } = 0.6;

        /// <summary>The share of object-directed moves that pass across the object rather than reach at it.</summary>
        public double PassShare { get; set; } = 0.3;

        /// <summary>The share of aims that start close to their object (4-20 mm from it), so a contact comes early in the move.</summary>
        public double NearStartShare { get; set; } = 0.25;

        /// <summary>
        /// An aim's signed offset from the object's surface is drawn uniformly from here to <see cref="OffsetMaxMeters"/> (one range
        /// for every aim: below 0 into the object, above 0 short of it), no deeper than the collision model measures that object.
        /// </summary>
        public float OffsetMinMeters { get; set; } = -0.040f;

        /// <summary>See <see cref="OffsetMinMeters"/>.</summary>
        public float OffsetMaxMeters { get; set; } = 0.060f;

        /// <summary>No contact is planned before this, seconds into the take: every tick before it keeps <see cref="QuietMarginMeters"/> from every object's box.</summary>
        public double QuietStartSeconds { get; set; } = 1.5;

        /// <summary>The take's last this many seconds keep <see cref="QuietMarginMeters"/> from every object's box, and come after the last object-directed move.</summary>
        public double QuietEndSeconds { get; set; } = 1.0;

        /// <summary>How far from every object's box (beyond the model's margin) the quiet start and end keep, m.</summary>
        public float QuietMarginMeters { get; set; } = SafetyScript.ApproachClearanceMeters;

        /// <summary>The tail after the last object-directed move, seconds (uniform between the two), at least <see cref="QuietEndSeconds"/>.</summary>
        public double TailMinSeconds { get; set; } = 1.2;

        /// <summary>See <see cref="TailMinSeconds"/>.</summary>
        public double TailMaxSeconds { get; set; } = 2.5;

        /// <summary>Every move's slowness (a factor on the hand model's durations) is lognormal with this sd about 1, within <see cref="SlowMin"/>-<see cref="SlowMax"/>: one distribution for every kind of move.</summary>
        public double SlowSigma { get; set; } = 0.30;

        public double SlowMin { get; set; } = 0.65;
        public double SlowMax { get; set; } = 1.8;

        /// <summary>The share of moves followed by a pause of <see cref="PauseMinSeconds"/>-<see cref="PauseMaxSeconds"/>, whatever the move.</summary>
        public double PauseShare { get; set; } = 0.30;

        public double PauseMinSeconds { get; set; } = 0.15;
        public double PauseMaxSeconds { get; set; } = 0.70;

        /// <summary>The share of object-directed moves followed by a pull back the way they came (a reverse).</summary>
        public double ReverseAfterShare { get; set; } = 0.35;

        /// <summary>A layout-blind part is a move to a goal with this share, back-and-forth strokes with <see cref="BlindReverseShare"/>, else small adjustments.</summary>
        public double BlindTransportShare { get; set; } = 0.55;

        public double BlindReverseShare { get; set; } = 0.25;

        /// <summary>Where layout-blind goals are drawn.</summary>
        public BlindReach Reach { get; set; } = new BlindReach { NearShare = 0.8, RadialScale = 0.55f, VerticalScale = 0.55f };

        /// <summary>Where they are drawn in the quiet start and end, which keep clear of every object: never low.</summary>
        public BlindReach QuietReach { get; set; } = new BlindReach { LowShare = 0, NearShare = 0.7, HeightLow = 0.0f };

        /// <summary>The speed caps (docs/adr/0038 decision 1, item 29): <see cref="DefaultCaps"/> unless told otherwise; null for none.</summary>
        public MotionCaps? Caps { get; set; } = DefaultCaps();

        /// <summary>How the hand moves under the caps (<see cref="DefaultStyle"/> unless told otherwise); null: the hand model's own pace.</summary>
        public HandStyle? Style { get; set; } = DefaultStyle();

        /// <summary>
        /// The caps the mixed script drives with by default: each joint's speed and acceleration at the 99th percentile of 9 human
        /// headset sessions on the real JetRover (docs/adr/0038, workstream R: the command smoothed 25 ms). Yaw, shoulder, elbow,
        /// wrist pitch, roll.
        /// </summary>
        public static MotionCaps DefaultCaps()
        {
            var caps = new MotionCaps();
            float[] speed = { 3.8f, 2.6f, 3.6f, 2.8f, 4.0f }, acceleration = { 40f, 32f, 51f, 44f, 47f };
            speed.CopyTo(caps.JointSpeed, 0);
            acceleration.CopyTo(caps.JointAcceleration, 0);
            return caps;
        }

        /// <summary>
        /// The mixed script's hand, so its motion sits under the caps rather than at them (workstream R: the scripted operator turned
        /// the wrist pitch about twice as fast as people at p90, the elbow 1.4 times, the yaw 0.8 times): pitch and roll changes 0.5
        /// and 0.55 of the hand model's, followed over the whole of each submovement, every submovement 0.9 times as long, and
        /// (<see cref="Reach"/>) near goals more round the base than out from it or up. Chosen from this script's dry runs against
        /// R's targets (docs/adr/0038, "M, motion").
        /// </summary>
        public static HandStyle DefaultStyle() => new HandStyle { PitchScale = 0.5, RollScale = 0.55, PitchTime = 1.0, Slow = 0.9 };

        /// <summary>Throws when the options make no sense together.</summary>
        public void Validate()
        {
            foreach ((double v, string name) in new[] { (ObjectShare, "object share"), (PassShare, "pass share"), (NearStartShare, "near-start share"), (PauseShare, "pause share"),
                         (ReverseAfterShare, "reverse share"), (BlindTransportShare, "blind transport share"), (BlindReverseShare, "blind reverse share") })
            {
                if (!(v >= 0 && v <= 1)) throw new ArgumentException($"the {name} must be in [0, 1], not {v}");
            }

            if (BlindTransportShare + BlindReverseShare > 1) throw new ArgumentException("the blind transport and reverse shares add up to more than 1");
            if (!(OffsetMinMeters < OffsetMaxMeters) || OffsetMinMeters < -0.08f || OffsetMaxMeters > 0.12f)
                throw new ArgumentException($"offsets must be -80 mm <= min < max <= 120 mm, not {OffsetMinMeters * 1000:0}-{OffsetMaxMeters * 1000:0} mm");
            if (!(QuietStartSeconds >= 0) || !(QuietEndSeconds >= 0) || !(QuietMarginMeters >= 0)) throw new ArgumentException("the quiet start, end and margin must be at least 0");
            if (!(TailMinSeconds >= QuietEndSeconds && TailMaxSeconds >= TailMinSeconds)) throw new ArgumentException("the tail must be at least the quiet end, min <= max");
            if (!(SlowMin > 0 && SlowMax >= SlowMin && SlowSigma >= 0)) throw new ArgumentException("slowness must be 0 < min <= max");
            if (!(PauseMinSeconds >= 0 && PauseMaxSeconds >= PauseMinSeconds)) throw new ArgumentException("pause seconds must be 0 <= min <= max");
            Caps?.Validate();
        }
    }

    /// <summary>
    /// One planned move of a mixed take (docs/adr/0038 decision 1), its times in plan seconds, as its <c>MV</c> record says it. For
    /// an aim or a pass: the object, the offset it was aimed at, and the arm's closest approach over it as planned (both by the
    /// collision model with the twin's links against the object's shape, else its box; NaN for the other kinds).
    /// </summary>
    public readonly struct PlannedMove
    {
        public readonly int Index;
        public readonly ScriptMoveKind Kind;
        public readonly byte ObjectId;
        public readonly float OffsetMeters, GapMeters;
        public readonly double Start, Contact, Closest, End;

        /// <summary>An aim's or pass's direction of travel about the object's bearing from the base, degrees (0: straight out from the base; NaN for the other kinds). Reports only.</summary>
        public readonly float ApproachDegrees;

        public PlannedMove(int index, ScriptMoveKind kind, byte objectId, float offsetMeters, float gapMeters, double start, double contact, double closest, double end,
            float approachDegrees = float.NaN)
        {
            ApproachDegrees = approachDegrees;
            Index = index;
            Kind = kind;
            ObjectId = objectId;
            OffsetMeters = offsetMeters;
            GapMeters = gapMeters;
            Start = start;
            Contact = contact;
            Closest = closest;
            End = end;
        }

        /// <summary>The record joint-drive sends at plan time <paramref name="sentAt"/> (its times seconds after that), of <paramref name="count"/> moves.</summary>
        public ScriptMove Record(double sentAt, int count) => new ScriptMove((ushort)Math.Min(Index, ushort.MaxValue), (ushort)Math.Min(count, ushort.MaxValue), Kind, ObjectId,
            OffsetMeters * 1000f, GapMeters * 1000f, (float)(Start - sentAt), (float)(Contact - sentAt), (float)(Closest - sentAt), (float)(End - sentAt));

        public override string ToString() =>
            FormattableString.Invariant($"move {Index}: {ScriptMoveCodec.Name(Kind)}") +
            (ObjectId != 0 ? FormattableString.Invariant($" object {ObjectId}, offset {OffsetMeters * 1000:0.0} mm, gap {GapMeters * 1000:0.0} mm") : "") +
            FormattableString.Invariant($", {Start:0.00}-{End:0.00} s") +
            (double.IsNaN(Contact) ? "" : FormattableString.Invariant($", contact {Contact:0.00} s")) +
            (double.IsNaN(Closest) ? "" : FormattableString.Invariant($", closest {Closest:0.00} s"));
    }

    /// <summary>
    /// The mixed script (docs/adr/0038 decision 1): one take of operator-like moves from two sources mixed together, so the motion
    /// does not give the outcome away and physics alone decides whether the take touches anything.
    /// <list type="bullet">
    /// <item><b>Layout-blind moves:</b> goals anywhere in reach (a share of them low, the gripper pointing down), back-and-forth
    /// strokes and adjustments, checked against the robot's own model only, so they may meet an object.</item>
    /// <item><b>Object-directed moves:</b> an <b>aim</b> reaches at a point on an object's own shape (its box when it has none) from a
    /// drawn direction and height and stops at a signed offset from its surface drawn from one continuous range (into it, or short
    /// of it); a <b>pass</b> sweeps across an object at a drawn lateral offset, so contact can come mid-move. Some aims start close
    /// to their object, so contact can come early in the move.</item>
    /// <item>Every move draws its slowness from one distribution, and any move may be followed by a pause; an object-directed one
    /// also by a pull back.</item>
    /// </list>
    /// The arm moves from the first tick; no tick before <see cref="MixedOptions.QuietStartSeconds"/> or in the take's last
    /// <see cref="MixedOptions.QuietEndSeconds"/> comes within <see cref="MixedOptions.QuietMarginMeters"/> of any object's box, and the
    /// take ends partway through a move, with the arm still moving. Nothing is checked against a category: what happened is
    /// computed afterwards from the physics. Deterministic from its seed and the objects.
    /// </summary>
    internal sealed class MixedTake
    {
        /// <summary>The tool line's step, m.</summary>
        public const float LineStepMeters = 0.005f;

        /// <summary>
        /// The lowest an aim or a pass meets an object, m above its bottom: the pads' centre, which pointing down hangs 15 mm above their
        /// lowest corner, kept clear of the collision model's floor (10 mm above the table). Lower parts of an object are met only by
        /// moves that come down onto it, which the mixed script leaves to its layout-blind moves.
        /// </summary>
        public const float LowestContactMeters = 0.026f;

        /// <summary>The highest an aim or a pass meets an object, m above its bottom: about as high as the pads reach near the table (the shell <see cref="PitchFor"/> follows).</summary>
        public const float HighestContactMeters = 0.20f;

        /// <summary>The share of aims that come straight out from the base (within 25 degrees) rather than round it.</summary>
        public const double RadialShare = 0.25;

        /// <summary>An object every one of whose last this many aims and passes found no plan is tried no more in the take.</summary>
        public const int MaxObjectMisses = 12;

        /// <summary>The point between the finger pads in the gripper's frame (their centres' midpoint, <see cref="ShapeGap.LeftPadAtZero"/>): where an aim's line is drawn through.</summary>
        public static readonly Vector3 ToolInGripper = (ShapeGap.LeftPadAtZero.Center + ShapeGap.RightPadAtZero.Center) / 2f;

        /// <summary>The farthest the gap measures clear of an object (<see cref="GapLadder.MaxGap"/>): a larger offset is extrapolated along the approach.</summary>
        private const float MeasuredGap = GapLadder.MaxGap - 0.002f;

        private readonly RobotArmProfile _profile;
        private readonly ArmCollisionModel _model;
        private readonly DriveEnvelope _envelope;
        private readonly IReadOnlyList<SafetyObject> _objects;
        private readonly MixedOptions _options;
        private readonly HumanOperator _hand;
        private readonly double _start; // plan seconds at the operator's first tick
        private readonly ShapeGap?[] _shapes;
        private readonly GapLadder?[] _ladders;
        private readonly List<Draft> _drafts = new List<Draft>();
        private readonly HashSet<int> _moved = new HashSet<int>(); // objects a move planned to push into, as later targets
        private readonly ArmCollisionModel[]? _walls; // the fixed objects' boxes at each of WallOffsets (null: no fixed object)
        private readonly List<Vector2> _avoid = new List<Vector2>();
        private SeededRng _rng;
        private double _lastObjectEnd = double.NegativeInfinity;

        /// <summary>Why object-directed moves were not planned, for a report: too low to meet, no line, no end at the offset, no start, the move there, the approach, a pass's sweep.</summary>
        private readonly int[] _objectMisses = new int[7];

        private int _objectTries, _objectMoves;
        private readonly int[] _lineBreaks = new int[6]; // diagnostics: no seed, then a walk's break: IK, envelope, model, branch flip, over the base
        private readonly Dictionary<int, int> _missesInARow = new Dictionary<int, int>();

        /// <summary>The object-directed moves tried and planned, and why the rest were not.</summary>
        internal string ObjectReport()
        {
            string counts = FormattableString.Invariant(
                $"object moves {_objectMoves} of {_objectTries} tried (missed: too low {_objectMisses[0]}, no line {_objectMisses[1]}, no end {_objectMisses[2]}, no start {_objectMisses[3]}, move there {_objectMisses[4]}, approach {_objectMisses[5]}, sweep {_objectMisses[6]}; ") +
                FormattableString.Invariant($"lines: no seed {_lineBreaks[0]}, walks broken by IK {_lineBreaks[1]}, envelope {_lineBreaks[2]}, model {_lineBreaks[3]}, flip {_lineBreaks[4]}, base {_lineBreaks[5]})");
            var objects = new List<string>();
            foreach (KeyValuePair<int, int> kv in _missesInARow)
            {
                SafetyObject o = _objects[kv.Key];
                objects.Add(FormattableString.Invariant(
                    $"{o.Id} r {new Vector2(o.Center.X, o.Center.Y).Length():0.00} az {MathF.Atan2(o.Center.Y, o.Center.X) * 180 / MathF.PI:0} h {(o.Max.Z - o.Min.Z) * 100:0}cm{(o.Movable ? "" : " fixed")} misses {kv.Value}"));
            }

            return counts + "; objects: " + string.Join(", ", objects);
        }

        private bool Miss(int why)
        {
            _objectMisses[why]++;
            return false;
        }

        /// <summary>A move as planned, before the plan is final: its times on the hand's timeline, its object's index (-1: none).</summary>
        private struct Draft
        {
            public ScriptMoveKind Kind;
            public int Object;
            public float Offset, Approach;
            public double Start, End;
        }

        private MixedTake(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, IReadOnlyList<SafetyObject> objects, MixedOptions options,
            HumanOperator hand, double start, SeededRng rng)
        {
            _profile = profile;
            _model = model;
            _envelope = envelope;
            _objects = objects;
            _options = options;
            _hand = hand;
            _start = start;
            _rng = rng;
            _shapes = new ShapeGap?[objects.Count];
            _ladders = new GapLadder?[objects.Count];
            var fixedObjects = new List<SafetyObject>();
            foreach (SafetyObject o in objects) if (!o.Movable) fixedObjects.Add(o);
            if (fixedObjects.Count > 0)
            {
                _walls = new ArmCollisionModel[WallOffsets.Length];
                for (int i = 0; i < WallOffsets.Length; i++)
                {
                    var grown = new SafetyObject[fixedObjects.Count];
                    for (int k = 0; k < grown.Length; k++) grown[k] = fixedObjects[k].Grown(WallOffsets[i]);
                    _walls[i] = SafetyScript.WithObjects(model, grown, 0f, 0f);
                }
            }
        }

        /// <summary>
        /// How close to a fixed object's box a layout-blind move may come (one drawn for each move), m: from 5 mm into it to 15 mm short
        /// of it. A person stops at a wall, a little short of it or touching it; a movable object stays fair game for physics.
        /// </summary>
        public static readonly float[] WallOffsets = { -0.005f, 0f, 0.005f, 0.010f, 0.015f };

        private ArmCollisionModel? DrawWalls() => _walls == null ? null : _walls[(int)(_rng.NextDouble() * _walls.Length) % _walls.Length];

        /// <summary>
        /// Plans a mixed take from <paramref name="current"/> at plan time <paramref name="t"/> (after <paramref name="prefix"/>, the
        /// plan's legs so far) for about <paramref name="seconds"/>. False, with the reason, when the arm is not clear of the objects
        /// or no movement keeps the quiet start and end.
        /// </summary>
        internal static bool TryPlan(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, IReadOnlyList<SafetyObject> objects, float speed,
            in ArmPose current, double t, double seconds, ref SeededRng rng, List<DriveLeg> prefix, MixedOptions options, out DrivePlan? plan, out string? error)
        {
            plan = null;
            error = null;
            try
            {
                options.Validate();
            }
            catch (ArgumentException e)
            {
                error = e.Message;
                return false;
            }

            if (!ArmCollision.IsValid(profile, SafetyScript.WithObjects(model, objects, options.QuietMarginMeters), current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {options.QuietMarginMeters * 100:0} cm of an object";
                return false;
            }

            var hand = new HumanOperator(profile, envelope, current, rng.NextUInt64());
            hand.EngageOnTheMove();
            hand.Caps = options.Caps;
            hand.Style = options.Style;
            hand.Guard = SafetyScript.WithObjects(model, objects, options.QuietMarginMeters);
            hand.GuardBefore = options.QuietStartSeconds;
            var take = new MixedTake(profile, model, envelope, objects, options, hand, t, new SeededRng(rng.NextUInt64()));
            return take.Plan(prefix, current, speed, seconds, out plan, out error);
        }

        /// <summary>What refused the hand's parts so far, for a report.</summary>
        internal string Refusals()
        {
            int[] r = _hand.Refusals;
            return $"ticks refused: IK {r[0]}, envelope or model {r[1]}, jump {r[2]}, quiet margin {r[3]}, caps {r[4]}, walls {r[5]}; {ObjectReport()}";
        }

        /// <summary>
        /// Where a mixed take may start from <paramref name="start"/> (docs/adr/0038; the twin check of 2026-11-23: the model's home was
        /// within the old scripts' 5 cm of an obstacle on 13% of the mixed scene's layouts): there, when it is in the envelope and the
        /// quiet margin from every object; else home, or one of a few poses turned and raised from home (the first that is clear by
        /// the margin and 1 cm more), reached by a slow setup move (straight, or a joint at a time) clear of the objects by 1 cm.
        /// False, and why, when none is.
        /// </summary>
        internal static bool TryStartPose(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, IReadOnlyList<SafetyObject> objects,
            MixedOptions options, in ArmPose start, out ArmPose target, out List<(ArmPose To, bool Step)> via, out string? error)
        {
            target = start;
            via = new List<(ArmPose, bool)>();
            error = null;
            ArmCollisionModel quiet = SafetyScript.WithObjects(model, objects, options.QuietMarginMeters);
            if (envelope.Contains(start) && ArmCollision.IsValid(profile, quiet, start)) return true;
            ArmCollisionModel roomy = SafetyScript.WithObjects(model, objects, options.QuietMarginMeters + 0.01f);
            ArmCollisionModel path = SafetyScript.WithObjects(model, objects, 0.01f);
            ArmPose home = model.Home;
            foreach (float lift in new[] { 0f, 0.2f })
            foreach (float yaw in new[] { 0f, 0.35f, -0.35f, 0.7f, -0.7f, 1.05f, -1.05f, 1.4f, -1.4f })
            {
                ArmPose p = DrivePlan.WithJoint(DrivePlan.WithJoint(home, 0, home.BaseYaw + yaw), 1, home.Proximal + lift);
                if (!envelope.Contains(p) || !ArmCollision.IsValid(profile, roomy, p)) continue;
                if (start.MaxJointDelta(p) <= 0.01f || DrivePlan.TryApproach(profile, path, start, p, out via, out _))
                {
                    target = p;
                    return true;
                }
            }

            error = $"no start pose near home is {options.QuietMarginMeters * 100 + 1:0} cm clear of the objects with a clear way there from {DrivePlan.Describe(start)}";
            return false;
        }

        private double Slowness() => Math.Clamp(Math.Exp(_options.SlowSigma * Draw.Normal(ref _rng)), _options.SlowMin, _options.SlowMax);

        private bool Plan(List<DriveLeg> prefix, in ArmPose current, float speed, double seconds, out DrivePlan? plan, out string? error)
        {
            plan = null;
            error = null;
            double tail = Draw.Uniform(ref _rng, _options.TailMinSeconds, _options.TailMaxSeconds);
            double mainEnd = Math.Max(_options.QuietStartSeconds + 1.0, seconds - tail);
            bool first = true;
            while (_hand.Free < mainEnd)
            {
                bool done = false;
                if (!first && _hand.Free >= _options.QuietStartSeconds - 0.9 && Draw.Chance(ref _rng, _options.ObjectShare)) done = TryObjectMove();
                if (!done) done = TryBlindMove(first);
                if (!done) done = Recentre(); // cornered (every part left the envelope or met the arm itself): back toward the middle
                if (!done)
                {
                    error = FormattableString.Invariant($"no movement from {_hand.Free:0.0} s on of the take ({Refusals()})");
                    return false;
                }

                first = false;
            }

            // The tail: first up and away from wherever the main part left the arm, to a point clear of every object, then free
            // movement clear of them all (its goals away from those pushed into) from where that move ends, and the cut partway
            // through a move.
            if (!Exit(out double tailStart))
            {
                error = FormattableString.Invariant($"no move from {_hand.Free:0.0} s on of the take ends clear of every object ({Refusals()})");
                return false;
            }

            _hand.GuardFrom = tailStart;
            _hand.AvoidGoals = _avoid.Count > 0 ? _avoid : null;
            double cut = double.NaN;
            for (int part = 0; part < 12 && double.IsNaN(cut); part++)
            {
                if (!TryBlindMove(false, tailOnly: true, out double s, out double e))
                {
                    error = FormattableString.Invariant($"no free movement keeps the take's quiet end ({_options.QuietEndSeconds:0.0} s clear of the objects, from {tailStart:0.0} s; {Refusals()})");
                    return false;
                }

                if (_hand.Free < Math.Max(tailStart + tail, seconds)) continue;
                double at = Math.Max(s + (0.5 + 0.3 * _rng.NextDouble()) * (e - s), tailStart + _options.QuietEndSeconds);
                if (at <= e) cut = at;
            }

            if (double.IsNaN(cut))
            {
                error = "the take's tail found no move to end in";
                return false;
            }

            _hand.AvoidGoals = null;
            var legs = new List<DriveLeg>(prefix);
            double t = _start;
            ArmPose at0 = current;
            _hand.AppendLegs(legs, ref t, ref at0);
            SafetyScript.CutAt(legs, ref t, ref at0, _start + cut);
            plan = DrivePlan.ForMixed(legs, speed, Moves(_start + cut), ObjectReport() + "; " + Refusals());
            return true;
        }

        /// <summary>
        /// When no layout-blind part works from where the hand is (cornered at the edge of the envelope, or folded against the arm's own
        /// body): a move back toward the middle of reach (the wrist 12-17 cm out, 8-16 cm up, within 40 degrees of where it is),
        /// keeping the quiet start's margin when it applies and the walls. False when that does not work either.
        /// </summary>
        private bool Recentre()
        {
            const double deg = Math.PI / 180.0;
            Vector3 at = _hand.TargetEnd;
            float here = MathF.Atan2(at.Y, at.X);
            for (int attempt = 0; attempt < 8; attempt++)
            {
                float r = (float)Draw.Uniform(ref _rng, 0.12, 0.17), z = (float)Draw.Uniform(ref _rng, 0.08, 0.16);
                float az = Math.Clamp(here + (float)(Draw.Uniform(ref _rng, -40, 40) * deg), (float)(-80 * deg), (float)(80 * deg));
                var goal = new Vector3(r * MathF.Cos(az), r * MathF.Sin(az), z);
                if (!_hand.MixedTo(goal, (float)(Draw.Uniform(ref _rng, -45, 0) * deg), null, Slowness(), 0.010, _model, out double s, out double e, walls: DrawWalls())) continue;
                Add(ScriptMoveKind.Free, -1, float.NaN, s, e);
                return true;
            }

            return false;
        }

        /// <summary>
        /// The tail's first move: up (5-10 cm, to at least 10 cm above the base frame's origin) and toward the middle of reach from
        /// where everything planned so far ends, until the arm is clear of every object by the quiet margin; when its motion ends,
        /// seconds on the hand's timeline. Tried from a few goals; false when none ends clear.
        /// </summary>
        private bool Exit(out double end)
        {
            end = double.NaN;
            const double deg = Math.PI / 180.0;
            Vector3 at = _hand.TargetEnd;
            var here = new Vector2(at.X, at.Y);
            float reach = _profile.ProximalLinkLength + _profile.DistalLinkLength - 0.03f;
            for (int attempt = 0; attempt < 24; attempt++)
            {
                // A goal whose own pose is clear of every object: first above and in from where the hand is, then anywhere in
                // front within the reach shell (the wrist at most 3 cm short of the arm's length from the shoulder).
                float r, az;
                if (attempt < 8)
                {
                    r = Math.Clamp(here.Length() - (float)Draw.Uniform(ref _rng, 0.0, 0.06), 0.10f, 0.17f);
                    az = MathF.Atan2(here.Y, here.X) + (float)(Draw.Uniform(ref _rng, -20, 20) * deg);
                }
                else
                {
                    r = (float)Draw.Uniform(ref _rng, 0.10, 0.17);
                    az = (float)(Draw.Uniform(ref _rng, -70, 70) * deg);
                }

                float top = _profile.BaseHeight + MathF.Sqrt(MathF.Max(0f, reach * reach - r * r));
                float z = MathF.Min(MathF.Max(at.Z + (float)Draw.Uniform(ref _rng, 0.05, 0.10), (float)Draw.Uniform(ref _rng, 0.06, 0.16)), top);
                var goal = new Vector3(r * MathF.Cos(az), r * MathF.Sin(az), z);
                float pitch = (float)(Draw.Uniform(ref _rng, -40, 0) * deg);
                ArmPose from = _hand.Poses.Count > 0 ? _hand.Poses[_hand.Poses.Count - 1] : _model.Home;
                if (!TrySolve(from, goal, pitch, out ArmPose there) || !_envelope.Contains(there) || !ArmCollision.IsValid(_profile, _hand.Guard!, there)) continue;
                HandSnapshot before = _hand.Save();
                int drafts = _drafts.Count;
                if (!_hand.MixedTo(goal, pitch, null, Slowness(), 0.010, _model, out double s, out double e, walls: DrawWalls())) continue;
                int k = Math.Min(_hand.Poses.Count - 1, (int)Math.Ceiling(e / HandModel.TickSeconds));
                if (!ArmCollision.IsValid(_profile, _hand.Guard!, _hand.Poses[k]))
                {
                    Undo(before, drafts);
                    continue;
                }

                Add(ScriptMoveKind.Free, -1, float.NaN, s, e);
                end = e;
                return true;
            }

            return false;
        }

        /// <summary>The moves as planned, measured on the final plan, in the order they begin; those after <paramref name="cut"/> (plan seconds) left out.</summary>
        private PlannedMove[] Moves(double cut)
        {
            _drafts.Sort((a, b) => a.Start.CompareTo(b.Start));
            var moves = new List<PlannedMove>(_drafts.Count);
            foreach (Draft d in _drafts)
            {
                if (_start + d.Start >= cut) continue;
                double contact = double.NaN, closest = double.NaN;
                float gap = float.NaN;
                byte id = 0;
                if (d.Object >= 0)
                {
                    id = _objects[d.Object].Id;
                    int k0 = Math.Max(0, (int)Math.Floor(d.Start / HandModel.TickSeconds));
                    int k1 = Math.Min(_hand.Poses.Count - 1, (int)Math.Ceiling(d.End / HandModel.TickSeconds));
                    gap = float.MaxValue;
                    for (int k = k0; k <= k1; k++)
                    {
                        float g = Gap(d.Object, _hand.Poses[k], _hand.Commands[k].Gripper);
                        if (g < gap)
                        {
                            gap = g;
                            closest = _start + k * HandModel.TickSeconds;
                        }

                        if (g < 0 && double.IsNaN(contact)) contact = _start + k * HandModel.TickSeconds;
                    }

                    if (gap == float.MaxValue) gap = float.NaN;
                }

                moves.Add(new PlannedMove(moves.Count + 1, d.Kind, id, d.Object >= 0 ? d.Offset : float.NaN, gap, _start + d.Start, contact, closest, _start + d.End,
                    d.Object >= 0 ? d.Approach : float.NaN));
            }

            return moves.ToArray();
        }

        private void Add(ScriptMoveKind kind, int obj, float offset, double start, double end, float approach = float.NaN) =>
            _drafts.Add(new Draft { Kind = kind, Object = obj, Offset = offset, Start = start, End = end, Approach = approach });

        /// <summary>The arm's signed gap to object <paramref name="oi"/> at <paramref name="pose"/>: against its shape with the twin's links (the gripper as given), else its box with the model's.</summary>
        private float Gap(int oi, in ArmPose pose, float gripper)
        {
            SafetyObject o = _objects[oi];
            if (o.Shape != null) return (_shapes[oi] ??= new ShapeGap(_profile, _model, o)).SignedGap(pose, gripper);
            return (_ladders[oi] ??= new GapLadder(_profile, _model, o)).SignedGap(pose);
        }

        /// <summary>The deepest <see cref="Gap"/> measures object <paramref name="oi"/>, m.</summary>
        private float DeepestGap(int oi)
        {
            SafetyObject o = _objects[oi];
            return o.Shape != null ? o.Shape.ShrinkSteps * ObjectShape.Step : SafetyScript.MaxDepth(o);
        }

        /// <summary>A layout-blind part (a move, strokes or adjustments; now and then followed by a pause), or, in the tail, one clear of every object.</summary>
        private bool TryBlindMove(bool first, bool tailOnly = false) => TryBlindMove(first, tailOnly, out _, out _);

        private bool TryBlindMove(bool first, bool tailOnly, out double start, out double end)
        {
            bool quiet = tailOnly || _hand.Free < _options.QuietStartSeconds;
            if (!_hand.MixedFree(_model, quiet ? _options.QuietReach : _options.Reach, Slowness(), _options.BlindTransportShare, _options.BlindReverseShare,
                    out ScriptMoveKind kind, out start, out end, DrawWalls()))
            {
                return false;
            }

            Add(kind, -1, float.NaN, start, end);
            if (!first && !tailOnly) MaybePause();
            return true;
        }

        private void MaybePause()
        {
            if (!Draw.Chance(ref _rng, _options.PauseShare)) return;
            _hand.MixedPause(Draw.Uniform(ref _rng, _options.PauseMinSeconds, _options.PauseMaxSeconds), out double s, out double e);
            Add(ScriptMoveKind.Pause, -1, float.NaN, s, e);
        }

        /// <summary>An aim or a pass at an object of the layout (a few tries over objects and geometries), with the move to where it starts; false when none plans.</summary>
        private bool TryObjectMove()
        {
            var targets = new List<int>(_objects.Count);
            for (int i = 0; i < _objects.Count; i++)
            {
                if (_objects[i].Scenery || _moved.Contains(i)) continue;
                if (_objects[i].Max.Z - _objects[i].Min.Z < LowestContactMeters + 0.016f) continue; // flat: met only from above
                if (_missesInARow.TryGetValue(i, out int misses) && misses >= MaxObjectMisses) continue; // out of reach, it seems
                Vector3 c = _objects[i].Center;
                float r = new Vector2(c.X, c.Y).Length();
                if (r - Footprint(_objects[i]) < 0.40f) targets.Add(i);
            }

            if (targets.Count == 0) return false;
            for (int attempt = 0; attempt < 14; attempt++)
            {
                int oi = targets[(int)(_rng.NextDouble() * targets.Count) % targets.Count];
                ScriptMoveKind kind = Draw.Chance(ref _rng, _options.PassShare) ? ScriptMoveKind.Pass : ScriptMoveKind.Aim;
                if (TryObjectMove(oi, kind))
                {
                    _missesInARow[oi] = 0;
                    return true;
                }

                _missesInARow[oi] = _missesInARow.TryGetValue(oi, out int m) ? m + 1 : 1;
                if (m + 1 >= MaxObjectMisses)
                {
                    targets.Remove(oi);
                    if (targets.Count == 0) return false;
                }
            }

            return false;
        }

        private static float Footprint(in SafetyObject o) => MathF.Max(o.Max.X - o.Min.X, o.Max.Y - o.Min.Y) / 2f;

        /// <summary>One aim or pass at object <paramref name="oi"/>: its line drawn and measured, then the hand's moves along it; the hand as it was when it does not plan.</summary>
        private bool TryObjectMove(int oi, ScriptMoveKind kind)
        {
            _objectTries++;
            SafetyObject o = _objects[oi];
            const double deg = Math.PI / 180.0;
            float bearing = MathF.Atan2(o.Center.Y, o.Center.X);
            var radialDir = new Vector3(MathF.Cos(bearing), MathF.Sin(bearing), 0f);
            var tangentDir = new Vector3(-MathF.Sin(bearing), MathF.Cos(bearing), 0f);
            float rObj = new Vector2(o.Center.X, o.Center.Y).Length();
            float halfRadial = Support(o, radialDir), halfTangential = Support(o, tangentDir);

            // The height the pads meet it at: from just above the model's floor to just below its top (a touch into its side, never
            // down through its top, where a shape may cover a hollow).
            float zLo = o.Min.Z + LowestContactMeters, zHi = o.Max.Z - (o.Shape != null ? ObjectShape.TopMarginMeters + 0.003f : 0.004f);
            zHi = MathF.Min(zHi, o.Min.Z + HighestContactMeters); // a tall fixture's upper part is out of the pads' reach
            if (zHi < zLo + 0.003f) return Miss(0);
            float h = (float)Draw.Uniform(ref _rng, zLo, zHi);

            // The pads' path. Mostly an arc round the base at one radius, from either side: the arm's reach is a thin shell round
            // the base, and an arc keeps it there as a hand sweeping sideways does. Some aims come straight out from the base
            // instead, a short way (the shell is about 6 cm deep at one pitch). Its height changes along it by a drawn slope.
            var tools = new List<Vector3>();
            float side = Draw.Chance(ref _rng, 0.5) ? 1f : -1f, slope = MathF.Tan((float)(Draw.Uniform(ref _rng, -12, 8) * deg)), approach;
            if (kind == ScriptMoveKind.Pass || !Draw.Chance(ref _rng, RadialShare))
            {
                float rLine;
                if (kind == ScriptMoveKind.Aim) rLine = rObj + (float)Draw.Uniform(ref _rng, -0.6, 0.6) * halfRadial;
                else
                {
                    float out_ = Draw.Chance(ref _rng, 0.5) ? 1f : -1f;
                    // Through a movable object, or beside it; a fixture only grazed or passed (it does not give way).
                    rLine = rObj + out_ * (float)(o.Movable ? Draw.Uniform(ref _rng, 0, halfRadial + 0.07) : Draw.Uniform(ref _rng, halfRadial - 0.010, halfRadial + 0.07));
                }

                if (rLine < 0.10f) return Miss(1);
                float s0 = -(halfTangential + 0.24f), s1 = kind == ScriptMoveKind.Aim ? 0f : halfTangential + 0.12f;
                for (float a = s0; a <= s1 + 1e-6f; a += LineStepMeters)
                {
                    float phi = bearing + side * a / rLine;
                    tools.Add(new Vector3(rLine * MathF.Cos(phi), rLine * MathF.Sin(phi), h + slope * a));
                }

                approach = side * 90f;
            }
            else
            {
                float delta = (float)(Draw.Uniform(ref _rng, -25, 25) * deg), elevation = MathF.Atan(slope);
                var u0 = new Vector3(MathF.Cos(bearing + delta) * MathF.Cos(elevation), MathF.Sin(bearing + delta) * MathF.Cos(elevation), MathF.Sin(elevation));
                var across = new Vector3(-MathF.Sin(bearing + delta), MathF.Cos(bearing + delta), 0f);
                Vector3 middle = new Vector3(o.Center.X, o.Center.Y, h) + (float)Draw.Uniform(ref _rng, -0.6, 0.6) * Support(o, across) * across;
                for (float a = -(Support(o, u0) + 0.14f); a <= 1e-6f; a += LineStepMeters) tools.Add(middle + a * u0);
                approach = delta * 180f / MathF.PI;
            }

            // The path is first solved where it meets the object's box (or passes nearest it), the wrist pitched for how far out
            // that is: the shell's pitch changes with the radius. Other pitches are tried there too.
            int meet = Meets(o, tools);
            float radius = new Vector2(tools[meet].X, tools[meet].Y).Length();
            float pitch = PitchFor(radius) + (float)(Draw.Uniform(ref _rng, -8, 8) * deg);
            float gripper = _hand.GripperEnd;
            List<Vector3>? wrists = null;
            List<ArmPose>? poses = null;
            List<float>? gaps = null;
            int first = 0;
            if (Line(oi, tools, meet, ref pitch, gripper, out List<Vector3> w, out List<ArmPose> q, out List<float> g, out int f0)) (wrists, poses, gaps, first) = (w, q, g, f0);

            if (wrists == null || poses == null || gaps == null) return Miss(1);

            int stage, end;
            float offset, f = 1f;
            bool near = false;
            if (kind == ScriptMoveKind.Aim)
            {
                // The end: where the gap first comes to the drawn offset (beyond what the model measures clear, extrapolated back
                // along the approach); a part of a step for the rest.
                float deepest = DeepestGap(oi);
                offset = (float)Draw.Uniform(ref _rng, Math.Max(_options.OffsetMinMeters, -(deepest - 0.0005f)), _options.OffsetMaxMeters);
                if (!AimIndex(gaps, offset, out end, out f)) return Miss(2);
                near = Draw.Chance(ref _rng, _options.NearStartShare) && end >= 3;
                if (near)
                {
                    float close = (float)Draw.Uniform(ref _rng, 0.004, 0.020);
                    stage = end - 2;
                    while (stage > 0 && gaps[stage] < close) stage--;
                }
                else
                {
                    stage = Math.Max(0, end - (int)MathF.Round((float)Draw.Uniform(ref _rng, 0.04, 0.12) / LineStepMeters));
                    while (stage > 0 && gaps[stage] < 0.006f) stage--;
                }

                if (gaps[stage] < 0.003f || end - stage < 2) return Miss(3);
            }
            else
            {
                // The closest point of the sweep; it starts well before and goes on past the object.
                int closest = 0;
                for (int i = 1; i < gaps.Count; i++) if (gaps[i] < gaps[closest]) closest = i;
                offset = gaps[closest];
                stage = closest - (int)MathF.Round((float)Draw.Uniform(ref _rng, 0.05, 0.12) / LineStepMeters);
                if (stage < 0) return Miss(6);
                while (stage > 0 && gaps[stage] < 0.006f) stage--;
                int exit = closest;
                while (exit < gaps.Count && gaps[exit] < Math.Max(offset, 0f) + 0.010f) exit++;
                if (exit >= gaps.Count || gaps[stage] < 0.003f) return Miss(6); // the sweep does not come out past the object
                end = Math.Min(gaps.Count - 1, exit + (int)MathF.Round((float)Draw.Uniform(ref _rng, 0.02, 0.06) / LineStepMeters));
            }

            var path = new List<Vector3>(end - stage + 1);
            for (int i = stage; i < end; i++) path.Add(wrists[i]);
            path.Add(kind == ScriptMoveKind.Aim ? Vector3.Lerp(wrists[end - 1], wrists[end], f) : wrists[end]);

            Vector3 u = path[path.Count - 1] - path[Math.Max(0, path.Count - 3)];
            u = u.Length() > 1e-6f ? Vector3.Normalize(u) : radialDir;
            HandSnapshot before = _hand.Save();
            int drafts = _drafts.Count;
            double lastObjectEnd = _lastObjectEnd;
            // To where it starts, clear of its object's box by what that point leaves of it (less 4 mm, the model's margin kept), blind
            // to the others.
            float boxGap = (_ladders[oi] ??= new GapLadder(_profile, _model, o)).SignedGap(poses[stage]);
            ArmCollisionModel staging = SafetyScript.WithObjects(_model, new[] { o.Grown(boxGap - 0.004f - _model.Margin) }, 0f);
            if (!_hand.MixedTo(wrists[stage], pitch, null, Slowness(), 0.004, staging, out double s, out double e, overTheTop: true))
            {
                Undo(before, drafts);
                return Miss(4);
            }

            Add(ScriptMoveKind.Free, -1, float.NaN, s, e);
            if (!near) MaybePause();
            if (!_hand.MixedAlong(path, pitch, Slowness(), kind == ScriptMoveKind.Aim ? 0.0025 : 0.008, kind == ScriptMoveKind.Aim ? 2 : 0, 0.5, _model, out s, out e))
            {
                Undo(before, drafts);
                return Miss(5);
            }

            _objectMoves++;
            Add(kind, oi, offset, s, e, approach);
            _lastObjectEnd = Math.Max(_lastObjectEnd, e);
            if (offset < 0 && o.Movable)
            {
                // Pushed into, it may be anywhere afterwards: no later move aims at it, and the tail's goals keep away from it.
                _moved.Add(oi);
                var c = new Vector2(o.Center.X, o.Center.Y);
                _avoid.Add(c);
                _avoid.Add(c + 0.10f * Vector2.Normalize(new Vector2(u.X, u.Y) + new Vector2(1e-6f, 0f)));
            }

            // Into a fixture, the hand always pulls back out (a person does; and the blind moves after keep out of fixtures); else now
            // and then.
            bool intoFixture = !o.Movable && offset < 0.005f;
            if (intoFixture || Draw.Chance(ref _rng, _options.ReverseAfterShare))
            {
                Vector3 back = _hand.TargetEnd - u * (float)Draw.Uniform(ref _rng, 0.03, 0.08) + new Vector3(0f, 0f, (float)Draw.Uniform(ref _rng, 0.01, 0.04));
                if (_hand.MixedTo(back, null, null, Slowness(), 0.008, _model, out s, out e))
                {
                    Add(ScriptMoveKind.Reverse, -1, float.NaN, s, e);
                    _lastObjectEnd = Math.Max(_lastObjectEnd, e);
                }
                else if (intoFixture)
                {
                    Undo(before, drafts);
                    _lastObjectEnd = lastObjectEnd;
                    _objectMoves--;
                    return Miss(5);
                }
            }
            else
            {
                MaybePause();
            }

            return true;
        }

        private void Undo(HandSnapshot before, int drafts)
        {
            _hand.Restore(before);
            _drafts.RemoveRange(drafts, _drafts.Count - drafts);
        }

        /// <summary>
        /// The wrist pitch that puts the pads <paramref name="radius"/> from the base axis near the table (measured 2026-11-22 over the
        /// IK and the envelope: pointing down they reach 18-24 cm out, 2.5-9 cm above the table; at -60 degrees 24-33 cm, 4-18 cm up;
        /// at -45 degrees 27-33 cm), radians, interpolated and clamped to that table.
        /// </summary>
        internal static float PitchFor(float radius)
        {
            float[] r = { 0.18f, 0.21f, 0.24f, 0.27f, 0.30f, 0.33f }, degrees = { -88f, -82f, -70f, -55f, -42f, -30f };
            if (radius <= r[0]) return degrees[0] * MathF.PI / 180f;
            for (int i = 1; i < r.Length; i++)
            {
                if (radius <= r[i]) return (degrees[i - 1] + (degrees[i] - degrees[i - 1]) * (radius - r[i - 1]) / (r[i] - r[i - 1])) * MathF.PI / 180f;
            }

            return degrees[degrees.Length - 1] * MathF.PI / 180f;
        }

        /// <summary>How far <paramref name="o"/>'s box reaches along <paramref name="axis"/> (a unit vector) from its centre, m.</summary>
        private static float Support(in SafetyObject o, Vector3 axis)
        {
            Vector3 x = Vector3.Transform(Vector3.UnitX, o.Rotation), y = Vector3.Transform(Vector3.UnitY, o.Rotation), z = Vector3.Transform(Vector3.UnitZ, o.Rotation);
            return MathF.Abs(Vector3.Dot(x, axis)) * o.HalfExtents.X + MathF.Abs(Vector3.Dot(y, axis)) * o.HalfExtents.Y + MathF.Abs(Vector3.Dot(z, axis)) * o.HalfExtents.Z;
        }

        /// <summary>
        /// Where an aim at <paramref name="offset"/> ends on its line: the first index whose gap is at or below it, and the fraction of
        /// the step before it that reaches it. Past what the model measures clear, the end is extrapolated back from where its measure
        /// begins, at the slope the line closes on the object there. False when the line never comes that close, or starts too close.
        /// </summary>
        internal static bool AimIndex(IReadOnlyList<float> gaps, float offset, out int end, out float fraction)
        {
            end = -1;
            fraction = 1f;
            if (gaps.Count < 3 || gaps[0] < Math.Min(offset + 0.02f, MeasuredGap)) return false;
            if (offset < MeasuredGap)
            {
                for (int i = 1; i < gaps.Count && end < 0; i++) if (gaps[i] <= offset) end = i;
                if (end < 0) return false;
                float g0 = gaps[end - 1], g1 = gaps[end];
                fraction = g0 > g1 ? Math.Clamp((g0 - offset) / (g0 - g1), 0f, 1f) : 1f;
                return true;
            }

            int first = -1;
            for (int i = 1; i < gaps.Count && first < 0; i++) if (gaps[i] < MeasuredGap) first = i;
            if (first < 0) return false;
            int m = Math.Min(4, gaps.Count - 1 - first);
            float slope = m > 0 ? (gaps[first] - gaps[first + m]) / (m * LineStepMeters) : 1f;
            slope = Math.Clamp(slope, 0.3f, 1f);
            float back = (offset - gaps[first]) / (slope * LineStepMeters);
            float at = first - back;
            if (at < 1f) return false;
            end = (int)MathF.Ceiling(at);
            fraction = 1f - (end - at);
            return end < gaps.Count;
        }

        /// <summary>
        /// An object-directed move's line: the pads' midpoint through <paramref name="tools"/> (5 mm apart), the wrist pitched at
        /// <paramref name="pitch"/>, solved by the headset's IK point to point outward from the object (from index
        /// <paramref name="meet"/>, where the path meets it, back toward the start and on toward the end), each part as far as its
        /// poses are in reach, in the envelope, free of the robot's own model and on one branch. The wrist targets in order, their
        /// poses, the arm's gap to the object at each, and the index in <paramref name="tools"/> of the first.
        /// </summary>
        private bool Line(int oi, List<Vector3> tools, int meet, ref float pitch, float gripper, out List<Vector3> wrists, out List<ArmPose> poses, out List<float> gaps,
            out int firstIndex)
        {
            int n = tools.Count;
            wrists = new List<Vector3>(n);
            poses = new List<ArmPose>(n);
            gaps = new List<float>(n);
            firstIndex = 0;
            var wrist = new Vector3?[n];
            var solved = new ArmPose[n];
            int seed = -1;
            // A first pose as near the object as the path has one, at the pitch asked for or the nearest one that reaches it (the
            // solver refuses what would hit the floor or the arm itself, and the envelope what bends a joint too far: which
            // pitches reach a point near the table changes with its radius and height); then outward from it both ways.
            float asked = pitch;
            var works = new List<int>(SeedPitchSteps.Length);
            for (int k = 0; k < SeedPitchSteps.Length; k++)
            {
                if (Seed(tools, meet, PitchStep(asked, k), wrist, solved) >= 0) works.Add(k);
                else if (works.Count > 0 && k >= 2) break; // past the run of pitches that work on this side
            }

            if (works.Count > 0)
            {
                // The middle of the pitches that work, as near the one drawn as that allows: a line pitched at the edge of what reaches
                // its first point breaks a step or two on.
                var degrees = works.Select(k => SeedPitchSteps[k]).OrderBy(d => d).ToList();
                float middle = degrees[degrees.Count / 2];
                int best = works.OrderBy(k => MathF.Abs(SeedPitchSteps[k] - middle)).First();
                pitch = PitchStep(asked, best);
                seed = Seed(tools, meet, pitch, wrist, solved);
            }

            if (seed < 0)
            {
                _lineBreaks[0]++;
                return false;
            }

            int first = seed, last = seed;
            for (int i = seed - 1; i >= 0; i--)
            {
                if (!wrist[i].HasValue) { _lineBreaks[5]++; break; }
                int why = Why(solved[i + 1], wrist[i]!.Value, pitch, out solved[i]);
                if (why > 0) { _lineBreaks[why]++; break; }
                first = i;
            }

            for (int i = seed + 1; i < n; i++)
            {
                if (!wrist[i].HasValue) { _lineBreaks[5]++; break; }
                int why = Why(solved[i - 1], wrist[i]!.Value, pitch, out solved[i]);
                if (why > 0) { _lineBreaks[why]++; break; }
                last = i;
            }

            for (int i = first; i <= last; i++)
            {
                wrists.Add(wrist[i]!.Value);
                poses.Add(solved[i]);
                gaps.Add(Gap(oi, solved[i], gripper));
            }

            firstIndex = first;
            return wrists.Count >= 3;
        }

        private static float PitchStep(float asked, int k) => Math.Clamp(asked + SeedPitchSteps[k] * MathF.PI / 180f, -89f * MathF.PI / 180f, -8f * MathF.PI / 180f);

        /// <summary>The wrists of <paramref name="tools"/> at <paramref name="pitch"/> into <paramref name="wrist"/>, and the first of them near <paramref name="meet"/> the IK reaches (its pose in <paramref name="solved"/>), or -1.</summary>
        private int Seed(List<Vector3> tools, int meet, float pitch, Vector3?[] wrist, ArmPose[] solved)
        {
            int n = tools.Count;
            float lh = ToolInGripper.X * MathF.Cos(pitch) - ToolInGripper.Z * MathF.Sin(pitch);
            float lv = ToolInGripper.X * MathF.Sin(pitch) + ToolInGripper.Z * MathF.Cos(pitch);
            for (int i = 0; i < n; i++)
            {
                Vector3 tool = tools[i];
                var radial = new Vector2(tool.X, tool.Y);
                wrist[i] = null;
                if (radial.Length() < 0.05f) continue; // over the base
                radial = Vector2.Normalize(radial);
                wrist[i] = new Vector3(tool.X - lh * radial.X, tool.Y - lh * radial.Y, tool.Z - lv);
            }

            for (int d = 0; d <= 6; d++)
            {
                foreach (int i in new[] { meet - d, meet + d })
                {
                    if (i < 0 || i >= n || !wrist[i].HasValue) continue;
                    if (Solve(_model.Home, wrist[i]!.Value, pitch, out solved[i], false)) return i;
                }
            }

            return -1;
        }

        /// <summary>The pitches a line's first pose tries, degrees about the one drawn: that one, then nearer ones first.</summary>
        private static readonly float[] SeedPitchSteps = { 0f, 8f, -8f, 16f, -16f, 24f, -24f, 32f, -32f, 40f, -40f, 48f, -48f };

        /// <summary>The first of <paramref name="tools"/> inside <paramref name="o"/>'s box, or, when none is, the one nearest it.</summary>
        internal static int Meets(in SafetyObject o, List<Vector3> tools)
        {
            Quaternion toBox = Quaternion.Conjugate(Quaternion.Normalize(o.Rotation));
            int nearest = 0;
            float best = float.MaxValue;
            for (int i = 0; i < tools.Count; i++)
            {
                Vector3 p = Vector3.Transform(tools[i] - o.Center, toBox);
                Vector3 outside = Vector3.Max(Vector3.Abs(p) - o.HalfExtents, Vector3.Zero);
                float d = outside.Length();
                if (d <= 0f) return i;
                if (d < best)
                {
                    best = d;
                    nearest = i;
                }
            }

            return nearest;
        }

        /// <summary>Why a chained point of a line fails: 0 none, 1 the IK, 2 the envelope, 3 the robot's model, 4 a branch flip.</summary>
        private int Why(in ArmPose from, Vector3 wrist, float pitch, out ArmPose pose)
        {
            if (!TrySolve(from, wrist, pitch, out pose)) return 1;
            if (!_envelope.Contains(pose)) return 2;
            if (!ArmCollision.IsValid(_profile, _model, pose)) return 3;
            return from.MaxJointDelta(pose) <= 0.15f ? 0 : 4;
        }

        /// <summary>One point of a line: the IK from <paramref name="from"/>, in the envelope, free of the robot's own model, and (<paramref name="chained"/>) no branch flip from it.</summary>
        private bool Solve(in ArmPose from, Vector3 wrist, float pitch, out ArmPose pose, bool chained) =>
            TrySolve(from, wrist, pitch, out pose) && _envelope.Contains(pose) && ArmCollision.IsValid(_profile, _model, pose) && (!chained || from.MaxJointDelta(pose) <= 0.15f);

        /// <summary>The lowest point of the arm as the twin's links are (<see cref="ShapeGap.TouchBodies"/>, the gripper as given), arm base frame z: how high a contact is.</summary>
        internal static float Lowest(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose, float gripper = SafetyScript.ApproachGripper)
        {
            Span<LinkFrame> frames = stackalloc LinkFrame[5];
            ArmCollision.ComputeFrames(profile, pose, frames);
            float low = float.MaxValue;
            foreach (CollisionBody b in ShapeGap.TouchBodies(model, gripper))
            {
                LinkFrame f = frames[(int)b.Link];
                for (int k = 0; k < 8; k++)
                {
                    var corner = new Vector3((k & 1) == 0 ? -b.HalfExtents.X : b.HalfExtents.X, (k & 2) == 0 ? -b.HalfExtents.Y : b.HalfExtents.Y,
                        (k & 4) == 0 ? -b.HalfExtents.Z : b.HalfExtents.Z);
                    low = MathF.Min(low, f.ToBase(b.Center + Vector3.Transform(corner, b.Rotation)).Z);
                }
            }

            return low;
        }

        /// <summary>The headset's IK from <paramref name="from"/> to the wrist <paramref name="target"/>, roll 0, until it gets there.</summary>
        private bool TrySolve(in ArmPose from, Vector3 target, float pitch, out ArmPose pose)
        {
            var solver = new ArmSolver();
            solver.Reset(_profile, from);
            pose = from;
            for (int i = 0; i < TrainingTargetSampler.MaxSolvesPerCheck; i++)
            {
                ArmSolution s = solver.Solve(_profile, target, pitch, 0f);
                if (s.Status == ArmSolveStatus.Approaching) continue;
                pose = s.Pose;
                return (s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted) && !s.ReachClamped &&
                       Vector3.Distance(s.Position, target) <= TrainingTargetSampler.ReachToleranceMeters;
            }

            return false;
        }
    }
}
