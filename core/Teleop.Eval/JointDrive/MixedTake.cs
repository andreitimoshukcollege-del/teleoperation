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
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>What the mixed script may be told (joint-drive's options; docs/adr/0038 decision 1).</summary>
    public sealed class MixedOptions
    {
        /// <summary>
        /// The share of moves aimed at an object (an aim or a pass), when one can be; the rest are layout-blind. 0.85 (0.6 before docs/adr/0038
        /// "M, motion"): an aim made as a free move is made plans less often, and this keeps the planned contacts a take as they were.
        /// </summary>
        public double ObjectShare { get; set; } = 0.85;

        /// <summary>The share of object-directed moves that pass across the object rather than reach at it (0.45: about one pass to five aims planned, as before docs/adr/0038 "M, motion").</summary>
        public double PassShare { get; set; } = 0.45;

        /// <summary>
        /// The share of aims that start close to their object (4-20 mm from it), so a contact comes early in the move. 0 by default: an aim's
        /// start is as far back along its line as a free move's goal is from the hand (one law, docs/adr/0038 "M, motion"), whose short
        /// draws start close anyway.
        /// </summary>
        public double NearStartShare { get; set; }

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

        /// <summary>The share of moves not pulled back from (<see cref="PullBackShare"/>) followed by a pause of <see cref="PauseMinSeconds"/>-<see cref="PauseMaxSeconds"/>, whatever the move.</summary>
        public double PauseShare { get; set; } = 0.35;

        public double PauseMinSeconds { get; set; } = 0.15;
        public double PauseMaxSeconds { get; set; } = 0.70;

        /// <summary>
        /// The share of moves of every kind (a layout-blind part, an aim, a pass) followed by a pull back the way they came (a reverse:
        /// 3-8 cm back, 1-4 cm up). One law for all (docs/adr/0038 "M, motion": only object moves pulled back, and a model of the path
        /// could tell them by it); a move into a fixture always pulls back out.
        /// </summary>
        public double PullBackShare { get; set; } = 0.15;

        /// <summary>
        /// The share of aims and passes that start a small adjustment's distance back (<see cref="Draw.AdjustDistance"/>); the rest start a
        /// layout-blind goal's distance back (<see cref="Draw.ReachDistance"/>): 0.1 puts aims' lengths on free moves' (docs/adr/0038 "M,
        /// motion"; free moves mix goals, adjustments and the longer moves to where aims start). Below 0: the share of small adjustments
        /// among the layout-blind parts that are not strokes (<see cref="BlindTransportShare"/>, <see cref="BlindReverseShare"/>).
        /// </summary>
        public double ShortApproachShare { get; set; } = 0.1;

        /// <summary>The most an aim or a pass turns the wrist's pitch on its way, degrees a metre of its length (free transports turn about 130 at the median).</summary>
        public double TurnDegreesPerMeter { get; set; } = 300;

        /// <summary>A layout-blind part is a move to a goal with this share, back-and-forth strokes with <see cref="BlindReverseShare"/>, else small adjustments.</summary>
        public double BlindTransportShare { get; set; } = 0.55;

        public double BlindReverseShare { get; set; } = 0.25;

        /// <summary>
        /// Where layout-blind goals are drawn: every one low (the wrist at most 5 cm above the base frame, the gripper pitched -45 to -85
        /// degrees, the pads 2-14 cm above the table), docs/adr/0038 "M, motion": aims are as low as the objects they reach at, and 40% low
        /// blind goals left the path's height telling the two apart (AUC 0.80 alone).
        /// </summary>
        public BlindReach Reach { get; set; } = new BlindReach { NearShare = 0.8, RadialScale = 0.55f, VerticalScale = 0.55f, LowShare = 1.0, LowHeightHigh = 0.05f, ToolBands = DefaultToolBands() };

        /// <summary>
        /// Where they are drawn in the quiet start and end, which keep clear of every object (the guard): as low as elsewhere (docs/adr/0038
        /// "M, motion": never low before, so 42% of the free moves, those of the start and the tail, were 24 cm up at the median, and the
        /// path's height told them from aims).
        /// </summary>
        public BlindReach QuietReach { get; set; } = new BlindReach { LowShare = 1.0, NearShare = 0.7, HeightLow = 0.0f, LowHeightHigh = 0.05f, ToolBands = DefaultToolBands() };

        /// <summary>
        /// The tool point's heights layout-blind goals are drawn at (<see cref="BlindReach.ToolBands"/>), base frame z (the table 19.6 cm below
        /// it): low, mid and high, as aims meet objects (docs/adr/0038 "M, motion", the 2026-11-23 review: every blind goal low had the
        /// path's height telling a take that hits from one that does not; aims and free moves share one height distribution).
        /// </summary>
        public static float[] DefaultToolBands() => new[] { -0.180f, -0.120f, 0.60f, -0.120f, -0.070f, 0.28f, -0.070f, 0.020f, 0.12f };

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

        /// <summary>
        /// The share of the take's parts (a layout-blind part, or an aim or a pass with the move to where it starts) that carry an accident
        /// (docs/adr/0038 "M, motion", world-models' request 3: <see cref="AccidentModel"/>), drawn before the part is chosen and from a draw of
        /// its own, so where and when accidents come does not depend on whether an object is near: in free space, in the quiet start and the
        /// tail (clear of the objects there, as every move is), and on aims alike. 0: none, and the take plans exactly as without them.
        /// </summary>
        public double AccidentShare { get; set; } = 0.12;

        /// <summary>
        /// On top of <see cref="AccidentShare"/>, the share of the main part's would-be layout-blind parts that are a move to a point near an object (a few cm off
        /// its surface, at a height it stands over) with an accident in it (drawn from the accidents' own generator too; when none plans, a
        /// layout-blind part carries it): accidents placed near objects, so about a third of the contacts come from accidents (docs/adr/0038
        /// "M, motion"). Recorded as a <c>free</c> move with its accident.
        /// </summary>
        public double AccidentNearShare { get; set; } = 0.4;

        /// <summary>The accidents' kinds, in proportion: jerk, overshoot, wrong way, slip, rush (<see cref="AccidentKinds"/>).</summary>
        public double[] AccidentWeights { get; set; } = { 1, 1, 1, 1, 1 };

        /// <summary>The kinds <see cref="AccidentWeights"/> weigh, in order.</summary>
        public static readonly ScriptMoveKind[] AccidentKinds = { ScriptMoveKind.Jerk, ScriptMoveKind.Overshoot, ScriptMoveKind.WrongWay, ScriptMoveKind.Slip, ScriptMoveKind.Rush };

        /// <summary>No accidents (<see cref="AccidentShare"/> and <see cref="AccidentNearShare"/> 0).</summary>
        public bool NoAccidents => !(AccidentShare > 0) && !(AccidentNearShare > 0);

        /// <summary>Throws when the options make no sense together.</summary>
        public void Validate()
        {
            if (!(AccidentShare >= 0 && AccidentShare <= 1) || !(AccidentNearShare >= 0 && AccidentNearShare <= 1))
                throw new ArgumentException($"the accident shares must be in [0, 1], not {AccidentShare} and {AccidentNearShare}");
            if (AccidentWeights == null || AccidentWeights.Length != AccidentKinds.Length || AccidentWeights.Any(w => !(w >= 0)) || (!NoAccidents && !(AccidentWeights.Sum() > 0)))
                throw new ArgumentException("the accident kinds' weights are five numbers at least 0, some above 0");

            foreach ((double v, string name) in new[] { (ObjectShare, "object share"), (PassShare, "pass share"), (NearStartShare, "near-start share"), (PauseShare, "pause share"),
                         (PullBackShare, "pull-back share"), (BlindTransportShare, "blind transport share"), (BlindReverseShare, "blind reverse share") })
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
            if (!(ShortApproachShare <= 1) || !(TurnDegreesPerMeter >= 0 && TurnDegreesPerMeter <= 2000)) throw new ArgumentException("the short approach share must be at most 1, the turn 0-2000 degrees a metre");
            Reach.Validate();
            QuietReach.Validate();
            Caps?.Validate();
        }
    }

    /// <summary>
    /// What part of the plan a mixed take's move is (docs/adr/0038 "M, motion"), for reports only (the dry run's <c>--features</c>): the
    /// record carries its kind alone, and a model of the path must not tell these apart.
    /// </summary>
    public enum MovePart : byte
    {
        Other = 0,

        /// <summary>A layout-blind move to a goal.</summary>
        BlindTransport = 1,

        /// <summary>Layout-blind back-and-forth strokes.</summary>
        BlindStrokes = 2,

        /// <summary>Layout-blind small adjustments.</summary>
        BlindAdjust = 3,

        /// <summary>The move to where an aim or a pass starts.</summary>
        Staging = 4,

        /// <summary>An aim or a pass.</summary>
        Approach = 5,

        /// <summary>A pull back after a move, drawn.</summary>
        PullBack = 6,

        /// <summary>A pull back out of a fixture that the draw did not give: the hand always pulls back out of one.</summary>
        ForcedPullBack = 7,

        Pause = 8,

        /// <summary>A cornered hand's move back toward the middle of reach.</summary>
        Recentre = 9,

        /// <summary>The tail's first move, up and clear.</summary>
        Exit = 10,

        /// <summary>A move to a point near an object with an accident in it (<see cref="MixedOptions.AccidentNearShare"/>).</summary>
        NearAccident = 11,
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

        /// <summary>What part of the plan it is (<see cref="MovePart"/>). Reports only.</summary>
        public readonly MovePart Part;

        /// <summary>For an accident, the <see cref="Index"/> of the move it came in (0: none). Reports only.</summary>
        public readonly int Overlies;

        public PlannedMove(int index, ScriptMoveKind kind, byte objectId, float offsetMeters, float gapMeters, double start, double contact, double closest, double end,
            float approachDegrees = float.NaN, MovePart part = MovePart.Other, int overlies = 0)
        {
            Overlies = overlies;
            ApproachDegrees = approachDegrees;
            Part = part;
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
    /// <item>Aims and passes move as free moves do (docs/adr/0038 "M, motion", measured by the dry run's <c>--features</c>): each is made
    /// by the same constructor as a layout-blind move to a goal (endpoint error, undershoot, corrections to the same tolerance, curved or
    /// in two pushes as often, the roll and the gripper as they go, the wrist's pitch turning on the way), starts as far back as a free
    /// move's goal is from the hand, and comes at the height free moves go (every layout-blind goal is low).</item>
    /// <item>Every move draws its slowness from one distribution, and every move is followed by a pull back or a pause by one law; a move
    /// that ends in or at a fixture always pulls back out.</item>
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

        /// <summary>
        /// The highest an aim or a pass meets an object, base frame z (docs/adr/0038 "M, motion", the 2026-11-23 review: contacts at every
        /// height): 29.6 cm above the table, where the arm still reaches with the wrist near level over the band objects stand in. Below it,
        /// an object is met anywhere up its height (<see cref="ReachTop"/>), the wrist pitched for that height (<see cref="PitchAt"/>).
        /// </summary>
        public const float HighestContactMeters = 0.10f;

        /// <summary>The lowest an aim or a pass meets an object raised on a stand, m above its bottom (the stand's top): the pads clear of it. The approach keeps off the stand (<see cref="HolderClearMeters"/>).</summary>
        public const float RaisedLowestMeters = 0.010f;

        /// <summary>An aim or a pass at a plank comes at it from this far below its bottom (from below: the arm's top meets its underside) up to its top.</summary>
        public const float PlankUnderMeters = 0.030f;

        /// <summary>An aim or a pass at what a stand holds (or at a plank) keeps this far off the stand (or the plank's supports): it meets its object, not what holds it up.</summary>
        public const float HolderClearMeters = 0.005f;

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
        private readonly List<int>?[] _holders;
        private readonly List<Draft> _drafts = new List<Draft>();
        private readonly HashSet<int> _moved = new HashSet<int>(); // objects a move planned to push into, as later targets
        private readonly ArmCollisionModel[]? _walls; // the fixed objects' boxes at each of WallOffsets (null: no fixed object)
        private readonly List<Vector2> _avoid = new List<Vector2>();
        private BlindReach? _quietHigh; // the quiet windows' goals, none low: their fallback
        private BlindReach? _tailReach; // the tail's goals after its exit: the quiet windows' without the low band
        private SeededRng _rng;
        private SeededRng _accidentRng; // where and when accidents come: a generator of its own, so drawing them changes no other draw's place
        private readonly int[] _accidentsMade = new int[MixedOptions.AccidentKinds.Length], _accidentsDropped = new int[MixedOptions.AccidentKinds.Length];

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

            return counts + "; objects: " + string.Join(", ", objects) + (_options.NoAccidents ? "" : "; " + AccidentReport());
        }

        /// <summary>The accidents made, and those drawn that no try could make (the move made without one), by kind.</summary>
        internal string AccidentReport()
        {
            var parts = new List<string>();
            for (int k = 0; k < MixedOptions.AccidentKinds.Length; k++)
                parts.Add(FormattableString.Invariant($"{ScriptMoveCodec.Name(MixedOptions.AccidentKinds[k])} {_accidentsMade[k]}/{_accidentsMade[k] + _accidentsDropped[k]}"));
            int[] r = _hand.AccidentRefusals;
            return "accidents made of drawn: " + string.Join(", ", parts) + $"; near objects {_nearMade} of {_nearTried} parts, tries {_nearTries}: unreachable {_nearUnreachable}, refused {_nearRefused} (their ticks refused: IK {r[0]}, envelope or model {r[1]}, jump {r[2]}, quiet margin {r[3]}, caps {r[4]})";
        }

        /// <summary>
        /// Whether the next part carries an accident, and its kind (docs/adr/0038 "M, motion"): drawn with <paramref name="share"/> from the
        /// accidents' own generator, before anything about the part is known; null for none.
        /// </summary>
        private ScriptMoveKind? DrawAccident(double share)
        {
            if (!(share > 0) || !Draw.Chance(ref _accidentRng, share)) return null;
            double[] w = _options.AccidentWeights;
            double u = _accidentRng.NextDouble() * w.Sum();
            for (int k = 0; k < w.Length; k++)
            {
                u -= w[k];
                if (u < 0 && w[k] > 0) return MixedOptions.AccidentKinds[k];
            }

            for (int k = w.Length - 1; k >= 0; k--) if (w[k] > 0) return MixedOptions.AccidentKinds[k];
            return null;
        }

        private void Tally(ScriptMoveKind accident, bool made)
        {
            int k = Array.IndexOf(MixedOptions.AccidentKinds, accident);
            if (made) _accidentsMade[k]++;
            else _accidentsDropped[k]++;
        }

        private bool Miss(int why)
        {
            _objectMisses[why]++;
            return false;
        }

        /// <summary>A move as planned, before the plan is final: its times on the hand's timeline, its object's index (-1: none).</summary>
        private struct Draft
        {
            public int Id, Base; // its own id, and for an accident the id of the move it came in (-1: none)
            public ScriptMoveKind Kind;
            public int Object;
            public float Offset, Approach;
            public double Start, End;
            public MovePart Part;
        }

        private MixedTake(in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, IReadOnlyList<SafetyObject> objects, MixedOptions options,
            HumanOperator hand, double start, SeededRng rng, SeededRng accidents)
        {
            _accidentRng = accidents;
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
            _holders = new List<int>?[objects.Count];
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
            // An accident's ticks keep to the arm's own limits: the servos ramp at the commanded speed, which joint-drive's --speed sets.
            hand.AccidentCaps = AccidentModel.CapsFor(options.Caps, speed);
            ulong takeSeed = rng.NextUInt64();
            // Where and when accidents come is drawn from a generator of its own (docs/adr/0038 "M, motion"), seeded from the take's.
            var take = new MixedTake(profile, model, envelope, objects, options, hand, t, new SeededRng(takeSeed), new SeededRng(takeSeed ^ 0xA5C1_DE47_5EED_0001UL));
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
                ScriptMoveKind? accident = DrawAccident(_options.AccidentShare);
                bool objectTime = !first && _hand.Free >= _options.QuietStartSeconds - 0.9;
                if (objectTime && Draw.Chance(ref _rng, _options.ObjectShare)) done = TryObjectMove(accident);
                // Accidents placed near objects, on top of those drawn for every part: a part that would be layout-blind is, with this share, a
                // move near an object with one (a layout-blind part carries it when none plans). The aims and passes stay as many.
                if (!done && objectTime && !accident.HasValue && (accident = DrawAccident(_options.AccidentNearShare)).HasValue) done = TryNearAccident(accident.Value);
                if (!done) done = TryBlindMove(first, accident: accident);
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
            if (!Exit(out double tailStart) && (!Recentre() || !Exit(out tailStart)))
            {
                error = FormattableString.Invariant($"no move from {_hand.Free:0.0} s on of the take ends clear of every object ({Refusals()})");
                return false;
            }

            _hand.GuardFrom = tailStart;
            _hand.AvoidGoals = _avoid.Count > 0 ? _avoid : null;
            double cut = double.NaN;
            for (int part = 0; part < 12 && double.IsNaN(cut); part++)
            {
                if (!TryBlindMove(false, tailOnly: true, out double s, out double e, DrawAccident(_options.AccidentShare)))
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
        /// When no layout-blind part works from where the hand is (cornered at the edge of the envelope, low by the table, or folded against
        /// the arm's own body): a lift straight up (4-10 cm), else a move back toward the middle of reach (the wrist 12-17 cm out, 8-16 cm up,
        /// within 40 degrees of where it is), keeping the quiet start's margin when it applies (not the walls). Also before the tail's exit
        /// is tried again. False when that does not work either.
        /// </summary>
        private bool Recentre()
        {
            const double deg = Math.PI / 180.0;
            Vector3 at = _hand.TargetEnd;
            float here = MathF.Atan2(at.Y, at.X);
            for (int attempt = 0; attempt < 11; attempt++)
            {
                // First straight up from where it is, the wrist as it is (the way out of a low pose by the table, where every layout-blind
                // goal is: docs/adr/0038 "M, motion"); then toward the middle of reach.
                Vector3 goal;
                float? pitch = null;
                if (attempt < 3) goal = at + new Vector3(0f, 0f, (float)Draw.Uniform(ref _rng, 0.04, 0.10));
                else
                {
                    float r = (float)Draw.Uniform(ref _rng, 0.12, 0.17), z = (float)Draw.Uniform(ref _rng, 0.08, 0.16);
                    float az = Math.Clamp(here + (float)(Draw.Uniform(ref _rng, -40, 40) * deg), (float)(-80 * deg), (float)(80 * deg));
                    goal = new Vector3(r * MathF.Cos(az), r * MathF.Sin(az), z);
                    pitch = (float)(Draw.Uniform(ref _rng, -45, 0) * deg);
                }

                // No walls: a hand cornered beside a fixture is often inside its wall already (a move blind to the other objects, an aim
                // or the move to where one starts, may end in one), and every wall refuses it.
                Vector3 from = _hand.TargetEnd;
                if (!_hand.MixedTo(goal, pitch, null, Slowness(), 0.010, _model, out double s, out double e)) continue;
                Add(ScriptMoveKind.Free, -1, float.NaN, s, e, part: MovePart.Recentre);
                AfterMove(_hand.TargetEnd - from, pullBack: true, walls: true);
                return true;
            }

            return false;
        }

        /// <summary>
        /// The tail's first move: up (3-7 cm, to a drawn 0-10 cm above the base frame's origin at least; from the 13th try 5-10 cm, to 6-16 cm)
        /// and toward the middle of reach from
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
                float z = attempt < 12
                    ? MathF.Min(MathF.Max(at.Z + (float)Draw.Uniform(ref _rng, 0.03, 0.07), (float)Draw.Uniform(ref _rng, 0.0, 0.10)), top)
                    : MathF.Min(MathF.Max(at.Z + (float)Draw.Uniform(ref _rng, 0.05, 0.10), (float)Draw.Uniform(ref _rng, 0.06, 0.16)), top);
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

                Add(ScriptMoveKind.Free, -1, float.NaN, s, e, part: MovePart.Exit);
                end = e;
                _hand.GuardFrom = e; // what follows keeps the quiet end's margin
                AfterMove(goal - at, pullBack: true, walls: true);
                _hand.GuardFrom = double.PositiveInfinity;
                return true;
            }

            return false;
        }

        /// <summary>The moves as planned, measured on the final plan, in the order they begin; those after <paramref name="cut"/> (plan seconds) left out.</summary>
        private PlannedMove[] Moves(double cut)
        {
            // In the order they begin; an accident that begins with its move after it (the move under way when a contact comes is the last to begin).
            _drafts.Sort((a, b) =>
            {
                int c = a.Start.CompareTo(b.Start);
                return c != 0 ? c : ScriptMoveCodec.IsAccident(a.Kind).CompareTo(ScriptMoveCodec.IsAccident(b.Kind));
            });
            var moves = new List<PlannedMove>(_drafts.Count);
            var indexOf = new Dictionary<int, int>();
            foreach (Draft d in _drafts) if (_start + d.Start < cut) indexOf[d.Id] = indexOf.Count + 1;
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
                    d.Object >= 0 ? d.Approach : float.NaN, d.Part, d.Base >= 0 && indexOf.TryGetValue(d.Base, out int over) ? over : 0));
            }

            return moves.ToArray();
        }

        private int Add(ScriptMoveKind kind, int obj, float offset, double start, double end, float approach = float.NaN, MovePart part = MovePart.Other, int overlies = -1)
        {
            _drafts.Add(new Draft { Id = _nextDraft, Base = overlies, Kind = kind, Object = obj, Offset = offset, Start = start, End = end, Approach = approach, Part = part });
            return _nextDraft++;
        }

        private int _nextDraft;
        private int _nearTried, _nearMade, _nearTries, _nearUnreachable, _nearRefused; // accidents placed near objects, for the report

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

        /// <summary>A layout-blind part (a move, strokes or adjustments; then now and then a pull back or a pause), or, in the tail, one clear of every object.</summary>
        private bool TryBlindMove(bool first, bool tailOnly = false, ScriptMoveKind? accident = null) => TryBlindMove(first, tailOnly, out _, out _, accident);

        private bool TryBlindMove(bool first, bool tailOnly, out double start, out double end, ScriptMoveKind? accident = null)
        {
            bool quiet = tailOnly || _hand.Free < _options.QuietStartSeconds;
            Vector3 from = _hand.TargetEnd;
            // The tail after its exit keeps off the low band (docs/adr/0038 "M, motion"): the next take starts where this one ends, and a take
            // that starts low sweeps the table in its quiet start, where a replay's add may then stand nothing (chained dry runs: 70% of groups
            // with an add, against 86% from home).
            BlindReach reach = tailOnly ? (_tailReach ??= _options.QuietReach.WithoutLow()) : quiet ? _options.QuietReach : _options.Reach;
            ScriptMoveKind kind = ScriptMoveKind.Free;
            start = end = _hand.Free;
            // With an accident drawn, first with it; when no try makes it, the part as it would have been without one.
            bool made = accident.HasValue && _hand.MixedFree(_model, reach, Slowness(), _options.BlindTransportShare, _options.BlindReverseShare, out kind, out start, out end, DrawWalls(), accident);
            double onset = _hand.AccidentOnset, recovered = _hand.AccidentEnd;
            if (accident.HasValue) Tally(accident.Value, made);
            if (!made && !_hand.MixedFree(_model, reach, Slowness(), _options.BlindTransportShare, _options.BlindReverseShare, out kind, out start, out end, DrawWalls()))
            {
                // The quiet start and end keep clear of every object: where low goals all come too near one, goals that are not low.
                if (!quiet || (reach.ToolBands == null ? reach.LowShare <= 0 : reach.ToolBands.Length <= 3)) return false;
                _quietHigh ??= reach.WithoutLow();
                if (!_hand.MixedFree(_model, _quietHigh, Slowness(), _options.BlindTransportShare, _options.BlindReverseShare, out kind, out start, out end, DrawWalls())) return false;
            }

            int baseMove = Add(kind, -1, float.NaN, start, end, part: _hand.LastFreePart);
            if (made) Add(accident!.Value, -1, float.NaN, onset, recovered, part: _hand.LastFreePart, overlies: baseMove);
            // A blind move that ends at a fixture (within 5 mm of its box) pulls back out of it, as an aim into one does: the pull back
            // follows the contact, whatever kind of move made it.
            AfterMove(_hand.TargetEnd - from, pullBack: true, forced: AtFixture(end), walls: true, atLeast: 0.02f);
            return true;
        }

        /// <summary>Whether the hand's pose where its motion ends (seconds <paramref name="end"/> on its timeline) is within 5 mm of a fixed object's box.</summary>
        private bool AtFixture(double end)
        {
            if (_walls == null || _hand.Poses.Count == 0) return false;
            int k = Math.Min(_hand.Poses.Count - 1, Math.Max(0, (int)Math.Ceiling(end / HandModel.TickSeconds)));
            return !ArmCollision.IsValid(_profile, _walls[FixtureTouchWall], _hand.Poses[k]);
        }

        /// <summary>The index in <see cref="WallOffsets"/> of the fixtures grown by 5 mm: a move that ends inside it pulls back out.</summary>
        private const int FixtureTouchWall = 2;

        /// <summary>
        /// After a move, one law for every kind (docs/adr/0038 "M, motion": before, only aims and passes pulled back, and they paused less):
        /// a pull back the way the move went (<paramref name="way"/>, the target's travel over it: 3-8 cm back and 1-4 cm up) with
        /// <see cref="MixedOptions.PullBackShare"/>, else a pause with <see cref="MixedOptions.PauseShare"/>. With <paramref name="pullBack"/>
        /// false (the move to where an aim starts) the same draws, but no pull back. <paramref name="forced"/> (a move into a fixture): the
        /// pull back whatever the draw, and false when it finds no way. A layout-blind move's pull back keeps to the fixtures as walls
        /// (<paramref name="walls"/>), as the move did. A pull back goes at least <paramref name="atLeast"/> back (out of a fixture: as deep
        /// as the arm went in, and 1.5 cm more).
        /// </summary>
        private bool AfterMove(Vector3 way, bool pullBack, bool forced = false, bool walls = false, float atLeast = 0f)
        {
            bool back = Draw.Chance(ref _rng, _options.PullBackShare);
            if ((back && pullBack) || forced)
            {
                Vector3 u = way.Length() > 0.005f ? Vector3.Normalize(way) : LastDirection();
                float distance = MathF.Max((float)Draw.Uniform(ref _rng, 0.03, 0.08), atLeast);
                Vector3 to = _hand.TargetEnd - u * distance + new Vector3(0f, 0f, (float)Draw.Uniform(ref _rng, 0.01, 0.04));
                if (!_hand.MixedTo(to, null, null, Slowness(), 0.008, _model, out double s, out double e, walls: walls ? DrawWalls() : null))
                {
                    return !forced; // no way back: on from where it is, unless it is in a fixture
                }

                Add(ScriptMoveKind.Reverse, -1, float.NaN, s, e, part: back && pullBack ? MovePart.PullBack : MovePart.ForcedPullBack);
                if (back) return true;
            }

            // The pause, drawn whenever the law gave no pull back (a forced one included: the pull back out of a fixture, then the pause
            // any move might have had).
            if (!back && Draw.Chance(ref _rng, _options.PauseShare))
            {
                _hand.MixedPause(Draw.Uniform(ref _rng, _options.PauseMinSeconds, _options.PauseMaxSeconds), out double s, out double e);
                Add(ScriptMoveKind.Pause, -1, float.NaN, s, e, part: MovePart.Pause);
            }

            return true;
        }

        /// <summary>The direction of the hand's last submovement that moved it (x when none did): a pull back's way after a move that ended about where it began.</summary>
        private Vector3 LastDirection()
        {
            List<Submovement> moves = _hand.Timeline.Moves;
            for (int i = moves.Count - 1; i >= 0; i--)
            {
                if (moves[i].Displacement.Length() > 1e-4f) return Vector3.Normalize(moves[i].Displacement);
            }

            return Vector3.UnitX;
        }

        /// <summary>An aim or a pass at an object of the layout (a few tries over objects and geometries), with the move to where it starts; false when none plans.</summary>
        private bool TryObjectMove(ScriptMoveKind? accident = null)
        {
            var targets = new List<int>(_objects.Count);
            var weights = new List<float>(_objects.Count);
            for (int i = 0; i < _objects.Count; i++)
            {
                SafetyObject o = _objects[i];
                if (o.Scenery || _moved.Contains(i)) continue;
                // A stand or a plank's support is never a target (docs/adr/0038 "M, motion", the 2026-11-23 review: the stands under raised
                // objects took 18 of 41 contacts): what it holds up is.
                if (o.Role == SceneRole.Stand || o.Role == SceneRole.Support) continue;
                if (!HeightRange(i, out float zLo, out float zHi)) continue; // flat on the table: met only from above
                if (_missesInARow.TryGetValue(i, out int misses) && misses >= MaxObjectMisses) continue; // out of reach, it seems
                Vector3 c = o.Center;
                float r = new Vector2(c.X, c.Y).Length();
                if (r - Footprint(o) >= 0.40f) continue;
                targets.Add(i);
                // Drawn by how much of its height the arm meets it over (to the power 1.5: low heights are where every object is), so contacts
                // spread over every height objects stand at, not one height (docs/adr/0038 "M, motion").
                weights.Add(MathF.Pow(MathF.Max(0.03f, zHi - zLo), 1.5f));
            }

            if (targets.Count == 0) return false;
            for (int attempt = 0; attempt < 14; attempt++)
            {
                int oi = targets[Pick(weights)];
                ScriptMoveKind kind = Draw.Chance(ref _rng, _options.PassShare) ? ScriptMoveKind.Pass : ScriptMoveKind.Aim;
                if (TryObjectMove(oi, kind, accident))
                {
                    _missesInARow[oi] = 0;
                    return true;
                }

                _missesInARow[oi] = _missesInARow.TryGetValue(oi, out int m) ? m + 1 : 1;
                if (m + 1 >= MaxObjectMisses)
                {
                    int k = targets.IndexOf(oi);
                    targets.RemoveAt(k);
                    weights.RemoveAt(k);
                    if (targets.Count == 0) return false;
                }
            }

            return false;
        }

        /// <summary>
        /// An accident placed near an object (docs/adr/0038 "M, motion", <see cref="MixedOptions.AccidentNearShare"/>): a move to a point 1-6 cm
        /// off an object's surface (toward the base, within 50 degrees of it, at a height drawn over the object's own as an aim's is), the wrist
        /// pitched for that height, with an accident of <paramref name="kind"/> in it, checked as a layout-blind move is (the robot's own model,
        /// the walls but in the accident); then the law every move follows. False when no try plans.
        /// </summary>
        private bool TryNearAccident(ScriptMoveKind kind)
        {
            const double deg = Math.PI / 180.0;
            var targets = new List<int>(_objects.Count);
            for (int i = 0; i < _objects.Count; i++)
            {
                SafetyObject o = _objects[i];
                if (o.Scenery || o.Role == SceneRole.Stand || o.Role == SceneRole.Support || !HeightRange(i, out _, out _)) continue;
                if (new Vector2(o.Center.X, o.Center.Y).Length() - Footprint(o) >= 0.40f) continue;
                targets.Add(i);
            }

            _nearTried++;
            if (targets.Count == 0) return false;
            for (int attempt = 0; attempt < 16; attempt++)
            {
                _nearTries++;
                int oi = targets[(int)(_rng.NextDouble() * targets.Count) % targets.Count];
                SafetyObject o = _objects[oi];
                HeightRange(oi, out float zLo, out float zHi);
                float bearing = MathF.Atan2(o.Center.Y, o.Center.X) + MathF.PI + (float)(Draw.Uniform(ref _rng, -50, 50) * deg);
                var dir = new Vector3(MathF.Cos(bearing), MathF.Sin(bearing), 0f);
                Vector3 tool = new Vector3(o.Center.X, o.Center.Y, (float)Draw.Uniform(ref _rng, zLo, zHi)) + dir * (Support(o, dir) + (float)Draw.Uniform(ref _rng, 0.01, 0.06));
                // The wrist pitched for the height (PitchAt), else up to 36 degrees either way: where the hand would stop must be in reach, in the
                // envelope and clear of the robot itself, or no try there plans.
                float pitch0 = PitchAt(new Vector2(tool.X, tool.Y).Length(), tool.Z) + (float)(Draw.Uniform(ref _rng, -8, 8) * deg), pitch = pitch0;
                ArmPose last = _hand.Poses.Count > 0 ? _hand.Poses[_hand.Poses.Count - 1] : _model.Home;
                bool reachable = false;
                foreach (float d in new[] { 0f, 12f, -12f, 24f, -24f, 36f, -36f })
                {
                    pitch = Math.Clamp(pitch0 + d * (float)deg, -89f * (float)deg, 10f * (float)deg);
                    if (TrySolve(last, WristFor(tool, pitch), pitch, out ArmPose there) && _envelope.Contains(there) && ArmCollision.IsValid(_profile, _model, there))
                    {
                        reachable = true;
                        break;
                    }
                }

                if (!reachable)
                {
                    _nearUnreachable++;
                    continue;
                }

                Vector3 goal = WristFor(tool, pitch), from = _hand.TargetEnd;

                if (!_hand.MixedTo(goal, pitch, null, Slowness(), HandModel.Tolerance, _model, out double s, out double e, walls: DrawWalls(), accident: kind))
                {
                    _nearRefused++;
                    continue;
                }
                double onset = _hand.AccidentOnset, recovered = _hand.AccidentEnd;
                if (double.IsNaN(onset)) continue; // made without its accident (a try over the top): not this
                int baseMove = Add(ScriptMoveKind.Free, -1, float.NaN, s, e, part: MovePart.NearAccident);
                Add(kind, -1, float.NaN, onset, recovered, part: MovePart.NearAccident, overlies: baseMove);
                Tally(kind, true);
                _nearMade++;
                // Into a movable object by the model, it may be anywhere afterwards: no later aim at it, and the tail's goals keep away.
                bool pushed = false;
                int k1 = Math.Min(_hand.Poses.Count - 1, (int)Math.Ceiling(recovered / HandModel.TickSeconds));
                for (int k = Math.Max(0, (int)Math.Floor(onset / HandModel.TickSeconds)); k <= k1 && o.Movable && !pushed; k++) pushed = Gap(oi, _hand.Poses[k], _hand.Commands[k].Gripper) < 0f;
                if (pushed && _moved.Add(oi)) _avoid.Add(new Vector2(o.Center.X, o.Center.Y));
                AfterMove(_hand.TargetEnd - from, pullBack: true, forced: AtFixture(e), walls: true, atLeast: 0.02f);
                return true;
            }

            return false;
        }

        /// <summary>An index into <paramref name="weights"/>, drawn in proportion to them.</summary>
        private int Pick(List<float> weights)
        {
            float total = 0f;
            foreach (float w in weights) total += w;
            double u = _rng.NextDouble() * total;
            for (int i = 0; i < weights.Count; i++)
            {
                u -= weights[i];
                if (u < 0) return i;
            }

            return weights.Count - 1;
        }

        /// <summary>
        /// The tool point's heights an aim or a pass meets object <paramref name="oi"/> at, base frame z (docs/adr/0038 "M, motion", the
        /// 2026-11-23 review: over the object's own height, base to top, within what the arm reaches): on the table, from just above the
        /// model's floor (<see cref="LowestContactMeters"/>); raised on a stand, from just above the stand's top
        /// (<see cref="RaisedLowestMeters"/>); a plank, from <see cref="PlankUnderMeters"/> below it (met from the side, or from below) to its
        /// top; up to just below its top (never down through it) and <see cref="ReachTop"/>. False when that leaves too little: a flat object
        /// on the table, met only from above.
        /// </summary>
        private bool HeightRange(int oi, out float zLo, out float zHi)
        {
            SafetyObject o = _objects[oi];
            float floor = _model.FloorZ + LowestContactMeters; // the lowest the pads' centre passes anywhere
            if (o.Role == SceneRole.Plank)
            {
                zLo = MathF.Max(floor, o.Min.Z - PlankUnderMeters);
                zHi = o.Max.Z;
            }
            else
            {
                // Raised: from just above what holds it up (a tilted object's box reaches below the stand's top it rests on).
                float bottom = o.Min.Z;
                foreach (int k in Holders(oi)) bottom = MathF.Max(bottom, _objects[k].Max.Z);
                zLo = MathF.Max(floor, Holders(oi).Count > 0 ? bottom + RaisedLowestMeters : o.Min.Z + LowestContactMeters);
                zHi = o.Max.Z - (o.Shape != null ? ObjectShape.TopMarginMeters + 0.003f : 0.004f);
            }

            zHi = MathF.Min(zHi, ReachTop(new Vector2(o.Center.X, o.Center.Y).Length()));
            return zHi >= zLo + 0.003f;
        }

        /// <summary>
        /// The highest the tool point reaches <paramref name="radius"/> from the base axis, base frame z: the wrist level (the pads 15 cm out
        /// from it) and 2 cm inside the arm's length from the shoulder, at most <see cref="HighestContactMeters"/>.
        /// </summary>
        internal float ReachTop(float radius)
        {
            float reach = _profile.ProximalLinkLength + _profile.DistalLinkLength - 0.02f, out_ = MathF.Max(0f, radius - ToolInGripper.X);
            return MathF.Min(HighestContactMeters, _profile.BaseHeight + MathF.Sqrt(MathF.Max(0f, reach * reach - out_ * out_)));
        }

        /// <summary>
        /// The objects that hold object <paramref name="oi"/> up (docs/adr/0038 decision 3): the stand under what is raised on one (its top
        /// under the object's bottom, the object's centre over it), a plank's supports (their tops under its ends). An approach keeps off them.
        /// </summary>
        private List<int> Holders(int oi)
        {
            if (_holders[oi] != null) return _holders[oi]!;
            var list = new List<int>();
            SafetyObject o = _objects[oi];
            for (int k = 0; k < _objects.Count; k++)
            {
                if (k == oi) continue;
                SafetyObject s = _objects[k];
                bool under = s.Max.Z <= o.Min.Z + 0.015f && s.Max.Z >= o.Min.Z - 0.03f; // a tilted object's box reaches below what it rests on
                bool over = o.Role == SceneRole.Plank
                    ? s.Max.X > o.Min.X && s.Min.X < o.Max.X && s.Max.Y > o.Min.Y && s.Min.Y < o.Max.Y
                    : o.Center.X >= s.Min.X && o.Center.X <= s.Max.X && o.Center.Y >= s.Min.Y && o.Center.Y <= s.Max.Y;
                if (under && over && (s.Role == SceneRole.Stand || s.Role == SceneRole.Support || !s.Movable)) list.Add(k);
            }

            _holders[oi] = list;
            return list;
        }

        /// <summary>The robot's own model with the holders of object <paramref name="oi"/> (<see cref="Holders"/>) as walls kept <see cref="HolderClearMeters"/> off (the model's margin included); the model itself when it has none.</summary>
        private ArmCollisionModel HolderWalls(int oi)
        {
            List<int> holders = Holders(oi);
            if (holders.Count == 0) return _model;
            var walls = new SafetyObject[holders.Count];
            for (int k = 0; k < walls.Length; k++) walls[k] = _objects[holders[k]].Grown(HolderClearMeters - _model.Margin);
            return SafetyScript.WithObjects(_model, walls, 0f);
        }

        private static float Footprint(in SafetyObject o) => MathF.Max(o.Max.X - o.Min.X, o.Max.Y - o.Min.Y) / 2f;

        /// <summary>One aim or pass at object <paramref name="oi"/>: its line drawn and measured, then the hand's moves along it; the hand as it was when it does not plan.</summary>
        private bool TryObjectMove(int oi, ScriptMoveKind kind, ScriptMoveKind? accident = null)
        {
            _objectTries++;
            SafetyObject o = _objects[oi];
            const double deg = Math.PI / 180.0;
            float bearing = MathF.Atan2(o.Center.Y, o.Center.X);
            var radialDir = new Vector3(MathF.Cos(bearing), MathF.Sin(bearing), 0f);
            var tangentDir = new Vector3(-MathF.Sin(bearing), MathF.Cos(bearing), 0f);
            float rObj = new Vector2(o.Center.X, o.Center.Y).Length();
            float halfRadial = Support(o, radialDir), halfTangential = Support(o, tangentDir);

            // The height the pads meet it at: uniform over its own height, base to top, within what the arm reaches (HeightRange: above the
            // model's floor or its stand's top, below its top, never down through it; a plank from just under it to its top).
            if (!HeightRange(oi, out float zLo, out float zHi)) return Miss(0);
            float h = (float)Draw.Uniform(ref _rng, zLo, zHi);
            bool plank = o.Role == SceneRole.Plank, under = plank && h < o.Min.Z - 0.008f;

            // The pads' path. Mostly an arc round the base at one radius, from either side: the arm's reach is a thin shell round
            // the base, and an arc keeps it there as a hand sweeping sideways does. Some aims come straight out from the base
            // instead, a short way (the shell is about 6 cm deep at one pitch). Its height changes along it by a drawn slope.
            var tools = new List<Vector3>();
            // A plank is met from the side or from below: its approach may rise into it (up to 20 degrees).
            float side = Draw.Chance(ref _rng, 0.5) ? 1f : -1f, slope = MathF.Tan((float)(Draw.Uniform(ref _rng, -12, plank ? 20 : 8) * deg)), approach;
            // Beside the robot (more than 60 degrees round), the arm reaches no further round than about 95 degrees (measured
            // 2026-11-23 over the IK: nothing at 100), so an arc comes from the front; a pass would have to go on past the object,
            // so beyond 80 degrees it is an aim.
            if (MathF.Abs(bearing) > 60f * MathF.PI / 180f) side = MathF.Sign(bearing);
            if (kind == ScriptMoveKind.Pass && MathF.Abs(bearing) > 80f * MathF.PI / 180f) kind = ScriptMoveKind.Aim;
            if (kind == ScriptMoveKind.Pass || !Draw.Chance(ref _rng, RadialShare))
            {
                float rLine;
                if (kind == ScriptMoveKind.Aim) rLine = rObj + (float)Draw.Uniform(ref _rng, -0.6, 0.6) * halfRadial;
                else
                {
                    float out_ = Draw.Chance(ref _rng, 0.5) ? 1f : -1f;
                    // Through a movable object, or beside it; a fixture only grazed or passed (it does not give way), but under a plank anywhere.
                    rLine = rObj + out_ * (float)(o.Movable || under ? Draw.Uniform(ref _rng, 0, halfRadial + 0.07) : Draw.Uniform(ref _rng, halfRadial - 0.010, halfRadial + 0.07));
                }

                if (rLine < 0.10f) return Miss(1);
                float s0 = -(halfTangential + 0.32f), s1 = kind == ScriptMoveKind.Aim ? 0f : halfTangential + 0.12f;
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
            // The wrist pitched for the height (PitchAt, as a layout-blind goal there is): pointing down near the table, near level high up a
            // tall object, so the pads lead into it rather than the wrist above them.
            float pitch = PitchAt(radius, tools[meet].Z) + (float)(Draw.Uniform(ref _rng, -8, 8) * deg);
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
            Vector3 goal;
            if (kind == ScriptMoveKind.Aim)
            {
                // The end: where the gap first comes to the drawn offset (beyond what the model measures clear, extrapolated back
                // along the approach); a part of a step for the rest.
                float deepest = DeepestGap(oi), lo = Math.Max(_options.OffsetMinMeters, -(deepest - 0.0005f)), hi = _options.OffsetMaxMeters;
                // A thin object (a plank, a rod, a small tin) is measured no deeper than its own half thickness: the range short of it shrinks
                // in proportion (to 15 mm at least), so it is touched as often as a thick one (docs/adr/0038 "M, motion": 4 of 62 aims at planks
                // touched them).
                if (lo > _options.OffsetMinMeters && _options.OffsetMinMeters < 0f) hi = MathF.Max(MathF.Min(hi, 0.015f), hi * lo / _options.OffsetMinMeters);
                offset = (float)Draw.Uniform(ref _rng, lo, hi);
                if (!AimIndex(gaps, offset, out end, out f)) return Miss(2);
                goal = Vector3.Lerp(wrists[end - 1], wrists[end], f);
                near = _options.NearStartShare > 0 && Draw.Chance(ref _rng, _options.NearStartShare) && end >= 3;
                if (near)
                {
                    float close = (float)Draw.Uniform(ref _rng, 0.004, 0.020);
                    stage = end - 2;
                    while (stage > 0 && gaps[stage] < close) stage--;
                }
                else
                {
                    // Where it starts: as far back along its line from its end as a free move's goal is from the hand (one law for both,
                    // docs/adr/0038 "M, motion"; aims came 4-12 cm), and clear of the object by 6 mm.
                    stage = Back(wrists, end - 1, goal, ApproachLength());
                    while (stage > 0 && gaps[stage] < 0.006f) stage--;
                }

                if (gaps[stage] < 0.003f || end - stage < 2) return Miss(3);
            }
            else
            {
                // The closest point of the sweep; it starts as far before it as a free move's goal is from the hand (at least 3 cm), and
                // goes on past the object.
                int closest = 0;
                for (int i = 1; i < gaps.Count; i++) if (gaps[i] < gaps[closest]) closest = i;
                stage = Back(wrists, closest, wrists[closest], MathF.Max(0.03f, ApproachLength()));
                if (Vector3.Distance(wrists[stage], wrists[closest]) < 0.03f) return Miss(6);
                while (stage > 0 && gaps[stage] < 0.006f) stage--;
                int exit = closest;
                while (exit < gaps.Count && gaps[exit] < Math.Max(gaps[closest], 0f) + 0.010f) exit++;
                if (exit >= gaps.Count || gaps[stage] < 0.003f) return Miss(6); // the sweep does not come out past the object
                end = Math.Min(gaps.Count - 1, exit + (int)MathF.Round((float)Draw.Uniform(ref _rng, 0.02, 0.06) / LineStepMeters));
                goal = wrists[end];
                // What it was aimed at: its line's closest approach. The hand goes across as a free move goes (straight, or curved, below),
                // not round the arc, a centimetre or two inside it at the object; how close it came is the move's gap, measured.
                offset = gaps[closest];
            }

            // An approach to what a stand holds, or to a plank, keeps off what holds it up (docs/adr/0038 "M, motion": meeting the stand first
            // made the stands the most touched objects): every point of its line from where it starts to where it ends, or else no aim here.
            ArmCollisionModel holders = HolderWalls(oi);
            if (!ReferenceEquals(holders, _model))
            {
                for (int i = stage; i <= end; i++) if (!ArmCollision.IsValid(_profile, holders, poses[i])) return Miss(5);
            }

            // The hand comes to where the move starts at a pitch drawn as a low layout-blind goal's is (docs/adr/0038 "M, motion": aims and
            // passes kept the pitch the move before had set, 1.5 degrees of turn at the median against 14 for free moves), and the move
            // turns the wrist to the line's on the way. The pads start where the line starts (the wrist placed for the pitch it comes at),
            // and the turn moves them off the straight way by a few millimetres at most (to first order the pads go straight when the
            // wrist and its pitch move together). Half the turn, then none, when the start is out of reach or within 6 mm of the object.
            float startPitch = pitch;
            Vector3 startWrist = wrists[stage];
            ArmPose startPose = poses[stage];
            // As a layout-blind goal's is: for a height drawn from the blind goals' bands (PitchAt, the move before an aim being a free one), or
            // a low goal's when the blind goals are drawn as before; so an aim turns the wrist on its way as much as a free move does.
            float turn = (_options.Reach.ToolBands != null
                ? PitchAt(new Vector2(tools[first + stage].X, tools[first + stage].Y).Length(), Draw.Band(ref _rng, _options.Reach.ToolBands)) + (float)(Draw.Uniform(ref _rng, -8, 8) * deg)
                : (float)(Draw.Uniform(ref _rng, _options.Reach.LowPitchLowDegrees, _options.Reach.LowPitchHighDegrees) * deg)) - pitch;
            // A free move's turn grows with its length (free transports: about 13 degrees per 10 cm at the median); a large turn over a
            // short approach broke the caps, and the moves they stretched were slower than free ones.
            float most = (float)(_options.TurnDegreesPerMeter * deg) * Vector3.Distance(goal, wrists[stage]);
            turn = Math.Clamp(turn, -most, most);
            for (int t = 0; t < 2 && MathF.Abs(turn) > 1e-3f; t++, turn /= 2f)
            {
                float p = Math.Clamp(pitch + turn, -89f * MathF.PI / 180f, -8f * MathF.PI / 180f);
                Vector3 there = WristFor(tools[first + stage], p);
                if (!TrySolve(poses[stage], there, p, out ArmPose at) || !_envelope.Contains(at) || !ArmCollision.IsValid(_profile, _model, at) || Gap(oi, at, gripper) < 0.006f) continue;
                (startPitch, startWrist, startPose) = (p, there, at);
                break;
            }

            Vector3 u = goal - startWrist;
            u = u.Length() > 1e-6f ? Vector3.Normalize(u) : radialDir;
            HandSnapshot before = _hand.Save();
            int drafts = _drafts.Count;
            // To where it starts, clear of its object's box by what that point leaves of it (less 4 mm, the model's margin kept; at most the
            // quiet margin, or a start far back would keep the hand out of a wide ring round the object), blind to the others.
            double s = 0, e = 0;
            bool staged = false;
            for (int t = 0; t < 2 && !staged; t++)
            {
                // At the pitch drawn; when that finds no way there, at the line's own (no turn on the way).
                if (t == 1)
                {
                    if (startPitch == pitch) break;
                    (startPitch, startWrist, startPose) = (pitch, wrists[stage], poses[stage]);
                    u = goal - startWrist;
                    u = u.Length() > 1e-6f ? Vector3.Normalize(u) : radialDir;
                }

                float boxGap = (_ladders[oi] ??= new GapLadder(_profile, _model, o)).SignedGap(startPose);
                ArmCollisionModel staging = SafetyScript.WithObjects(holders, new[] { o.Grown(MathF.Min(boxGap - 0.004f - _model.Margin, _options.QuietMarginMeters)) }, 0f);
                staged = _hand.MixedTo(startWrist, startPitch, null, Slowness(), HandModel.Tolerance, staging, out s, out e, overTheTop: true);
            }

            if (!staged)
            {
                Undo(before, drafts);
                return Miss(4);
            }

            Add(ScriptMoveKind.Free, -1, float.NaN, s, e, part: MovePart.Staging);
            if (!near) AfterMove(u, pullBack: false); // the law's draws (a pause, never a pull back: the approach starts here)

            // The aim or pass itself, made as a free move is made (docs/adr/0038 "M, motion": aims and passes followed their line in
            // short segments to a tighter stop, and a model of the path could tell them by it): one goal, the same endpoint error,
            // undershoot and corrections to the same tolerance, sometimes curved or in two pushes, the roll and the gripper as they go.
            // With an accident (docs/adr/0038 "M, motion"), first with it; when no try makes it, the move as it would have been without one.
            bool withAccident = accident.HasValue && _hand.MixedTo(goal, pitch, null, Slowness(), HandModel.Tolerance, holders, out s, out e, accident: accident);
            double onset = _hand.AccidentOnset, recovered = _hand.AccidentEnd;
            if (!withAccident && !_hand.MixedTo(goal, pitch, null, Slowness(), HandModel.Tolerance, holders, out s, out e))
            {
                Undo(before, drafts);
                return Miss(5);
            }

            _objectMoves++;
            int baseMove = Add(kind, oi, offset, s, e, approach, MovePart.Approach);
            bool pushed = false;
            if (withAccident)
            {
                Add(accident!.Value, oi, offset, onset, recovered, approach, MovePart.Approach, baseMove);
                // Whether the accident went into its object by the model: then, movable, it may be anywhere afterwards, as when aimed into.
                int k1 = Math.Min(_hand.Poses.Count - 1, (int)Math.Ceiling(recovered / HandModel.TickSeconds));
                for (int k = Math.Max(0, (int)Math.Floor(onset / HandModel.TickSeconds)); k <= k1 && !pushed; k++) pushed = Gap(oi, _hand.Poses[k], _hand.Commands[k].Gripper) < 0f;
            }

            if ((offset < 0 || pushed) && o.Movable)
            {
                // Pushed into, it may be anywhere afterwards: no later move aims at it, and the tail's goals keep away from it.
                _moved.Add(oi);
                var c = new Vector2(o.Center.X, o.Center.Y);
                _avoid.Add(c);
                _avoid.Add(c + 0.10f * Vector2.Normalize(new Vector2(u.X, u.Y) + new Vector2(1e-6f, 0f)));
            }

            // Then the law every move follows; into a fixture, the hand always pulls back out (a person does; and the blind moves after
            // keep out of fixtures).
            bool intoFixture = !o.Movable && offset < 0.005f;
            float deep = 0f;
            if (intoFixture)
            {
                // How deep the arm went into its box (the hand's own error included): the pull back goes that far and 1.5 cm more.
                int k1 = Math.Min(_hand.Poses.Count - 1, (int)Math.Ceiling(e / HandModel.TickSeconds));
                for (int k = Math.Max(0, (int)Math.Floor(s / HandModel.TickSeconds)); k <= k1; k++) deep = MathF.Max(deep, -_ladders[oi]!.SignedGap(_hand.Poses[k]));
            }

            if (!AfterMove(u, pullBack: true, forced: intoFixture, atLeast: deep + 0.015f))
            {
                Undo(before, drafts);
                _objectMoves--;
                return Miss(5);
            }

            if (accident.HasValue) Tally(accident.Value, withAccident);
            return true;
        }

        /// <summary>
        /// How far back an aim or a pass starts, m: as far as a layout-blind part that is not strokes goes (one law, docs/adr/0038 "M,
        /// motion"): small adjustments' distance with their share of those parts (<see cref="MixedOptions.BlindTransportShare"/>,
        /// <see cref="MixedOptions.BlindReverseShare"/>), else a goal's (<see cref="Draw.ReachDistance"/>).
        /// </summary>
        private float ApproachLength()
        {
            double adjust = 1.0 - _options.BlindTransportShare - _options.BlindReverseShare, parts = 1.0 - _options.BlindReverseShare;
            double share = _options.ShortApproachShare >= 0 ? _options.ShortApproachShare : parts > 0 ? adjust / parts : 0;
            return Draw.Chance(ref _rng, share) ? Draw.AdjustDistance(ref _rng) : Draw.ReachDistance(ref _rng);
        }

        /// <summary>The last index at or before <paramref name="from"/>, walking back along a line, whose wrist is at least <paramref name="length"/> from <paramref name="target"/>; 0 when none is.</summary>
        private static int Back(List<Vector3> wrists, int from, Vector3 target, float length)
        {
            int i = Math.Max(0, Math.Min(from, wrists.Count - 1));
            while (i > 0 && Vector3.Distance(wrists[i], target) < length) i--;
            return i;
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

        /// <summary>
        /// The wrist pitch for the tool point (the pads' midpoint) <paramref name="radius"/> from the base axis at <paramref name="z"/> (base frame),
        /// <paramref name="table"/> the table's top, radians (docs/adr/0038 "M, motion", the 2026-11-23 review: the approach follows the
        /// height): near the table the reach shell's (<see cref="PitchFor"/>: pointing down close in, toward level farther out); higher up toward
        /// level, -35 degrees 10 cm above the table, -20 at 20 cm, -10 at 30 cm, blended from 5 to 15 cm up. One rule for aims, passes and
        /// layout-blind goals, so the wrist's pitch says no more than the height does.
        /// </summary>
        internal static float PitchAt(float radius, float z, float table)
        {
            float above = z - table;
            float low = PitchFor(radius);
            float high = Math.Clamp(-35f + (above - 0.10f) * 150f, -40f, -5f) * MathF.PI / 180f;
            float w = Math.Clamp((above - 0.05f) / 0.10f, 0f, 1f);
            return low + (high - low) * w;
        }

        private float PitchAt(float radius, float z) => PitchAt(radius, z, _model.FloorZ);

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

        /// <summary>The wrist that puts the pads' midpoint at <paramref name="tool"/> with the wrist at <paramref name="pitch"/> (roll 0), as <see cref="Seed"/> places a line's.</summary>
        private static Vector3 WristFor(Vector3 tool, float pitch)
        {
            float lh = ToolInGripper.X * MathF.Cos(pitch) - ToolInGripper.Z * MathF.Sin(pitch);
            float lv = ToolInGripper.X * MathF.Sin(pitch) + ToolInGripper.Z * MathF.Cos(pitch);
            var radial = new Vector2(tool.X, tool.Y);
            radial = radial.Length() > 1e-4f ? Vector2.Normalize(radial) : Vector2.UnitX;
            return new Vector3(tool.X - lh * radial.X, tool.Y - lh * radial.Y, tool.Z - lv);
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
