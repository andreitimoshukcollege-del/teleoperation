using System;
using System.Collections.Generic;
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
        public double ObjectShare { get; set; } = 0.45;

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
        public double PauseShare { get; set; } = 0.12;

        public double PauseMinSeconds { get; set; } = 0.10;
        public double PauseMaxSeconds { get; set; } = 0.50;

        /// <summary>The share of object-directed moves followed by a pull back the way they came (a reverse).</summary>
        public double ReverseAfterShare { get; set; } = 0.35;

        /// <summary>A layout-blind part is a move to a goal with this share, back-and-forth strokes with <see cref="BlindReverseShare"/>, else small adjustments.</summary>
        public double BlindTransportShare { get; set; } = 0.55;

        public double BlindReverseShare { get; set; } = 0.25;

        /// <summary>Where layout-blind goals are drawn.</summary>
        public BlindReach Reach { get; set; } = new BlindReach();

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
        /// the wrist pitch about twice as fast as people at p90, the elbow 1.4 times): pitch and roll changes 0.55 and 0.7 of the
        /// hand model's, followed over the whole of each submovement, and every submovement 1.25 times as long. Chosen from this
        /// script's dry runs against R's targets (docs/adr/0038, "M, motion").
        /// </summary>
        public static HandStyle DefaultStyle() => new HandStyle { PitchScale = 0.55, RollScale = 0.7, PitchTime = 1.0, Slow = 1.25 };

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
        private readonly List<Vector2> _avoid = new List<Vector2>();
        private SeededRng _rng;
        private double _lastObjectEnd = double.NegativeInfinity;

        /// <summary>Why object-directed moves were not planned, for a report: too low to meet, no line, no end at the offset, no start, the move there, the approach, a pass's sweep.</summary>
        private readonly int[] _objectMisses = new int[7];

        private int _objectTries, _objectMoves;

        /// <summary>The object-directed moves tried and planned, and why the rest were not.</summary>
        internal string ObjectReport() => FormattableString.Invariant(
            $"object moves {_objectMoves} of {_objectTries} tried (missed: too low {_objectMisses[0]}, no line {_objectMisses[1]}, no end {_objectMisses[2]}, no start {_objectMisses[3]}, move there {_objectMisses[4]}, approach {_objectMisses[5]}, sweep {_objectMisses[6]})");

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
        }

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

            if (!ArmCollision.IsValid(profile, SafetyScript.WithObjects(model, objects, SafetyScript.ClearanceMeters), current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {SafetyScript.ClearanceMeters * 100:0} cm of an object";
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
            return $"ticks refused: IK {r[0]}, envelope or model {r[1]}, jump {r[2]}, quiet margin {r[3]}, caps {r[4]}; {ObjectReport()}";
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
                if (!_hand.MixedTo(goal, pitch, null, Slowness(), 0.010, _model, out double s, out double e)) continue;
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
                    out ScriptMoveKind kind, out start, out end))
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
                Vector3 c = _objects[i].Center;
                float r = new Vector2(c.X, c.Y).Length();
                if (r - Footprint(_objects[i]) < 0.40f) targets.Add(i);
            }

            if (targets.Count == 0) return false;
            for (int attempt = 0; attempt < 10; attempt++)
            {
                int oi = targets[(int)(_rng.NextDouble() * targets.Count) % targets.Count];
                ScriptMoveKind kind = Draw.Chance(ref _rng, _options.PassShare) ? ScriptMoveKind.Pass : ScriptMoveKind.Aim;
                if (TryObjectMove(oi, kind)) return true;
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
            float alpha = kind == ScriptMoveKind.Aim
                ? bearing + (float)(Draw.Uniform(ref _rng, -100, 100) * deg)
                : bearing + (Draw.Chance(ref _rng, 0.5) ? 1f : -1f) * (float)((90 + Draw.Uniform(ref _rng, -35, 35)) * deg);
            float elevation = (float)(Draw.Uniform(ref _rng, -15, 10) * deg);
            var u = new Vector3(MathF.Cos(alpha) * MathF.Cos(elevation), MathF.Sin(alpha) * MathF.Cos(elevation), MathF.Sin(elevation));
            var across = new Vector3(-MathF.Sin(alpha), MathF.Cos(alpha), 0f);
            float halfAcross = Support(o, across), halfAlong = Support(o, u);

            // The height the pads meet it at: from just above the model's floor to just below its top (a touch into its side, never
            // down through its top, where a shape may cover a hollow).
            float zLo = o.Min.Z + LowestContactMeters, zHi = o.Max.Z - (o.Shape != null ? ObjectShape.TopMarginMeters + 0.003f : 0.004f);
            if (zHi < zLo + 0.003f) return Miss(0);
            float h = (float)Draw.Uniform(ref _rng, zLo, zHi);
            float pitch = (float)((h - o.Min.Z < 0.06f ? Draw.Uniform(ref _rng, -88, -55) : Draw.Uniform(ref _rng, -80, -15)) * deg);
            float lateral;
            if (kind == ScriptMoveKind.Aim) lateral = (float)Draw.Uniform(ref _rng, -0.6, 0.6) * halfAcross;
            else
            {
                float side = Draw.Chance(ref _rng, 0.5) ? 1f : -1f;
                // Through a movable object, or beside it; a fixture only grazed or passed (it does not give way).
                lateral = side * (float)(o.Movable ? Draw.Uniform(ref _rng, 0, halfAcross + 0.07) : Draw.Uniform(ref _rng, halfAcross - 0.010, halfAcross + 0.07));
            }

            Vector3 middle = new Vector3(o.Center.X, o.Center.Y, h) + lateral * across;
            float s0 = -(halfAlong + 0.24f), s1 = kind == ScriptMoveKind.Aim ? 0f : halfAlong + 0.12f;
            float gripper = _hand.GripperEnd;
            if (!Line(oi, middle, u, s0, s1, pitch, gripper, out List<Vector3> wrists, out List<ArmPose> poses, out List<float> gaps)) return Miss(1);

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
                end = exit + (int)MathF.Round((float)Draw.Uniform(ref _rng, 0.02, 0.06) / LineStepMeters);
                if (end >= gaps.Count || gaps[stage] < 0.003f) return Miss(6);
            }

            var path = new List<Vector3>(end - stage + 1);
            for (int i = stage; i < end; i++) path.Add(wrists[i]);
            path.Add(kind == ScriptMoveKind.Aim ? Vector3.Lerp(wrists[end - 1], wrists[end], f) : wrists[end]);

            HandSnapshot before = _hand.Save();
            int drafts = _drafts.Count;
            // To where it starts, clear of its object's box by what that point leaves of it (less 4 mm, the model's margin kept), blind
            // to the others.
            float boxGap = (_ladders[oi] ??= new GapLadder(_profile, _model, o)).SignedGap(poses[stage]);
            ArmCollisionModel staging = SafetyScript.WithObjects(_model, new[] { o.Grown(boxGap - 0.004f - _model.Margin) }, 0f);
            if (!_hand.MixedTo(wrists[stage], pitch, null, Slowness(), 0.004, staging, out double s, out double e))
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
            float approach = (alpha - bearing) * 180f / MathF.PI;
            while (approach > 180f) approach -= 360f;
            while (approach < -180f) approach += 360f;
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

            if (Draw.Chance(ref _rng, _options.ReverseAfterShare))
            {
                Vector3 back = _hand.TargetEnd - u * (float)Draw.Uniform(ref _rng, 0.03, 0.08) + new Vector3(0f, 0f, (float)Draw.Uniform(ref _rng, 0.01, 0.04));
                if (_hand.MixedTo(back, null, null, Slowness(), 0.008, _model, out s, out e))
                {
                    Add(ScriptMoveKind.Reverse, -1, float.NaN, s, e);
                    _lastObjectEnd = Math.Max(_lastObjectEnd, e);
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
        /// The aim's line: the pads' midpoint from <paramref name="middle"/> + <paramref name="s0"/>·<paramref name="u"/> to
        /// <paramref name="middle"/> + <paramref name="s1"/>·<paramref name="u"/>, the wrist pitched at <paramref name="pitch"/>, solved
        /// by the headset's IK point to point; from its first pose in reach, in the envelope and free of the robot's own model, to the
        /// first that is not (or flips branch). The wrist targets, their poses, and the arm's gap to the object at each.
        /// </summary>
        private bool Line(int oi, Vector3 middle, Vector3 u, float s0, float s1, float pitch, float gripper, out List<Vector3> wrists, out List<ArmPose> poses,
            out List<float> gaps)
        {
            int n = (int)MathF.Ceiling((s1 - s0) / LineStepMeters) + 1;
            wrists = new List<Vector3>(n);
            poses = new List<ArmPose>(n);
            gaps = new List<float>(n);
            float lh = ToolInGripper.X * MathF.Cos(pitch) - ToolInGripper.Z * MathF.Sin(pitch);
            float lv = ToolInGripper.X * MathF.Sin(pitch) + ToolInGripper.Z * MathF.Cos(pitch);
            ArmPose at = _model.Home;
            bool started = false;
            for (int i = 0; i < n; i++)
            {
                Vector3 tool = middle + (s0 + i * LineStepMeters) * u;
                var radial = new Vector2(tool.X, tool.Y);
                if (radial.Length() < 0.05f)
                {
                    if (started) break;
                    continue; // over the base: the line starts beyond it
                }

                radial = Vector2.Normalize(radial);
                var wrist = new Vector3(tool.X - lh * radial.X, tool.Y - lh * radial.Y, tool.Z - lv);
                bool ok = TrySolve(at, wrist, pitch, out ArmPose pose) && _envelope.Contains(pose) && ArmCollision.IsValid(_profile, _model, pose)
                          && (!started || at.MaxJointDelta(pose) <= 0.15f);
                if (!ok)
                {
                    if (started) break;
                    continue;
                }

                started = true;
                at = pose;
                wrists.Add(wrist);
                poses.Add(pose);
                gaps.Add(Gap(oi, pose, gripper));
            }

            return wrists.Count >= 3;
        }

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
