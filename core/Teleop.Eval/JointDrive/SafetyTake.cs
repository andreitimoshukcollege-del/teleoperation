using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>One event a safety take planned (docs/adr/0036 amendment 5), its times in plan seconds; its kind as the take's <c>SE</c> record names it.</summary>
    public readonly struct SafetyEvent
    {
        /// <summary>1 for the take's first event.</summary>
        public readonly int Index;
        public readonly ScriptEventKind Kind;

        /// <summary>The scene id of the object it aims at.</summary>
        public readonly byte ObjectId;

        /// <summary>The arm leaves free movement for the object: the move to where the final approach starts begins.</summary>
        public readonly double ApproachStart;

        /// <summary>The final approach begins.</summary>
        public readonly double FinalStart;

        /// <summary>The planned first contact (a touch, a knock), or the closest approach (a stop).</summary>
        public readonly double Contact;

        /// <summary>A stop's stillness or a touch's moment in contact ends; a knock's lift away ends.</summary>
        public readonly double End;

        /// <summary>Free movement takes over again: the next part of the take may start.</summary>
        public readonly double Clear;

        public SafetyEvent(int index, ScriptEventKind kind, byte objectId, double approachStart, double finalStart, double contact, double end, double clear)
        {
            Index = index;
            Kind = kind;
            ObjectId = objectId;
            ApproachStart = approachStart;
            FinalStart = finalStart;
            Contact = contact;
            End = end;
            Clear = clear;
        }

        public override string ToString() => FormattableString.Invariant(
            $"event {Index}: {ScriptEventCodec.Name(Kind)} object {ObjectId}, approach {ApproachStart:0.00} s, final {FinalStart:0.00} s, contact {Contact:0.00} s, end {End:0.00} s, clear {Clear:0.00} s");
    }

    /// <summary>
    /// What a collide_with_obstacle take needs to go on past a knock (docs/adr/0036 amendment 5). A knock sends its object
    /// wherever physics takes it, so the next event cannot be planned before the objects have settled: the plan goes on with
    /// free movement away from the knocked object (the runway), and joint-drive, once it sees every object at rest, asks
    /// <see cref="Continue"/> for a plan that is the same up to a point of the runway and plans the next event from there against
    /// the objects as they now stand. Without that, the runway ends the take.
    /// </summary>
    public sealed class SafetyContinuation
    {
        private readonly SafetyTake _take;
        private readonly List<HandSnapshot> _runway;

        internal SafetyContinuation(SafetyTake take, List<HandSnapshot> runway, SeededRng rng, SafetyEvent[] events, byte[] used, double knockEnd, byte knocked,
            Vector2[] away)
        {
            _take = take;
            _runway = runway;
            Rng = rng;
            Events = events;
            Used = used;
            KnockEnd = knockEnd;
            Knocked = knocked;
            Away = away;
        }

        /// <summary>When the knock's swipe and lift end, plan seconds: from then on the objects are watched for rest.</summary>
        public double KnockEnd { get; }

        /// <summary>The scene id of the object knocked.</summary>
        public byte Knocked { get; }

        /// <summary>The index the next event would have.</summary>
        public int NextIndex => Events.Length + 1;

        /// <summary>Which way the swipe sent the knocked object: a horizontal unit vector, arm base frame.</summary>
        public Vector2 KnockWay => (Away[1] - Away[0]) / SafetyTake.AwayMeters;

        internal SeededRng Rng { get; }
        internal SafetyEvent[] Events { get; }
        internal byte[] Used { get; }
        internal Vector2[] Away { get; }

        /// <summary>The first point of the runway at or after <paramref name="notBefore"/> (plan seconds) the plan can go on from another way, or NaN when the runway ends before.</summary>
        public double SwitchAfter(double notBefore)
        {
            foreach (HandSnapshot s in _runway)
            {
                double at = _take.PlanTime(s.Free);
                if (at >= notBefore) return at;
            }

            return double.NaN;
        }

        internal HandSnapshot? SnapshotAt(double planTime)
        {
            foreach (HandSnapshot s in _runway)
            {
                if (Math.Abs(_take.PlanTime(s.Free) - planTime) < 1e-9) return s;
            }

            return null;
        }

        /// <summary>
        /// A plan identical to the current one before <paramref name="switchAt"/> (a <see cref="SwitchAfter"/> point), from there
        /// on planning the next event against <paramref name="objects"/> (where they now are, at rest), and after it the rest of
        /// the take as at the start; or, when no approach works, ending the take after a tail. Deterministic: the same objects and
        /// point give the same plan. One call at a time.
        /// </summary>
        public DrivePlan Continue(IReadOnlyList<SafetyObject> objects, double switchAt, out string outcome) => _take.Continue(this, objects, switchAt, out outcome);
    }

    /// <summary>
    /// A safety take as joint-drive plans it since 2026-11-21 (docs/adr/0036 amendment 5): the arm already moving as it begins
    /// (no reaction time), and in the three event categories 1-3 events (<see cref="SafetyOptions.MinEvents"/>), each after free
    /// movement of a random length, so events happen anywhere in the take, never only at its end, and the take ends after a tail
    /// of free movement, never inside an event. Each event aims at an object as the category's rules choose one
    /// (<see cref="SafetyScript.TryAnyEvent"/>), preferring one not aimed at yet. A stop or a touch leaves the world as it was,
    /// so all of them are planned at the start; a knock does not, so the event after one is planned once joint-drive has seen
    /// the objects settle (<see cref="SafetyContinuation"/>). Free movement between events and after the last keeps
    /// <see cref="SafetyScript.ApproachClearanceMeters"/> from every object (after a knock, from every other object, and its
    /// goals away from the knocked one). reach_no_obstacle has no events: free movement for the take's length.
    /// </summary>
    internal sealed class SafetyTake
    {
        /// <summary>The shortest an event lasts from leaving free movement to free movement again, about, seconds: an event that cannot fit is not planned.</summary>
        public const double MinEventSeconds = 3.0;

        /// <summary>
        /// Free movement after a knock until the next event is planned, at most, seconds: long enough for the objects to settle and
        /// the next event to be planned while the arm keeps moving (joint-drive's budgets). It ends the take when nothing replaces it,
        /// and never runs past <see cref="SafetyOptions.MaxTakeSeconds"/>.
        /// </summary>
        public const double RunwaySeconds = 12.0;

        /// <summary>A runway shorter than this, seconds (the take nearly at its longest), leaves no time to settle and plan: the take ends after a tail instead.</summary>
        public const double MinRunwaySeconds = 7.0;

        /// <summary>After a knock, free goals are drawn this far from where the object stood and from where it was sent, m.</summary>
        public const float AwayMeters = 0.10f;

        private readonly DriveScript _script;
        private readonly RobotArmProfile _profile;
        private readonly ArmCollisionModel _model;
        private readonly DriveEnvelope _envelope;
        private readonly SafetyOptions _options;
        private readonly float _speed;
        private readonly List<DriveLeg> _prefix; // the legs before the operator starts (the move home, when the arm was not clear)
        private readonly double _start; // plan seconds at the operator's first tick
        private readonly ArmPose _startPose;
        private readonly HumanOperator _hand;
        private readonly int _drawn;

        private SafetyTake(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, SafetyOptions options,
            float speed, List<DriveLeg> prefix, double start, in ArmPose startPose, HumanOperator hand, int drawn)
        {
            _script = script;
            _profile = profile;
            _model = model;
            _envelope = envelope;
            _options = options;
            _speed = speed;
            _prefix = new List<DriveLeg>(prefix);
            _start = start;
            _startPose = startPose;
            _hand = hand;
            _drawn = drawn;
        }

        internal double PlanTime(double handTime) => _start + handTime;

        /// <summary>
        /// Plans the take from <paramref name="current"/> at <paramref name="t"/> (after <paramref name="prefix"/>, the plan's legs so
        /// far), reach_no_obstacle until <paramref name="end"/>. False, with the reason, when the arm is not clear of the objects,
        /// no free movement exists, or (in an event category) no object can be approached for the first event.
        /// </summary>
        internal static bool TryPlan(DriveScript script, in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope,
            IReadOnlyList<SafetyObject> objects, float speed, in ArmPose current, double t, double end, ref SeededRng rng, List<DriveLeg> prefix,
            SafetyOptions options, out DrivePlan? plan, out string? error)
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

            ArmCollisionModel clear = SafetyScript.WithObjects(model, objects, SafetyScript.ClearanceMeters);
            if (!ArmCollision.IsValid(profile, clear, current))
            {
                error = $"the arm at {DrivePlan.Describe(current)} is already within {SafetyScript.ClearanceMeters * 100:0} cm of an object";
                return false;
            }

            var hand = new HumanOperator(profile, envelope, current, rng.NextUInt64());
            hand.EngageOnTheMove();
            if (script == DriveScript.ReachNoObstacle)
            {
                var parts = new List<double>();
                if (!hand.FreeUntil(end - t, clear, parts) || parts.Count == 0)
                {
                    error = $"no free movement from {DrivePlan.Describe(current)} keeps clear of the objects";
                    return false;
                }

                // The take ends partway (50-80%) through its last move, the arm still moving (docs/adr/0036 amendment 3).
                var reach = new SafetyTake(script, profile, model, envelope, options, speed, prefix, t, current, hand, 0);
                double from = parts[parts.Count - 1], to = Math.Max(from, hand.Timeline.MotionEnd);
                plan = reach.Build(new List<SafetyEvent>(), t + from + (0.5 + 0.3 * rng.NextDouble()) * (to - from), null);
                return true;
            }

            if (objects.Count == 0)
            {
                error = "the scene has no objects to approach";
                return false;
            }

            int drawn = Math.Min(options.MaxEvents, options.MinEvents + (int)(rng.NextDouble() * (options.MaxEvents - options.MinEvents + 1)));
            double lead = Draw.Uniform(ref rng, options.LeadMinSeconds, options.LeadMaxSeconds);
            if (!hand.FreeUntil(lead, clear))
            {
                error = $"no free movement from {DrivePlan.Describe(current)} keeps clear of the objects";
                return false;
            }

            var take = new SafetyTake(script, profile, model, envelope, options, speed, prefix, t, current, hand, drawn);
            if (!take.PlanEvents(1, objects, ref rng, new List<SafetyEvent>(), new HashSet<byte>(), out plan))
            {
                error = $"no object could be approached for {SafetyScript.Category(script)} from {DrivePlan.Describe(current)}";
                return false;
            }

            return true;
        }

        /// <summary>
        /// Plans events <paramref name="first"/> to the drawn number from where the operator is, against <paramref name="objects"/>,
        /// each after free movement between, then the end: a runway after a knock with more to come, else a tail. False only when
        /// the take has no event at all (the first finds no approach).
        /// </summary>
        private bool PlanEvents(int first, IReadOnlyList<SafetyObject> objects, ref SeededRng rng, List<SafetyEvent> events, HashSet<byte> used,
            out DrivePlan? plan)
        {
            plan = null;
            ArmCollisionModel near = SafetyScript.WithObjects(_model, objects, SafetyScript.ApproachClearanceMeters);
            Func<int, ArmCollisionModel> after = _script == DriveScript.CollideWithObstacle
                ? oi => SafetyScript.WithObjects(_model, Without(objects, oi), SafetyScript.ApproachClearanceMeters)
                : _ => near;
            HandSnapshot beforeBetween = _hand.Save();
            int knocked = -1;
            Vector3 way = default;
            for (int i = first; i <= _drawn; i++)
            {
                if (!SafetyScript.TryAnyEvent(_script, _profile, _model, _envelope, objects, _hand, ref rng, _options, used, after, true,
                        out SafetyScript.PlannedEvent p))
                {
                    _hand.Restore(beforeBetween); // and the free movement before it with it
                    if (events.Count == 0) return false;
                    break;
                }

                if (events.Count > 0 && p.Clear + _options.TailMinSeconds > _options.MaxTakeSeconds)
                {
                    _hand.Restore(beforeBetween); // the take would run too long
                    break;
                }

                events.Add(new SafetyEvent(i, Kind(p.Kind), objects[p.Object].Id, PlanTime(p.ApproachStart), PlanTime(p.FinalStart),
                    PlanTime(p.Contact), PlanTime(p.End), PlanTime(p.Clear)));
                used.Add(objects[p.Object].Id);
                knocked = p.Kind == HandEvent.Through ? p.Object : -1;
                way = p.Way;
                if (i == _drawn) break;

                if (knocked >= 0)
                {
                    // The world is not what was planned around until the objects settle: the next event is planned then.
                    if (TryRunway(objects, knocked, way, rng, events, used, out plan)) return true;
                    break;
                }

                beforeBetween = _hand.Save();
                double between = Draw.Uniform(ref rng, _options.BetweenMinSeconds, _options.BetweenMaxSeconds);
                if (_hand.Free + between + MinEventSeconds + _options.TailMinSeconds > _options.MaxTakeSeconds) break;
                if (!_hand.FreeUntil(_hand.Free + between, near))
                {
                    _hand.Restore(beforeBetween);
                    break;
                }
            }

            plan = Tail(objects, knocked, way, ref rng, events);
            return true;
        }

        /// <summary>After a knock with more events to come: free movement away from the knocked object, a point to go on from after each part, and the take's end at its end.</summary>
        private bool TryRunway(IReadOnlyList<SafetyObject> objects, int knocked, Vector3 way, SeededRng rng, List<SafetyEvent> events, HashSet<byte> used,
            out DrivePlan? plan)
        {
            plan = null;
            double end = Math.Min(_hand.Free + RunwaySeconds, _options.MaxTakeSeconds);
            if (end - _hand.Free < MinRunwaySeconds) return false;
            HandSnapshot before = _hand.Save();
            Vector2[] away = Away(objects[knocked], way);
            var runway = new List<HandSnapshot>();
            _hand.AvoidGoals = away;
            bool ok = _hand.FreeUntil(end, SafetyScript.WithObjects(_model, Without(objects, knocked), SafetyScript.ApproachClearanceMeters), null, runway);
            _hand.AvoidGoals = null;
            if (!ok || runway.Count == 0)
            {
                _hand.Restore(before);
                return false;
            }

            var used_ = new byte[used.Count];
            used.CopyTo(used_);
            var next = new SafetyContinuation(this, runway, rng, events.ToArray(), used_, events[events.Count - 1].End, objects[knocked].Id, away);
            plan = Build(events, PlanTime(end), next);
            return true;
        }

        /// <summary>
        /// The take's end: free movement for a tail (<see cref="SafetyOptions.TailMinSeconds"/>-<see cref="SafetyOptions.TailMaxSeconds"/>,
        /// longer to make <see cref="SafetyOptions.MinTakeSeconds"/>), cut while the arm moves. After a knock it keeps clear of the
        /// other objects and goes away from the knocked one. Should no free movement fit, the take ends during the move that leaves
        /// the last event.
        /// </summary>
        private DrivePlan Tail(IReadOnlyList<SafetyObject> objects, int knocked, Vector3 way, ref SeededRng rng, List<SafetyEvent> events)
        {
            double from = _hand.Free;
            double tail = Math.Max(Draw.Uniform(ref rng, _options.TailMinSeconds, _options.TailMaxSeconds), _options.MinTakeSeconds - from);
            tail = Math.Min(tail, Math.Max(_options.TailMinSeconds, _options.MaxTakeSeconds - from));
            HandSnapshot before = _hand.Save();
            _hand.AvoidGoals = knocked >= 0 ? Away(objects[knocked], way) : null;
            bool ok = _hand.FreeUntil(from + tail + 0.6, SafetyScript.WithObjects(_model, knocked >= 0 ? Without(objects, knocked) : objects,
                SafetyScript.ApproachClearanceMeters));
            _hand.AvoidGoals = null;
            if (ok) return Build(events, PlanTime(from + tail), null);
            _hand.Restore(before);
            double end = Math.Max(from, _hand.Timeline.MotionEnd);
            return Build(events, PlanTime(from + (0.5 + 0.3 * rng.NextDouble()) * (end - from)), null);
        }

        /// <summary>See <see cref="SafetyContinuation.Continue"/>.</summary>
        internal DrivePlan Continue(SafetyContinuation c, IReadOnlyList<SafetyObject> objects, double switchAt, out string outcome)
        {
            lock (_hand)
            {
                HandSnapshot at = c.SnapshotAt(switchAt) ?? throw new ArgumentException($"{switchAt} s is not a point the runway can go on from", nameof(switchAt));
                _hand.Restore(at);
                SeededRng rng = c.Rng;
                var events = new List<SafetyEvent>(c.Events);
                var used = new HashSet<byte>(c.Used);

                // Where the plan would go on, the arm must be clear of the objects as they now stand (the knocked one among them).
                ArmCollisionModel near = SafetyScript.WithObjects(_model, objects, SafetyScript.ApproachClearanceMeters);
                for (int k = Math.Max(0, (int)Math.Floor(at.Free / HandModel.TickSeconds)); k < _hand.Poses.Count; k++)
                {
                    if (ArmCollision.IsValid(_profile, near, _hand.Poses[k])) continue;
                    outcome = $"the arm comes within {SafetyScript.ApproachClearanceMeters * 100:0} cm of an object as the objects now stand: the take ends here";
                    return Build(events, switchAt, null);
                }

                int before = events.Count;
                if (at.Free + MinEventSeconds + _options.TailMinSeconds > _options.MaxTakeSeconds)
                {
                    outcome = "no time for another event: the take ends after a tail";
                    return Tail(objects, -1, default, ref rng, events);
                }

                if (!PlanEvents(c.NextIndex, objects, ref rng, events, used, out DrivePlan? plan) || plan == null)
                {
                    outcome = "no approach to any object as they now stand: the take ends after a tail";
                    return Tail(objects, -1, default, ref rng, events);
                }

                outcome = plan.Events.Count > before
                    ? $"event {before + 1} planned against the objects as they now stand ({plan.Events.Count} of {_drawn} drawn)"
                    : "no approach to any object as they now stand: the take ends after a tail";
                return plan;
            }
        }

        /// <summary>The plan: the legs before the operator, every pose it was solved through, ended at <paramref name="cut"/> (plan seconds) while it moves.</summary>
        private DrivePlan Build(List<SafetyEvent> events, double cut, SafetyContinuation? pending)
        {
            var legs = new List<DriveLeg>(_prefix);
            double t = _start;
            ArmPose current = _startPose;
            _hand.AppendLegs(legs, ref t, ref current);
            SafetyScript.CutAt(legs, ref t, ref current, cut);
            return DrivePlan.ForSafety(legs, _script, _speed, events.ToArray(), _drawn, pending);
        }

        private static ScriptEventKind Kind(HandEvent kind) => kind switch
        {
            HandEvent.Stop => ScriptEventKind.Stop,
            HandEvent.Touch => ScriptEventKind.Touch,
            _ => ScriptEventKind.Knock,
        };

        private static List<SafetyObject> Without(IReadOnlyList<SafetyObject> objects, int index)
        {
            var others = new List<SafetyObject>(objects.Count);
            for (int i = 0; i < objects.Count; i++) if (i != index) others.Add(objects[i]);
            return others;
        }

        /// <summary>Where free goals stay away from after a knock: where the object stood, and <see cref="AwayMeters"/> further along the way the swipe sent it.</summary>
        private static Vector2[] Away(in SafetyObject o, Vector3 way)
        {
            var c = new Vector2(o.Center.X, o.Center.Y);
            var w = new Vector2(way.X, way.Y);
            w = w.Length() > 1e-6f ? Vector2.Normalize(w) : Vector2.Zero;
            return new[] { c, c + AwayMeters * w };
        }
    }
}
