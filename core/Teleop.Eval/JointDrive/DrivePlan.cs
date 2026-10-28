using System;
using System.Collections.Generic;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.JointDrive
{
    /// <summary>What <c>joint-drive</c> does with the arm (docs/adr/0021, <c>conditions.motion</c> <c>joint-drive:&lt;script&gt;</c>).</summary>
    public enum DriveScript
    {
        /// <summary>Smooth moves between random waypoints, with pauses: teleop-like motion at a known speed.</summary>
        Waypoints,

        /// <summary>One joint at a time jumps and holds, so the relay moves it at the requested speed: what <c>armpred sysid</c> needs to see the servo's speed limit and lag.</summary>
        Steps,

        /// <summary>
        /// The headset's training course driven by the robot itself (<see cref="CourseScript"/>): REACH,
        /// TRACE and SHUTTLE rounds on the course's own targets, solved by the headset's IK, at the
        /// course's speed levels, with the gripper opened and closed. Scripted data shaped like teleop.
        /// </summary>
        Course,
    }

    /// <summary>
    /// The joint ranges <c>joint-drive</c> may use: the collision model's ranges (docs/adr/0020, the
    /// IK's limits), each end pulled in by a margin. Joint order is the model's: 0 yaw, 1 proximal,
    /// 2 distal, 3 wrist, 4 roll.
    /// </summary>
    public readonly struct DriveEnvelope
    {
        public const int JointCount = 5;

        private readonly AngleRange _yaw, _proximal, _distal, _wrist, _roll;

        public DriveEnvelope(AngleRange yaw, AngleRange proximal, AngleRange distal, AngleRange wrist, AngleRange roll)
        {
            _yaw = yaw;
            _proximal = proximal;
            _distal = distal;
            _wrist = wrist;
            _roll = roll;
        }

        public static DriveEnvelope FromModel(ArmCollisionModel model, float marginRadians)
        {
            static AngleRange Shrink(AngleRange r, float m)
            {
                if (r.Max - r.Min <= 2 * m)
                {
                    throw new ArgumentException($"a margin of {m} rad leaves nothing of the range [{r.Min}, {r.Max}]");
                }

                return new AngleRange(r.Min + m, r.Max - m);
            }

            return new DriveEnvelope(
                Shrink(model.BaseYaw, marginRadians), Shrink(model.Proximal, marginRadians), Shrink(model.Distal, marginRadians),
                Shrink(model.Wrist, marginRadians), Shrink(model.Roll, marginRadians));
        }

        public AngleRange this[int joint] => joint switch
        {
            0 => _yaw,
            1 => _proximal,
            2 => _distal,
            3 => _wrist,
            4 => _roll,
            _ => throw new ArgumentOutOfRangeException(nameof(joint)),
        };

        public bool Contains(in ArmPose pose)
        {
            for (int j = 0; j < JointCount; j++)
            {
                if (!this[j].Contains(DrivePlan.Joint(pose, j))) return false;
            }

            return true;
        }

        public static string JointName(int joint) => joint switch
        {
            0 => "base yaw",
            1 => "proximal",
            2 => "distal",
            3 => "wrist",
            4 => "roll",
            _ => joint.ToString(),
        };
    }

    /// <summary>Which part of the session a leg belongs to.</summary>
    public enum LegPhase
    {
        /// <summary>From the measured pose to the model's home, slowly.</summary>
        Approach,
        Script,
        /// <summary>Back to home at the end, slowly.</summary>
        Return,
    }

    /// <summary>
    /// One move: from <see cref="From"/> to <see cref="To"/> starting at <see cref="Start"/> (seconds
    /// from the plan's start), taking <see cref="Duration"/> (0 for a step, which is commanded at
    /// once), then holding <see cref="To"/> for <see cref="Hold"/>.
    /// </summary>
    public readonly struct DriveLeg
    {
        public readonly double Start;
        public readonly double Duration;
        public readonly double Hold;
        public readonly ArmPose From;
        public readonly ArmPose To;
        public readonly LegPhase Phase;

        /// <summary>The speed the commands ask for during this leg, rad/s; NaN for the plan's own.</summary>
        public readonly float Speed;

        /// <summary>The gripper fraction at the leg's start and end (ramped between); NaN holds it as reported.</summary>
        public readonly float GripperFrom, GripperTo;

        /// <summary>A constant-speed leg (a trace's segment) instead of a minimum-jerk one.</summary>
        public readonly bool Linear;

        public DriveLeg(double start, double duration, double hold, ArmPose from, ArmPose to, LegPhase phase,
            float speed = float.NaN, float gripperFrom = float.NaN, float gripperTo = float.NaN, bool linear = false)
        {
            Start = start;
            Duration = duration;
            Hold = hold;
            From = from;
            To = to;
            Phase = phase;
            Speed = speed;
            GripperFrom = gripperFrom;
            GripperTo = gripperTo;
            Linear = linear;
        }

        public double End => Start + Duration + Hold;
    }

    /// <summary>
    /// A whole <c>joint-drive</c> session, planned before anything moves: approach home, the script,
    /// return home. Deterministic for a given start pose, script, speed, length and seed.
    ///
    /// <para><b>Every pose it can command is checked first.</b> Each leg's path is sampled in joint
    /// space at most <see cref="PathStepRadians"/> apart, and every sample must pass
    /// <see cref="ArmCollision.IsValid"/> (the model RobotHost also checks every command against,
    /// docs/adr/0020). Script legs must also stay inside the <see cref="DriveEnvelope"/>. The approach
    /// starts wherever the arm is, possibly outside the envelope, so it is held to the collision model
    /// only. A smooth leg follows its straight joint-space path (minimum-jerk timing changes when, not
    /// where), so checking the path covers every pose sent. A step moves one joint, so the relay's
    /// own per-joint move is on that path too.</para>
    /// </summary>
    public sealed class DrivePlan
    {
        /// <summary>The largest joint-space gap between two checked poses on a path, radians.</summary>
        public const float PathStepRadians = 0.02f;

        /// <summary>Peak joint speed of the approach and return legs, rad/s.</summary>
        public const float ApproachSpeedRadPerSec = 0.5f;

        /// <summary>The peak of minimum-jerk velocity over its mean: 15/8.</summary>
        private const double MinJerkPeak = 1.875;

        private const int MaxDrawAttempts = 400;

        private readonly List<DriveLeg> _legs;
        private int _cursor;

        private DrivePlan(List<DriveLeg> legs, DriveScript script, float speed)
        {
            _legs = legs;
            Script = script;
            SpeedRadPerSec = speed;
        }

        public IReadOnlyList<DriveLeg> Legs => _legs;

        public DriveScript Script { get; }

        public float SpeedRadPerSec { get; }

        public double TotalSeconds => _legs.Count == 0 ? 0 : _legs[_legs.Count - 1].End;

        /// <summary>
        /// Plans a session. False, with the reason, when the arm's start pose cannot reach home on a
        /// path the model calls free, or the script cannot find a valid move.
        /// </summary>
        public static bool TryBuild(
            DriveScript script, in RobotArmProfile profile, in ArmPose start, float speedRadPerSec, double seconds, ulong seed,
            in DriveEnvelope envelope, out DrivePlan? plan, out string? error)
        {
            plan = null;
            error = null;
            ArmCollisionModel? model = profile.Collision;
            if (model == null)
            {
                error = "the profile has no collision model, so no pose can be checked";
                return false;
            }

            if (!(speedRadPerSec > 0f) || !(seconds > 0))
            {
                error = "speed and seconds must be positive";
                return false;
            }

            ArmPose home = model.Home;
            if (!envelope.Contains(home) || !ArmCollision.IsValid(profile, model, home))
            {
                error = "the collision model's home pose is outside the envelope; use a smaller margin";
                return false;
            }

            var legs = new List<DriveLeg>();
            double t = 0;
            if (start.MaxJointDelta(home) > 0.01f)
            {
                if (!TryApproach(profile, model, start, home, out List<(ArmPose To, bool Step)> via, out error))
                {
                    return false;
                }

                ArmPose from = start;
                for (int i = 0; i < via.Count; i++)
                {
                    (ArmPose to, bool step) = via[i];
                    float delta = from.MaxJointDelta(to);
                    // A step is commanded at once and the relay moves it at the approach speed: wait for it.
                    double d = step ? 0 : SmoothDuration(delta, ApproachSpeedRadPerSec);
                    double hold = step ? delta / ApproachSpeedRadPerSec + 0.5 : i == via.Count - 1 ? 1.0 : 0.3;
                    legs.Add(new DriveLeg(t, d, hold, from, to, LegPhase.Approach, ApproachSpeedRadPerSec));
                    t += d + hold;
                    from = to;
                }
            }

            var rng = new SeededRng(seed);
            ArmPose current = home;
            double end = t + seconds;
            int stepJoint = 0;
            if (script == DriveScript.Course)
            {
                if (!CourseScript.Append(profile, model, envelope, ref current, ref t, end, ref rng, legs, out error))
                {
                    return false;
                }
            }

            while (script != DriveScript.Course && t < end)
            {
                DriveLeg leg;
                string? why = script == DriveScript.Waypoints
                    ? NextWaypoint(profile, model, envelope, current, speedRadPerSec, t, ref rng, out leg)
                    : NextStep(profile, model, envelope, current, t, ref stepJoint, ref rng, out leg);
                if (why != null)
                {
                    error = why;
                    return false;
                }

                legs.Add(leg);
                t = leg.End;
                current = leg.To;
            }

            if (current.MaxJointDelta(home) > 0.01f)
            {
                if (!PathValid(profile, model, envelope, current, home))
                {
                    error = $"no free path back to home from {Describe(current)}";
                    return false;
                }

                legs.Add(new DriveLeg(t, SmoothDuration(current.MaxJointDelta(home), ApproachSpeedRadPerSec), 0.5, current, home, LegPhase.Return));
            }

            plan = new DrivePlan(legs, script, speedRadPerSec);
            return true;
        }

        /// <summary>The pose the plan commands at <paramref name="t"/> seconds. Fastest when called with a rising time.</summary>
        public ArmPose Sample(double t)
        {
            if (_legs.Count == 0) throw new InvalidOperationException("empty plan");
            if (_cursor >= _legs.Count || _legs[_cursor].Start > t) _cursor = 0;
            while (_cursor < _legs.Count - 1 && _legs[_cursor].End <= t) _cursor++;
            DriveLeg leg = _legs[_cursor];
            double into = t - leg.Start;
            if (into <= 0) return leg.From;
            if (leg.Duration <= 0 || into >= leg.Duration) return leg.To;
            double u = into / leg.Duration;
            double s = leg.Linear ? u : u * u * u * (10 - u * (15 - 6 * u));
            return ArmPose.Lerp(leg.From, leg.To, (float)s);
        }

        /// <summary>The speed the commands ask for at <paramref name="t"/>: the leg's, else the plan's.</summary>
        public float SpeedAt(double t)
        {
            float s = LegAt(t).Speed;
            return float.IsNaN(s) ? SpeedRadPerSec : s;
        }

        /// <summary>The gripper fraction to command at <paramref name="t"/>, ramped over the leg; NaN to hold it as reported.</summary>
        public float GripperAt(double t)
        {
            DriveLeg leg = LegAt(t);
            if (float.IsNaN(leg.GripperTo)) return float.NaN;
            if (float.IsNaN(leg.GripperFrom) || leg.Duration <= 0) return leg.GripperTo;
            double u = Math.Min(1.0, Math.Max(0.0, (t - leg.Start) / Math.Max(leg.Duration, CourseScript.GripperRampSeconds)));
            return (float)(leg.GripperFrom + (leg.GripperTo - leg.GripperFrom) * u);
        }

        /// <summary>The leg in force at <paramref name="t"/>.</summary>
        public DriveLeg LegAt(double t)
        {
            Sample(t);
            return _legs[_cursor];
        }

        /// <summary>The largest single-joint move tried to lift a resting arm clear of its support, degrees.</summary>
        public const float MaxEscapeDegrees = 20f;

        /// <summary>
        /// How to get from the arm's measured pose to home, collision-free by the model: straight when
        /// that is clear, else one joint at a time in the first order whose every leg is clear. An arm that
        /// rests on something (sagged onto its support with its torque off, or parked) first makes the
        /// smallest single-joint move, up to <see cref="MaxEscapeDegrees"/>, that frees it, with only the
        /// contacts the resting pose already has allowed on the way (<see cref="ArmContacts"/>); that move
        /// is a step (<c>Step</c>), commanded at once, since RobotHost refuses a commanded pose that collides.
        /// </summary>
        public static bool TryApproach(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose start, in ArmPose home,
            out List<(ArmPose To, bool Step)> via, out string? error)
        {
            via = new List<(ArmPose, bool)>();
            error = null;
            if (ArmCollision.IsValid(profile, model, start))
            {
                if (TryReach(profile, model, start, home, via)) return true;
                error = $"no collision-free path from the arm's measured pose ({Describe(start)}) to home ({Describe(home)}), " +
                        "straight or one joint at a time, so nothing was sent. Move the arm to a clear pose first " +
                        "(just move-arm 0.15 0 0.08), then run again";
                return false;
            }

            ArmCollisionModel? allowed = ArmContacts.AllowPoseContacts(profile, model, start, out string contacts);
            if (allowed == null)
            {
                error = $"the arm's measured pose ({Describe(start)}) collides by the model ({contacts}), and not only by resting on " +
                        "something, so nothing was sent. Move the arm to a clear pose first (just move-arm 0.15 0 0.08), then run again";
                return false;
            }

            int[] order = { 1, 2, 3, 0, 4 }; // lift with the shoulder and elbow first, as a person would
            for (int deg = 3; deg <= MaxEscapeDegrees; deg++)
            {
                foreach (int j in order)
                {
                    foreach (float sign in new[] { 1f, -1f })
                    {
                        ArmPose free = WithJoint(start, j, Joint(start, j) + sign * deg * MathF.PI / 180f);
                        if (!ArmCollision.IsValid(profile, model, free) || !PathValid(profile, allowed, null, start, free)) continue;
                        var rest = new List<(ArmPose, bool)>();
                        if (!TryReach(profile, model, free, home, rest)) continue;
                        via.Add((free, true));
                        via.AddRange(rest);
                        return true;
                    }
                }
            }

            error = $"the arm rests on something ({contacts}) at {Describe(start)}, and no single-joint move of up to " +
                    $"{MaxEscapeDegrees:0}° frees it on a path to home, so nothing was sent. Lift it clear first (just move-arm 0.15 0 0.08)";
            return false;
        }

        private static bool TryReach(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose from, in ArmPose home, List<(ArmPose, bool)> via)
        {
            if (PathValid(profile, model, null, from, home))
            {
                via.Add((home, false));
                return true;
            }

            var differ = new List<int>();
            for (int j = 0; j < DriveEnvelope.JointCount; j++) if (MathF.Abs(Joint(from, j) - Joint(home, j)) > 0.01f) differ.Add(j);
            foreach (int[] perm in Permutations(differ.ToArray()))
            {
                var legs = new List<(ArmPose, bool)>();
                ArmPose p = from;
                bool ok = true;
                foreach (int j in perm)
                {
                    ArmPose next = WithJoint(p, j, Joint(home, j));
                    if (!PathValid(profile, model, null, p, next)) { ok = false; break; }
                    legs.Add((next, false));
                    p = next;
                }

                if (ok)
                {
                    via.AddRange(legs);
                    return true;
                }
            }

            return false;
        }

        private static IEnumerable<int[]> Permutations(int[] items)
        {
            if (items.Length <= 1)
            {
                yield return items;
                yield break;
            }

            for (int i = 0; i < items.Length; i++)
            {
                int[] others = new int[items.Length - 1];
                for (int k = 0, o = 0; k < items.Length; k++) if (k != i) others[o++] = items[k];
                foreach (int[] rest in Permutations(others))
                {
                    var p = new int[items.Length];
                    p[0] = items[i];
                    Array.Copy(rest, 0, p, 1, rest.Length);
                    yield return p;
                }
            }
        }

        /// <summary>Whether every pose on the straight joint-space path is free (and, with an envelope, inside it).</summary>
        public static bool PathValid(in RobotArmProfile profile, ArmCollisionModel model, DriveEnvelope? envelope, in ArmPose from, in ArmPose to)
        {
            int steps = Math.Max(1, (int)MathF.Ceiling(from.MaxJointDelta(to) / PathStepRadians));
            for (int s = 0; s <= steps; s++)
            {
                ArmPose p = ArmPose.Lerp(from, to, s / (float)steps);
                if (envelope.HasValue && !envelope.Value.Contains(p)) return false;
                if (!ArmCollision.IsValid(profile, model, p)) return false;
            }

            return true;
        }

        public static float Joint(in ArmPose pose, int joint) => joint switch
        {
            0 => pose.BaseYaw,
            1 => pose.Proximal,
            2 => pose.Distal,
            3 => pose.Wrist,
            4 => pose.Roll,
            _ => throw new ArgumentOutOfRangeException(nameof(joint)),
        };

        public static ArmPose WithJoint(in ArmPose pose, int joint, float value) => new ArmPose(
            joint == 0 ? value : pose.BaseYaw,
            joint == 1 ? value : pose.Proximal,
            joint == 2 ? value : pose.Distal,
            joint == 3 ? value : pose.Wrist,
            joint == 4 ? value : pose.Roll);

        public static string Describe(in ArmPose p) =>
            FormattableString.Invariant($"yaw {Deg(p.BaseYaw):0}°, proximal {Deg(p.Proximal):0}°, distal {Deg(p.Distal):0}°, wrist {Deg(p.Wrist):0}°, roll {Deg(p.Roll):0}°");

        private static double Deg(float radians) => radians * 180.0 / Math.PI;

        internal static double SmoothDuration(float distance, float peakSpeed) => Math.Max(0.3, MinJerkPeak * distance / peakSpeed);

        /// <summary>
        /// A smooth move to a random pose: usually near the current one (each joint up to ±0.6 rad
        /// away), sometimes anywhere in the envelope. Its peak speed is 40-100% of the requested speed,
        /// and half the moves end in a pause of 0.2-1.2 s, as a person's do.
        /// </summary>
        private static string? NextWaypoint(
            in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, in ArmPose current, float speed,
            double t, ref SeededRng rng, out DriveLeg leg)
        {
            leg = default;
            for (int attempt = 0; attempt < MaxDrawAttempts; attempt++)
            {
                bool local = rng.NextDouble() < 0.7;
                ArmPose target = current;
                for (int j = 0; j < DriveEnvelope.JointCount; j++)
                {
                    AngleRange r = envelope[j];
                    float value = local
                        ? Joint(current, j) + (float)((rng.NextDouble() * 2 - 1) * 0.6)
                        : r.Min + (float)rng.NextDouble() * (r.Max - r.Min);
                    target = WithJoint(target, j, r.Clamp(value));
                }

                float distance = current.MaxJointDelta(target);
                if (distance < 0.05f || !PathValid(profile, model, envelope, current, target)) continue;
                double peak = speed * (0.4 + 0.6 * rng.NextDouble());
                double hold = rng.NextDouble() < 0.5 ? 0 : 0.2 + rng.NextDouble();
                leg = new DriveLeg(t, Math.Max(0.3, MinJerkPeak * distance / peak), hold, current, target, LegPhase.Script);
                return null;
            }

            return $"no valid waypoint from {Describe(current)} in {MaxDrawAttempts} draws";
        }

        /// <summary>
        /// The next joint in turn jumps by 0.15-0.8 rad, either way, and holds 1.2-1.8 s. A jump that
        /// would leave the envelope or collide tries the other way, then half the size, then the next
        /// joint.
        /// </summary>
        private static string? NextStep(
            in RobotArmProfile profile, ArmCollisionModel model, in DriveEnvelope envelope, in ArmPose current,
            double t, ref int stepJoint, ref SeededRng rng, out DriveLeg leg)
        {
            leg = default;
            for (int tried = 0; tried < DriveEnvelope.JointCount; tried++)
            {
                int j = stepJoint;
                stepJoint = (stepJoint + 1) % DriveEnvelope.JointCount;
                float amplitude = 0.15f + 0.65f * (float)rng.NextDouble();
                float sign = rng.NextDouble() < 0.5 ? -1f : 1f;
                double hold = 1.2 + 0.6 * rng.NextDouble();
                foreach (float size in new[] { amplitude, amplitude * 0.5f })
                {
                    foreach (float s in new[] { sign, -sign })
                    {
                        ArmPose target = WithJoint(current, j, Joint(current, j) + s * size);
                        if (PathValid(profile, model, envelope, current, target))
                        {
                            leg = new DriveLeg(t, 0, hold, current, target, LegPhase.Script);
                            return null;
                        }
                    }
                }
            }

            return $"no joint can step from {Describe(current)}";
        }
    }
}
