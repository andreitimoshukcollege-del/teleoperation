using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.StopLatency
{
    /// <summary>What a stop trial does at a moment of the plan.</summary>
    public enum StopPhase
    {
        /// <summary>The joint is commanded at its speed (after <see cref="StopLatencyPlan.AccelSeconds"/> of speeding up).</summary>
        Move,

        /// <summary>The command stopped dead and holds where it stopped: what is measured.</summary>
        Hold,

        /// <summary>Back to the start pose, slowly and smoothly.</summary>
        Return,

        /// <summary>Still at the start pose before the next trial.</summary>
        Rest,
    }

    /// <summary>One trial: <see cref="Joint"/> (0 yaw, 1 proximal, 2 distal, 3 wrist) moved from the start pose at
    /// <see cref="SpeedRadPerSec"/> in <see cref="Direction"/>, the command stopping dead at <see cref="StopAt"/> (plan seconds)
    /// with the joint <see cref="ExcursionRadians"/> from its start, held there until <see cref="HoldEnd"/>.</summary>
    public sealed class StopTrialPlan
    {
        public StopTrialPlan(int index, int joint, float speed, float direction, double start, double stopAt, double holdEnd, double returnEnd,
            double end, float excursion, in ArmPose held)
        {
            Index = index;
            Joint = joint;
            SpeedRadPerSec = speed;
            Direction = direction;
            Start = start;
            StopAt = stopAt;
            HoldEnd = holdEnd;
            ReturnEnd = returnEnd;
            End = end;
            ExcursionRadians = excursion;
            Held = held;
        }

        public int Index { get; }
        public int Joint { get; }
        public float SpeedRadPerSec { get; }
        public float Direction { get; }
        public double Start { get; }
        public double StopAt { get; }
        public double HoldEnd { get; }
        public double ReturnEnd { get; }
        public double End { get; }
        public float ExcursionRadians { get; }

        /// <summary>The pose the command holds from <see cref="StopAt"/>.</summary>
        public ArmPose Held { get; }
    }

    /// <summary>
    /// The stop-latency measurement's moves (docs/adr/0038 workstream R, item 32), pure and deterministic: from a start pose,
    /// each trial swings one joint at a constant speed, reached over <see cref="AccelSeconds"/>, for
    /// <c>runUp</c> seconds, then the command <b>stops dead</b> mid-move and holds where it stopped for <c>hold</c> seconds
    /// (as the headset's command does when the clutch is let go), returns smoothly at <see cref="ReturnSpeedRadPerSec"/>,
    /// and rests <see cref="RestSeconds"/>. What the arm does after the stop (how long it keeps moving, how far, how far past
    /// the held command) is the measurement. Trials alternate direction; a trial whose path leaves the envelope or collides
    /// by the model in one direction goes the other way, and is left out when neither fits.
    /// </summary>
    public sealed class StopLatencyPlan
    {
        /// <summary>The move reaches its speed linearly over this long (a jump to full speed would be a step, not a move).</summary>
        public const double AccelSeconds = 0.1;

        /// <summary>The way back to the start, peak speed (joint-drive's approach speed).</summary>
        public const float ReturnSpeedRadPerSec = DrivePlan.ApproachSpeedRadPerSec;

        /// <summary>Still at the start pose between trials.</summary>
        public const double RestSeconds = 1.0;

        /// <summary>The longest swing a trial may command, radians (60 degrees).</summary>
        public const float MaxExcursionRadians = 60f * MathF.PI / 180f;

        /// <summary>The fastest a trial may move a joint, rad/s: well under the headset slider's 6.</summary>
        public const float MaxSpeedRadPerSec = 3f;

        private readonly List<StopTrialPlan> _trials;

        private StopLatencyPlan(in ArmPose start, List<StopTrialPlan> trials, List<string> skipped, double runUp)
        {
            StartPose = start;
            _trials = trials;
            Skipped = skipped;
            RunUpSeconds = runUp;
            TotalSeconds = trials.Count > 0 ? trials[^1].End : 0;
        }

        public ArmPose StartPose { get; }
        public IReadOnlyList<StopTrialPlan> Trials => _trials;

        /// <summary>Trials left out, and why.</summary>
        public IReadOnlyList<string> Skipped { get; }

        public double RunUpSeconds { get; }
        public double TotalSeconds { get; }

        /// <summary>How far a trial's joint travels before its command stops: the speed-up, then <paramref name="runUp"/> at speed.</summary>
        public static float Excursion(float speed, double runUp) => (float)(speed * (AccelSeconds / 2 + runUp));

        public static bool TryBuild(in RobotArmProfile profile, ArmCollisionModel model, DriveEnvelope envelope, in ArmPose start,
            IReadOnlyList<int> joints, IReadOnlyList<float> speeds, int repeats, double runUp, double hold,
            out StopLatencyPlan? plan, out string? error)
        {
            plan = null;
            error = null;
            if (joints.Count == 0 || joints.Any(j => j < 0 || j > 3)) { error = "joints must be among 0 (yaw), 1 (proximal), 2 (distal), 3 (wrist)"; return false; }
            if (speeds.Count == 0 || speeds.Any(s => !(s > 0f) || s > MaxSpeedRadPerSec)) { error = $"speeds must be in (0, {MaxSpeedRadPerSec}] rad/s"; return false; }
            if (repeats < 1 || repeats > 10) { error = "repeats must be 1 to 10"; return false; }
            if (!(runUp >= 0.2 && runUp <= 1.0)) { error = "the run-up must be 0.2 to 1.0 s: long enough for the arm to trail its command steadily"; return false; }
            if (!(hold >= 0.8 && hold <= 5.0)) { error = "the hold must be 0.8 to 5 s: long enough for the arm to come to rest"; return false; }
            if (!envelope.Contains(start) || !ArmCollision.IsValid(profile, model, start)) { error = $"the start pose ({DrivePlan.Describe(start)}) is outside the envelope or collides by the model"; return false; }

            var trials = new List<StopTrialPlan>();
            var skipped = new List<string>();
            double t = 0;
            int n = 0;
            foreach (float speed in speeds)
            {
                float excursion = Excursion(speed, runUp);
                foreach (int joint in joints)
                {
                    for (int r = 0; r < repeats; r++)
                    {
                        float preferred = r % 2 == 0 ? 1f : -1f;
                        float? direction = null;
                        foreach (float d in new[] { preferred, -preferred })
                        {
                            if (excursion > MaxExcursionRadians) break;
                            ArmPose to = DrivePlan.WithJoint(start, joint, DrivePlan.Joint(start, joint) + d * excursion);
                            if (DrivePlan.PathValid(profile, model, envelope, start, to))
                            {
                                direction = d;
                                break;
                            }
                        }

                        string what = FormattableString.Invariant($"{DriveEnvelope.JointName(joint)} at {speed:0.##} rad/s ({excursion * 180 / MathF.PI:0} degrees)");
                        if (direction == null)
                        {
                            skipped.Add(excursion > MaxExcursionRadians
                                ? $"{what}: over the {MaxExcursionRadians * 180 / MathF.PI:0}-degree limit (shorten the run-up)"
                                : $"{what}: leaves the envelope or collides by the model either way");
                            continue;
                        }

                        float dir = direction.Value;
                        ArmPose held = DrivePlan.WithJoint(start, joint, DrivePlan.Joint(start, joint) + dir * excursion);
                        double stopAt = t + AccelSeconds + runUp;
                        double holdEnd = stopAt + hold;
                        double returnEnd = holdEnd + DrivePlan.SmoothDuration(excursion, ReturnSpeedRadPerSec);
                        double end = returnEnd + RestSeconds;
                        trials.Add(new StopTrialPlan(n++, joint, speed, dir, t, stopAt, holdEnd, returnEnd, end, excursion, held));
                        t = end;
                    }
                }
            }

            if (trials.Count == 0)
            {
                error = "no trial fits: " + string.Join("; ", skipped);
                return false;
            }

            plan = new StopLatencyPlan(start, trials, skipped, runUp);
            return true;
        }

        /// <summary>The trial running at plan time <paramref name="t"/> (the last one after the plan's end), or -1 before any.</summary>
        public int TrialAt(double t)
        {
            for (int i = 0; i < _trials.Count; i++)
            {
                if (t < _trials[i].End) return t >= _trials[i].Start ? i : -1;
            }

            return _trials.Count - 1;
        }

        /// <summary>The commanded pose at plan time <paramref name="t"/>, and its phase.</summary>
        public ArmPose Sample(double t, out StopPhase phase, out int trial)
        {
            trial = TrialAt(t);
            phase = StopPhase.Rest;
            if (trial < 0) return StartPose;
            StopTrialPlan p = _trials[trial];
            float s0 = DrivePlan.Joint(StartPose, p.Joint);
            double u = t - p.Start;
            if (t < p.StopAt)
            {
                phase = StopPhase.Move;
                double travelled = u <= AccelSeconds
                    ? p.SpeedRadPerSec * u * u / (2 * AccelSeconds)
                    : p.SpeedRadPerSec * (AccelSeconds / 2 + (u - AccelSeconds));
                return DrivePlan.WithJoint(StartPose, p.Joint, s0 + p.Direction * (float)Math.Min(travelled, p.ExcursionRadians));
            }

            if (t < p.HoldEnd)
            {
                phase = StopPhase.Hold;
                return p.Held;
            }

            if (t < p.ReturnEnd)
            {
                phase = StopPhase.Return;
                double f = HandModel.MinimumJerk((t - p.HoldEnd) / (p.ReturnEnd - p.HoldEnd));
                return DrivePlan.WithJoint(StartPose, p.Joint, s0 + p.Direction * p.ExcursionRadians * (float)(1 - f));
            }

            return StartPose;
        }
    }

    /// <summary>What one trial measured after its command stopped. Angles in degrees, the tool point in cm.</summary>
    public readonly struct StopResult
    {
        public StopResult(int trial, int joint, float speed, int samples, double stopMs, double travelDeg, double errorAtStopDeg, double pastTargetDeg,
            double toolTravelCm, double toolErrorAtStopCm, bool settled)
        {
            Trial = trial;
            Joint = joint;
            SpeedRadPerSec = speed;
            Samples = samples;
            StopMs = stopMs;
            TravelDeg = travelDeg;
            ErrorAtStopDeg = errorAtStopDeg;
            PastTargetDeg = pastTargetDeg;
            ToolTravelCm = toolTravelCm;
            ToolErrorAtStopCm = toolErrorAtStopCm;
            Settled = settled;
        }

        public int Trial { get; }
        public int Joint { get; }
        public float SpeedRadPerSec { get; }

        /// <summary>Fresh readings in the hold.</summary>
        public int Samples { get; }

        /// <summary>From the stop until the joint's reading stays within <see cref="StopMeter.SettleRadians"/> of where it ends.</summary>
        public double StopMs { get; }

        /// <summary>How far the joint went after its command stopped: its final reading against its reading at the stop.</summary>
        public double TravelDeg { get; }

        /// <summary>How far the reading trailed the command when the command stopped.</summary>
        public double ErrorAtStopDeg { get; }

        /// <summary>The furthest the reading went past the held command, in the direction of the move (0 when it never did).</summary>
        public double PastTargetDeg { get; }

        public double ToolTravelCm { get; }
        public double ToolErrorAtStopCm { get; }

        /// <summary>Whether the joint was still within <see cref="StopMeter.SettleRadians"/> over the hold's last part: false
        /// means the hold was too short to see it stop (or it never did).</summary>
        public bool Settled { get; }
    }

    /// <summary>
    /// Takes the measured poses as they arrive (any one clock, seconds) and reports each trial's stop. Pure. The readings'
    /// own way back (the driver's read cycle, the relay, the network) delays every reading alike, so the stop time it reports
    /// includes about one reading's delay; the recorded session, read on the robot's clock (<c>scripts/gpu/arm_response.py
    /// --only stops</c>), is the reference.
    /// </summary>
    public sealed class StopMeter
    {
        /// <summary>A joint has stopped once its reading stays this close to where it ends: 2 servo units (0.48 degrees).</summary>
        public const float SettleRadians = 2f / 238.732407f;

        /// <summary>The joint's final place is the median of the hold's last this-many seconds.</summary>
        public const double FinalWindowSeconds = 0.3;

        /// <summary>The tool point: this far along the gripper's axis from the wrist pivot (world-models' TCP offset).</summary>
        public const float ToolOffsetMeters = 0.15f;

        private readonly List<(double T, ArmPose Pose)> _samples = new List<(double, ArmPose)>();

        public void Add(double t, in ArmPose measured) => _samples.Add((t, measured));

        public int Count => _samples.Count;

        /// <summary>The tool point at <paramref name="pose"/>: the wrist pivot plus <see cref="ToolOffsetMeters"/> along the
        /// gripper's axis, whose pitch is proximal + distal + wrist (the IK's), turned by the yaw.</summary>
        public static System.Numerics.Vector3 Tool(in RobotArmProfile profile, in ArmPose pose)
        {
            System.Numerics.Vector3 w = ArmKinematics.Forward(profile, pose.BaseYaw, pose.Proximal, pose.Distal);
            float pitch = pose.Proximal + pose.Distal + pose.Wrist;
            var axis = new System.Numerics.Vector3(MathF.Cos(pitch) * MathF.Cos(pose.BaseYaw), MathF.Cos(pitch) * MathF.Sin(pose.BaseYaw), MathF.Sin(pitch));
            return w + ToolOffsetMeters * axis;
        }

        /// <summary>The trial's stop, from the readings between <paramref name="stopAt"/> and <paramref name="holdEnd"/> (the
        /// meter's clock), with the newest reading at or before the stop as where the arm was then. False with no reading
        /// before the stop or too few in the hold.</summary>
        public bool TryMeasure(in RobotArmProfile profile, StopTrialPlan trial, double stopAt, double holdEnd, out StopResult result)
        {
            result = default;
            int before = _samples.FindLastIndex(s => s.T <= stopAt);
            if (before < 0) return false;
            var hold = _samples.Where(s => s.T > stopAt && s.T <= holdEnd).ToList();
            var tail = hold.Where(s => s.T >= holdEnd - FinalWindowSeconds).Select(s => s.Pose).ToList();
            if (hold.Count < 5 || tail.Count < 2) return false;

            int j = trial.Joint;
            ArmPose atStop = _samples[before].Pose;
            float final = Median(tail.Select(p => DrivePlan.Joint(p, j)));
            double settleAt = stopAt;
            float past = 0f;
            float held = DrivePlan.Joint(trial.Held, j);
            foreach ((double t, ArmPose p) in hold)
            {
                float v = DrivePlan.Joint(p, j);
                if (MathF.Abs(v - final) > SettleRadians) settleAt = t;
                past = MathF.Max(past, (v - held) * trial.Direction);
            }

            ArmPose finalPose = new ArmPose(Median(tail.Select(p => p.BaseYaw)), Median(tail.Select(p => p.Proximal)), Median(tail.Select(p => p.Distal)),
                Median(tail.Select(p => p.Wrist)), Median(tail.Select(p => p.Roll)));
            bool settled = tail.All(p => MathF.Abs(DrivePlan.Joint(p, j) - final) <= SettleRadians);
            const double Deg = 180.0 / Math.PI;
            result = new StopResult(trial.Index, j, trial.SpeedRadPerSec, hold.Count, (settleAt - stopAt) * 1000.0,
                Math.Abs(final - DrivePlan.Joint(atStop, j)) * Deg, Math.Abs(held - DrivePlan.Joint(atStop, j)) * Deg, past * Deg,
                (Tool(profile, finalPose) - Tool(profile, atStop)).Length() * 100.0, (Tool(profile, trial.Held) - Tool(profile, atStop)).Length() * 100.0,
                settled);
            return true;
        }

        private static float Median(IEnumerable<float> values)
        {
            var s = values.OrderBy(v => v).ToList();
            return s.Count % 2 == 1 ? s[s.Count / 2] : 0.5f * (s[s.Count / 2 - 1] + s[s.Count / 2]);
        }
    }
}
