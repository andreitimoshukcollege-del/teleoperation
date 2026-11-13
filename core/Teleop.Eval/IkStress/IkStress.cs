using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.IkStress
{
    /// <summary>
    /// A long, synthetic teleoperation session of the FR3 Duo through the headset's absolute hand mapping, scored for what the user
    /// reported on 2026-11-03: the arm "gets stuck and confused in some positions". The operator (<see cref="OperatorScript"/>)
    /// reaches for goals all over a person's reach, with the grips people use (palm down to pick, pointing forward, palm in,
    /// wrist rolls), at human speeds with minimum-jerk moves, dwells with tremor, loses tracking, lets go and locks again, and
    /// sometimes reaches beyond the robot. Each frame drives <see cref="SerialChainTeleop"/> as <c>SerialChainOperatorBridge</c>
    /// does (EngageAt on a grab, Track while held, Release when let go or lost), at 90 Hz.
    ///
    /// <para><b>The oracle.</b> A frame where the tool is off its target only counts against the IK when the target could have
    /// been reached: a slow multi-start solve (the arm's command and many seeded random starts, collision-checked against the
    /// whole robot with the other arm where it is) finds a pose within 2 mm and 2°. Unreachable targets are the operator
    /// asking too much: the arm stopping short there is right.</para>
    /// <list type="bullet">
    /// <item><b>Stuck:</b> the hand has stopped (its target within 1 cm and 3° of where it was), the target is reachable, and
    /// <see cref="IkStressOptions.StuckSeconds"/> later the tool is still off it (more than 2 cm or 10°). While the hand moves,
    /// the arm trails it by design (its gripper speed caps, 1 m/s and 3 rad/s, below a quick hand's): that is lag, reported
    /// but not an episode.</item>
    /// <item><b>Wrong way:</b> the same, with the tool within 3 cm of the target but pointing more than 30° off it.</item>
    /// <item><b>Swing:</b> a joint command moving more than 0.08 rad in one frame.</item>
    /// </list>
    /// Deterministic for a seed.
    /// </summary>
    public sealed class IkStressOptions
    {
        public double Seconds { get; set; } = 300;
        public double Rate { get; set; } = 90;
        public double StuckSeconds { get; set; } = 1.0;
        public double OffDistance { get; set; } = 0.02;
        public double OffAngle { get; set; } = 10 * Math.PI / 180;
        public double WrongWayAngle { get; set; } = 30 * Math.PI / 180;
        public int OracleStarts { get; set; } = 48;

        /// <summary>Print every frame of one arm between two times: (arm, from, to), or null.</summary>
        public (int Arm, double From, double To)? Trace { get; set; }

        /// <summary>The teleop's settings (its defaults when null): what is being measured.</summary>
        public SerialChainTeleopOptions? Teleop { get; set; }
    }

    public sealed class IkEpisode
    {
        public string Kind = "";
        public ulong Seed;
        public int Arm;
        public double Start, End;
        public double WorstPositionMm, WorstAngleDeg;
        public string Grip = "";
        public RigidTransform Target, Tool;
        public double[] Joints = Array.Empty<double>();
        public double[] Reached = Array.Empty<double>();
        public string Status = "";

        /// <summary>A recovery moved the arm during the episode, and it ended with the tool on its target: slow, not stuck.</summary>
        public bool Recovering, Arrived;

        public double Seconds => End - Start;
    }

    public sealed class IkStressResult
    {
        public ulong Seed;
        public int Frames, EngagedFrames, OffFrames, OffReachableFrames, OracleCalls, SwingFrames;
        public double MaxSwing;
        public readonly List<IkEpisode> Episodes = new List<IkEpisode>();
        public readonly List<double> PositionMm = new List<double>(), AngleDeg = new List<double>();
        public readonly Dictionary<string, int> StatusFrames = new Dictionary<string, int>();

        /// <summary>Off a reachable target: each run's length (s), the status in those frames, and the frames spent recovering.</summary>
        public readonly List<double> OffRuns = new List<double>();
        public readonly Dictionary<string, int> OffReachableStatus = new Dictionary<string, int>();
        public int RecoveringFrames, OffReachableMoving;

        /// <summary>Each time the hand stopped on a reachable target: how long after it stopped the tool got there (s), or the hold's length when it never did.</summary>
        public readonly List<double> SettleSeconds = new List<double>();
        public int SettleMissed;
        public int Searches, SearchesNoBetter, Transits, TransitsBlocked;

        /// <summary>How long each <see cref="SerialChainTeleop.Step"/> of both arms took on this machine (ms).</summary>
        public readonly List<double> StepMs = new List<double>();

        public int Stuck => Episodes.Count(e => e.Kind == "stuck");
        public int WrongWay => Episodes.Count(e => e.Kind == "wrong-way");
        public int Slow => Episodes.Count(e => e.Kind == "slow");
        public IEnumerable<IkEpisode> Failures => Episodes.Where(e => e.Kind != "slow");
    }

    /// <summary>The synthetic operator: per arm, a hand pose in the body frame (Core: x forward, y left, z up, from the head) over time.</summary>
    public sealed class OperatorScript
    {
        /// <summary>The grips: a palm orientation in the body frame (palm x fingertips, y the hand's left, z out of its back).</summary>
        public static readonly (string Name, double Yaw, double Pitch, double Roll)[] Grips =
        {
            ("forward-flat", 0, 0, 0),
            ("forward-palm-in", 0, 0, Math.PI / 2),
            ("forward-palm-out", 0, 0, -Math.PI / 2),
            ("down-pinch", 0, Math.PI / 2, 0),
            ("down-pinch-turned", Math.PI / 2, Math.PI / 2, 0),
            ("down-angled", 0, Math.PI / 4, 0),
            ("down-angled-in", 0, Math.PI / 4, Math.PI / 2),
            ("inward", Math.PI / 3, 0, 0),
            ("inward-down", Math.PI / 3, Math.PI / 3, 0),
            ("outward", -Math.PI / 4, 0, 0),
            ("up-tilt", 0, -Math.PI / 6, 0),
            ("rolled-over", 0, 0, Math.PI),
        };

        private readonly double _side;
        private SeededRng _rng;
        private readonly List<Segment> _segments = new List<Segment>();
        private readonly List<(double Start, double End)> _lost = new List<(double, double)>();
        private readonly List<(double Start, double End)> _open = new List<(double, double)>();
        private readonly double _tremorPhase;

        private struct Segment
        {
            public double Start, End;
            public RigidTransform From, To;
            public string Grip;
            public bool Wander;
            public double WanderAmplitude, WanderFrequency;
        }

        /// <param name="left">The left hand (its own side is +y).</param>
        public OperatorScript(bool left, ulong seed, double seconds)
        {
            _side = left ? 1 : -1;
            _rng = new SeededRng(seed * 2 + (left ? 1UL : 0UL));
            _tremorPhase = 2 * Math.PI * _rng.NextDouble();
            RigidTransform pose = Pose(0.30, 0.20 * _side, -0.40, 0, 0, 0);
            string grip = "forward-flat";
            double t = 0;
            while (t < seconds + 5)
            {
                double u = _rng.NextDouble();
                RigidTransform to;
                double duration;
                bool wander = false;
                if (u < 0.55)
                {
                    // A reach: a goal in a person's reach on this hand's side (sometimes across), with a grip.
                    var g = Grips[(int)(_rng.NextDouble() * Grips.Length) % Grips.Length];
                    grip = g.Name;
                    to = Goal(g.Yaw, g.Pitch, g.Roll, 0.35);
                    duration = Math.Max(0.35, Distance(pose, to) / (0.25 + 0.75 * _rng.NextDouble())) + 0.2 * Angle(pose, to);
                }
                else if (u < 0.70)
                {
                    // The wrist alone: roll, pitch or yaw where the hand is.
                    double roll = (_rng.NextDouble() * 2 - 1) * 2.6, pitch = (_rng.NextDouble() * 2 - 1) * 1.2, yaw = (_rng.NextDouble() * 2 - 1) * 1.0;
                    to = pose.Compose(Pose(0, 0, 0, yaw * 0.5, pitch * 0.5, roll * 0.5));
                    to = new RigidTransform(pose.X, pose.Y, pose.Z, to.Qw, to.Qx, to.Qy, to.Qz);
                    grip += "+wrist";
                    duration = 0.5 + 1.0 * _rng.NextDouble();
                }
                else if (u < 0.85)
                {
                    // Fine work: a small wander where the hand is.
                    to = pose;
                    wander = true;
                    duration = 2 + 3 * _rng.NextDouble();
                }
                else if (u < 0.93)
                {
                    // A dwell.
                    to = pose;
                    duration = 0.3 + 1.7 * _rng.NextDouble();
                }
                else
                {
                    // Beyond the robot: far forward, far down, far out, or high; the next reach comes back.
                    double k = _rng.NextDouble();
                    to = k < 0.25 ? Pose(0.85, 0.2 * _side, -0.35, 0, 0, 0)
                        : k < 0.5 ? Pose(0.35, 0.2 * _side, -0.95, 0, Math.PI / 2, 0)
                        : k < 0.75 ? Pose(0.30, 0.75 * _side, -0.40, -_side * 0.8, 0, 0)
                        : Pose(0.25, 0.15 * _side, 0.10, 0, -0.5, 0);
                    grip = "beyond";
                    duration = 0.8 + 0.8 * _rng.NextDouble();
                }

                _segments.Add(new Segment
                {
                    Start = t, End = t + duration, From = pose, To = to, Grip = grip, Wander = wander,
                    WanderAmplitude = 0.02 + 0.04 * _rng.NextDouble(), WanderFrequency = 0.3 + 0.7 * _rng.NextDouble(),
                });
                t += duration;
                pose = to;
                if (u < 0.55 && _rng.NextDouble() < 0.7)
                {
                    // After most reaches, a hold: grasping, placing, looking.
                    double hold = 0.4 + 2.1 * _rng.NextDouble();
                    _segments.Add(new Segment { Start = t, End = t + hold, From = pose, To = pose, Grip = grip });
                    t += hold;
                }
            }

            // Tracking lost now and then (0.1-1 s), and the hand opened (the arm let go) for 0.5-3 s.
            for (double s = 3; s < seconds; s += 1)
            {
                if (_rng.NextDouble() < 0.02) _lost.Add((s, s + 0.1 + 0.9 * _rng.NextDouble()));
                if (_rng.NextDouble() < 0.012) _open.Add((s, s + 0.5 + 2.5 * _rng.NextDouble()));
            }
        }

        public bool Tracked(double t) => !_lost.Any(l => t >= l.Start && t < l.End);

        public bool Held(double t) => !_open.Any(o => t >= o.Start && t < o.End);

        public string GripAt(double t) => _segments.LastOrDefault(s => s.Start <= t).Grip ?? "";

        /// <summary>The hand in the body frame at <paramref name="t"/>, with tremor.</summary>
        public RigidTransform At(double t)
        {
            int i = _segments.FindIndex(s => t < s.End);
            Segment g = _segments[i < 0 ? _segments.Count - 1 : i];
            double u = Math.Max(0, Math.Min(1, (t - g.Start) / (g.End - g.Start)));
            double s = u * u * u * (10 - 15 * u + 6 * u * u); // minimum jerk
            RigidTransform p = RigidTransform.Interpolate(g.From, g.To, s);
            double x = p.X, y = p.Y, z = p.Z;
            if (g.Wander)
            {
                double w = 2 * Math.PI * g.WanderFrequency * (t - g.Start);
                x += g.WanderAmplitude * Math.Sin(w);
                y += g.WanderAmplitude * Math.Sin(1.3 * w + 1);
                z += g.WanderAmplitude * Math.Sin(0.7 * w + 2);
            }

            // Tremor: about 1.5 mm and 1° at 8-10 Hz.
            double tr = 2 * Math.PI * 9 * t + _tremorPhase;
            x += 0.0015 * Math.Sin(tr);
            y += 0.0015 * Math.Sin(1.1 * tr + 1);
            z += 0.0015 * Math.Sin(0.9 * tr + 2);
            RigidTransform shake = RigidTransform.Rotation(1, 0.3, 0.2, 0.017 * Math.Sin(1.05 * tr));
            RigidTransform r = new RigidTransform(0, 0, 0, p.Qw, p.Qx, p.Qy, p.Qz).Compose(shake);
            return new RigidTransform(x, y, z, r.Qw, r.Qx, r.Qy, r.Qz);
        }

        private RigidTransform Goal(double yaw, double pitch, double roll, double jitter)
        {
            double forward = 0.12 + 0.50 * _rng.NextDouble();
            double side = _side * (-0.15 + 0.65 * _rng.NextDouble());
            double down = -0.78 + 0.70 * _rng.NextDouble();
            double j() => (_rng.NextDouble() * 2 - 1) * jitter;
            // A hand turned toward the midline when it is across it, as an arm does.
            return Pose(forward, side, down, _side * yaw + j(), pitch + j(), _side * roll + j());
        }

        private static RigidTransform Pose(double x, double y, double z, double yaw, double pitch, double roll)
        {
            RigidTransform r = RigidTransform.Rotation(0, 0, 1, yaw).Compose(RigidTransform.Rotation(0, 1, 0, pitch)).Compose(RigidTransform.Rotation(1, 0, 0, roll));
            return new RigidTransform(x, y, z, r.Qw, r.Qx, r.Qy, r.Qz);
        }

        private static double Distance(in RigidTransform a, in RigidTransform b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

        private static double Angle(in RigidTransform a, in RigidTransform b) => SerialChainTeleop.AngleBetween(a, b);
    }

    /// <summary>Whether a tool pose can be reached at all: a slow multi-start solve, collision-checked with the other arm where it is.</summary>
    public sealed class ReachOracle
    {
        private readonly SerialChainProfile _profile;
        private readonly SerialChainCollisionModel? _collision;
        private readonly SerialChainSolver[] _solvers;
        private readonly int _starts;
        private SeededRng _rng;

        public ReachOracle(SerialChainProfile profile, SerialChainCollisionModel? collision, int starts, ulong seed)
        {
            _profile = profile;
            _collision = collision;
            _starts = starts;
            _rng = new SeededRng(seed ^ 0x5EEDUL);
            var options = new SerialChainSolverOptions { Iterations = 300, RestGain = 0, LimitGain = 0.005 };
            _solvers = profile.Arms.Select(a => new SerialChainSolver(a, options)).ToArray();
        }

        /// <summary>A collision-free pose of arm <paramref name="arm"/> reaching <paramref name="target"/> within 2 mm and 2°, or null.</summary>
        public double[]? Reach(int arm, in RigidTransform target, double[] whole, int offset, double[] current)
        {
            ChainArm a = _profile.Arms[arm];
            int n = a.Joints.Length;
            var start = new double[n];
            var solution = new double[n];
            var pose = (double[])whole.Clone();
            for (int k = 0; k <= _starts; k++)
            {
                for (int j = 0; j < n; j++)
                {
                    ChainJointLimits l = a.Joints[j].Limits;
                    start[j] = k == 0 ? current[j] : l.SoftLower + (l.SoftUpper - l.SoftLower) * _rng.NextDouble();
                }

                ChainSolveResult r = _solvers[arm].Solve(target, start, solution);
                if (r.PositionError > 0.002 || r.OrientationError > 2 * Math.PI / 180) continue;
                for (int j = 0; j < n; j++) pose[offset + j] = solution[j];
                if (_collision == null || !SerialChainCollision.Check(_profile, _collision, pose).IsHit) return (double[])solution.Clone();
            }

            return null;
        }
    }

    public static class IkStressRunner
    {
        public static IkStressResult Run(SerialChainProfile profile, SerialChainCollisionModel? collision, ulong seed, IkStressOptions options)
        {
            var teleop = new SerialChainTeleop(profile, collision, options.Teleop);
            var mapping = new BodyAnchoredMapping(profile, BodyAnchoredMapping.NeutralTools(profile));
            mapping.Recentre(RigidTransform.Identity);
            if (!teleop.TryTakePose(ReadyMeasured(profile))) throw new InvalidOperationException("the ready pose was refused");
            int arms = profile.Arms.Length;
            var scripts = new OperatorScript[arms];
            var offsets = new int[arms];
            for (int a = 0, o = 0; a < arms; a++)
            {
                scripts[a] = new OperatorScript(profile.Arms[a].Mount.Y >= 0, seed, options.Seconds);
                offsets[a] = o;
                o += profile.Arms[a].Joints.Length;
            }

            var oracle = new ReachOracle(profile, collision, options.OracleStarts, seed);
            var result = new IkStressResult { Seed = seed };
            double dt = 1.0 / options.Rate;
            var engagedAt = new double[arms];
            var offSince = new double[arms];
            var wrongSince = new double[arms];
            var reachableKnown = new bool[arms];
            var lastReach = new double[arms][];
            var open = new IkEpisode?[arms];
            var openWrong = new IkEpisode?[arms];
            var lastOracle = new double[arms];
            var lastOracleTarget = new RigidTransform[arms];
            var before = new double[arms][];
            var lastGoal = new RigidTransform[arms];
            var runStart = new double[arms];
            var anchor = new RigidTransform[arms];
            var settledSince = new double[arms];
            var settleOpen = new bool[arms];
            var settleReachable = new bool[arms];
            for (int a = 0; a < arms; a++)
            {
                runStart[a] = double.NaN;
                offSince[a] = wrongSince[a] = double.NaN;
                lastOracle[a] = double.NegativeInfinity;
                lastReach[a] = Array.Empty<double>();
            }

            var whole = new double[profile.Arms.Sum(x => x.Joints.Length)];
            int frames = (int)(options.Seconds * options.Rate);
            for (int f = 0; f < frames; f++)
            {
                double t = f * dt;
                for (int a = 0; a < arms; a++)
                {
                    OperatorScript s = scripts[a];
                    bool tracked = s.Tracked(t), held = s.Held(t);
                    if (!tracked || !held)
                    {
                        if (teleop.IsEngaged(a)) teleop.Release(a);
                        Close(result, open, a, t);
                        Close(result, openWrong, a, t);
                        offSince[a] = wrongSince[a] = double.NaN;
                        continue;
                    }

                    RigidTransform target = mapping.Map(a, s.At(t));
                    // The hand has stopped while its target stays within 1 cm and 3° of where it settled.
                    if (Distance(target, anchor[a]) > 0.01 || SerialChainTeleop.AngleBetween(target, anchor[a]) > 0.05)
                    {
                        // The hold ended before the tool got there: a miss, if the hold was long enough to judge and the target reachable.
                        if (settleOpen[a] && settleReachable[a] && t - settledSince[a] >= 0.3)
                        {
                            result.SettleSeconds.Add(t - settledSince[a]);
                            result.SettleMissed++;
                        }

                        anchor[a] = target;
                        settledSince[a] = t;
                        settleOpen[a] = true;
                        settleReachable[a] = false;
                    }

                    if (!teleop.IsEngaged(a))
                    {
                        teleop.EngageAt(a, target);
                        engagedAt[a] = t;
                    }
                    else
                    {
                        teleop.Track(a, target);
                    }

                    before[a] = teleop.Commanded(a).ToArray();
                }

                long started = System.Diagnostics.Stopwatch.GetTimestamp();
                teleop.Step(dt);
                result.StepMs.Add((System.Diagnostics.Stopwatch.GetTimestamp() - started) * 1000.0 / System.Diagnostics.Stopwatch.Frequency);
                result.Frames++;
                for (int a = 0; a < arms; a++)
                {
                    string status = teleop.Status(a).ToString();
                    result.StatusFrames[status] = result.StatusFrames.TryGetValue(status, out int c) ? c + 1 : 1;
                    if (!teleop.IsEngaged(a)) continue;
                    double[] q = teleop.Commanded(a).ToArray();
                    double swing = 0;
                    if (before[a] != null) for (int j = 0; j < q.Length; j++) swing = Math.Max(swing, Math.Abs(q[j] - before[a][j]));
                    result.MaxSwing = Math.Max(result.MaxSwing, swing);
                    if (swing > 0.08) result.SwingFrames++;
                    if (options.Trace is (int ta, double tf, double tt) && ta == a && t >= tf && t <= tt)
                    {
                        RigidTransform tl = teleop.Tool(a), tg = teleop.Target(a);
                        Console.WriteLine(FormattableString.Invariant($"  t {t:0.000} {teleop.Status(a),-12} {(teleop.IsRecovering(a) ? "R" : " ")} err {1000 * Distance(tl, tg),6:0} mm {SerialChainTeleop.AngleBetween(tl, tg) * 57.3,5:0}° ") +
                                          string.Join(" ", q.Select(v => v.ToString("0.00", System.Globalization.CultureInfo.InvariantCulture))) +
                                          $"  s{teleop.Searches}/{teleop.SearchesNoBetter} m{teleop.Transits}/{teleop.TransitsBlocked}");
                    }

                    if (t - engagedAt[a] < 2.0) continue; // the slide in after a grab
                    result.EngagedFrames++;
                    RigidTransform tool = teleop.Tool(a), goal = teleop.Target(a);
                    double pos = Distance(tool, goal), ang = SerialChainTeleop.AngleBetween(tool, goal);
                    result.PositionMm.Add(1000 * pos);
                    result.AngleDeg.Add(ang * 180 / Math.PI);
                    if (teleop.IsRecovering(a)) result.RecoveringFrames++;
                    bool off = pos > options.OffDistance || ang > options.OffAngle;
                    RigidTransform previousGoal = lastGoal[a];
                    lastGoal[a] = goal;
                    bool wrong = pos < 0.03 && ang > options.WrongWayAngle;
                    if (!off && !double.IsNaN(runStart[a])) { result.OffRuns.Add(t - runStart[a]); runStart[a] = double.NaN; }
                    if (!off && settleOpen[a])
                    {
                        // On target: how long after the hand stopped (0 when it was there all along).
                        if (settleReachable[a]) result.SettleSeconds.Add(Math.Max(0, t - settledSince[a]));
                        settleOpen[a] = false;
                    }
                    if (!off)
                    {
                        Close(result, open, a, t, arrived: true);
                        Close(result, openWrong, a, t, arrived: true);
                        offSince[a] = wrongSince[a] = double.NaN;
                        continue;
                    }

                    result.OffFrames++;
                    if (double.IsNaN(offSince[a])) offSince[a] = Math.Max(t, settledSince[a] + options.StuckSeconds);
                    // Ask the oracle at most every 0.1 s, or when the target has moved.
                    if (t - lastOracle[a] >= 0.1 || Distance(goal, lastOracleTarget[a]) > 0.01 || SerialChainTeleop.AngleBetween(goal, lastOracleTarget[a]) > 0.05)
                    {
                        for (int b = 0; b < arms; b++)
                        {
                            ReadOnlySpan<double> cq = teleop.Commanded(b);
                            for (int j = 0; j < cq.Length; j++) whole[offsets[b] + j] = cq[j];
                        }

                        double[]? reached = oracle.Reach(a, goal, whole, offsets[a], q);
                        result.OracleCalls++;
                        reachableKnown[a] = reached != null;
                        lastReach[a] = reached ?? Array.Empty<double>();
                        lastOracle[a] = t;
                        lastOracleTarget[a] = goal;
                    }

                    if (!reachableKnown[a])
                    {
                        // The operator asked too much: no episode, and the clock starts again when it comes back in reach.
                        Close(result, open, a, t);
                        Close(result, openWrong, a, t);
                        offSince[a] = wrongSince[a] = double.NaN;
                        continue;
                    }

                    result.OffReachableFrames++;
                    if (settleOpen[a] && t - settledSince[a] < 0.05) settleReachable[a] = true;
                    if (settleOpen[a] && !settleReachable[a]) settleReachable[a] = true;
                    // Only once the hand has stopped does an off frame count toward an episode.
                    double settled = t - settledSince[a];
                    if (settled < options.StuckSeconds)
                    {
                        offSince[a] = wrongSince[a] = double.NaN;
                        Close(result, open, a, t);
                        Close(result, openWrong, a, t);
                    }
                    string why = status + (teleop.IsRecovering(a) ? "+recovering" : "");
                    result.OffReachableStatus[why] = result.OffReachableStatus.TryGetValue(why, out int w) ? w + 1 : 1;
                    if (Distance(goal, previousGoal) > 0.25 * dt || SerialChainTeleop.AngleBetween(goal, previousGoal) > 0.5 * dt) result.OffReachableMoving++;
                    if (double.IsNaN(runStart[a])) runStart[a] = t;
                    if (settled < options.StuckSeconds) continue;
                    if (double.IsNaN(offSince[a])) offSince[a] = settledSince[a];
                    if (wrong && double.IsNaN(wrongSince[a])) wrongSince[a] = settledSince[a];
                    if (!wrong) wrongSince[a] = double.NaN;
                    Track(result, open, a, "stuck", offSince[a], t, options.StuckSeconds, pos, ang, scripts[a].GripAt(t), goal, tool, q, lastReach[a], status, teleop.IsRecovering(a));
                    if (wrong) Track(result, openWrong, a, "wrong-way", wrongSince[a], t, options.StuckSeconds, pos, ang, scripts[a].GripAt(t), goal, tool, q, lastReach[a], status, teleop.IsRecovering(a));
                }
            }

            for (int a = 0; a < arms; a++)
            {
                Close(result, open, a, options.Seconds);
                Close(result, openWrong, a, options.Seconds);
            }

            foreach (IkEpisode e in result.Episodes) e.Seed = seed;
            result.Searches = teleop.Searches;
            result.SearchesNoBetter = teleop.SearchesNoBetter;
            result.Transits = teleop.Transits;
            result.TransitsBlocked = teleop.TransitsBlocked;
            return result;
        }

        private static void Track(IkStressResult result, IkEpisode?[] open, int a, string kind, double since, double t, double threshold,
            double pos, double ang, string grip, in RigidTransform goal, in RigidTransform tool, double[] q, double[] reach, string status, bool recovering)
        {
            if (double.IsNaN(since) || t - since < threshold) return;
            IkEpisode e = open[a] ??= new IkEpisode { Kind = kind, Arm = a, Start = since };
            e.End = t;
            e.Recovering |= recovering;
            if (1000 * pos >= e.WorstPositionMm || ang * 180 / Math.PI >= e.WorstAngleDeg)
            {
                e.WorstPositionMm = Math.Max(e.WorstPositionMm, 1000 * pos);
                e.WorstAngleDeg = Math.Max(e.WorstAngleDeg, ang * 180 / Math.PI);
                e.Grip = grip;
                e.Target = goal;
                e.Tool = tool;
                e.Joints = q;
                e.Reached = reach;
                e.Status = status;
            }
        }

        private static void Close(IkStressResult result, IkEpisode?[] open, int a, double t, bool arrived = false)
        {
            if (open[a] == null) return;
            open[a]!.End = t;
            open[a]!.Arrived = arrived;
            if (arrived && open[a]!.Recovering) open[a]!.Kind = "slow";
            result.Episodes.Add(open[a]!);
            open[a] = null;
        }

        private static double Distance(in RigidTransform a, in RigidTransform b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

        /// <summary>Franka's ready pose on each arm (the right mirrored), hands open.</summary>
        public static JointSample[] ReadyMeasured(SerialChainProfile profile)
        {
            var samples = new List<JointSample>();
            foreach (ChainArm arm in profile.Arms)
            {
                bool mirrored = arm.Mount.Y < 0;
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    double q = mirrored && j % 2 == 0 ? -BodyAnchoredMapping.FrankaReady[j] : BodyAnchoredMapping.FrankaReady[j];
                    samples.Add(new JointSample(arm.Joints[j].MotorId, (float)q, true));
                }

                if (arm.HasGripper) samples.Add(new JointSample(arm.Gripper.MotorId, 1f, true));
            }

            return samples.ToArray();
        }
    }
}
