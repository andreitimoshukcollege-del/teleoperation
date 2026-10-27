using System;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Training
{
    /// <summary>What a reach target asks of the gripper as well as the wrist.</summary>
    public enum GripperGoal : byte
    {
        Any = 0,
        Open = 1,
        Closed = 2,
    }

    /// <summary>
    /// The headset's training course (docs/adr/0024): rounds of reach, trace and shuttle over the
    /// robot's workspace, each at a speed level the course picks by coverage. Plain C#, time injected
    /// (seconds, the caller's monotonic clock), draws from <see cref="SeededRng"/> seeded by the course
    /// id; <see cref="Update"/> allocates nothing. Positions are in the arm's base frame (metres).
    /// <list type="bullet">
    /// <item>A round is announced for <see cref="CountdownSeconds"/> (phase idle, the next task shown),
    /// runs (active), and ends with its score shown for <see cref="ResultSeconds"/> (done). Then the
    /// next one is announced, until <see cref="Stop"/>.</item>
    /// <item>A running round's clock stops while the robot is not ready (suspended): nothing it does
    /// then counts, and the data it would label is not being recorded as driving.</item>
    /// </list>
    /// What scores is the commanded wrist (the clutch's target after the IK), never the robot.
    /// </summary>
    public sealed class TrainingCourse
    {
        /// <summary>The fixed speed levels, rad/s; index <see cref="VariedLevel"/> is the varied one.</summary>
        public static readonly float[] SpeedLevels = { 1.0f, 2.5f, 3.8f, 6.0f };

        public const int LevelCount = 5;
        public const int VariedLevel = 4;
        public const int PurposeCount = 2;

        public const double CountdownSeconds = 3.0;
        public const double ResultSeconds = 3.0;

        public const int ReachTargets = 8;
        public const float ReachRadius = 0.025f;
        public const double ReachHoldSeconds = 0.4;
        public const double ReachRoundSeconds = 90.0;
        public const float ReachSpacing = 0.08f;
        public const float GripperOpenMax = 0.3f;
        public const float GripperClosedMin = 0.7f;

        public const float TraceRadius = 0.03f;
        public const double TraceRoundSeconds = 45.0;
        public const int TraceWaypoints = 5;
        public const int TracePoints = 96;

        public const float ShuttleRadius = 0.03f;
        public const double ShuttleRoundSeconds = 30.0;
        public const float ShuttleMinApart = 0.15f;
        public const float ShuttleMaxApart = 0.30f;

        /// <summary>In a varied round the level changes after this many seconds of driving, drawn uniformly.</summary>
        public const double VariedMinSeconds = 8.0, VariedMaxSeconds = 15.0;

        private readonly TrainingTargetSampler _sampler;
        private readonly Vector3[] _targets = new Vector3[ReachTargets];
        private readonly GripperGoal[] _goals = new GripperGoal[ReachTargets];
        private readonly Vector3[] _path = new Vector3[TracePoints];
        private readonly float[] _pathAlong = new float[TracePoints + 1];
        private readonly Vector3[] _waypoints = new Vector3[TraceWaypoints];
        private readonly float[] _coverage = new float[PurposeCount * LevelCount];

        private SeededRng _rng;
        private uint _courseId;
        private uint _round;
        private TrainingTask _task;
        private TrainingPhase _phase;
        private TrainingPurpose _roundPurpose;
        private int _level;
        private int _currentLevel;
        private double _phaseStart;
        private double _activeSeconds;
        private double _lastUpdate;
        private double _nextLevelChange;
        private int _targetCount;
        private int _activeTarget;
        private double _holdSeconds;
        private int _hits;
        private double _insideSeconds;
        private float _pathLength;
        private float _beadAlong;
        private float _score;
        private bool _disturbed;
        private bool _disturbedThisRound;

        /// <param name="minHeightMeters">The clutch's floor (the headset's <c>ClutchMinHeightMeters</c>).</param>
        public TrainingCourse(RobotArmProfile profile, float minHeightMeters)
        {
            _sampler = new TrainingTargetSampler(profile, minHeightMeters);
        }

        /// <summary>Whether the course is running (announcing, driving, or showing a result).</summary>
        public bool Running { get; private set; }

        /// <summary>What the next round's data is for; a running round keeps the purpose it started with.</summary>
        public TrainingPurpose Purpose { get; set; }

        public uint CourseId => _courseId;
        public uint Round => _round;
        public TrainingTask Task => _task;
        public TrainingPhase Phase => _phase;

        /// <summary>The round's level index (0-3 fixed, <see cref="VariedLevel"/> varied); -1 when stopped.</summary>
        public int Level => Running ? _level : -1;

        /// <summary>The speed the course wants sent now, rad/s; NaN when it is stopped (the slider decides).</summary>
        public float SpeedRadPerSec => Running ? SpeedLevels[_currentLevel] : float.NaN;

        /// <summary>The round's score so far: targets reached, touches, or the percentage of time on the bead.</summary>
        public float Score => _score;

        /// <summary>Seconds left in the current phase (countdown, round, or result), on the round's own clock while active.</summary>
        public double SecondsLeft { get; private set; }

        /// <summary>The state the headset reports (docs/adr/0024 decision 7).</summary>
        public TrainingState State => new TrainingState(Running ? _task : TrainingTask.None, Running ? _phase : TrainingPhase.Idle,
            Running ? _roundPurpose : Purpose, _disturbedThisRound, _courseId, _round,
            Running ? (_level == VariedLevel ? float.NaN : SpeedLevels[_level]) : float.NaN, _score);

        /// <summary>Reach: the targets, the one to go for now, and what each asks of the gripper.</summary>
        public int TargetCount => Running ? _targetCount : 0;
        public int ActiveTarget => _activeTarget;
        public Vector3 Target(int i) => _targets[i];
        public GripperGoal Goal(int i) => _goals[i];

        /// <summary>How long the wrist has been on the active target, 0..1 of the hold it needs.</summary>
        public float HoldProgress => (float)Math.Min(1.0, _holdSeconds / ReachHoldSeconds);

        /// <summary>Trace: the curve and the bead.</summary>
        public int PathCount => Running && _task == TrainingTask.Trace ? TracePoints : 0;
        public Vector3 PathPoint(int i) => _path[i];
        public Vector3 Bead => PointAlong(_beadAlong);

        /// <summary>Seconds driven (active) for a purpose and level, over every course so far.</summary>
        public float CoverageSeconds(TrainingPurpose purpose, int level) => _coverage[(int)purpose * LevelCount + level];

        /// <summary>The coverage counts, purpose-major (train levels 0-4, then test), for the host to persist.</summary>
        public ReadOnlySpan<float> Coverage => _coverage;

        /// <summary>Restores counts saved from <see cref="Coverage"/>; a wrong length or a bad value is ignored.</summary>
        public void LoadCoverage(ReadOnlySpan<float> counts)
        {
            if (counts.Length != _coverage.Length) return;
            for (int i = 0; i < counts.Length; i++)
            {
                if (!(counts[i] >= 0f) || float.IsInfinity(counts[i])) return;
            }

            counts.CopyTo(_coverage);
        }

        /// <summary>Starts the course: the first round is announced. <paramref name="courseId"/> seeds every draw and labels the rounds.</summary>
        public void Start(uint courseId, double now)
        {
            _courseId = courseId;
            _rng = new SeededRng(courseId == 0 ? 1UL : courseId);
            _round = 0;
            _task = TrainingTask.None;
            Running = true;
            _lastUpdate = now;
            Announce(now);
        }

        public void Stop()
        {
            Running = false;
            _phase = TrainingPhase.Idle;
            _task = TrainingTask.None;
            _targetCount = 0;
            _score = 0f;
            SecondsLeft = 0;
        }

        /// <summary>
        /// One display frame. <paramref name="wrist"/> is the commanded wrist (arm base frame),
        /// <paramref name="gripperFraction"/> the gripper command (0 open .. 1 closed),
        /// <paramref name="robotReady"/> whether commands reach a holding robot, and
        /// <paramref name="disturbed"/> whether the headset's network disturbance or added delay is on.
        /// </summary>
        public void Update(double now, Vector3 wrist, float gripperFraction, bool robotReady, bool disturbed)
        {
            double dt = Math.Max(0.0, Math.Min(0.25, now - _lastUpdate)); // a stall does not count as driving
            _lastUpdate = now;
            _disturbed = disturbed;
            if (!Running)
            {
                return;
            }

            switch (_phase)
            {
                case TrainingPhase.Idle:
                    SecondsLeft = Math.Max(0.0, CountdownSeconds - (now - _phaseStart));
                    if (SecondsLeft <= 0.0)
                    {
                        _phase = robotReady ? TrainingPhase.Active : TrainingPhase.Suspended;
                        _phaseStart = now;
                        _activeSeconds = 0;
                        _disturbedThisRound = disturbed;
                        _nextLevelChange = NextChange();
                    }

                    break;

                case TrainingPhase.Active:
                case TrainingPhase.Suspended:
                    _phase = robotReady ? TrainingPhase.Active : TrainingPhase.Suspended;
                    if (_phase == TrainingPhase.Active)
                    {
                        _disturbedThisRound |= disturbed;
                        Drive(dt, wrist, gripperFraction);
                    }

                    double length = RoundSeconds(_task);
                    SecondsLeft = Math.Max(0.0, length - _activeSeconds);
                    if (_activeSeconds >= length || (_task == TrainingTask.Reach && _hits >= _targetCount))
                    {
                        _phase = TrainingPhase.Done;
                        _phaseStart = now;
                        SecondsLeft = ResultSeconds;
                    }

                    break;

                case TrainingPhase.Done:
                    SecondsLeft = Math.Max(0.0, ResultSeconds - (now - _phaseStart));
                    if (SecondsLeft <= 0.0)
                    {
                        Announce(now);
                    }

                    break;
            }
        }

        private void Drive(double dt, Vector3 wrist, float gripper)
        {
            _activeSeconds += dt;
            _coverage[(int)_roundPurpose * LevelCount + _level] += (float)dt;
            if (_level == VariedLevel && _activeSeconds >= _nextLevelChange)
            {
                int next = (int)(_rng.NextDouble() * (SpeedLevels.Length - 1));
                _currentLevel = next >= _currentLevel ? next + 1 : next; // always a different level
                _nextLevelChange = _activeSeconds + NextChange();
            }

            switch (_task)
            {
                case TrainingTask.Reach:
                    if (_activeTarget < _targetCount && Vector3.Distance(wrist, _targets[_activeTarget]) <= ReachRadius &&
                        GripperOk(_goals[_activeTarget], gripper))
                    {
                        _holdSeconds += dt;
                        if (_holdSeconds >= ReachHoldSeconds)
                        {
                            _hits++;
                            _activeTarget++;
                            _holdSeconds = 0;
                        }
                    }
                    else
                    {
                        _holdSeconds = 0;
                    }

                    _score = _hits;
                    break;

                case TrainingTask.Trace:
                    _beadAlong = (float)((_activeSeconds * BeadSpeed(SpeedLevels[_currentLevel])) % _pathLength);
                    if (Vector3.Distance(wrist, PointAlong(_beadAlong)) <= TraceRadius)
                    {
                        _insideSeconds += dt;
                    }

                    _score = _activeSeconds > 0 ? (float)(100.0 * _insideSeconds / _activeSeconds) : 0f;
                    break;

                case TrainingTask.Shuttle:
                    if (Vector3.Distance(wrist, _targets[_activeTarget]) <= ShuttleRadius)
                    {
                        _hits++;
                        _activeTarget = 1 - _activeTarget;
                    }

                    _score = _hits;
                    break;
            }
        }

        /// <summary>The bead's speed along a trace, m/s: faster at a faster level, within what the arm follows comfortably.</summary>
        public static float BeadSpeed(float levelRadPerSec) => 0.05f + 0.03f * levelRadPerSec;

        public static double RoundSeconds(TrainingTask task) => task switch
        {
            TrainingTask.Reach => ReachRoundSeconds,
            TrainingTask.Trace => TraceRoundSeconds,
            TrainingTask.Shuttle => ShuttleRoundSeconds,
            _ => 0.0,
        };

        private static bool GripperOk(GripperGoal goal, float fraction) =>
            goal == GripperGoal.Any || (goal == GripperGoal.Open ? fraction <= GripperOpenMax : fraction >= GripperClosedMin);

        private double NextChange() => VariedMinSeconds + _rng.NextDouble() * (VariedMaxSeconds - VariedMinSeconds);

        /// <summary>The next round: task in turn, the least-covered level for the purpose, fresh targets.</summary>
        private void Announce(double now)
        {
            _round++;
            _task = _task == TrainingTask.Reach ? TrainingTask.Trace : _task == TrainingTask.Trace ? TrainingTask.Shuttle : TrainingTask.Reach;
            _roundPurpose = Purpose;
            _level = LeastCovered(_roundPurpose);
            _currentLevel = _level == VariedLevel ? (int)(_rng.NextDouble() * SpeedLevels.Length) : _level;
            _phase = TrainingPhase.Idle;
            _phaseStart = now;
            SecondsLeft = CountdownSeconds;
            _activeSeconds = 0;
            _hits = 0;
            _holdSeconds = 0;
            _insideSeconds = 0;
            _activeTarget = 0;
            _score = 0f;
            _beadAlong = 0f;
            _disturbedThisRound = _disturbed;

            bool built = _task switch
            {
                TrainingTask.Reach => BuildReach(),
                TrainingTask.Trace => BuildTrace(),
                _ => BuildShuttle(),
            };
            if (!built)
            {
                // No room for this task (a profile with a tiny envelope): a reach always fits.
                _task = TrainingTask.Reach;
                BuildReach();
            }
        }

        private int LeastCovered(TrainingPurpose purpose)
        {
            int best = 0;
            for (int i = 1; i < LevelCount; i++)
            {
                if (_coverage[(int)purpose * LevelCount + i] < _coverage[(int)purpose * LevelCount + best]) best = i;
            }

            return best;
        }

        private bool BuildReach()
        {
            _targetCount = 0;
            Vector3? last = null;
            for (int i = 0; i < ReachTargets; i++)
            {
                if (!_sampler.TrySample(ref _rng, out Vector3 t, last, ReachSpacing)) break;
                _targets[_targetCount] = t;
                double g = _rng.NextDouble();
                _goals[_targetCount] = g < 0.5 ? GripperGoal.Any : g < 0.75 ? GripperGoal.Open : GripperGoal.Closed;
                _targetCount++;
                last = t;
            }

            return _targetCount > 0;
        }

        private bool BuildShuttle()
        {
            for (int attempt = 0; attempt < 40; attempt++)
            {
                if (!_sampler.TrySample(ref _rng, out Vector3 a)) return false;
                for (int k = 0; k < 20; k++)
                {
                    if (!_sampler.TrySample(ref _rng, out Vector3 b, a, ShuttleMinApart)) break;
                    if (Vector3.Distance(a, b) <= ShuttleMaxApart)
                    {
                        _targets[0] = a;
                        _targets[1] = b;
                        _goals[0] = _goals[1] = GripperGoal.Any;
                        _targetCount = 2;
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// A closed Catmull-Rom curve through reachable waypoints taken in order of azimuth, kept only
        /// if the arm can follow it all the way round.
        /// </summary>
        private bool BuildTrace()
        {
            for (int attempt = 0; attempt < 30; attempt++)
            {
                if (!_sampler.TrySample(ref _rng, out _waypoints[0])) return false;
                int n = 1;
                for (int k = 0; k < 200 && n < TraceWaypoints; k++)
                {
                    if (!_sampler.TrySample(ref _rng, out Vector3 w, _waypoints[n - 1], 0.06f, 50)) break;
                    if (Vector3.Distance(w, _waypoints[0]) > 0.22f) continue; // one compact loop, not the whole workspace
                    _waypoints[n++] = w;
                }

                if (n < 3) continue;
                SortByAzimuth(n);
                for (int i = 0; i < TracePoints; i++)
                {
                    float u = (float)i / TracePoints * n;
                    int seg = (int)u;
                    float s = u - seg;
                    _path[i] = CatmullRom(_waypoints[(seg + n - 1) % n], _waypoints[seg % n], _waypoints[(seg + 1) % n], _waypoints[(seg + 2) % n], s);
                }

                _pathAlong[0] = 0f;
                for (int i = 1; i <= TracePoints; i++)
                {
                    _pathAlong[i] = _pathAlong[i - 1] + Vector3.Distance(_path[i - 1], _path[i % TracePoints]);
                }

                _pathLength = _pathAlong[TracePoints];
                if (_pathLength < 0.1f) continue;
                if (_sampler.IsFollowable(_path))
                {
                    _targetCount = 0;
                    return true;
                }
            }

            return false;
        }

        private void SortByAzimuth(int n)
        {
            Vector3 c = Vector3.Zero;
            for (int i = 0; i < n; i++) c += _waypoints[i];
            c /= n;
            for (int i = 1; i < n; i++)
            {
                Vector3 w = _waypoints[i];
                float a = Azimuth(w, c);
                int j = i - 1;
                while (j >= 0 && Azimuth(_waypoints[j], c) > a)
                {
                    _waypoints[j + 1] = _waypoints[j];
                    j--;
                }

                _waypoints[j + 1] = w;
            }
        }

        // Around the loop's centre, in the plane facing the arm: yaw (as an arc at the centre's radius)
        // spreads the points sideways and height up and down, so ordering by this angle gives a loop,
        // not a figure of eight.
        private static float Azimuth(Vector3 p, Vector3 c)
        {
            float radius = MathF.Sqrt(c.X * c.X + c.Y * c.Y);
            return MathF.Atan2(p.Z - c.Z, (MathF.Atan2(p.Y, p.X) - MathF.Atan2(c.Y, c.X)) * radius);
        }

        private Vector3 PointAlong(float along)
        {
            if (_pathLength <= 0f) return _path[0];
            int i = 0;
            while (i < TracePoints - 1 && _pathAlong[i + 1] < along) i++;
            float span = _pathAlong[i + 1] - _pathAlong[i];
            float s = span > 0f ? (along - _pathAlong[i]) / span : 0f;
            return Vector3.Lerp(_path[i], _path[(i + 1) % TracePoints], s);
        }

        private static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            float t2 = t * t, t3 = t2 * t;
            return 0.5f * (2f * p1 + (p2 - p0) * t + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 + (3f * p1 - p0 - 3f * p2 + p3) * t3);
        }
    }
}
