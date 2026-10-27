using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// The learned joint model's input, layout v3 (trainer/contract/MODEL_FORMAT.md; docs/adr/0025): a
    /// C# copy of <c>trainer/armpred/features/layout.py</c>'s <c>FeatureState</c>, built call by call
    /// from exactly what an <see cref="Contracts.ICommandAwarePredictor"/> is given. The trainer is the
    /// source; <c>core/testdata/learned/golden-v3.json</c> pins this copy to it, and a change to either
    /// side changes both and regenerates the golden (<c>armpred golden-learned</c>).
    ///
    /// <para>Per predicted joint (motors 1-5, then the gripper, 10), 251 values; then 114 arm-wide values,
    /// the last 101 the network block, which stays zero until a network model exists (the
    /// <c>absent</c> forecast level). Values are computed in double and stored as float, as the
    /// Python does; angles are in model units (radians; the gripper in servo radians of travel, its
    /// 0..1 fraction / <see cref="GripperFractionPerRadian"/>).</para>
    ///
    /// <para>The prior is a <see cref="RelayReplayPredictor"/> run speed-blind (it is passed 0 as the
    /// requested speed), owned and fed by this window. Allocation-free after construction.</para>
    /// </summary>
    public sealed class LearnedJointWindow
    {
        public static readonly byte[] Motors = { 1, 2, 3, 4, 5, 10 };
        public const int JointCount = 6;
        public const int ArmJointCount = 5;
        public const byte GripperMotor = 10;
        public const double GripperFractionPerRadian = 0.4513;

        public const int PathPoints = 80;
        public const double PathStepSeconds = 0.02;
        public const double PathAheadSeconds = 0.2;
        public const int Readings = 16;
        public const int Innovations = 8;
        public const double TimeScaleSeconds = 0.5;
        public const double AngleScaleRad = 0.2;
        public const double InnovationScaleRad = 0.05;
        public const double SlopeWindowSeconds = 0.04;
        public const double SlopeScaleRadPerSec = 2.0;
        public const int CommandRing = 96;
        public const int WindowFull = 64;
        public const int FreshRing = 64;
        public const double FallbackStalenessSeconds = 1.0;

        public const int JointFeatures = 2 + PathPoints * 2 + Readings * 3 + Innovations * 4 + JointCount + 2 + 1;
        public const int BaseArmFeatures = 13;
        public const int NetworkFeatures = 2 * 9 * (4 + 1) + 2 + 2 + 6 + 1;
        public const int ArmFeatures = BaseArmFeatures + NetworkFeatures;
        public const int InputLength = JointCount * JointFeatures + ArmFeatures;

        private static readonly byte[] GravityMotors = { 2, 3, 4 };

        private readonly RelayReplayPredictor _prior;
        private readonly long _tps;

        // Commands, oldest first: effective time, and the predicted motors' targets (wire units).
        private readonly long[] _cmdEff = new long[CommandRing];
        private readonly double[] _cmdValue = new double[CommandRing * JointCount];
        private readonly bool[] _cmdHas = new bool[CommandRing * JointCount];
        private int _cmdHead, _cmdCount;
        private bool _hasCommand;
        private long _lastCommandEff;

        // Per motor, oldest first, model units.
        private readonly long[] _readT = new long[JointCount * Readings];
        private readonly double[] _readV = new double[JointCount * Readings];
        private readonly int[] _readHead = new int[JointCount], _readCount = new int[JointCount];
        private readonly long[] _innT = new long[JointCount * Innovations];
        private readonly double[] _innE = new double[JointCount * Innovations];
        private readonly long[] _innH = new long[JointCount * Innovations];
        private readonly int[] _innHead = new int[JointCount], _innCount = new int[JointCount];

        private readonly long[] _fresh = new long[FreshRing];
        private int _freshHead, _freshCount;
        private bool _hasFresh;
        private long _lastFresh;

        private readonly double[] _newest = new double[JointCount];
        private readonly bool[] _hasNewest = new bool[JointCount];

        private readonly JointSample[] _scratch;
        private readonly double[] _now = new double[JointCount], _before = new double[JointCount];
        private readonly bool[] _hasNow = new bool[JointCount], _hasBefore = new bool[JointCount];
        private readonly int[] _inForce = new int[PathPoints];

        public LearnedJointWindow(JointPredictorConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _prior = new RelayReplayPredictor(config);
            _tps = config.TicksPerSecond;
            _scratch = new JointSample[config.MaxJoints];
            Reset();
        }

        /// <summary>The prior, for its diagnostics and its own prediction (the fallback).</summary>
        public RelayReplayPredictor Prior => _prior;

        public void Reset()
        {
            _prior.Reset();
            _cmdHead = _cmdCount = 0;
            _hasCommand = false;
            _lastCommandEff = 0;
            Array.Clear(_readHead, 0, JointCount);
            Array.Clear(_readCount, 0, JointCount);
            Array.Clear(_innHead, 0, JointCount);
            Array.Clear(_innCount, 0, JointCount);
            _freshHead = _freshCount = 0;
            _hasFresh = false;
            _lastFresh = 0;
            Array.Clear(_hasNewest, 0, JointCount);
        }

        public static int RowOf(byte motor)
        {
            for (int i = 0; i < JointCount; i++) if (Motors[i] == motor) return i;
            return -1;
        }

        public static double ToModel(byte motor, double value) => motor == GripperMotor ? value / GripperFractionPerRadian : value;

        public static double ToWire(byte motor, double value) => motor == GripperMotor ? value * GripperFractionPerRadian : value;

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets)
        {
            _prior.ObserveCommand(effectiveTicks, targets, 0f);
            if (_hasCommand && effectiveTicks < _lastCommandEff)
            {
                return; // the prior rejected it; so does the window
            }

            _hasCommand = true;
            _lastCommandEff = effectiveTicks;
            int slot = (_cmdHead + _cmdCount) % CommandRing;
            if (_cmdCount == CommandRing)
            {
                _cmdHead = (_cmdHead + 1) % CommandRing;
            }
            else
            {
                _cmdCount++;
            }

            _cmdEff[slot] = effectiveTicks;
            for (int r = 0; r < JointCount; r++) _cmdHas[slot * JointCount + r] = false;
            for (int i = 0; i < targets.Length; i++)
            {
                int r = RowOf(targets[i].MotorId);
                if (r < 0) continue;
                _cmdHas[slot * JointCount + r] = true;
                _cmdValue[slot * JointCount + r] = targets[i].Value;
            }
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            bool fresh = !_hasFresh || sensedTicks > _lastFresh;
            if (fresh)
            {
                // The prior's error on this reading, before it sees it.
                if (_prior.TryPredict(sensedTicks, _scratch, out int n))
                {
                    long horizon = _prior.HasState ? sensedTicks - _prior.StateTicks : 0;
                    for (int i = 0; i < joints.Length; i++)
                    {
                        int r = RowOf(joints[i].MotorId);
                        if (r < 0 || !joints[i].Sensed) continue;
                        int p = Find(_scratch, n, joints[i].MotorId);
                        if (p < 0) continue;
                        double error = ToModel(joints[i].MotorId, joints[i].Value) - ToModel(joints[i].MotorId, _scratch[p].Value);
                        int at = Push(_innHead, _innCount, r, Innovations);
                        _innT[at] = sensedTicks;
                        _innE[at] = error;
                        _innH[at] = horizon;
                    }
                }

                for (int i = 0; i < joints.Length; i++)
                {
                    int r = RowOf(joints[i].MotorId);
                    if (r < 0 || !joints[i].Sensed) continue;
                    double value = ToModel(joints[i].MotorId, joints[i].Value);
                    int at = Push(_readHead, _readCount, r, Readings);
                    _readT[at] = sensedTicks;
                    _readV[at] = value;
                    _newest[r] = value;
                    _hasNewest[r] = true;
                }

                _hasFresh = true;
                _lastFresh = sensedTicks;
                int f = (_freshHead + _freshCount) % FreshRing;
                if (_freshCount == FreshRing) _freshHead = (_freshHead + 1) % FreshRing;
                else _freshCount++;
                _fresh[f] = sensedTicks;
            }

            _prior.ObserveState(sensedTicks, joints);
        }

        /// <summary>
        /// Builds the window for <paramref name="targetTicks"/> into <paramref name="x"/>
        /// (<see cref="InputLength"/> floats) and the prior's prediction there, per predicted joint in
        /// model units, into <paramref name="prior"/>. False, writing nothing meaningful, when the prior
        /// cannot predict or an arm joint has no fresh reading yet. <paramref name="valid"/> is false when
        /// the newest reading is more than <see cref="FallbackStalenessSeconds"/> old: the caller falls
        /// back to the prior then.
        /// </summary>
        public bool TryBuild(long targetTicks, Span<float> x, Span<double> prior, out bool valid)
        {
            valid = false;
            if (x.Length < InputLength || prior.Length < JointCount) return false;
            if (!Predict(targetTicks, _now, _hasNow) || !_hasFresh) return false;
            for (int r = 0; r < ArmJointCount; r++) if (!_hasNewest[r]) return false;
            if (!Predict(targetTicks - (long)(SlopeWindowSeconds * _tps), _before, _hasBefore))
            {
                Array.Clear(_hasBefore, 0, JointCount);
            }

            x.Slice(0, InputLength).Clear();
            double tps = _tps;
            double staleness = (targetTicks - _lastFresh) / tps;

            // The commanded path's grid, newest first: T + PATH_AHEAD_S, then back every PATH_STEP_S; for
            // each point, the newest command at or before it.
            long ahead = (long)Math.Round(PathAheadSeconds * tps);
            int c = _cmdCount - 1;
            for (int i = 0; i < PathPoints; i++)
            {
                long grid = targetTicks + ahead - (long)Math.Round(i * PathStepSeconds * tps);
                while (c >= 0 && Eff(c) > grid) c--;
                _inForce[i] = c;
            }

            for (int row = 0; row < JointCount; row++)
            {
                byte m = Motors[row];
                bool sensed = _hasNewest[row];
                double v = sensed ? _newest[row] : (_hasNow[row] ? _now[row] : 0.0);
                double p = _hasNow[row] ? _now[row] : v;
                prior[row] = p;
                Span<float> f = x.Slice(row * JointFeatures, JointFeatures);
                int k = 0;
                f[k] = (float)Squash(p - v, AngleScaleRad);
                f[k + 1] = (float)((p - (_hasBefore[row] ? _before[row] : p)) / SlopeWindowSeconds / SlopeScaleRadPerSec);
                k += 2;
                for (int i = 0; i < PathPoints; i++)
                {
                    int ci = _inForce[i];
                    while (ci >= 0 && !_cmdHas[Slot(ci) * JointCount + row]) ci--;
                    if (ci >= 0)
                    {
                        f[k] = (float)Squash(ToModel(m, _cmdValue[Slot(ci) * JointCount + row]) - v, AngleScaleRad);
                        f[k + 1] = 1f;
                    }

                    k += 2;
                }

                int nr = _readCount[row];
                for (int i = 0; i < Readings; i++)
                {
                    if (i < nr)
                    {
                        int at = row * Readings + (_readHead[row] + nr - 1 - i) % Readings;
                        f[k] = (float)((_readT[at] - targetTicks) / tps / TimeScaleSeconds);
                        f[k + 1] = (float)Squash(_readV[at] - v, AngleScaleRad);
                        f[k + 2] = 1f;
                    }

                    k += 3;
                }

                int ni = _innCount[row];
                for (int i = 0; i < Innovations; i++)
                {
                    if (i < ni)
                    {
                        int at = row * Innovations + (_innHead[row] + ni - 1 - i) % Innovations;
                        f[k] = (float)((_innT[at] - targetTicks) / tps / TimeScaleSeconds);
                        f[k + 1] = (float)Squash(_innE[at], InnovationScaleRad);
                        f[k + 2] = (float)(_innH[at] / tps / TimeScaleSeconds);
                        f[k + 3] = 1f;
                    }

                    k += 4;
                }

                f[k + row] = 1f;
                k += JointCount;
                f[k] = (float)Math.Sin(v);
                f[k + 1] = (float)Math.Cos(v);
                f[k + 2] = sensed ? 1f : 0f;
            }

            Span<float> arm = x.Slice(JointCount * JointFeatures, ArmFeatures);
            int a = 0;
            for (int g = 0; g < GravityMotors.Length; g++)
            {
                double n = _newest[RowOf(GravityMotors[g])];
                arm[a] = (float)Math.Sin(n);
                arm[a + 1] = (float)Math.Cos(n);
                a += 2;
            }

            arm[a] = (float)(staleness / TimeScaleSeconds);
            double windowStart = targetTicks - 1.0 * tps, recentStart = targetTicks - 2.0 * tps;
            int inLastSecond = 0;
            long maxGap = 0, previous = 0;
            int recent = 0;
            for (int i = 0; i < _freshCount; i++)
            {
                long t = _fresh[(_freshHead + i) % FreshRing];
                if (t > targetTicks) continue;
                if (t >= windowStart) inLastSecond++;
                if (t >= recentStart)
                {
                    if (recent > 0) maxGap = Math.Max(maxGap, t - previous);
                    previous = t;
                    recent++;
                }
            }

            arm[a + 1] = (float)(inLastSecond / 20.0);
            arm[a + 2] = (float)(maxGap / tps / TimeScaleSeconds);
            if (_cmdCount > 0)
            {
                arm[a + 3] = (float)((Eff(_cmdCount - 1) - targetTicks) / tps / TimeScaleSeconds);
                int grip = RowOf(GripperMotor);
                int newest = -1, oldest = -1, old = -1;
                double cutoff = targetTicks - 0.5 * tps;
                for (int i = 0; i < _cmdCount; i++)
                {
                    if (!_cmdHas[Slot(i) * JointCount + grip]) continue;
                    if (oldest < 0) oldest = i;
                    newest = i;
                    if (Eff(i) <= cutoff) old = i;
                }

                if (newest >= 0)
                {
                    double last = _cmdValue[Slot(newest) * JointCount + grip];
                    arm[a + 4] = (float)last;
                    arm[a + 5] = (float)(last - _cmdValue[Slot(old >= 0 ? old : oldest) * JointCount + grip]);
                }

                arm[a + 6] = (float)(Math.Min(_cmdCount, WindowFull) / (double)WindowFull);
            }

            valid = staleness <= FallbackStalenessSeconds;
            return true;
        }

        private bool Predict(long ticks, double[] values, bool[] has)
        {
            Array.Clear(has, 0, JointCount);
            if (!_prior.TryPredict(ticks, _scratch, out int n)) return false;
            for (int i = 0; i < n; i++)
            {
                int r = RowOf(_scratch[i].MotorId);
                if (r < 0) continue;
                values[r] = ToModel(_scratch[i].MotorId, _scratch[i].Value);
                has[r] = true;
            }

            return true;
        }

        private static double Squash(double x, double scale) => Math.Asinh(x / scale);

        private int Slot(int i) => (_cmdHead + i) % CommandRing;

        private long Eff(int i) => _cmdEff[Slot(i)];

        private static int Push(int[] head, int[] count, int row, int capacity)
        {
            int at;
            if (count[row] == capacity)
            {
                at = row * capacity + head[row];
                head[row] = (head[row] + 1) % capacity;
            }
            else
            {
                at = row * capacity + (head[row] + count[row]) % capacity;
                count[row]++;
            }

            return at;
        }

        private static int Find(JointSample[] samples, int n, byte motor)
        {
            for (int i = 0; i < n; i++) if (samples[i].MotorId == motor) return i;
            return -1;
        }
    }
}
