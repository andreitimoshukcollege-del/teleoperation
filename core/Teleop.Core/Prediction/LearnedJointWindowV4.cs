using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// The learned joint model's input for a serial-chain robot, layout v4 (trainer/contract/MODEL_FORMAT.md; docs/adr/0028
    /// decision 1): a C# copy of <c>trainer/armpred/features/layout_v4.py</c>'s <c>FeatureState</c>, built call by call
    /// from what an <see cref="Contracts.ICommandAwarePredictor"/> is given. The trainer is the source;
    /// <c>core/testdata/learned/golden-v4.json</c> pins this copy to it (<c>armpred golden-learned-v4</c>).
    ///
    /// <para>One row of <see cref="JointFeatures"/> per motor in the <see cref="LearnedChainLayout"/>'s order, then one block
    /// of <see cref="ArmFeatures"/> per arm, the last 101 the network block, zero until a network model exists. Values are
    /// computed in double and stored as float, as the Python does. A hand is its 0..1 fraction ×
    /// <see cref="HandModelPerFraction"/> in model units.</para>
    ///
    /// <para>The prior is a <see cref="FrankaReplayPredictor"/> on the config's shaper, owned and fed by this window.
    /// Layout v3 (<see cref="LearnedJointWindow"/>, the JetRover's) is a separate class and is not touched.
    /// Allocation-free after construction.</para>
    /// </summary>
    public sealed class LearnedJointWindowV4
    {
        public const string LayoutId = "v4";
        public const double HandModelPerFraction = 1.0;
        public const int PathPoints = 80;
        public const double PathStepSeconds = 0.02;
        public const double PathAheadSeconds = 0.2;
        public const int Readings = 16;
        public const int Innovations = 8;
        public const double TimeScaleSeconds = 0.5;
        public const double AngleScaleRad = 0.05;
        public const double InnovationScaleRad = 0.005;
        public const double SlopeWindowSeconds = 0.04;
        public const double SlopeScaleRadPerSec = 2.0;
        public const int CommandRing = 96;
        public const int WindowFull = 64;
        public const int FreshRing = 128;
        public const double FreshRateHz = 50.0;
        public const double FallbackStalenessSeconds = 1.0;
        public const int Identity = LearnedChainLayout.ArmJoints + 1;

        public const int JointFeatures = 2 + PathPoints * 2 + Readings * 3 + Innovations * 4 + Identity + 2 + 1;
        public const int BaseArmFeatures = 2 * LearnedChainLayout.ArmJoints + 3 + 7;
        public const int NetworkFeatures = LearnedJointWindow.NetworkFeatures;
        public const int ArmFeatures = BaseArmFeatures + NetworkFeatures;

        /// <summary>The hand's identity entry in a row: where the model's head reads which cap applies.</summary>
        public const int HandFlagIndex = 2 + PathPoints * 2 + Readings * 3 + Innovations * 4 + Identity - 1;

        private readonly LearnedChainLayout _layout;
        private readonly FrankaReplayPredictor _prior;
        private readonly long _tps;
        private readonly int _rows;

        private readonly long[] _cmdEff = new long[CommandRing];
        private readonly double[] _cmdValue;
        private readonly bool[] _cmdHas;
        private int _cmdHead, _cmdCount;
        private bool _hasCommand;
        private long _lastCommandEff;

        private readonly long[] _readT;
        private readonly double[] _readV;
        private readonly int[] _readHead, _readCount;
        private readonly long[] _innT;
        private readonly double[] _innE;
        private readonly long[] _innH;
        private readonly int[] _innHead, _innCount;

        private readonly long[] _fresh = new long[FreshRing];
        private int _freshHead, _freshCount;
        private bool _hasFresh;
        private long _lastFresh;

        private readonly double[] _newest;
        private readonly bool[] _hasNewest;

        private readonly JointSample[] _scratch;
        private readonly double[] _now, _before;
        private readonly bool[] _hasNow, _hasBefore;
        private readonly int[] _inForce = new int[PathPoints];
        private readonly int[] _handRow;

        public LearnedJointWindowV4(JointPredictorConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _layout = config.ChainLayout ?? throw new ArgumentException("layout v4 needs the robot's rows (JointPredictorConfig.ChainLayout)", nameof(config));
            _prior = new FrankaReplayPredictor(config);
            _tps = config.TicksPerSecond;
            _rows = _layout.Rows;
            _cmdValue = new double[CommandRing * _rows];
            _cmdHas = new bool[CommandRing * _rows];
            _readT = new long[_rows * Readings];
            _readV = new double[_rows * Readings];
            _readHead = new int[_rows];
            _readCount = new int[_rows];
            _innT = new long[_rows * Innovations];
            _innE = new double[_rows * Innovations];
            _innH = new long[_rows * Innovations];
            _innHead = new int[_rows];
            _innCount = new int[_rows];
            _newest = new double[_rows];
            _hasNewest = new bool[_rows];
            _scratch = new JointSample[config.MaxJoints];
            _now = new double[_rows];
            _before = new double[_rows];
            _hasNow = new bool[_rows];
            _hasBefore = new bool[_rows];
            _handRow = new int[_layout.Arms];
            for (int a = 0; a < _layout.Arms; a++)
            {
                _handRow[a] = -1;
                for (int r = 0; r < _rows; r++) if (_layout.ArmOf(r) == a && _layout.IsHand(r)) _handRow[a] = r;
            }

            Reset();
        }

        public LearnedChainLayout Layout => _layout;

        public FrankaReplayPredictor Prior => _prior;

        public int InputLength => _rows * JointFeatures + _layout.Arms * ArmFeatures;

        public int Rows => _rows;

        public void Reset()
        {
            _prior.Reset();
            _cmdHead = _cmdCount = 0;
            _hasCommand = false;
            _lastCommandEff = 0;
            Array.Clear(_readHead, 0, _rows);
            Array.Clear(_readCount, 0, _rows);
            Array.Clear(_innHead, 0, _rows);
            Array.Clear(_innCount, 0, _rows);
            _freshHead = _freshCount = 0;
            _hasFresh = false;
            _lastFresh = 0;
            Array.Clear(_hasNewest, 0, _rows);
        }

        public double ToModel(int row, double value) => _layout.IsHand(row) ? value * HandModelPerFraction : value;

        public double ToWire(int row, double value) => _layout.IsHand(row) ? value / HandModelPerFraction : value;

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets)
        {
            _prior.ObserveCommand(effectiveTicks, targets, 0f);
            if (_hasCommand && effectiveTicks < _lastCommandEff)
            {
                return;
            }

            _hasCommand = true;
            _lastCommandEff = effectiveTicks;
            int slot = (_cmdHead + _cmdCount) % CommandRing;
            if (_cmdCount == CommandRing) _cmdHead = (_cmdHead + 1) % CommandRing;
            else _cmdCount++;
            _cmdEff[slot] = effectiveTicks;
            for (int r = 0; r < _rows; r++) _cmdHas[slot * _rows + r] = false;
            for (int i = 0; i < targets.Length; i++)
            {
                int r = _layout.RowOf(targets[i].MotorId);
                if (r < 0) continue;
                _cmdHas[slot * _rows + r] = true;
                _cmdValue[slot * _rows + r] = targets[i].Value;
            }
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            bool fresh = !_hasFresh || sensedTicks > _lastFresh;
            if (fresh)
            {
                if (_prior.TryPredict(sensedTicks, _scratch, out int n))
                {
                    long horizon = _prior.HasState ? sensedTicks - _prior.StateTicks : 0;
                    for (int i = 0; i < joints.Length; i++)
                    {
                        int r = _layout.RowOf(joints[i].MotorId);
                        if (r < 0 || !joints[i].Sensed) continue;
                        int p = Find(_scratch, n, joints[i].MotorId);
                        if (p < 0) continue;
                        double error = ToModel(r, joints[i].Value) - ToModel(r, _scratch[p].Value);
                        int at = Push(_innHead, _innCount, r, Innovations);
                        _innT[at] = sensedTicks;
                        _innE[at] = error;
                        _innH[at] = horizon;
                    }
                }

                for (int i = 0; i < joints.Length; i++)
                {
                    int r = _layout.RowOf(joints[i].MotorId);
                    if (r < 0 || !joints[i].Sensed) continue;
                    double value = ToModel(r, joints[i].Value);
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
        /// Builds the window for <paramref name="targetTicks"/> into <paramref name="x"/> (<see cref="InputLength"/> floats)
        /// and the prior's prediction per row, in model units, into <paramref name="prior"/>. False when the prior cannot
        /// predict or an arm joint has no fresh reading yet; <paramref name="valid"/> false when the newest reading is more
        /// than <see cref="FallbackStalenessSeconds"/> old. Either way the caller falls back to the prior.
        /// </summary>
        public bool TryBuild(long targetTicks, Span<float> x, Span<double> prior, out bool valid)
        {
            valid = false;
            if (x.Length < InputLength || prior.Length < _rows) return false;
            if (!Predict(targetTicks, _now, _hasNow) || !_hasFresh) return false;
            for (int r = 0; r < _rows; r++) if (!_layout.IsHand(r) && !_hasNewest[r]) return false;
            if (!Predict(targetTicks - (long)(SlopeWindowSeconds * _tps), _before, _hasBefore))
            {
                Array.Clear(_hasBefore, 0, _rows);
            }

            x.Slice(0, InputLength).Clear();
            double tps = _tps;
            double staleness = (targetTicks - _lastFresh) / tps;

            long ahead = (long)Math.Round(PathAheadSeconds * tps);
            int c = _cmdCount - 1;
            for (int i = 0; i < PathPoints; i++)
            {
                long grid = targetTicks + ahead - (long)Math.Round(i * PathStepSeconds * tps);
                while (c >= 0 && Eff(c) > grid) c--;
                _inForce[i] = c;
            }

            for (int row = 0; row < _rows; row++)
            {
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
                    while (ci >= 0 && !_cmdHas[Slot(ci) * _rows + row]) ci--;
                    if (ci >= 0)
                    {
                        f[k] = (float)Squash(ToModel(row, _cmdValue[Slot(ci) * _rows + row]) - v, AngleScaleRad);
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

                f[k + _layout.IdentityOf(row)] = 1f;
                k += Identity;
                f[k] = (float)Math.Sin(v);
                f[k + 1] = (float)Math.Cos(v);
                f[k + 2] = sensed ? 1f : 0f;
            }

            double windowStart = targetTicks - 1.0 * tps, recentStart = targetTicks - 2.0 * tps;
            int inLastSecond = 0, recent = 0;
            long maxGap = 0, previous = 0;
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

            for (int a = 0; a < _layout.Arms; a++)
            {
                Span<float> block = x.Slice(_rows * JointFeatures + a * ArmFeatures, ArmFeatures);
                int k = 0;
                for (int r = 0; r < _rows; r++)
                {
                    if (_layout.ArmOf(r) != a || _layout.IsHand(r)) continue;
                    block[k] = (float)Math.Sin(_newest[r]);
                    block[k + 1] = (float)Math.Cos(_newest[r]);
                    k += 2;
                }

                k = 2 * LearnedChainLayout.ArmJoints;
                for (int axis = 0; axis < 3; axis++) block[k + axis] = (float)_layout.Gravity(a, axis);
                k += 3;
                block[k] = (float)(staleness / TimeScaleSeconds);
                block[k + 1] = (float)(inLastSecond / FreshRateHz);
                block[k + 2] = (float)(maxGap / tps / TimeScaleSeconds);
                if (_cmdCount > 0)
                {
                    block[k + 3] = (float)((Eff(_cmdCount - 1) - targetTicks) / tps / TimeScaleSeconds);
                    int hand = _handRow[a];
                    if (hand >= 0)
                    {
                        int newest = -1, oldest = -1, old = -1;
                        double cutoff = targetTicks - 0.5 * tps;
                        for (int i = 0; i < _cmdCount; i++)
                        {
                            if (!_cmdHas[Slot(i) * _rows + hand]) continue;
                            if (oldest < 0) oldest = i;
                            newest = i;
                            if (Eff(i) <= cutoff) old = i;
                        }

                        if (newest >= 0)
                        {
                            double last = _cmdValue[Slot(newest) * _rows + hand];
                            block[k + 4] = (float)last;
                            block[k + 5] = (float)(last - _cmdValue[Slot(old >= 0 ? old : oldest) * _rows + hand]);
                        }
                    }

                    block[k + 6] = (float)(Math.Min(_cmdCount, WindowFull) / (double)WindowFull);
                }
            }

            valid = staleness <= FallbackStalenessSeconds;
            return true;
        }

        private bool Predict(long ticks, double[] values, bool[] has)
        {
            Array.Clear(has, 0, _rows);
            if (!_prior.TryPredict(ticks, _scratch, out int n)) return false;
            for (int i = 0; i < n; i++)
            {
                int r = _layout.RowOf(_scratch[i].MotorId);
                if (r < 0) continue;
                values[r] = ToModel(r, _scratch[i].Value);
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
