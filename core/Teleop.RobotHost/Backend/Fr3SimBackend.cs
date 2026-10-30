using System;
using Teleop.Core.Prediction;
using Teleop.Core.Types;

namespace Teleop.RobotHost.Backend
{
    /// <summary>How the simulated FR3 tracks its shaped trajectory (docs/adr/0027 decision 3).</summary>
    public sealed class Fr3SimOptions
    {
        /// <summary>The arm's tracking behind the shaped trajectory: a second-order lag of this natural frequency (Hz).</summary>
        public double TrackHz { get; set; } = 20;

        public double TrackDamping { get; set; } = 1.0;

        /// <summary>Gaussian noise on every reading (radians; fractions for a hand), seeded. Off by default.</summary>
        public double NoiseRad { get; set; }

        public ulong Seed { get; set; } = 1;

        /// <summary>The most simulated time one catch-up runs (s); a host stalled for longer skips the rest, and it is counted.</summary>
        public double MaxCatchUpSeconds { get; set; } = 1.0;

        /// <summary>Semi-implicit Euler at 1 ms stays accurate below omega * dt = 0.5.</summary>
        public const double MaxTrackHz = 80;

        public string? Validate() =>
            !(TrackHz > 0 && TrackHz <= MaxTrackHz) ? $"the tracking frequency must be in (0, {MaxTrackHz}] Hz" :
            !(TrackDamping > 0) ? "the tracking damping must be positive" :
            !(NoiseRad >= 0) ? "the reading noise must not be negative" :
            !(MaxCatchUpSeconds > 0) ? "the catch-up limit must be positive" : null;
    }

    /// <summary>
    /// A simulated FR3 Duo in-process (docs/adr/0027 decision 3): a stand-in for the hardware, so the live loop
    /// (headset, RobotHost, joint-state downlink, recording) runs end to end before a robot exists. It is never
    /// the robot: its sessions say <c>"source": "sim-host"</c>.
    ///
    /// <para>On the host's clock it runs, in 1 ms steps up to the time it is asked about:</para>
    /// <list type="bullet">
    /// <item>Core's <see cref="JointTrajectoryShaper"/> toward the newest target, as the real arm server will;</item>
    /// <item>behind the shaper, the arm's tracking of it: a second-order lag per arm joint
    /// (<see cref="Fr3SimOptions.TrackHz"/>, <see cref="Fr3SimOptions.TrackDamping"/>), standing in for Franka's
    /// internal joint controller. The hands follow their gate exactly;</item>
    /// <item>readings of the tracked position with optional seeded noise, measured at the last whole step.</item>
    /// </list>
    /// <para>Disabled, every target becomes where the shaper has the motor, so the arm comes to rest under the
    /// limits. The truth for a recording (<see cref="ReadTruth"/>) is the tracked position without noise.</para>
    /// </summary>
    public sealed class Fr3SimBackend : IArmBackend
    {
        private readonly TrajectoryShaperConfig _shaper;
        private readonly Fr3SimOptions _options;
        private readonly long _stepTicks;
        private readonly long _maxCatchUpTicks;
        private readonly double _omega;
        private readonly double _dt;

        private readonly ShapedJointState[] _shaped;
        private readonly double[] _target;
        private readonly double[] _tracked;
        private readonly double[] _trackedVelocity;
        private SeededRng _rng;
        private long _stepTicksDone;
        private bool _started;

        /// <param name="initial">Each motor's starting position, in the shaper's motor order.</param>
        public Fr3SimBackend(TrajectoryShaperConfig shaper, double[] initial, long ticksPerSecond, Fr3SimOptions? options = null)
        {
            _shaper = shaper ?? throw new ArgumentNullException(nameof(shaper));
            _options = options ?? new Fr3SimOptions();
            string? invalid = _options.Validate();
            if (invalid != null) throw new ArgumentException(invalid, nameof(options));
            if (initial == null || initial.Length != shaper.Count) throw new ArgumentException($"one starting position per motor ({shaper.Count})", nameof(initial));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));

            _stepTicks = Math.Max(1L, (long)Math.Round(shaper.StepSeconds * ticksPerSecond));
            _dt = _stepTicks / (double)ticksPerSecond;
            _maxCatchUpTicks = (long)(_options.MaxCatchUpSeconds * ticksPerSecond);
            _omega = 2 * Math.PI * _options.TrackHz;
            _rng = new SeededRng(_options.Seed);

            int n = shaper.Count;
            _shaped = new ShapedJointState[n];
            _target = new double[n];
            _tracked = new double[n];
            _trackedVelocity = new double[n];
            for (int m = 0; m < n; m++)
            {
                _shaped[m] = ShapedJointState.AtRest(initial[m]);
                _target[m] = initial[m];
                _tracked[m] = initial[m];
            }

            Enabled = true;
        }

        public bool Enabled { get; private set; }

        /// <summary>Simulated time skipped because the host fell more than the catch-up limit behind (ticks).</summary>
        public long SkippedTicks { get; private set; }

        /// <summary>Whole 1 ms steps simulated so far.</summary>
        public long Steps { get; private set; }

        /// <summary>
        /// Called every <see cref="TruthEverySteps"/> steps with the step's time and every motor's tracked position
        /// without noise, in the shaper's order: the recording's truth, spaced by the simulation's own steps
        /// rather than by the host's loop. Null: none.
        /// </summary>
        public Action<long, double[]>? TruthSink { get; set; }

        public int TruthEverySteps { get; set; } = 4;

        public void Send(ReadOnlySpan<JointSample> targets, long hostTicks)
        {
            Advance(hostTicks);
            if (!Enabled) return;
            for (int i = 0; i < targets.Length; i++)
            {
                int m = _shaper.IndexOf(targets[i].MotorId);
                if (m >= 0) _target[m] = targets[i].Value;
            }
        }

        public int ReadState(long hostTicks, Span<JointSample> output, out long sensedTicks)
        {
            Advance(hostTicks);
            sensedTicks = _stepTicksDone;
            int n = Math.Min(output.Length, _shaper.Count);
            for (int m = 0; m < n; m++)
            {
                double value = _tracked[m];
                if (_options.NoiseRad > 0) value += _options.NoiseRad * Gaussian();
                output[m] = new JointSample(_shaper[m].MotorId, (float)value, true);
            }

            return n;
        }

        /// <summary>The tracked positions without noise, as of the last whole step (the recording's truth).</summary>
        public int ReadTruth(long hostTicks, Span<double> output, out long sensedTicks)
        {
            Advance(hostTicks);
            sensedTicks = _stepTicksDone;
            int n = Math.Min(output.Length, _shaper.Count);
            for (int m = 0; m < n; m++) output[m] = _tracked[m];
            return n;
        }

        public void SetEnabled(bool enabled, long hostTicks)
        {
            Advance(hostTicks);
            if (!enabled)
            {
                for (int m = 0; m < _target.Length; m++) _target[m] = _shaped[m].Q;
            }

            Enabled = enabled;
        }

        private void Advance(long hostTicks)
        {
            if (!_started)
            {
                _started = true;
                _stepTicksDone = hostTicks;
                return;
            }

            if (hostTicks - _stepTicksDone > _maxCatchUpTicks)
            {
                long skip = hostTicks - _stepTicksDone - _maxCatchUpTicks;
                SkippedTicks += skip;
                _stepTicksDone += skip;
            }

            while (_stepTicksDone + _stepTicks <= hostTicks)
            {
                for (int m = 0; m < _shaped.Length; m++)
                {
                    ShapedJointLimits limits = _shaper[m];
                    JointTrajectoryShaper.Step(ref _shaped[m], _target[m], limits, _shaper);
                    if (limits.IsFinger)
                    {
                        _tracked[m] = _shaped[m].Q;
                        _trackedVelocity[m] = 0;
                        continue;
                    }

                    double accel = _omega * _omega * (_shaped[m].Q - _tracked[m]) - 2 * _options.TrackDamping * _omega * _trackedVelocity[m];
                    _trackedVelocity[m] += accel * _dt;
                    _tracked[m] += _trackedVelocity[m] * _dt;
                }

                _stepTicksDone += _stepTicks;
                Steps++;
                if (TruthSink != null && Steps % TruthEverySteps == 0) TruthSink(_stepTicksDone, _tracked);
            }
        }

        /// <summary>A standard normal draw (Box-Muller) from the seeded generator.</summary>
        private double Gaussian()
        {
            double u1 = 1.0 - _rng.NextDouble(), u2 = _rng.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }
    }
}
