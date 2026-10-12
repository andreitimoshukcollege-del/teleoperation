using System;
using Teleop.Core.Contracts;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// <c>hold-last</c> (docs/adr/0018): the last measured joints, unchanged, ignoring commands. The
    /// joint-space "no prediction" baseline every <see cref="ICommandAwarePredictor"/> is reported
    /// against (docs/metrics.md §8), as <c>none</c> is for pose predictors.
    /// </summary>
    public sealed class HoldLastJointPredictor : ICommandAwarePredictor
    {
        private readonly JointSample[] _last;
        private int _count;
        private bool _hasState;
        private long _stateTicks;
        private int _accepted;
        private int _rejected;
        private long _lastHorizon;

        public HoldLastJointPredictor(JointPredictorConfig config)
        {
            if (config == null) throw new ArgumentNullException(nameof(config));
            _last = new JointSample[config.MaxJoints];
            Reset();
        }

        public PredictorDiagnostics Diagnostics =>
            new PredictorDiagnostics(_lastHorizon, _hasState ? _stateTicks : 0, _accepted, _rejected);

        public void Reset()
        {
            _count = 0;
            _hasState = false;
            _stateTicks = 0;
            _accepted = 0;
            _rejected = 0;
            _lastHorizon = 0;
        }

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec)
        {
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            if (_hasState && sensedTicks < _stateTicks)
            {
                _rejected++;
                return;
            }

            _count = Math.Min(joints.Length, _last.Length);
            joints.Slice(0, _count).CopyTo(_last);
            _stateTicks = sensedTicks;
            _hasState = true;
            _accepted++;
        }

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count)
        {
            count = 0;
            if (!_hasState)
            {
                return false;
            }

            _lastHorizon = Math.Max(0, targetTicks - _stateTicks);
            count = Math.Min(_count, output.Length);
            _last.AsSpan(0, count).CopyTo(output);
            return true;
        }
    }
}
