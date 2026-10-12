using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Predicts a robot's joint-space state now from a stale measurement <i>and the commands sent
    /// since</i> (docs/adr/0018). The operator knows every target it sent and when; for a servo arm
    /// that is a stronger signal than the measured trajectory alone, which is all an
    /// <see cref="IPredictor{TState}"/> sees.
    ///
    /// <para>All ticks are in one clock domain (the operator's). The host converts the robot's
    /// sensed time with <c>ClockSync</c>, and passes each command's <i>effective</i> time: when it is
    /// expected to reach the robot (send time plus the estimated uplink delay). The same
    /// requirements as any predictor apply (Prediction/CLAUDE.md): deterministic, robust to
    /// out-of-order and duplicate input and to gaps, allocation-free, config-driven.</para>
    /// </summary>
    public interface ICommandAwarePredictor
    {
        /// <summary>
        /// A command took (or will take) effect at <paramref name="effectiveTicks"/>.
        /// <paramref name="speedRadPerSec"/> is the speed the robot was asked to move at; 0 means it
        /// was not asked, so it moves at the robot's own default.
        /// </summary>
        void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec);

        /// <summary>The robot's joints as measured at <paramref name="sensedTicks"/>.</summary>
        void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints);

        /// <summary>
        /// The predicted joints at <paramref name="targetTicks"/>, one per measured joint, in the
        /// order last observed. False until a state has been observed. Returns the count written.
        /// </summary>
        bool TryPredict(long targetTicks, Span<JointSample> output, out int count);

        void Reset();

        PredictorDiagnostics Diagnostics { get; }
    }
}
