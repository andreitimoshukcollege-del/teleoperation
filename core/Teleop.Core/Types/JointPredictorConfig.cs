// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Parameters for an <see cref="Contracts.ICommandAwarePredictor"/> (docs/adr/0018). Ticks are in
    /// the predictor's clock domain (the operator's).
    /// </summary>
    public sealed class JointPredictorConfig
    {
        /// <summary>The most joints a state or command may carry; arrays are sized to it once.</summary>
        public int MaxJoints { get; }

        /// <summary>How many commands are remembered for replay.</summary>
        public int CommandHistoryCapacity { get; }

        /// <summary>The fastest an arm joint moves toward its target (rad/s): the servo model's one parameter.</summary>
        public float MaxJointSpeedRadPerSec { get; }

        /// <summary>Motors whose value is a fraction that follows its command at once (grippers). May be empty.</summary>
        public byte[] InstantMotorIds { get; }

        /// <summary>The furthest past the last measurement a prediction extrapolates.</summary>
        public long MaxHorizonTicks { get; }

        public long TicksPerSecond { get; }

        public JointPredictorConfig(
            int maxJoints, int commandHistoryCapacity, float maxJointSpeedRadPerSec,
            byte[] instantMotorIds, long maxHorizonTicks, long ticksPerSecond)
        {
            if (maxJoints <= 0) throw new System.ArgumentOutOfRangeException(nameof(maxJoints));
            if (commandHistoryCapacity <= 0) throw new System.ArgumentOutOfRangeException(nameof(commandHistoryCapacity));
            if (!(maxJointSpeedRadPerSec > 0f)) throw new System.ArgumentOutOfRangeException(nameof(maxJointSpeedRadPerSec));
            if (maxHorizonTicks < 0) throw new System.ArgumentOutOfRangeException(nameof(maxHorizonTicks));
            if (ticksPerSecond <= 0) throw new System.ArgumentOutOfRangeException(nameof(ticksPerSecond));

            MaxJoints = maxJoints;
            CommandHistoryCapacity = commandHistoryCapacity;
            MaxJointSpeedRadPerSec = maxJointSpeedRadPerSec;
            InstantMotorIds = (byte[])(instantMotorIds ?? new byte[0]).Clone();
            MaxHorizonTicks = maxHorizonTicks;
            TicksPerSecond = ticksPerSecond;
        }
    }
}
