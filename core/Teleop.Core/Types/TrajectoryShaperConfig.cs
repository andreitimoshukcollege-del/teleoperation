using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// One motor's limits for <see cref="Prediction.JointTrajectoryShaper"/> (docs/adr/0027 decision 1),
    /// restated from the robot's profile so Core needs no reference to it. An arm joint's units are radians.
    /// A finger's are the wire's 0..1 opening fraction, and it is driven through the shaper's finger gate
    /// rather than as a joint.
    /// </summary>
    public readonly struct ShapedJointLimits
    {
        public readonly byte MotorId;

        /// <summary>A gripper finger (a newest-wins gate at constant speed) rather than an arm joint.</summary>
        public readonly bool IsFinger;

        /// <summary>The hard limits: what the position-based velocity limit is measured to.</summary>
        public readonly double Lower;
        public readonly double Upper;

        /// <summary>The soft limits: targets are kept <see cref="TrajectoryShaperConfig.PositionMargin"/> inside them.</summary>
        public readonly double SoftLower;
        public readonly double SoftUpper;

        /// <summary>The robot's own limits (per second, per second squared, per second cubed); the shaper uses a fraction of each.</summary>
        public readonly double Velocity;
        public readonly double Acceleration;
        public readonly double Jerk;

        /// <summary>
        /// Franka's position-based velocity limit: toward a hard limit at distance d, at most
        /// <c>offset + sqrt(2 · deceleration · d)</c>. Deceleration 0 is none.
        /// </summary>
        public readonly double PositionBasedDeceleration;
        public readonly double PositionBasedVelocityOffset;

        /// <summary>A finger's speed (fractions per second) and its deadband (a fraction).</summary>
        public readonly double FingerSpeed;
        public readonly double FingerDeadband;

        private ShapedJointLimits(
            byte motorId, bool isFinger, double lower, double upper, double softLower, double softUpper,
            double velocity, double acceleration, double jerk, double positionBasedDeceleration, double positionBasedVelocityOffset,
            double fingerSpeed, double fingerDeadband)
        {
            MotorId = motorId;
            IsFinger = isFinger;
            Lower = lower;
            Upper = upper;
            SoftLower = softLower;
            SoftUpper = softUpper;
            Velocity = velocity;
            Acceleration = acceleration;
            Jerk = jerk;
            PositionBasedDeceleration = positionBasedDeceleration;
            PositionBasedVelocityOffset = positionBasedVelocityOffset;
            FingerSpeed = fingerSpeed;
            FingerDeadband = fingerDeadband;
        }

        /// <summary>An arm joint.</summary>
        public static ShapedJointLimits Joint(
            byte motorId, double lower, double upper, double softLower, double softUpper,
            double velocity, double acceleration, double jerk,
            double positionBasedDeceleration = 0, double positionBasedVelocityOffset = 0)
        {
            if (!(lower < upper)) throw new ArgumentException($"motor {motorId}: the lower limit must be below the upper");
            if (!(lower <= softLower && softLower < softUpper && softUpper <= upper)) throw new ArgumentException($"motor {motorId}: the soft limits must be inside the hard limits");
            if (!(velocity > 0) || !(acceleration > 0) || !(jerk > 0)) throw new ArgumentException($"motor {motorId}: velocity, acceleration and jerk must be positive");
            if (!(positionBasedDeceleration >= 0) || !(positionBasedVelocityOffset >= 0)) throw new ArgumentException($"motor {motorId}: the position-based velocity limit must not be negative");
            return new ShapedJointLimits(motorId, false, lower, upper, softLower, softUpper, velocity, acceleration, jerk,
                positionBasedDeceleration, positionBasedVelocityOffset, 0, 0);
        }

        /// <summary>A gripper finger, in the wire's 0..1 fraction.</summary>
        public static ShapedJointLimits Finger(byte motorId, double speedFractionPerSecond, double deadbandFraction)
        {
            if (!(speedFractionPerSecond > 0) || double.IsInfinity(speedFractionPerSecond)) throw new ArgumentException($"motor {motorId}: the finger speed must be positive and finite");
            if (!(deadbandFraction >= 0) || !(deadbandFraction < 1)) throw new ArgumentException($"motor {motorId}: the finger deadband must be in [0, 1)");
            return new ShapedJointLimits(motorId, true, 0, 1, 0, 1, speedFractionPerSecond, 0, 0, 0, 0, speedFractionPerSecond, deadbandFraction);
        }
    }

    /// <summary>
    /// How <see cref="Prediction.JointTrajectoryShaper"/> turns absolute targets into a smooth trajectory
    /// within a robot's limits (docs/adr/0027 decision 1): the step, the share of each limit it may use, and
    /// every motor's limits. One instance serves the robot's backend and the <c>franka-replay</c> prior,
    /// so both shape alike.
    /// </summary>
    public sealed class TrajectoryShaperConfig
    {
        /// <summary>The FR3's control period: 1 kHz.</summary>
        public const double DefaultStepSeconds = 0.001;

        /// <summary>Shares of the robot's velocity, acceleration and jerk limits the shaper may use, so a real robot never sees a command at its limit.</summary>
        public const double DefaultVelocityScale = 0.9;
        public const double DefaultAccelerationScale = 0.9;
        public const double DefaultJerkScale = 0.9;

        /// <summary>The share of the (scaled) acceleration a stop toward a target is planned with: the rest is room to follow the plan.</summary>
        public const double DefaultBrakingScale = 0.5;

        /// <summary>Within reach of the target, the approach becomes first order with this time constant (s), so it settles without chatter.</summary>
        public const double DefaultSettleSeconds = 0.02;

        /// <summary>Targets are kept this far inside the soft limits (radians).</summary>
        public const double DefaultPositionMargin = 0.02;

        public double StepSeconds { get; }
        public double VelocityScale { get; }
        public double AccelerationScale { get; }
        public double JerkScale { get; }
        public double BrakingScale { get; }
        public double SettleSeconds { get; }
        public double PositionMargin { get; }

        private readonly ShapedJointLimits[] _joints;

        public TrajectoryShaperConfig(
            ShapedJointLimits[] joints,
            double stepSeconds = DefaultStepSeconds,
            double velocityScale = DefaultVelocityScale,
            double accelerationScale = DefaultAccelerationScale,
            double jerkScale = DefaultJerkScale,
            double brakingScale = DefaultBrakingScale,
            double settleSeconds = DefaultSettleSeconds,
            double positionMargin = DefaultPositionMargin)
        {
            if (joints == null) throw new ArgumentNullException(nameof(joints));
            if (!(stepSeconds > 0) || double.IsInfinity(stepSeconds)) throw new ArgumentOutOfRangeException(nameof(stepSeconds));
            if (!(velocityScale > 0 && velocityScale <= 1)) throw new ArgumentOutOfRangeException(nameof(velocityScale), "a share of the limit: (0, 1]");
            if (!(accelerationScale > 0 && accelerationScale <= 1)) throw new ArgumentOutOfRangeException(nameof(accelerationScale), "a share of the limit: (0, 1]");
            if (!(jerkScale > 0 && jerkScale <= 1)) throw new ArgumentOutOfRangeException(nameof(jerkScale), "a share of the limit: (0, 1]");
            if (!(brakingScale > 0 && brakingScale <= 1)) throw new ArgumentOutOfRangeException(nameof(brakingScale), "a share of the acceleration: (0, 1]");
            if (!(settleSeconds > stepSeconds) || double.IsInfinity(settleSeconds)) throw new ArgumentOutOfRangeException(nameof(settleSeconds), "must be longer than one step, and finite");
            if (!(positionMargin >= 0) || double.IsInfinity(positionMargin)) throw new ArgumentOutOfRangeException(nameof(positionMargin));

            for (int i = 0; i < joints.Length; i++)
            {
                for (int k = 0; k < i; k++)
                {
                    if (joints[k].MotorId == joints[i].MotorId) throw new ArgumentException($"motor {joints[i].MotorId} is listed twice", nameof(joints));
                }
            }

            _joints = (ShapedJointLimits[])joints.Clone();
            StepSeconds = stepSeconds;
            VelocityScale = velocityScale;
            AccelerationScale = accelerationScale;
            JerkScale = jerkScale;
            BrakingScale = brakingScale;
            SettleSeconds = settleSeconds;
            PositionMargin = positionMargin;
        }

        public int Count => _joints.Length;

        public ShapedJointLimits this[int index] => _joints[index];

        /// <summary>The index of <paramref name="motorId"/>'s limits, or -1.</summary>
        public int IndexOf(byte motorId)
        {
            for (int i = 0; i < _joints.Length; i++)
            {
                if (_joints[i].MotorId == motorId) return i;
            }

            return -1;
        }
    }
}
