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

        /// <summary>
        /// Motors whose value is not in radians (a gripper's 0..1 fraction) but still moves at the
        /// servo's speed: each moves at the joint speed times its <see cref="SpeedScales"/> entry
        /// (units per radian). Parallel arrays; may be empty.
        /// </summary>
        public byte[] ScaledMotorIds { get; }

        public float[] SpeedScales { get; }

        /// <summary>The furthest past the last measurement a prediction extrapolates.</summary>
        public long MaxHorizonTicks { get; }

        public long TicksPerSecond { get; }

        // The JetRover relay's gate (robot/ros servo_controller.py setPos, and
        // Teleop.RobotHost GenericArmPlantConfig.Default), as the defaults for relay-replay. Core
        // cannot reference either, so the numbers are restated here, once, with where they come from.

        /// <summary>The JetRover's pulse units per radian: 1000 pulses span 240 degrees (GenericArmPlantConfig.PulsePerRadian).</summary>
        public const double JetRoverPulsesPerRadian = 1000.0 / (240.0 * System.Math.PI / 180.0);

        /// <summary>setPos's MIN_MOVE_DURATION_SECONDS.</summary>
        public const float DefaultRelayMinMoveSeconds = 0.05f;

        /// <summary>setPos's MIN_MEANINGFUL_PULSE_DELTA (5 pulses), in radians: about 0.021.</summary>
        public const float DefaultRelayDeadbandRad = (float)(5.0 / JetRoverPulsesPerRadian);

        /// <summary>GenericArmPlantConfig.Default.PulsesPerSecond (2200), in rad/s: about 9.2.</summary>
        public const float DefaultRelayMoveSpeedRadPerSec = (float)(2200.0 / JetRoverPulsesPerRadian);

        /// <summary>
        /// relay-replay: the shortest move the relay writes (s). Every move lasts
        /// max(this, distance / <see cref="RelayMoveSpeedRadPerSec"/>), and the servo takes no new
        /// command for that long (its write cooldown).
        /// </summary>
        public float RelayMinMoveSeconds { get; }

        /// <summary>relay-replay: a target nearer than this (rad) to the last one written is dropped.</summary>
        public float RelayDeadbandRad { get; }

        /// <summary>
        /// relay-replay: the speed (rad/s) the relay sizes a move's duration by, when a command asks
        /// for none or for more. Distance is from the last target written, not from where the servo is.
        /// </summary>
        public float RelayMoveSpeedRadPerSec { get; }

        /// <summary>
        /// relay-replay: how much older than its frame's sensed stamp each joint's reading is, because
        /// the robot reads its servos one after another and stamps the frame after the last read.
        /// Parallel arrays; empty (the default) takes every reading as of its stamp. Seconds.
        /// </summary>
        public byte[] SensedAgeMotorIds { get; }

        public float[] SensedAgeSeconds { get; }

        // The servo behind the relay's gate, for relay-replay-servo only: no other predictor reads
        // these. Every default is a guess until it is fitted from recordings (trainer
        // `armpred sysid`, docs/adr/0021).

        /// <summary>
        /// relay-replay-servo only: ADR 0018's guess of the unloaded servo's speed, about the HTD-35H's
        /// no-load speed (rad/s). A guess until fitted from recordings (trainer <c>armpred sysid</c>,
        /// docs/adr/0021).
        /// </summary>
        public const float DefaultServoMaxSpeedRadPerSec = 5.0f;

        /// <summary>
        /// relay-replay-servo only: the servo's lag behind its setpoint (s). A guess until fitted from
        /// recordings (trainer <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public const float DefaultServoLagSeconds = 0.02f;

        /// <summary>
        /// relay-replay-servo only: the time between a command's estimated arrival and the relay offering
        /// it to a servo (s): RobotHost's loop (about 2.5 ms on average), relay_node's poll (about 10 ms)
        /// and the driver's cycle (about 10 ms). A guess until fitted from recordings (trainer
        /// <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public const float DefaultServoPipelineDelaySeconds = 0.02f;

        /// <summary>
        /// relay-replay-servo only: the driver remembers its last pulse, so a servo's first move is sized
        /// from where it was measured. A guess until checked against recordings (trainer <c>armpred sysid</c>,
        /// docs/adr/0021).
        /// </summary>
        public const bool DefaultServoFirstMoveFromMeasured = true;

        /// <summary>
        /// relay-replay-servo only: the servo filter's step (s), a tenth of the default lag. A numerical
        /// choice rather than a property of the servo, and a guess too: whether it is fine enough depends
        /// on the lag, which is a guess until fitted from recordings (trainer <c>armpred sysid</c>,
        /// docs/adr/0021).
        /// </summary>
        public const float DefaultServoStepSeconds = 0.002f;

        /// <summary>
        /// relay-replay-servo only: the fastest the servo moves toward the relay's setpoint (rad/s,
        /// times a scaled joint's <see cref="SpeedScales"/> entry). Positive; +infinity is no cap. The
        /// default is a guess until fitted from recordings (trainer <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public float ServoMaxSpeedRadPerSec { get; }

        /// <summary>
        /// relay-replay-servo only: the time constant of the servo's first-order lag behind the relay's
        /// setpoint (s). Zero or positive, finite; zero is no lag. The default is a guess until fitted
        /// from recordings (trainer <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public float ServoLagSeconds { get; }

        /// <summary>
        /// relay-replay-servo only: how long after its effective time a command reaches the relay's gate
        /// (s), a delay the host's send + RTT/2 does not see. Zero or positive, finite. The default is a
        /// guess until fitted from recordings (trainer <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public float ServoPipelineDelaySeconds { get; }

        /// <summary>
        /// relay-replay-servo only: when true, a servo with no target written yet takes its sensed
        /// measurement as the last target written, so the deadband and the first move's duration are
        /// measured from it; when false, or with no sensed measurement, the first move lasts the minimum,
        /// as in relay-replay. Whether the real driver behaves so is a guess until checked against
        /// recordings (trainer <c>armpred sysid</c>, docs/adr/0021).
        /// </summary>
        public bool ServoFirstMoveFromMeasured { get; }

        /// <summary>
        /// relay-replay-servo only: the step the servo filter is integrated at (s), rounded to whole
        /// ticks, at least one. Positive, finite. A numerical choice; whether the default is fine enough
        /// depends on the lag, a guess until fitted from recordings (trainer <c>armpred sysid</c>,
        /// docs/adr/0021).
        /// </summary>
        public float ServoStepSeconds { get; }

        public JointPredictorConfig(
            int maxJoints, int commandHistoryCapacity, float maxJointSpeedRadPerSec,
            byte[] instantMotorIds, long maxHorizonTicks, long ticksPerSecond,
            byte[]? scaledMotorIds = null, float[]? speedScales = null,
            float relayMinMoveSeconds = DefaultRelayMinMoveSeconds,
            float relayDeadbandRad = DefaultRelayDeadbandRad,
            float relayMoveSpeedRadPerSec = DefaultRelayMoveSpeedRadPerSec,
            byte[]? sensedAgeMotorIds = null, float[]? sensedAgeSeconds = null,
            float servoMaxSpeedRadPerSec = DefaultServoMaxSpeedRadPerSec,
            float servoLagSeconds = DefaultServoLagSeconds,
            float servoPipelineDelaySeconds = DefaultServoPipelineDelaySeconds,
            bool servoFirstMoveFromMeasured = DefaultServoFirstMoveFromMeasured,
            float servoStepSeconds = DefaultServoStepSeconds,
            Contracts.IInferenceBackend? model = null,
            TrajectoryShaperConfig? shaper = null,
            LearnedChainLayout? chainLayout = null)
        {
            if (!(servoMaxSpeedRadPerSec > 0f)) throw new System.ArgumentOutOfRangeException(nameof(servoMaxSpeedRadPerSec), "the servo speed must be positive (+infinity is no cap)");
            if (!(servoLagSeconds >= 0f) || float.IsInfinity(servoLagSeconds)) throw new System.ArgumentOutOfRangeException(nameof(servoLagSeconds), "the servo lag must be zero or positive, and finite");
            if (!(servoPipelineDelaySeconds >= 0f) || float.IsInfinity(servoPipelineDelaySeconds)) throw new System.ArgumentOutOfRangeException(nameof(servoPipelineDelaySeconds), "the pipeline delay must be zero or positive, and finite");
            if (!(servoStepSeconds > 0f) || float.IsInfinity(servoStepSeconds)) throw new System.ArgumentOutOfRangeException(nameof(servoStepSeconds), "the servo step must be positive and finite");

            sensedAgeMotorIds ??= new byte[0];
            sensedAgeSeconds ??= new float[0];
            if (sensedAgeMotorIds.Length != sensedAgeSeconds.Length) throw new System.ArgumentException("sensedAgeMotorIds and sensedAgeSeconds must be the same length");
            foreach (float age in sensedAgeSeconds)
            {
                if (!(age >= 0f)) throw new System.ArgumentOutOfRangeException(nameof(sensedAgeSeconds), "every sensed age must be zero or positive");
            }

            if (!(relayMinMoveSeconds >= 0f)) throw new System.ArgumentOutOfRangeException(nameof(relayMinMoveSeconds));
            if (!(relayDeadbandRad >= 0f)) throw new System.ArgumentOutOfRangeException(nameof(relayDeadbandRad));
            if (!(relayMoveSpeedRadPerSec > 0f)) throw new System.ArgumentOutOfRangeException(nameof(relayMoveSpeedRadPerSec));

            scaledMotorIds ??= new byte[0];
            speedScales ??= new float[0];
            if (scaledMotorIds.Length != speedScales.Length) throw new System.ArgumentException("scaledMotorIds and speedScales must be the same length");
            foreach (float scale in speedScales)
            {
                if (!(scale > 0f)) throw new System.ArgumentOutOfRangeException(nameof(speedScales), "every speed scale must be positive");
            }

            if (maxJoints <= 0) throw new System.ArgumentOutOfRangeException(nameof(maxJoints));
            if (commandHistoryCapacity <= 0) throw new System.ArgumentOutOfRangeException(nameof(commandHistoryCapacity));
            if (!(maxJointSpeedRadPerSec > 0f)) throw new System.ArgumentOutOfRangeException(nameof(maxJointSpeedRadPerSec));
            if (maxHorizonTicks < 0) throw new System.ArgumentOutOfRangeException(nameof(maxHorizonTicks));
            if (ticksPerSecond <= 0) throw new System.ArgumentOutOfRangeException(nameof(ticksPerSecond));

            MaxJoints = maxJoints;
            CommandHistoryCapacity = commandHistoryCapacity;
            MaxJointSpeedRadPerSec = maxJointSpeedRadPerSec;
            InstantMotorIds = (byte[])(instantMotorIds ?? new byte[0]).Clone();
            ScaledMotorIds = (byte[])scaledMotorIds.Clone();
            SpeedScales = (float[])speedScales.Clone();
            MaxHorizonTicks = maxHorizonTicks;
            TicksPerSecond = ticksPerSecond;
            RelayMinMoveSeconds = relayMinMoveSeconds;
            RelayDeadbandRad = relayDeadbandRad;
            RelayMoveSpeedRadPerSec = relayMoveSpeedRadPerSec;
            SensedAgeMotorIds = (byte[])sensedAgeMotorIds.Clone();
            SensedAgeSeconds = (float[])sensedAgeSeconds.Clone();
            ServoMaxSpeedRadPerSec = servoMaxSpeedRadPerSec;
            ServoLagSeconds = servoLagSeconds;
            ServoPipelineDelaySeconds = servoPipelineDelaySeconds;
            ServoFirstMoveFromMeasured = servoFirstMoveFromMeasured;
            ServoStepSeconds = servoStepSeconds;
            Model = model;
            Shaper = shaper;
            ChainLayout = chainLayout;
        }

        /// <summary>
        /// The learned model a <c>learned</c> predictor runs (docs/adr/0025); null for every other
        /// predictor. Loaded once by the host and shared, never reloaded on the frame path.
        /// </summary>
        public Contracts.IInferenceBackend? Model { get; }

        /// <summary>
        /// The robot's trajectory shaper (docs/adr/0027 decision 1), for <c>franka-replay</c>; null for every
        /// other predictor. The host builds it from the robot's profile, the same one its backend shapes with.
        /// </summary>
        public TrajectoryShaperConfig? Shaper { get; }

        /// <summary>
        /// A serial-chain robot's rows for layout v4 (docs/adr/0028), for <c>learned</c> with an FR3 model; null for the
        /// JetRover's layout v3. With it, <c>learned</c> needs <see cref="Shaper"/> too: its prior is <c>franka-replay</c>.
        /// </summary>
        public LearnedChainLayout? ChainLayout { get; }

        /// <summary>This config with <paramref name="model"/> as its <see cref="Model"/>; every other value the same.</summary>
        public JointPredictorConfig WithModel(Contracts.IInferenceBackend? model) =>
            new JointPredictorConfig(MaxJoints, CommandHistoryCapacity, MaxJointSpeedRadPerSec, InstantMotorIds, MaxHorizonTicks,
                TicksPerSecond, ScaledMotorIds, SpeedScales, RelayMinMoveSeconds, RelayDeadbandRad, RelayMoveSpeedRadPerSec,
                SensedAgeMotorIds, SensedAgeSeconds, ServoMaxSpeedRadPerSec, ServoLagSeconds, ServoPipelineDelaySeconds,
                ServoFirstMoveFromMeasured, ServoStepSeconds, model, Shaper, ChainLayout);

        /// <summary>This config with <paramref name="shaper"/> as its <see cref="Shaper"/>; every other value the same.</summary>
        public JointPredictorConfig WithShaper(TrajectoryShaperConfig? shaper) =>
            new JointPredictorConfig(MaxJoints, CommandHistoryCapacity, MaxJointSpeedRadPerSec, InstantMotorIds, MaxHorizonTicks,
                TicksPerSecond, ScaledMotorIds, SpeedScales, RelayMinMoveSeconds, RelayDeadbandRad, RelayMoveSpeedRadPerSec,
                SensedAgeMotorIds, SensedAgeSeconds, ServoMaxSpeedRadPerSec, ServoLagSeconds, ServoPipelineDelaySeconds,
                ServoFirstMoveFromMeasured, ServoStepSeconds, Model, shaper, ChainLayout);

        /// <summary>This config with <paramref name="layout"/> as its <see cref="ChainLayout"/>; every other value the same.</summary>
        public JointPredictorConfig WithChainLayout(LearnedChainLayout? layout) =>
            new JointPredictorConfig(MaxJoints, CommandHistoryCapacity, MaxJointSpeedRadPerSec, InstantMotorIds, MaxHorizonTicks,
                TicksPerSecond, ScaledMotorIds, SpeedScales, RelayMinMoveSeconds, RelayDeadbandRad, RelayMoveSpeedRadPerSec,
                SensedAgeMotorIds, SensedAgeSeconds, ServoMaxSpeedRadPerSec, ServoLagSeconds, ServoPipelineDelaySeconds,
                ServoFirstMoveFromMeasured, ServoStepSeconds, Model, Shaper, layout);
    }
}
