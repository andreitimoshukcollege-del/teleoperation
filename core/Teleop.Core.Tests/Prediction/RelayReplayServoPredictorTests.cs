using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class RelayReplayServoPredictorTests
{
    /// <summary>1000 ticks per second: one tick is 1 ms, so the default 2 ms servo step is 2 ticks.</summary>
    private const long Tps = 1000;
    private const byte Arm = 1, Gripper = 10;
    private const float Speed = 2f;          // the relay's move speed, rad/s: a 1 rad move lasts 500 ms
    private const float MinMove = 0.05f;     // s: 50 ms
    private const float Deadband = 0.02f;    // rad
    private const float NoCap = float.PositiveInfinity;

    /// <summary>The relay-replay tests' config, with the servo parameters neutral unless given.</summary>
    private static JointPredictorConfig Config(float servoSpeed = NoCap, float lag = 0f, float delay = 0f,
        bool firstMoveFromMeasured = false, float step = 0.002f, long maxHorizonMs = 1000, int capacity = 64,
        float minMove = MinMove, byte[]? ageIds = null, float[]? ageSeconds = null,
        byte[]? scaledIds = null, float[]? scales = null, byte[]? instantIds = null) =>
        new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: capacity, maxJointSpeedRadPerSec: 5f,
            instantMotorIds: instantIds ?? new[] { Gripper }, maxHorizonTicks: maxHorizonMs, ticksPerSecond: Tps,
            scaledMotorIds: scaledIds, speedScales: scales,
            relayMinMoveSeconds: minMove, relayDeadbandRad: Deadband, relayMoveSpeedRadPerSec: Speed,
            sensedAgeMotorIds: ageIds, sensedAgeSeconds: ageSeconds,
            servoMaxSpeedRadPerSec: servoSpeed, servoLagSeconds: lag, servoPipelineDelaySeconds: delay,
            servoFirstMoveFromMeasured: firstMoveFromMeasured, servoStepSeconds: step);

    private static JointSample[] State(float arm, float gripper) =>
        new[] { new JointSample(Arm, arm, true), new JointSample(Gripper, gripper, false) };

    private static JointSample[] Command(float arm, float gripper) =>
        new[] { new JointSample(Arm, arm, false), new JointSample(Gripper, gripper, false) };

    private static float Predict(ICommandAwarePredictor p, long at, byte motor)
    {
        var output = new JointSample[8];
        Assert.True(p.TryPredict(at, output, out int count));
        for (int i = 0; i < count; i++)
        {
            if (output[i].MotorId == motor) return output[i].Value;
        }

        throw new Xunit.Sdk.XunitException($"motor {motor} not predicted");
    }

    private static RelayReplayPredictor.GateCounts GateOf(ICommandAwarePredictor p) => p switch
    {
        RelayReplayPredictor r => r.Gate,
        RelayReplayServoPredictor s => s.Gate,
        _ => throw new Xunit.Sdk.XunitException($"{p.GetType().Name} has no gate"),
    };

    /// <summary>A measurement of 0 at 0 and a first command to 0 at 0: the servo is settled at 0 by 50 ms.</summary>
    private static T Settled<T>(T p) where T : ICommandAwarePredictor
    {
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(0f, 0f), 0f);
        return p;
    }

    // ------------------------------------------------------------------------------------------------
    // (1) The identity: at neutral parameters, relay-replay bit for bit.

    /// <summary>
    /// Configs that exercise every path of the gate and the replay: reading ages, a scaled non-instant
    /// gripper, an instant one, a ring that wraps, no minimum move and no deadband, and servo steps of one
    /// tick, a non-divisor and one longer than the horizon (only the partial step runs).
    /// </summary>
    private static JointPredictorConfig Variant(int variant, float servoSpeed, float lag, float delay, bool firstMove)
    {
        byte[] arm = { 1, 2, 3, 4, 5 };
        switch (variant)
        {
            case 0: // the headset's: a scaled gripper that goes through the gate, reading ages
                return new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 6f,
                    instantMotorIds: new byte[0], maxHorizonTicks: 500_000_000L, ticksPerSecond: 1_000_000_000L,
                    scaledMotorIds: new[] { Gripper }, speedScales: new[] { 0.358f },
                    sensedAgeMotorIds: arm, sensedAgeSeconds: new[] { 0.045f, 0.035f, 0.025f, 0.015f, 0.005f },
                    servoMaxSpeedRadPerSec: servoSpeed, servoLagSeconds: lag, servoPipelineDelaySeconds: delay,
                    servoFirstMoveFromMeasured: firstMove);
            case 1: // a small ring and an instant gripper, steps of one tick
                return new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 4, maxJointSpeedRadPerSec: 5f,
                    instantMotorIds: new[] { Gripper }, maxHorizonTicks: 300L, ticksPerSecond: Tps,
                    relayMinMoveSeconds: 0.04f, relayDeadbandRad: 0.01f, relayMoveSpeedRadPerSec: 7f,
                    servoMaxSpeedRadPerSec: servoSpeed, servoLagSeconds: lag, servoPipelineDelaySeconds: delay,
                    servoFirstMoveFromMeasured: firstMove, servoStepSeconds: 0.001f);
            case 2: // no minimum move, no deadband, a step that does not divide the horizon
                return new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 16, maxJointSpeedRadPerSec: 5f,
                    instantMotorIds: new[] { Gripper }, maxHorizonTicks: 1000L, ticksPerSecond: Tps,
                    relayMinMoveSeconds: 0f, relayDeadbandRad: 0f, relayMoveSpeedRadPerSec: 3f,
                    sensedAgeMotorIds: new byte[] { 1, 3 }, sensedAgeSeconds: new[] { 0.03f, 0.011f },
                    servoMaxSpeedRadPerSec: servoSpeed, servoLagSeconds: lag, servoPipelineDelaySeconds: delay,
                    servoFirstMoveFromMeasured: firstMove, servoStepSeconds: 0.0137f);
            default: // a step longer than the horizon
                return new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 5f,
                    instantMotorIds: new[] { Gripper }, maxHorizonTicks: 400_000L, ticksPerSecond: 1_000_000L,
                    sensedAgeMotorIds: arm, sensedAgeSeconds: new[] { 0.04f, 0.03f, 0.02f, 0.01f, 0f },
                    servoMaxSpeedRadPerSec: servoSpeed, servoLagSeconds: lag, servoPipelineDelaySeconds: delay,
                    servoFirstMoveFromMeasured: firstMove, servoStepSeconds: 2f);
        }
    }

    public static IEnumerable<object[]> VariantsAndSeeds()
    {
        for (int variant = 0; variant < 4; variant++)
        {
            for (ulong seed = 1; seed <= 6; seed++) yield return new object[] { variant, seed };
        }
    }

    [Theory]
    [MemberData(nameof(VariantsAndSeeds))]
    public void AtNeutralParameters_EveryOutputIsBitIdenticalToRelayReplay(int variant, ulong seed)
    {
        JointPredictorConfig config = Variant(variant, NoCap, 0f, 0f, false);
        var relay = new RelayReplayPredictor(config);
        var servo = new RelayReplayServoPredictor(config);
        int predictions = Drive(seed, config.TicksPerSecond, new ICommandAwarePredictor[] { relay, servo });

        Assert.True(predictions > 300, $"only {predictions} predictions compared");
        Assert.Equal(relay.Gate.Written, servo.Gate.Written);
        Assert.Equal(relay.Gate.DeadbandDrops, servo.Gate.DeadbandDrops);
        Assert.Equal(relay.Gate.Coalesced, servo.Gate.Coalesced);
        Assert.True(relay.Gate.Written > 0);
    }

    /// <summary>
    /// Drives every predictor with the same seeded call sequence and asserts that every prediction (ok,
    /// count, motor, value bits, sensed flag) and every diagnostic agrees across them. The sequence has
    /// commands that ask for speeds (or none, or more than the relay's), a few older commands and states
    /// (rejected), duplicated calls, commands for a subset of joints and for a motor the gate has no
    /// room for, states with unsensed arm joints in a changing order, a 700 ms gap with no state, and a
    /// Reset. Returns how many predictions were compared.
    /// </summary>
    private static int Drive(ulong seed, long tps, ICommandAwarePredictor[] predictors)
    {
        var rng = new SeededRng(seed);
        long ms = tps / 1000;
        byte[] motors = { 1, 2, 3, 4, 5, Gripper, 7 };
        var position = new float[motors.Length];
        var goal = new float[motors.Length];
        var outputs = new JointSample[predictors.Length][];
        for (int i = 0; i < predictors.Length; i++) outputs[i] = new JointSample[8];

        long start = 3_000 * ms;
        long nextCommand = start, nextState = start + 3 * ms, nextPredict = start + ms, nextGoal = start;
        long lastSensed = 0;
        float speed = 0f;
        int compared = 0;
        for (long now = start; now < start + 4_000 * ms; now += ms)
        {
            if (now == start + 2_500 * ms)
            {
                foreach (ICommandAwarePredictor p in predictors) p.Reset();
            }

            if (now >= nextGoal)
            {
                for (int m = 0; m < motors.Length; m++) goal[m] = (float)(rng.NextDouble() * 2.4 - 1.2);
                goal[5] = rng.NextDouble() < 0.5 ? 0f : 1f;
                double s = rng.NextDouble();
                speed = s < 0.3 ? 0f : s < 0.6 ? 2.5f : s < 0.85 ? 1.2f : 50f;
                nextGoal = now + (long)((150 + 750 * rng.NextDouble()) * ms);
            }

            if (now >= nextCommand)
            {
                for (int m = 0; m < motors.Length; m++) position[m] += 0.2f * (goal[m] - position[m]);
                double kind = rng.NextDouble();
                int count = kind < 0.1 ? 1 + (int)(rng.NextDouble() * 6) : kind < 0.13 ? 7 : 6;
                var targets = new JointSample[count];
                for (int m = 0; m < count; m++) targets[m] = new JointSample(motors[m], position[m], false);
                long effective = now + 4 * ms;
                if (rng.NextDouble() < 0.03) effective -= 60 * ms;        // older than the newest: rejected
                int repeats = rng.NextDouble() < 0.05 ? 2 : 1;            // a duplicate
                for (int r = 0; r < repeats; r++)
                {
                    foreach (ICommandAwarePredictor p in predictors) p.ObserveCommand(effective, targets, speed);
                }

                nextCommand = now + (long)((10 + 20 * rng.NextDouble()) * ms);
            }

            bool gap = now > start + 1_200 * ms && now < start + 1_900 * ms;
            if (now >= nextState && !gap)
            {
                long sensed = rng.NextDouble() < 0.6 || lastSensed == 0 ? now - 7 * ms : lastSensed;
                if (rng.NextDouble() < 0.04) sensed -= 150 * ms;          // an out-of-order frame
                int count = rng.NextDouble() < 0.1 ? 4 : 6;
                int rotate = rng.NextDouble() < 0.2 ? (int)(rng.NextDouble() * 6) : 0;
                var joints = new JointSample[count];
                for (int i = 0; i < count; i++)
                {
                    int m = (i + rotate) % 6;
                    bool isSensed = m < 5 && rng.NextDouble() < 0.85;
                    float value = position[m] - (float)(0.08 * (rng.NextDouble() - 0.5));
                    joints[i] = new JointSample(motors[m], value, isSensed);
                }

                int repeats = rng.NextDouble() < 0.05 ? 2 : 1;
                for (int r = 0; r < repeats; r++)
                {
                    foreach (ICommandAwarePredictor p in predictors) p.ObserveState(sensed, joints);
                }

                lastSensed = sensed;
                nextState = now + 20 * ms;
            }

            if (now >= nextPredict)
            {
                long target = now + (long)((rng.NextDouble() * 180.0 - 40.0) * ms);
                bool firstOk = predictors[0].TryPredict(target, outputs[0], out int firstCount);
                PredictorDiagnostics first = predictors[0].Diagnostics;
                for (int i = 1; i < predictors.Length; i++)
                {
                    string where = $"seed {seed}, t {target}, predictor {i}";
                    bool ok = predictors[i].TryPredict(target, outputs[i], out int count);
                    Assert.True(firstOk == ok, where);
                    Assert.True(firstCount == count, where);
                    for (int j = 0; j < count; j++)
                    {
                        JointSample a = outputs[0][j], b = outputs[i][j];
                        Assert.True(a.MotorId == b.MotorId && a.Sensed == b.Sensed, where);
                        Assert.True(BitConverter.SingleToInt32Bits(a.Value) == BitConverter.SingleToInt32Bits(b.Value),
                            $"{where}, motor {a.MotorId}: {a.Value:R} vs {b.Value:R}");
                    }

                    PredictorDiagnostics d = predictors[i].Diagnostics;
                    Assert.True(first.HorizonTicks == d.HorizonTicks && first.LastObservationTicks == d.LastObservationTicks
                        && first.AcceptedObservations == d.AcceptedObservations && first.RejectedObservations == d.RejectedObservations, where);
                }

                if (firstOk) compared++;
                nextPredict = now + (long)((3 + 12 * rng.NextDouble()) * ms);
            }
        }

        return compared;
    }

    [Fact]
    public void AtNeutralParameters_TheRegistrysTwoEntriesAgree()
    {
        var config = Config();
        ICommandAwarePredictor relay = Registries.JointPredictors["relay-replay"](config);
        ICommandAwarePredictor servo = Registries.JointPredictors["relay-replay-servo"](config);
        Assert.True(Drive(99, Tps, new[] { relay, servo }) > 300);
    }

    // ------------------------------------------------------------------------------------------------
    // The filter, against relay-replay as the oracle for the setpoint at every grid point.

    [Theory]
    [InlineData(1UL)]
    [InlineData(2UL)]
    [InlineData(3UL)]
    public void TheFilter_StepsOnRelayReplaysSetpoint_AtEveryGridPoint(ulong seed)
    {
        // The oracle asks relay-replay for c(t) at each grid point, one call per point, with every command
        // shifted by the pipeline delay. relay-replay never predicts before its measurement, so nothing
        // here has a reading age, and every arm joint is sensed so neither carries its own estimate.
        const long tps = 1_000_000;
        const float servoSpeed = 1.7f, lag = 0.03f, delay = 0.015f, step = 0.003f;
        const byte Scaled = 6;
        JointPredictorConfig Make(float s, float l, float d) => new JointPredictorConfig(
            maxJoints: 6, commandHistoryCapacity: 32, maxJointSpeedRadPerSec: 5f, instantMotorIds: new[] { Gripper },
            maxHorizonTicks: 400_000L, ticksPerSecond: tps, scaledMotorIds: new[] { Scaled }, speedScales: new[] { 0.5f },
            relayMoveSpeedRadPerSec: 3f, servoMaxSpeedRadPerSec: s, servoLagSeconds: l, servoPipelineDelaySeconds: d,
            servoFirstMoveFromMeasured: false, servoStepSeconds: step);
        var servo = new RelayReplayServoPredictor(Make(servoSpeed, lag, delay));
        var relay = new RelayReplayPredictor(Make(NoCap, 0f, 0f));
        long h = (long)Math.Round(step * (double)tps), shift = (long)Math.Round(delay * (double)tps);
        byte[] motors = { 1, 2, 3, Scaled, Gripper };

        var rng = new SeededRng(seed);
        long ms = tps / 1000;
        var position = new float[motors.Length];
        var goal = new float[motors.Length];
        var measured = new float[motors.Length];
        long stateTicks = 0;
        bool hasState = false;
        var output = new JointSample[8];
        int checkedJoints = 0;
        for (long now = 0, next = 0; now < 3_000 * ms; now += ms)
        {
            if (now % (400 * ms) == 0)
            {
                for (int m = 0; m < motors.Length; m++) goal[m] = (float)(rng.NextDouble() * 2.0 - 1.0);
            }

            if (now % (21 * ms) == 0)
            {
                for (int m = 0; m < motors.Length; m++) position[m] += 0.3f * (goal[m] - position[m]);
                var targets = new JointSample[motors.Length];
                for (int m = 0; m < motors.Length; m++) targets[m] = new JointSample(motors[m], position[m], false);
                float speed = rng.NextDouble() < 0.5 ? 0f : 1.5f;
                servo.ObserveCommand(now, targets, speed);
                relay.ObserveCommand(now + shift, targets, speed);
            }

            if (now % (47 * ms) == 5 * ms)
            {
                var joints = new JointSample[motors.Length];
                for (int m = 0; m < motors.Length; m++)
                {
                    measured[m] = position[m] - (float)(0.3 * rng.NextDouble());
                    joints[m] = new JointSample(motors[m], measured[m], motors[m] != Gripper);
                }

                servo.ObserveState(now, joints);
                relay.ObserveState(now, joints);
                stateTicks = now;
                hasState = true;
            }

            if (hasState && now >= next)
            {
                next = now + (long)((2 + 9 * rng.NextDouble()) * ms);
                long target = now + (long)(rng.NextDouble() * 500 * ms);
                long end = Math.Max(stateTicks, Math.Min(target, stateTicks + 400_000L));
                Assert.True(servo.TryPredict(target, output, out int count));
                for (int m = 0; m < count; m++)
                {
                    float expected;
                    if (motors[m] == Gripper)
                    {
                        expected = Predict(relay, end, Gripper);
                    }
                    else
                    {
                        byte motor = motors[m];
                        double cap = (double)servoSpeed * (motor == Scaled ? 0.5f : 1f);
                        expected = ReferenceFilter(t => Predict(relay, t, motor), stateTicks, measured[m], end, h, tps, cap, lag);
                    }

                    Assert.True(BitConverter.SingleToInt32Bits(expected) == BitConverter.SingleToInt32Bits(output[m].Value),
                        $"seed {seed}, t {target}, motor {motors[m]}: expected {expected:R}, got {output[m].Value:R}");
                    checkedJoints++;
                }
            }
        }

        Assert.True(checkedJoints > 1000);
    }

    /// <summary>The class doc's filter, the slow way: the setpoint comes from <paramref name="c"/> at every point.</summary>
    private static float ReferenceFilter(Func<long, float> c, long anchor, float measured, long end, long h, long tps, double cap, double lag)
    {
        double Step(double p, double target, double dt)
        {
            double lagged = lag > 0 ? target + (p - target) * Math.Exp(-dt / lag) : target;
            double limit = cap * dt;
            double step = lagged - p;
            return step > limit ? p + limit : step < -limit ? p - limit : lagged;
        }

        double p = measured;
        long t = anchor;
        double fullStep = h / (double)tps;
        for (long k = 0, steps = (end - anchor) / h; k < steps; k++)
        {
            t += h;
            p = Step(p, c(t), fullStep);
        }

        if (end > t) p = Step(p, c(end), (end - t) / (double)tps);
        return (float)p;
    }

    // ------------------------------------------------------------------------------------------------
    // (2) The speed cap.

    [Fact]
    public void TheSpeedCap_LimitsHowFastAMeasurementFarBehindTheReplayIsClosed()
    {
        // Settled at 0, then a move to 1 rad from 100 to 600. A measurement at 350 says 0 where the replay
        // had 0.5: relay-replay closes the whole 1 rad in the 250 ms left, at 4 rad/s.
        var relay = Settled(new RelayReplayPredictor(Config()));
        var servo = Settled(new RelayReplayServoPredictor(Config(servoSpeed: 1.5f)));
        foreach (ICommandAwarePredictor p in new ICommandAwarePredictor[] { relay, servo })
        {
            p.ObserveCommand(100, Command(1f, 0f), 0f);
            p.ObserveState(350, State(0f, 0f));
        }

        Assert.Equal(0.5f, Predict(relay, 475, Arm), 4);
        Assert.Equal(0.1875f, Predict(servo, 475, Arm), 4);     // 1.5 rad/s × 125 ms

        float previous = Predict(servo, 350, Arm);
        Assert.Equal(0f, previous);
        for (long t = 351; t <= 1350; t++)
        {
            float now = Predict(servo, t, Arm);
            Assert.True(now - previous <= 1.5f * 0.001f + 1e-6f, $"{now - previous} rad in the 1 ms to {t}");
            Assert.True(now >= previous, $"moved back at {t}");
            previous = now;
        }

        Assert.Equal(1f, Predict(servo, 1100, Arm));            // 1 rad at 1.5 rad/s: there by 1017
    }

    [Fact]
    public void TheSpeedCap_OfAScaledJoint_IsTheCapTimesItsScale()
    {
        var config = Config(servoSpeed: 1f, scaledIds: new[] { Gripper }, scales: new[] { 0.5f }, instantIds: new byte[0]);
        var p = new RelayReplayServoPredictor(config);
        p.ObserveState(0, new[] { new JointSample(Arm, 0f, true), new JointSample(Gripper, 0f, true) });
        p.ObserveCommand(10, Command(0f, 1f), 0f);              // the gripper to 1: a 50 ms first move
        Assert.Equal(0.1f, Predict(p, 210, Gripper), 4);        // 0.5 per s for 200 ms, not 1 rad/s
    }

    // ------------------------------------------------------------------------------------------------
    // (3) The lag.

    [Fact]
    public void TheLag_ApproachesTheTargetLikeOneMinusExp_OnceTheMoveIsDone()
    {
        const double tau = 0.02;
        var servo = new RelayReplayServoPredictor(Config(lag: (float)tau));
        var relay = new RelayReplayPredictor(Config());
        foreach (ICommandAwarePredictor p in new ICommandAwarePredictor[] { servo, relay })
        {
            p.ObserveState(0, State(0f, 0f));
            p.ObserveCommand(100, Command(1f, 0f), 0f);         // the first write: a 50 ms move, 100..150
        }

        // relay-replay is there when the move ends; the servo lags it.
        Assert.Equal(1f, Predict(relay, 150, Arm));
        double behind = 1.0 - Predict(servo, 150, Arm);
        Assert.InRange(behind, 0.2, 0.5);

        // From then on the setpoint holds, and what is left decays as exp(-t / tau).
        for (long t = 150; t <= 400; t++)
        {
            double expected = 1.0 - behind * Math.Exp(-(t - 150) / 1000.0 / tau);
            Assert.Equal(expected, Predict(servo, t, Arm), 2e-6);   // a float's rounding, not a model error
        }
    }

    [Fact]
    public void TheLag_OfAStep_IsOneMinusExp_FromTheStepBefore()
    {
        // No minimum move and no last target: the first write is a step. The setpoint is sampled at each
        // step's end, so the step at 10 is taken from the grid point before it, 8.
        const double tau = 0.02;
        var p = new RelayReplayServoPredictor(Config(lag: (float)tau, minMove: 0f));
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(10, Command(1f, 0f), 0f);
        for (long t = 0; t < 10; t++) Assert.Equal(0f, Predict(p, t, Arm));
        for (long t = 10; t <= 200; t++)
        {
            Assert.Equal(1.0 - Math.Exp(-(t - 8) / 1000.0 / tau), Predict(p, t, Arm), 2e-6);
        }
    }

    // ------------------------------------------------------------------------------------------------
    // (4) The first move, sized from the measurement.

    [Fact]
    public void TheFirstMove_IsSizedFromTheSensedMeasurement()
    {
        var sized = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: true));
        var minimum = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: false));
        foreach (ICommandAwarePredictor p in new ICommandAwarePredictor[] { sized, minimum })
        {
            p.ObserveState(0, State(0.2f, 0f));
            p.ObserveCommand(100, Command(1f, 0f), 0f);
        }

        // 0.8 rad from the measured 0.2 at 2 rad/s: 400 ms, not the 50 ms minimum.
        Assert.Equal(0.6f, Predict(sized, 300, Arm), 4);
        Assert.Equal(1f, Predict(sized, 500, Arm), 5);
        Assert.Equal(1f, Predict(minimum, 150, Arm), 5);

        // The cooldown is the move's: a command at 400 waits until 500.
        sized.ObserveCommand(400, Command(0.5f, 0f), 0f);
        Assert.Equal(1f, Predict(sized, 500, Arm), 5);
        Assert.Equal(1, sized.Gate.Written);
    }

    [Fact]
    public void TheFirstTarget_InsideTheDeadbandOfTheMeasurement_IsDropped_AndTheNextIsSizedFromItToo()
    {
        var p = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: true));
        p.ObserveState(0, State(0.5f, 0f));
        p.ObserveCommand(100, Command(0.51f, 0f), 0f);          // 0.01 from the measured 0.5: dropped
        Assert.Equal(1, p.Gate.DeadbandDrops);
        Assert.Equal(0.5f, Predict(p, 300, Arm), 5);

        p.ObserveCommand(120, Command(1.5f, 0f), 0f);           // 1 rad from 0.5: 500 ms, 120..620
        Assert.Equal(1f, Predict(p, 370, Arm), 4);

        var off = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: false));
        off.ObserveState(0, State(0.5f, 0f));
        off.ObserveCommand(100, Command(0.51f, 0f), 0f);        // no last target: written, 50 ms
        Assert.Equal(0, off.Gate.DeadbandDrops);
        Assert.Equal(0.51f, Predict(off, 150, Arm), 5);
    }

    [Fact]
    public void WithoutASensedMeasurement_TheFirstMoveLastsTheMinimum()
    {
        // An unsensed value is the robot's belief, not where the servo is.
        var unsensed = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: true));
        unsensed.ObserveState(0, new[] { new JointSample(Arm, 0.2f, false), new JointSample(Gripper, 0f, false) });
        unsensed.ObserveCommand(100, Command(1f, 0f), 0f);
        Assert.Equal(0.6f, Predict(unsensed, 125, Arm), 4);     // 0.2 to 1 over 100..150
        Assert.Equal(1f, Predict(unsensed, 150, Arm), 5);

        // And with no measurement yet.
        var blind = new RelayReplayServoPredictor(Config(firstMoveFromMeasured: true));
        blind.ObserveCommand(100, Command(1f, 0f), 0f);
        blind.ObserveState(110, State(0.2f, 0f));               // mid-move: to 1 by 150
        Assert.Equal(1f, Predict(blind, 150, Arm), 5);
    }

    // ------------------------------------------------------------------------------------------------
    // (5) The pipeline delay.

    [Fact]
    public void ThePipelineDelay_ShiftsEveryMove_AsIfEachCommandTookEffectThatMuchLater()
    {
        var delayed = new RelayReplayServoPredictor(Config(delay: 0.02f));
        var shifted = new RelayReplayPredictor(Config());
        delayed.ObserveState(0, State(0f, 0f));
        shifted.ObserveState(0, State(0f, 0f));
        for (long t = 0; t <= 400; t += 21)
        {
            JointSample[] command = Command(MathF.Sin(t * 0.01f), t % 2 == 0 ? 0.3f : 0.9f);
            delayed.ObserveCommand(t, command, t % 3 == 0 ? 1.5f : 0f);
            shifted.ObserveCommand(t + 20, command, t % 3 == 0 ? 1.5f : 0f);
            if (t % 84 == 0)
            {
                delayed.ObserveState(t, State(0.9f * MathF.Sin(t * 0.01f), 0f));
                shifted.ObserveState(t, State(0.9f * MathF.Sin(t * 0.01f), 0f));
            }
        }

        for (long t = 336; t <= 900; t++)
        {
            Assert.Equal(Predict(shifted, t, Arm), Predict(delayed, t, Arm));
            Assert.Equal(Predict(shifted, t, Gripper), Predict(delayed, t, Gripper));
        }
    }

    [Fact]
    public void ThePipelineDelay_DelaysTheMoveAndAnInstantJoint()
    {
        var p = Settled(new RelayReplayServoPredictor(Config(delay: 0.02f)));
        p.ObserveCommand(100, Command(0.5f, 0.8f), 0f);         // 0.5 rad at 2 rad/s: 250 ms, from 120
        Assert.Equal(0f, Predict(p, 120, Arm));
        Assert.Equal(0.25f, Predict(p, 245, Arm), 4);
        Assert.Equal(0f, Predict(p, 119, Gripper));
        Assert.Equal(0.8f, Predict(p, 120, Gripper));
    }

    [Fact]
    public void OutOfOrderCommands_AreJudgedByTheEffectiveTimeAsGiven()
    {
        var p = Settled(new RelayReplayServoPredictor(Config(delay: 0.02f)));
        p.ObserveCommand(100, Command(0.5f, 0f), 0f);
        p.ObserveCommand(90, Command(9f, 0f), 0f);              // older than 100, though 90 + 20 is not
        Assert.Equal(1, p.Diagnostics.RejectedObservations);
        Assert.Equal(0.5f, Predict(p, 600, Arm), 5);
    }

    // ------------------------------------------------------------------------------------------------
    // Unmeasured joints, robustness and the horizon.

    [Fact]
    public void AnUnmeasuredJoint_KeepsTheServosEstimate_NotTheBeliefAndNotTheSetpoint()
    {
        var servo = Settled(new RelayReplayServoPredictor(Config(servoSpeed: 1f)));
        var relay = Settled(new RelayReplayPredictor(Config()));
        foreach (ICommandAwarePredictor p in new ICommandAwarePredictor[] { servo, relay })
        {
            p.ObserveState(0, new[] { new JointSample(Arm, 0f, false) });
            p.ObserveCommand(100, Command(1f, 0f), 0f);         // 1 rad at 2 rad/s: 100..600
            p.ObserveState(300, new[] { new JointSample(Arm, 1f, false) });   // the belief: 1
        }

        // The setpoint was at 0.4 at 300, the servo, capped at 1 rad/s, at 0.2; the belief is ignored.
        Assert.Equal(0.4f, Predict(relay, 300, Arm), 4);
        Assert.Equal(0.2f, Predict(servo, 300, Arm), 4);
        Assert.Equal(0.3f, Predict(servo, 400, Arm), 4);       // still capped, toward 1 by 600
        Assert.Equal(0.6f, Predict(relay, 400, Arm), 4);
    }

    [Fact]
    public void AMeasurementAfterTheMoveEnded_HoldsTheSetpoint_EvenWhileTheServoWouldStillBeMoving()
    {
        // relay-replay's re-anchoring, kept as specified: a measurement between moves says the servo
        // stopped there, so the setpoint holds it, and so does the servo, until the next move.
        var p = Settled(new RelayReplayServoPredictor(Config(lag: 0.02f)));
        p.ObserveCommand(100, Command(0.5f, 0f), 0f);           // 100..350
        p.ObserveState(360, State(0.45f, 0f));                  // the lagged servo, 10 ms after the move
        Assert.Equal(0.45f, Predict(p, 500, Arm), 5);
        p.ObserveCommand(500, Command(1f, 0f), 0f);             // 0.5 rad from the last target: 500..750
        Assert.True(Predict(p, 600, Arm) > 0.45f);
    }

    [Fact]
    public void OutOfOrderMeasurementsAndCommands_AreRejected()
    {
        var p = new RelayReplayServoPredictor(Config(servoSpeed: 3f, lag: 0.02f, delay: 0.02f, firstMoveFromMeasured: true));
        p.ObserveState(100, State(0.5f, 0f));
        p.ObserveState(90, State(9f, 0f));                      // older: rejected
        p.ObserveCommand(200, Command(0.5f, 0f), 0f);           // within the deadband of the measured 0.5
        p.ObserveCommand(150, Command(9f, 0f), 0f);             // older than the newest: rejected

        Assert.Equal(0.5f, Predict(p, 400, Arm), 5);
        Assert.Equal(2, p.Diagnostics.RejectedObservations);
        Assert.Equal(2, p.Diagnostics.AcceptedObservations);
        Assert.Equal(1, p.Gate.DeadbandDrops);
    }

    [Fact]
    public void DuplicateMeasurementsAndCommands_AreHarmless()
    {
        JointPredictorConfig config = Config(servoSpeed: 3f, lag: 0.02f, delay: 0.02f, firstMoveFromMeasured: true);
        var once = Settled(new RelayReplayServoPredictor(config));
        once.ObserveCommand(100, Command(1f, 0f), 0f);
        once.ObserveState(300, State(0.3f, 0f));

        var twice = Settled(new RelayReplayServoPredictor(config));
        twice.ObserveCommand(100, Command(1f, 0f), 0f);
        twice.ObserveCommand(100, Command(1f, 0f), 0f);         // the same command again: pending, the same target
        twice.ObserveState(300, State(0.3f, 0f));
        twice.ObserveState(300, State(0.3f, 0f));

        for (long t = 300; t <= 900; t += 7)
        {
            Assert.Equal(Predict(once, t, Arm), Predict(twice, t, Arm));
        }
    }

    [Fact]
    public void AGapOfSeveralHundredMs_StillPredictsFromTheReplay_ThroughTheServo()
    {
        var p = new RelayReplayServoPredictor(Config(servoSpeed: 5f, lag: 0.02f, delay: 0.02f, maxHorizonMs: 1000));
        p.ObserveState(0, State(0f, 0f));
        for (long t = 0; t <= 400; t += 20)
        {
            p.ObserveCommand(t, Command(t * 0.0015f, 0f), 0f);  // a 1.5 rad/s ramp, 50 Hz-like
        }

        // The setpoint is relay-replay's 20 ms later: 0.57 at 470, when the last target is written, and
        // 0.6 from 520. The servo lags the ramp by about 1.5 rad/s × 20 ms, then settles: 130 ms (6.5
        // lags) later it is there.
        Assert.InRange(Predict(p, 470, Arm), 0.5f, 0.56f);
        Assert.Equal(0.6f, Predict(p, 650, Arm), 3);

        // A measurement a second after the only other one re-anchors, and the horizon still bounds the prediction.
        p.ObserveState(1_000, State(0.58f, 0f));
        Assert.Equal(0.58f, Predict(p, 1_000, Arm), 5);
        Assert.Equal(Predict(p, 2_000, Arm), Predict(p, 9_000, Arm));
        Assert.Equal(1_000, p.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void APredictionBeforeTheMeasurement_ReturnsTheMeasurement()
    {
        var p = new RelayReplayServoPredictor(Config(servoSpeed: 3f, lag: 0.02f));
        p.ObserveState(100, State(0.4f, 0f));
        p.ObserveCommand(100, Command(1f, 0f), 0f);
        Assert.Equal(0.4f, Predict(p, 50, Arm), 5);
        Assert.Equal(0, p.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void NothingObserved_PredictsNothing()
    {
        var p = new RelayReplayServoPredictor(Config(servoSpeed: 3f, lag: 0.02f, delay: 0.02f, firstMoveFromMeasured: true));
        Assert.False(p.TryPredict(100, new JointSample[6], out int count));
        Assert.Equal(0, count);
    }

    // ------------------------------------------------------------------------------------------------
    // (6) Reset, (7) allocation, (8) determinism.

    private static JointPredictorConfig NonNeutral() => Config(servoSpeed: 3f, lag: 0.04f, delay: 0.015f,
        firstMoveFromMeasured: true, ageIds: new[] { Arm }, ageSeconds: new[] { 0.03f });

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        static float[] Run(RelayReplayServoPredictor p)
        {
            p.ObserveState(0, State(0.1f, 0f));
            for (long t = 0; t < 300; t += 21) p.ObserveCommand(t, Command(MathF.Sin(t * 0.01f), t / 300f), t % 2 == 0 ? 1f : 0f);
            p.ObserveState(170, State(0.2f, 0.3f));
            var values = new float[16];
            for (int i = 0; i < values.Length; i++) values[i] = Predict(p, 170 + i * 25, i % 2 == 0 ? Arm : Gripper);
            return values;
        }

        var used = new RelayReplayServoPredictor(NonNeutral());
        float[] first = Run(used);
        used.ObserveCommand(5000, Command(3f, 1f), 0f);
        used.ObserveState(5000, State(2f, 1f));
        used.Reset();

        Assert.False(used.TryPredict(200, new JointSample[6], out _));
        Assert.Equal(0, used.Diagnostics.AcceptedObservations);
        Assert.Equal(0, used.Diagnostics.RejectedObservations);
        Assert.Equal(0, used.Diagnostics.HorizonTicks);
        Assert.Equal(0, used.Gate.Written);
        Assert.Equal(0, used.Gate.DeadbandDrops);
        Assert.Equal(0, used.Gate.Coalesced);

        // After Reset it behaves exactly as a new instance: no gate state, moves or measurement survive.
        Assert.Equal(first, Run(used));
        Assert.Equal(first, Run(new RelayReplayServoPredictor(NonNeutral())));
    }

    [Theory]
    [InlineData(0, 11UL)]
    [InlineData(1, 12UL)]
    [InlineData(2, 13UL)]
    [InlineData(3, 14UL)]
    public void IsDeterministic_AcrossTwoInstances(int variant, ulong seed)
    {
        JointPredictorConfig config = Variant(variant, 2.5f, 0.03f, 0.02f, true);
        var a = new RelayReplayServoPredictor(config);
        var b = new RelayReplayServoPredictor(config);
        Assert.True(Drive(seed, config.TicksPerSecond, new ICommandAwarePredictor[] { a, b }) > 300);
        Assert.Equal(a.Gate.Written, b.Gate.Written);
    }

    [Fact]
    public void AwayFromNeutral_ItDiffersFromRelayReplay()
    {
        // A guard on the identity test above: the same sequence with a cap, a lag and a delay must not agree.
        JointPredictorConfig config = Variant(0, 2.5f, 0.03f, 0.02f, true);
        var relay = new RelayReplayPredictor(config);
        var servo = new RelayReplayServoPredictor(config);
        Assert.ThrowsAny<Exception>(() => Drive(5, config.TicksPerSecond, new ICommandAwarePredictor[] { relay, servo }));
    }

    [Fact]
    public void TryPredict_AllocatesNothing()
    {
        var p = Settled(new RelayReplayServoPredictor(NonNeutral()));
        for (long t = 0; t < 640; t += 10) p.ObserveCommand(t, Command(t / 640f, 0.5f), 0f);
        p.ObserveState(300, State(0.2f, 0.5f));
        var output = new JointSample[6];
        long at = 0;
        AllocationAssert.Zero(() => p.TryPredict(300 + at++ % 700, output, out _));
    }

    [Fact]
    public void ObserveCommandAndState_AllocateNothing()
    {
        var p = new RelayReplayServoPredictor(NonNeutral());
        JointSample[] state = State(0f, 0f);
        var commands = new[] { Command(0f, 0f), Command(1f, 1f) };
        long t = 0;
        AllocationAssert.Zero(() =>
        {
            t += 30;
            p.ObserveCommand(t, commands[(int)(t / 30 % 2)], 0f);
            p.ObserveState(t, state);
        });
    }

    // ------------------------------------------------------------------------------------------------
    // The config and the registry.

    [Fact]
    public void TheConfig_RejectsBadServoParameters()
    {
        JointPredictorConfig Make(float speed = 5f, float lag = 0.02f, float delay = 0.02f, float step = 0.002f) =>
            new JointPredictorConfig(6, 64, 5f, null!, 1000, Tps, servoMaxSpeedRadPerSec: speed, servoLagSeconds: lag,
                servoPipelineDelaySeconds: delay, servoStepSeconds: step);
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(speed: 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(speed: -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(speed: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(lag: -0.01f));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(lag: float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(delay: -0.01f));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(delay: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(step: 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => Make(step: float.PositiveInfinity));

        // No cap, no lag and no delay are all valid.
        Make(speed: float.PositiveInfinity, lag: 0f, delay: 0f);
    }

    [Fact]
    public void TheConfigDefaults_AreTheGuesses()
    {
        var config = new JointPredictorConfig(6, 64, 5f, new byte[0], 1000, Tps);
        Assert.Equal(5f, config.ServoMaxSpeedRadPerSec);
        Assert.Equal(0.02f, config.ServoLagSeconds);
        Assert.Equal(0.02f, config.ServoPipelineDelaySeconds);
        Assert.True(config.ServoFirstMoveFromMeasured);
        Assert.Equal(0.002f, config.ServoStepSeconds);
    }

    [Fact]
    public void AStepShorterThanATick_IsOneTick()
    {
        // At 1000 ticks per second a 0.1 ms step rounds to no ticks; it runs at one, which agrees with 1 ms.
        var fine = Settled(new RelayReplayServoPredictor(Config(lag: 0.02f, step: 0.0001f)));
        var oneTick = Settled(new RelayReplayServoPredictor(Config(lag: 0.02f, step: 0.001f)));
        fine.ObserveCommand(100, Command(1f, 0f), 0f);
        oneTick.ObserveCommand(100, Command(1f, 0f), 0f);
        for (long t = 100; t <= 300; t += 13) Assert.Equal(Predict(oneTick, t, Arm), Predict(fine, t, Arm));
    }

    [Fact]
    public void TheRegistry_BuildsRelayReplayServo()
    {
        Assert.IsType<RelayReplayServoPredictor>(Registries.JointPredictors["relay-replay-servo"](Config()));
    }
}
