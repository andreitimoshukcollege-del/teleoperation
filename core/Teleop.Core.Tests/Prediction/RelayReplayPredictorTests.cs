using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class RelayReplayPredictorTests
{
    /// <summary>1000 ticks per second: one tick is 1 ms.</summary>
    private const long Tps = 1000;
    private const byte Arm = 1, Gripper = 10;
    private const float Speed = 2f;          // rad/s: a 1 rad move lasts 500 ms
    private const float MinMove = 0.05f;     // s: 50 ms
    private const float Deadband = 0.02f;    // rad

    private static JointPredictorConfig Config(long maxHorizonMs = 1000, int capacity = 64,
        byte[]? ageIds = null, float[]? ageSeconds = null) =>
        new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: capacity, maxJointSpeedRadPerSec: 5f,
            instantMotorIds: new[] { Gripper }, maxHorizonTicks: maxHorizonMs, ticksPerSecond: Tps,
            relayMinMoveSeconds: MinMove, relayDeadbandRad: Deadband, relayMoveSpeedRadPerSec: Speed,
            sensedAgeMotorIds: ageIds, sensedAgeSeconds: ageSeconds);

    private static JointSample[] State(float arm, float gripper) =>
        new[] { new JointSample(Arm, arm, true), new JointSample(Gripper, gripper, false) };

    private static JointSample[] Command(float arm, float gripper) =>
        new[] { new JointSample(Arm, arm, false), new JointSample(Gripper, gripper, false) };

    private static float Predict(ICommandAwarePredictor p, long at, byte motor)
    {
        var output = new JointSample[6];
        Assert.True(p.TryPredict(at, output, out int count));
        for (int i = 0; i < count; i++)
        {
            if (output[i].MotorId == motor) return output[i].Value;
        }

        throw new Xunit.Sdk.XunitException($"motor {motor} not predicted");
    }

    /// <summary>A measurement of 0 at 0 and a first command to 0 at 0: the servo is settled at 0 by 50 ms.</summary>
    private static RelayReplayPredictor Settled(JointPredictorConfig? config = null)
    {
        var p = new RelayReplayPredictor(config ?? Config());
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(0f, 0f), 0f);
        return p;
    }

    [Fact]
    public void NothingObserved_PredictsNothing()
    {
        var p = new RelayReplayPredictor(Config());
        Assert.False(p.TryPredict(100, new JointSample[6], out int count));
        Assert.Equal(0, count);
    }

    [Fact]
    public void NoCommands_HoldsTheMeasurement()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(100, State(0.3f, 0.5f));
        Assert.Equal(0.3f, Predict(p, 300, Arm), 5);
    }

    [Fact]
    public void AServosFirstWrite_LastsTheMinimumMove()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(100, Command(0.5f, 0f), 0f);
        Assert.Equal(0f, Predict(p, 100, Arm), 5);
        Assert.Equal(0.25f, Predict(p, 125, Arm), 4);   // linear over 50 ms
        Assert.Equal(0.5f, Predict(p, 150, Arm), 5);
    }

    [Fact]
    public void AMove_LastsItsDistanceFromTheLastTargetOverTheSpeed()
    {
        var p = Settled();
        p.ObserveCommand(100, Command(1f, 0f), 0f);     // 1 rad at 2 rad/s: 500 ms
        Assert.Equal(0.5f, Predict(p, 350, Arm), 4);
        Assert.Equal(1f, Predict(p, 600, Arm), 5);
        Assert.Equal(2, p.Gate.Written);
    }

    [Fact]
    public void ARequestedSpeedBelowTheRelaySpeed_SizesTheMove_AndOneAboveItIsCapped()
    {
        var slow = Settled();
        slow.ObserveCommand(100, Command(1f, 0f), speedRadPerSec: 1f);   // 1000 ms
        Assert.Equal(0.25f, Predict(slow, 350, Arm), 4);

        var fast = Settled();
        fast.ObserveCommand(100, Command(1f, 0f), speedRadPerSec: 50f);  // capped at 2 rad/s
        Assert.Equal(0.5f, Predict(fast, 350, Arm), 4);
    }

    [Fact]
    public void ACommandDuringTheCooldown_WaitsAndTheNewestIsWrittenWhenItEnds()
    {
        var p = Settled();
        p.ObserveCommand(60, Command(0.2f, 0f), 0f);    // written: 0.2 rad, 100 ms, cooling down to 160
        p.ObserveCommand(80, Command(0.3f, 0f), 0f);    // pending
        p.ObserveCommand(120, Command(0.4f, 0f), 0f);   // pending, replaces 0.3

        // At 160 the relay writes 0.4: 0.2 rad from 0.2, another 100 ms.
        Assert.Equal(0.2f, Predict(p, 160, Arm), 4);
        Assert.Equal(0.3f, Predict(p, 210, Arm), 4);
        Assert.Equal(0.4f, Predict(p, 260, Arm), 4);
        Assert.Equal(1, p.Gate.Coalesced);

        // A later command makes the gate store that write; the replay is unchanged.
        p.ObserveCommand(300, Command(0.4f, 0f), 0f);
        Assert.Equal(0.3f, Predict(p, 210, Arm), 4);
        Assert.Equal(3, p.Gate.Written);
    }

    [Fact]
    public void ATargetInsideTheDeadband_IsDropped_AndStartsNoCooldown()
    {
        var p = Settled();
        p.ObserveCommand(100, Command(0.5f, 0f), 0f);   // 250 ms, to 350
        p.ObserveCommand(400, Command(0.51f, 0f), 0f);  // 0.01 from 0.5: dropped
        Assert.Equal(1, p.Gate.DeadbandDrops);
        Assert.Equal(0.5f, Predict(p, 450, Arm), 5);

        p.ObserveCommand(401, Command(0.6f, 0f), 0f);   // free to write at once: 0.1 rad, 50 ms
        Assert.Equal(0.55f, Predict(p, 426, Arm), 4);
    }

    [Fact]
    public void APendingTargetInsideTheDeadband_IsDroppedWhenTheCooldownEnds()
    {
        var p = Settled();
        p.ObserveCommand(60, Command(0.2f, 0f), 0f);    // to 160
        p.ObserveCommand(100, Command(0.21f, 0f), 0f);  // pending, within the deadband of 0.2
        Assert.Equal(0.2f, Predict(p, 250, Arm), 5);
        p.ObserveCommand(170, Command(0.5f, 0f), 0f);   // the pending one is dropped at 160, this is written at 170
        Assert.Equal(1, p.Gate.DeadbandDrops);
        Assert.Equal(0.35f, Predict(p, 245, Arm), 4);   // 0.3 rad over 150 ms, half way
    }

    [Fact]
    public void AMeasurementMidMove_ContinuesFromTheMeasurementToThatMovesTarget()
    {
        var p = Settled();
        p.ObserveCommand(100, Command(1f, 0f), 0f);     // 100..600
        p.ObserveState(350, State(0.3f, 0f));           // the model had 0.5

        Assert.Equal(0.3f, Predict(p, 350, Arm), 5);
        Assert.Equal(0.65f, Predict(p, 475, Arm), 4);   // 0.7 rad over the 250 ms left
        Assert.Equal(1f, Predict(p, 600, Arm), 5);
    }

    [Fact]
    public void AMeasurementBetweenMoves_HoldsUntilTheNextMove_ThenMovesFromThere()
    {
        var p = Settled();
        p.ObserveCommand(100, Command(0.5f, 0f), 0f);   // 100..350
        p.ObserveState(400, State(0.45f, 0f));          // settled short of the target
        p.ObserveCommand(500, Command(1f, 0f), 0f);     // 0.5 rad from the last target: 250 ms
        Assert.Equal(0.45f, Predict(p, 500, Arm), 5);
        Assert.Equal(0.725f, Predict(p, 625, Arm), 4);  // from 0.45, half way to 1.0
    }

    [Fact]
    public void ASensedAge_AnchorsTheReadingBeforeItsStamp()
    {
        var aged = Settled(Config(ageIds: new[] { Arm }, ageSeconds: new[] { 0.05f }));
        aged.ObserveCommand(100, Command(1f, 0f), 0f);  // 100..600, 0.5 at 350
        aged.ObserveState(400, State(0.5f, 0f));        // read at 350, stamped at 400
        Assert.Equal(0.6f, Predict(aged, 400, Arm), 4);

        var unaged = Settled();
        unaged.ObserveCommand(100, Command(1f, 0f), 0f);
        unaged.ObserveState(400, State(0.5f, 0f));
        Assert.Equal(0.5f, Predict(unaged, 400, Arm), 4);
    }

    [Fact]
    public void TheGripper_FollowsItsCommandAtOnce()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(100, State(0f, 0f));
        p.ObserveCommand(120, Command(0f, 0.8f), 0f);
        p.ObserveCommand(121, Command(0f, 0.81f), 0f);  // no deadband for an instant joint
        Assert.Equal(0f, Predict(p, 110, Gripper), 5);
        Assert.Equal(0.8f, Predict(p, 120, Gripper), 5);
        Assert.Equal(0.81f, Predict(p, 130, Gripper), 5);
    }

    [Fact]
    public void AScaledJoint_HasItsDeadbandAndSpeedScaled()
    {
        var config = new JointPredictorConfig(6, 64, 5f, new byte[0], 1000, Tps,
            scaledMotorIds: new[] { Gripper }, speedScales: new[] { 0.5f },
            relayMinMoveSeconds: MinMove, relayDeadbandRad: Deadband, relayMoveSpeedRadPerSec: Speed);
        var p = new RelayReplayPredictor(config);
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(0f, 0f), 0f);
        p.ObserveCommand(100, Command(0f, 0.005f), 0f); // within 0.02 × 0.5: dropped
        Assert.Equal(2, p.Gate.Written);                // the first command's two joints only
        p.ObserveCommand(200, Command(0f, 0.5f), 0f);   // 0.5 at 2 × 0.5 per s: 500 ms
        Assert.Equal(0.25f, Predict(p, 450, Gripper), 4);
    }

    [Fact]
    public void AnUnmeasuredJoint_KeepsTheModelsEstimate_NotTheCommandedBelief()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(0, new[] { new JointSample(Arm, 0f, false) });
        p.ObserveCommand(0, Command(0f, 0f), 0f);
        p.ObserveCommand(100, Command(1f, 0f), 0f);     // 100..600
        p.ObserveState(350, new[] { new JointSample(Arm, 1f, false) });
        Assert.Equal(0.5f, Predict(p, 350, Arm), 4);
        Assert.Equal(0.75f, Predict(p, 475, Arm), 4);
    }

    [Fact]
    public void ThePrediction_NeverExceedsTheHorizon()
    {
        var p = Settled(Config(maxHorizonMs: 100));
        p.ObserveCommand(0, Command(10f, 0f), 0f);       // inside the first write's cooldown: pending to 50
        Assert.Equal(0.1f, Predict(p, 10_000, Arm), 4); // 50 ms of a 2 rad/s move
        Assert.Equal(100, p.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void APredictionBeforeTheMeasurement_ReturnsTheMeasurement()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(100, State(0.4f, 0f));
        p.ObserveCommand(100, Command(1f, 0f), 0f);
        Assert.Equal(0.4f, Predict(p, 50, Arm), 5);
    }

    [Fact]
    public void OutOfOrderMeasurementsAndCommands_AreRejected()
    {
        var p = new RelayReplayPredictor(Config());
        p.ObserveState(100, State(0.5f, 0f));
        p.ObserveState(90, State(9f, 0f));               // older: rejected
        p.ObserveCommand(200, Command(0.5f, 0f), 0f);
        p.ObserveCommand(150, Command(9f, 0f), 0f);      // older than the newest: rejected

        Assert.Equal(0.5f, Predict(p, 400, Arm), 5);
        Assert.Equal(2, p.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void DuplicateMeasurementsAndCommands_AreHarmless()
    {
        var once = Settled();
        once.ObserveCommand(100, Command(1f, 0f), 0f);
        once.ObserveState(300, State(0.3f, 0f));

        var twice = Settled();
        twice.ObserveCommand(100, Command(1f, 0f), 0f);
        twice.ObserveCommand(100, Command(1f, 0f), 0f);  // the same command again: pending, the same target
        twice.ObserveState(300, State(0.3f, 0f));
        twice.ObserveState(300, State(0.3f, 0f));

        for (long t = 300; t <= 700; t += 50)
        {
            Assert.Equal(Predict(once, t, Arm), Predict(twice, t, Arm));
        }
    }

    [Fact]
    public void AGapOfSeveralHundredMs_StillPredictsFromTheReplay()
    {
        var p = new RelayReplayPredictor(Config(maxHorizonMs: 1000));
        p.ObserveState(0, State(0f, 0f));
        for (long t = 0; t <= 400; t += 20)
        {
            p.ObserveCommand(t, Command(t * 0.0015f, 0f), 0f);   // a 1.5 rad/s ramp, 50 Hz-like
        }

        // Written every 50 ms (the minimum move), each the newest pending target; the last, 0.6,
        // is written when the cooldown ends at 450. 600 ms after the only measurement it is there.
        Assert.Equal(0.6f, Predict(p, 600, Arm), 4);
        Assert.Equal(0.57f, Predict(p, 450, Arm), 4);
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        static float[] Run(RelayReplayPredictor p)
        {
            p.ObserveState(0, State(0f, 0f));
            for (long t = 0; t < 300; t += 21) p.ObserveCommand(t, Command(MathF.Sin(t * 0.01f), t / 300f), 0f);
            p.ObserveState(170, State(0.2f, 0.3f));
            var values = new float[8];
            for (int i = 0; i < values.Length; i++) values[i] = Predict(p, 170 + i * 40, Arm);
            return values;
        }

        var used = new RelayReplayPredictor(Config());
        float[] first = Run(used);
        used.ObserveCommand(5000, Command(3f, 1f), 0f);
        used.Reset();

        Assert.False(used.TryPredict(200, new JointSample[6], out _));
        Assert.Equal(0, used.Diagnostics.AcceptedObservations);
        Assert.Equal(0, used.Diagnostics.RejectedObservations);
        Assert.Equal(0, used.Gate.Written);
        Assert.Equal(0, used.Gate.DeadbandDrops);
        Assert.Equal(0, used.Gate.Coalesced);

        // After Reset it behaves exactly as a new instance: no gate state or moves survive.
        Assert.Equal(first, Run(used));
        Assert.Equal(first, Run(new RelayReplayPredictor(Config())));
    }

    [Fact]
    public void IsDeterministic()
    {
        float Run()
        {
            var p = new RelayReplayPredictor(Config(ageIds: new[] { Arm }, ageSeconds: new[] { 0.03f }));
            p.ObserveState(3, State(0.1f, 0.2f));
            for (long t = 0; t < 300; t += 21)
            {
                p.ObserveCommand(t, Command(MathF.Sin(t * 0.01f), t / 300f), 0f);
                if (t % 50 == 0) p.ObserveState(t, State(MathF.Sin(t * 0.009f), 0f));
            }

            return Predict(p, 333, Arm) + Predict(p, 333, Gripper);
        }

        Assert.Equal(Run(), Run());
    }

    [Fact]
    public void ARingThatWraps_KeepsTheNewestMoves()
    {
        var p = new RelayReplayPredictor(Config(capacity: 4));
        p.ObserveState(0, State(0f, 0f));
        // Each target is 0.1 rad on, so each is written, 50 ms apart.
        for (int k = 0; k < 10; k++) p.ObserveCommand(k * 50, Command(0.1f * k, 0f), 0f);
        // Only the moves to 0.6..0.9 remain; the measurement at 0 holds until the oldest of them.
        Assert.Equal(0.9f, Predict(p, 600, Arm), 4);
        Assert.Equal(0f, Predict(p, 299, Arm), 4);
    }

    [Fact]
    public void TryPredict_AllocatesNothing()
    {
        var p = Settled(Config(ageIds: new[] { Arm }, ageSeconds: new[] { 0.02f }));
        for (long t = 0; t < 640; t += 10) p.ObserveCommand(t, Command(t / 640f, 0.5f), 0f);
        var output = new JointSample[6];
        long at = 0;
        AllocationAssert.Zero(() => p.TryPredict(at++ % 600, output, out _));
    }

    [Fact]
    public void ObserveCommandAndState_AllocateNothing()
    {
        var p = new RelayReplayPredictor(Config(ageIds: new[] { Arm }, ageSeconds: new[] { 0.02f }));
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

    [Fact]
    public void TheConfig_RejectsMismatchedOrNegativeSensedAges_AndBadGateParameters()
    {
        Assert.Throws<ArgumentException>(() => Config(ageIds: new[] { Arm }, ageSeconds: new float[0]));
        Assert.Throws<ArgumentOutOfRangeException>(() => Config(ageIds: new[] { Arm }, ageSeconds: new[] { -0.01f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointPredictorConfig(6, 64, 5f, null!, 1000, Tps, relayMoveSpeedRadPerSec: 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointPredictorConfig(6, 64, 5f, null!, 1000, Tps, relayDeadbandRad: -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointPredictorConfig(6, 64, 5f, null!, 1000, Tps, relayMinMoveSeconds: -1f));
    }

    [Fact]
    public void TheConfigDefaults_AreTheJetRoversRelay()
    {
        var config = new JointPredictorConfig(6, 64, 5f, new byte[0], 1000, Tps);
        Assert.Equal(0.05f, config.RelayMinMoveSeconds);
        Assert.Equal(5.0 / (1000.0 / (240.0 * Math.PI / 180.0)), config.RelayDeadbandRad, 6);
        Assert.Equal(2200.0 / (1000.0 / (240.0 * Math.PI / 180.0)), config.RelayMoveSpeedRadPerSec, 4);
        Assert.Empty(config.SensedAgeMotorIds);
    }

    [Fact]
    public void TheRegistry_BuildsRelayReplay()
    {
        Assert.IsType<RelayReplayPredictor>(Registries.JointPredictors["relay-replay"](Config()));
    }
}
