using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class ServoReplayPredictorTests
{
    /// <summary>1000 ticks per second: one tick is 1 ms.</summary>
    private const long Tps = 1000;
    private const byte Arm = 1, Gripper = 10;
    private const float Speed = 2f; // rad/s: 0.002 rad per ms

    private static JointPredictorConfig Config(long maxHorizonMs = 500, int capacity = 64) =>
        new JointPredictorConfig(maxJoints: 6, commandHistoryCapacity: capacity, maxJointSpeedRadPerSec: Speed,
            instantMotorIds: new[] { Gripper }, maxHorizonTicks: maxHorizonMs, ticksPerSecond: Tps);

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

    [Fact]
    public void NothingObserved_PredictsNothing()
    {
        var p = new ServoReplayPredictor(Config());
        Assert.False(p.TryPredict(100, new JointSample[6], out int count));
        Assert.Equal(0, count);
    }

    [Fact]
    public void NoCommands_HoldsTheMeasurement()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0.3f, 0.5f));
        Assert.Equal(0.3f, Predict(p, 300, Arm), 5);
    }

    [Fact]
    public void ACommandAfterTheMeasurement_MovesTheJointAtTheModelSpeed_FromWhenItTakesEffect()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0f, 0f));
        p.ObserveCommand(150, Command(1f, 0f), 0f);

        Assert.Equal(0f, Predict(p, 150, Arm), 5);           // not yet moving
        Assert.Equal(0.1f, Predict(p, 200, Arm), 4);         // 50 ms × 2 rad/s
        Assert.Equal(0.9f, Predict(p, 1000, Arm), 4);        // capped by the 500 ms horizon at 600: 450 ms of motion
        Assert.Equal(1f, Predict(p, 600, Arm) + 0.1f, 4);    // the same point, asked for directly
    }

    [Fact]
    public void ARequestedSpeed_BelowTheServoLimit_IsWhatTheJointMovesAt()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(1f, 0f), speedRadPerSec: 0.5f);
        Assert.Equal(0.05f, Predict(p, 100, Arm), 4);   // 100 ms × 0.5 rad/s
    }

    [Fact]
    public void ARequestedSpeed_AboveTheServoLimit_IsCappedAtIt()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(1f, 0f), speedRadPerSec: 50f);
        Assert.Equal(0.2f, Predict(p, 100, Arm), 4);    // capped at 2 rad/s
    }

    [Fact]
    public void ACommandBeforeTheMeasurement_IsWhatTheJointWasStillMovingToward()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveCommand(50, Command(1f, 0f), 0f);
        p.ObserveState(100, State(0.2f, 0f));
        Assert.Equal(0.3f, Predict(p, 150, Arm), 4);
    }

    [Fact]
    public void LaterCommands_ReplaceTheTarget_AndTheJointTurnsAround()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(1f, 0f), 0f);
        p.ObserveCommand(100, Command(-1f, 0f), 0f);
        // +0.2 by 100 ms, then back down at the same speed for 50 ms.
        Assert.Equal(0.1f, Predict(p, 150, Arm), 4);
    }

    [Fact]
    public void AnUnmeasuredJoint_KeepsTheModelsEstimate_NotTheCommandedBelief()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, new[] { new JointSample(Arm, 0f, false) });
        p.ObserveCommand(0, Command(1f, 0f), 0f);
        // The robot already "believes" the target (1.0), but the joint is not measured.
        p.ObserveState(100, new[] { new JointSample(Arm, 1f, false) });
        Assert.Equal(0.2f, Predict(p, 100, Arm), 4);    // 100 ms at 2 rad/s, not 1.0
        Assert.Equal(0.4f, Predict(p, 200, Arm), 4);
    }

    [Fact]
    public void AMeasuredJoint_StillSnapsToTheMeasurement()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, new[] { new JointSample(Arm, 0f, true) });
        p.ObserveCommand(0, Command(1f, 0f), 0f);
        p.ObserveState(100, new[] { new JointSample(Arm, 0.05f, true) });
        Assert.Equal(0.05f, Predict(p, 100, Arm), 4);
    }

    [Fact]
    public void TheGripper_FollowsItsCommandAtOnce()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0f, 0f));
        p.ObserveCommand(120, Command(0f, 0.8f), 0f);
        Assert.Equal(0f, Predict(p, 110, Gripper), 5);
        Assert.Equal(0.8f, Predict(p, 121, Gripper), 5);
    }

    [Fact]
    public void ThePrediction_NeverExceedsTheHorizon()
    {
        var p = new ServoReplayPredictor(Config(maxHorizonMs: 100));
        p.ObserveState(0, State(0f, 0f));
        p.ObserveCommand(0, Command(10f, 0f), 0f);
        Assert.Equal(0.2f, Predict(p, 10_000, Arm), 4);
        Assert.Equal(100, p.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void APredictionBeforeTheMeasurement_ReturnsTheMeasurement()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0.4f, 0f));
        p.ObserveCommand(100, Command(1f, 0f), 0f);
        Assert.Equal(0.4f, Predict(p, 50, Arm), 5);
    }

    [Fact]
    public void OutOfOrderMeasurementsAndCommands_AreRejected()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0.5f, 0f));
        p.ObserveState(90, State(9f, 0f));          // older: rejected
        p.ObserveCommand(200, Command(0.5f, 0f), 0f);
        p.ObserveCommand(150, Command(9f, 0f), 0f);      // older than the newest: rejected

        Assert.Equal(0.5f, Predict(p, 400, Arm), 5);
        Assert.Equal(2, p.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void DuplicateMeasurements_AreHarmless()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveCommand(100, Command(1f, 0f), 0f);
        p.ObserveState(100, State(0f, 0f));
        p.ObserveState(100, State(0f, 0f));
        Assert.Equal(0.1f, Predict(p, 150, Arm), 4);
    }

    [Fact]
    public void AGapOfSeveralHundredMs_StillPredictsFromTheReplay()
    {
        var p = new ServoReplayPredictor(Config(maxHorizonMs: 1000));
        p.ObserveState(0, State(0f, 0f));
        for (long t = 0; t <= 400; t += 20)
        {
            p.ObserveCommand(t, Command(t / 1000f, 0f), 0f);   // a slow ramp the joint can follow
        }

        Assert.Equal(0.4f, Predict(p, 600, Arm), 2);
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(100, State(0.5f, 0f));
        p.ObserveCommand(100, Command(1f, 0f), 0f);
        p.Reset();
        Assert.False(p.TryPredict(200, new JointSample[6], out _));
        Assert.Equal(0, p.Diagnostics.AcceptedObservations);
    }

    [Fact]
    public void IsDeterministic()
    {
        float Run()
        {
            var p = new ServoReplayPredictor(Config());
            p.ObserveState(3, State(0.1f, 0.2f));
            for (long t = 0; t < 300; t += 21) p.ObserveCommand(t, Command(MathF.Sin(t * 0.01f), t / 300f), 0f);
            return Predict(p, 333, Arm) + Predict(p, 333, Gripper);
        }

        Assert.Equal(Run(), Run());
    }

    [Fact]
    public void ARingThatWrapsKeepsTheNewestCommands()
    {
        var p = new ServoReplayPredictor(Config(capacity: 4));
        p.ObserveState(0, State(0f, 0f));
        for (long t = 1; t <= 10; t++) p.ObserveCommand(t, Command(t < 10 ? 5f : 0.001f, 0f), 0f);
        // Only commands 7..10 remain; the last one, at 10 ms, aims at 0.001.
        Assert.Equal(0.001f, Predict(p, 200, Arm), 4);
    }

    [Fact]
    public void TryPredict_AllocatesNothing()
    {
        var p = new ServoReplayPredictor(Config());
        p.ObserveState(0, State(0f, 0f));
        for (long t = 0; t < 640; t += 10) p.ObserveCommand(t, Command(1f, 0.5f), 0f);
        var output = new JointSample[6];
        long at = 0;
        AllocationAssert.Zero(() => p.TryPredict(at++ % 600, output, out _));
    }

    [Fact]
    public void ObserveCommandAndState_AllocateNothing()
    {
        var p = new ServoReplayPredictor(Config());
        JointSample[] command = Command(1f, 0.5f), state = State(0f, 0f);
        long t = 0;
        AllocationAssert.Zero(() =>
        {
            t++;
            p.ObserveCommand(t, command, 0f);
            p.ObserveState(t, state);
        });
    }

    [Fact]
    public void TheRegistry_BuildsBothJointPredictors()
    {
        Assert.IsType<ServoReplayPredictor>(Registries.JointPredictors["servo-replay"](Config()));
        Assert.IsType<HoldLastJointPredictor>(Registries.JointPredictors["hold-last"](Config()));
    }

    [Fact]
    public void HoldLast_IgnoresCommands()
    {
        var p = new HoldLastJointPredictor(Config());
        p.ObserveState(0, State(0.25f, 0f));
        p.ObserveCommand(10, Command(1f, 1f), 0f);
        Assert.Equal(0.25f, Predict(p, 500, Arm), 5);
        Assert.Equal(0f, Predict(p, 500, Gripper), 5);
    }
}
