using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class FrankaReplayPredictorTests
{
    /// <summary>A microsecond tick: the shaper's 1 ms step is 1000 ticks.</summary>
    private const long Tps = 1_000_000;
    private const long Ms = Tps / 1000;
    private const byte Arm = 1, Elbow = 4, Hand = 8, Unshaped = 30;

    private static readonly TrajectoryShaperConfig Shaper = JointTrajectoryShaperTests.Config();

    private static JointPredictorConfig Config(long maxHorizonMs = 1000, int capacity = 64) =>
        new JointPredictorConfig(maxJoints: 8, commandHistoryCapacity: capacity, maxJointSpeedRadPerSec: 5f,
            instantMotorIds: new[] { Hand }, maxHorizonTicks: maxHorizonMs * Ms, ticksPerSecond: Tps, shaper: Shaper);

    private static JointSample[] Samples(bool sensed, float arm, float elbow, float hand) =>
        new[] { new JointSample(Arm, arm, sensed), new JointSample(Elbow, elbow, sensed), new JointSample(Hand, hand, sensed) };

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

    /// <summary>
    /// A robot that is exactly the shaper at 1 ms, as the FR3 backend is with no tracking lag: commands every
    /// 21 ms (a sine on each joint, the hand closing at 600 ms) take effect at their effective times, and the
    /// operator sees each command at once. It is measured every 20 ms on its own step grid, and each measurement
    /// reaches the operator <paramref name="measurementDelayMs"/> later. Calls <paramref name="each"/> at every
    /// millisecond, after that millisecond's input, with the robot's true joint 1 and 4.
    /// </summary>
    private static void RunRobot(ICommandAwarePredictor p, int milliseconds, Action<long, double, double> each,
        int measurementDelayMs = 100)
    {
        ShapedJointState arm = ShapedJointState.AtRest(0), elbow = ShapedJointState.AtRest(-1.5), hand = ShapedJointState.AtRest(0);
        double armTarget = 0, elbowTarget = -1.5, handTarget = 0;
        var inFlight = new Queue<(long Sensed, JointSample[] Joints)>();
        p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
        for (long ms = 0; ms < milliseconds; ms++)
        {
            long t = ms * Ms;
            if (ms % 21 == 0)
            {
                float a = (float)(ms < 1200 ? 0.8 * Math.Sin(ms / 400.0) : 0.8 * Math.Sin(1200 / 400.0));
                float e = (float)(-1.5 + 0.5 * Math.Sin(ms / 300.0));
                float h = ms > 600 ? 1f : 0f;
                p.ObserveCommand(t, Samples(false, a, e, h), 0f);
                armTarget = a;
                elbowTarget = e;
                handTarget = h;
            }

            if (ms > 0 && ms % 20 == 0) inFlight.Enqueue((t, Samples(true, (float)arm.Q, (float)elbow.Q, (float)hand.Q)));
            while (inFlight.Count > 0 && inFlight.Peek().Sensed + measurementDelayMs * Ms <= t)
            {
                var m = inFlight.Dequeue();
                p.ObserveState(m.Sensed, m.Joints);
            }

            each(t, arm.Q, elbow.Q);

            JointTrajectoryShaper.Step(ref arm, armTarget, JointTrajectoryShaperTests.Joint1, Shaper);
            JointTrajectoryShaper.Step(ref elbow, elbowTarget, JointTrajectoryShaperTests.Joint4, Shaper);
            JointTrajectoryShaper.Step(ref hand, handTarget, JointTrajectoryShaperTests.Finger, Shaper);
        }
    }

    [Fact]
    public void NothingObserved_PredictsNothing()
    {
        var p = new FrankaReplayPredictor(Config());
        Assert.False(p.TryPredict(100, new JointSample[8], out int count));
        Assert.Equal(0, count);
    }

    [Fact]
    public void WithNoCommand_ItHoldsWhereMeasured()
    {
        var p = new FrankaReplayPredictor(Config());
        p.ObserveState(0, Samples(true, 0.3f, -1f, 0.5f));
        Assert.Equal(0.3f, Predict(p, 500 * Ms, Arm));
        Assert.Equal(-1f, Predict(p, 500 * Ms, Elbow));
        Assert.Equal(0.5f, Predict(p, 500 * Ms, Hand));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(40)]
    [InlineData(150)]
    [InlineData(400)]
    public void OnARobotThatIsTheShaper_ItPredictsTheRobotNowToWithinFloatRounding(int measurementDelayMs)
    {
        // Measurements arrive late; the commands are all known. The only error left is the float rounding of
        // the measured values the replay re-anchors on.
        var p = new FrankaReplayPredictor(Config());
        double worst = 0;
        RunRobot(p, 2500, (t, arm, elbow) =>
        {
            worst = Math.Max(worst, Math.Abs(Predict(p, t, Arm) - arm));
            worst = Math.Max(worst, Math.Abs(Predict(p, t, Elbow) - elbow));
        }, measurementDelayMs);

        Assert.True(worst < 1e-5, $"worst {worst} rad");
    }

    [Fact]
    public void HoldLast_OnTheSameRobot_IsFarWorse()
    {
        // The exactness above is the shaper, not the test: hold-last on the same run is off by tenths of a radian.
        var p = Registries.JointPredictors["hold-last"](Config());
        double worst = 0;
        RunRobot(p, 2500, (t, arm, _) => worst = Math.Max(worst, Math.Abs(Predict(p, t, Arm) - arm)), 150);
        Assert.True(worst > 0.1, $"worst {worst} rad");
    }

    [Fact]
    public void AMeasurementThatDisagreesIsAbsorbed_ThePredictionStillEndsAtTheTarget()
    {
        var p = new FrankaReplayPredictor(Config());
        p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
        p.ObserveCommand(0, Samples(false, 1f, -1.5f, 0f), 0f);
        Assert.True(Predict(p, 100 * Ms, Arm) > 0.01f);

        // The arm is measured well behind where the replay has it, mid-move.
        p.ObserveState(200 * Ms, Samples(true, 0.05f, -1.5f, 0f));
        float soon = Predict(p, 210 * Ms, Arm);
        Assert.True(soon > 0.05f && soon < 0.2f, $"soon {soon}");
        Assert.Equal(1f, Predict(p, 1200 * Ms, Arm), 4);
    }

    [Fact]
    public void TheFingerFollowsTheGate()
    {
        var p = new FrankaReplayPredictor(Config());
        p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
        p.ObserveCommand(10 * Ms, Samples(false, 0f, -1.5f, 1f), 0f);
        Assert.Equal(0f, Predict(p, 10 * Ms, Hand));
        Assert.Equal(0.5f, Predict(p, 110 * Ms, Hand), 3);
        Assert.Equal(1f, Predict(p, 400 * Ms, Hand));
    }

    [Fact]
    public void AMotorTheShaperDoesNotListIsHeld()
    {
        var p = new FrankaReplayPredictor(Config());
        p.ObserveState(0, new[] { new JointSample(Unshaped, 0.7f, true), new JointSample(Arm, 0f, true) });
        p.ObserveCommand(0, new[] { new JointSample(Unshaped, -1f, false), new JointSample(Arm, 0.5f, false) }, 0f);
        Assert.Equal(0.7f, Predict(p, 800 * Ms, Unshaped));
        Assert.Equal(0.5f, Predict(p, 800 * Ms, Arm), 4);
    }

    [Fact]
    public void OutOfOrderInput_IsRejectedAndCounted()
    {
        var p = new FrankaReplayPredictor(Config());
        p.ObserveState(100 * Ms, Samples(true, 0f, -1.5f, 0f));
        p.ObserveCommand(100 * Ms, Samples(false, 0.5f, -1.5f, 0f), 0f);
        float before = Predict(p, 300 * Ms, Arm);

        p.ObserveState(50 * Ms, Samples(true, 0.4f, -1.5f, 0f));      // older than the current measurement
        p.ObserveCommand(60 * Ms, Samples(false, -0.5f, -1.5f, 0f), 0f); // older than the newest command

        Assert.Equal(before, Predict(p, 300 * Ms, Arm));
        Assert.Equal(2, p.Diagnostics.RejectedObservations);
        Assert.Equal(2, p.Diagnostics.AcceptedObservations);
    }

    [Fact]
    public void DuplicateInput_ChangesNothing()
    {
        var once = new FrankaReplayPredictor(Config());
        var twice = new FrankaReplayPredictor(Config());
        foreach (var p in new[] { once, twice })
        {
            p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
            p.ObserveCommand(5 * Ms, Samples(false, 0.5f, -1.2f, 1f), 0f);
            p.ObserveState(40 * Ms, Samples(true, 0.01f, -1.49f, 0.1f));
        }

        twice.ObserveCommand(5 * Ms, Samples(false, 0.5f, -1.2f, 1f), 0f);
        twice.ObserveState(40 * Ms, Samples(true, 0.01f, -1.49f, 0.1f));

        foreach (byte motor in new[] { Arm, Elbow, Hand })
        {
            for (long t = 40 * Ms; t < 900 * Ms; t += 37 * Ms) Assert.Equal(Predict(once, t, motor), Predict(twice, t, motor));
        }
    }

    [Fact]
    public void AGapOfSeveralHundredMilliseconds_KeepsReplayingThenSettlesPastTheHorizon()
    {
        var p = new FrankaReplayPredictor(Config(maxHorizonMs: 500));
        p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
        p.ObserveCommand(0, Samples(false, 1f, -1.5f, 0f), 0f);

        // 700 ms with no measurement: the prediction is capped at the 500 ms horizon, still moving toward 1.
        float capped = Predict(p, 700 * Ms, Arm);
        Assert.Equal(Predict(p, 500 * Ms, Arm), capped);
        Assert.Equal(500 * Ms, p.Diagnostics.HorizonTicks);

        // A measurement after more than the horizon is taken as settled where measured, still steering to the target.
        p.ObserveState(700 * Ms, Samples(true, 0.9f, -1.5f, 0f));
        Assert.Equal(0.9f, Predict(p, 700 * Ms, Arm));
        Assert.Equal(1f, Predict(p, 1200 * Ms, Arm), 4);
    }

    [Fact]
    public void ReplayIsDeterministic()
    {
        var a = new FrankaReplayPredictor(Config());
        var b = new FrankaReplayPredictor(Config());
        var outA = new List<float>();
        var outB = new List<float>();
        RunRobot(a, 1500, (t, _, _) => { if (t % (11 * Ms) == 0) outA.Add(Predict(a, t + 30 * Ms, Arm)); });
        RunRobot(b, 1500, (t, _, _) => { if (t % (11 * Ms) == 0) outB.Add(Predict(b, t + 30 * Ms, Arm)); });
        Assert.Equal(outA.Select(BitConverter.SingleToInt32Bits), outB.Select(BitConverter.SingleToInt32Bits));
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var fresh = new FrankaReplayPredictor(Config());
        var used = new FrankaReplayPredictor(Config());
        RunRobot(used, 800, (_, _, _) => { });
        used.Reset();

        Assert.False(used.TryPredict(0, new JointSample[8], out _));
        Assert.Equal(0, used.Diagnostics.AcceptedObservations);

        var outFresh = new List<float>();
        var outUsed = new List<float>();
        RunRobot(fresh, 900, (t, _, _) => { if (t % (13 * Ms) == 0) outFresh.Add(Predict(fresh, t + 50 * Ms, Elbow)); });
        RunRobot(used, 900, (t, _, _) => { if (t % (13 * Ms) == 0) outUsed.Add(Predict(used, t + 50 * Ms, Elbow)); });
        Assert.Equal(outFresh, outUsed);
    }

    [Fact]
    public void ASmallRing_KeepsThePushedOutTargetInForce()
    {
        var p = new FrankaReplayPredictor(Config(capacity: 2));
        p.ObserveState(0, Samples(true, 0f, -1.5f, 0f));
        p.ObserveCommand(10 * Ms, Samples(false, 0.3f, -1.5f, 0f), 0f);
        p.ObserveCommand(800 * Ms, Samples(false, -0.2f, -1.5f, 0f), 0f);
        p.ObserveCommand(900 * Ms, Samples(false, -0.2f, -1.5f, 0f), 0f);

        // The first command was pushed out of the two-slot ring, but it stays in force before the oldest kept:
        // by 750 ms the arm has settled on it, and then it heads for the newer target.
        Assert.Equal(0.3f, Predict(p, 750 * Ms, Arm), 4);
        Assert.True(Predict(p, 1000 * Ms, Arm) < 0.3f);
    }

    [Fact]
    public void TheHotPathAllocatesNothing()
    {
        var p = new FrankaReplayPredictor(Config());
        var output = new JointSample[8];
        JointSample[] command = Samples(false, 0.5f, -1.2f, 1f);
        JointSample[] state = Samples(true, 0.1f, -1.4f, 0.2f);
        p.ObserveState(0, state);
        long t = 0;
        AllocationAssert.Zero(() =>
        {
            t += Ms;
            if (t % (21 * Ms) == 0) p.ObserveCommand(t, command, 0f);
            if (t % (20 * Ms) == 0) p.ObserveState(t, state);
            p.TryPredict(t + 80 * Ms, output, out _);
        }, iterations: 2_000);
    }

    [Fact]
    public void TheRegistry_BuildsItWithAShaper_AndRefusesWithout()
    {
        Assert.IsType<FrankaReplayPredictor>(Registries.JointPredictors["franka-replay"](Config()));
        Assert.Throws<ArgumentException>(() => Registries.JointPredictors["franka-replay"](Config().WithShaper(null)));
    }
}
