using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

/// <summary>
/// The trajectory shaper (docs/adr/0027 decision 1) keeps a real Franka arm out of its reflexes, so these tests
/// check the limits on every step, not just the end state. The limits are an FR3's joint 1 and joint 4, and a
/// Franka Hand finger, as core/RobotProfiles/fr3-duo.json gives them.
/// </summary>
public class JointTrajectoryShaperTests
{
    internal static readonly ShapedJointLimits Joint1 = ShapedJointLimits.Joint(1,
        -2.9007400166666666, 2.9007400166666666, -2.9007400166666666, 2.9007400166666666, 2.62, 10, 5000, 6.0, 0.6520000381679385);

    /// <summary>Joint 2 has the FR3's gentlest position-based deceleration (2.585 rad/s^2), below the shaper's braking acceleration.</summary>
    internal static readonly ShapedJointLimits Joint2 = ShapedJointLimits.Joint(2,
        -1.8360900166666667, 1.8360900166666667, -1.8360900166666667, 1.8360900166666667, 2.62, 10, 5000, 2.585, 0.2499685851463105);

    internal static readonly ShapedJointLimits Joint4 = ShapedJointLimits.Joint(4,
        -3.077020016666667, -0.11693708333333333, -3.077020016666667, -0.11693708333333333, 2.62, 10, 5000, 4.0, 0.3542181933842238);

    internal static readonly ShapedJointLimits Finger = ShapedJointLimits.Finger(8, 0.2 / 0.04, 0.02);

    internal static TrajectoryShaperConfig Config() => new TrajectoryShaperConfig(new[] { Joint1, Joint2, Joint4, Finger });

    /// <summary>Steps <paramref name="steps"/> times toward <paramref name="target"/>, asserting every step stays within the shaper's shares of the limits.</summary>
    private static ShapedJointState Run(ShapedJointState s, double target, ShapedJointLimits l, TrajectoryShaperConfig c, int steps,
        bool checkAcceleration = true)
    {
        double dt = c.StepSeconds;
        double vMax = l.Velocity * c.VelocityScale, aMax = l.Acceleration * c.AccelerationScale, jMax = l.Jerk * c.JerkScale;
        for (int k = 0; k < steps; k++)
        {
            double previousA = s.A;
            JointTrajectoryShaper.Step(ref s, target, l, c);
            Assert.True(Math.Abs(s.V) <= vMax + 1e-12, $"step {k}: |v| {s.V} over {vMax}");
            if (checkAcceleration)
            {
                Assert.True(Math.Abs(s.A) <= aMax + 1e-9, $"step {k}: |a| {s.A} over {aMax}");
                Assert.True(Math.Abs(s.A - previousA) <= jMax * dt + 1e-9, $"step {k}: jerk {(s.A - previousA) / dt} over {jMax}");
            }

            Assert.True(s.Q >= l.Lower && s.Q <= l.Upper, $"step {k}: q {s.Q} outside the hard limits");
        }

        return s;
    }

    [Fact]
    public void ALongMoveStaysInsideEveryLimitAndSettlesOnTheTarget()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = Run(ShapedJointState.AtRest(-1.5), 1.5, Joint1, c, 4000);

        Assert.True(Math.Abs(s.Q - 1.5) < 1e-6, $"q {s.Q}");
        Assert.True(Math.Abs(s.V) < 1e-4, $"v {s.V}");
    }

    [Fact]
    public void ALongMoveReachesTheVelocityShareInTheMiddle()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = ShapedJointState.AtRest(-2.0);
        double fastest = 0;
        for (int k = 0; k < 3000; k++)
        {
            JointTrajectoryShaper.Step(ref s, 2.0, Joint1, c);
            fastest = Math.Max(fastest, s.V);
        }

        Assert.Equal(2.62 * c.VelocityScale, fastest, 9);
    }

    [Fact]
    public void ATargetPastTheSoftLimitIsHeldInsideItByTheMargin()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = Run(ShapedJointState.AtRest(-1.0), -0.05, Joint4, c, 3000);

        Assert.True(Math.Abs(s.Q - (Joint4.SoftUpper - c.PositionMargin)) < 1e-6, $"q {s.Q}");
    }

    [Fact]
    public void NearAHardLimitThePositionBasedLimitCapsTheVelocityTowardIt()
    {
        // Joint 4 running at 2 rad/s toward its upper limit from 0.2 rad away, faster than the position-based
        // limit allows there: every step's velocity is within 0.9 (offset + sqrt(2 * deceleration * distance)) of
        // the limit, measured from where the step began. Only here may the acceleration exceed its share.
        TrajectoryShaperConfig c = Config();
        var s = new ShapedJointState { Q = Joint4.Upper - 0.2, V = 2.0, A = 0 };
        bool bound = false;
        for (int k = 0; k < 2000; k++)
        {
            double before = s.Q;
            JointTrajectoryShaper.Step(ref s, 10.0, Joint4, c);
            double allowed = c.VelocityScale * (Joint4.PositionBasedVelocityOffset + Math.Sqrt(2 * Joint4.PositionBasedDeceleration * Math.Max(0, Joint4.Upper - before)));
            Assert.True(s.V <= allowed + 1e-12, $"step {k}: v {s.V} over the position-based {allowed}");
            if (allowed < Joint4.Velocity * c.VelocityScale && s.V > 0.99 * allowed) bound = true;
        }

        Assert.True(bound, "the position-based limit never bound, so this test checked nothing");
    }

    [Fact]
    public void AStepTargetChangeMidMoveKeepsJerkAndAccelerationBounded()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = Run(ShapedJointState.AtRest(0), 1.0, Joint1, c, 300);
        Assert.True(s.V > 1.0);
        s = Run(s, -1.0, Joint1, c, 3000);   // reverse at speed
        Assert.True(Math.Abs(s.Q + 1.0) < 1e-6, $"q {s.Q}");
    }

    [Fact]
    public void AStreamOfCloseTargetsIsTrackedWithoutOvershoot()
    {
        // A 48 Hz ramp at 0.5 rad/s, held for 21 steps each, then held still.
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = ShapedJointState.AtRest(0);
        double target = 0;
        for (int command = 0; command < 96; command++)
        {
            target += 0.5 / 48.0;
            s = Run(s, target, Joint1, c, 21);
        }

        double maxPast = 0;
        for (int k = 0; k < 1000; k++)
        {
            JointTrajectoryShaper.Step(ref s, target, Joint1, c);
            maxPast = Math.Max(maxPast, s.Q - target);
        }

        Assert.True(maxPast < 1e-3, $"overshoot {maxPast} rad");
        Assert.True(Math.Abs(s.Q - target) < 1e-6);
    }

    [Fact]
    public void TheFingerIsANewestWinsGateAtConstantSpeed()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState s = ShapedJointState.AtRest(0);

        // Within the deadband of the last move's target: no move.
        for (int k = 0; k < 10; k++) JointTrajectoryShaper.Step(ref s, 0.015, Finger, c);
        Assert.Equal(0.0, s.Q);
        Assert.False(s.Moving);

        // A move to 1 at 5 fractions per second takes 200 steps; a new target during it is not taken.
        for (int k = 0; k < 100; k++) JointTrajectoryShaper.Step(ref s, 1.0, Finger, c);
        Assert.True(s.Moving);
        Assert.Equal(0.5, s.Q, 9);
        for (int k = 0; k < 100; k++) JointTrajectoryShaper.Step(ref s, 0.3, Finger, c);
        Assert.Equal(1.0, s.Q);
        Assert.False(s.Moving);

        // When it ends, the target in force then is the next move.
        JointTrajectoryShaper.Step(ref s, 0.3, Finger, c);
        Assert.True(s.Moving);
        Assert.Equal(0.3, s.To);
        for (int k = 0; k < 200; k++) JointTrajectoryShaper.Step(ref s, 0.3, Finger, c);
        Assert.Equal(0.3, s.Q, 12);

        // A target outside 0..1 is clamped.
        for (int k = 0; k < 400; k++) JointTrajectoryShaper.Step(ref s, 7.0, Finger, c);
        Assert.Equal(1.0, s.Q);
    }

    /// <summary>All seven FR3 v2 joints, as core/RobotProfiles/fr3-duo.json gives them (acceleration 10, jerk 5000).</summary>
    internal static readonly ShapedJointLimits[] Fr3Arm =
    {
        Joint1,
        Joint2,
        ShapedJointLimits.Joint(3, -2.9007400166666666, 2.9007400166666666, -2.9007400166666666, 2.9007400166666666, 2.62, 10, 5000, 3.5, 0.20053055661577582),
        Joint4,
        ShapedJointLimits.Joint(5, -2.87630335, 2.87630335, -2.87630335, 2.87630335, 5.26, 10, 5000, 17, 0.5738321197718637),
        ShapedJointLimits.Joint(6, 0.43982265, 4.62163335, 0.43982265, 4.62163335, 4.18, 10, 5000, 5.5, 0.4884649342105264),
        ShapedJointLimits.Joint(7, -3.05083335, 3.05083335, -3.05083335, 3.05083335, 5.26, 10, 5000, 17, 0.4591952376425851),
    };

    public static IEnumerable<object[]> Seeds() => Enumerable.Range(1, 20).Select(i => new object[] { (ulong)i });

    [Theory]
    [MemberData(nameof(Seeds))]
    public void RandomTargetsAnywhereInRange_NeverBreakALimit(ulong seed)
    {
        // Targets drawn anywhere in (and past) each joint's range, changed at random intervals from one step to
        // 0.4 s, so reversals at full speed and runs at the limits happen: every step of every joint is checked
        // for velocity, acceleration, jerk and position.
        var c = new TrajectoryShaperConfig(Fr3Arm);
        var rng = new SeededRng(seed);
        foreach (ShapedJointLimits l in Fr3Arm)
        {
            ShapedJointState s = ShapedJointState.AtRest(0.5 * (l.Lower + l.Upper));
            for (int segment = 0; segment < 200; segment++)
            {
                double target = l.Lower - 0.2 + (l.Upper - l.Lower + 0.4) * rng.NextDouble();
                s = Run(s, target, l, c, 1 + (int)(400 * rng.NextDouble()));
            }
        }
    }

    [Fact]
    public void ShapingIsDeterministic()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState a = ShapedJointState.AtRest(0.1), b = ShapedJointState.AtRest(0.1);
        for (int k = 0; k < 2000; k++)
        {
            double target = Math.Sin(k * 0.01);
            JointTrajectoryShaper.Step(ref a, target, Joint1, c);
            JointTrajectoryShaper.Step(ref b, target, Joint1, c);
            Assert.Equal(BitConverter.DoubleToInt64Bits(a.Q), BitConverter.DoubleToInt64Bits(b.Q));
            Assert.Equal(BitConverter.DoubleToInt64Bits(a.V), BitConverter.DoubleToInt64Bits(b.V));
        }
    }

    [Fact]
    public void SteppingAllocatesNothing()
    {
        TrajectoryShaperConfig c = Config();
        ShapedJointState joint = ShapedJointState.AtRest(0), finger = ShapedJointState.AtRest(0);
        int k = 0;
        AllocationAssert.Zero(() =>
        {
            double target = (k++ / 500) % 2 == 0 ? 1.0 : -1.0;
            JointTrajectoryShaper.Step(ref joint, target, Joint1, c);
            JointTrajectoryShaper.Step(ref finger, target, Finger, c);
        });
    }

    [Fact]
    public void ConfigRefusesBadLimitsAndDuplicateMotors()
    {
        Assert.Throws<ArgumentException>(() => ShapedJointLimits.Joint(1, 1, -1, 1, -1, 1, 1, 1));
        Assert.Throws<ArgumentException>(() => ShapedJointLimits.Joint(1, -1, 1, -2, 1, 1, 1, 1));
        Assert.Throws<ArgumentException>(() => ShapedJointLimits.Joint(1, -1, 1, -1, 1, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => ShapedJointLimits.Finger(8, 0, 0.02));
        Assert.Throws<ArgumentException>(() => new TrajectoryShaperConfig(new[] { Joint1, Joint1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new TrajectoryShaperConfig(new[] { Joint1 }, velocityScale: 1.5));
        Assert.Throws<ArgumentOutOfRangeException>(() => new TrajectoryShaperConfig(new[] { Joint1 }, settleSeconds: 0.0005));
        Assert.Equal(3, Config().IndexOf(8));
        Assert.Equal(-1, Config().IndexOf(9));
    }
}
