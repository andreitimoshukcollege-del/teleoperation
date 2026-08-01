using System.Numerics;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class ConstantVelocityPredictorTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>
    /// Float tolerance for a quantity that has been through one differencing and one
    /// multiplication. Tight enough that a wrong formula fails; loose enough that float rounding
    /// on a 32-bit quaternion product does not.
    /// </summary>
    private const float Tolerance = 1e-4f;

    private static ManualClock NewClock() => new ManualClock(TicksPerSecond);

    private static PredictorConfig Config(
        long maxHorizonTicks = 400,
        long maxObservationGapTicks = 100,
        int historyCapacity = 8,
        float maxLinearSpeed = 10f,
        float maxAngularSpeed = 10f) =>
        new PredictorConfig(
            maxHorizonTicks,
            maxObservationGapTicks,
            historyCapacity,
            // The four fields const-vel documents that it ignores, set to values that would be
            // catastrophic if it ever started reading them.
            smoothingAlpha: 99f,
            smoothingBeta: -99f,
            processNoise: -1f,
            measurementNoise: -1f,
            maxLinearSpeed,
            maxAngularSpeed);

    private static ConstantVelocityPredictor NewPredictor(PredictorConfig? config = null) =>
        new ConstantVelocityPredictor(config ?? Config(), NewClock());

    // --- A synthetic trajectory with exactly constant linear and angular velocity. -------------

    private static readonly Vector3 TrajectoryOrigin = new Vector3(0.1f, -0.2f, 0.3f);
    private static readonly Vector3 TrajectoryVelocity = new Vector3(0.5f, -1.25f, 2f);

    /// <summary>1.2 rad/s about a normalized, deliberately non-axis-aligned axis.</summary>
    private static readonly Vector3 TrajectoryAngularRate =
        Vector3.Normalize(new Vector3(1f, 2f, -2f)) * 1.2f;

    private static readonly Quaternion TrajectoryInitialRotation =
        Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.3f);

    private static Pose TrajectoryAt(long ticks)
    {
        float seconds = (float)ticks / TicksPerSecond;
        return new Pose(
            TrajectoryOrigin + TrajectoryVelocity * seconds,
            MotionMath.IntegrateWorld(TrajectoryInitialRotation, TrajectoryAngularRate * seconds));
    }

    private static Stamped<Pose> TrajectorySample(long ticks) =>
        new Stamped<Pose>(ticks, TrajectoryAt(ticks));

    private static Stamped<Pose> Sample(long captureTicks, Vector3 position, Quaternion? rotation = null) =>
        new Stamped<Pose>(captureTicks, new Pose(position, rotation ?? Quaternion.Identity));

    private static void AssertSamePose(Pose expected, Pose actual)
    {
        Assert.Equal(expected.Position, actual.Position);
        Assert.Equal(expected.Rotation, actual.Rotation);
    }

    private static void AssertPoseClose(Pose expected, Pose actual, float tolerance = Tolerance)
    {
        Assert.Equal(expected.Position.X, actual.Position.X, tolerance);
        Assert.Equal(expected.Position.Y, actual.Position.Y, tolerance);
        Assert.Equal(expected.Position.Z, actual.Position.Z, tolerance);
        Assert.Equal(0f, PoseMath.OrientationErrorRadians(expected, actual), tolerance);
    }

    private static void AssertSameDiagnostics(PredictorDiagnostics expected, PredictorDiagnostics actual)
    {
        Assert.Equal(expected.HorizonTicks, actual.HorizonTicks);
        Assert.Equal(expected.LastObservationTicks, actual.LastObservationTicks);
        Assert.Equal(expected.AcceptedObservations, actual.AcceptedObservations);
        Assert.Equal(expected.RejectedObservations, actual.RejectedObservations);
        Assert.Equal(expected.HasUncertainty, actual.HasUncertainty);
    }

    // 1. Fail fast on configuration that would otherwise silently produce wrong output forever.
    [Fact]
    public void Constructor_RejectsAHistoryCapacityBelowTwo()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(historyCapacity: 1), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(historyCapacity: 0), NewClock()));
    }

    [Fact]
    public void Constructor_RejectsInvalidBounds()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(maxHorizonTicks: -1), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(maxObservationGapTicks: 0), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(maxLinearSpeed: -1f), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(maxAngularSpeed: -1f), NewClock()));
    }

    [Fact]
    public void Constructor_RejectsAClockWithANonPositiveRate()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ConstantVelocityPredictor(Config(), new ManualClock(0)));
        Assert.Throws<ArgumentNullException>(
            () => new ConstantVelocityPredictor(Config(), null!));
    }

    // 2. Documented pre-observation value.
    [Fact]
    public void Predict_BeforeAnyObservation_ReturnsExactlyPoseIdentity()
    {
        var predictor = NewPredictor();

        Pose predicted = predictor.Predict(5_000);

        Assert.Equal(Vector3.Zero, predicted.Position);
        Assert.Equal(Quaternion.Identity, predicted.Rotation);
        Assert.NotEqual(default(Quaternion), predicted.Rotation);
        AssertSameDiagnostics(PredictorDiagnostics.None, predictor.Diagnostics);
    }

    [Fact]
    public void Predict_WithASingleObservation_HoldsThatPoseExactly()
    {
        var predictor = NewPredictor();
        Stamped<Pose> only = TrajectorySample(100);
        predictor.Observe(only);

        // No pair, so no rate: it must hold rather than invent a velocity.
        AssertSamePose(only.Value, predictor.Predict(400));
    }

    // 3. Determinism.
    [Fact]
    public void IdenticalCallSequences_ProduceBitIdenticalOutput()
    {
        var a = NewPredictor();
        var b = NewPredictor();

        for (int i = 1; i <= 200; i++)
        {
            // Accepted, out-of-window, duplicate and mid-window reinsertions all in one sequence.
            long captureTicks = i % 7 == 0 ? i - 5 : (i % 11 == 0 ? i - 1 : i * 10);
            var obs = new Stamped<Pose>(
                captureTicks,
                new Pose(
                    new Vector3(i * 0.01f, i * -0.02f, i * 0.03f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitY, i * 0.017f)));

            a.Observe(obs);
            b.Observe(obs);

            long target = i * 10 + 37;
            AssertSamePose(a.Predict(target), b.Predict(target));
            AssertSameDiagnostics(a.Diagnostics, b.Diagnostics);
        }
    }

    [Fact]
    public void Predict_RepeatedWithTheSameTarget_ReturnsTheSameValue()
    {
        var predictor = NewPredictor();
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));

        AssertSamePose(predictor.Predict(120), predictor.Predict(120));
    }

    // 4. Exact recovery of a known constant-velocity, constant-angular-rate trajectory.
    [Theory]
    [InlineData(0)]
    [InlineData(50)]
    [InlineData(100)]
    [InlineData(200)]
    [InlineData(400)]
    public void Predict_OnAConstantVelocityTrajectory_RecoversGroundTruthAtEveryHorizon(long horizonTicks)
    {
        var predictor = NewPredictor();
        for (long t = 0; t <= 100; t += 20)
        {
            predictor.Observe(TrajectorySample(t));
        }

        long newestTicks = 100;
        Pose predicted = predictor.Predict(newestTicks + horizonTicks);

        AssertPoseClose(TrajectoryAt(newestTicks + horizonTicks), predicted, 1e-3f);
        Assert.Equal(horizonTicks, predictor.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void Predict_AtTheNewestObservationsOwnStamp_ReproducesItBitIdentically()
    {
        var predictor = NewPredictor();
        predictor.Observe(TrajectorySample(0));
        Stamped<Pose> newest = TrajectorySample(20);
        predictor.Observe(newest);

        // Zero horizon: no integration at all, so this must be exact rather than merely close.
        AssertSamePose(newest.Value, predictor.Predict(20));
    }

    [Fact]
    public void Predict_UsesOnlyTheTwoNewestSamples_NotAWholeWindowFit()
    {
        var predictor = NewPredictor();
        // A fast leg then a slow one. A whole-window fit would average them; this predictor must
        // report the newest leg only.
        predictor.Observe(Sample(0, new Vector3(0f, 0f, 0f)));
        predictor.Observe(Sample(10, new Vector3(0.1f, 0f, 0f)));   // 10 m/s
        predictor.Observe(Sample(20, new Vector3(0.11f, 0f, 0f)));  // 1 m/s

        Pose predicted = predictor.Predict(30);

        // 1 m/s over 10 ms = 0.01 m, so 0.12 -- not the 0.165 a whole-window fit would give.
        Assert.Equal(0.12f, predicted.Position.X, Tolerance);
    }

    // 5. Ordering: in-window reinsertion, out-of-window rejection, duplicates.
    [Fact]
    public void Observe_InWindowReinsertion_IsAcceptedAndDoesNotPerturbTheCurrentRate()
    {
        var predictor = NewPredictor(Config(historyCapacity: 8));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(10));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(30));

        Pose before = predictor.Predict(130);

        // 15 is newer than the oldest retained sample (0) and duplicates nothing, so it is spliced
        // into the middle of the window.
        predictor.Observe(TrajectorySample(15));

        Assert.Equal(5, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(0, predictor.Diagnostics.RejectedObservations);
        Assert.Equal(30, predictor.Diagnostics.LastObservationTicks);

        // The rate comes from the two newest samples (20, 30), which the reinsertion did not touch,
        // so the current estimate is bit-identical. This is the order-independence property.
        AssertSamePose(before, predictor.Predict(130));
    }

    [Fact]
    public void Observe_OutOfWindowSample_IsRejectedAndCounted_AndChangesNothing()
    {
        var predictor = NewPredictor(Config(historyCapacity: 3));
        predictor.Observe(TrajectorySample(10));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(30));

        Pose before = predictor.Predict(130);

        predictor.Observe(Sample(5, new Vector3(-99f, -99f, -99f)));   // older than the oldest
        predictor.Observe(Sample(10, new Vector3(-99f, -99f, -99f)));  // equal to the oldest

        Assert.Equal(3, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(2, predictor.Diagnostics.RejectedObservations);
        AssertSamePose(before, predictor.Predict(130));
    }

    [Fact]
    public void Observe_WhenTheBufferIsFull_EvictionNarrowsTheAcceptanceWindow()
    {
        var predictor = NewPredictor(Config(historyCapacity: 3));
        predictor.Observe(TrajectorySample(10));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(30));

        // 15 is in window right now (newer than the oldest retained, 10).
        var alsoNew = NewPredictor(Config(historyCapacity: 3));
        alsoNew.Observe(TrajectorySample(10));
        alsoNew.Observe(TrajectorySample(20));
        alsoNew.Observe(TrajectorySample(30));
        alsoNew.Observe(TrajectorySample(15));
        Assert.Equal(4, alsoNew.Diagnostics.AcceptedObservations);
        Assert.Equal(0, alsoNew.Diagnostics.RejectedObservations);

        // ...but accepting a newer sample first evicts the oldest, raising the window floor to 20,
        // and the same 15 is then out of window.
        predictor.Observe(TrajectorySample(40));
        predictor.Observe(TrajectorySample(15));
        Assert.Equal(4, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(1, predictor.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void Observe_TwiceWithAnIdenticalSample_LeavesTheSameStateAsOnce()
    {
        var once = NewPredictor();
        var twice = NewPredictor();

        once.Observe(TrajectorySample(0));
        once.Observe(TrajectorySample(20));

        twice.Observe(TrajectorySample(0));
        twice.Observe(TrajectorySample(20));
        twice.Observe(TrajectorySample(20));
        twice.Observe(TrajectorySample(0));

        AssertSamePose(once.Predict(120), twice.Predict(120));
        Assert.Equal(2, twice.Diagnostics.AcceptedObservations);
        Assert.Equal(2, twice.Diagnostics.RejectedObservations);
        Assert.Equal(once.Diagnostics.LastObservationTicks, twice.Diagnostics.LastObservationTicks);
    }

    [Fact]
    public void Observe_DuplicateStampInTheMiddleOfTheWindow_IsRejected()
    {
        var predictor = NewPredictor(Config(historyCapacity: 8));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(10));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(30));

        Pose before = predictor.Predict(130);
        predictor.Observe(Sample(10, new Vector3(-99f, -99f, -99f)));

        Assert.Equal(1, predictor.Diagnostics.RejectedObservations);
        AssertSamePose(before, predictor.Predict(130));
    }

    // 6. Gap policy: collapse the rate rather than difference across a stall.
    [Fact]
    public void Observe_AcrossAMultiHundredMillisecondGap_CollapsesTheRateInsteadOfFabricatingOne()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 100, historyCapacity: 8));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));

        // Sanity: the rate is real before the gap, so a 100 ms prediction actually moves.
        Assert.True(
            MathF.Abs(predictor.Predict(120).Position.X - TrajectoryAt(20).Position.X) > 0.01f,
            "precondition: the predictor should be extrapolating before the gap");

        // 500 ms with nothing arriving, then one sample. Differencing across it would produce a
        // fictitious velocity from a whole gap's worth of real motion; the policy is to zero it.
        Stamped<Pose> afterGap = TrajectorySample(520);
        predictor.Observe(afterGap);

        AssertSamePose(afterGap.Value, predictor.Predict(620));
        AssertSamePose(afterGap.Value, predictor.Predict(920));
        Assert.Equal(3, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(0, predictor.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void Observe_AfterAGap_RecoversTheRateOnTheNextInBoundPair()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 100, historyCapacity: 8));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(520)); // gap: rate collapses
        predictor.Observe(TrajectorySample(540)); // in-bound pair: rate is real again

        AssertPoseClose(TrajectoryAt(640), predictor.Predict(640), 1e-3f);
    }

    [Fact]
    public void Observe_AtExactlyTheGapBound_IsStillDifferenced()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 100, historyCapacity: 8));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(100)); // gap == bound, which is not "exceeds"

        AssertPoseClose(TrajectoryAt(200), predictor.Predict(200), 1e-3f);
    }

    // 7. Clamps.
    [Fact]
    public void Predict_ClampsTheHorizonToMaxHorizonTicks_OnTheFutureSideOnly()
    {
        var predictor = NewPredictor(Config(maxHorizonTicks: 100));
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));

        // Asked for 300 ms ahead, capped at 100: the answer is the 100 ms prediction, and the
        // reported horizon is the clamped one, because that is what was actually extrapolated.
        Pose clamped = predictor.Predict(320);
        Assert.Equal(100, predictor.Diagnostics.HorizonTicks);
        AssertPoseClose(TrajectoryAt(120), clamped, 1e-3f);

        // A target in the past is interpolation, not extrapolation, and is not clamped.
        Pose interpolated = predictor.Predict(-280);
        Assert.Equal(-300, predictor.Diagnostics.HorizonTicks);
        AssertPoseClose(TrajectoryAt(-280), interpolated, 1e-3f);
    }

    [Fact]
    public void Predict_ClampsLinearSpeedBeforeExtrapolating()
    {
        var predictor = NewPredictor(Config(maxLinearSpeed: 1f, maxAngularSpeed: 100f));
        // 0.5 m in 10 ms is 50 m/s: exactly the mis-stamped-sample spike the cap exists for.
        predictor.Observe(Sample(0, Vector3.Zero));
        predictor.Observe(Sample(10, new Vector3(0.5f, 0f, 0f)));

        Pose predicted = predictor.Predict(110); // 100 ms ahead

        // Clamped to 1 m/s before the multiply: 0.1 m of travel, not 5 m.
        Assert.Equal(0.6f, predicted.Position.X, Tolerance);
    }

    [Fact]
    public void Predict_ClampsAngularSpeedBeforeExtrapolating()
    {
        float cap = 1f;
        var predictor = NewPredictor(Config(maxLinearSpeed: 100f, maxAngularSpeed: cap));
        var start = Quaternion.Identity;
        var spun = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 1f); // 1 rad in 10 ms = 100 rad/s
        predictor.Observe(Sample(0, Vector3.Zero, start));
        predictor.Observe(Sample(10, Vector3.Zero, spun));

        Pose predicted = predictor.Predict(110); // 100 ms ahead

        // Clamped to 1 rad/s: 0.1 rad further than the newest sample's 1 rad, not 10 rad further.
        var expected = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 1.1f));
        Assert.Equal(0f, PoseMath.OrientationErrorRadians(expected, predicted), Tolerance);
    }

    [Fact]
    public void Diagnostics_NeverClaimUncertainty()
    {
        var predictor = NewPredictor();
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));
        predictor.Predict(120);

        Assert.False(predictor.Diagnostics.HasUncertainty);
    }

    // 8. Reset.
    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var predictor = NewPredictor();
        for (long t = 0; t <= 100; t += 20)
        {
            predictor.Observe(TrajectorySample(t));
        }

        predictor.Observe(Sample(1, Vector3.Zero)); // rejected, bumping the counter
        predictor.Predict(200);

        predictor.Reset();

        AssertSameDiagnostics(PredictorDiagnostics.None, predictor.Diagnostics);

        Pose predicted = predictor.Predict(1_000);
        Assert.Equal(Vector3.Zero, predicted.Position);
        Assert.Equal(Quaternion.Identity, predicted.Rotation);

        // The window floor cleared with it: the next trial's first sample is accepted at a stamp
        // far below the 100 the previous trial reached, and no residual velocity leaks across.
        Stamped<Pose> next = Sample(0, new Vector3(8f, 8f, 8f));
        predictor.Observe(next);
        AssertSamePose(next.Value, predictor.Predict(400));
        Assert.Equal(1, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(0, predictor.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void Reset_MakesTheInstanceBehaveExactlyLikeAFreshOne()
    {
        var reused = NewPredictor();
        var fresh = NewPredictor();

        for (int i = 1; i <= 50; i++)
        {
            reused.Observe(Sample(i * 100, new Vector3(i, i, i)));
            reused.Predict(i * 100 + 50);
        }

        reused.Reset();

        for (int i = 1; i <= 30; i++)
        {
            long captureTicks = i % 4 == 0 ? i - 2 : i * 10;
            var obs = new Stamped<Pose>(
                captureTicks,
                new Pose(
                    new Vector3(i * 0.05f, 0f, i * -0.01f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, i * 0.02f)));
            reused.Observe(obs);
            fresh.Observe(obs);

            AssertSamePose(fresh.Predict(i * 10 + 25), reused.Predict(i * 10 + 25));
            AssertSameDiagnostics(fresh.Diagnostics, reused.Diagnostics);
        }
    }

    // 9. Allocation-free hot path, including the insertion sort.
    [Fact]
    public void Observe_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks += 10;
            predictor.Observe(new Stamped<Pose>(
                captureTicks,
                new Pose(new Vector3(captureTicks * 0.001f, 1f, 2f), Quaternion.Identity)));
        });
    }

    [Fact]
    public void Observe_WithAMidWindowInsertion_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor(Config(historyCapacity: 8, maxObservationGapTicks: 1_000_000));
        long captureTicks = 0;

        // Alternate a new newest sample with a reinsertion behind it, so the insertion sort shifts
        // entries on most iterations rather than always appending at the end.
        AllocationAssert.Zero(() =>
        {
            captureTicks += 100;
            predictor.Observe(new Stamped<Pose>(captureTicks, Pose.Identity));
            predictor.Observe(new Stamped<Pose>(captureTicks - 50, Pose.Identity));
        });
    }

    [Fact]
    public void Observe_WhenRejected_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(1_000_000, Vector3.Zero));
        Stamped<Pose> stale = Sample(1, Vector3.One);

        AllocationAssert.Zero(() => predictor.Observe(stale));
    }

    [Fact]
    public void Predict_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));
        long target = 20;

        AllocationAssert.Zero(() =>
        {
            target++;
            predictor.Predict(target);
        });
    }

    [Fact]
    public void Predict_BeforeAnyObservation_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();

        AllocationAssert.Zero(() => predictor.Predict(1_000));
    }
}
