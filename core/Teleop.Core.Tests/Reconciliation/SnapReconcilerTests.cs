using System.Numerics;
using Teleop.Core.Metrics;
using Teleop.Core.Reconciliation;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Reconciliation;

public class SnapReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    private const float Tolerance = 1e-4f;

    private const string CorrectionMagnitudeMm = "correction_magnitude_mm";
    private const string CorrectionMagnitudeDeg = "correction_magnitude_deg";
    private const string TimeToConvergenceMs = "time_to_convergence_ms";
    private const string JerkMmS3 = "jerk_mm_s3";

    private static ReconcilerConfig Config(
        float positionToleranceMeters = 1e-4f,
        float orientationToleranceRadians = 1e-4f) =>
        new ReconcilerConfig(
            positionToleranceMeters,
            orientationToleranceRadians,
            // The four fields snap documents that it ignores, set to values that would visibly
            // change its behaviour if it ever started reading them: a one-tick convergence bound, a
            // 1 mm/s rate cap that would stretch every correction over minutes, and a zero rollback
            // depth.
            maxTimeToConvergenceTicks: 1,
            maxCorrectionLinearSpeedMetersPerSecond: 0.001f,
            maxCorrectionAngularSpeedRadPerSecond: 0.001f,
            rollbackHistoryCapacity: 0);

    private sealed class Fixture
    {
        public readonly InMemoryMetricTracker Metrics = new InMemoryMetricTracker(256);
        public readonly SnapReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new SnapReconciler(
                config ?? Config(), Metrics, new ManualClock(TicksPerSecond));
        }

        public int CountOf(string name)
        {
            int count = 0;
            for (int i = 0; i < Metrics.Count; i++)
            {
                if (Metrics[i].Name == name)
                {
                    count++;
                }
            }

            return count;
        }
    }

    private static Pose PoseAt(float x, float angleRadians = 0f) =>
        new Pose(new Vector3(x, 0f, 0f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angleRadians));

    private static void AssertSamePose(Pose expected, Pose actual)
    {
        Assert.Equal(expected.Position, actual.Position);
        Assert.Equal(expected.Rotation, actual.Rotation);
    }

    /// <summary>
    /// Drives three pass-through frames of steady 1 m/s motion so the three-sample jerk history is
    /// full, and returns the tick and predicted pose the next frame should use. Jerk is a third
    /// derivative: without a trajectory to interrupt there is nothing to differentiate.
    /// </summary>
    private static (long NextTicks, Pose NextPredicted) DriveSteadyMotion(Fixture fixture)
    {
        for (int frame = 0; frame < 3; frame++)
        {
            long ticks = frame * FrameTicks;
            Pose predicted = PoseAt(frame * 0.01f);
            AssertSamePose(predicted, fixture.Reconciler.Reconcile(predicted, ticks));
        }

        return (3 * FrameTicks, PoseAt(0.03f));
    }

    // 1. Constructor validation.
    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentNullException>(() => new SnapReconciler(Config(), null!, clock));
        Assert.Throws<ArgumentNullException>(() => new SnapReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SnapReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SnapReconciler(Config(positionToleranceMeters: -1f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SnapReconciler(Config(orientationToleranceRadians: -1f), metrics, clock));
    }

    [Fact]
    public void IsConverged_OnAFreshInstance_IsTrue()
    {
        var fixture = new Fixture();

        Assert.True(fixture.Reconciler.IsConverged);
    }

    // 2. Bounded convergence, at the tightest bound the interface allows: exactly one call.
    [Fact]
    public void Reconcile_ConvergesInExactlyOneCall_RegardlessOfMagnitude()
    {
        var fixture = new Fixture();
        Pose predictedAtCapture = PoseAt(0f);
        var authoritative = new Stamped<Pose>(100, PoseAt(50f, 2f)); // an absurd 50 m disagreement

        fixture.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);

        // Observe does not change the visible output -- it queues.
        Assert.False(fixture.Reconciler.IsConverged);

        Pose output = fixture.Reconciler.Reconcile(PoseAt(0.01f), 110);

        // Exactly the authoritative pose, bit for bit: a snap has no residual and no blend factor.
        AssertSamePose(authoritative.Value, output);
        Assert.True(fixture.Reconciler.IsConverged);

        // And the next frame is untouched pass-through -- the correction is not re-applied.
        Pose next = PoseAt(0.02f);
        AssertSamePose(next, fixture.Reconciler.Reconcile(next, 120));
        Assert.True(fixture.Reconciler.IsConverged);
    }

    [Fact]
    public void Reconcile_RecordsTimeToConvergenceOfZero()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(new Stamped<Pose>(100, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);

        fixture.Reconciler.Reconcile(PoseAt(0f), 110);

        Assert.True(fixture.Metrics.TryGetLatest(TimeToConvergenceMs, out double value, out long ticks));
        Assert.Equal(0.0, value);
        Assert.Equal(110, ticks);
    }

    // 3. The C1-continuity witness. This reconciler CANNOT satisfy IReconciler's "no position or
    //    velocity discontinuity" clause -- a snap is that discontinuity by definition -- so this
    //    test proves and quantifies the violation instead of asserting continuity. If a change ever
    //    made this pass by producing a small jerk, that is a regression: every other reconciler is
    //    calibrated against this baseline, and a quietly-smoothed baseline shrinks their measured
    //    benefit for reasons that have nothing to do with them.
    [Fact]
    public void Reconcile_OnASnap_ProducesAJerkFarBeyondAnythingC1Continuous()
    {
        // Derived, not arbitrary. Steady 1 m/s motion sampled every 10 ms, interrupted by a 0.5 m
        // snap on the fourth frame:
        //   v01 = v12 = 1 m/s, v23 = (0.02 + 0.5 - 0.02) / 0.01 = 50 m/s
        //   a0  = 0 m/s^2, a1 = (50 - 1) / 0.01 = 4900 m/s^2
        //   jerk = (4900 - 0) / 0.01 = 490 000 m/s^3 = 4.9e8 mm/s^3
        // The threshold below is two thirds of that analytic value, so it cannot be met by anything
        // that spreads the same correction over even two frames.
        const double analyticJerkMillimetresPerSecondCubed = 4.9e8;
        const double grossDiscontinuityThreshold = 1e8;

        var fixture = new Fixture();
        (long ticks, Pose predicted) = DriveSteadyMotion(fixture);

        // Truth for the capture instant of the third frame disagrees by 0.5 m.
        var authoritative = new Stamped<Pose>(2 * FrameTicks, PoseAt(0.52f));
        fixture.Reconciler.Observe(authoritative, PoseAt(0.02f), PredictorDiagnostics.None);

        Pose snapped = fixture.Reconciler.Reconcile(predicted, ticks);
        AssertSamePose(authoritative.Value, snapped);

        Assert.True(fixture.Metrics.TryGetLatest(JerkMmS3, out double jerk, out long jerkTicks));
        Assert.Equal(ticks, jerkTicks);
        Assert.True(
            jerk > grossDiscontinuityThreshold,
            $"expected a gross jerk witness above {grossDiscontinuityThreshold:E1} mm/s^3, got {jerk:E3}");
        Assert.Equal(analyticJerkMillimetresPerSecondCubed, jerk, 0.01 * analyticJerkMillimetresPerSecondCubed);

        // The velocity discontinuity, stated directly: displayed speed goes from 1 m/s to 50 m/s
        // inside one 10 ms frame. C1 continuity forbids exactly this.
        const float displayedSpeedBefore = 1f;
        float displayedSpeedAtSnap = (snapped.Position.X - 0.02f) / ((float)FrameTicks / TicksPerSecond);
        Assert.Equal(50f, displayedSpeedAtSnap, 1e-3f);
        Assert.True(displayedSpeedAtSnap - displayedSpeedBefore > 40f);
    }

    [Fact]
    public void Reconcile_WithoutEnoughHistory_EmitsNoJerkRatherThanAnInventedOne()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(new Stamped<Pose>(0, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);

        // The very first frame of a trial: one output point, so no third derivative exists.
        fixture.Reconciler.Reconcile(PoseAt(0f), 0);

        Assert.Equal(0, fixture.CountOf(JerkMmS3));
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
    }

    // 4. Within tolerance: no correction at all. This is what makes `none` + `snap` degenerate to
    //    true pass-through, which docs/metrics.md 8 requires in every comparison.
    [Fact]
    public void Observe_WithinTolerance_BeginsNoCorrectionAndEmitsNoMetrics()
    {
        var fixture = new Fixture(Config(positionToleranceMeters: 0.001f, orientationToleranceRadians: 0.001f));
        Pose predictedAtCapture = PoseAt(1f, 0.5f);
        var authoritative = new Stamped<Pose>(100, PoseAt(1.0005f, 0.5005f)); // inside both tolerances

        fixture.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0, fixture.Metrics.Count);

        // Pass-through must be bit-exact, not "close": the operator sees the prediction unmodified.
        Pose predicted = PoseAt(1.001f, 0.501f);
        AssertSamePose(predicted, fixture.Reconciler.Reconcile(predicted, 110));
    }

    [Fact]
    public void Observe_ExactlyAtTolerance_IsStillWithinIt()
    {
        var fixture = new Fixture(Config(positionToleranceMeters: 0.01f, orientationToleranceRadians: 1f));
        fixture.Reconciler.Observe(
            new Stamped<Pose>(100, PoseAt(0.01f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0, fixture.Metrics.Count);
    }

    [Fact]
    public void Observe_OrientationOnlyDisagreement_StillCorrects()
    {
        var fixture = new Fixture(Config(positionToleranceMeters: 0.01f, orientationToleranceRadians: 0.01f));
        // Positions identical, orientations 0.5 rad apart: either axis alone must trigger.
        fixture.Reconciler.Observe(
            new Stamped<Pose>(100, PoseAt(1f, 0.5f)), PoseAt(1f, 0f), PredictorDiagnostics.None);

        Assert.False(fixture.Reconciler.IsConverged);
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));
    }

    // 5. Metric values, cross-checked against PoseMath rather than recomputed by hand.
    [Fact]
    public void Observe_RecordsCorrectionMagnitudeInMillimetresAndDegrees_StampedAtCapture()
    {
        var fixture = new Fixture();
        var predictedAtCapture = new Pose(
            new Vector3(1f, 2f, 3f), Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.2f));
        var authoritativePose = new Pose(
            new Vector3(1.03f, 1.96f, 3f), Quaternion.CreateFromAxisAngle(Vector3.UnitY, 0.5f));
        var authoritative = new Stamped<Pose>(4_242, authoritativePose);

        fixture.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);

        float expectedMeters = PoseMath.PositionErrorMeters(predictedAtCapture, authoritativePose);
        float expectedRadians = PoseMath.OrientationErrorRadians(predictedAtCapture, authoritativePose);

        Assert.True(fixture.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double mm, out long mmTicks));
        Assert.True(fixture.Metrics.TryGetLatest(CorrectionMagnitudeDeg, out double deg, out long degTicks));

        Assert.Equal(expectedMeters * 1000.0, mm, 6);
        Assert.Equal(expectedRadians * (180.0 / Math.PI), deg, 6);

        // Sanity on the units themselves: 0.03 m east and 0.04 m south is 0.05 m, i.e. 50 mm.
        Assert.Equal(50.0, mm, 1e-3);
        Assert.Equal(0.3 * (180.0 / Math.PI), deg, 1e-3);

        // Stamped at the event's own time -- the capture instant -- not at a frame time or a clock
        // read, per IMetricSink.Record's `ticks` parameter.
        Assert.Equal(4_242, mmTicks);
        Assert.Equal(4_242, degTicks);
    }

    // 6. Duplicate Observe must not be counted as a second correction.
    [Fact]
    public void Observe_WithADuplicateStamp_IsNotCountedAsASecondCorrection()
    {
        var fixture = new Fixture();
        var authoritative = new Stamped<Pose>(100, PoseAt(1f));

        fixture.Reconciler.Observe(authoritative, PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Observe(authoritative, PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Observe(authoritative, PoseAt(0f), PredictorDiagnostics.None);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));

        fixture.Reconciler.Reconcile(PoseAt(0f), 110);
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
        Assert.True(fixture.Reconciler.IsConverged);
    }

    [Fact]
    public void Observe_OutOfOrder_IsIgnoredWholeAndDoesNotRetarget()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(new Stamped<Pose>(200, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);

        // A stale sample must not replace the newer pending target, and must not emit a metric.
        fixture.Reconciler.Observe(new Stamped<Pose>(199, PoseAt(-99f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        AssertSamePose(PoseAt(1f), fixture.Reconciler.Reconcile(PoseAt(0f), 210));
    }

    [Fact]
    public void Observe_OutOfOrderWhenNothingIsPending_ChangesNothing()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(new Stamped<Pose>(200, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Reconcile(PoseAt(0f), 210);

        fixture.Reconciler.Observe(new Stamped<Pose>(150, PoseAt(-99f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(fixture.Reconciler.IsConverged);
        Pose predicted = PoseAt(0.5f);
        AssertSamePose(predicted, fixture.Reconciler.Reconcile(predicted, 220));
    }

    // 7. Retarget, do not stack.
    [Fact]
    public void Observe_TwiceBeforeOneReconcile_RetargetsRatherThanStacking()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(new Stamped<Pose>(100, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Observe(new Stamped<Pose>(110, PoseAt(2f)), PoseAt(0f), PredictorDiagnostics.None);

        Pose output = fixture.Reconciler.Reconcile(PoseAt(0f), 120);

        // The newest truth wins outright; there is no intermediate hop through the older target.
        AssertSamePose(PoseAt(2f), output);
        Assert.True(fixture.Reconciler.IsConverged);

        // Two real disagreements were measured, so two magnitudes are reported -- but only one
        // correction was ever *applied*, so exactly one convergence event exists.
        Assert.Equal(2, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));

        // The queue is genuinely empty: the next frame passes through.
        Pose next = PoseAt(2.01f);
        AssertSamePose(next, fixture.Reconciler.Reconcile(next, 130));
    }

    // 8. Idempotent Reconcile.
    [Fact]
    public void Reconcile_TwiceAtTheSameTick_ReturnsTheSameStateAndDoesNotAdvanceTwice()
    {
        var fixture = new Fixture();
        (long ticks, Pose predicted) = DriveSteadyMotion(fixture);
        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(0.52f)), PoseAt(0.02f), PredictorDiagnostics.None);

        Pose first = fixture.Reconciler.Reconcile(predicted, ticks);
        int metricsAfterFirst = fixture.Metrics.Count;

        Pose second = fixture.Reconciler.Reconcile(predicted, ticks);

        AssertSamePose(first, second);
        Assert.Equal(metricsAfterFirst, fixture.Metrics.Count);

        // A repeat call ignores `predicted` entirely, as documented: it replays the cached output.
        Pose third = fixture.Reconciler.Reconcile(PoseAt(-777f), ticks);
        AssertSamePose(first, third);
        Assert.Equal(metricsAfterFirst, fixture.Metrics.Count);
    }

    [Fact]
    public void Reconcile_AtAnEarlierTick_IsAlsoANoOp()
    {
        var fixture = new Fixture();
        Pose atTen = fixture.Reconciler.Reconcile(PoseAt(1f), 10);

        Pose rewound = fixture.Reconciler.Reconcile(PoseAt(2f), 5);

        AssertSamePose(atTen, rewound);
    }

    [Fact]
    public void Reconcile_WithNothingPending_ReturnsThePredictionBitIdentically()
    {
        var fixture = new Fixture();
        var predicted = new Pose(
            new Vector3(0.123456f, -7.65f, 3.14159f),
            Quaternion.Normalize(new Quaternion(0.1f, 0.2f, 0.3f, 0.9f)));

        Pose output = fixture.Reconciler.Reconcile(predicted, 1_000);

        AssertSamePose(predicted, output);
        Assert.Equal(0, fixture.Metrics.Count);
    }

    // 9. Predictor uncertainty: this reconciler ignores it, which is how it degrades gracefully.
    [Fact]
    public void Observe_BehavesIdenticallyWithAndWithoutPredictorUncertainty()
    {
        var withoutUncertainty = new Fixture();
        var withUncertainty = new Fixture();

        var authoritative = new Stamped<Pose>(100, PoseAt(1f, 0.4f));
        Pose predictedAtCapture = PoseAt(0f);

        var none = PredictorDiagnostics.None;
        var uncertain = new PredictorDiagnostics(
            horizonTicks: 100,
            lastObservationTicks: 50,
            acceptedObservations: 10,
            rejectedObservations: 1,
            hasUncertainty: true,
            positionSigmaMeters: 0.25f,
            orientationSigmaRadians: 0.1f);

        Assert.False(none.HasUncertainty);
        Assert.True(uncertain.HasUncertainty);

        withoutUncertainty.Reconciler.Observe(authoritative, predictedAtCapture, none);
        withUncertainty.Reconciler.Observe(authoritative, predictedAtCapture, uncertain);

        AssertSamePose(
            withoutUncertainty.Reconciler.Reconcile(predictedAtCapture, 110),
            withUncertainty.Reconciler.Reconcile(predictedAtCapture, 110));

        Assert.True(withoutUncertainty.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double a, out _));
        Assert.True(withUncertainty.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double b, out _));
        Assert.Equal(a, b);
    }

    // 10. Reset.
    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var fixture = new Fixture();
        (long ticks, Pose predicted) = DriveSteadyMotion(fixture);
        fixture.Reconciler.Observe(
            new Stamped<Pose>(9_000, PoseAt(5f)), PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Reconcile(predicted, ticks);
        fixture.Reconciler.Observe(
            new Stamped<Pose>(9_100, PoseAt(6f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.False(fixture.Reconciler.IsConverged);
        fixture.Reconciler.Reset();

        // No pending correction survives.
        Assert.True(fixture.Reconciler.IsConverged);

        // The frame clock baseline cleared: a tick far below the previous trial's is accepted.
        Pose next = PoseAt(0.1f);
        AssertSamePose(next, fixture.Reconciler.Reconcile(next, 0));

        // The accepted-capture baseline cleared: a capture stamp far below the previous trial's
        // 9100 is accepted rather than silently ignored for the whole opening of the next trial.
        fixture.Reconciler.Observe(new Stamped<Pose>(1, PoseAt(0.9f)), PoseAt(0f), PredictorDiagnostics.None);
        Assert.False(fixture.Reconciler.IsConverged);
        AssertSamePose(PoseAt(0.9f), fixture.Reconciler.Reconcile(PoseAt(0.2f), 10));

        // The jerk history cleared: only two output samples exist since the reset, so no third
        // derivative is emitted -- the previous trial's positions must not leak into this one's.
        int jerkSamplesSinceReset = 0;
        for (int i = 0; i < fixture.Metrics.Count; i++)
        {
            if (fixture.Metrics[i].Name == JerkMmS3 && fixture.Metrics[i].Ticks == 10)
            {
                jerkSamplesSinceReset++;
            }
        }

        Assert.Equal(0, jerkSamplesSinceReset);
    }

    [Fact]
    public void Reset_MakesTheInstanceBehaveExactlyLikeAFreshOne()
    {
        var reused = new Fixture();
        var fresh = new Fixture();

        for (int i = 1; i <= 50; i++)
        {
            reused.Reconciler.Observe(
                new Stamped<Pose>(i * 100, PoseAt(i)), PoseAt(0f), PredictorDiagnostics.None);
            reused.Reconciler.Reconcile(PoseAt(i * 0.5f), i * 100 + 10);
        }

        reused.Reconciler.Reset();
        reused.Metrics.Reset();

        for (int i = 1; i <= 30; i++)
        {
            long captureTicks = i % 5 == 0 ? i - 3 : i * 10;
            var authoritative = new Stamped<Pose>(captureTicks, PoseAt(i * 0.02f, i * 0.01f));
            Pose predictedAtCapture = PoseAt(i * 0.021f, i * 0.011f);

            reused.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);
            fresh.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);

            Assert.Equal(fresh.Reconciler.IsConverged, reused.Reconciler.IsConverged);

            Pose predicted = PoseAt(i * 0.022f);
            AssertSamePose(
                fresh.Reconciler.Reconcile(predicted, i * 10 + 5),
                reused.Reconciler.Reconcile(predicted, i * 10 + 5));

            Assert.Equal(fresh.Metrics.Count, reused.Metrics.Count);
            for (int m = 0; m < fresh.Metrics.Count; m++)
            {
                Assert.Equal(fresh.Metrics[m].Name, reused.Metrics[m].Name);
                Assert.Equal(fresh.Metrics[m].Value, reused.Metrics[m].Value);
                Assert.Equal(fresh.Metrics[m].Ticks, reused.Metrics[m].Ticks);
            }
        }
    }

    // 11. Allocation-free hot path.
    [Fact]
    public void Observe_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks++;
            fixture.Reconciler.Observe(
                new Stamped<Pose>(captureTicks, PoseAt(1f, 0.3f)),
                PoseAt(0f),
                PredictorDiagnostics.None);
        });
    }

    [Fact]
    public void Observe_WhenWithinTolerance_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture(Config(positionToleranceMeters: 1f, orientationToleranceRadians: 1f));
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks++;
            fixture.Reconciler.Observe(
                new Stamped<Pose>(captureTicks, PoseAt(0.001f)),
                PoseAt(0f),
                PredictorDiagnostics.None);
        });
    }

    [Fact]
    public void Observe_WhenRejectedAsStale_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(
            new Stamped<Pose>(long.MaxValue / 2, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);
        var stale = new Stamped<Pose>(1, PoseAt(2f));

        AllocationAssert.Zero(
            () => fixture.Reconciler.Observe(stale, PoseAt(0f), PredictorDiagnostics.None));
    }

    [Fact]
    public void Reconcile_WithACorrectionPending_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long ticks = 0;
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks++;
            ticks++;
            fixture.Reconciler.Observe(
                new Stamped<Pose>(captureTicks, PoseAt(captureTicks * 0.001f)),
                PoseAt(0f),
                PredictorDiagnostics.None);
            fixture.Reconciler.Reconcile(PoseAt(0f), ticks);
        });
    }

    [Fact]
    public void Reconcile_WithNothingPending_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long ticks = 0;

        AllocationAssert.Zero(() =>
        {
            ticks++;
            fixture.Reconciler.Reconcile(PoseAt(ticks * 0.001f), ticks);
        });
    }

    [Fact]
    public void Reconcile_WhenIdempotent_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Reconcile(PoseAt(1f), 1_000);

        AllocationAssert.Zero(() => fixture.Reconciler.Reconcile(PoseAt(1f), 1_000));
    }
}
