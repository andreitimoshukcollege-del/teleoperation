using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Metrics;
using Teleop.Core.Reconciliation;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Reconciliation;

/// <summary>
/// <c>exp-smooth-c1</c>. The suite is deliberately the same shape as
/// <c>SpringReconcilerTests</c> — same fixture, same driving helper, same
/// <c>..._ADeliberateTradeoff</c> pins — so that a head-to-head on this axis compares convergence
/// laws rather than test harnesses, plus three things `spring`'s suite does not have:
/// <list type="bullet">
/// <item><see cref="TheOffsetLeavesOnsetAtZeroVelocity"/> — the C1-at-onset property proved by
/// frame-interval refinement rather than asserted.</item>
/// <item><see cref="TheOffsetVelocityDoesNotStepAcrossARetarget"/> — the same proof at a retarget,
/// which is the case a lossy link produces constantly and which `spring` documents but does not
/// test.</item>
/// <item>The three closed-form derivative coefficients, which are what distinguish this law's
/// response from a critically damped one at the same convergence budget.</item>
/// </list>
/// </summary>
public class EasedExponentialReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The step correction under test, 5 cm. Small enough that the uncapped peak offset speed
    /// (2.453 * 0.05 / 0.1 = about 1.23 m/s here) stays well under the rate caps, so the caps are
    /// exercised only by the test that is actually about them.
    /// </summary>
    private const float CorrectionMeters = 0.05f;

    private const float PositionTolerance = 1e-3f;

    /// <summary>
    /// The <c>x</c> solving <c>exp(1 - x - e^(-x)) = 0.01</c>. A private copy of the
    /// implementation's <c>EasedSettleConstant</c>, kept here only so
    /// <see cref="EasedSettleConstantMatchesItsDefiningEquation"/> can check the literal against its
    /// own defining equation. Every other test derives the time constant from
    /// <see cref="EasedExponentialReconciler.TimeConstantSeconds"/> instead, so it asserts against
    /// the implementation rather than against a second copy of the formula.
    /// </summary>
    private const double EasedSettleConstant = 5.6014778;

    /// <summary>Peak of <c>|d/dx exp(1 - x - e^(-x))|</c>, at <c>x = ln(phi^2) = 0.96242</c>.</summary>
    private const double PeakSpeedCoefficient = 0.43797148;

    /// <summary>Peak of <c>|d2/dx2 ...|</c>, attained at <c>x = 0</c>.</summary>
    private const double PeakAccelerationCoefficient = 1.0;

    /// <summary>Peak of <c>|d3/dx3 ...|</c>, at <c>x = 0.27159</c>.</summary>
    private const double PeakJerkCoefficient = 1.24961663;

    private const string CorrectionMagnitudeMm = "correction_magnitude_mm";
    private const string CorrectionMagnitudeDeg = "correction_magnitude_deg";
    private const string TimeToConvergenceMs = "time_to_convergence_ms";
    private const string JerkMmS3 = "jerk_mm_s3";

    private static ReconcilerConfig Config(
        float positionToleranceMeters = PositionTolerance,
        float orientationToleranceRadians = 1e-3f,
        long maxTimeToConvergenceTicks = BudgetTicks,
        float maxCorrectionLinearSpeedMetersPerSecond = 100f,
        float maxCorrectionAngularSpeedRadPerSecond = 100f,
        int rollbackHistoryCapacity = 0) =>
        new ReconcilerConfig(
            positionToleranceMeters,
            orientationToleranceRadians,
            maxTimeToConvergenceTicks,
            maxCorrectionLinearSpeedMetersPerSecond,
            maxCorrectionAngularSpeedRadPerSecond,
            rollbackHistoryCapacity);

    private sealed class Fixture
    {
        public readonly InMemoryMetricTracker Metrics = new InMemoryMetricTracker(8192);
        public readonly EasedExponentialReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new EasedExponentialReconciler(
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
    /// One step correction, driven the way the pipeline drives it, and identical to
    /// <c>SpringReconcilerTests</c>'s helper so the two suites exercise the same stimulus. Three
    /// pass-through frames hold the display at the origin and fill the jerk history, then truth
    /// arrives disagreeing by <paramref name="correction"/>, and from then on the predictor's output
    /// already carries that truth — which is the jump this reconciler exists to hide.
    /// </summary>
    private static List<(long Ticks, Pose Displayed)> DriveStepCorrection(
        Fixture fixture,
        int framesAfterCorrection,
        long frameTicks = FrameTicks,
        float correction = CorrectionMeters,
        float correctionAngleRadians = 0f)
    {
        for (int frame = 0; frame < 3; frame++)
        {
            long ticks = frame * frameTicks;
            AssertSamePose(PoseAt(0f), fixture.Reconciler.Reconcile(PoseAt(0f), ticks));
        }

        long onsetCaptureTicks = 2 * frameTicks;
        fixture.Reconciler.Observe(
            new Stamped<Pose>(onsetCaptureTicks, PoseAt(correction, correctionAngleRadians)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        var trajectory = new List<(long Ticks, Pose Displayed)>(framesAfterCorrection);
        Pose corrected = PoseAt(correction, correctionAngleRadians);
        for (int frame = 3; frame < 3 + framesAfterCorrection; frame++)
        {
            long ticks = frame * frameTicks;
            trajectory.Add((ticks, fixture.Reconciler.Reconcile(corrected, ticks)));
        }

        return trajectory;
    }

    /// <summary>
    /// The residual offset itself, in metres, recovered from the displayed pose as
    /// <c>displayed - predicted</c>. The predictor is parked on the corrected pose from the onset
    /// frame onward, so this is exact up to one float ulp of <see cref="CorrectionMeters"/>
    /// (about 3.7e-9 m) per sample — small enough for the third difference at the frame intervals
    /// the derivative tests use, and each of those tests states its own margin.
    /// </summary>
    private static List<double> OffsetSeries(
        Fixture fixture, int frames, long frameTicks, float correction = CorrectionMeters)
    {
        var trajectory = DriveStepCorrection(fixture, frames, frameTicks, correction);
        var offsets = new List<double>(trajectory.Count);
        foreach ((long _, Pose displayed) in trajectory)
        {
            offsets.Add(displayed.Position.X - (double)correction);
        }

        return offsets;
    }

    /// <summary>Central differences on an evenly spaced series; loses the two endpoints.</summary>
    private static List<double> CentralDifference(IReadOnlyList<double> series, double h)
    {
        var result = new List<double>(Math.Max(0, series.Count - 2));
        for (int i = 1; i < series.Count - 1; i++)
        {
            result.Add((series[i + 1] - series[i - 1]) / (2.0 * h));
        }

        return result;
    }

    private static double MaxMagnitude(IReadOnlyList<double> series)
    {
        double max = 0.0;
        foreach (double value in series)
        {
            max = Math.Max(max, Math.Abs(value));
        }

        return max;
    }

    // ---- 1. Constructor validation ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(16);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentNullException>(
            () => new EasedExponentialReconciler(Config(), null!, clock));
        Assert.Throws<ArgumentNullException>(
            () => new EasedExponentialReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(
                Config(positionToleranceMeters: -1f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(
                Config(orientationToleranceRadians: -1f), metrics, clock));
    }

    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(16);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(
                Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new EasedExponentialReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
    }

    // ---- 2. The law's constants, checked against their defining equations ----

    /// <summary>
    /// The settle constant is a precomputed root, so nothing in the compiler can catch it drifting
    /// from what it is supposed to solve. Recompute the envelope at that <c>x</c> and assert it
    /// lands on 1 % of the initial error — the same guard
    /// <c>SpringReconcilerTests.CriticalSettleConstantMatchesItsDefiningEquation</c> provides for
    /// its own constant, and it pins the fact that both reconcilers use the <b>same</b> 1 %
    /// definition of "converged within the budget".
    /// </summary>
    [Fact]
    public void EasedSettleConstantMatchesItsDefiningEquation()
    {
        double x = EasedSettleConstant;
        double envelope = Math.Exp(1.0 - x - Math.Exp(-x));

        Assert.Equal(0.01, envelope, 7);

        // The equivalent implicit form, x + e^(-x) = 1 + ln(100), which is how the root was solved.
        Assert.Equal(1.0 + Math.Log(100.0), x + Math.Exp(-x), 6);
    }

    /// <summary>
    /// <c>tau</c> comes from the convergence budget and nothing else, and it is 17.8 % shorter than
    /// the plain single-pole law would need for the same 1 % deadline — that shortening is exactly
    /// what pays for leaving the origin at rest.
    /// </summary>
    [Fact]
    public void TimeConstantIsDerivedFromTheConvergenceBudget()
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: 250));

        double expected = 0.25 / EasedSettleConstant;
        Assert.Equal(expected, fixture.Reconciler.TimeConstantSeconds, 6);

        // Halving the budget halves the time constant: the response shape is budget-relative.
        var half = new Fixture(Config(maxTimeToConvergenceTicks: 125));
        Assert.Equal(
            fixture.Reconciler.TimeConstantSeconds / 2f,
            half.Reconciler.TimeConstantSeconds,
            6);

        // And it is shorter than a plain single pole's tau for the same deadline, by ln(100)/x.
        double plainTau = 0.25 / Math.Log(100.0);
        Assert.True(
            fixture.Reconciler.TimeConstantSeconds < plainTau,
            "the eased law must run a shorter time constant to make the same deadline");
        Assert.Equal(0.822, fixture.Reconciler.TimeConstantSeconds / plainTau, 3);
    }

    // ---- 3. Degenerate cases, ordering ----

    /// <summary>
    /// The <c>none</c> + smoothed-reconciler pairing must reduce to exact pass-through: a stream of
    /// zero-magnitude "corrections" would inflate the correction rate with events that never
    /// happened.
    /// </summary>
    [Fact]
    public void Observe_WithinTolerance_CorrectsNothingAndEmitsNothing()
    {
        var fixture = new Fixture();

        fixture.Reconciler.Observe(
            new Stamped<Pose>(10, PoseAt(1e-5f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeDeg));

        Pose predicted = PoseAt(0.25f, 0.5f);
        AssertSamePose(predicted, fixture.Reconciler.Reconcile(predicted, 20));
        Assert.Equal(0, fixture.CountOf(JerkMmS3));
        Assert.Equal(0, fixture.CountOf(TimeToConvergenceMs));
    }

    [Fact]
    public void FreshInstanceIsConverged()
    {
        Assert.True(new Fixture().Reconciler.IsConverged);
    }

    [Fact]
    public void Observe_IgnoresStaleAndDuplicateSamplesWhole()
    {
        var fixture = new Fixture();
        var first = new Stamped<Pose>(100, PoseAt(CorrectionMeters));

        fixture.Reconciler.Observe(first, PoseAt(0f), PredictorDiagnostics.None);
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));

        fixture.Reconciler.Observe(first, PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Observe(
            new Stamped<Pose>(50, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));
    }

    /// <summary>
    /// Requirement 5: this reconciler must behave identically whether the predictor supplies
    /// uncertainty or not, which it reaches by not reading the field at all. Asserted on the whole
    /// trajectory rather than on a flag, because "ignores it" is only credible as an observable.
    /// </summary>
    [Fact]
    public void DegradesGracefullyWhenThePredictorSuppliesNoUncertainty()
    {
        var withNone = new Fixture();
        var withCovariance = new Fixture();

        var sample = new Stamped<Pose>(20, PoseAt(CorrectionMeters));
        var rich = new PredictorDiagnostics(
            horizonTicks: 100,
            lastObservationTicks: 50,
            acceptedObservations: 10,
            rejectedObservations: 1,
            hasUncertainty: true,
            positionSigmaMeters: 0.02f,
            orientationSigmaRadians: 0.03f);
        Assert.True(rich.HasUncertainty, "the test needs a diagnostics value that reports uncertainty");

        for (int frame = 0; frame < 3; frame++)
        {
            withNone.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
            withCovariance.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        withNone.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        withCovariance.Reconciler.Observe(sample, PoseAt(0f), rich);

        for (int frame = 3; frame < 40; frame++)
        {
            Pose a = withNone.Reconciler.Reconcile(PoseAt(CorrectionMeters), frame * FrameTicks);
            Pose b = withCovariance.Reconciler.Reconcile(PoseAt(CorrectionMeters), frame * FrameTicks);
            AssertSamePose(a, b);
        }
    }

    // ---- 4. Bounded convergence (requirement 1) ----

    [Fact]
    public void AConstantCorrectionConvergesWithinTheBudget()
    {
        var fixture = new Fixture();
        int framesToBudget = (int)(BudgetTicks / FrameTicks);
        var trajectory = DriveStepCorrection(fixture, framesToBudget + 4);

        long onsetTicks = 3 * FrameTicks;
        long deadlineTicks = onsetTicks + BudgetTicks;

        Assert.True(fixture.Reconciler.IsConverged, "did not converge at all");

        double convergedMs = Assert.Single(Values(fixture, TimeToConvergenceMs));
        Assert.True(
            convergedMs <= BudgetTicks,
            $"time_to_convergence_ms was {convergedMs}, above the {BudgetTicks} ms budget");

        Pose atDeadline = trajectory.Find(entry => entry.Ticks == deadlineTicks).Displayed;
        Assert.True(
            Math.Abs(atDeadline.Position.X - CorrectionMeters) <= PositionTolerance,
            $"displayed X was {atDeadline.Position.X}, not within tolerance of {CorrectionMeters}");
    }

    /// <summary>
    /// The stated bound itself, independent of the configured tolerance: by the budget the residual
    /// offset is at 1 % of the initial error, which is the definition
    /// <see cref="EasedSettleConstantMatchesItsDefiningEquation"/> pins. Checked on a correction
    /// large enough that 1 % of it still exceeds the tolerance, which is the caveat the type doc
    /// states — the bound is relative, not absolute.
    /// </summary>
    [Fact]
    public void TheStatedBoundIsOnePercentOfTheInitialErrorAtTheBudget()
    {
        const float LargeCorrection = 1.0f;
        var fixture = new Fixture();

        // Decay starts one frame after the onset frame, so the budget elapses at frame 3 + 10.
        var offsets = OffsetSeries(fixture, 20, FrameTicks, LargeCorrection);

        int framesToBudget = (int)(BudgetTicks / FrameTicks);
        double atBudget = Math.Abs(offsets[framesToBudget]) / LargeCorrection;

        Assert.Equal(0.01, atBudget, 4);
    }

    [Fact]
    public void TimeToConvergenceIsEmittedOncePerEpisodeAndIsNonZero()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, (int)(BudgetTicks / FrameTicks) + 10);

        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
        double ms = Assert.Single(Values(fixture, TimeToConvergenceMs));
        Assert.True(ms > 0.0, $"expected a positive convergence time, got {ms}");
    }

    [Fact]
    public void ARateCapStretchesConvergenceRatherThanBreakingIt()
    {
        // A cap far below the uncapped peak speed of roughly 1.23 m/s.
        var capped = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: 0.1f));
        var uncapped = new Fixture();

        const int Frames = 200;
        DriveStepCorrection(capped, Frames);
        DriveStepCorrection(uncapped, Frames);

        Assert.True(capped.Reconciler.IsConverged, "the capped correction never converged");
        double cappedMs = Assert.Single(Values(capped, TimeToConvergenceMs));
        double uncappedMs = Assert.Single(Values(uncapped, TimeToConvergenceMs));

        Assert.True(
            cappedMs > uncappedMs,
            $"expected the rate cap to slow convergence: capped {cappedMs} ms vs {uncappedMs} ms");
    }

    [Fact]
    public void AGapOfSeveralHundredMillisecondsStillConvergesCleanly()
    {
        var fixture = new Fixture();

        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        Pose corrected = PoseAt(CorrectionMeters);
        fixture.Reconciler.Reconcile(corrected, 3 * FrameTicks);

        // 500 ms of nothing, then one frame: five budgets, so the correction is over. The gated
        // decay's exponential underflows here, which is the path the step's invMu guard covers.
        Pose afterGap = fixture.Reconciler.Reconcile(corrected, 3 * FrameTicks + 500);

        Assert.False(float.IsNaN(afterGap.Position.X));
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(CorrectionMeters, afterGap.Position.X, 5);
    }

    // ---- 5. The law's shape: closed form and its derivative coefficients ----

    /// <summary>
    /// The whole claim in one assertion: the offset follows <c>o0 exp(1 - x - e^(-x))</c>, not
    /// <c>o0 (1 + x) e^(-x)</c>. The exact per-step map telescopes to this envelope over unequal
    /// intervals, so agreement is to float accumulation rather than to an integration tolerance —
    /// a 1e-6 m margin on a 0.05 m correction is 2e-5 relative, well above the ~1e-8 per-step float
    /// rounding and well below any plausible formula error.
    /// </summary>
    [Fact]
    public void TheOffsetFollowsItsClosedFormEnvelope()
    {
        var fixture = new Fixture();
        var offsets = OffsetSeries(fixture, 25, FrameTicks);
        double tau = fixture.Reconciler.TimeConstantSeconds;
        double dt = FrameTicks / (double)TicksPerSecond;

        for (int k = 0; k < offsets.Count; k++)
        {
            double x = k * dt / tau;
            double expected = -CorrectionMeters * Math.Exp(1.0 - x - Math.Exp(-x));
            Assert.True(
                Math.Abs(offsets[k] - expected) <= 1e-6,
                $"frame {k}: offset {offsets[k]} vs closed form {expected}");
        }
    }

    /// <summary>
    /// Peak offset speed, the quantity a rate cap governs. <c>0.437971 |o0| / tau</c>, attained at
    /// <c>x = 0.96242</c>. Measured with a 2 ms frame at a 1 s budget so the peak is sampled
    /// densely; the central-difference truncation there is
    /// <c>(h^2/6)|phi'''|/|phi'| ~ 0.04 %</c>, so a 1 % window is a bound, not a fitted value.
    /// </summary>
    [Fact]
    public void PeakOffsetSpeedMatchesItsClosedFormCoefficient()
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: 1000));
        var offsets = OffsetSeries(fixture, 300, frameTicks: 2);
        double tau = fixture.Reconciler.TimeConstantSeconds;
        double h = 2 / (double)TicksPerSecond;

        double measured = MaxMagnitude(CentralDifference(offsets, h));
        double analytic = PeakSpeedCoefficient * CorrectionMeters / tau;

        Assert.Equal(1.0, measured / analytic, 2);
    }

    /// <summary>
    /// Peak offset acceleration, <c>|o0| / tau^2</c>, attained at <c>x = 0</c>. Because the peak is
    /// at the boundary a central second difference cannot reach it — the earliest one is centred at
    /// <c>x = 2h/tau = 0.0224</c>, where the closed form is 0.977 of its peak. The window below
    /// covers that offset plus O(h^2) truncation, and the upper bound is a real property: the
    /// analytic maximum is at <c>t = 0</c>, so nothing in the sampled series may exceed it.
    /// </summary>
    [Fact]
    public void PeakOffsetAccelerationMatchesItsClosedFormCoefficient()
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: 1000));
        var offsets = OffsetSeries(fixture, 300, frameTicks: 2);
        double tau = fixture.Reconciler.TimeConstantSeconds;
        double h = 2 / (double)TicksPerSecond;

        double measured = MaxMagnitude(CentralDifference(CentralDifference(offsets, h), h));
        double analytic = PeakAccelerationCoefficient * CorrectionMeters / (tau * tau);
        double ratio = measured / analytic;

        Assert.True(ratio > 0.94, $"peak acceleration {ratio:F4} of analytic, expected > 0.94");
        Assert.True(ratio < 1.02, $"peak acceleration {ratio:F4} of analytic, expected < 1.02");
    }

    /// <summary>
    /// Peak offset jerk, <c>1.249617 |o0| / tau^3</c> at <c>x = 0.27159</c> — the coefficient that
    /// most distinguishes this law's response from a critically damped one at the same budget.
    /// Measured with a 10 ms frame rather than the 2 ms the speed test uses: a third difference
    /// divides by <c>(2h)^3</c>, so the float noise floor of the recovered offset series
    /// (about one ulp of 0.05 m per sample) grows as <c>h^-3</c> and at 2 ms would be several
    /// percent of the signal. At 10 ms it is under 0.1 %, and the truncation error is under 1 %.
    /// </summary>
    [Fact]
    public void PeakOffsetJerkMatchesItsClosedFormCoefficient()
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: 1000));
        var offsets = OffsetSeries(fixture, 250, frameTicks: 10);
        double tau = fixture.Reconciler.TimeConstantSeconds;
        double h = 10 / (double)TicksPerSecond;

        var jerk = CentralDifference(
            CentralDifference(CentralDifference(offsets, h), h), h);
        double measured = MaxMagnitude(jerk);
        double analytic = PeakJerkCoefficient * CorrectionMeters / (tau * tau * tau);
        double ratio = measured / analytic;

        Assert.True(ratio > 0.95, $"peak jerk {ratio:F4} of analytic, expected > 0.95");
        Assert.True(ratio < 1.05, $"peak jerk {ratio:F4} of analytic, expected < 1.05");
    }

    [Fact]
    public void TheDisplayedTrajectoryNeverOvershootsTheTarget()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 40);

        float previous = float.NegativeInfinity;
        foreach ((long ticks, Pose displayed) in trajectory)
        {
            float x = displayed.Position.X;
            Assert.True(
                x <= CorrectionMeters + PositionTolerance,
                $"overshot at tick {ticks}: displayed {x} exceeds target {CorrectionMeters}");
            Assert.True(
                x >= previous - PositionTolerance,
                $"reversed at tick {ticks}: {x} is below the previous {previous}");
            previous = x;
        }
    }

    // ---- 6. C1 continuity (requirement 2), with no exception claimed ----

    [Fact]
    public void TheDisplayedPoseIsContinuousAcrossCorrectionOnset()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 3);

        Assert.Equal(0f, trajectory[0].Displayed.Position.X, 6);
    }

    /// <summary>
    /// <b>The property this reconciler exists for.</b> A plain single-pole lag leaves onset at
    /// <c>|o0|/tau</c> — a velocity step that does not shrink with the frame interval, which is why
    /// it would need requirement 2 amended. This law leaves at <b>zero</b>, so the first frame's
    /// apparent velocity is only the second-order term <c>|o0| dt / (2 tau^2)</c> and must halve
    /// every time the frame interval halves.
    ///
    /// Asserting a single small number at one frame rate would not distinguish the two; refinement
    /// does. The absolute check at the finest interval is the same statement in the other direction:
    /// <c>dt/(2 tau) = 1.1 %</c> of the plain law's initial rate, so a 2 % ceiling is a bound with
    /// margin, and a C0 law would sit at 100 %.
    /// </summary>
    [Fact]
    public void TheOffsetLeavesOnsetAtZeroVelocity()
    {
        const long SlowBudgetTicks = 1000;

        double coarse = OnsetVelocity(32, SlowBudgetTicks);
        double fine = OnsetVelocity(16, SlowBudgetTicks);
        double finer = OnsetVelocity(8, SlowBudgetTicks);
        double finest = OnsetVelocity(4, SlowBudgetTicks);

        Assert.True(fine < coarse * 0.6, $"halving dt should halve the onset velocity: {coarse} -> {fine}");
        Assert.True(finer < fine * 0.6, $"and again: {fine} -> {finer}");
        Assert.True(finest < finer * 0.6, $"and again: {finer} -> {finest}");

        // The plain single-pole law's initial rate, which is the step being avoided.
        var reference = new Fixture(Config(maxTimeToConvergenceTicks: SlowBudgetTicks));
        double plainInitialRate = CorrectionMeters / reference.Reconciler.TimeConstantSeconds;
        Assert.True(
            finest < 0.02 * plainInitialRate,
            $"onset velocity {finest} is not negligible against the C0 law's {plainInitialRate}");
    }

    private static double OnsetVelocity(long frameTicks, long budgetTicks)
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: budgetTicks));
        var offsets = OffsetSeries(fixture, 3, frameTicks);
        double seconds = frameTicks / (double)TicksPerSecond;
        return Math.Abs(offsets[1] - offsets[0]) / seconds;
    }

    /// <summary>
    /// The other half of C1 across the whole trajectory, matching
    /// <c>SpringReconcilerTests.MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined</c>
    /// exactly so the two are comparable: refine the frame interval and the largest per-frame
    /// velocity step must shrink with it. A C1 trajectory's step is O(dt); a snap's is O(1/dt) and
    /// grows instead. A slow budget is used for the same reason `spring`'s test uses one — the O(dt)
    /// behaviour is asymptotic in <c>dt/tau</c> and the tick resolution here is 1 ms.
    /// </summary>
    [Fact]
    public void MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined()
    {
        const long SlowBudgetTicks = 1000;

        double coarse = MaxVelocityStep(frameTicks: 8, budgetTicks: SlowBudgetTicks);
        double fine = MaxVelocityStep(frameTicks: 4, budgetTicks: SlowBudgetTicks);
        double finer = MaxVelocityStep(frameTicks: 2, budgetTicks: SlowBudgetTicks);

        Assert.True(
            fine < coarse * 0.75,
            $"halving dt should shrink the velocity step: {coarse} -> {fine}");
        Assert.True(
            finer < fine * 0.75,
            $"halving dt again should shrink it further: {fine} -> {finer}");
    }

    private static double MaxVelocityStep(long frameTicks, long budgetTicks)
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: budgetTicks));
        int frames = (int)(4 * budgetTicks / frameTicks);
        var trajectory = DriveStepCorrection(fixture, frames, frameTicks);

        double seconds = frameTicks / (double)TicksPerSecond;
        double maxStep = 0.0;
        double previousVelocity = 0.0;

        for (int i = 1; i < trajectory.Count; i++)
        {
            double velocity =
                (trajectory[i].Displayed.Position.X - trajectory[i - 1].Displayed.Position.X) / seconds;
            maxStep = Math.Max(maxStep, Math.Abs(velocity - previousVelocity));
            previousVelocity = velocity;
        }

        return maxStep;
    }

    /// <summary>
    /// <b>C1 at a retarget, which is the case a lossy link produces constantly.</b> A second
    /// correction arriving mid-decay re-seeds the offset's <i>position</i> (that is what cancels the
    /// predictor's jump), so the offset's velocity is the thing that must not step. It is preserved
    /// exactly by construction — the momentum state is chosen from the pre-seed velocity — so the
    /// one-sided velocity difference across the retarget is only the O(dt) sampling term and must
    /// halve as the frame interval halves.
    ///
    /// A law that restarted from rest on every packet would put <c>|v_before|</c> into that
    /// difference and it would <b>not</b> shrink; the absolute check below is that comparison, and
    /// the O(dt) truncation at the coarsest interval is about 1.4 % of <c>|v_before|</c>, so a 5 %
    /// ceiling is a bound with margin against a counterfactual that would read 100 %.
    /// </summary>
    [Fact]
    public void TheOffsetVelocityDoesNotStepAcrossARetarget()
    {
        const long SlowBudgetTicks = 1000;

        (double coarse, double coarseBefore) = RetargetVelocityStep(16, SlowBudgetTicks);
        (double fine, double _) = RetargetVelocityStep(8, SlowBudgetTicks);
        (double finer, double _) = RetargetVelocityStep(4, SlowBudgetTicks);

        Assert.True(
            Math.Abs(coarseBefore) > 0.05,
            $"the retarget must land while the offset is genuinely moving; it was {coarseBefore} m/s");

        Assert.True(fine < coarse * 0.75, $"halving dt should shrink the step: {coarse} -> {fine}");
        Assert.True(finer < fine * 0.75, $"halving dt again should shrink it: {fine} -> {finer}");

        Assert.True(
            coarse < 0.05 * Math.Abs(coarseBefore),
            $"velocity step {coarse} is not small against the in-flight velocity {coarseBefore}; " +
            "a law that restarted from rest would put the whole of it here");
    }

    /// <summary>
    /// Drives a correction, lets it decay a quarter of a budget, retargets, and returns the
    /// one-sided offset-velocity difference across the retarget together with the in-flight velocity
    /// it should have matched. Offsets are taken relative to whichever pose the predictor is parked
    /// on at the time, so the predictor's own jump is removed and what remains is the reconciler's
    /// contribution.
    /// </summary>
    private static (double Step, double VelocityBefore) RetargetVelocityStep(
        long frameTicks, long budgetTicks)
    {
        const float First = 0.05f;
        const float Second = 0.08f;

        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: budgetTicks));

        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * frameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * frameTicks, PoseAt(First)), PoseAt(0f), PredictorDiagnostics.None);

        int framesBeforeRetarget = (int)(budgetTicks / 4 / frameTicks);
        double penultimate = 0.0;
        double last = 0.0;
        int retargetFrame = 3 + framesBeforeRetarget;

        for (int frame = 3; frame < retargetFrame; frame++)
        {
            Pose displayed = fixture.Reconciler.Reconcile(PoseAt(First), frame * frameTicks);
            penultimate = last;
            last = displayed.Position.X - (double)First;
        }

        double seconds = frameTicks / (double)TicksPerSecond;
        double velocityBefore = (last - penultimate) / seconds;

        fixture.Reconciler.Observe(
            new Stamped<Pose>((retargetFrame - 1) * frameTicks, PoseAt(Second)),
            PoseAt(First),
            PredictorDiagnostics.None);

        double seeded = fixture.Reconciler
            .Reconcile(PoseAt(Second), retargetFrame * frameTicks).Position.X - (double)Second;
        double next = fixture.Reconciler
            .Reconcile(PoseAt(Second), (retargetFrame + 1) * frameTicks).Position.X - (double)Second;

        double velocityAfter = (next - seeded) / seconds;
        return (Math.Abs(velocityAfter - velocityBefore), velocityBefore);
    }

    /// <summary>
    /// <b>The display is held for exactly one frame at a correction, and that is deliberate.</b>
    /// Inherited unchanged from every other offset-carrying reconciler on this axis and pinned here
    /// for the same reason <c>SpringReconcilerTests</c> pins it: it is a genuine O(1) velocity
    /// dropout that was measured, found to be the better of the two available behaviours, and kept.
    /// The obvious fix (<c>offset -= error</c>) was swept across all three smoothed reconcilers and
    /// regressed jerk p99 by 81-865 % on impaired profiles while collapsing the spread between
    /// reconcilers to 1.01x. See <c>Reconciliation/CLAUDE.md</c>'s "Tried and rejected".
    ///
    /// It is pinned in <i>this</i> suite specifically so that a one-sided change here could not
    /// silently skew a head-to-head: "C1-preserving" for this reconciler means relative to the
    /// shared baseline, and this test is what keeps the baseline shared.
    /// </summary>
    [Fact]
    public void TheDisplayIsHeldForOneFrameAtACorrection_ADeliberateTradeoff()
    {
        var fixture = new Fixture();
        const float PerFrame = 0.01f;

        for (int frame = 0; frame < 4; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(frame * PerFrame), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(3 * FrameTicks, PoseAt(3 * PerFrame + CorrectionMeters)),
            PoseAt(3 * PerFrame),
            PredictorDiagnostics.None);

        Pose atSeedFrame = fixture.Reconciler.Reconcile(
            PoseAt(4 * PerFrame + CorrectionMeters), 4 * FrameTicks);

        Assert.Equal(3 * PerFrame, atSeedFrame.Position.X, 5);
    }

    /// <summary>
    /// The same hold across a retarget. Kept for the reason the onset test above documents, and
    /// matching <c>TheOneFrameHoldAtARetargetIsSharedWithSpring</c> in
    /// <c>TimeBudgetedBlendReconcilerTests</c>.
    /// </summary>
    [Fact]
    public void TheDisplayIsHeldForOneFrameAtARetarget_ADeliberateTradeoff()
    {
        var fixture = new Fixture();
        const float PerFrame = 0.01f;

        for (int frame = 0; frame < 4; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(frame * PerFrame), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(3 * FrameTicks, PoseAt(3 * PerFrame + CorrectionMeters)),
            PoseAt(3 * PerFrame),
            PredictorDiagnostics.None);

        Pose before = PoseAt(0f);
        for (int frame = 4; frame < 6; frame++)
        {
            before = fixture.Reconciler.Reconcile(
                PoseAt(frame * PerFrame + CorrectionMeters), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(5 * FrameTicks, PoseAt(5 * PerFrame + 2f * CorrectionMeters)),
            PoseAt(5 * PerFrame + CorrectionMeters),
            PredictorDiagnostics.None);

        Pose atRetarget = fixture.Reconciler.Reconcile(
            PoseAt(6 * PerFrame + 2f * CorrectionMeters), 6 * FrameTicks);

        Assert.Equal(before.Position.X, atRetarget.Position.X, 5);
    }

    // ---- 7. Correction-cost metrics (requirement 3) ----

    /// <summary>
    /// The correction-magnitude definition must be byte-for-byte the one <c>snap</c> uses. It is a
    /// control across this axis — measured from the predictor's disagreement before any reconciler
    /// acts — so a difference here would mean the sweep was comparing metric implementations.
    /// </summary>
    [Fact]
    public void CorrectionMagnitudeMatchesSnapExactly()
    {
        var eased = new Fixture();
        var snapMetrics = new InMemoryMetricTracker(256);
        var snap = new SnapReconciler(Config(), snapMetrics, new ManualClock(TicksPerSecond));

        var sample = new Stamped<Pose>(100, PoseAt(CorrectionMeters, 0.25f));
        eased.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        snap.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Assert.Equal(
            Values(snapMetrics, CorrectionMagnitudeMm), Values(eased.Metrics, CorrectionMagnitudeMm));
        Assert.Equal(
            Values(snapMetrics, CorrectionMagnitudeDeg), Values(eased.Metrics, CorrectionMagnitudeDeg));
    }

    /// <summary>
    /// <c>jerk_mm_s3</c> is emitted on every advancing frame once the shared estimator's history is
    /// full, not once per correction. The cadence is what makes the percentiles comparable across
    /// the axis: a per-correction emission would compare one sample against a hundred.
    /// </summary>
    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull()
    {
        var fixture = new Fixture();
        const int FramesAfter = 30;
        DriveStepCorrection(fixture, FramesAfter);

        // Three pass-through frames fill the history; jerk is available from the fourth frame on.
        Assert.Equal(FramesAfter, fixture.CountOf(JerkMmS3));
    }

    // ---- 8. Determinism, idempotence, reset (requirement 4) ----

    [Fact]
    public void TwoInstancesGivenTheSameInputProduceIdenticalOutput()
    {
        var a = new Fixture();
        var b = new Fixture();

        var first = DriveStepCorrection(a, 40);
        var second = DriveStepCorrection(b, 40);

        for (int i = 0; i < first.Count; i++)
        {
            AssertSamePose(first[i].Displayed, second[i].Displayed);
        }
    }

    [Fact]
    public void Reconcile_AtOrBeforeTheLastTick_IsANoOp()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, 5);

        long ticks = 7 * FrameTicks;
        Pose corrected = PoseAt(CorrectionMeters);
        Pose once = fixture.Reconciler.Reconcile(corrected, ticks);
        int jerkAfterOnce = fixture.CountOf(JerkMmS3);

        Pose again = fixture.Reconciler.Reconcile(corrected, ticks);
        Pose backwards = fixture.Reconciler.Reconcile(corrected, ticks - 5);

        AssertSamePose(once, again);
        AssertSamePose(once, backwards);
        Assert.Equal(jerkAfterOnce, fixture.CountOf(JerkMmS3));
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var reused = new Fixture();
        DriveStepCorrection(reused, 4);
        Assert.False(reused.Reconciler.IsConverged);

        reused.Reconciler.Reset();
        Assert.True(reused.Reconciler.IsConverged);

        var fresh = new Fixture();
        var afterReset = DriveStepCorrection(reused, 20);
        var fromNew = DriveStepCorrection(fresh, 20);

        for (int i = 0; i < fromNew.Count; i++)
        {
            AssertSamePose(fromNew[i].Displayed, afterReset[i].Displayed);
        }
    }

    [Fact]
    public void ASecondCorrectionMidDecayExtendsTheSameEpisode()
    {
        var fixture = new Fixture();

        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(20, PoseAt(CorrectionMeters)), PoseAt(0f), PredictorDiagnostics.None);
        fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 30);
        fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 40);

        fixture.Reconciler.Observe(
            new Stamped<Pose>(45, PoseAt(2f * CorrectionMeters)),
            PoseAt(CorrectionMeters),
            PredictorDiagnostics.None);

        for (int frame = 5; frame < 60; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(2f * CorrectionMeters), frame * FrameTicks);
        }

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(2, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
    }

    [Fact]
    public void AnOrientationOnlyCorrectionConvergesOnTheGeodesic()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(
            fixture, 40, correction: 0f, correctionAngleRadians: 0.4f);

        Assert.True(fixture.Reconciler.IsConverged);
        Pose target = PoseAt(0f, 0.4f);
        Assert.True(
            PoseMath.OrientationErrorRadians(target, trajectory[^1].Displayed) <= 1e-3f,
            "orientation did not converge onto the corrected prediction");
    }

    // ---- 9. Allocation (requirement 4) ----

    [Fact]
    public void Reconcile_DuringACorrection_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long ticks = 0;
        long captureTicks = 0;
        Pose corrected = PoseAt(CorrectionMeters);

        AllocationAssert.Zero(() =>
        {
            ticks += FrameTicks;
            captureTicks += FrameTicks;
            fixture.Reconciler.Observe(
                new Stamped<Pose>(captureTicks, PoseAt(CorrectionMeters, 0.1f)),
                PoseAt(0f),
                PredictorDiagnostics.None);
            fixture.Reconciler.Reconcile(corrected, ticks);
        });
    }

    [Fact]
    public void Reconcile_WhileConverged_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long ticks = 0;
        Pose predicted = PoseAt(0.1f, 0.2f);

        AllocationAssert.Zero(() =>
        {
            ticks += FrameTicks;
            fixture.Reconciler.Reconcile(predicted, ticks);
        });
    }

    [Fact]
    public void Observe_WhenRejectedAsStale_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Observe(
            new Stamped<Pose>(long.MaxValue / 2, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);
        var stale = new Stamped<Pose>(1, PoseAt(1f));

        AllocationAssert.Zero(
            () => fixture.Reconciler.Observe(stale, PoseAt(0f), PredictorDiagnostics.None));
    }

    private static List<double> Values(Fixture fixture, string name) => Values(fixture.Metrics, name);

    private static List<double> Values(InMemoryMetricTracker metrics, string name)
    {
        var values = new List<double>();
        for (int i = 0; i < metrics.Count; i++)
        {
            if (metrics[i].Name == name)
            {
                values.Add(metrics[i].Value);
            }
        }

        return values;
    }
}
