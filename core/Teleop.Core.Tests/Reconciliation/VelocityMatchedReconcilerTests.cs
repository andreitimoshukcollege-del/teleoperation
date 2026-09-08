using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Reconciliation;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Reconciliation;

public class VelocityMatchedReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The stated worst-case convergence window: the budget divided by the time-warp floor. This is
    /// the bound <c>VelocityMatchedReconciler</c>'s type doc claims and the number the
    /// bounded-convergence test asserts against. Derived from the implementation's own public
    /// constant rather than hardcoded, so the two cannot drift apart.
    /// </summary>
    private const long StatedWindowTicks =
        (long)(BudgetTicks / VelocityMatchedReconciler.MinimumTimeWarp);

    /// <summary>
    /// The step correction the main tests use, 5 cm. Matches <c>SpringReconcilerTests</c> exactly so
    /// the two suites exercise the same operating point.
    /// </summary>
    private const float CorrectionMeters = 0.05f;

    /// <summary>
    /// A small correction, 5 mm -- the scale of a real prediction error at a 100 ms horizon rather
    /// than the scale of a trial-opening transient. The apparent-speed tests use this because the
    /// masking rule compares the correction's own demanded speed against the operator's speed, and
    /// at the 5 cm scale the demanded speed is so far above any plausible hand speed that the warp
    /// saturates at its floor and the adaptive term cannot be observed at all. That saturation is
    /// itself a finding (see docs/research-log/2026-09-08-velocity-match.md), not a reason to hide
    /// it -- but a test of the adaptive term has to be run where the term is live.
    /// </summary>
    private const float SmallCorrectionMeters = 0.005f;

    private const float PositionTolerance = 1e-3f;

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
        public readonly VelocityMatchedReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new VelocityMatchedReconciler(
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

        public List<double> ValuesOf(string name)
        {
            var values = new List<double>();
            for (int i = 0; i < Metrics.Count; i++)
            {
                if (Metrics[i].Name == name)
                {
                    values.Add(Metrics[i].Value);
                }
            }

            return values;
        }
    }

    /// <summary>
    /// The reference position-first reconciler this candidate's central claim is stated against.
    /// Same config, same clock, same driver -- only the reconciler differs, which is
    /// <c>Reconciliation/CLAUDE.md</c>'s experiment-design rule applied at unit-test scale.
    /// </summary>
    private sealed class SpringFixture
    {
        public readonly InMemoryMetricTracker Metrics = new InMemoryMetricTracker(8192);
        public readonly SpringReconciler Reconciler;

        public SpringFixture(ReconcilerConfig? config = null)
        {
            Reconciler = new SpringReconciler(
                config ?? Config(), Metrics, new ManualClock(TicksPerSecond));
        }

        public List<double> ValuesOf(string name)
        {
            var values = new List<double>();
            for (int i = 0; i < Metrics.Count; i++)
            {
                if (Metrics[i].Name == name)
                {
                    values.Add(Metrics[i].Value);
                }
            }

            return values;
        }
    }

    /// <summary>
    /// The predictor's output at <paramref name="ticks"/>: translating along +Y at
    /// <paramref name="operatorSpeed"/> metres/second throughout, displaced along X by
    /// <paramref name="xOffset"/> and rotated about Z by <paramref name="angleRadians"/>.
    ///
    /// The operator's motion is on Y and the correction is on X <b>on purpose</b>: they are
    /// orthogonal, so a test can read the offset off as <c>displayed - predicted</c> without the
    /// apparent motion contaminating it.
    /// </summary>
    private static Pose PredictedAt(
        long ticks, float xOffset, float angleRadians, float operatorSpeed)
    {
        float seconds = ticks / (float)TicksPerSecond;
        return new Pose(
            new Vector3(xOffset, operatorSpeed * seconds, 0f),
            Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angleRadians));
    }

    private readonly struct Frame
    {
        public readonly long Ticks;
        public readonly Pose Displayed;
        public readonly Pose Predicted;

        public Frame(long ticks, Pose displayed, Pose predicted)
        {
            Ticks = ticks;
            Displayed = displayed;
            Predicted = predicted;
        }
    }

    /// <summary>
    /// One step correction, driven the way the pipeline drives it, against <b>any</b> reconciler so
    /// that <c>velocity-match</c> and <c>spring</c> can be put through identical input.
    ///
    /// <paramref name="framesBefore"/> pass-through frames fill the jerk history and warm the
    /// apparent-speed estimate, then truth arrives disagreeing by <paramref name="correction"/> along
    /// X, and from then on the predictor's output already carries that truth -- which is the jump a
    /// reconciler exists to hide. Returns one entry per frame from the correction onward.
    /// </summary>
    private static List<Frame> DriveStepCorrection(
        IReconciler<Pose> reconciler,
        int framesAfter,
        int framesBefore = 3,
        long frameTicks = FrameTicks,
        float correction = CorrectionMeters,
        float correctionAngleRadians = 0f,
        float operatorSpeed = 0f)
    {
        for (int frame = 0; frame < framesBefore; frame++)
        {
            long ticks = frame * frameTicks;
            reconciler.Reconcile(PredictedAt(ticks, 0f, 0f, operatorSpeed), ticks);
        }

        long captureTicks = (framesBefore - 1) * frameTicks;
        reconciler.Observe(
            new Stamped<Pose>(
                captureTicks,
                PredictedAt(captureTicks, correction, correctionAngleRadians, operatorSpeed)),
            PredictedAt(captureTicks, 0f, 0f, operatorSpeed),
            PredictorDiagnostics.None);

        var trajectory = new List<Frame>(framesAfter);
        for (int frame = framesBefore; frame < framesBefore + framesAfter; frame++)
        {
            long ticks = frame * frameTicks;
            Pose predicted = PredictedAt(ticks, correction, correctionAngleRadians, operatorSpeed);
            trajectory.Add(new Frame(ticks, reconciler.Reconcile(predicted, ticks), predicted));
        }

        return trajectory;
    }

    /// <summary>
    /// The quantity this candidate is about: the speed the reconciler injects into the displayed
    /// trajectory, i.e. the rate of change of <c>displayed - predicted</c>. Everything the operator
    /// sees that is not their own motion is in here, so this -- not the displayed speed -- is the
    /// perturbation of apparent motion.
    /// </summary>
    private static double PeakInjectedSpeed(List<Frame> trajectory, long frameTicks = FrameTicks)
    {
        double seconds = frameTicks / (double)TicksPerSecond;
        double peak = 0.0;

        for (int i = 1; i < trajectory.Count; i++)
        {
            Vector3 previous = trajectory[i - 1].Displayed.Position - trajectory[i - 1].Predicted.Position;
            Vector3 current = trajectory[i].Displayed.Position - trajectory[i].Predicted.Position;
            peak = Math.Max(peak, (current - previous).Length() / seconds);
        }

        return peak;
    }

    private static Pose PoseAt(float x, float angleRadians = 0f) =>
        new Pose(new Vector3(x, 0f, 0f), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angleRadians));

    private static void AssertSamePose(Pose expected, Pose actual)
    {
        Assert.Equal(expected.Position, actual.Position);
        Assert.Equal(expected.Rotation, actual.Rotation);
    }

    // ---- 1. Constructor validation ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);

        Assert.Throws<ArgumentNullException>(
            () => new VelocityMatchedReconciler(Config(), null!, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentNullException>(
            () => new VelocityMatchedReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(positionToleranceMeters: -1f), metrics, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(orientationToleranceRadians: -1f), metrics, new ManualClock(TicksPerSecond)));
    }

    /// <summary>
    /// The budget sets both the natural frequency and the apparent-speed smoothing timescale, and a
    /// zero rate cap could never converge. Accepting either would produce a reconciler that silently
    /// never finishes -- the "lags indefinitely" failure Reconciliation/CLAUDE.md calls a bug rather
    /// than a tradeoff.
    /// </summary>
    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(maxTimeToConvergenceTicks: -1), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new VelocityMatchedReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
    }

    /// <summary>
    /// Guards the precomputed settle constant against its defining equation, so it and the 1% settle
    /// fraction it encodes cannot drift apart silently -- and, because this candidate deliberately
    /// reuses <c>spring</c>'s value, so that "converged" keeps meaning the same thing for both.
    /// </summary>
    [Fact]
    public void CriticalSettleConstantMatchesItsDefiningEquation()
    {
        var fixture = new Fixture();
        double budgetSeconds = BudgetTicks / (double)TicksPerSecond;
        double x = fixture.Reconciler.NaturalFrequencyRadiansPerSecond * budgetSeconds;

        Assert.Equal(0.01, (1.0 + x) * Math.Exp(-x), 5);
    }

    /// <summary>
    /// At <c>w = 1</c> this reconciler's dynamics must be <c>spring</c>'s dynamics, which requires
    /// the same natural frequency from the same budget. If these ever diverged, the head-to-head
    /// would be comparing two response shapes rather than the masking rule.
    /// </summary>
    [Fact]
    public void NaturalFrequencyMatchesSpringForTheSameBudget()
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        Assert.Equal(
            spring.Reconciler.NaturalFrequencyRadiansPerSecond,
            velocityMatch.Reconciler.NaturalFrequencyRadiansPerSecond);
    }

    // ---- 2. The degenerate pass-through case ----

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

    // ---- 3. Ordering: stale and duplicate samples ----

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

    // ---- 4. Bounded convergence: the stated bound ----

    /// <summary>
    /// The stated bound, and the whole reason this candidate is a tradeoff rather than a lag: a
    /// stationary operator is the worst case for the masking rule (nothing to hide the correction
    /// behind), so the time warp sits at its floor and convergence takes up to
    /// <c>MaxTimeToConvergenceTicks / MinimumTimeWarp</c>. The test asserts <b>both</b> halves:
    /// it does converge inside the stated window, and it does <i>not</i> converge inside the plain
    /// budget. The second half is what stops the bound being vacuous -- a bound of "4x the budget"
    /// that was actually met in 1x would mean the mechanism is not engaged at all.
    /// </summary>
    [Fact]
    public void AStationaryOperatorConvergesWithinTheStatedWindowAndNotWithinThePlainBudget()
    {
        var fixture = new Fixture();
        int framesToWindow = (int)(StatedWindowTicks / FrameTicks);
        var trajectory = DriveStepCorrection(fixture.Reconciler, framesToWindow + 8);

        long onsetTicks = 3 * FrameTicks;

        Assert.True(fixture.Reconciler.IsConverged, "did not converge at all");

        double convergedMs = Assert.Single(fixture.ValuesOf(TimeToConvergenceMs));
        Assert.True(
            convergedMs <= StatedWindowTicks,
            $"time_to_convergence_ms was {convergedMs}, above the stated {StatedWindowTicks} ms window");
        Assert.True(
            convergedMs > BudgetTicks,
            $"time_to_convergence_ms was {convergedMs} ms, inside the plain {BudgetTicks} ms budget: " +
            "the time warp cannot have been engaged, so the stated bound is vacuous");

        // And the displayed pose is genuinely on the corrected prediction by the stated deadline.
        long deadlineTicks = onsetTicks + StatedWindowTicks;
        Pose atDeadline = trajectory.Find(entry => entry.Ticks == deadlineTicks).Displayed;
        Assert.True(
            Math.Abs(atDeadline.Position.X - CorrectionMeters) <= PositionTolerance,
            $"displayed X was {atDeadline.Position.X}, not within tolerance of {CorrectionMeters}");
    }

    /// <summary>
    /// The residual is a <i>bounded time</i>, not a permanent offset. Driven well past the stated
    /// window the offset keeps decaying toward exactly zero rather than parking somewhere inside
    /// tolerance -- which matters because a reconciler that parks would post a flattering
    /// <c>jerk_mm_s3</c> for reasons that are not merit.
    /// </summary>
    [Fact]
    public void TheResidualOffsetDecaysToZeroRatherThanParkingInsideTolerance()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture.Reconciler, 400);

        Frame last = trajectory[^1];
        float residual = (last.Displayed.Position - last.Predicted.Position).Length();

        Assert.True(
            residual < 1e-7f,
            $"residual offset was {residual} m long after the stated window; it should have decayed " +
            "to zero, not parked inside tolerance");
    }

    [Fact]
    public void TimeToConvergenceIsEmittedOncePerEpisodeAndIsNonZero()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture.Reconciler, (int)(StatedWindowTicks / FrameTicks) + 20);

        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
        double ms = Assert.Single(fixture.ValuesOf(TimeToConvergenceMs));
        Assert.True(ms > 0.0, $"expected a positive convergence time, got {ms}");
    }

    [Fact]
    public void ARateCapStretchesConvergenceRatherThanBreakingIt()
    {
        // A cap below the uncapped peak warped-time offset speed of roughly 1.2 m/s.
        var capped = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: 0.5f));
        var uncapped = new Fixture();

        const int frames = 1200;
        DriveStepCorrection(capped.Reconciler, frames);
        DriveStepCorrection(uncapped.Reconciler, frames);

        Assert.True(capped.Reconciler.IsConverged, "the capped correction never converged");
        double cappedMs = Assert.Single(capped.ValuesOf(TimeToConvergenceMs));
        double uncappedMs = Assert.Single(uncapped.ValuesOf(TimeToConvergenceMs));

        Assert.True(
            cappedMs > uncappedMs,
            $"expected the rate cap to slow convergence: capped {cappedMs} ms vs {uncappedMs} ms");
    }

    // ---- 5. No overshoot, and C1 continuity ----

    [Fact]
    public void TheDisplayedTrajectoryNeverOvershootsTheTarget()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture.Reconciler, 120);

        float previous = float.NegativeInfinity;
        foreach (Frame frame in trajectory)
        {
            float x = frame.Displayed.Position.X;
            Assert.True(
                x <= CorrectionMeters + PositionTolerance,
                $"overshot at tick {frame.Ticks}: displayed {x} exceeds target {CorrectionMeters}");
            Assert.True(
                x >= previous - PositionTolerance,
                $"reversed at tick {frame.Ticks}: {x} is below the previous {previous}");
            previous = x;
        }
    }

    /// <summary>
    /// The crisp half of C1: the correction begins from the pose that was already on screen, so the
    /// displayed position does not move on the onset frame and there is no velocity step to see.
    /// </summary>
    [Fact]
    public void TheDisplayedPoseIsContinuousAcrossCorrectionOnset()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture.Reconciler, 3);

        Assert.Equal(0f, trajectory[0].Displayed.Position.X, 6);
    }

    /// <summary>
    /// The other half of C1, and the only form of it that distinguishes a continuous velocity from a
    /// discontinuous one on a sampled signal: refine the frame interval and the largest per-frame
    /// velocity step must shrink with it. A C1 trajectory's velocity step is O(dt); a snap's is
    /// O(1/dt) and grows instead. A single absolute bound at one frame rate would pass for both.
    ///
    /// Structure and rationale follow
    /// <c>SpringReconcilerTests.MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined</c>,
    /// including the deliberately slow 1 s budget: the O(dt) behaviour is asymptotic in
    /// <c>omega*dt</c>, and at the default 100 ms budget the ratios say more about sampling the
    /// dynamics too sparsely than about continuity.
    ///
    /// For this reconciler the test carries extra weight, because there is a third way it could have
    /// failed that <c>spring</c> does not have: the time warp multiplies the visible velocity, so a
    /// warp that jumped between frames would be a velocity discontinuity. It cannot jump only because
    /// the apparent-speed estimate is exponentially smoothed with an O(dt) per-frame change. This
    /// test is what holds that claim up.
    /// </summary>
    [Fact]
    public void MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined()
    {
        const long slowBudgetTicks = 1000;

        double coarse = MaxVelocityStep(frameTicks: 8, budgetTicks: slowBudgetTicks);
        double fine = MaxVelocityStep(frameTicks: 4, budgetTicks: slowBudgetTicks);
        double finer = MaxVelocityStep(frameTicks: 2, budgetTicks: slowBudgetTicks);

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

        // Four stated windows' worth of frames, so the whole decay is inside the sample.
        int frames = (int)(4 * budgetTicks / VelocityMatchedReconciler.MinimumTimeWarp / frameTicks);
        var trajectory = DriveStepCorrection(fixture.Reconciler, frames, frameTicks: frameTicks);

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

    // ---- 6. The central claim: apparent motion is better preserved ----

    /// <summary>
    /// <b>This is the candidate's central claim, as a test rather than a comment.</b> Under an
    /// identical step correction, identical config and identical driving, <c>velocity-match</c> must
    /// inject substantially less velocity into the displayed trajectory than the position-first
    /// reference (<c>spring</c>) does -- and must pay for it in convergence time.
    ///
    /// The operator is stationary here, so every bit of displayed motion is injected by the
    /// reconciler and "preserving apparent motion" reduces to "injecting less speed". The expected
    /// factor is <c>1 / MinimumTimeWarp</c> = 4, because the warp sits at its floor: peak speed for a
    /// critically damped response scales linearly with the natural frequency, and warping time by
    /// <c>w</c> scales the effective frequency by <c>w</c>.
    ///
    /// The second assertion is the honest half. If convergence did <i>not</i> get slower, the
    /// injected speed could not have fallen -- 5 cm has to be covered either way -- and a result
    /// showing both would be a broken measurement, not a free lunch.
    /// </summary>
    [Fact]
    public void InjectsMuchLessVelocityThanSpringUnderAStepCorrection_AndPaysForItInTime()
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        const int frames = 600;
        var velocityMatchTrajectory = DriveStepCorrection(velocityMatch.Reconciler, frames);
        var springTrajectory = DriveStepCorrection(spring.Reconciler, frames);

        double velocityMatchPeak = PeakInjectedSpeed(velocityMatchTrajectory);
        double springPeak = PeakInjectedSpeed(springTrajectory);

        Assert.True(
            velocityMatchPeak < springPeak * 0.35,
            $"peak injected speed was {velocityMatchPeak:F4} m/s against spring's {springPeak:F4} m/s; " +
            $"expected roughly {VelocityMatchedReconciler.MinimumTimeWarp:F2}x");

        double velocityMatchMs = Assert.Single(velocityMatch.ValuesOf(TimeToConvergenceMs));
        double springMs = Assert.Single(spring.ValuesOf(TimeToConvergenceMs));

        Assert.True(
            velocityMatchMs > springMs,
            $"convergence must be the price paid: velocity-match {velocityMatchMs} ms vs " +
            $"spring {springMs} ms. Lower injected speed with no time cost would be a broken " +
            "measurement, not a win");
    }

    /// <summary>
    /// The masking rule is <b>live</b>, not inert: the faster the operator is already moving, the
    /// less this reconciler slows the correction down.
    ///
    /// Measured as a <b>ratio against <c>spring</c> under identical driving</b>, not as an absolute
    /// convergence time, and the reason is a confound worth recording rather than hiding. Every
    /// offset-carrying reconciler seeds from <c>lastDisplayed - predicted</c>, so one frame of
    /// unhidden operator motion lands in the initial offset: at 2 m/s and a 10 ms frame that is 20 mm,
    /// four times the 5 mm correction under test. A faster operator therefore hands <i>both</i>
    /// reconcilers a larger correction to absorb, and the absolute convergence times are not
    /// comparable across apparent speeds. (An earlier version of this test asserted on absolute times
    /// and failed for exactly that reason -- 70 ms at 0.3 m/s against 80 ms at 2 m/s -- which is the
    /// driver's artifact, not the reconciler's behaviour.) Dividing by <c>spring</c>'s time on the
    /// same input cancels it, because <c>spring</c> is affected identically and is not affected by
    /// apparent speed at all.
    ///
    /// Run at the 5 mm correction scale: at 5 cm the correction's own demanded speed exceeds 1 m/s
    /// and the warp starts at its floor for any plausible hand speed, which is a real characteristic
    /// of the design (recorded in the log) but would make this test assert nothing.
    /// </summary>
    [Fact]
    public void ApparentMotionSpeedsTheCorrectionUpRelativeToSpring()
    {
        double still = ConvergenceRatioToSpringAtOperatorSpeed(0f);
        double slow = ConvergenceRatioToSpringAtOperatorSpeed(0.3f);
        double fast = ConvergenceRatioToSpringAtOperatorSpeed(2f);

        Assert.True(
            still > slow,
            $"a moving operator should mask the correction: still {still:F2}x spring vs slow {slow:F2}x");
        Assert.True(
            slow > fast,
            $"a faster operator should mask more of it: slow {slow:F2}x spring vs fast {fast:F2}x");
        Assert.True(
            fast <= 1.01,
            $"at high apparent speed the correction should cost no more time than spring's, got {fast:F2}x");
    }

    private static double ConvergenceRatioToSpringAtOperatorSpeed(float operatorSpeed)
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        // 60 warm-up frames is six smoothing time constants at the 100 ms budget, so the
        // apparent-speed estimate is settled before the correction arrives.
        DriveStepCorrection(
            velocityMatch.Reconciler,
            framesAfter: 600,
            framesBefore: 60,
            correction: SmallCorrectionMeters,
            operatorSpeed: operatorSpeed);
        DriveStepCorrection(
            spring.Reconciler,
            framesAfter: 600,
            framesBefore: 60,
            correction: SmallCorrectionMeters,
            operatorSpeed: operatorSpeed);

        Assert.True(velocityMatch.Reconciler.IsConverged);
        Assert.True(spring.Reconciler.IsConverged);

        return Assert.Single(velocityMatch.ValuesOf(TimeToConvergenceMs))
            / Assert.Single(spring.ValuesOf(TimeToConvergenceMs));
    }

    /// <summary>
    /// The limiting case, and the property that makes the head-to-head one-sided: when the apparent
    /// speed is high enough to mask a full-rate correction, the warp saturates at 1 and this
    /// reconciler's displayed trajectory is <c>spring</c>'s. It is never <i>more</i> aggressive than
    /// <c>spring</c> at the same budget -- it either matches it or is gentler.
    ///
    /// Note what the numbers here say: reaching <c>w = 1</c> for a 5 mm correction on a 100 ms budget
    /// needs about 2 m/s of apparent speed. That is a fast, deliberate arm movement, which means the
    /// saturated-at-the-floor regime is the common one in practice. Recorded in the log as the main
    /// caveat a judge should carry into a head-to-head.
    /// </summary>
    [Fact]
    public void AtHighApparentSpeedTheTrajectoryMatchesSpring()
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        const float operatorSpeed = 2f;
        var velocityMatchTrajectory = DriveStepCorrection(
            velocityMatch.Reconciler, framesAfter: 60, framesBefore: 60,
            correction: SmallCorrectionMeters, operatorSpeed: operatorSpeed);
        var springTrajectory = DriveStepCorrection(
            spring.Reconciler, framesAfter: 60, framesBefore: 60,
            correction: SmallCorrectionMeters, operatorSpeed: operatorSpeed);

        for (int i = 0; i < velocityMatchTrajectory.Count; i++)
        {
            Assert.Equal(
                springTrajectory[i].Displayed.Position.X,
                velocityMatchTrajectory[i].Displayed.Position.X,
                6);
        }
    }

    // ---- 7. The apparent-motion estimator ----

    [Fact]
    public void TheApparentSpeedEstimateTracksAConstantVelocityPredictor()
    {
        var fixture = new Fixture();
        const float operatorSpeed = 1.5f;

        // Ten smoothing time constants: (1 - e^-10) of the way there.
        for (int frame = 0; frame < 100; frame++)
        {
            long ticks = frame * FrameTicks;
            fixture.Reconciler.Reconcile(PredictedAt(ticks, 0f, 0f, operatorSpeed), ticks);
        }

        Assert.Equal(operatorSpeed, fixture.Reconciler.ApparentLinearSpeedMetersPerSecond, 3);
        Assert.Equal(0f, fixture.Reconciler.ApparentAngularSpeedRadiansPerSecond, 5);
    }

    /// <summary>
    /// The sharpest failure mode in the design, pinned. On the frame a correction is seeded the
    /// predictor's own output contains the step it just absorbed, so its finite difference is not
    /// operator motion. If the estimator consumed it, it would report a large apparent speed at
    /// exactly the instant a correction begins -- licensing the most aggressive possible correction
    /// at the moment the premise says to be gentlest. The estimate is held for that one frame
    /// instead.
    ///
    /// The operator is stationary throughout, so the correct estimate is exactly zero at every
    /// point, including across a 50 mm predictor jump that would otherwise read as 5 m/s.
    /// </summary>
    [Fact]
    public void ThePredictorsOwnJumpIsNotMistakenForApparentMotion()
    {
        var fixture = new Fixture();

        for (int frame = 0; frame < 3; frame++)
        {
            long ticks = frame * FrameTicks;
            fixture.Reconciler.Reconcile(PoseAt(0f), ticks);
        }

        Assert.Equal(0f, fixture.Reconciler.ApparentLinearSpeedMetersPerSecond);

        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        // The seeding frame: the predictor has jumped by 5 cm in 10 ms, i.e. an apparent 5 m/s.
        fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 3 * FrameTicks);
        Assert.Equal(0f, fixture.Reconciler.ApparentLinearSpeedMetersPerSecond);

        // And the frame after: both predictions are post-jump, so the difference is clean again.
        fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 4 * FrameTicks);
        Assert.Equal(0f, fixture.Reconciler.ApparentLinearSpeedMetersPerSecond);
    }

    /// <summary>
    /// Duplicate, out-of-order and gapped authoritative samples cannot disturb the velocity estimate
    /// or the displayed trajectory. Not because they are filtered carefully, but because
    /// <c>Observe</c> feeds the estimator nothing at all -- the estimate comes from the predictor's
    /// per-frame output. This test is the observable consequence of that design choice: a clean
    /// stream and a stream buried in duplicates, stale stamps and a several-hundred-millisecond gap
    /// must produce identical output.
    /// </summary>
    [Fact]
    public void DuplicateOutOfOrderAndGappedObservationsDoNotDisturbTheTrajectory()
    {
        var clean = new Fixture();
        var noisy = new Fixture();

        var cleanTrajectory = new List<Pose>();
        var noisyTrajectory = new List<Pose>();

        for (int frame = 0; frame < 3; frame++)
        {
            long ticks = frame * FrameTicks;
            clean.Reconciler.Reconcile(PoseAt(0f), ticks);
            noisy.Reconciler.Reconcile(PoseAt(0f), ticks);
        }

        var sample = new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters));
        clean.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        // The same sample, plus two re-deliveries and two samples from the past, in a jumbled order.
        noisy.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        noisy.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        noisy.Reconciler.Observe(
            new Stamped<Pose>(FrameTicks, PoseAt(3f)), PoseAt(0f), PredictorDiagnostics.None);
        noisy.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        noisy.Reconciler.Observe(
            new Stamped<Pose>(0, PoseAt(-7f)), PoseAt(0f), PredictorDiagnostics.None);

        Pose corrected = PoseAt(CorrectionMeters);
        for (int frame = 3; frame < 200; frame++)
        {
            long ticks = frame * FrameTicks;
            cleanTrajectory.Add(clean.Reconciler.Reconcile(corrected, ticks));
            noisyTrajectory.Add(noisy.Reconciler.Reconcile(corrected, ticks));

            // Keep re-delivering the same stale samples mid-decay.
            noisy.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        }

        for (int i = 0; i < cleanTrajectory.Count; i++)
        {
            AssertSamePose(cleanTrajectory[i], noisyTrajectory[i]);
        }

        Assert.Equal(
            clean.Reconciler.ApparentLinearSpeedMetersPerSecond,
            noisy.Reconciler.ApparentLinearSpeedMetersPerSecond);
        Assert.Equal(1, noisy.CountOf(CorrectionMagnitudeMm));
    }

    // ---- 8. Correction-cost metrics: identical definitions and cadence ----

    /// <summary>
    /// The correction-magnitude definition must be exactly the one <c>snap</c> and <c>spring</c> use.
    /// It measures the predictor's disagreement with truth <i>before</i> any reconciler acts, so it
    /// is identical across the axis by construction -- and this test is what keeps it that way.
    /// </summary>
    [Fact]
    public void CorrectionMagnitudeMatchesSnapExactly()
    {
        var velocityMatch = new Fixture();
        var snapMetrics = new InMemoryMetricTracker(256);
        var snap = new SnapReconciler(Config(), snapMetrics, new ManualClock(TicksPerSecond));

        var sample = new Stamped<Pose>(100, PoseAt(CorrectionMeters, 0.25f));
        velocityMatch.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        snap.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(
            velocityMatch.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double mm, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeMm, out double snapMm, out _));
        Assert.Equal(snapMm, mm);

        Assert.True(
            velocityMatch.Metrics.TryGetLatest(CorrectionMagnitudeDeg, out double deg, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeDeg, out double snapDeg, out _));
        Assert.Equal(snapDeg, deg);
    }

    /// <summary>
    /// Cadence, which decides whether a head-to-head compares algorithms or populations: one
    /// <c>jerk_mm_s3</c> per advancing frame once four displayed positions exist, in flight or not,
    /// exactly as <c>snap</c> and <c>spring</c> emit it. This candidate converges later than
    /// <c>spring</c> by design, so if emission were conditioned on a correction being in flight the
    /// two would contribute different numbers of samples and the percentiles would be
    /// uninterpretable.
    /// </summary>
    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull()
    {
        var fixture = new Fixture();
        int framesAfterCorrection = (int)(StatedWindowTicks / FrameTicks) + 20;
        DriveStepCorrection(fixture.Reconciler, framesAfterCorrection);

        Assert.Equal(framesAfterCorrection, fixture.CountOf(JerkMmS3));
        Assert.True(fixture.Reconciler.IsConverged, "expected the correction to have landed");

        long ticks = (3 + framesAfterCorrection) * FrameTicks;
        for (int frame = 0; frame < 10; frame++)
        {
            ticks += FrameTicks;
            fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), ticks);
        }

        Assert.Equal(framesAfterCorrection + 10, fixture.CountOf(JerkMmS3));
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
    }

    /// <summary>
    /// Cadence equality made explicit against the reference: driven identically, the two reconcilers
    /// must emit the same number of <c>jerk_mm_s3</c> samples. Unequal sample-set size is the failure
    /// mode that has already produced two wrong results on this axis today.
    /// </summary>
    [Fact]
    public void JerkCadenceIsIdenticalToSprings()
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        const int frames = 200;
        DriveStepCorrection(velocityMatch.Reconciler, frames);
        DriveStepCorrection(spring.Reconciler, frames);

        Assert.Equal(frames, velocityMatch.CountOf(JerkMmS3));
        Assert.Equal(frames, spring.ValuesOf(JerkMmS3).Count);
    }

    /// <summary>
    /// <b>A warning to whoever runs the head-to-head, pinned as a test so it cannot be rediscovered
    /// the hard way.</b>
    ///
    /// Over a window containing one correction and a long quiet tail, <c>jerk_mm_s3</c> percentiles
    /// below about p99 rank these two reconcilers <i>opposite</i> to how their peak jerk ranks them,
    /// and the percentile is the misleading one. Measured here (600 frames at 100 Hz, one 5 cm step,
    /// stationary operator):
    ///
    /// <list type="bullet">
    /// <item>max jerk: velocity-match 6.2e5, spring 7.2e6 mm/s³ -- velocity-match roughly 12x
    /// lower, which is the direction the gentler response predicts.</item>
    /// <item>p99: velocity-match 1.0e5, spring 4.6e5 -- still roughly 4.5x lower.</item>
    /// <item>p95: velocity-match 4.0e3, spring 3.7 -- velocity-match roughly 1000x <i>higher</i>.</item>
    /// </list>
    ///
    /// The cause is duration, not severity. Both emit exactly one sample per advancing frame (that
    /// equality is asserted separately), so the sample sets are the same size -- but the correction
    /// occupies 4x as many frames here, so ~111 frames carry non-trivial jerk against spring's ~32.
    /// At p95 of 600 samples the cut falls at rank 570, which is inside spring's tail of exact zeros
    /// and inside velocity-match's still-decaying tail. The percentile is therefore reporting <i>how
    /// long the correction lasted</i>, which <c>time_to_convergence_ms</c> already reports directly
    /// and honestly.
    ///
    /// This is not an argument for changing the metric -- <c>jerk_mm_s3</c>'s definition and cadence
    /// are shared across the axis and must not move (docs/metrics.md §5). It is an argument for
    /// reading peak/p99 jerk alongside convergence time, and for being suspicious of a jerk
    /// percentile computed over a trace where corrections are sparse.
    /// </summary>
    [Fact]
    public void JerkPercentilesBelowP99RankByCorrectionDurationRatherThanSeverity()
    {
        var velocityMatch = new Fixture();
        var spring = new SpringFixture();

        const int frames = 600;
        DriveStepCorrection(velocityMatch.Reconciler, frames);
        DriveStepCorrection(spring.Reconciler, frames);

        List<double> velocityMatchJerk = velocityMatch.ValuesOf(JerkMmS3);
        List<double> springJerk = spring.ValuesOf(JerkMmS3);
        velocityMatchJerk.Sort();
        springJerk.Sort();

        Assert.Equal(springJerk.Count, velocityMatchJerk.Count);

        double velocityMatchMax = velocityMatchJerk[^1];
        double springMax = springJerk[^1];
        Assert.True(
            velocityMatchMax < springMax * 0.5,
            $"the gentler response must produce a lower peak jerk: velocity-match {velocityMatchMax:E2} " +
            $"vs spring {springMax:E2} mm/s^3");

        double velocityMatchP95 = velocityMatchJerk[(int)(0.95 * (velocityMatchJerk.Count - 1))];
        double springP95 = springJerk[(int)(0.95 * (springJerk.Count - 1))];
        Assert.True(
            velocityMatchP95 > springP95,
            $"the p95 inversion this test exists to document has disappeared: velocity-match " +
            $"{velocityMatchP95:E2} vs spring {springP95:E2} mm/s^3. That is not necessarily a bug, " +
            "but the doc comment above is now wrong and the head-to-head guidance needs revisiting");
    }

    // ---- 9. Determinism, idempotence, gaps, reset ----

    [Fact]
    public void TwoInstancesGivenTheSameInputProduceIdenticalOutput()
    {
        var a = new Fixture();
        var b = new Fixture();

        var left = DriveStepCorrection(
            a.Reconciler, framesAfter: 120, framesBefore: 40, operatorSpeed: 0.4f);
        var right = DriveStepCorrection(
            b.Reconciler, framesAfter: 120, framesBefore: 40, operatorSpeed: 0.4f);

        Assert.Equal(left.Count, right.Count);
        for (int i = 0; i < left.Count; i++)
        {
            Assert.Equal(left[i].Ticks, right[i].Ticks);
            AssertSamePose(left[i].Displayed, right[i].Displayed);
        }

        Assert.Equal(
            a.Reconciler.ApparentLinearSpeedMetersPerSecond,
            b.Reconciler.ApparentLinearSpeedMetersPerSecond);
    }

    /// <summary>
    /// Two calls at the same tick must return the same state and must not advance anything twice --
    /// including, for this reconciler, the apparent-speed estimate, which would otherwise let a
    /// repeated frame nudge the warp.
    /// </summary>
    [Fact]
    public void Reconcile_AtOrBeforeTheLastTick_IsANoOp()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(
            fixture.Reconciler, framesAfter: 20, framesBefore: 40, operatorSpeed: 0.4f);
        Frame last = trajectory[^1];

        int jerkBefore = fixture.CountOf(JerkMmS3);
        float speedBefore = fixture.Reconciler.ApparentLinearSpeedMetersPerSecond;

        AssertSamePose(
            last.Displayed, fixture.Reconciler.Reconcile(PoseAt(999f), last.Ticks));
        AssertSamePose(
            last.Displayed, fixture.Reconciler.Reconcile(PoseAt(999f), last.Ticks - FrameTicks));

        Assert.Equal(jerkBefore, fixture.CountOf(JerkMmS3));
        Assert.Equal(speedBefore, fixture.Reconciler.ApparentLinearSpeedMetersPerSecond);
    }

    /// <summary>
    /// Real traces contain gaps of several hundred milliseconds. A gap must not produce NaN through
    /// the exponential or the warp ratio, must not overshoot, and must still land on the corrected
    /// prediction. The warp ratio is the new hazard here: over a long interval the trial step's
    /// residual velocity underflows toward zero, and dividing by it is exactly the <c>0/0</c> the
    /// speed epsilon exists to prevent.
    /// </summary>
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

        // 1500 ms of nothing, then one frame: well past the stated window, so the correction is over.
        Pose afterGap = fixture.Reconciler.Reconcile(corrected, 3 * FrameTicks + 1500);

        Assert.False(float.IsNaN(afterGap.Position.X));
        Assert.False(float.IsNaN(fixture.Reconciler.ApparentLinearSpeedMetersPerSecond));
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(CorrectionMeters, afterGap.Position.X, 5);
    }

    /// <summary>
    /// A reused instance must behave exactly like a brand-new one. The piece specific to this
    /// reconciler is the apparent-speed estimate: carrying one trial's operator motion across the
    /// boundary would set the next trial's correction rate from unrelated data, which would look
    /// like a suspiciously good result rather than like a bug.
    /// </summary>
    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var reused = new Fixture();

        // Only five frames past the correction, so it is still visibly in flight. (A longer run at
        // this apparent speed converges sooner than one might expect: the warp is a ratio against
        // the correction's *current* demanded speed, so it rises as the offset shrinks and the tail
        // of a correction is absorbed at close to full rate.)
        DriveStepCorrection(
            reused.Reconciler, framesAfter: 5, framesBefore: 60, operatorSpeed: 1.5f);
        Assert.False(reused.Reconciler.IsConverged);
        Assert.True(reused.Reconciler.ApparentLinearSpeedMetersPerSecond > 1f);

        reused.Reconciler.Reset();
        Assert.True(reused.Reconciler.IsConverged);
        Assert.Equal(0f, reused.Reconciler.ApparentLinearSpeedMetersPerSecond);
        Assert.Equal(0f, reused.Reconciler.ApparentAngularSpeedRadiansPerSecond);

        var fresh = new Fixture();
        var afterReset = DriveStepCorrection(reused.Reconciler, 60);
        var fromNew = DriveStepCorrection(fresh.Reconciler, 60);

        for (int i = 0; i < fromNew.Count; i++)
        {
            AssertSamePose(fromNew[i].Displayed, afterReset[i].Displayed);
        }
    }

    /// <summary>
    /// A correction arriving while an earlier one is still decaying must bend the trajectory, not
    /// restart it: the warped-time offset velocity is carried across the re-seed, and the convergence
    /// episode keeps its original onset so the reported time covers the whole episode.
    /// </summary>
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

        for (int frame = 5; frame < 300; frame++)
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
            fixture.Reconciler, 200, correction: 0f, correctionAngleRadians: 0.4f);

        Assert.True(fixture.Reconciler.IsConverged);
        Pose target = PoseAt(0f, 0.4f);
        Assert.True(
            PoseMath.OrientationErrorRadians(target, trajectory[^1].Displayed) <= 1e-3f,
            "orientation did not converge onto the corrected prediction");
    }

    // ---- 10. Allocation ----

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

    /// <summary>
    /// The moving-operator path allocates nothing either -- the apparent-motion estimator is the new
    /// per-frame work this reconciler adds, and it must stay on the struct-only path.
    /// </summary>
    [Fact]
    public void Reconcile_WithAMovingOperator_Allocates_Zero_Bytes()
    {
        var fixture = new Fixture();
        long ticks = 0;

        AllocationAssert.Zero(() =>
        {
            ticks += FrameTicks;
            fixture.Reconciler.Reconcile(PredictedAt(ticks, 0f, 0.2f, 1.5f), ticks);
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
}
