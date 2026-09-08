using System;
using System.Collections.Generic;
using System.Numerics;
using Teleop.Core.Metrics;
using Teleop.Core.Reconciliation;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Reconciliation;

public class SpringReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The step correction under test, 5 cm. Small enough that the critically damped peak speed
    /// (about 1.2 m/s here) stays well under the rate caps, so the caps are exercised only by the
    /// test that is actually about them and cannot silently distort the others.
    /// </summary>
    private const float CorrectionMeters = 0.05f;

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
        public readonly InMemoryMetricTracker Metrics = new InMemoryMetricTracker(4096);
        public readonly SpringReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new SpringReconciler(
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
    /// One step correction, driven the way the pipeline drives it. Three pass-through frames hold
    /// the display at the origin and fill the jerk history, then truth arrives disagreeing by
    /// <paramref name="correction"/>, and from then on the predictor's output already carries that
    /// truth -- which is the jump this reconciler exists to hide.
    ///
    /// Returns one entry per frame from the correction onward, so a test can assert on the whole
    /// displayed trajectory rather than on its endpoints.
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

    // ---- 1. Constructor validation ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);

        Assert.Throws<ArgumentNullException>(
            () => new SpringReconciler(Config(), null!, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentNullException>(
            () => new SpringReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(
                Config(positionToleranceMeters: -1f), metrics, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(
                Config(orientationToleranceRadians: -1f), metrics, new ManualClock(TicksPerSecond)));
    }

    /// <summary>
    /// Unlike <c>snap</c>, which documents these three as ignored, this reconciler cannot express a
    /// response without a budget and cannot converge at all with a zero rate cap. Accepting them
    /// would produce a reconciler that silently never finishes -- the exact "lags indefinitely"
    /// failure Reconciliation/CLAUDE.md calls a bug rather than a tradeoff.
    /// </summary>
    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(Config(maxTimeToConvergenceTicks: -1), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new SpringReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
    }

    /// <summary>
    /// Guards the precomputed <c>CriticalSettleConstant</c> against its own defining equation, so
    /// the constant and the 1% settle fraction it encodes cannot drift apart silently. This is the
    /// test the constant's doc comment promises.
    /// </summary>
    [Fact]
    public void CriticalSettleConstantMatchesItsDefiningEquation()
    {
        var fixture = new Fixture();
        double budgetSeconds = BudgetTicks / (double)TicksPerSecond;
        double x = fixture.Reconciler.NaturalFrequencyRadiansPerSecond * budgetSeconds;

        // (1 + x) e^(-x) is the critically damped envelope at the budget, normalised by the
        // initial error. It must land on the documented 1%.
        double envelope = (1.0 + x) * Math.Exp(-x);
        Assert.Equal(0.01, envelope, 5);
    }

    [Fact]
    public void NaturalFrequencyIsDerivedFromTheConvergenceBudget()
    {
        var tight = new Fixture(Config(maxTimeToConvergenceTicks: BudgetTicks));
        var loose = new Fixture(Config(maxTimeToConvergenceTicks: 2 * BudgetTicks));

        // Halving the budget doubles the natural frequency: w = x / budgetSeconds.
        Assert.Equal(
            2f * loose.Reconciler.NaturalFrequencyRadiansPerSecond,
            tight.Reconciler.NaturalFrequencyRadiansPerSecond,
            3);
    }

    // ---- 2. The degenerate pass-through case ----

    /// <summary>
    /// The <c>none</c> + smoothed-reconciler pairing must reduce to exact pass-through, for the same
    /// reason <c>snap</c> documents: a stream of zero-magnitude "corrections" would inflate the
    /// correction rate with events that never happened.
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

    // ---- 3. Ordering: stale and duplicate samples ----

    [Fact]
    public void Observe_IgnoresStaleAndDuplicateSamplesWhole()
    {
        var fixture = new Fixture();
        var first = new Stamped<Pose>(100, PoseAt(CorrectionMeters));

        fixture.Reconciler.Observe(first, PoseAt(0f), PredictorDiagnostics.None);
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));

        // Same stamp again: a re-delivered datagram must not become a second correction.
        fixture.Reconciler.Observe(first, PoseAt(0f), PredictorDiagnostics.None);
        // And an older stamp.
        fixture.Reconciler.Observe(
            new Stamped<Pose>(50, PoseAt(1f)), PoseAt(0f), PredictorDiagnostics.None);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));
    }

    // ---- 4. Bounded convergence ----

    [Fact]
    public void AConstantCorrectionConvergesWithinTheBudget()
    {
        var fixture = new Fixture();
        // The correction onsets on the first frame after Observe, at tick 3 * FrameTicks; the
        // budget then allows BudgetTicks more. Drive a little past it to prove it does not merely
        // happen to be converging at the deadline.
        int framesToBudget = (int)(BudgetTicks / FrameTicks);
        var trajectory = DriveStepCorrection(fixture, framesToBudget + 4);

        long onsetTicks = 3 * FrameTicks;
        long deadlineTicks = onsetTicks + BudgetTicks;

        Assert.True(fixture.Reconciler.IsConverged, "did not converge at all");

        double convergedMs = Assert.Single(
            Values(fixture, TimeToConvergenceMs));
        Assert.True(
            convergedMs <= BudgetTicks,
            $"time_to_convergence_ms was {convergedMs}, above the {BudgetTicks} ms budget");

        // And the displayed pose is genuinely on the corrected prediction by the deadline.
        Pose atDeadline = trajectory.Find(entry => entry.Ticks == deadlineTicks).Displayed;
        Assert.True(
            Math.Abs(atDeadline.Position.X - CorrectionMeters) <= PositionTolerance,
            $"displayed X was {atDeadline.Position.X}, not within tolerance of {CorrectionMeters}");
    }

    /// <summary>
    /// <c>time_to_convergence_ms</c> is emitted exactly once per convergence episode, not once per
    /// frame: correction rate and convergence time are both reported metrics and a per-frame
    /// emission would make the episode look like dozens of corrections.
    /// </summary>
    [Fact]
    public void TimeToConvergenceIsEmittedOncePerEpisodeAndIsNonZero()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, (int)(BudgetTicks / FrameTicks) + 10);

        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
        double ms = Assert.Single(Values(fixture, TimeToConvergenceMs));

        // Unlike snap, which reports 0 by definition, a spread correction takes real time. That
        // difference is the whole quantity this axis trades against jerk.
        Assert.True(ms > 0.0, $"expected a positive convergence time, got {ms}");
    }

    [Fact]
    public void ARateCapStretchesConvergenceRatherThanBreakingIt()
    {
        // A cap far below the uncapped peak speed of roughly 1.2 m/s.
        var capped = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: 0.1f));
        var uncapped = new Fixture();

        int frames = 200;
        DriveStepCorrection(capped, frames);
        DriveStepCorrection(uncapped, frames);

        Assert.True(capped.Reconciler.IsConverged, "the capped correction never converged");
        double cappedMs = Assert.Single(Values(capped, TimeToConvergenceMs));
        double uncappedMs = Assert.Single(Values(uncapped, TimeToConvergenceMs));

        Assert.True(
            cappedMs > uncappedMs,
            $"expected the rate cap to slow convergence: capped {cappedMs} ms vs {uncappedMs} ms");
    }

    // ---- 5. No overshoot, and C1 continuity ----

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

    /// <summary>
    /// The crisp half of the C1 requirement: the correction begins from the pose that was already
    /// on screen, so the displayed position does not move at all on the onset frame and there is no
    /// velocity step to see. This is what seeding the offset from <c>lastDisplayed - predicted</c>
    /// buys, and it is the property <c>snap</c> structurally cannot have.
    /// </summary>
    [Fact]
    public void TheDisplayedPoseIsContinuousAcrossCorrectionOnset()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 3);

        // The last pass-through frame displayed the origin; the onset frame must display it too,
        // even though the predictor has already jumped to the corrected pose.
        Assert.Equal(0f, trajectory[0].Displayed.Position.X, 6);
    }

    /// <summary>
    /// The other half of C1, and the one that actually distinguishes a continuous velocity from a
    /// discontinuous one on a sampled signal: refine the frame interval and the largest per-frame
    /// velocity step must shrink with it. A C1 trajectory's velocity step is O(dt); a snap's is
    /// O(1/dt) and grows instead. Asserting a single absolute bound at one frame rate would pass
    /// for both.
    ///
    /// <b>Uses a deliberately slow convergence budget (1 s, so w is about 6.6 rad/s).</b> The O(dt)
    /// behaviour is asymptotic in <c>w*dt</c>, and at this project's default 100 ms budget
    /// <c>w*dt</c> is around 0.5 at 100 Hz -- coarse enough that the measured ratios are 0.81 and
    /// 0.66 rather than 0.5, which says nothing about continuity and everything about sampling the
    /// dynamics too sparsely to see it. With a 1 s budget the same refinement gives 0.53 and 0.51.
    /// The tick resolution here is 1 ms, so slowing the response is the only way to reach the
    /// regime; the alternative would be a microsecond clock for one test.
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
    /// <b>The display is held for exactly one frame at a correction, and that is deliberate.</b>
    /// Pinned here because it is a genuine C1 violation that was measured, found to be the better
    /// of the two available behaviours, and kept — not an oversight.
    ///
    /// Seeding the offset from the previous frame's displayed pose cancels the predictor's
    /// correction jump <i>exactly, by construction</i>: whatever the predictor did, the display is
    /// pinned to where it already was. The cost is that the predictor's genuine motion over that
    /// frame is cancelled too, so the display freezes for one frame — an O(1) velocity dropout that
    /// does not shrink as the frame interval falls.
    ///
    /// The obvious alternative — advance the offset, then apply the disagreement measured in
    /// <c>Observe</c> (<c>offset -= error</c>) — was implemented across all three reconcilers and
    /// swept on 2026-09-08. It is worse. The error is measured at <i>capture</i> time against
    /// <c>predictedAtCapture</c>, while the predictor's actual jump lands at <i>now</i> after it
    /// re-anchors and extrapolates, so the mismatch leaks a residual step onto every correction
    /// frame. That residual scales with correction frequency: jerk p99 improved 97% on <c>lan</c>
    /// (~75 corrections) but regressed 81–865% on the impaired profiles (~526 and ~625), and the
    /// spread between the three reconcilers collapsed from 2.8–3.7x to <b>1.01x</b> — the metric
    /// stopped measuring the convergence law and started measuring the shared artifact. See
    /// <c>Reconciliation/CLAUDE.md</c>.
    ///
    /// Removing the hold properly needs exact cancellation <i>and</i> motion continuity, which
    /// means advancing the displayed pose by the predictor's estimated rate before seeding. That is
    /// a real change and an open question, not something to patch in casually.
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

        // Held: the display is exactly where it was, despite the predictor advancing a full frame
        // and absorbing a correction. This is the exact-cancellation property being bought.
        Assert.Equal(3 * PerFrame, atSeedFrame.Position.X, 5);
    }

    /// <summary>
    /// The same hold across a retarget — a second correction arriving mid-decay, which is the
    /// common case on a lossy link. Kept for the reason the onset test above documents.
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

    // ---- 6. Correction-cost metrics ----

    /// <summary>
    /// The correction-magnitude definition must be byte-for-byte the one <c>snap</c> uses, or a
    /// <c>snap</c>-vs-<c>spring</c> sweep would be comparing metric implementations rather than
    /// reconcilers. Both delegate to <see cref="PoseMath"/>; this pins the observable consequence.
    /// </summary>
    [Fact]
    public void CorrectionMagnitudeMatchesSnapExactly()
    {
        var spring = new Fixture();
        var snapMetrics = new InMemoryMetricTracker(256);
        var snap = new SnapReconciler(Config(), snapMetrics, new ManualClock(TicksPerSecond));

        var sample = new Stamped<Pose>(100, PoseAt(CorrectionMeters, 0.25f));
        spring.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        snap.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(spring.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double springMm, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeMm, out double snapMm, out _));
        Assert.Equal(snapMm, springMm);

        Assert.True(spring.Metrics.TryGetLatest(CorrectionMagnitudeDeg, out double springDeg, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeDeg, out double snapDeg, out _));
        Assert.Equal(snapDeg, springDeg);
    }

    /// <summary>
    /// Jerk is a property of the displayed trajectory, not of a correction event, so it is emitted
    /// on every advancing frame once four displayed positions exist -- during a correction and
    /// after it alike. docs/metrics.md §5 defines it that way for a reason this axis feels
    /// directly: conditioning emission on "a correction is in flight" gave <c>snap</c> one sample
    /// per correction and a smoothed reconciler roughly one per frame of one, so pooled
    /// percentiles compared different populations rather than different reconcilers.
    /// </summary>
    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull()
    {
        var fixture = new Fixture();
        int framesAfterCorrection = (int)(BudgetTicks / FrameTicks) + 20;
        DriveStepCorrection(fixture, framesAfterCorrection);

        // The three pass-through frames before the correction fill the history without emitting --
        // a third derivative needs four points. Every frame from then on emits exactly one.
        Assert.Equal(framesAfterCorrection, fixture.CountOf(JerkMmS3));
        Assert.True(fixture.Reconciler.IsConverged, "expected the correction to have landed");

        // Convergence ends the *reporting episode*, not the emission: these frames are past it.
        long ticks = (3 + framesAfterCorrection) * FrameTicks;
        for (int frame = 0; frame < 10; frame++)
        {
            ticks += FrameTicks;
            fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), ticks);
        }

        Assert.Equal(framesAfterCorrection + 10, fixture.CountOf(JerkMmS3));

        // And still exactly one convergence report, however many jerk frames went by.
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
    }

    // ---- 7. Determinism, idempotence, gaps, reset ----

    [Fact]
    public void TwoInstancesGivenTheSameInputProduceIdenticalOutput()
    {
        var a = new Fixture();
        var b = new Fixture();

        var left = DriveStepCorrection(a, 30);
        var right = DriveStepCorrection(b, 30);

        Assert.Equal(left.Count, right.Count);
        for (int i = 0; i < left.Count; i++)
        {
            Assert.Equal(left[i].Ticks, right[i].Ticks);
            AssertSamePose(left[i].Displayed, right[i].Displayed);
        }
    }

    [Fact]
    public void Reconcile_AtOrBeforeTheLastTick_IsANoOp()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 4);
        (long lastTicks, Pose lastDisplayed) = trajectory[^1];

        int jerkBefore = fixture.CountOf(JerkMmS3);

        // A repeat at the same tick, and a frame delivered out of order.
        AssertSamePose(
            lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks));
        AssertSamePose(
            lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks - FrameTicks));

        Assert.Equal(jerkBefore, fixture.CountOf(JerkMmS3));
    }

    /// <summary>
    /// Real traces contain gaps of several hundred milliseconds. A gap must not produce NaN through
    /// the exponential, must not overshoot, and must still land on the corrected prediction.
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

        // 500 ms of nothing, then one frame. Five budgets have elapsed, so the correction is over.
        Pose afterGap = fixture.Reconciler.Reconcile(corrected, 3 * FrameTicks + 500);

        Assert.False(float.IsNaN(afterGap.Position.X));
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(CorrectionMeters, afterGap.Position.X, 5);
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var reused = new Fixture();
        DriveStepCorrection(reused, 4);
        Assert.False(reused.Reconciler.IsConverged);

        reused.Reconciler.Reset();
        Assert.True(reused.Reconciler.IsConverged);

        // A reused instance must behave exactly like a brand-new one, including the accepted-capture
        // baseline: a reconciler that remembered the previous trial's highest stamp would ignore the
        // whole opening stretch of the next one.
        var fresh = new Fixture();
        var afterReset = DriveStepCorrection(reused, 20);
        var fromNew = DriveStepCorrection(fresh, 20);

        for (int i = 0; i < fromNew.Count; i++)
        {
            AssertSamePose(fromNew[i].Displayed, afterReset[i].Displayed);
        }
    }

    /// <summary>
    /// A correction arriving while an earlier one is still decaying must bend the trajectory, not
    /// restart it: the offset velocity is carried across the re-seed, and the convergence episode
    /// keeps its original onset so the reported time covers the whole episode.
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

        // Mid-decay, a further disagreement.
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
        // Two corrections, one convergence episode.
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

    // ---- 8. Allocation ----

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

    private static List<double> Values(Fixture fixture, string name)
    {
        var values = new List<double>();
        for (int i = 0; i < fixture.Metrics.Count; i++)
        {
            if (fixture.Metrics[i].Name == name)
            {
                values.Add(fixture.Metrics[i].Value);
            }
        }

        return values;
    }
}
