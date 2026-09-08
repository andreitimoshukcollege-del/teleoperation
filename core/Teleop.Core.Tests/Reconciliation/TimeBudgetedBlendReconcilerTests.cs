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

public class TimeBudgetedBlendReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The step correction under test, 5 cm. Deliberately the same value
    /// <c>SpringReconcilerTests</c> uses, so the two suites exercise the same operating point, and
    /// small enough that the blend's peak speed (<c>15/8 * 0.05 / 0.1 = 0.94 m/s</c>) stays far under
    /// the rate caps -- the caps are exercised only by the tests that are actually about them and
    /// cannot silently stretch the deadline in the others.
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
        public readonly InMemoryMetricTracker Metrics = new InMemoryMetricTracker(8192);
        public readonly TimeBudgetedBlendReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new TimeBudgetedBlendReconciler(
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
    /// One step correction, driven the way the pipeline drives it, and deliberately the same shape as
    /// <c>SpringReconcilerTests.DriveStepCorrection</c> so the two suites are describing the same
    /// scenario. Three pass-through frames hold the display at the origin and fill the jerk history,
    /// then truth arrives disagreeing by <paramref name="correction"/>, and from then on the
    /// predictor's output already carries that truth -- which is the jump this reconciler exists to
    /// hide.
    ///
    /// Returns one entry per frame from the correction onward, so a test can assert on the whole
    /// displayed trajectory rather than on its endpoints.
    /// </summary>
    private static List<(long Ticks, Pose Displayed)> DriveStepCorrection(
        IReconciler<Pose> reconciler,
        int framesAfterCorrection,
        long frameTicks = FrameTicks,
        float correction = CorrectionMeters,
        float correctionAngleRadians = 0f)
    {
        for (int frame = 0; frame < 3; frame++)
        {
            long ticks = frame * frameTicks;
            AssertSamePose(PoseAt(0f), reconciler.Reconcile(PoseAt(0f), ticks));
        }

        long onsetCaptureTicks = 2 * frameTicks;
        reconciler.Observe(
            new Stamped<Pose>(onsetCaptureTicks, PoseAt(correction, correctionAngleRadians)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        var trajectory = new List<(long Ticks, Pose Displayed)>(framesAfterCorrection);
        Pose corrected = PoseAt(correction, correctionAngleRadians);
        for (int frame = 3; frame < 3 + framesAfterCorrection; frame++)
        {
            long ticks = frame * frameTicks;
            trajectory.Add((ticks, reconciler.Reconcile(corrected, ticks)));
        }

        return trajectory;
    }

    /// <summary>The tick at which a correction seeded by <see cref="DriveStepCorrection"/> onsets.</summary>
    private static long OnsetTicks(long frameTicks = FrameTicks) => 3 * frameTicks;

    // ---- 1. Constructor validation ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);

        Assert.Throws<ArgumentNullException>(
            () => new TimeBudgetedBlendReconciler(Config(), null!, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentNullException>(
            () => new TimeBudgetedBlendReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(positionToleranceMeters: -1f), metrics, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(orientationToleranceRadians: -1f), metrics, new ManualClock(TicksPerSecond)));
    }

    /// <summary>
    /// A zero budget leaves the blend nothing to schedule against and a zero rate cap would demand an
    /// infinite deadline -- the "lags indefinitely" failure Reconciliation/CLAUDE.md calls a bug
    /// rather than a tradeoff. Same rejections <c>spring</c> makes, for the same reason.
    /// </summary>
    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(maxTimeToConvergenceTicks: -1), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TimeBudgetedBlendReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
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
    public void FreshInstanceIsConvergedAndHasNoBlend()
    {
        var fixture = new Fixture();
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0L, fixture.Reconciler.BlendDurationTicks);
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

    /// <summary>
    /// A duplicate arriving <i>mid-blend</i> must not restart the schedule either. The deadline is the
    /// product this reconciler sells; a re-delivered datagram silently pushing it back would make the
    /// bound unfalsifiable in exactly the conditions (lossy links) the project studies.
    /// </summary>
    [Fact]
    public void Observe_DuplicateMidBlend_DoesNotRestartTheDeadline()
    {
        var fixture = new Fixture();

        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        var sample = new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters));
        fixture.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Pose corrected = PoseAt(CorrectionMeters);
        fixture.Reconciler.Reconcile(corrected, OnsetTicks());
        fixture.Reconciler.Reconcile(corrected, OnsetTicks() + FrameTicks);

        fixture.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Pose atDeadline = fixture.Reconciler.Reconcile(corrected, OnsetTicks() + BudgetTicks);
        AssertSamePose(corrected, atDeadline);
    }

    // ---- 4. Exact convergence at the deadline: the whole point of this reconciler ----

    /// <summary>
    /// The distinguishing claim, and the one that separates this from <c>spring</c>: at the deadline
    /// the residual offset is <b>identically zero</b>, not within a stated relative envelope, and this
    /// holds for <i>any</i> correction magnitude. The assertion is exact pose equality -- once the
    /// offset is bitwise zero, <c>Compose</c> short-circuits and returns the prediction unmodified, so
    /// the displayed pose is bit-identical to the corrected prediction rather than merely close to it.
    ///
    /// Seven magnitudes over ten orders of magnitude, from a micrometre to ten metres. <c>spring</c>
    /// cannot pass this test at any of them: its envelope is <c>1%</c> of the initial error at the
    /// budget, so the residual it leaves scales with the correction. The tolerances are set to 1e-9 so
    /// that even the micrometre correction is a real correction rather than one <c>Observe</c> rejects,
    /// and the rate caps are set high so the budget, not a cap, is the deadline for every row.
    /// </summary>
    [Theory]
    [InlineData(1e-6f)]
    [InlineData(1e-4f)]
    [InlineData(0.01f)]
    [InlineData(0.05f)]
    [InlineData(0.5f)]
    [InlineData(1.3f)]
    [InlineData(10f)]
    public void TheResidualIsExactlyZeroAtTheDeadline_ForAnyCorrectionMagnitude(float correction)
    {
        var fixture = new Fixture(Config(
            positionToleranceMeters: 1e-9f,
            orientationToleranceRadians: 1e-9f,
            maxCorrectionLinearSpeedMetersPerSecond: 1e9f,
            maxCorrectionAngularSpeedRadPerSecond: 1e9f));

        int framesToBudget = (int)(BudgetTicks / FrameTicks);
        var trajectory = DriveStepCorrection(
            fixture.Reconciler, framesToBudget + 2, correction: correction);

        long deadlineTicks = OnsetTicks() + BudgetTicks;

        Pose corrected = PoseAt(correction);
        Pose oneFrameEarly =
            trajectory.Find(entry => entry.Ticks == deadlineTicks - FrameTicks).Displayed;
        Pose atDeadline = trajectory.Find(entry => entry.Ticks == deadlineTicks).Displayed;
        Pose afterDeadline =
            trajectory.Find(entry => entry.Ticks == deadlineTicks + FrameTicks).Displayed;

        // Still short of truth one frame before the deadline -- otherwise the "exactly at the
        // deadline" assertion below would be satisfied by a reconciler that had simply snapped early.
        Assert.NotEqual(corrected.Position, oneFrameEarly.Position);

        AssertSamePose(corrected, atDeadline);
        AssertSamePose(corrected, afterDeadline);
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0L, fixture.Reconciler.BlendDurationTicks);
    }

    /// <summary>
    /// The deadline is honoured even when it does not land on a frame boundary: with the schedule
    /// running past it between two frames, the first frame at or after the deadline retires the blend
    /// exactly rather than evaluating the polynomial past <c>u = 1</c>, where the <c>(1-u)^3</c> factor
    /// changes sign and the offset would grow <i>away</i> from truth. This is the corner a
    /// finite-duration blend has and an exponential decay does not, so it gets its own test.
    /// </summary>
    [Fact]
    public void ADeadlineFallingBetweenTwoFramesRetiresExactlyOnTheNextFrame()
    {
        // 97 ticks against a 10-tick frame: the deadline at tick 30 + 97 = 127 falls between frames.
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: 97));

        var trajectory = DriveStepCorrection(fixture.Reconciler, 20);
        Pose corrected = PoseAt(CorrectionMeters);

        Pose before = trajectory.Find(entry => entry.Ticks == 120).Displayed;
        Pose after = trajectory.Find(entry => entry.Ticks == 130).Displayed;

        Assert.NotEqual(corrected.Position, before.Position);
        AssertSamePose(corrected, after);

        // And it did not sail past the target on the way: monotone, no sign flip.
        Assert.True(before.Position.X < corrected.Position.X);
        Assert.True(before.Position.X > 0f);
    }

    /// <summary>
    /// The blend never travels past truth and never reverses. A finite blend could in principle
    /// overshoot -- the quintic from rest does not, and this pins that, because an overshoot would read
    /// to an operator as the robot wobbling after every packet.
    /// </summary>
    [Fact]
    public void TheDisplayedTrajectoryNeverOvershootsTheTarget()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture.Reconciler, 40);

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
    /// The blend from rest is exactly <c>o0 * (1 - smootherstep(u))</c>, so the displayed pose is
    /// <c>target * smootherstep(u)</c>. This pins the identity the implementation's factored form
    /// relies on -- <c>(1-u)^3 (1 + 3u + 6u^2) = 1 - 10u^3 + 15u^4 - 6u^5</c> -- against an independent
    /// statement of the polynomial, so a change to the coefficients cannot pass unnoticed.
    /// </summary>
    [Fact]
    public void AnIsolatedBlendFollowsSmootherstepExactly()
    {
        const long slowBudgetTicks = 1000;
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: slowBudgetTicks));

        var trajectory = DriveStepCorrection(fixture.Reconciler, 100);
        long onset = OnsetTicks();

        foreach ((long ticks, Pose displayed) in trajectory)
        {
            double u = (ticks - onset) / (double)slowBudgetTicks;
            if (u > 1.0)
            {
                u = 1.0;
            }

            double smootherstep = u * u * u * (u * (u * 6.0 - 15.0) + 10.0);
            double expected = CorrectionMeters * smootherstep;

            Assert.True(
                Math.Abs(displayed.Position.X - expected) < 1e-7,
                $"at tick {ticks} (u={u}) displayed {displayed.Position.X}, expected {expected}");
        }
    }

    // ---- 5. C1 continuity ----

    /// <summary>
    /// The crisp half of the C1 requirement: the correction begins from the pose that was already on
    /// screen, so the displayed position does not move at all on the onset frame and there is no
    /// position step to see. Same property, same seeding, as <c>spring</c>; <c>snap</c> structurally
    /// cannot have it.
    /// </summary>
    [Fact]
    public void TheDisplayedPoseIsContinuousAcrossCorrectionOnset()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture.Reconciler, 3);

        Assert.Equal(0f, trajectory[0].Displayed.Position.X, 6);
    }

    /// <summary>
    /// The half of C1 that actually distinguishes a continuous velocity from a discontinuous one on a
    /// sampled signal: refine the frame interval and the largest per-frame velocity step must shrink
    /// with it. A C1 trajectory's velocity step is O(dt); a snap's is O(1/dt) and grows instead.
    /// Asserting a single absolute bound at one frame rate would pass for both -- the reasoning
    /// <c>SpringReconcilerTests.MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined</c>
    /// documents, reused here deliberately so the two suites make the same claim by the same means.
    ///
    /// The window spans <b>onset, mid-blend and completion</b> (four budgets of frames), so a velocity
    /// step at any of the three boundaries would show up as an O(1) floor that refinement cannot move.
    /// Completion is the boundary a finite blend can get wrong and an exponential cannot, which is the
    /// reason the terminal conditions are <c>o'(1) = o''(1) = 0</c> rather than just <c>o(1) = 0</c>.
    ///
    /// <b>Uses a deliberately slow budget (1 s).</b> The tick resolution here is 1 ms, so the only way
    /// to reach the regime where <c>dt</c> is small compared with the response is to slow the response;
    /// the alternative would be a microsecond clock for one test. Same constraint, and the same
    /// remedy, as the <c>spring</c> version.
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
        var trajectory = DriveStepCorrection(fixture.Reconciler, frames, frameTicks);

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

    // ---- 6. Retarget mid-blend ----

    /// <summary>
    /// A correction arriving mid-blend must <b>bend</b> the trajectory, not restart it from rest: the
    /// in-flight offset velocity is carried into the new blend's boundary conditions. Measured as the
    /// ratio of the offset's speed on the frame after the retarget to its speed on the frame before,
    /// which tends to 1 as the frame interval is refined.
    ///
    /// This is the test that would fail if the velocity were dropped -- a from-rest re-seed makes the
    /// new blend's initial velocity zero, so the ratio would collapse toward 0 rather than approach 1,
    /// and the displayed output would carry a velocity step on every retarget. Refining <c>dt</c>
    /// rather than asserting one number matters for the same reason it does in the C1 test above.
    /// </summary>
    [Fact]
    public void ARetargetMidBlendCarriesTheOffsetVelocityIntoTheNewBlend()
    {
        double coarse = OffsetSpeedRatioAcrossRetarget(frameTicks: 8);
        double fine = OffsetSpeedRatioAcrossRetarget(frameTicks: 2);

        // Not a from-rest restart: that would put this near zero at every frame interval.
        Assert.True(coarse > 0.5, $"the offset velocity looks dropped, not carried: ratio {coarse}");

        Assert.True(
            Math.Abs(fine - 1.0) < Math.Abs(coarse - 1.0),
            $"refining dt should bring the ratio closer to 1: {coarse} -> {fine}");
        Assert.True(
            Math.Abs(fine - 1.0) < 0.1,
            $"expected the carried velocity to match the in-flight velocity, ratio was {fine}");
    }

    /// <summary>
    /// Drives a blend, retargets it at a fixed <i>absolute</i> tick (so refining the frame interval
    /// does not move the retarget along the first blend's schedule), and returns
    /// <c>|offset velocity just after| / |offset velocity just before|</c>. The offset is recovered as
    /// <c>displayed - predicted</c>, which the test knows exactly.
    ///
    /// Velocities are measured on frame pairs that lie <i>entirely</i> within one blend or the other.
    /// The retarget frame itself is excluded on purpose: the offset jumps there by exactly the
    /// predictor's own jump, which is what cancels it out of the displayed pose, so a difference
    /// spanning that frame measures the predictor's step rather than the blend's velocity.
    /// </summary>
    private static double OffsetSpeedRatioAcrossRetarget(long frameTicks)
    {
        const long slowBudgetTicks = 1000;
        const long retargetTicks = 400;
        const float firstTarget = CorrectionMeters;
        const float secondTarget = 2f * CorrectionMeters;

        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: slowBudgetTicks));
        var reconciler = fixture.Reconciler;

        for (int frame = 0; frame < 3; frame++)
        {
            reconciler.Reconcile(PoseAt(0f), frame * frameTicks);
        }

        reconciler.Observe(
            new Stamped<Pose>(2 * frameTicks, PoseAt(firstTarget)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        var offsets = new List<(long Ticks, double Offset)>();
        int totalFrames = (int)(2 * slowBudgetTicks / frameTicks);
        float target = firstTarget;

        for (int frame = 3; frame < 3 + totalFrames; frame++)
        {
            long ticks = frame * frameTicks;

            if (ticks == retargetTicks)
            {
                reconciler.Observe(
                    new Stamped<Pose>(ticks - 1, PoseAt(secondTarget)),
                    PoseAt(firstTarget),
                    PredictorDiagnostics.None);
                target = secondTarget;
            }

            Pose predicted = PoseAt(target);
            Pose displayed = reconciler.Reconcile(predicted, ticks);
            offsets.Add((ticks, displayed.Position.X - predicted.Position.X));
        }

        int retargetIndex = offsets.FindIndex(entry => entry.Ticks == retargetTicks);
        Assert.True(retargetIndex >= 2, "the retarget frame was not sampled");

        double seconds = frameTicks / (double)TicksPerSecond;
        double before =
            (offsets[retargetIndex - 1].Offset - offsets[retargetIndex - 2].Offset) / seconds;
        double after =
            (offsets[retargetIndex + 1].Offset - offsets[retargetIndex].Offset) / seconds;

        Assert.True(Math.Abs(before) > 1e-4, "the in-flight offset was barely moving; test is vacuous");
        return after / before;
    }

    /// <summary>
    /// A retarget starts a fresh <i>full</i> deadline rather than inheriting the remainder of the old
    /// one, and the residual is exactly zero at that new deadline. Keeping the old deadline would make
    /// a correction landing just before it into a near-snap; see the type doc.
    /// </summary>
    [Fact]
    public void ARetargetMidBlendRestartsTheDeadlineAndStillLandsExactlyOnIt()
    {
        var fixture = new Fixture();
        var reconciler = fixture.Reconciler;

        for (int frame = 0; frame < 3; frame++)
        {
            reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        reconciler.Reconcile(PoseAt(CorrectionMeters), 30);
        reconciler.Reconcile(PoseAt(CorrectionMeters), 40);
        Assert.Equal(BudgetTicks, reconciler.BlendDurationTicks);

        // Retarget at tick 50, half a budget into the first blend.
        reconciler.Observe(
            new Stamped<Pose>(45, PoseAt(2f * CorrectionMeters)),
            PoseAt(CorrectionMeters),
            PredictorDiagnostics.None);

        Pose retargeted = PoseAt(2f * CorrectionMeters);
        reconciler.Reconcile(retargeted, 50);
        Assert.Equal(BudgetTicks, reconciler.BlendDurationTicks);

        // The first blend's deadline was 130; the new one is 50 + 100 = 150.
        Pose atOldDeadline = reconciler.Reconcile(retargeted, 130);
        Assert.NotEqual(retargeted.Position, atOldDeadline.Position);

        for (long ticks = 140; ticks < 150; ticks += FrameTicks)
        {
            reconciler.Reconcile(retargeted, ticks);
        }

        AssertSamePose(retargeted, reconciler.Reconcile(retargeted, 150));
        Assert.True(reconciler.IsConverged);

        Assert.Equal(2, fixture.CountOf(CorrectionMagnitudeMm));
        // Two corrections, one convergence episode -- the same episode accounting as spring.
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
    }

    /// <summary>
    /// <b>A documented shared artifact, not a property of this reconciler.</b> Seeding the offset from
    /// the <i>last displayed</i> pose means the displayed pose is held for exactly one frame at a
    /// retarget, because the new offset is computed against the previous frame's output. That is a
    /// one-frame velocity dropout, it is O(1) in the frame interval, and <c>spring</c> has it
    /// identically -- both reconcilers inherit it from the residual-offset seeding rule, not from
    /// their convergence laws.
    ///
    /// This test exists so a head-to-head cannot be confounded by one candidate having the artifact
    /// and the other not: it asserts the two behave the same way here. If a future change removes the
    /// hold from one of them, this test fails and the comparison gets re-examined rather than
    /// silently shifting. Removing it from both (by seeding from the offset advanced to the current
    /// frame) is a real improvement to the axis and is written up in
    /// docs/research-log/2026-09-08-budget-blend.md; it is deliberately not done here, because
    /// changing a shared behaviour of <c>spring</c> is not this file's business.
    /// </summary>
    [Fact]
    public void TheOneFrameHoldAtARetargetIsSharedWithSpring()
    {
        var blendMetrics = new InMemoryMetricTracker(1024);
        var springMetrics = new InMemoryMetricTracker(1024);

        var blend = new TimeBudgetedBlendReconciler(
            Config(), blendMetrics, new ManualClock(TicksPerSecond));
        var spring = new SpringReconciler(Config(), springMetrics, new ManualClock(TicksPerSecond));

        (Pose beforeBlend, Pose atBlend, Pose afterBlend) = RetargetHoldSamples(blend);
        (Pose beforeSpring, Pose atSpring, Pose afterSpring) = RetargetHoldSamples(spring);

        AssertHeld(beforeBlend, atBlend);
        AssertHeld(beforeSpring, atSpring);

        Assert.NotEqual(atBlend.Position, afterBlend.Position);
        Assert.NotEqual(atSpring.Position, afterSpring.Position);
    }

    /// <summary>
    /// The retarget hold is exact in exact arithmetic and exact to float rounding in practice: the
    /// output is <c>predicted + (lastDisplayed - predicted)</c>, and that round trip loses about an ulp
    /// of the predicted magnitude. Both reconcilers reconstruct the held pose the same way and so miss
    /// bit-equality by the same tiny amount -- around 4e-9 m on a 3 mm displacement -- which is why
    /// this compares to 1e-7 m rather than exactly. Contrast
    /// <see cref="TheResidualIsExactlyZeroAtTheDeadline_ForAnyCorrectionMagnitude"/>, which <i>is</i>
    /// bit-exact, because there the offset is bitwise zero and <c>Compose</c> short-circuits instead of
    /// adding anything.
    /// </summary>
    private static void AssertHeld(Pose expected, Pose actual)
    {
        Assert.True(
            Vector3.Distance(expected.Position, actual.Position) < 1e-7f,
            $"expected the display to be held at {expected.Position}, got {actual.Position}");
        Assert.True(PoseMath.OrientationErrorRadians(expected, actual) < 1e-6f);
    }

    private static (Pose Before, Pose At, Pose After) RetargetHoldSamples(IReconciler<Pose> reconciler)
    {
        for (int frame = 0; frame < 3; frame++)
        {
            reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        reconciler.Reconcile(PoseAt(CorrectionMeters), 30);
        reconciler.Reconcile(PoseAt(CorrectionMeters), 40);
        Pose before = reconciler.Reconcile(PoseAt(CorrectionMeters), 50);

        reconciler.Observe(
            new Stamped<Pose>(55, PoseAt(2f * CorrectionMeters)),
            PoseAt(CorrectionMeters),
            PredictorDiagnostics.None);

        Pose retargeted = PoseAt(2f * CorrectionMeters);
        Pose at = reconciler.Reconcile(retargeted, 60);
        Pose after = reconciler.Reconcile(retargeted, 70);

        return (before, at, after);
    }

    // ---- 7. Rate caps versus the deadline ----

    /// <summary>
    /// <c>PeakSpeedFactor</c> is the number that converts a rate cap into a minimum blend duration, so
    /// it has to be the true peak of the implemented polynomial and not a remembered value. Measured
    /// against a finely sampled run: a 1 s budget at a 1 ms frame gives a thousand samples of the
    /// blend, whose fastest frame must land on <c>(15/8) * o0 / T</c>.
    /// </summary>
    [Fact]
    public void PeakSpeedFactorMatchesTheImplementedBlendPolynomial()
    {
        const long slowBudgetTicks = 1000;
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: slowBudgetTicks));

        var trajectory = DriveStepCorrection(fixture.Reconciler, 1200, frameTicks: 1);

        double seconds = 1 / (double)TicksPerSecond;
        double peak = 0.0;
        for (int i = 1; i < trajectory.Count; i++)
        {
            double velocity =
                (trajectory[i].Displayed.Position.X - trajectory[i - 1].Displayed.Position.X) / seconds;
            peak = Math.Max(peak, Math.Abs(velocity));
        }

        double budgetSeconds = slowBudgetTicks / (double)TicksPerSecond;
        double expected = TimeBudgetedBlendReconciler.PeakSpeedFactor * CorrectionMeters / budgetSeconds;

        Assert.True(
            Math.Abs(peak - expected) < 0.01 * expected,
            $"peak blend speed {peak} m/s does not match PeakSpeedFactor's prediction {expected} m/s");
    }

    /// <summary>
    /// When the linear cap forbids absorbing the correction inside the configured budget, the deadline
    /// is <b>lengthened at seed time</b> to the shortest duration the cap allows -- rather than the
    /// velocity being clamped every frame, which is how <c>spring</c> does it and which a scheduled
    /// trajectory cannot survive. The cap therefore wins over the configured budget, exactly as
    /// <c>ExperimentConfig</c> documents, and the hard-deadline property survives the conflict: the
    /// residual is still exactly zero, at the longer deadline.
    ///
    /// Numbers: 0.7 m at a 5 m/s cap needs <c>15/8 * 0.7 / 5 = 0.2625 s</c>, so 263 ticks after
    /// ceiling-rounding -- two and a half times the 100 ms budget. Deliberately <b>not</b> a round
    /// number of ticks and not a frame boundary, so the assertion cannot be satisfied by a duration
    /// that happens to coincide with the frame grid.
    ///
    /// <c>BlendDurationTicks</c> is read while the blend is still in flight: it reports zero once the
    /// blend retires, which is what makes it a statement about the <i>current</i> correction.
    /// </summary>
    [Fact]
    public void ARateCapLengthensTheDeadlineRatherThanBeingExceeded()
    {
        const float correction = 0.7f;
        const float cap = 5f;
        const long expectedDurationTicks = 263;

        var fixture = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: cap));
        var trajectory = DriveStepCorrection(fixture.Reconciler, 2, correction: correction);

        Assert.Equal(expectedDurationTicks, fixture.Reconciler.BlendDurationTicks);

        Pose corrected = PoseAt(correction);
        for (int frame = 5; frame < 45; frame++)
        {
            long ticks = frame * FrameTicks;
            trajectory.Add((ticks, fixture.Reconciler.Reconcile(corrected, ticks)));
        }

        double seconds = FrameTicks / (double)TicksPerSecond;
        double peak = 0.0;
        for (int i = 1; i < trajectory.Count; i++)
        {
            double velocity =
                (trajectory[i].Displayed.Position.X - trajectory[i - 1].Displayed.Position.X) / seconds;
            peak = Math.Max(peak, Math.Abs(velocity));
        }

        Assert.True(peak <= cap * 1.001, $"peak displayed speed {peak} m/s exceeded the {cap} m/s cap");

        // The deadline is tick 30 + 263 = 293, between frames; the first frame at or after it retires
        // the blend exactly.
        Pose before = trajectory.Find(entry => entry.Ticks == 290).Displayed;
        Pose atDeadline = trajectory.Find(entry => entry.Ticks == 300).Displayed;

        Assert.NotEqual(corrected.Position, before.Position);
        AssertSamePose(corrected, atDeadline);

        // The convergence report reflects the lengthened deadline, not the configured budget.
        double convergedMs = Assert.Single(Values(fixture, TimeToConvergenceMs));
        Assert.True(
            convergedMs > BudgetTicks,
            $"expected the cap to stretch convergence past the {BudgetTicks} ms budget, got {convergedMs}");
        // Bounded by the first frame at or after the deadline: the deadline itself falls between
        // frames here, and a reconciler cannot report convergence on a frame that did not happen.
        Assert.True(
            convergedMs <= expectedDurationTicks + FrameTicks,
            $"convergence at {convergedMs} ms overran its own {expectedDurationTicks} ms deadline");
    }

    /// <summary>
    /// The angular cap sizes the same shared deadline. Both channels finish together, so
    /// <c>time_to_convergence_ms</c> names one instant rather than the later of two.
    ///
    /// 1.1 rad at 5 rad/s needs <c>15/8 * 1.1 / 5 = 0.4125 s</c>, i.e. 413 ticks after ceiling
    /// rounding, four times the 100 ms budget. 1.1 rad rather than a rounder angle on purpose: the
    /// offset rotation is recovered through a quaternion round trip, so an angle whose exact
    /// requirement landed on a whole tick would flip the ceiling by one on float noise alone and the
    /// test would be asserting the rounding rather than the sizing.
    /// </summary>
    [Fact]
    public void TheAngularCapAlsoLengthensTheSharedDeadline()
    {
        var fixture = new Fixture(Config(maxCorrectionAngularSpeedRadPerSecond: 5f));
        DriveStepCorrection(fixture.Reconciler, 5, correction: 0f, correctionAngleRadians: 1.1f);

        Assert.Equal(413L, fixture.Reconciler.BlendDurationTicks);
    }

    [Fact]
    public void ASmallCorrectionUnderBothCapsUsesTheConfiguredBudgetUnchanged()
    {
        var fixture = new Fixture(Config(
            maxCorrectionLinearSpeedMetersPerSecond: 5f,
            maxCorrectionAngularSpeedRadPerSecond: 10f));
        DriveStepCorrection(fixture.Reconciler, 3, correction: 0.005f);

        Assert.Equal(BudgetTicks, fixture.Reconciler.BlendDurationTicks);
    }

    // ---- 8. Correction-cost metrics ----

    /// <summary>
    /// The correction-magnitude definition must be byte-for-byte the one <c>snap</c> and
    /// <c>spring</c> use, or a three-way sweep would be comparing metric implementations rather than
    /// reconcilers. All three delegate to <see cref="PoseMath"/>; this pins the observable consequence.
    /// </summary>
    [Fact]
    public void CorrectionMagnitudeMatchesSnapAndSpringExactly()
    {
        var blend = new Fixture();
        var snapMetrics = new InMemoryMetricTracker(256);
        var springMetrics = new InMemoryMetricTracker(256);
        var snap = new SnapReconciler(Config(), snapMetrics, new ManualClock(TicksPerSecond));
        var spring = new SpringReconciler(Config(), springMetrics, new ManualClock(TicksPerSecond));

        var sample = new Stamped<Pose>(100, PoseAt(CorrectionMeters, 0.25f));
        blend.Reconciler.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        snap.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);
        spring.Observe(sample, PoseAt(0f), PredictorDiagnostics.None);

        Assert.True(blend.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double blendMm, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeMm, out double snapMm, out _));
        Assert.True(springMetrics.TryGetLatest(CorrectionMagnitudeMm, out double springMm, out _));
        Assert.Equal(snapMm, blendMm);
        Assert.Equal(springMm, blendMm);

        Assert.True(blend.Metrics.TryGetLatest(CorrectionMagnitudeDeg, out double blendDeg, out _));
        Assert.True(snapMetrics.TryGetLatest(CorrectionMagnitudeDeg, out double snapDeg, out _));
        Assert.True(springMetrics.TryGetLatest(CorrectionMagnitudeDeg, out double springDeg, out _));
        Assert.Equal(snapDeg, blendDeg);
        Assert.Equal(springDeg, blendDeg);
    }

    /// <summary>
    /// Jerk is a property of the displayed trajectory, not of a correction event, so it is emitted on
    /// every advancing frame once four displayed positions exist -- during a correction and after it
    /// alike. This is the cadence clause that makes the head-to-head legitimate: an implementation
    /// emitting on any other schedule would give the sweep a different-sized population and its
    /// percentiles would compare populations rather than algorithms.
    /// </summary>
    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull()
    {
        var fixture = new Fixture();
        int framesAfterCorrection = (int)(BudgetTicks / FrameTicks) + 20;
        DriveStepCorrection(fixture.Reconciler, framesAfterCorrection);

        // The three pass-through frames before the correction fill the history without emitting --
        // a third derivative needs four points. Every frame from then on emits exactly one.
        Assert.Equal(framesAfterCorrection, fixture.CountOf(JerkMmS3));
        Assert.True(fixture.Reconciler.IsConverged, "expected the correction to have landed");

        // Convergence ends the *reporting episode*, not the emission: these frames are past it, and
        // past the deadline too.
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
    /// The emission cadence must match <c>spring</c>'s exactly, sample for sample, or a pooled
    /// percentile over the two is comparing populations. Asserted directly rather than inferred: the
    /// same driven scenario must produce the same number of samples of every metric from both.
    /// </summary>
    [Fact]
    public void EmissionCadenceIsSampleForSampleIdenticalToSpring()
    {
        var blendMetrics = new InMemoryMetricTracker(4096);
        var springMetrics = new InMemoryMetricTracker(4096);
        var blend = new TimeBudgetedBlendReconciler(
            Config(), blendMetrics, new ManualClock(TicksPerSecond));
        var spring = new SpringReconciler(Config(), springMetrics, new ManualClock(TicksPerSecond));

        DriveStepCorrection(blend, 60);
        DriveStepCorrection(spring, 60);

        foreach (string metric in new[]
                 { CorrectionMagnitudeMm, CorrectionMagnitudeDeg, JerkMmS3, TimeToConvergenceMs })
        {
            Assert.Equal(CountOf(springMetrics, metric), CountOf(blendMetrics, metric));
        }
    }

    [Fact]
    public void TimeToConvergenceIsEmittedOncePerEpisodeAndIsBoundedByTheDeadline()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture.Reconciler, (int)(BudgetTicks / FrameTicks) + 10);

        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));
        double ms = Assert.Single(Values(fixture, TimeToConvergenceMs));

        // Unlike snap, which reports 0 by definition, a spread correction takes real time.
        Assert.True(ms > 0.0, $"expected a positive convergence time, got {ms}");
        Assert.True(ms <= BudgetTicks, $"convergence at {ms} ms overran the {BudgetTicks} ms deadline");
    }

    // ---- 9. Determinism, idempotence, gaps, reset ----

    [Fact]
    public void TwoInstancesGivenTheSameInputProduceIdenticalOutput()
    {
        var a = new Fixture();
        var b = new Fixture();

        var left = DriveStepCorrection(a.Reconciler, 30);
        var right = DriveStepCorrection(b.Reconciler, 30);

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
        var trajectory = DriveStepCorrection(fixture.Reconciler, 4);
        (long lastTicks, Pose lastDisplayed) = trajectory[^1];

        int jerkBefore = fixture.CountOf(JerkMmS3);

        // A repeat at the same tick, and a frame delivered out of order.
        AssertSamePose(lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks));
        AssertSamePose(
            lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks - FrameTicks));

        Assert.Equal(jerkBefore, fixture.CountOf(JerkMmS3));
    }

    /// <summary>
    /// Real traces contain gaps of several hundred milliseconds. A gap lands past the deadline, where
    /// the blend is retired rather than evaluated -- no polynomial is sampled past <c>u = 1</c>, no
    /// NaN, no overshoot, and the displayed pose is exactly the corrected prediction.
    /// </summary>
    [Fact]
    public void AGapOfSeveralHundredMillisecondsLandsExactlyOnTruth()
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

        // 500 ms of nothing, then one frame: five budgets have elapsed, so the blend is long over.
        Pose afterGap = fixture.Reconciler.Reconcile(corrected, 3 * FrameTicks + 500);

        Assert.False(float.IsNaN(afterGap.Position.X));
        AssertSamePose(corrected, afterGap);
        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0L, fixture.Reconciler.BlendDurationTicks);
    }

    /// <summary>
    /// A gap that lands <i>inside</i> the blend must evaluate the schedule at the frame that actually
    /// arrived rather than stepping it once per missed frame. A finite blend is a function of absolute
    /// time, so this is exact and not an approximation -- the displayed pose after a 45 ms gap equals
    /// what a run that never dropped a frame shows at the same tick.
    /// </summary>
    [Fact]
    public void AGapInsideTheBlendIsFrameRateIndependent()
    {
        var dense = new Fixture();
        var sparse = new Fixture();

        Pose corrected = PoseAt(CorrectionMeters);

        foreach (Fixture fixture in new[] { dense, sparse })
        {
            for (int frame = 0; frame < 3; frame++)
            {
                fixture.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
            }

            fixture.Reconciler.Observe(
                new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
                PoseAt(0f),
                PredictorDiagnostics.None);

            fixture.Reconciler.Reconcile(corrected, 30);
        }

        for (long ticks = 40; ticks <= 85; ticks += FrameTicks)
        {
            dense.Reconciler.Reconcile(corrected, ticks);
        }

        Pose denseAt85 = dense.Reconciler.Reconcile(corrected, 85);
        Pose sparseAt85 = sparse.Reconciler.Reconcile(corrected, 85);

        Assert.Equal(denseAt85.Position.X, sparseAt85.Position.X, 6);
    }

    [Fact]
    public void Reset_RestoresTheAsConstructedState()
    {
        var reused = new Fixture();
        DriveStepCorrection(reused.Reconciler, 4);
        Assert.False(reused.Reconciler.IsConverged);
        Assert.NotEqual(0L, reused.Reconciler.BlendDurationTicks);

        reused.Reconciler.Reset();
        Assert.True(reused.Reconciler.IsConverged);
        Assert.Equal(0L, reused.Reconciler.BlendDurationTicks);

        // A reused instance must behave exactly like a brand-new one, including the accepted-capture
        // baseline: a reconciler that remembered the previous trial's highest stamp would ignore the
        // whole opening stretch of the next one.
        var fresh = new Fixture();
        var afterReset = DriveStepCorrection(reused.Reconciler, 20);
        var fromNew = DriveStepCorrection(fresh.Reconciler, 20);

        for (int i = 0; i < fromNew.Count; i++)
        {
            AssertSamePose(fromNew[i].Displayed, afterReset[i].Displayed);
        }
    }

    [Fact]
    public void AnOrientationOnlyCorrectionConvergesExactlyOnTheGeodesic()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(
            fixture.Reconciler, 40, correction: 0f, correctionAngleRadians: 0.4f);

        Assert.True(fixture.Reconciler.IsConverged);

        Pose target = PoseAt(0f, 0.4f);
        Pose atDeadline =
            trajectory.Find(entry => entry.Ticks == OnsetTicks() + BudgetTicks).Displayed;

        // Exactly, not within tolerance: the offset is bitwise zero at the deadline, so Compose
        // returns the prediction unmodified.
        AssertSamePose(target, atDeadline);
        Assert.Equal(0f, PoseMath.OrientationErrorRadians(target, trajectory[^1].Displayed), 6);
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

    private static int CountOf(InMemoryMetricTracker metrics, string name)
    {
        int count = 0;
        for (int i = 0; i < metrics.Count; i++)
        {
            if (metrics[i].Name == name)
            {
                count++;
            }
        }

        return count;
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
