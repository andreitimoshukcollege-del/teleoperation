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
/// <c>exp-smooth</c> is C0 but <b>not</b> C1, and this suite is built around measuring that rather
/// than asserting continuity it cannot have — the pattern <see cref="SnapReconcilerTests"/>
/// established for the axis's one already-granted exception to requirement 2.
///
/// It owes more than <c>snap</c>'s suite does, because this violation is finite and analytic where a
/// snap's is simply "the whole correction in one frame". Section 5 below therefore proves, against
/// closed forms rather than magic numbers:
/// <list type="number">
/// <item>the onset offset-velocity step is exactly <c>|o0| / tau</c>;</item>
/// <item>it does <b>not</b> shrink as the frame interval is refined — <c>O(1)</c> in <c>dt</c>,
/// which is what distinguishes it from both a C1 response (<c>O(dt)</c>) and a snap
/// (<c>O(1/dt)</c>);</item>
/// <item>it scales <b>linearly with correction magnitude</b> and <b>inversely with the convergence
/// budget</b> — the scaling a human weighing an amendment to requirement 2 has to weigh;</item>
/// <item>the jerk consequence: an onset spike an order of magnitude above the smooth part of the
/// same response, diverging as <c>O(1/dt²)</c>.</item>
/// </list>
/// </summary>
public class ExponentialSmoothingReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The step correction under test, 5 cm. Small enough that the single pole's peak offset speed
    /// here (<c>|o0|/tau</c>, about 2.3 m/s) stays well under the rate caps, so the caps are
    /// exercised only by the test that is actually about them.
    /// </summary>
    private const float CorrectionMeters = 0.05f;

    private const float PositionTolerance = 1e-3f;

    /// <summary>
    /// <c>ln(1/0.01)</c>: the settle constant the implementation derives <c>tau</c> from. Duplicated
    /// here only so <see cref="SettleConstantMatchesItsDefiningEquation"/> can check the
    /// implementation's own <see cref="ExponentialSmoothingReconciler.TimeConstantSeconds"/> against
    /// the defining equation; every other test reads <c>tau</c> off the instance rather than
    /// re-deriving it.
    /// </summary>
    private const double SettleConstant = 4.605170185988091;

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
        public readonly ExponentialSmoothingReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new ExponentialSmoothingReconciler(
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
    /// One step correction, driven the way the pipeline drives it, matching
    /// <c>SpringReconcilerTests.DriveStepCorrection</c> frame for frame so the two suites exercise
    /// the same scenario. Three pass-through frames hold the display at the origin and fill the jerk
    /// history, then truth arrives disagreeing by <paramref name="correction"/>, and from then on the
    /// predictor's output already carries that truth — which is the jump this reconciler exists to
    /// hide.
    ///
    /// The <b>predicted</b> pose is returned alongside the displayed one because the residual offset
    /// is <c>displayed - predicted</c>, and the offset is the quantity every C1 measurement in
    /// section 5 is made on. Measuring the offset rather than the displayed pose keeps the shared
    /// one-frame hold (an artifact of seeding, common to the whole axis) out of the numbers that are
    /// supposed to characterise the single-pole law.
    ///
    /// Asserting pass-through on frames 0–2 is load-bearing, not decoration: it establishes that the
    /// offset is <i>identically zero</i> before onset, hence that its velocity there is exactly zero,
    /// which is the "before" side of the velocity step the witness tests measure.
    /// </summary>
    private static List<(long Ticks, Pose Predicted, Pose Displayed)> DriveStepCorrection(
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

        var trajectory = new List<(long Ticks, Pose Predicted, Pose Displayed)>(framesAfterCorrection);
        Pose corrected = PoseAt(correction, correctionAngleRadians);
        for (int frame = 3; frame < 3 + framesAfterCorrection; frame++)
        {
            long ticks = frame * frameTicks;
            trajectory.Add((ticks, corrected, fixture.Reconciler.Reconcile(corrected, ticks)));
        }

        return trajectory;
    }

    /// <summary>The residual offset along X, i.e. <c>displayed - predicted</c>.</summary>
    private static double OffsetX(in (long Ticks, Pose Predicted, Pose Displayed) frame) =>
        frame.Displayed.Position.X - frame.Predicted.Position.X;

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

    // ---- 1. Constructor validation ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentNullException>(
            () => new ExponentialSmoothingReconciler(Config(), null!, clock));
        Assert.Throws<ArgumentNullException>(
            () => new ExponentialSmoothingReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(
                Config(positionToleranceMeters: -1f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(
                Config(orientationToleranceRadians: -1f), metrics, clock));
    }

    /// <summary>
    /// Unlike <c>snap</c>, a budget and rate caps are <b>required</b>: a zero budget has no defined
    /// time constant, and a zero rate cap could never converge, which would violate requirement 1
    /// rather than express a tradeoff.
    /// </summary>
    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(
                Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new ExponentialSmoothingReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
    }

    /// <summary>
    /// The settle constant and the envelope fraction it is derived from must not drift apart
    /// silently. Where <c>spring</c>'s equivalent constant is a Lambert-W root with no closed form,
    /// this one is exactly <c>ln(100)</c>, so the round trip <c>e^(-T/tau) = 0.01</c> can be checked
    /// to the full precision the stored <c>float</c> time constant allows rather than to a few
    /// decimals.
    ///
    /// That precision is <b>float, not double</b>: <c>tau</c> is a <c>float</c> field, so the round
    /// trip lands on 0.010000001, about 1.1e-7 relative — one float ulp at this magnitude. The
    /// assertion is therefore a relative bound of 2e-7 rather than an absolute one; claiming more
    /// would be asserting past the type's precision, and claiming less would let a genuinely wrong
    /// constant through.
    /// </summary>
    [Fact]
    public void SettleConstantMatchesItsDefiningEquation()
    {
        var fixture = new Fixture();
        double budgetSeconds = BudgetTicks / (double)TicksPerSecond;
        double x = budgetSeconds / fixture.Reconciler.TimeConstantSeconds;

        Assert.Equal(SettleConstant, x, 5);
        AssertRelative(0.01, Math.Exp(-x), 2e-7);
    }

    [Fact]
    public void TheTimeConstantIsDerivedFromTheConvergenceBudget()
    {
        var tight = new Fixture(Config(maxTimeToConvergenceTicks: BudgetTicks));
        var loose = new Fixture(Config(maxTimeToConvergenceTicks: 2 * BudgetTicks));

        // tau = T / ln(100): doubling the budget doubles the time constant.
        Assert.Equal(
            2f * tight.Reconciler.TimeConstantSeconds,
            loose.Reconciler.TimeConstantSeconds,
            6);
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

    // ---- 4. Bounded convergence (requirement 1) ----

    /// <summary>
    /// The stated bound: <c>|o(t)| = |o0| e^(-t/tau)</c> is at 1% of the initial error when the
    /// budget elapses. Requirement 1 permits "zero, or a stated bound", and an exponential has a
    /// clean one — this is the half of the contract the single pole satisfies outright.
    /// </summary>
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
    /// The envelope itself, checked frame by frame against <c>|o0| e^(-t/tau)</c> rather than only at
    /// the deadline: the claim is that the decay law <i>is</i> the single pole, not merely that it
    /// happens to land in time.
    /// </summary>
    [Fact]
    public void TheResidualFollowsTheSinglePoleEnvelopeExactly()
    {
        var fixture = new Fixture();
        double tau = fixture.Reconciler.TimeConstantSeconds;
        var trajectory = DriveStepCorrection(fixture, 12);

        for (int i = 0; i < trajectory.Count; i++)
        {
            double elapsedSeconds = (trajectory[i].Ticks - trajectory[0].Ticks) / (double)TicksPerSecond;
            double expected = -CorrectionMeters * Math.Exp(-elapsedSeconds / tau);
            Assert.Equal(expected, OffsetX(trajectory[i]), 6);
        }
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

    /// <summary>
    /// A rate cap stretches convergence rather than breaking it, and the stretched bound is stated:
    /// <c>|o0| / v_max + T</c>. While the cap binds the offset shrinks strictly at <c>v_max</c>,
    /// which can last at most <c>|o0|/v_max</c>; from whatever remains the uncapped exponential
    /// reaches 1% within the budget.
    /// </summary>
    [Fact]
    public void ARateCapStretchesConvergenceWithinAStatedBound()
    {
        const float cappedSpeed = 0.1f;
        var capped = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: cappedSpeed));
        var uncapped = new Fixture();

        DriveStepCorrection(capped, 200);
        DriveStepCorrection(uncapped, 200);

        Assert.True(capped.Reconciler.IsConverged, "the capped correction never converged");
        double cappedMs = Assert.Single(Values(capped, TimeToConvergenceMs));
        double uncappedMs = Assert.Single(Values(uncapped, TimeToConvergenceMs));

        Assert.True(
            cappedMs > uncappedMs,
            $"expected the rate cap to slow convergence: capped {cappedMs} ms vs {uncappedMs} ms");

        double boundMs = (CorrectionMeters / cappedSpeed) * 1000.0 + BudgetTicks;
        Assert.True(
            cappedMs <= boundMs,
            $"capped convergence took {cappedMs} ms, above the stated bound of {boundMs} ms");
    }

    // ---- 5. Requirement 2: the exception, measured rather than asserted ----

    /// <summary>
    /// <b>The witness test.</b> A single-pole lag is C0 but not C1: the offset is continuous across
    /// onset (seeded from the displayed pose, so the visible <i>position</i> does not jump) while the
    /// offset <i>velocity</i> steps from exactly zero to <c>-o0/tau</c> in one instant. That is
    /// inherent to a first-order law — a first-order system reaches its terminal rate immediately —
    /// and this test states the size of the exception as a number rather than leaving it as a claim
    /// in a comment.
    ///
    /// The "before" side is exactly zero: <see cref="DriveStepCorrection"/> asserts pass-through on
    /// frames 0–2, so the offset is identically zero there and so is its derivative.
    ///
    /// The sampled step over one frame is <c>|o0| (1 - e^(-dt/tau)) / dt</c>, which is the continuous
    /// <c>|o0|/tau</c> reduced by the finite difference; both are asserted, the second as the limit
    /// the first approaches from below.
    /// </summary>
    [Fact]
    public void AtCorrectionOnset_TheOffsetVelocityStepsByItsClosedForm_AWitnessNotAClaim()
    {
        var fixture = new Fixture();
        double tau = fixture.Reconciler.TimeConstantSeconds;
        var trajectory = DriveStepCorrection(fixture, 4);

        double dt = FrameTicks / (double)TicksPerSecond;

        // Before onset: offset identically zero, hence zero velocity. The seed frame carries the
        // full offset already (seeding does not decay), so the first interval is the step.
        Assert.Equal(-CorrectionMeters, OffsetX(trajectory[0]), 6);

        double measuredStep = Math.Abs(OffsetX(trajectory[1]) - OffsetX(trajectory[0])) / dt;

        double sampledClosedForm = CorrectionMeters * (1.0 - Math.Exp(-dt / tau)) / dt;
        double continuousClosedForm = CorrectionMeters / tau;

        Assert.Equal(sampledClosedForm, measuredStep, 4);

        // ~1.845 m/s of injected velocity appearing from nothing, for a 5 cm correction on a 100 ms
        // budget. The continuous law is the ceiling the sampled step approaches as dt shrinks.
        Assert.True(
            measuredStep < continuousClosedForm,
            $"the sampled step {measuredStep} should sit below the continuous limit {continuousClosedForm}");
        Assert.True(
            measuredStep > 0.75 * continuousClosedForm,
            $"the sampled step {measuredStep} is nowhere near the continuous limit {continuousClosedForm}");

        // Stated plainly, so a change that quietly smoothed the onset would fail here: this is a
        // velocity discontinuity, and requirement 2 forbids exactly this.
        Assert.True(
            measuredStep > 1.0,
            $"expected a witness velocity step above 1 m/s, got {measuredStep} m/s");
    }

    /// <summary>
    /// The half of the C1 question that actually distinguishes a continuous velocity from a
    /// discontinuous one on a sampled signal — and the exact inverse of
    /// <c>SpringReconcilerTests.MaxPerFrameVelocityStepShrinksAsTheFrameIntervalIsRefined</c>.
    ///
    /// A C1 trajectory's per-frame velocity step is <c>O(dt)</c> and vanishes under refinement; a
    /// snap's is <c>O(1/dt)</c> and grows without bound. A single pole's is <b><c>O(1)</c></b>: it
    /// converges <i>upward</i> to the finite constant <c>|o0|/tau</c> and stops there. That
    /// finiteness is the whole of what the single pole buys over a snap, and its failure to vanish is
    /// the whole of what it costs against a C1 law.
    ///
    /// Uses a slow (1 s) budget for the reason <c>spring</c>'s counterpart test documents: the tick
    /// resolution here is 1 ms, so slowing the response is the only way to reach the <c>dt &lt;&lt;
    /// tau</c> regime where the asymptotics are visible.
    /// </summary>
    [Fact]
    public void TheOnsetVelocityStepDoesNotShrinkAsTheFrameIntervalIsRefined()
    {
        const long slowBudgetTicks = 1000;

        double coarse = OnsetOffsetVelocityStep(frameTicks: 8, budgetTicks: slowBudgetTicks);
        double fine = OnsetOffsetVelocityStep(frameTicks: 4, budgetTicks: slowBudgetTicks);
        double finer = OnsetOffsetVelocityStep(frameTicks: 2, budgetTicks: slowBudgetTicks);

        double limit = CorrectionMeters
            / new Fixture(Config(maxTimeToConvergenceTicks: slowBudgetTicks))
                .Reconciler.TimeConstantSeconds;

        // Monotonically approaching the limit from below rather than decaying toward zero. A C1
        // response would roughly halve at each refinement; this one moves by under 2%.
        Assert.True(fine > coarse, $"expected the step to grow toward its limit: {coarse} -> {fine}");
        Assert.True(finer > fine, $"expected the step to keep growing: {fine} -> {finer}");
        Assert.True(
            fine > 0.9 * coarse && finer > 0.9 * fine,
            $"a C1 step would shrink with dt; these did not: {coarse}, {fine}, {finer}");

        // And it is genuinely bounded away from zero: within 1% of |o0|/tau at the finest interval.
        Assert.Equal(limit, finer, 2);
        Assert.True(
            finer > 0.98 * limit,
            $"expected the refined step {finer} to sit within 2% of the closed-form limit {limit}");
    }

    /// <summary>
    /// <b>The first half of the comparison the panel needs: how the exception scales with correction
    /// magnitude.</b> <c>|dv| = |o0| / tau</c> is linear in <c>|o0|</c>, so the violation is largest
    /// exactly on the largest corrections — which is the regime that matters for nausea. A 1.3 m
    /// correction on a 100 ms budget injects a 60 m/s velocity step.
    /// </summary>
    [Fact]
    public void TheOnsetVelocityStepScalesLinearlyWithCorrectionMagnitude()
    {
        double atOneX = OnsetOffsetVelocityStep(FrameTicks, BudgetTicks, CorrectionMeters);
        double atTwoX = OnsetOffsetVelocityStep(FrameTicks, BudgetTicks, 2f * CorrectionMeters);
        double atTenX = OnsetOffsetVelocityStep(FrameTicks, BudgetTicks, 10f * CorrectionMeters);

        Assert.Equal(2.0, atTwoX / atOneX, 3);
        Assert.Equal(10.0, atTenX / atOneX, 3);

        // The consequence spelled out at the scale this project actually meets: the ~1.3 m opening
        // transient every trial in a sweep begins with would inject roughly 60 m/s.
        //
        // WITH THE RATE CAP OFF. At the operating point the numbered experiments actually use
        // (maxCorrectionLinearSpeedMetersPerSecond: 5) that step is clipped to 5 m/s -- see
        // TheRateCapBoundsTheOnsetVelocityStep. The linear law is the *uncapped* law and it
        // saturates at v_max; quoting 60 m/s without that qualification would overstate the
        // exception by an order of magnitude.
        double tau = new Fixture().Reconciler.TimeConstantSeconds;
        double uncappedStepForStartupTransient = 1.3 / tau;
        Assert.True(
            uncappedStepForStartupTransient > 55.0,
            $"expected the uncapped 1.3 m transient to inject a very large step, got {uncappedStepForStartupTransient} m/s");
    }

    /// <summary>
    /// <b>The qualification that changes the shape of the exception, found by running the candidate
    /// through the real harness rather than by unit test.</b>
    ///
    /// The onset velocity step is not <c>|o0|/tau</c> unconditionally — it is
    /// <c>min(|o0|/tau, v_max)</c>. The seed frame carries the whole offset, and the first advancing
    /// frame removes <c>offset * (1 - e^(-dt/tau))</c> clamped to <c>v_max * dt</c>, so
    /// <see cref="ReconcilerConfig.MaxCorrectionLinearSpeedMetersPerSecond"/> bounds the
    /// discontinuity directly.
    ///
    /// This matters for how requirement 2 should be read. The linear-in-magnitude law says the
    /// violation grows without bound on large corrections; the cap says it does not, because every
    /// numbered experiment on this axis configures <c>5 m/s</c>. Above
    /// <c>|o0| = v_max * tau</c> (about 11 cm at a 100 ms budget) the exception stops scaling with
    /// correction magnitude and becomes a configured constant. The <i>order</i> of the jerk
    /// divergence is unchanged — a step of <c>v_max</c> across one frame is still
    /// <c>O(1/dt²)</c> in jerk — but its coefficient is bounded by config rather than by the
    /// correction.
    /// </summary>
    [Fact]
    public void TheRateCapBoundsTheOnsetVelocityStep()
    {
        // The value experiments/exp-003 and exp-004 configure.
        const float capMetersPerSecond = 5f;

        double belowTheCap = OnsetOffsetVelocityStep(
            FrameTicks, BudgetTicks, CorrectionMeters, capMetersPerSecond);
        double theStartupTransient = OnsetOffsetVelocityStep(
            FrameTicks, BudgetTicks, 1.3f, capMetersPerSecond);

        // A 5 cm correction stays under the cap, so the closed form still governs it.
        Assert.True(
            belowTheCap < capMetersPerSecond,
            $"a 5 cm correction should not reach the cap; step was {belowTheCap} m/s");

        // The ~1.3 m opening transient is clipped exactly to the cap, not to 60 m/s.
        AssertRelative(capMetersPerSecond, theStartupTransient, 1e-3);
    }

    /// <summary>
    /// <b>The second half: how the exception scales with the convergence budget.</b>
    /// <c>tau = T / ln(100)</c>, so <c>|dv| = |o0| ln(100) / T</c> — halving the budget doubles the
    /// discontinuity. Together with the magnitude law above, this is the closed-form scaling a human
    /// weighing an amendment to requirement 2 has to weigh: the exception's size is not a fixed
    /// property of the algorithm, it is set by the operating point.
    /// </summary>
    [Fact]
    public void TheOnsetVelocityStepScalesInverselyWithTheConvergenceBudget()
    {
        // A fine frame interval so the sampled step sits close to its continuous limit; at a coarse
        // dt the finite difference itself would blunt the ratio.
        double atFullBudget = OnsetOffsetVelocityStep(frameTicks: 2, budgetTicks: 1000);
        double atHalfBudget = OnsetOffsetVelocityStep(frameTicks: 2, budgetTicks: 500);

        Assert.Equal(2.0, atHalfBudget / atFullBudget, 1);
        Assert.True(
            Math.Abs(atHalfBudget / atFullBudget - 2.0) < 0.02,
            $"expected halving the budget to double the step, ratio was {atHalfBudget / atFullBudget}");
    }

    /// <summary>
    /// <b>The jerk consequence of the step, quantified against a stated threshold.</b>
    ///
    /// With equal frame spacing the shared <see cref="DisplayedJerkEstimator"/> cascade reduces to
    /// the third difference <c>(p3 - 3p2 + 3p1 - p0)/dt³</c>. Driving the standard step correction,
    /// the displayed sequence from the seed frame is <c>0, c(1-d), c(1-d²), c(1-d³), ...</c> with
    /// <c>d = e^(-dt/tau)</c>, so the emitted jerks are, in order and in closed form:
    ///
    /// <code>
    ///   frame 3 (seed):  0
    ///   frame 4:         c (1-d)         / dt³      the onset transient begins
    ///   frame 5:         c (1-d)(2-d)    / dt³      the peak
    ///   frame 6 onward:  c (1-d)³ d^k    / dt³      the smooth part of the response
    /// </code>
    ///
    /// The stated threshold is derived from the response itself rather than picked: the onset peak is
    /// <c>(2-d)/(1-d)²</c> times the smooth steady-state jerk of the <i>same</i> correction, which at
    /// the default 100 ms budget and 100 Hz is <b>10.05x</b>. A reconciler whose onset transient is an
    /// order of magnitude above its own converged response is not C1, and this says so with a number.
    /// </summary>
    [Fact]
    public void TheOnsetJerkSpikeMatchesItsClosedFormAndDwarfsTheSmoothPartOfTheSameResponse()
    {
        var fixture = new Fixture();
        double tau = fixture.Reconciler.TimeConstantSeconds;
        DriveStepCorrection(fixture, 6);

        double dt = FrameTicks / (double)TicksPerSecond;
        double d = Math.Exp(-dt / tau);
        double scale = 1000.0 * CorrectionMeters / (dt * dt * dt); // mm/s³

        var jerks = Values(fixture, JerkMmS3);
        Assert.Equal(6, jerks.Count);

        Assert.Equal(0.0, jerks[0], 6);
        AssertRelative(scale * (1.0 - d), jerks[1], 1e-3);
        AssertRelative(scale * (1.0 - d) * (2.0 - d), jerks[2], 1e-3);
        AssertRelative(scale * Math.Pow(1.0 - d, 3.0), jerks[3], 1e-3);

        double peak = jerks[2];
        double smooth = jerks[3];

        // The stated threshold, derived from the response: (2-d)/(1-d)^2.
        double expectedRatio = (2.0 - d) / ((1.0 - d) * (1.0 - d));
        Assert.Equal(10.05, expectedRatio, 2);
        AssertRelative(expectedRatio, peak / smooth, 1e-3);

        // And in absolute terms, at the axis's default operating point: a 5 cm correction, 100 ms
        // budget, 100 Hz. 2.5e7 mm/s³ of peak jerk out of a correction whose smooth part is 2.5e6.
        const double onsetJerkWitnessThreshold = 1e7;
        Assert.True(
            peak > onsetJerkWitnessThreshold,
            $"expected an onset jerk witness above {onsetJerkWitnessThreshold:E1} mm/s^3, got {peak:E3}");
    }

    /// <summary>
    /// The honest counterweight to <see cref="TheOnsetVelocityStepDoesNotShrinkAsTheFrameIntervalIsRefined"/>.
    /// The velocity step is finite and frame-rate independent — but <c>jerk_mm_s3</c>, which is the
    /// metric this axis is actually compared on, is not: a velocity step of <c>|o0|/tau</c> spread
    /// across one frame is an acceleration of <c>|o0|/(tau dt)</c> and a jerk of
    /// <c>~2|o0|/(tau dt²)</c>. So the onset spike <b>diverges as <c>O(1/dt²)</c></b>: halving the
    /// frame interval roughly quadruples it.
    ///
    /// Reporting the finite velocity step without this would flatter the design. For reference, the
    /// exception the axis has already granted — <c>snap</c> — diverges one order faster, as
    /// <c>O(1/dt³)</c>.
    /// </summary>
    [Fact]
    public void TheOnsetJerkSpikeDivergesQuadraticallyAsTheFrameIntervalIsRefined()
    {
        const long slowBudgetTicks = 1000;

        double coarse = PeakOnsetJerk(frameTicks: 8, budgetTicks: slowBudgetTicks);
        double fine = PeakOnsetJerk(frameTicks: 4, budgetTicks: slowBudgetTicks);
        double finer = PeakOnsetJerk(frameTicks: 2, budgetTicks: slowBudgetTicks);

        double firstRatio = fine / coarse;
        double secondRatio = finer / fine;

        Assert.True(
            firstRatio > 3.5 && firstRatio < 4.5,
            $"halving dt should roughly quadruple the onset jerk (O(1/dt^2)); ratio was {firstRatio}");
        Assert.True(
            secondRatio > 3.5 && secondRatio < 4.5,
            $"halving dt again should quadruple it again; ratio was {secondRatio}");
    }

    /// <summary>
    /// The third face of the same limitation: a second correction arriving mid-decay steps the offset
    /// velocity again, from <c>-o_old/tau</c> to <c>-o_new/tau</c>, i.e. by
    /// <c>|o_new - o_old| / tau</c>. <c>spring</c> avoids this by carrying its offset velocity across
    /// the re-seed; a first-order law has no velocity state to carry, so the offset velocity is a
    /// pure function of the offset and re-seeding necessarily steps it.
    ///
    /// This matters more than the onset step on a lossy link, where retargets are the common case:
    /// the axis's impaired profiles produce ~500-600 corrections per trial against ~75 on
    /// <c>lan</c>, so this discontinuity fires on nearly every one of them.
    ///
    /// Asserted against the law <c>v = -o/tau</c> using offsets read off the trajectory, so the test
    /// checks the implementation's dynamics rather than a hand-derived scenario constant.
    /// </summary>
    [Fact]
    public void ARetargetMidDecayStepsTheOffsetVelocityByItsClosedForm()
    {
        const long budgetTicks = 1000;
        const long frameTicks = 2;
        const float firstCorrection = 0.05f;
        const float secondCorrection = 0.10f;

        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: budgetTicks));
        double tau = fixture.Reconciler.TimeConstantSeconds;
        double dt = frameTicks / (double)TicksPerSecond;

        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * frameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * frameTicks, PoseAt(firstCorrection)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        const int retargetFrame = 20;
        var offsets = new List<double>();
        var ticksOf = new List<long>();

        for (int frame = 3; frame <= retargetFrame; frame++)
        {
            Pose displayed = fixture.Reconciler.Reconcile(PoseAt(firstCorrection), frame * frameTicks);
            offsets.Add(displayed.Position.X - firstCorrection);
            ticksOf.Add(frame * frameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(retargetFrame * frameTicks, PoseAt(secondCorrection)),
            PoseAt(firstCorrection),
            PredictorDiagnostics.None);

        var afterOffsets = new List<double>();
        for (int frame = retargetFrame + 1; frame <= retargetFrame + 3; frame++)
        {
            Pose displayed = fixture.Reconciler.Reconcile(PoseAt(secondCorrection), frame * frameTicks);
            afterOffsets.Add(displayed.Position.X - secondCorrection);
        }

        double offsetBefore = offsets[^1];
        double offsetAfter = afterOffsets[0];

        double velocityBefore = (offsets[^1] - offsets[^2]) / dt;
        double velocityAfter = (afterOffsets[1] - afterOffsets[0]) / dt;

        double measuredStep = Math.Abs(velocityAfter - velocityBefore);
        double closedForm = Math.Abs(offsetAfter - offsetBefore) / tau;

        Assert.True(
            measuredStep > 0.1,
            $"expected a real retarget velocity step, got {measuredStep} m/s");
        AssertRelative(closedForm, measuredStep, 0.05);

        // With a predictor that is constant between the two samples, |o_new - o_old| reduces to the
        // change in the predicted pose, so the retarget step is |Δpredicted| / tau -- linear in the
        // retarget magnitude, exactly like the onset step.
        AssertRelative(
            Math.Abs(secondCorrection - firstCorrection) / tau, measuredStep, 0.05);
    }

    /// <summary>
    /// The crisp half of requirement 2 that the single pole <i>does</i> satisfy: the displayed
    /// position is continuous across onset. The correction begins from the pose already on screen, so
    /// the display does not jump even though the predictor has. C0, not C1 — and this test exists so
    /// the distinction is visible in the suite rather than only in prose.
    /// </summary>
    [Fact]
    public void TheDisplayedPositionIsContinuousAcrossCorrectionOnset()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 3);

        Assert.Equal(0f, trajectory[0].Displayed.Position.X, 6);
    }

    [Fact]
    public void TheDisplayedTrajectoryNeverOvershootsTheTarget()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, 40);

        float previous = float.NegativeInfinity;
        foreach ((long ticks, _, Pose displayed) in trajectory)
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
    /// <b>The one-frame display hold is inherited from the axis, deliberately.</b> Seeding the offset
    /// from the previous frame's displayed pose cancels the predictor's correction jump exactly, and
    /// as a side effect cancels the predictor's genuine motion over that frame, so the display
    /// freezes for one frame. That is an <c>O(1)</c> velocity dropout shared by every offset-carrying
    /// reconciler here.
    ///
    /// The obvious fix (<c>offset -= error</c>) was implemented across the smoothed reconcilers, swept
    /// on 2026-09-08 and measured <i>worse</i>: jerk p99 regressed 81–865% on the impaired profiles
    /// and the spread between reconcilers collapsed to 1.01x, i.e. the metric stopped discriminating
    /// between convergence laws. It was reverted. See <c>Reconciliation/CLAUDE.md</c>'s "Tried and
    /// rejected" and the <c>..._ADeliberateTradeoff</c> tests in the other suites.
    ///
    /// Pinned here so this reconciler sits on exactly the same footing as the rest of the axis: a
    /// one-sided "fix" would skew a head-to-head for a reason that has nothing to do with the
    /// convergence law.
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

    /// <summary>The same hold across a retarget, for the reason the onset test above documents.</summary>
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

    // ---- 6. Correction-cost metrics: definitions and cadence ----

    /// <summary>
    /// <b>Admissibility, not a comparison.</b> <c>correction_magnitude_mm</c>/<c>_deg</c> are measured
    /// from the predictor's disagreement <i>before</i> any reconciler acts, so they are identical
    /// across the axis by construction and act as a control. Emission cadence decides whether two
    /// implementations are comparable at all: if a candidate emitted a metric on a different
    /// schedule, pooled percentiles would compare populations rather than algorithms.
    ///
    /// Checked against <c>snap</c> — the axis's baseline, present in every recorded result — on one
    /// driven scenario: same metric names, same counts, same magnitude values, same stamps.
    /// </summary>
    [Fact]
    public void CorrectionMagnitudeAndEmissionCadenceMatchTheBaselineExactly()
    {
        var mine = new Fixture();
        var baselineMetrics = new InMemoryMetricTracker(8192);
        var baseline = new SnapReconciler(Config(), baselineMetrics, new ManualClock(TicksPerSecond));

        var authoritative = new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters, 0.2f));
        Pose predictedAtCapture = PoseAt(0f);

        for (int frame = 0; frame < 3; frame++)
        {
            mine.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
            baseline.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        mine.Reconciler.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);
        baseline.Observe(authoritative, predictedAtCapture, PredictorDiagnostics.None);

        Pose corrected = PoseAt(CorrectionMeters, 0.2f);
        for (int frame = 3; frame < 40; frame++)
        {
            mine.Reconciler.Reconcile(corrected, frame * FrameTicks);
            baseline.Reconcile(corrected, frame * FrameTicks);
        }

        foreach (string metric in new[]
                 { CorrectionMagnitudeMm, CorrectionMagnitudeDeg, JerkMmS3, TimeToConvergenceMs })
        {
            int mineCount = mine.CountOf(metric);
            int baselineCount = 0;
            for (int i = 0; i < baselineMetrics.Count; i++)
            {
                if (baselineMetrics[i].Name == metric)
                {
                    baselineCount++;
                }
            }

            Assert.True(
                mineCount == baselineCount,
                $"{metric}: emitted {mineCount} samples against the baseline's {baselineCount} -- " +
                "a cadence difference makes pooled percentiles incomparable");
        }

        Assert.True(mine.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double mineMm, out long mineTicks));
        Assert.True(baselineMetrics.TryGetLatest(CorrectionMagnitudeMm, out double baseMm, out long baseTicks));
        Assert.Equal(baseMm, mineMm);
        Assert.Equal(baseTicks, mineTicks);

        Assert.True(mine.Metrics.TryGetLatest(CorrectionMagnitudeDeg, out double mineDeg, out _));
        Assert.True(baselineMetrics.TryGetLatest(CorrectionMagnitudeDeg, out double baseDeg, out _));
        Assert.Equal(baseDeg, mineDeg);
    }

    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceTheHistoryIsFull()
    {
        var fixture = new Fixture();
        int framesAfterCorrection = (int)(BudgetTicks / FrameTicks) + 20;
        DriveStepCorrection(fixture, framesAfterCorrection);

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
        (long lastTicks, _, Pose lastDisplayed) = trajectory[^1];

        int jerkBefore = fixture.CountOf(JerkMmS3);

        AssertSamePose(lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks));
        AssertSamePose(
            lastDisplayed, fixture.Reconciler.Reconcile(PoseAt(999f), lastTicks - FrameTicks));

        Assert.Equal(jerkBefore, fixture.CountOf(JerkMmS3));
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

    /// <summary>
    /// Requirement 5, reached by not depending on predictor uncertainty at all — the same way
    /// <c>spring</c> and <c>snap</c> reach it. Identical output with and without a covariance-bearing
    /// <see cref="PredictorDiagnostics"/>.
    /// </summary>
    [Fact]
    public void BehavesIdenticallyWithAndWithoutPredictorUncertainty()
    {
        var without = new Fixture();
        var with = new Fixture();

        var uncertain = new PredictorDiagnostics(
            horizonTicks: 100,
            lastObservationTicks: 50,
            acceptedObservations: 10,
            rejectedObservations: 1,
            hasUncertainty: true,
            positionSigmaMeters: 0.25f,
            orientationSigmaRadians: 0.1f);

        Assert.False(PredictorDiagnostics.None.HasUncertainty);
        Assert.True(uncertain.HasUncertainty);

        for (int frame = 0; frame < 3; frame++)
        {
            without.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
            with.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        var authoritative = new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters, 0.3f));
        without.Reconciler.Observe(authoritative, PoseAt(0f), PredictorDiagnostics.None);
        with.Reconciler.Observe(authoritative, PoseAt(0f), uncertain);

        Pose corrected = PoseAt(CorrectionMeters, 0.3f);
        for (int frame = 3; frame < 30; frame++)
        {
            AssertSamePose(
                without.Reconciler.Reconcile(corrected, frame * FrameTicks),
                with.Reconciler.Reconcile(corrected, frame * FrameTicks));
        }

        Assert.Equal(without.Metrics.Count, with.Metrics.Count);
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

    // ---- helpers used by the section-5 measurements ----

    /// <summary>
    /// The sampled offset-velocity step at correction onset: the offset is exactly zero and static
    /// before the seed frame, so the first interval after it <i>is</i> the step.
    /// </summary>
    private static double OnsetOffsetVelocityStep(
        long frameTicks,
        long budgetTicks,
        float correction = CorrectionMeters,
        float maxLinearSpeed = 100f)
    {
        var fixture = new Fixture(Config(
            maxTimeToConvergenceTicks: budgetTicks,
            maxCorrectionLinearSpeedMetersPerSecond: maxLinearSpeed));
        var trajectory = DriveStepCorrection(fixture, 3, frameTicks, correction);
        double dt = frameTicks / (double)TicksPerSecond;

        return Math.Abs(OffsetX(trajectory[1]) - OffsetX(trajectory[0])) / dt;
    }

    /// <summary>
    /// The largest <c>jerk_mm_s3</c> emitted in the frames immediately following onset, which by the
    /// closed form in
    /// <see cref="TheOnsetJerkSpikeMatchesItsClosedFormAndDwarfsTheSmoothPartOfTheSameResponse"/> is
    /// the second frame after the seed.
    /// </summary>
    private static double PeakOnsetJerk(long frameTicks, long budgetTicks)
    {
        var fixture = new Fixture(Config(maxTimeToConvergenceTicks: budgetTicks));
        DriveStepCorrection(fixture, 10, frameTicks);

        double peak = 0.0;
        foreach (double value in Values(fixture, JerkMmS3))
        {
            peak = Math.Max(peak, value);
        }

        return peak;
    }

    private static void AssertRelative(double expected, double actual, double relativeTolerance)
    {
        double allowed = Math.Abs(expected) * relativeTolerance;
        Assert.True(
            Math.Abs(actual - expected) <= allowed,
            $"expected {expected:G9} +/- {allowed:G9}, got {actual:G9}");
    }
}
