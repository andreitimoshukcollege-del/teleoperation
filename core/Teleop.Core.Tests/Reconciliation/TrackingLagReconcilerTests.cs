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
/// Tests for <c>exp-smooth-track</c> / <see cref="TrackingLagReconciler"/>: the no-offset
/// first-order tracking lag.
///
/// This suite is deliberately shaped around the two halves of the axis's bounded-convergence
/// requirement, because for this law they disagree with each other and the disagreement is the
/// result:
/// <list type="bullet">
/// <item><see cref="UnderStaticTruthTheErrorDecaysExponentiallyToTheStatedEnvelope"/> proves the
/// half that is satisfied.</item>
/// <item><see cref="AgainstMovingTruthTheDisplayedPoseSettlesAtTheClosedFormBiasAndStaysThere"/>
/// proves the half that is not, against the closed form rather than against a threshold.</item>
/// </list>
/// and around a <b>witness</b> for the C1 clause (<see cref="APredictorJumpStepsTheDisplayedVelocity"/>)
/// in the style <c>snap</c>'s suite uses, because this law does not have C1 continuity and asserting
/// it would be asserting something false.
///
/// No comparison against another reconciler appears in this file. The one place another
/// implementation is named is a comment explaining why a shared behaviour is shared.
/// </summary>
public class TrackingLagReconcilerTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>A frame interval, 100 Hz.</summary>
    private const long FrameTicks = 10;

    /// <summary>The convergence budget every test uses unless it says otherwise: 100 ms.</summary>
    private const long BudgetTicks = 100;

    /// <summary>
    /// The step correction under test, 5 cm. Small enough that the resulting chase speed stays far
    /// under the default rate caps, so the caps are exercised only by the test that is about them.
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
        public readonly TrackingLagReconciler Reconciler;

        public Fixture(ReconcilerConfig? config = null)
        {
            Reconciler = new TrackingLagReconciler(
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
    /// One step correction, driven the way the pipeline drives it: three frames of agreement fill
    /// the jerk history, then truth arrives disagreeing by <paramref name="correction"/>, and from
    /// then on the predictor's output already carries that truth -- which is the jump this
    /// reconciler has to absorb.
    ///
    /// Returns one entry per frame from the correction onward.
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
    /// Drives the reconciler against a prediction advancing at a constant speed along +X, the way a
    /// steadily moving operator drives it. Returns the displayed X and the prediction's X per frame
    /// so the lag between them can be measured directly.
    /// </summary>
    private static List<(long Ticks, float PredictedX, float DisplayedX)> DriveConstantVelocity(
        Fixture fixture, int frames, float speedMetersPerSecond, long frameTicks = FrameTicks)
    {
        var samples = new List<(long, float, float)>(frames);
        float frameSeconds = frameTicks / (float)TicksPerSecond;

        for (int frame = 0; frame < frames; frame++)
        {
            long ticks = frame * frameTicks;
            float predictedX = speedMetersPerSecond * frame * frameSeconds;
            Pose displayed = fixture.Reconciler.Reconcile(PoseAt(predictedX), ticks);
            samples.Add((ticks, predictedX, displayed.Position.X));
        }

        return samples;
    }

    // ---- 1. Construction and configuration ----

    [Fact]
    public void Constructor_RejectsMissingDependenciesAndInvalidConfiguration()
    {
        var metrics = new InMemoryMetricTracker(8);

        Assert.Throws<ArgumentNullException>(
            () => new TrackingLagReconciler(Config(), null!, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentNullException>(
            () => new TrackingLagReconciler(Config(), metrics, null!));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(Config(), metrics, new ManualClock(0)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(
                Config(positionToleranceMeters: -1f), metrics, new ManualClock(TicksPerSecond)));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(
                Config(orientationToleranceRadians: -1f), metrics, new ManualClock(TicksPerSecond)));
    }

    /// <summary>
    /// A zero budget has no defined time constant and a zero rate cap would freeze the display
    /// permanently -- which would be the "lags indefinitely" failure the axis calls a bug rather
    /// than a tradeoff, and this law is close enough to that line already without accepting a
    /// configuration that walks straight over it.
    /// </summary>
    [Fact]
    public void Constructor_RejectsANonPositiveBudgetOrRateCap()
    {
        var metrics = new InMemoryMetricTracker(8);
        var clock = new ManualClock(TicksPerSecond);

        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(Config(maxTimeToConvergenceTicks: 0), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(Config(maxTimeToConvergenceTicks: -1), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(
                Config(maxCorrectionLinearSpeedMetersPerSecond: 0f), metrics, clock));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new TrackingLagReconciler(
                Config(maxCorrectionAngularSpeedRadPerSecond: 0f), metrics, clock));
    }

    /// <summary>
    /// Guards the precomputed <c>LagSettleConstant</c> against its own defining equation, so the
    /// constant and the 1% settle fraction it encodes cannot drift apart silently. This is the test
    /// the constant's doc comment promises.
    /// </summary>
    [Fact]
    public void LagSettleConstantMatchesItsDefiningEquation()
    {
        var fixture = new Fixture();
        double budgetSeconds = BudgetTicks / (double)TicksPerSecond;
        double x = budgetSeconds / fixture.Reconciler.TimeConstantSeconds;

        // e^(-x) is the first-order envelope at the budget, normalised by the initial error. It
        // must land on the documented 1%.
        Assert.Equal(0.01, Math.Exp(-x), 6);
    }

    [Fact]
    public void TimeConstantIsDerivedFromTheConvergenceBudget()
    {
        var tenth = new Fixture(Config(maxTimeToConvergenceTicks: BudgetTicks / 10));
        var full = new Fixture(Config(maxTimeToConvergenceTicks: BudgetTicks));

        // tau is proportional to the budget, so a tenth of the budget is a tenth of the constant.
        Assert.Equal(
            full.Reconciler.TimeConstantSeconds / 10f,
            tenth.Reconciler.TimeConstantSeconds,
            6);

        // And the absolute value is budget / ln(100).
        Assert.Equal(
            (float)(BudgetTicks / (double)TicksPerSecond / Math.Log(100.0)),
            full.Reconciler.TimeConstantSeconds,
            6);
    }

    // ---- 2. Requirement 1, the half this law satisfies: static truth ----

    /// <summary>
    /// <b>Bounded convergence under a genuinely constant correction.</b> With the prediction held
    /// still, the displayed error is exactly geometric -- <c>e_n = e_0 r^n</c> with
    /// <c>r = exp(-dt/tau)</c> -- so it matches the continuous envelope <c>|e0| exp(-t/tau)</c> at
    /// every frame time, and it is at 1% of the initial error one budget after onset. That is the
    /// stated bound <see cref="Teleop.Core.Contracts.IReconciler{TState}"/> permits in place of
    /// "reaches zero".
    ///
    /// Asserted against the closed form frame by frame rather than only at the endpoint, because an
    /// endpoint assertion would pass for any law that happens to land in the right place.
    /// </summary>
    [Fact]
    public void UnderStaticTruthTheErrorDecaysExponentiallyToTheStatedEnvelope()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 20);

        float tau = fixture.Reconciler.TimeConstantSeconds;
        float frameSeconds = FrameTicks / (float)TicksPerSecond;

        for (int n = 0; n < trajectory.Count; n++)
        {
            float error = CorrectionMeters - trajectory[n].Displayed.Position.X;

            // The first advancing frame after onset already takes one step, hence (n + 1).
            float expected = CorrectionMeters * MathF.Exp(-(n + 1) * frameSeconds / tau);

            Assert.Equal(expected, error, 6);
        }

        // Ten frames of 10 ms is the 100 ms budget: the envelope must be at the documented 1%.
        float errorAtBudget = CorrectionMeters - trajectory[9].Displayed.Position.X;
        Assert.Equal(0.01f * CorrectionMeters, errorAtBudget, 6);
    }

    /// <summary>
    /// The static-truth response reaches the configured position tolerance in bounded time and stays
    /// inside it, and the reconciler reports that through <c>IsConverged</c> and one
    /// <c>time_to_convergence_ms</c> sample. This is the assertion the bounded-convergence clause
    /// actually asks for; the test above is the shape of the approach.
    /// </summary>
    [Fact]
    public void UnderStaticTruthTheDisplayLandsInsideToleranceAndReportsItOnce()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 60);

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(1, fixture.CountOf(TimeToConvergenceMs));

        float finalError = CorrectionMeters - trajectory[^1].Displayed.Position.X;
        Assert.True(
            finalError <= PositionTolerance,
            $"static-truth error must land inside the {PositionTolerance} m tolerance, was {finalError} m");

        // The bound is relative (1% of the initial error at the budget), so a 5 cm correction needs
        // slightly more than one budget to reach a 1 mm absolute tolerance. Bounded, and stated.
        Assert.True(fixture.Metrics.TryGetLatest(TimeToConvergenceMs, out double convergenceMs, out _));
        Assert.InRange(convergenceMs, 0.0, 4.0 * BudgetTicks);
    }

    // ---- 3. Requirement 1, the half this law does NOT satisfy: moving truth ----

    /// <summary>
    /// <b>The defining property of this candidate, proved rather than asserted.</b> Against a
    /// prediction advancing at constant speed the displayed pose settles at the exact discrete fixed
    /// point <c>e* = v dt r / (1 - r)</c> and <b>stays there</b> -- the error at twenty budgets is
    /// the same as at ten, so the display never catches up while motion continues.
    ///
    /// Three separate things are checked, because each could fail on its own:
    /// <list type="number">
    /// <item>the settled lag matches <see cref="TrackingLagReconciler.SteadyStateLagMeters"/>, i.e.
    /// the published closed form is the real bias and not an optimistic one;</item>
    /// <item>the lag does not shrink between the tenth and twentieth budget, i.e. it is a bias and
    /// not a slow transient;</item>
    /// <item>the lag exceeds the configured convergence tolerance, i.e. this is a visible failure to
    /// converge and not a technicality below the noise floor.</item>
    /// </list>
    /// </summary>
    [Fact]
    public void AgainstMovingTruthTheDisplayedPoseSettlesAtTheClosedFormBiasAndStaysThere()
    {
        const float SpeedMetersPerSecond = 0.5f;

        var fixture = new Fixture();
        var samples = DriveConstantVelocity(fixture, frames: 201, SpeedMetersPerSecond);

        float frameSeconds = FrameTicks / (float)TicksPerSecond;
        float closedForm = fixture.Reconciler.SteadyStateLagMeters(SpeedMetersPerSecond, frameSeconds);

        float lagAtTenBudgets = samples[100].PredictedX - samples[100].DisplayedX;
        float lagAtTwentyBudgets = samples[200].PredictedX - samples[200].DisplayedX;

        Assert.Equal(closedForm, lagAtTwentyBudgets, 5);
        Assert.Equal(lagAtTenBudgets, lagAtTwentyBudgets, 5);

        Assert.True(
            lagAtTwentyBudgets > PositionTolerance,
            $"the tracking bias must exceed the convergence tolerance for this to be a real failure " +
            $"to converge: bias {lagAtTwentyBudgets} m vs tolerance {PositionTolerance} m");

        // And it is never converged, for the whole duration of the motion.
        Assert.False(fixture.Reconciler.IsConverged || lagAtTwentyBudgets <= PositionTolerance);
    }

    /// <summary>
    /// The bias scales linearly with operator speed. This is what makes it a property of the law
    /// rather than of the trace: doubling the speed doubles the lag, so there is no operating point
    /// at which it becomes negligible other than standing still.
    /// </summary>
    [Fact]
    public void TheTrackingBiasIsProportionalToOperatorSpeed()
    {
        var slow = new Fixture();
        var fast = new Fixture();

        var slowSamples = DriveConstantVelocity(slow, frames: 201, speedMetersPerSecond: 0.25f);
        var fastSamples = DriveConstantVelocity(fast, frames: 201, speedMetersPerSecond: 0.5f);

        float slowLag = slowSamples[200].PredictedX - slowSamples[200].DisplayedX;
        float fastLag = fastSamples[200].PredictedX - fastSamples[200].DisplayedX;

        Assert.Equal(2f * slowLag, fastLag, 5);
    }

    /// <summary>
    /// The closed form's continuous limit is <c>v * tau</c>: as the frame interval shrinks the
    /// discrete fixed point converges to it from below, with a first-order deficit of <c>v dt / 2</c>.
    /// Stated in the type doc; checked here so the doc cannot drift from the arithmetic.
    /// </summary>
    [Fact]
    public void TheClosedFormBiasApproachesSpeedTimesTauAsTheFrameIntervalShrinks()
    {
        var fixture = new Fixture();
        float tau = fixture.Reconciler.TimeConstantSeconds;
        const float Speed = 0.5f;

        float continuous = Speed * tau;

        float coarse = fixture.Reconciler.SteadyStateLagMeters(Speed, 0.010f);
        float fine = fixture.Reconciler.SteadyStateLagMeters(Speed, 0.001f);
        float finer = fixture.Reconciler.SteadyStateLagMeters(Speed, 0.0001f);

        // Monotone increasing toward the continuous-time value, and always below it.
        Assert.True(coarse < fine && fine < finer && finer < continuous);

        // At a frame interval small against tau, the whole deficit is the first-order term v*dt/2 --
        // which is what makes "the display settles v*tau behind" the right summary of the bias.
        float deficit = continuous - finer;
        Assert.InRange(deficit / (Speed * 0.0001f / 2f), 0.98f, 1.02f);
    }

    /// <summary>
    /// When the motion stops, the bias decays away on the same exponential as any other static
    /// correction, and the reconciler reports convergence. The bias is permanent <i>while motion
    /// continues</i>, which is a different and weaker claim than "permanent", and the difference is
    /// worth pinning so nobody reads the tests above as saying the display never catches up at all.
    /// </summary>
    [Fact]
    public void TheBiasDecaysOnceTheOperatorStops()
    {
        const float Speed = 0.5f;

        var fixture = new Fixture();
        var samples = DriveConstantVelocity(fixture, frames: 101, Speed);
        float restingX = samples[^1].PredictedX;

        Assert.True(restingX - samples[^1].DisplayedX > PositionTolerance);

        // Hold the prediction still for four budgets.
        float finalError = 0f;
        for (int frame = 101; frame < 141; frame++)
        {
            Pose displayed = fixture.Reconciler.Reconcile(PoseAt(restingX), frame * FrameTicks);
            finalError = restingX - displayed.Position.X;
        }

        Assert.True(
            finalError <= PositionTolerance,
            $"once truth stops moving the lag must decay inside tolerance; was {finalError} m");
    }

    // ---- 4. Requirement 2: C0 yes, C1 no -- a quantified witness, not a claim ----

    /// <summary>
    /// <b>The C1 violation, measured.</b> When the prediction jumps by <c>d</c>, the displayed
    /// velocity steps from zero to <c>(1 - exp(-dt/tau)) d / dt</c> on the very next frame. There is
    /// no seeding, no hold and no ramp, so the step is immediate and its size is exactly the
    /// published closed form.
    ///
    /// This is a witness test in the style <c>snap</c>'s suite uses for its own deliberate
    /// discontinuity: it asserts the discontinuity exists and pins its magnitude, rather than
    /// asserting a continuity this law does not have. If this test ever started failing because the
    /// step was zero, the implementation would no longer be a first-order lag.
    /// </summary>
    [Fact]
    public void APredictorJumpStepsTheDisplayedVelocity()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 4);

        float frameSeconds = FrameTicks / (float)TicksPerSecond;
        float tau = fixture.Reconciler.TimeConstantSeconds;

        // Before the jump the display was static at x = 0, so its velocity was exactly zero.
        float velocityOnJumpFrame = (trajectory[0].Displayed.Position.X - 0f) / frameSeconds;
        float expected = fixture.Reconciler.VelocityStepAtPredictorJump(CorrectionMeters, frameSeconds);

        Assert.Equal(expected, velocityOnJumpFrame, 4);

        // The violation is real: strictly positive, so C1 does not hold.
        Assert.True(velocityOnJumpFrame > 0f);

        // And it is bounded by d/tau at any frame interval -- the structural difference from a
        // snap's d/dt, which has no such bound.
        Assert.True(
            velocityOnJumpFrame < CorrectionMeters / tau,
            $"velocity step {velocityOnJumpFrame} m/s must stay under d/tau = {CorrectionMeters / tau} m/s");
    }

    /// <summary>
    /// The bound above is the load-bearing part, so it is checked the way it would fail: by halving
    /// the frame interval. A position-discontinuous reconciler's velocity step doubles when the
    /// frame interval halves; this one grows by roughly 12% and converges on <c>d/tau</c>.
    ///
    /// Deliberately expressed as "the ratio stays well under 2" rather than as a tuned constant --
    /// the claim is that the step does not scale with <c>1/dt</c>, and that is what is asserted.
    /// </summary>
    [Fact]
    public void TheVelocityStepDoesNotScaleWithTheInverseFrameInterval()
    {
        var coarse = new Fixture();
        var fine = new Fixture();

        var coarseTrajectory = DriveStepCorrection(coarse, framesAfterCorrection: 2, frameTicks: 10);
        var fineTrajectory = DriveStepCorrection(fine, framesAfterCorrection: 2, frameTicks: 5);

        float coarseVelocity = coarseTrajectory[0].Displayed.Position.X / (10f / TicksPerSecond);
        float fineVelocity = fineTrajectory[0].Displayed.Position.X / (5f / TicksPerSecond);

        float ratio = fineVelocity / coarseVelocity;
        Assert.InRange(ratio, 1.0f, 1.3f);

        float tau = coarse.Reconciler.TimeConstantSeconds;
        Assert.True(fineVelocity < CorrectionMeters / tau);
    }

    /// <summary>
    /// <b>C0 holds.</b> The displayed position never steps: the frame-one displacement is a fraction
    /// of the correction, and halving the frame interval roughly halves it, so the displacement
    /// tends to zero with <c>dt</c>. That is the definition of a continuous position trajectory
    /// sampled on a frame clock, and it is the requirement-2 half this law does satisfy.
    /// </summary>
    [Fact]
    public void TheDisplayedPositionNeverStepsAndItsDisplacementShrinksWithTheFrameInterval()
    {
        var coarse = new Fixture();
        var fine = new Fixture();

        float coarseStep = DriveStepCorrection(coarse, 1, frameTicks: 10)[0].Displayed.Position.X;
        float fineStep = DriveStepCorrection(fine, 1, frameTicks: 5)[0].Displayed.Position.X;

        Assert.True(coarseStep < CorrectionMeters);
        Assert.True(fineStep < coarseStep);
        Assert.InRange(fineStep / coarseStep, 0.4f, 0.6f);
    }

    /// <summary>
    /// The displayed trajectory approaches truth monotonically and never overshoots it: a first-order
    /// lag has no oscillatory mode, so the display cannot travel past the pose it is chasing. That
    /// matters for the same reason it matters for a critically damped response -- overshoot reads to
    /// an operator as the robot wobbling after every packet.
    /// </summary>
    [Fact]
    public void TheApproachIsMonotonicAndNeverOvershoots()
    {
        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 40);

        float previous = 0f;
        foreach ((long _, Pose displayed) in trajectory)
        {
            float x = displayed.Position.X;
            Assert.True(x >= previous, "the approach must be monotonic");
            Assert.True(x <= CorrectionMeters, "a first-order lag must never overshoot its target");
            previous = x;
        }
    }

    // ---- 5. Orientation ----

    /// <summary>
    /// Orientation follows the same law along the geodesic, reusing
    /// <c>MotionMath.RelativeRotationVector</c> and <c>MotionMath.IntegrateWorld</c> rather than a
    /// second copy of the math. Checked against the same scalar envelope as position: the angle
    /// remaining decays as <c>angle0 exp(-t/tau)</c>.
    ///
    /// <b>The remaining angle is measured with <c>MotionMath.RelativeRotationVector</c> rather than
    /// with <c>PoseMath.OrientationErrorRadians</c>, and that is a deliberate choice about the
    /// instrument, not about the reconciler.</b> <c>OrientationErrorRadians</c> is
    /// <c>2*acos(|dot|)</c>, which loses roughly half the float mantissa as the angle approaches
    /// zero: it reads 0.0039 rad where the true remaining angle is 0.0040, an 2.3% error at that
    /// magnitude. <c>RelativeRotationVector</c> goes through <c>atan2</c>, which stays well
    /// conditioned there. Both measure the same quantity; only one can resolve it at 4 mrad. The
    /// tolerance-facing assertion below still uses <c>PoseMath</c>, because that is what the
    /// reconciler itself uses to decide convergence — this test is careful about which instrument
    /// answers which question rather than adjusting either of them.
    /// </summary>
    [Fact]
    public void OrientationFollowsTheSameExponentialAlongTheGeodesic()
    {
        const float AngleRadians = 0.4f;

        var fixture = new Fixture();
        var trajectory = DriveStepCorrection(
            fixture, framesAfterCorrection: 20, correction: 0f, correctionAngleRadians: AngleRadians);

        float tau = fixture.Reconciler.TimeConstantSeconds;
        float frameSeconds = FrameTicks / (float)TicksPerSecond;
        Pose target = PoseAt(0f, AngleRadians);

        for (int n = 0; n < trajectory.Count; n++)
        {
            float remaining = MotionMath
                .RelativeRotationVector(trajectory[n].Displayed.Rotation, target.Rotation)
                .Length();
            float expected = AngleRadians * MathF.Exp(-(n + 1) * frameSeconds / tau);

            // Relative, because the quantity spans four orders of magnitude across the trajectory.
            Assert.InRange(remaining / expected, 0.999f, 1.001f);
        }

        // And the reconciler's own convergence instrument agrees that it has landed.
        Assert.True(
            PoseMath.OrientationErrorRadians(trajectory[^1].Displayed, target) <= 1e-3f);
        Assert.True(fixture.Reconciler.IsConverged);
    }

    // ---- 6. Rate caps ----

    /// <summary>
    /// The linear rate cap bounds the displayed step to <c>maxSpeed * dt</c>. Note what the cap
    /// bounds here: this law has no separable correction, so the cap bounds <i>total displayed
    /// speed</i> rather than correction speed. That is documented on the type and is a direct
    /// consequence of having no offset state.
    /// </summary>
    [Fact]
    public void TheLinearRateCapBoundsTheDisplayedStep()
    {
        const float Cap = 0.5f;

        var fixture = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: Cap));
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 10);

        float frameSeconds = FrameTicks / (float)TicksPerSecond;
        float previous = 0f;

        foreach ((long _, Pose displayed) in trajectory)
        {
            float step = displayed.Position.X - previous;
            Assert.True(
                step <= Cap * frameSeconds + 1e-6f,
                $"step {step} m exceeded the cap budget {Cap * frameSeconds} m");
            previous = displayed.Position.X;
        }

        // The cap is doing something: uncapped, the first step would be larger.
        var uncapped = new Fixture();
        float uncappedStep = DriveStepCorrection(uncapped, 1)[0].Displayed.Position.X;
        Assert.True(uncappedStep > Cap * frameSeconds);
    }

    /// <summary>
    /// A rate cap slows convergence rather than preventing it -- the documented interaction, matching
    /// the rest of the axis. The capped response still lands inside tolerance, just later.
    /// </summary>
    [Fact]
    public void ARateCapSlowsConvergenceWithoutPreventingIt()
    {
        var fixture = new Fixture(Config(maxCorrectionLinearSpeedMetersPerSecond: 0.5f));
        var trajectory = DriveStepCorrection(fixture, framesAfterCorrection: 120);

        float finalError = CorrectionMeters - trajectory[^1].Displayed.Position.X;
        Assert.True(finalError <= PositionTolerance);
        Assert.True(fixture.Reconciler.IsConverged);
    }

    // ---- 7. Observe bookkeeping: identical to the rest of the axis by construction ----

    /// <summary>
    /// A disagreement beyond tolerance emits exactly one <c>correction_magnitude_mm</c> and one
    /// <c>correction_magnitude_deg</c>, stamped at the sample's own capture tick, in millimetres and
    /// degrees. Identical definition, stamp and cadence to the rest of the axis -- these two are a
    /// control across reconcilers, never a differentiator, and this test exists to keep it that way.
    /// </summary>
    [Fact]
    public void ObserveEmitsCorrectionMagnitudeOnceAtTheSamplesCaptureTick()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, framesAfterCorrection: 1);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));

        Assert.True(fixture.Metrics.TryGetLatest(CorrectionMagnitudeMm, out double mm, out long ticks));
        Assert.Equal(CorrectionMeters * 1000.0, mm, 3);
        Assert.Equal(2 * FrameTicks, ticks);
    }

    /// <summary>
    /// A duplicate or stale sample is ignored whole: no second correction is counted, because
    /// correction rate is derived by counting these samples and double-counting corrupts it.
    /// </summary>
    [Fact]
    public void ObserveIgnoresDuplicateAndStaleSamples()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, framesAfterCorrection: 1);

        // Same stamp again (duplicate), then an older one (stale).
        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);
        fixture.Reconciler.Observe(
            new Stamped<Pose>(FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeDeg));
    }

    /// <summary>
    /// A sample that agrees within tolerance is no correction at all, not a zero-magnitude one: no
    /// metric is emitted and <c>IsConverged</c> never goes false. This is the degenerate
    /// already-agreeing case the whole axis documents.
    /// </summary>
    [Fact]
    public void AnAgreeingSampleIsNotACorrection()
    {
        var fixture = new Fixture();
        fixture.Reconciler.Reconcile(PoseAt(0f), 0);

        fixture.Reconciler.Observe(
            new Stamped<Pose>(FrameTicks, PoseAt(PositionTolerance / 2f)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeDeg));
        Assert.True(fixture.Reconciler.IsConverged);
    }

    /// <summary>
    /// Requirement 5, graceful degradation: this reconciler ignores predictor uncertainty entirely,
    /// so diagnostics carrying a covariance produce a bit-identical trajectory to
    /// <c>PredictorDiagnostics.None</c>. Reached by not depending on the field at all, which is the
    /// strongest form of "degrades gracefully when the predictor supplies none".
    /// </summary>
    [Fact]
    public void PredictorUncertaintyIsIgnoredEntirely()
    {
        var withNone = new Fixture();
        var withSigma = new Fixture();

        var uncertain = new PredictorDiagnostics(
            horizonTicks: 50,
            lastObservationTicks: 20,
            acceptedObservations: 7,
            rejectedObservations: 1,
            hasUncertainty: true,
            positionSigmaMeters: 0.25f,
            orientationSigmaRadians: 0.1f);

        for (int frame = 0; frame < 3; frame++)
        {
            withNone.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
            withSigma.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        withNone.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);
        withSigma.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            uncertain);

        for (int frame = 3; frame < 30; frame++)
        {
            Pose a = withNone.Reconciler.Reconcile(PoseAt(CorrectionMeters), frame * FrameTicks);
            Pose b = withSigma.Reconciler.Reconcile(PoseAt(CorrectionMeters), frame * FrameTicks);
            AssertSamePose(a, b);
        }
    }

    // ---- 8. Reconcile: cadence, idempotence, pass-through ----

    /// <summary>
    /// <c>jerk_mm_s3</c> is emitted on <b>every</b> advancing frame once four displayed positions
    /// exist -- never per correction. Same shared <c>DisplayedJerkEstimator</c> and same cadence as
    /// the rest of the axis, which is the only thing that makes pooled jerk percentiles populations
    /// of the same quantity.
    /// </summary>
    [Fact]
    public void JerkIsEmittedOnEveryAdvancingFrameOnceFourPositionsExist()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, framesAfterCorrection: 20);

        // 23 frames total; the first three fill the history, so 20 emissions.
        Assert.Equal(20, fixture.CountOf(JerkMmS3));
    }

    /// <summary>
    /// Two calls with the same <c>nowTicks</c> return the same state, do not advance the lag, and do
    /// not emit a second jerk sample. The same guard covers a frame delivered out of order.
    /// </summary>
    [Fact]
    public void ReconcileIsIdempotentInNowTicks()
    {
        var fixture = new Fixture();
        DriveStepCorrection(fixture, framesAfterCorrection: 6);

        // DriveStepCorrection's last advancing frame was index 8, so index 9 is the next new one.
        int jerkBefore = fixture.CountOf(JerkMmS3);
        Pose first = fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 9 * FrameTicks);
        Pose repeat = fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 9 * FrameTicks);
        Pose older = fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), 4 * FrameTicks);

        AssertSamePose(first, repeat);
        AssertSamePose(first, older);
        Assert.Equal(jerkBefore + 1, fixture.CountOf(JerkMmS3));
    }

    /// <summary>
    /// The first frame of a trial adopts the prediction bit-identically -- there is no previously
    /// displayed pose to lag behind, and starting at <c>Pose.Identity</c> would invent an opening
    /// correction. A constant prediction then passes through bit-identically on every later frame,
    /// which is the degenerate already-agreeing case.
    ///
    /// <b>Note the scope of this claim.</b> It holds only while the prediction is constant. A
    /// <i>moving</i> prediction never passes through, because the display lags it — that is the
    /// candidate's defining bias, not an exception to this test.
    /// </summary>
    [Fact]
    public void AConstantPredictionPassesThroughBitIdentically()
    {
        var fixture = new Fixture();
        Pose predicted = PoseAt(1.3f, 0.2f);

        for (int frame = 0; frame < 10; frame++)
        {
            AssertSamePose(predicted, fixture.Reconciler.Reconcile(predicted, frame * FrameTicks));
        }

        Assert.True(fixture.Reconciler.IsConverged);
        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeMm));
    }

    /// <summary>
    /// <b>The survivorship warning, pinned as a test.</b> While the operator keeps moving, an open
    /// convergence episode cannot close, because the steady-state bias never falls inside tolerance.
    /// So this reconciler emits <b>no</b> <c>time_to_convergence_ms</c> sample for the whole motion,
    /// where an offset-carrying law would emit one per correction.
    ///
    /// The definition of the metric is unchanged (docs/metrics.md §5) -- what differs is which
    /// episodes ever reach the terminating condition. Anyone pooling
    /// <c>time_to_convergence_ms</c> percentiles across this axis must know that a small, fast-looking
    /// sample here is survivorship rather than speed. This test exists so that fact is discovered by
    /// reading the suite rather than by misreading a sweep.
    /// </summary>
    [Fact]
    public void NoConvergenceIsReportedWhileTheOperatorKeepsMoving()
    {
        const float Speed = 0.5f;

        var fixture = new Fixture();

        // Three agreeing frames, then a qualifying sample opens an episode, then continuous motion.
        for (int frame = 0; frame < 3; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(0f), frame * FrameTicks);
        }

        fixture.Reconciler.Observe(
            new Stamped<Pose>(2 * FrameTicks, PoseAt(CorrectionMeters)),
            PoseAt(0f),
            PredictorDiagnostics.None);

        float frameSeconds = FrameTicks / (float)TicksPerSecond;
        for (int frame = 3; frame < 300; frame++)
        {
            fixture.Reconciler.Reconcile(PoseAt(Speed * frame * frameSeconds), frame * FrameTicks);
        }

        Assert.Equal(0, fixture.CountOf(TimeToConvergenceMs));
        Assert.False(fixture.Reconciler.IsConverged);

        // The correction magnitude sample was still emitted, so the two metric families genuinely
        // disagree about how many events happened. That asymmetry is the finding.
        Assert.Equal(1, fixture.CountOf(CorrectionMagnitudeMm));
    }

    /// <summary>
    /// <b>A bug this suite caught, pinned so it cannot come back.</b> <c>IsConverged</c> is a
    /// statement about the <i>display</i>, not about whether a correction episode is open. The
    /// first version of this class defined it as the offset family does -- "nothing pending, nothing
    /// in flight" -- and it therefore reported <c>true</c> while the display sat a steady 8.5 mm
    /// behind truth, simply because no authoritative sample had happened to disagree beyond
    /// tolerance. For an offset-carrying law that definition is sound (no correction in flight
    /// implies zero residual implies the display is the prediction); for this law the implication is
    /// false, and the flag was quietly overstating the reconciler.
    ///
    /// Here there is no <c>Observe</c> at all, so no episode ever opens, and the flag must still
    /// report the tracking bias.
    /// </summary>
    [Fact]
    public void IsConvergedIsAboutTheDisplayNotAboutACorrectionEpisode()
    {
        var fixture = new Fixture();
        var samples = DriveConstantVelocity(fixture, frames: 101, speedMetersPerSecond: 0.5f);

        Assert.Equal(0, fixture.CountOf(CorrectionMagnitudeMm));
        Assert.True(samples[^1].PredictedX - samples[^1].DisplayedX > PositionTolerance);
        Assert.False(fixture.Reconciler.IsConverged);
    }

    // ---- 9. Registry ----

    /// <summary>
    /// The registry key resolves and constructs a working instance. Invariant 5: static
    /// registration, hand-written, no reflection -- so the entry is only real if something
    /// exercises it.
    /// </summary>
    [Fact]
    public void RegistryKeyResolvesAndConstructsAWorkingInstance()
    {
        Assert.True(Teleop.Core.Registry.Registries.Reconcilers.TryGetValue("exp-smooth-track", out var factory));

        var metrics = new InMemoryMetricTracker(64);
        var reconciler = factory!(Config(), metrics, new ManualClock(TicksPerSecond));

        Assert.IsType<TrackingLagReconciler>(reconciler);
        Assert.True(reconciler.IsConverged);
        AssertSamePose(PoseAt(0.5f), reconciler.Reconcile(PoseAt(0.5f), 0));
    }

    // ---- 10. Reset, determinism, allocation ----

    /// <summary>
    /// <c>Reset</c> returns the reconciler to its as-constructed state, including the
    /// accepted-capture baseline: a reconciler that remembered the previous trial's highest capture
    /// stamp would silently ignore the opening stretch of the next one. Proved by driving a reset
    /// instance and a fresh instance identically and comparing both the trajectories and the metric
    /// streams.
    /// </summary>
    [Fact]
    public void ResetReturnsTheReconcilerToItsAsConstructedState()
    {
        var reused = new Fixture();
        DriveStepCorrection(reused, framesAfterCorrection: 30, correction: 0.2f);
        reused.Reconciler.Reset();
        reused.Metrics.Reset();

        Assert.True(reused.Reconciler.IsConverged);

        var fresh = new Fixture();

        var reusedTrajectory = DriveStepCorrection(reused, framesAfterCorrection: 15);
        var freshTrajectory = DriveStepCorrection(fresh, framesAfterCorrection: 15);

        for (int i = 0; i < freshTrajectory.Count; i++)
        {
            AssertSamePose(freshTrajectory[i].Displayed, reusedTrajectory[i].Displayed);
        }

        Assert.Equal(fresh.Metrics.Count, reused.Metrics.Count);
        for (int i = 0; i < fresh.Metrics.Count; i++)
        {
            Assert.Equal(fresh.Metrics[i].Name, reused.Metrics[i].Name);
            Assert.Equal(fresh.Metrics[i].Value, reused.Metrics[i].Value);
            Assert.Equal(fresh.Metrics[i].Ticks, reused.Metrics[i].Ticks);
        }
    }

    /// <summary>
    /// Two instances given identical input produce bit-identical output. Nothing here reads a clock
    /// or a random number, so this is a statement about the arithmetic rather than about the
    /// harness -- and it is what makes a replayed sweep reproducible.
    /// </summary>
    [Fact]
    public void TwoInstancesGivenIdenticalInputAgreeBitForBit()
    {
        var a = new Fixture();
        var b = new Fixture();

        var first = DriveStepCorrection(a, framesAfterCorrection: 40, correction: 0.31f);
        var second = DriveStepCorrection(b, framesAfterCorrection: 40, correction: 0.31f);

        for (int i = 0; i < first.Count; i++)
        {
            AssertSamePose(first[i].Displayed, second[i].Displayed);
        }
    }

    /// <summary>
    /// Invariant 8: the per-frame path allocates nothing. <c>Reconcile</c> is called every frame and
    /// <c>Observe</c> on every arriving sample, so both are hot.
    /// </summary>
    [Fact]
    public void ReconcileAllocatesNothing()
    {
        var fixture = new Fixture(Config());
        long ticks = 0;

        AllocationAssert.Zero(() =>
        {
            ticks += FrameTicks;
            fixture.Reconciler.Reconcile(PoseAt(CorrectionMeters), ticks);
        });
    }

    [Fact]
    public void ObserveAllocatesNothing()
    {
        var fixture = new Fixture(Config());
        long ticks = 0;

        AllocationAssert.Zero(() =>
        {
            ticks += FrameTicks;
            fixture.Reconciler.Observe(
                new Stamped<Pose>(ticks, PoseAt(CorrectionMeters)),
                PoseAt(0f),
                PredictorDiagnostics.None);
        });
    }
}
