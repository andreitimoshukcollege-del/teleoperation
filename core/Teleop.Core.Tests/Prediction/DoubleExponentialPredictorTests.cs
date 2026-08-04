using System.Numerics;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class DoubleExponentialPredictorTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    private const float Tolerance = 1e-4f;

    /// <summary>Sample spacing of the synthetic trajectory, 50 Hz.</summary>
    private const long SampleIntervalTicks = 20;

    private static ManualClock NewClock() => new ManualClock(TicksPerSecond);

    private static PredictorConfig Config(
        float smoothingAlpha = 0.5f,
        float smoothingBeta = 0.5f,
        long maxHorizonTicks = 400,
        long maxObservationGapTicks = 100,
        float maxLinearSpeed = 100f,
        float maxAngularSpeed = 100f) =>
        new PredictorConfig(
            maxHorizonTicks,
            maxObservationGapTicks,
            // The three fields double-exp documents that it ignores, set to values that would be
            // catastrophic if it ever started reading them (a capacity of zero would throw in any
            // predictor that allocated a buffer from it).
            historyCapacity: 0,
            smoothingAlpha,
            smoothingBeta,
            processNoise: -1f,
            measurementNoise: -1f,
            maxLinearSpeed,
            maxAngularSpeed);

    private static DoubleExponentialPredictor NewPredictor(PredictorConfig? config = null) =>
        new DoubleExponentialPredictor(config ?? Config(), NewClock());

    // --- A synthetic trajectory with exactly constant linear and angular velocity. -------------

    private static readonly Vector3 TrajectoryOrigin = new Vector3(0.1f, -0.2f, 0.3f);
    private static readonly Vector3 TrajectoryVelocity = new Vector3(0.5f, -1.25f, 2f);

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

    private static void AssertSameDiagnostics(PredictorDiagnostics expected, PredictorDiagnostics actual)
    {
        Assert.Equal(expected.HorizonTicks, actual.HorizonTicks);
        Assert.Equal(expected.LastObservationTicks, actual.LastObservationTicks);
        Assert.Equal(expected.AcceptedObservations, actual.AcceptedObservations);
        Assert.Equal(expected.RejectedObservations, actual.RejectedObservations);
        Assert.Equal(expected.HasUncertainty, actual.HasUncertainty);
    }

    // 1. Fail fast on smoothing factors outside [0, 1].
    [Theory]
    [InlineData(-0.001f)]
    [InlineData(1.001f)]
    [InlineData(2f)]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    public void Constructor_RejectsAlphaOutsideTheUnitInterval(float alpha)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(smoothingAlpha: alpha), NewClock()));
    }

    [Theory]
    [InlineData(-0.001f)]
    [InlineData(1.001f)]
    [InlineData(2f)]
    [InlineData(float.NaN)]
    [InlineData(float.NegativeInfinity)]
    public void Constructor_RejectsBetaOutsideTheUnitInterval(float beta)
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(smoothingBeta: beta), NewClock()));
    }

    [Theory]
    [InlineData(0f)]
    [InlineData(1f)]
    [InlineData(0.5f)]
    public void Constructor_AcceptsTheClosedUnitInterval(float value)
    {
        _ = new DoubleExponentialPredictor(
            Config(smoothingAlpha: value, smoothingBeta: value), NewClock());
    }

    [Fact]
    public void Constructor_RejectsInvalidBoundsAndClocks()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(maxHorizonTicks: -1), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(maxObservationGapTicks: 0), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(maxLinearSpeed: -1f), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(maxAngularSpeed: -1f), NewClock()));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => new DoubleExponentialPredictor(Config(), new ManualClock(0)));
        Assert.Throws<ArgumentNullException>(
            () => new DoubleExponentialPredictor(Config(), null!));
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
    public void Predict_AfterTheSeedingObservation_HoldsThatPoseWithNoTrend()
    {
        var predictor = NewPredictor();
        Stamped<Pose> seed = TrajectorySample(100);
        predictor.Observe(seed);

        // The seed sets level to the observation and trend to zero: nothing is known about motion
        // from one sample, and inventing a trend is exactly what this filter must not do.
        AssertSamePose(seed.Value, predictor.Predict(500));
    }

    // 3. Determinism.
    [Fact]
    public void IdenticalCallSequences_ProduceBitIdenticalOutput()
    {
        var a = NewPredictor();
        var b = NewPredictor();

        for (int i = 1; i <= 200; i++)
        {
            // Accepted, stale, duplicate and gap-crossing observations in one sequence.
            long captureTicks = i % 7 == 0 ? i - 5 : (i % 11 == 0 ? i - 1 : i * 20);
            var obs = new Stamped<Pose>(
                captureTicks,
                new Pose(
                    new Vector3(i * 0.01f, i * -0.02f, i * 0.03f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitY, i * 0.017f)));

            a.Observe(obs);
            b.Observe(obs);

            long target = i * 20 + 37;
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

    // 4. Convergence: the lag is a documented tradeoff, so this asserts convergence *within N
    //    samples*, not from sample one.
    [Fact]
    public void Predict_OnAConstantVelocityTrajectory_ConvergesWithinABoundedNumberOfSamples()
    {
        // 5 mm at a 100 ms horizon on a trajectory moving 2.41 m/s, i.e. within 2% of the 241 mm
        // being extrapolated. Not an arbitrary number: it is the scale at which the remaining
        // filter transient is smaller than the correction cost of the reconciler that follows.
        const float convergedWithinMeters = 0.005f;
        const long horizonTicks = 100;
        const int sampleBudget = 15;
        const int totalSamples = 60;

        var predictor = NewPredictor(Config(smoothingAlpha: 0.5f, smoothingBeta: 0.5f));

        int convergedAtSample = -1;
        float firstSampleError = 0f;
        float worstErrorAfterConvergence = 0f;

        for (int k = 0; k < totalSamples; k++)
        {
            long captureTicks = k * SampleIntervalTicks;
            predictor.Observe(TrajectorySample(captureTicks));

            Pose predicted = predictor.Predict(captureTicks + horizonTicks);
            float error = PoseMath.PositionErrorMeters(
                TrajectoryAt(captureTicks + horizonTicks), predicted);

            if (k == 1)
            {
                firstSampleError = error;
            }

            if (convergedAtSample < 0)
            {
                if (error < convergedWithinMeters)
                {
                    convergedAtSample = k;
                }
            }
            else
            {
                worstErrorAfterConvergence = MathF.Max(worstErrorAfterConvergence, error);
            }
        }

        Assert.True(
            convergedAtSample >= 0 && convergedAtSample <= sampleBudget,
            $"expected convergence within {sampleBudget} samples, got {convergedAtSample}");

        // ...and it stays converged rather than crossing the threshold once on the way past.
        Assert.True(
            worstErrorAfterConvergence < convergedWithinMeters,
            $"worst error after convergence was {worstErrorAfterConvergence} m");

        // The lag is real and is the tradeoff being bought: early predictions are far worse than
        // the converged ones. If this ever stopped holding, the filter would not be smoothing.
        Assert.True(
            firstSampleError > convergedWithinMeters * 10f,
            $"expected a visible startup transient, got {firstSampleError} m at sample 1");
    }

    [Fact]
    public void Predict_OnAConstantAngularRateTrajectory_ConvergesInOrientationToo()
    {
        const float convergedWithinRadians = 0.005f;
        const long horizonTicks = 100;

        var predictor = NewPredictor();
        for (int k = 0; k < 60; k++)
        {
            predictor.Observe(TrajectorySample(k * SampleIntervalTicks));
        }

        long newestTicks = 59 * SampleIntervalTicks;
        Pose predicted = predictor.Predict(newestTicks + horizonTicks);

        Assert.Equal(
            0f,
            PoseMath.OrientationErrorRadians(TrajectoryAt(newestTicks + horizonTicks), predicted),
            convergedWithinRadians);
    }

    // 5. The degenerate parameterization, as an implementation cross-check.
    [Fact]
    public void Predict_WithAlphaOneAndBetaZero_TracksTheLastObservationWithZeroTrend()
    {
        var predictor = NewPredictor(Config(smoothingAlpha: 1f, smoothingBeta: 0f));

        Stamped<Pose> newest = default;
        for (int k = 0; k < 10; k++)
        {
            newest = TrajectorySample(k * SampleIntervalTicks);
            predictor.Observe(newest);
        }

        // alpha = 1 puts the level exactly on the observation; beta = 0 leaves the trend at its
        // seeded zero forever. The filter therefore degenerates to `none` -- which is precisely the
        // cross-check: two independent implementations must agree at the shared corner.
        Assert.Equal(newest.Value.Position, predictor.Predict(newest.CaptureTicks).Position);
        Assert.Equal(newest.Value.Position, predictor.Predict(newest.CaptureTicks + 400).Position);
        Assert.Equal(
            0f,
            PoseMath.OrientationErrorRadians(newest.Value, predictor.Predict(newest.CaptureTicks + 400)),
            Tolerance);
    }

    [Fact]
    public void Predict_WithAlphaZero_IgnoresObservationsAfterTheSeed()
    {
        var predictor = NewPredictor(Config(smoothingAlpha: 0f, smoothingBeta: 0f));
        Stamped<Pose> seed = TrajectorySample(0);
        predictor.Observe(seed);
        for (int k = 1; k < 10; k++)
        {
            predictor.Observe(TrajectorySample(k * SampleIntervalTicks));
        }

        // alpha = 0 means "never fold in an observation"; beta = 0 means "never learn a trend". The
        // level therefore never leaves the seed. Degenerate but well-defined, and it must not
        // diverge or produce NaN.
        Pose predicted = predictor.Predict(1_000);
        Assert.Equal(seed.Value.Position.X, predicted.Position.X, Tolerance);
        Assert.Equal(seed.Value.Position.Y, predicted.Position.Y, Tolerance);
        Assert.Equal(seed.Value.Position.Z, predicted.Position.Z, Tolerance);
    }

    // 6. Ordering: reject, never reinsert.
    [Fact]
    public void Observe_OutOfOrder_IsRejectedAndCounted_AndChangesNothing()
    {
        var predictor = NewPredictor();
        predictor.Observe(TrajectorySample(0));
        predictor.Observe(TrajectorySample(20));
        predictor.Observe(TrajectorySample(40));

        Pose before = predictor.Predict(140);

        // 30 falls between two already-folded samples. const-vel would splice it into its window;
        // this filter has no window to splice into, so it rejects rather than folding a sample in
        // with a backwards dt.
        predictor.Observe(TrajectorySample(30));

        AssertSamePose(before, predictor.Predict(140));
        Assert.Equal(3, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(1, predictor.Diagnostics.RejectedObservations);
        Assert.Equal(40, predictor.Diagnostics.LastObservationTicks);
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

        AssertSamePose(once.Predict(120), twice.Predict(120));
        Assert.Equal(2, twice.Diagnostics.AcceptedObservations);
        Assert.Equal(1, twice.Diagnostics.RejectedObservations);
    }

    // 7. Gap policy: full re-seed.
    [Fact]
    public void Observe_AcrossAMultiHundredMillisecondGap_ReSeedsInsteadOfSmoothingAcrossIt()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 100));
        for (int k = 0; k < 20; k++)
        {
            predictor.Observe(TrajectorySample(k * SampleIntervalTicks));
        }

        // Precondition: a real trend has been learned by now.
        long lastTicks = 19 * SampleIntervalTicks;
        Assert.True(
            PoseMath.PositionErrorMeters(TrajectoryAt(lastTicks), predictor.Predict(lastTicks + 100)) > 0.1f,
            "precondition: the filter should be extrapolating before the gap");

        // 600 ms with nothing arriving. A re-seed puts the level on the arriving sample and the
        // trend back to zero, so the prediction holds that pose at every horizon -- rather than
        // smoothing a 600 ms-old level into it and stranding the output between two positions the
        // operator was never at.
        Stamped<Pose> afterGap = TrajectorySample(lastTicks + 600);
        predictor.Observe(afterGap);

        AssertSamePose(afterGap.Value, predictor.Predict(afterGap.CaptureTicks));
        AssertSamePose(afterGap.Value, predictor.Predict(afterGap.CaptureTicks + 400));
        Assert.Equal(21, predictor.Diagnostics.AcceptedObservations);
        Assert.Equal(0, predictor.Diagnostics.RejectedObservations);
    }

    [Fact]
    public void Observe_AtExactlyTheGapBound_IsStillSmoothed()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 100, smoothingAlpha: 1f, smoothingBeta: 1f));
        predictor.Observe(Sample(0, Vector3.Zero));
        predictor.Observe(Sample(100, new Vector3(0.1f, 0f, 0f))); // gap == bound, not "exceeds"

        // alpha = beta = 1 makes the trend exactly the last first difference: 1 m/s.
        Assert.Equal(0.2f, predictor.Predict(200).Position.X, Tolerance);
    }

    // 8. Clamps.
    [Fact]
    public void Predict_ClampsTheHorizonToMaxHorizonTicks_OnTheFutureSideOnly()
    {
        var predictor = NewPredictor(Config(
            maxHorizonTicks: 100, smoothingAlpha: 1f, smoothingBeta: 1f, maxObservationGapTicks: 1_000));
        predictor.Observe(Sample(0, Vector3.Zero));
        predictor.Observe(Sample(100, new Vector3(0.1f, 0f, 0f))); // trend = 1 m/s

        predictor.Predict(500);
        Assert.Equal(100, predictor.Diagnostics.HorizonTicks);

        predictor.Predict(0);
        Assert.Equal(-100, predictor.Diagnostics.HorizonTicks);
        Assert.Equal(0f, predictor.Predict(0).Position.X, Tolerance);
    }

    [Fact]
    public void Predict_ClampsLinearSpeedBeforeExtrapolating()
    {
        var predictor = NewPredictor(Config(
            smoothingAlpha: 1f, smoothingBeta: 1f, maxLinearSpeed: 1f, maxAngularSpeed: 100f));
        // 0.5 m in 10 ms is 50 m/s.
        predictor.Observe(Sample(0, Vector3.Zero));
        predictor.Observe(Sample(10, new Vector3(0.5f, 0f, 0f)));

        Pose predicted = predictor.Predict(110);

        Assert.Equal(0.6f, predicted.Position.X, Tolerance);
    }

    [Fact]
    public void Predict_ClampsAngularSpeedBeforeExtrapolating()
    {
        var predictor = NewPredictor(Config(
            smoothingAlpha: 1f, smoothingBeta: 1f, maxLinearSpeed: 100f, maxAngularSpeed: 1f));
        predictor.Observe(Sample(0, Vector3.Zero, Quaternion.Identity));
        predictor.Observe(Sample(10, Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 1f)));

        Pose predicted = predictor.Predict(110);

        var expected = new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, 1.1f));
        Assert.Equal(0f, PoseMath.OrientationErrorRadians(expected, predicted), 1e-3f);
    }

    // Regression tests for the divergence a Phase-5 sweep actually recorded: several observations
    // arriving with very small gaps between them (datagrams draining back-to-back right after a
    // trace-driven network burst clears) turned an ordinary position delta into a huge apparent
    // instantaneous speed, which -- before Observe() clamped it -- compounded across consecutive
    // calls into float.PositiveInfinity/NaN. These assert the *stored* trend is bounded going
    // forward, not just Predict()'s output for one call.
    // Extracted directly from a real Teleop.Eval sweep trial (double-exp predictor, the
    // "synthetic-burst" trace-driven network profile) that diverged to float.PositiveInfinity
    // before this fix: (dt in seconds, observed position.X) pairs, in order, captured from the
    // actual failing run. Several dt values are genuinely microsecond-scale (as small as 2E-07s)
    // -- several datagrams draining back-to-back once a congestion burst clears -- which is far
    // smaller than a synthetic worst-case guess would likely have picked, and is exactly what
    // makes this a faithful reproduction rather than an approximation.
    private static readonly (double DtSeconds, float PositionX)[] BurstReproSequence =
    {
        (0.01, 0.0049999165f), (0.01, 0.009999333f), (0.01, 0.019994667f), (0.02, 0.029982002f),
        (0.02, 0.034971423f), (0.01, 0.044939276f), (0.01, 0.049916707f), (0.02, 0.059856102f),
        (0.02, 0.06977156f), (0.02, 0.079659104f), (0.02, 0.08459117f), (0.01, 0.09442945f),
        (0.02, 0.099334665f), (0.01, 0.10422995f), (0.01, 0.10911481f), (0.01, 0.113988765f),
        (0.01, 0.11885131f), (0.01, 0.12370198f), (0.01, 0.12854028f), (0.01, 0.13336572f),
        (0.01, 0.13817783f), (0.01, 0.1429761f), (0.01, 0.15252931f), (0.01, 0.15728328f),
        (0.01, 0.16202152f), (0.01, 0.16674355f), (0.02, 0.17613712f), (0.2200075, 0.27401197f),
        (0.01, 0.2781805f), (0.0001231, 0.2781805f), (0.02, 0.28232124f), (0.0099485, 0.29051757f),
        (3.77E-05, 0.29051757f), (0.0099768, 0.29457238f), (0.01, 0.29457238f),
        (0.0098523, 0.29859772f), (0.01, 0.3025932f), (0.0098851, 0.30655843f),
        (0.0102078, 0.3143965f), (0.0001204, 0.3143965f), (0.0099311, 0.3182686f),
        (0.0198607, 0.32210883f), (0.0098819, 0.3259169f), (0.0100335, 0.32969233f),
        (0.0100268, 0.33343482f), (0.0100214, 0.33714396f), (0.0100171, 0.3408194f),
        (0.0100137, 0.34446073f), (0.010011, 0.3480676f), (0.0100088, 0.35163972f),
        (0.010007, 0.35517663f), (0.0100056, 0.35867804f), (0.0100045, 0.36557293f),
        (3.6E-06, 0.36557293f), (0.0100029, 0.3689657f), (0.0100023, 0.37232155f),
        (0.0200018, 0.37892127f), (1.5E-06, 0.37892127f), (0.0100012, 0.38216448f),
        (0.0100009, 0.38536945f), (0.0100007, 0.3885359f), (0.0100006, 0.39166346f),
        (0.0200005, 0.3978008f), (4E-07, 0.3978008f), (0.0200003, 0.40377906f),
        (2E-07, 0.40377906f), (0.0100002, 0.40670776f), (0.0100002, 0.4095958f),
        (0.0100001, 0.41244286f), (0.0100001, 0.4152487f), (0.0200001, 0.41801298f),
        (0.0100001, 0.42341593f), (0.01, 0.426054f), (0.01, 0.4286495f), (0.02, 0.43371162f),
        (0.01, 0.43617773f), (0.02, 0.4409789f), (0.02, 0.44331345f), (0.01, 0.44560367f),
        (0.01, 0.44784933f),
    };

    [Fact]
    public void Observe_RealBurstDrainSequence_KeepsPositionTrendFiniteAndBounded()
    {
        // Same smoothing values as the sweep config this was captured from: low alpha*beta means
        // the trend recursion decays slowly (~3% per step here), so a burst of oversized forcing
        // terms from tiny-dt observations accumulates across many steps rather than dying out
        // immediately -- which is what let it reach float.PositiveInfinity/NaN in the real run.
        var predictor = new DoubleExponentialPredictor(
            Config(
                smoothingAlpha: 0.3f, smoothingBeta: 0.1f, maxLinearSpeed: 10f, maxAngularSpeed: 10f,
                maxObservationGapTicks: 1_000_000_000),
            new ManualClock(TicksPerSecondForBurstRepro));

        long ticks = 0;
        predictor.Observe(Sample(ticks, Vector3.Zero));

        foreach ((double dtSeconds, float positionX) in BurstReproSequence)
        {
            ticks += Math.Max(1, (long)Math.Round(dtSeconds * TicksPerSecondForBurstRepro));
            predictor.Observe(Sample(ticks, new Vector3(positionX, 0f, 0f)));

            Pose predicted = predictor.Predict(ticks + 50);
            Assert.True(float.IsFinite(predicted.Position.X), $"diverged to {predicted.Position.X}");

            // The real observed trajectory never exceeds 0.5m; this is a generous bound that the
            // pre-fix code blew past by ten orders of magnitude on this exact sequence.
            Assert.True(
                Math.Abs(predicted.Position.X) < 1000f,
                $"predicted X {predicted.Position.X} implausibly far from the observed trajectory");
        }
    }

    // High resolution (matches the real sweep's 10,000,000 ticks/sec), not this file's usual 1000
    // -- the captured microsecond-scale dt values need sub-millisecond tick resolution to replay
    // faithfully; a 1000-ticks/sec clock cannot represent a gap smaller than 1 ms at all.
    private const long TicksPerSecondForBurstRepro = 10_000_000;

    // Defense-in-depth, not a captured reproduction: no real run was observed diverging on the
    // rotation side specifically (the actual sweep failure was in prediction_position_error_mm).
    // The rotation recursion has the identical mathematical shape as the position one (substitute
    // MotionMath.RelativeRotationVector for subtraction), so it is clamped for the same reason by
    // symmetry, and this checks that clamp does not break normal unit-quaternion output -- it is
    // not, on its own, evidence that an unclamped rotation trend was ever observed running away.
    [Fact]
    public void Observe_ConsecutiveVerySmallDt_KeepsRotationTrendFiniteAndBounded()
    {
        const float maxAngularSpeed = 5f;
        var predictor = NewPredictor(Config(
            smoothingAlpha: 0.3f, smoothingBeta: 0.1f, maxLinearSpeed: 100f,
            maxAngularSpeed: maxAngularSpeed, maxObservationGapTicks: 1_000_000));

        predictor.Observe(Sample(0, Vector3.Zero, Quaternion.Identity));

        long ticks = 0;
        for (int i = 1; i <= 500; i++)
        {
            ticks += 1;
            Quaternion rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, i * 0.01f);
            predictor.Observe(Sample(ticks, Vector3.Zero, rotation));

            Pose predicted = predictor.Predict(ticks + 50);
            Assert.True(
                float.IsFinite(predicted.Rotation.X) && float.IsFinite(predicted.Rotation.Y) &&
                float.IsFinite(predicted.Rotation.Z) && float.IsFinite(predicted.Rotation.W),
                $"step {i}: rotation diverged to {predicted.Rotation}");
            Assert.True(
                Math.Abs(predicted.Rotation.Length() - 1f) < 0.01f,
                $"step {i}: predicted rotation is not a unit quaternion: {predicted.Rotation}");
        }
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

    // 9. Reset.
    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var predictor = NewPredictor();
        for (int k = 0; k < 20; k++)
        {
            predictor.Observe(TrajectorySample(k * SampleIntervalTicks));
        }

        predictor.Observe(TrajectorySample(1)); // rejected, bumping the counter
        predictor.Predict(1_000);

        predictor.Reset();

        AssertSameDiagnostics(PredictorDiagnostics.None, predictor.Diagnostics);

        Pose predicted = predictor.Predict(1_000);
        Assert.Equal(Vector3.Zero, predicted.Position);
        Assert.Equal(Quaternion.Identity, predicted.Rotation);

        // The staleness baseline cleared with it: the next trial's first sample re-seeds at a stamp
        // far below the 380 the previous trial reached, and no residual trend leaks across.
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
            long captureTicks = i % 4 == 0 ? i - 2 : i * 20;
            var obs = new Stamped<Pose>(
                captureTicks,
                new Pose(
                    new Vector3(i * 0.05f, 0f, i * -0.01f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, i * 0.02f)));
            reused.Observe(obs);
            fresh.Observe(obs);

            AssertSamePose(fresh.Predict(i * 20 + 25), reused.Predict(i * 20 + 25));
            AssertSameDiagnostics(fresh.Diagnostics, reused.Diagnostics);
        }
    }

    // 10. Allocation-free hot path.
    [Fact]
    public void Observe_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks += 20;
            predictor.Observe(new Stamped<Pose>(
                captureTicks,
                new Pose(
                    new Vector3(captureTicks * 0.001f, 1f, 2f),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitZ, captureTicks * 0.0001f))));
        });
    }

    [Fact]
    public void Observe_WhenReSeedingAcrossAGap_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor(Config(maxObservationGapTicks: 10));
        long captureTicks = 0;

        AllocationAssert.Zero(() =>
        {
            captureTicks += 1_000; // always a gap, so every call takes the re-seed path
            predictor.Observe(new Stamped<Pose>(captureTicks, Pose.Identity));
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
