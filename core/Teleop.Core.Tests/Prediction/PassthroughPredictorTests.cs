using System.Numerics;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

public class PassthroughPredictorTests
{
    /// <summary>Milliseconds as ticks: 1000 ticks per second, so one tick is 1 ms.</summary>
    private const long TicksPerSecond = 1000;

    /// <summary>
    /// Deliberately hostile parameters. Every value here is either meaningless or outright invalid
    /// for a real predictor -- a horizon cap of one tick, a history capacity of zero, smoothing
    /// factors far outside [0, 1], zero speed caps. <c>none</c> must construct and behave
    /// identically regardless, because it reads none of them.
    /// </summary>
    private static PredictorConfig HostileConfig() => new PredictorConfig(
        maxHorizonTicks: 1,
        maxObservationGapTicks: 1,
        historyCapacity: 0,
        smoothingAlpha: 12f,
        smoothingBeta: -3f,
        processNoise: -1f,
        measurementNoise: -1f,
        maxLinearSpeed: 0f,
        maxAngularSpeed: 0f);

    private static PassthroughPredictor NewPredictor() => new PassthroughPredictor(HostileConfig());

    private static Stamped<Pose> Sample(long captureTicks, float x) =>
        new Stamped<Pose>(
            captureTicks,
            new Pose(
                new Vector3(x, x * 2f, x * -3f),
                Quaternion.CreateFromAxisAngle(Vector3.UnitZ, x * 0.1f)));

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

    // 1. It ignores the whole config, including values every other predictor rejects.
    [Fact]
    public void Constructor_AcceptsAConfigNoOtherPredictorWould()
    {
        var predictor = new PassthroughPredictor(HostileConfig());

        predictor.Observe(Sample(100, 1f));

        // A MaxHorizonTicks of 1 is in the config and is not applied: the reported horizon is the
        // raw 900 tick staleness, and the returned pose is the observation regardless.
        Pose predicted = predictor.Predict(1_000);
        AssertSamePose(Sample(100, 1f).Value, predicted);
        Assert.Equal(900, predictor.Diagnostics.HorizonTicks);
    }

    // 2. Documented pre-observation value: Pose.Identity, not default(Pose).
    [Fact]
    public void Predict_BeforeAnyObservation_ReturnsExactlyPoseIdentity()
    {
        var predictor = NewPredictor();

        Pose predicted = predictor.Predict(12_345);

        Assert.Equal(Vector3.Zero, predicted.Position);
        Assert.Equal(Quaternion.Identity, predicted.Rotation);

        // The distinction that matters: default(Quaternion) is the all-zero, non-unit quaternion,
        // which is not a rotation at all and would produce NaN out of every downstream geodesic
        // angle. Identity has W = 1.
        Assert.Equal(1f, predicted.Rotation.W);
        Assert.NotEqual(default(Quaternion), predicted.Rotation);
        Assert.Equal(0f, PoseMath.OrientationErrorRadians(Pose.Identity, predicted), 6);
    }

    [Fact]
    public void Diagnostics_OnAFreshInstance_AreExactlyNone()
    {
        var predictor = NewPredictor();

        AssertSameDiagnostics(PredictorDiagnostics.None, predictor.Diagnostics);
    }

    [Fact]
    public void Diagnostics_NeverClaimUncertainty()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(10, 1f));
        predictor.Predict(20);

        Assert.False(predictor.Diagnostics.HasUncertainty);
    }

    // 3. Determinism.
    [Fact]
    public void IdenticalCallSequences_ProduceBitIdenticalOutput()
    {
        var a = NewPredictor();
        var b = NewPredictor();

        for (int i = 1; i <= 200; i++)
        {
            // A mix of accepted, stale and duplicate observations so the sequence exercises every
            // branch rather than only the happy path.
            long captureTicks = i % 7 == 0 ? i - 5 : (i % 11 == 0 ? i - 1 : i);
            var obs = Sample(captureTicks, i * 0.01f);

            a.Observe(obs);
            b.Observe(obs);

            long target = i * 10;
            AssertSamePose(a.Predict(target), b.Predict(target));
            AssertSameDiagnostics(a.Diagnostics, b.Diagnostics);
        }
    }

    // 4. The defining behaviour: targetTicks does not affect the pose.
    [Fact]
    public void Predict_ReturnsTheRetainedObservation_ForEveryTargetTime()
    {
        var predictor = NewPredictor();
        Stamped<Pose> obs = Sample(1_000, 2.5f);
        predictor.Observe(obs);

        // Far past, exactly at capture, near future, absurd future -- all the same pose.
        AssertSamePose(obs.Value, predictor.Predict(0));
        AssertSamePose(obs.Value, predictor.Predict(1_000));
        AssertSamePose(obs.Value, predictor.Predict(1_050));
        AssertSamePose(obs.Value, predictor.Predict(long.MaxValue / 2));
    }

    [Fact]
    public void Predict_RepeatedWithTheSameTarget_ReturnsTheSameValue()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(100, 1f));

        Pose first = predictor.Predict(400);
        Pose second = predictor.Predict(400);

        AssertSamePose(first, second);
    }

    // 5. Out-of-order observation: rejected, counted, state untouched.
    [Fact]
    public void Observe_OutOfOrder_IsRejectedAndCounted_AndChangesNothing()
    {
        var predictor = NewPredictor();
        Stamped<Pose> newest = Sample(500, 5f);
        predictor.Observe(newest);
        predictor.Predict(600);

        var stale = new Stamped<Pose>(
            499, new Pose(new Vector3(-99f, -99f, -99f), Quaternion.CreateFromAxisAngle(Vector3.UnitX, 3f)));
        predictor.Observe(stale);

        AssertSamePose(newest.Value, predictor.Predict(600));

        PredictorDiagnostics diagnostics = predictor.Diagnostics;
        Assert.Equal(1, diagnostics.AcceptedObservations);
        Assert.Equal(1, diagnostics.RejectedObservations);
        Assert.Equal(500, diagnostics.LastObservationTicks);
    }

    // 6. Duplicate observation: idempotent, per IPredictor.Observe's explicit clause.
    [Fact]
    public void Observe_TwiceWithAnIdenticalSample_LeavesTheSameStateAsOnce()
    {
        var once = NewPredictor();
        var twice = NewPredictor();
        Stamped<Pose> obs = Sample(750, 3f);

        once.Observe(obs);
        twice.Observe(obs);
        twice.Observe(obs);

        AssertSamePose(once.Predict(800), twice.Predict(800));
        Assert.Equal(1, twice.Diagnostics.AcceptedObservations);
        Assert.Equal(1, twice.Diagnostics.RejectedObservations);
        Assert.Equal(once.Diagnostics.LastObservationTicks, twice.Diagnostics.LastObservationTicks);
    }

    [Fact]
    public void Observe_DuplicateOfADifferentPoseAtTheSameStamp_DoesNotOverwrite()
    {
        var predictor = NewPredictor();
        Stamped<Pose> first = Sample(200, 1f);
        var conflicting = new Stamped<Pose>(200, new Pose(new Vector3(42f, 42f, 42f), Quaternion.Identity));

        predictor.Observe(first);
        predictor.Observe(conflicting);

        AssertSamePose(first.Value, predictor.Predict(200));
    }

    // 7. The max-by reduction is commutative: arrival order cannot change the outcome.
    [Fact]
    public void Observe_AnyPermutationOfTheSameSamples_LeavesTheSameRetainedObservation()
    {
        Stamped<Pose>[] samples =
        {
            Sample(100, 1f), Sample(200, 2f), Sample(300, 3f), Sample(400, 4f),
        };
        int[][] permutations =
        {
            new[] { 0, 1, 2, 3 },
            new[] { 3, 2, 1, 0 },
            new[] { 1, 3, 0, 2 },
            new[] { 2, 0, 3, 1 },
        };

        Pose expected = samples[3].Value;

        foreach (int[] permutation in permutations)
        {
            var predictor = NewPredictor();
            foreach (int index in permutation)
            {
                predictor.Observe(samples[index]);
            }

            AssertSamePose(expected, predictor.Predict(500));
            Assert.Equal(400, predictor.Diagnostics.LastObservationTicks);

            // Idempotent as well as commutative: replaying the whole set again changes nothing but
            // the rejection count.
            foreach (int index in permutation)
            {
                predictor.Observe(samples[index]);
            }

            AssertSamePose(expected, predictor.Predict(500));
            Assert.Equal(400, predictor.Diagnostics.LastObservationTicks);
        }
    }

    // 8. A multi-hundred-millisecond gap. There is no gap policy here, by design.
    [Fact]
    public void Predict_AcrossAMultiHundredMillisecondGap_ReportsTrueStalenessAndDoesNotCompensate()
    {
        var predictor = NewPredictor();
        Stamped<Pose> obs = Sample(1_000, 1f);
        predictor.Observe(obs);

        // 750 ms with nothing arriving. The pose does not move -- that is the baseline's whole
        // behaviour -- and the horizon reports the full staleness, unclamped, so a scorer can bin
        // the resulting error against the 750 ms that actually produced it.
        AssertSamePose(obs.Value, predictor.Predict(1_750));
        Assert.Equal(750, predictor.Diagnostics.HorizonTicks);

        AssertSamePose(obs.Value, predictor.Predict(2_500));
        Assert.Equal(1_500, predictor.Diagnostics.HorizonTicks);
    }

    [Fact]
    public void Diagnostics_HorizonIsNegativeWhenTheTargetPrecedesTheObservation()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(1_000, 1f));

        predictor.Predict(900);

        // PredictorDiagnostics.HorizonTicks documents a negative value as legal and meaningful.
        Assert.Equal(-100, predictor.Diagnostics.HorizonTicks);
    }

    // 9. Reset.
    [Fact]
    public void Reset_RestoresAsConstructedState()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(9_000, 7f));
        predictor.Observe(Sample(8_000, 6f)); // rejected, bumping the counter
        predictor.Predict(9_500);

        predictor.Reset();

        AssertSameDiagnostics(PredictorDiagnostics.None, predictor.Diagnostics);

        Pose predicted = predictor.Predict(10_000);
        Assert.Equal(Vector3.Zero, predicted.Position);
        Assert.Equal(Quaternion.Identity, predicted.Rotation);

        // The retention baseline itself cleared, not just the visible pose: the next trial's first
        // sample must be accepted at a stamp far below the 9000 the previous trial reached.
        Stamped<Pose> next = Sample(0, 1f);
        predictor.Observe(next);
        AssertSamePose(next.Value, predictor.Predict(0));
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
            reused.Observe(Sample(i * 100, i));
            reused.Predict(i * 100 + 50);
        }

        reused.Reset();

        for (int i = 1; i <= 20; i++)
        {
            long captureTicks = i % 4 == 0 ? i - 2 : i;
            var obs = Sample(captureTicks, i * 0.5f);
            reused.Observe(obs);
            fresh.Observe(obs);

            AssertSamePose(fresh.Predict(i * 3), reused.Predict(i * 3));
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
            captureTicks++;
            predictor.Observe(new Stamped<Pose>(
                captureTicks,
                new Pose(new Vector3(1f, 2f, 3f), Quaternion.Identity)));
        });
    }

    [Fact]
    public void Observe_WhenRejected_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(long.MaxValue / 2, 1f));
        Stamped<Pose> stale = Sample(1, 2f);

        AllocationAssert.Zero(() => predictor.Observe(stale));
    }

    [Fact]
    public void Predict_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(0, 1f));
        long target = 0;

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

    [Fact]
    public void Diagnostics_Allocates_Zero_Bytes()
    {
        var predictor = NewPredictor();
        predictor.Observe(Sample(10, 1f));
        predictor.Predict(20);
        long sink = 0;

        AllocationAssert.Zero(() => sink += predictor.Diagnostics.HorizonTicks);
        Assert.True(sink >= 0);
    }
}
