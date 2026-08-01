using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Registry;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Registry;

public class RegistriesTests
{
    private static readonly PredictorConfig SamplePredictorConfig = new PredictorConfig(
        maxHorizonTicks: 4_000_000, maxObservationGapTicks: 2_000_000, historyCapacity: 16,
        smoothingAlpha: 0.3f, smoothingBeta: 0.1f, processNoise: 0.01f, measurementNoise: 0.001f,
        maxLinearSpeed: 5f, maxAngularSpeed: 10f);

    private static readonly ReconcilerConfig SampleReconcilerConfig = new ReconcilerConfig(
        convergencePositionToleranceMeters: 0.001f, convergenceOrientationToleranceRadians: 0.01f,
        maxTimeToConvergenceTicks: 1_000_000, maxCorrectionLinearSpeedMetersPerSecond: 5f,
        maxCorrectionAngularSpeedRadPerSecond: 10f, rollbackHistoryCapacity: 16);

    [Theory]
    [InlineData("none")]
    [InlineData("const-vel")]
    [InlineData("double-exp")]
    public void Predictors_KeyResolvesAndConstructsAWorkingInstance(string key)
    {
        Assert.True(Registries.Predictors.TryGetValue(key, out var factory));

        var clock = new ManualClock();
        IPredictor<Pose> predictor = factory(SamplePredictorConfig, clock);

        Assert.NotNull(predictor);
        // A working instance: Predict before any Observe must not throw and returns Identity.
        Assert.Equal(Pose.Identity.ToString(), predictor.Predict(0).ToString());
    }

    [Fact]
    public void Predictors_HasExactlyTheThreeExpectedKeys()
    {
        Assert.Equal(3, Registries.Predictors.Count);
        Assert.Contains("none", Registries.Predictors.Keys);
        Assert.Contains("const-vel", Registries.Predictors.Keys);
        Assert.Contains("double-exp", Registries.Predictors.Keys);
    }

    [Fact]
    public void Predictors_KeyLookupIsOrdinal_NotCaseInsensitive()
    {
        Assert.False(Registries.Predictors.TryGetValue("NONE", out _));
        Assert.False(Registries.Predictors.TryGetValue("None", out _));
    }

    [Fact]
    public void Reconcilers_SnapKeyResolvesAndConstructsAWorkingInstance()
    {
        Assert.True(Registries.Reconcilers.TryGetValue("snap", out var factory));

        var clock = new ManualClock();
        var metrics = new InMemoryMetricTracker(capacity: 8);
        IReconciler<Pose> reconciler = factory(SampleReconcilerConfig, metrics, clock);

        Assert.NotNull(reconciler);
        Assert.True(reconciler.IsConverged);
    }

    [Fact]
    public void Codecs_RawKeyResolvesAndConstructsAWorkingInstance()
    {
        Assert.True(Registries.Codecs.TryGetValue("raw", out var factory));

        ICommandCodec codec = factory();

        Assert.NotNull(codec);
        Assert.True(codec.MaxEncodedBytes > 0);
    }

    [Fact]
    public void Transports_LoopbackKeyResolvesAndConstructsAWorkingInstance()
    {
        Assert.True(Registries.Transports.TryGetValue("loopback", out var factory));

        ITransport transport = factory(128, 16);

        Assert.NotNull(transport);
        Assert.Equal(128, transport.MaxPayloadBytes);
    }

    [Fact]
    public void PlayoutPolicies_IsDeclaredAndEmpty()
    {
        Assert.NotNull(Registries.PlayoutPolicies);
        Assert.Empty(Registries.PlayoutPolicies);
    }

    [Fact]
    public void Arbiters_IsDeclaredAndEmpty()
    {
        Assert.NotNull(Registries.Arbiters);
        Assert.Empty(Registries.Arbiters);
    }
}
