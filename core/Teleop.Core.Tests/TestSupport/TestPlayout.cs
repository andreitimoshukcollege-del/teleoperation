using Teleop.Core.Buffering;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.TestSupport;

/// <summary>
/// Playout policies for tests that are about something else. Pipeline tests exercise the
/// two-phase receive, not the buffering axis, so they take <see cref="Immediate"/> and the split
/// stays invisible to them; the policies' own behaviour is covered by
/// <c>Buffering/ImmediatePlayoutTests</c> and <c>Buffering/FixedDelayPlayoutTests</c>.
/// </summary>
internal static class TestPlayout
{
    /// <summary>
    /// Capacity large enough that no pipeline test can fill it. Sized above the in-flight ring
    /// those tests use so a full buffer never becomes the reason one of them fails.
    /// </summary>
    private const int Capacity = 64;

    internal static PlayoutPolicyConfig Config(long delayBudgetTicks = 0) => new PlayoutPolicyConfig(
        historyCapacity: Capacity,
        initialDelayBudgetTicks: delayBudgetTicks,
        minDelayBudgetTicks: 0,
        maxDelayBudgetTicks: delayBudgetTicks,
        targetPercentile: 0.95, delayWindowSamples: 64,
        delayProcessNoise: 0.01f,
        delayMeasurementNoise: 0.001f,
        maxAdaptationRatePerSecond: 0.0,
        lossWeight: 0.5);

    internal static IPlayoutPolicy<Pose> Immediate(IMetricSink metrics, ITimeAuthority clock) =>
        new ImmediatePlayout(Config(), metrics, clock);

    internal static IPlayoutPolicy<Pose> Fixed(IMetricSink metrics, ITimeAuthority clock, long delayBudgetTicks) =>
        new FixedDelayPlayout(Config(delayBudgetTicks), metrics, clock);
}
