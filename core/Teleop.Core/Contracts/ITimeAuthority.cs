// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// The single source of time for everything in Core. Nothing else in Core reads a clock:
    /// no <c>DateTime</c>, no <c>Stopwatch</c>, no <c>Time.time</c>. That restriction is what
    /// makes replay bit-deterministic and latency figures trustworthy, so this interface is
    /// deliberately tiny — the less it can do, the fewer ways determinism can be lost.
    ///
    /// Every <c>long</c> tick value elsewhere in Core (<c>Stamped{T}.CaptureTicks</c>,
    /// <c>IPredictor.Predict(targetTicks)</c>, <c>IMetricSink.Record(..., ticks)</c>) is on the
    /// timebase of the authority injected into that component. Mixing timebases is a bug.
    ///
    /// Implementations live in <c>Time/</c>: a manually advanced clock for replay and tests,
    /// and a host-driven clock supplied by Unity or Teleop.Eval.
    /// </summary>
    public interface ITimeAuthority
    {
        /// <summary>
        /// Number of ticks in one second. Fixed for the lifetime of the instance, so callers
        /// may cache it. Use it for every tick/second/millisecond conversion rather than
        /// hard-coding a tick rate; nothing in Core assumes a particular resolution.
        /// </summary>
        long TicksPerSecond { get; }

        /// <summary>
        /// Current time on this authority's timebase. Monotonic and non-decreasing: repeated
        /// reads within one simulation step must return the same value, so a step is a pure
        /// function of the time handed to it. Ticks are not wall-clock and carry no epoch;
        /// only differences between them are meaningful.
        /// </summary>
        long NowTicks { get; }
    }
}
