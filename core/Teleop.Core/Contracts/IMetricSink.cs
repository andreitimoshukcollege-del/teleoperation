// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Records named scalar samples. This is the one push-style API in Core: everywhere else,
    /// a component exposes a <c>Diagnostics</c> property returning a struct and never calls out
    /// to report anything. Measurement is the exception because a reconciler emits correction
    /// cost on every step and nobody is polling it at that rate.
    ///
    /// The sink is a recorder, not a logger and not a metric definition. It stores whatever
    /// name it is given, which is precisely why the discipline lives elsewhere: <b>every name
    /// passed here must be defined in docs/metrics.md</b>, in the same PR that first emits it.
    /// An undefined metric is how a research project ends up with numbers nobody can interpret
    /// six months later.
    ///
    /// Implementations live in <c>Metrics/</c> (a null sink, an in-memory tracker) and in the
    /// hosts, since writing a <c>metrics.csv</c> is I/O and I/O is not Core's. Implementations
    /// must not allocate per call — the hot path calls this several times per frame — and must
    /// not reorder or drop samples.
    /// </summary>
    public interface IMetricSink
    {
        /// <summary>
        /// Record one sample.
        /// </summary>
        /// <param name="name">
        /// Metric name as defined in docs/metrics.md. Pass a constant or an interned literal:
        /// a name built with string concatenation or interpolation allocates on the hot path.
        /// Names are opaque to the sink and compared by value.
        /// </param>
        /// <param name="value">
        /// The sample, in the unit docs/metrics.md states for that metric — milliseconds,
        /// millimetres, degrees. The sink neither knows nor converts units, so a caller
        /// emitting metres into a millimetre metric produces a wrong figure that nothing will
        /// catch. <c>double</c> rather than <c>float</c> so tick differences and jerk values
        /// survive without precision loss.
        /// </param>
        /// <param name="ticks">
        /// When the sample applies, on the emitting component's <c>ITimeAuthority</c> timebase
        /// — the event's own time, not the time it happened to be recorded. Never a wall clock:
        /// a sink that stamped samples itself would make replay output differ run to run.
        /// </param>
        void Record(string name, double value, long ticks);
    }
}
