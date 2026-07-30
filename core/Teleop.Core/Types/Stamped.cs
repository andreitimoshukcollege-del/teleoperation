// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Any state value paired with the time it was captured. Capture time — not arrival time —
    /// is the stamp that matters: it is <c>t_capture</c> in docs/metrics.md and the origin of
    /// every latency and prediction-error figure the project reports.
    ///
    /// Ticks are on the timebase of the <c>ITimeAuthority</c> that produced them; they are not
    /// wall-clock and only differences between them are meaningful.
    ///
    /// Samples are routinely delivered out of order and duplicated, so a stamp is the only
    /// reliable ordering key. Consumers must not assume that successive values arrive with
    /// increasing <see cref="CaptureTicks"/>.
    /// </summary>
    /// <typeparam name="TState">
    /// The captured state. Use a value type on the hot path; this wrapper is a struct so that
    /// carrying a sample around does not allocate, and a reference <c>TState</c> gives that up.
    /// </typeparam>
    public readonly struct Stamped<TState>
    {
        /// <summary>Time the value was sampled at its source, in ticks.</summary>
        public readonly long CaptureTicks;

        /// <summary>The captured state.</summary>
        public readonly TState Value;

        public Stamped(long captureTicks, TState value)
        {
            CaptureTicks = captureTicks;
            Value = value;
        }

        public override string ToString() => $"Stamped(t={CaptureTicks}, {Value})";
    }
}
