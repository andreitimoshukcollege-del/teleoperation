using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Metrics
{
    /// <summary>
    /// Discards every sample. The value to inject where a component requires an
    /// <see cref="IMetricSink"/> but a run has no interest in recording metrics — a sweep trial
    /// that only cares about the final scored result, or a unit test exercising a component that
    /// takes a sink purely as a constructor dependency it never calls conditionally.
    /// Allocation-free.
    /// </summary>
    public sealed class NullMetricSink : IMetricSink
    {
        public void Record(string name, double value, long ticks)
        {
        }
    }
}
