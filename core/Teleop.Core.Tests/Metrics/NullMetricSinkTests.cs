using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Metrics;

namespace Teleop.Core.Tests.Metrics;

public class NullMetricSinkTests
{
    [Fact]
    public void Record_DoesNotThrow()
    {
        var sink = new NullMetricSink();

        var exception = Record.Exception(() => sink.Record("m2p_ms", 42.0, 100));

        Assert.Null(exception);
    }

    [Fact]
    public void Record_Allocates_Zero_Bytes()
    {
        var sink = new NullMetricSink();
        AllocationAssert.Zero(() => sink.Record("m2p_ms", 42.0, 100));
    }
}
