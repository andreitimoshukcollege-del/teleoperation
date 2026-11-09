using Teleop.Core.Contracts;
using Teleop.Core.Transport.Impairments;
using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    /// <summary>
    /// Uplink profiles that name a network dataset's trace (docs/adr/0035 N4): <c>trace:&lt;dataset&gt;/&lt;id&gt;</c> under
    /// <c>--uplink-traces</c>, a version-2 trace replayed by time, rescaled to the host's clock; refused when it does not load.
    /// </summary>
    public sealed class UplinkProfilesTests : IDisposable
    {
        private const long Tps = 1_000_000_000;
        private readonly string _root = Path.Combine(Path.GetTempPath(), "uplink-traces-" + Guid.NewGuid().ToString("N"));

        public UplinkProfilesTests()
        {
            Directory.CreateDirectory(Path.Combine(_root, "mahimahi", "traces"));
            // 10 MHz ticks, a datagram every 20 ms: 15 ms, lost, 450 ms.
            File.WriteAllText(Path.Combine(_root, "mahimahi", "traces", "Verizon-LTE-short-down.trace"), "TRACE|2|10000000|200000\n150000\nL\n4500000\n");
            File.WriteAllText(Path.Combine(_root, "mahimahi", "traces", "old.trace"), "TRACE|1|10000000\n150000\n");
        }

        public void Dispose() => Directory.Delete(_root, recursive: true);

        [Fact]
        public void ATrace_ResolvesToATimedReplay_InTheHostsTicks()
        {
            Assert.True(UplinkProfiles.TryResolve("trace:mahimahi/Verizon-LTE-short-down", Tps, _root, out INetworkImpairment[] imps, out string? error), error);
            var t = Assert.IsType<TimedTraceImpairment>(Assert.Single(imps));
            Assert.Equal(3, t.SampleCount);
            Assert.Equal(20_000_000L, t.IntervalTicks); // 20 ms in ns
            t.Bind(1);
            var fate = new Teleop.Core.Types.DatagramFate { SendTicks = 5 };
            t.ApplyOnSend(ref fate);
            t.ApplyOnDeliver(ref fate);
            Assert.Equal(15_000_000L, fate.DelayTicks); // 15 ms in ns
            Assert.True(UplinkProfiles.TryResolve("150ms-20j-0.5loss", Tps, null, out _, out _)); // Core's names still resolve
        }

        [Theory]
        [InlineData("trace:mahimahi/missing")] // no such file
        [InlineData("trace:../mahimahi/Verizon-LTE-short-down")] // not a plain dataset name
        [InlineData("trace:mahimahi/..")]
        [InlineData("trace:mahimahi")] // no id
        [InlineData("trace:mahimahi/old")] // version 1: no interval, no losses
        public void ATraceThatDoesNotLoad_IsRefusedWithAReason(string name)
        {
            Assert.False(UplinkProfiles.TryResolve(name, Tps, _root, out _, out string? error));
            Assert.False(string.IsNullOrEmpty(error));
        }

        [Fact]
        public void WithoutARoot_NoTraceCanBeNamed_AndASchedulesTracesAreChecked()
        {
            Assert.False(UplinkProfiles.TryResolve("trace:mahimahi/Verizon-LTE-short-down", Tps, null, out _, out string? error));
            Assert.Contains("--uplink-traces", error);
            Assert.True(UplinkSchedule.TryParse("schedule 3 trace:mahimahi/Verizon-LTE-short-down@0;lan@60", Tps, _root, out UplinkSchedule? s, out error), error);
            Assert.Equal("trace:mahimahi/Verizon-LTE-short-down", s!.ProfileOf(0));
            Assert.False(UplinkSchedule.TryParse("schedule 3 trace:mahimahi/missing@0", Tps, _root, out _, out _));
            Assert.False(UplinkSchedule.TryParse("schedule 3 trace:mahimahi/Verizon-LTE-short-down@0", Tps, out _, out _)); // no root
        }

        [Fact]
        public void TheReceiver_SwitchesToATrace()
        {
            var socket = new Teleop.Core.Transport.LoopbackTransport(64, 64);
            var receiver = new UplinkImpairedReceiver(socket, Tps, 64, _root);
            receiver.SetProfile("trace:mahimahi/Verizon-LTE-short-down", 9);
            Assert.Equal("trace:mahimahi/Verizon-LTE-short-down", receiver.Profile);
            Assert.Throws<ArgumentException>(() => receiver.SetProfile("trace:mahimahi/missing", 9));
        }
    }
}
