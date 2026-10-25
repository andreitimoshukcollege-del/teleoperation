using Teleop.RobotHost;

namespace Teleop.RobotHost.Tests
{
    public class RobotHostArgsTests
    {
        private static readonly string[] Base =
            { "--local-port", "6000", "--relay-socket", "/tmp/a.sock", "--local-relay-socket", "/tmp/b.sock" };

        [Fact]
        public void PulsesPerSecond_IsOptional_AndParsed()
        {
            Assert.Null(RobotHostArgs.TryParse(Base, out _)!.Value.PulsesPerSecond);
            var parsed = RobotHostArgs.TryParse(Base.Concat(new[] { "--pulses-per-second", "900" }).ToArray(), out string? error);
            Assert.Null(error);
            Assert.Equal(900f, parsed!.Value.PulsesPerSecond);
        }

        [Theory]
        [InlineData("0")]
        [InlineData("-5")]
        [InlineData("fast")]
        public void PulsesPerSecond_MustBeAPositiveNumber(string value)
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { "--pulses-per-second", value }).ToArray(), out string? error));
            Assert.NotNull(error);
        }
    
        [Fact]
        public void DiscoveryPort_IsOptional_AndParsed()
        {
            Assert.Null(RobotHostArgs.TryParse(Base, out _)!.Value.DiscoveryPort);
            var parsed = RobotHostArgs.TryParse(Base.Concat(new[] { "--discovery-port", "6005" }).ToArray(), out string? error);
            Assert.Null(error);
            Assert.Equal(6005, parsed!.Value.DiscoveryPort);
        }

        [Theory]
        [InlineData("0")]
        [InlineData("70000")]
        [InlineData("port")]
        public void DiscoveryPort_MustBeAPort(string value)
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { "--discovery-port", value }).ToArray(), out string? error));
            Assert.NotNull(error);
        }
    }
}
