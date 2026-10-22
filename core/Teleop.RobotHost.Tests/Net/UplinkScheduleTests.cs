using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    /// <summary>
    /// The robot-side uplink schedule (docs/adr/0021 decision 13): the trainer's named-schedule grammar,
    /// refused whole on any name Core does not know, switched by elapsed time.
    /// </summary>
    public class UplinkScheduleTests
    {
        private const long Tps = 1_000_000_000;

        [Fact]
        public void ASchedule_ParsesAndSwitchesByElapsedTime()
        {
            Assert.True(UplinkSchedule.TryParse("schedule 7 lan@0;150ms-20j-0.5loss@45;none@90", Tps, out UplinkSchedule? s, out string? error), error);
            Assert.Equal(7UL, s!.Seed);
            Assert.Equal(3, s.Count);
            Assert.Equal("lan", s.ProfileOf(s.IndexAt(0.0)));
            Assert.Equal("lan", s.ProfileOf(s.IndexAt(44.999)));
            Assert.Equal("150ms-20j-0.5loss", s.ProfileOf(s.IndexAt(45.0)));
            Assert.Equal("none", s.ProfileOf(s.IndexAt(1e6)));
        }

        [Theory]
        [InlineData("schedule 1 lan@0;150ms-typo@45")] // unknown profile
        [InlineData("schedule 1 150ms-20j-0.5loss@5")] // does not start at 0
        [InlineData("schedule 1 lan@0;50ms-5j@10;lan@10")] // offsets must increase
        [InlineData("schedule x lan@0")] // not a seed
        [InlineData("schedule 1")] // no segments
        public void AMalformedSchedule_IsRefused(string request)
        {
            Assert.False(UplinkSchedule.TryParse(request, Tps, out UplinkSchedule? s, out string? error));
            Assert.Null(s);
            Assert.False(string.IsNullOrEmpty(error));
        }

        [Fact]
        public void OnlyTheScheduleForm_IsASchedule()
        {
            Assert.True(UplinkSchedule.IsSchedule("schedule 1 lan@0"));
            Assert.False(UplinkSchedule.IsSchedule("150ms-20j-0.5loss 1"));
            Assert.False(UplinkSchedule.IsSchedule(null));
        }
    }
}
