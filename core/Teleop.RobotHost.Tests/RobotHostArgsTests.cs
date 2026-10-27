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

        private static readonly string[] Recording =
            { "--joint-local-port", "6002", "--record-control", "/home/jetson/recordings/.active" };

        [Fact]
        public void AutoRecord_IsOff_UnlessAskedFor()
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(Recording).ToArray(), out _)!.Value.AutoRecord);
        }

        [Fact]
        public void AutoRecord_HasTheAdrDefaults()
        {
            var parsed = RobotHostArgs.TryParse(Base.Concat(Recording).Concat(new[] { "--auto-record", "--profile-path", "/p/jetrover.json" }).ToArray(),
                out string? error);
            Assert.Null(error);
            var auto = parsed!.Value.AutoRecord!;
            Assert.Equal(30, auto.IdleSeconds);
            Assert.Equal(60, auto.MaxMinutes);
            Assert.Equal(1, auto.StopFreeGigabytes);
            Assert.Equal(2, auto.StartFreeGigabytes);
            Assert.Equal(15, auto.DriverWaitSeconds);
            Assert.Equal(5, auto.MinSplitSeconds);
            Assert.Equal(5, auto.HeadsetSeconds);
            Assert.Null(auto.RecordingsRoot);
            Assert.Equal("/p/jetrover.json", auto.ProfilePath); // the profile RobotHost loaded, unless told otherwise
            Assert.Null(auto.RobotHostBuildInfoPath);
            Assert.Null(auto.DriverBuildInfoPath);
        }

        [Fact]
        public void AutoRecord_EveryThresholdAndPath_IsAnArgument()
        {
            var parsed = RobotHostArgs.TryParse(Base.Concat(Recording).Concat(new[]
            {
                "--auto-record", "--auto-record-idle-seconds", "12.5", "--auto-record-max-minutes", "30", "--auto-record-stop-free-gb", "0.5",
                "--auto-record-start-free-gb", "1.5", "--auto-record-driver-wait-seconds", "4", "--auto-record-min-split-seconds", "0",
                "--auto-record-headset-seconds", "2.5", "--auto-record-root", "/r", "--auto-record-profile", "/a.json", "--build-info-robothost", "/b1", "--build-info-driver", "/b2",
            }).ToArray(), out string? error);
            Assert.Null(error);
            var auto = parsed!.Value.AutoRecord!;
            Assert.Equal(12.5, auto.IdleSeconds);
            Assert.Equal(30, auto.MaxMinutes);
            Assert.Equal(0.5, auto.StopFreeGigabytes);
            Assert.Equal(1.5, auto.StartFreeGigabytes);
            Assert.Equal(4, auto.DriverWaitSeconds);
            Assert.Equal(0, auto.MinSplitSeconds);
            Assert.Equal(2.5, auto.HeadsetSeconds);
            Assert.Equal("/r", auto.RecordingsRoot);
            Assert.Equal("/a.json", auto.ProfilePath);
            Assert.Equal("/b1", auto.RobotHostBuildInfoPath);
            Assert.Equal("/b2", auto.DriverBuildInfoPath);
        }

        [Theory]
        [InlineData("--auto-record-idle-seconds", "0")]
        [InlineData("--auto-record-idle-seconds", "soon")]
        [InlineData("--auto-record-max-minutes", "-1")]
        [InlineData("--auto-record-stop-free-gb", "-1")]
        [InlineData("--auto-record-start-free-gb", "0.5")] // below the stop threshold (1)
        [InlineData("--auto-record-driver-wait-seconds", "NaN")]
        [InlineData("--auto-record-min-split-seconds", "-2")]
        [InlineData("--auto-record-headset-seconds", "0")]
        public void AutoRecord_RefusesAnUnusableThreshold(string flag, string value)
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(Recording).Concat(new[] { "--auto-record", flag, value }).ToArray(), out string? error));
            Assert.NotNull(error);
        }

        [Fact]
        public void AutoRecord_NeedsTheControlFileAndTheJointPort()
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { "--auto-record", "--joint-local-port", "6002" }).ToArray(), out string? e1));
            Assert.Contains("--record-control", e1);
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { "--auto-record", "--record-control", "/x" }).ToArray(), out string? e2));
            Assert.Contains("--joint-local-port", e2);
        }

        [Fact]
        public void ArmPower_HasTheDecidedDefaults_AndNoRestPose()
        {
            var power = RobotHostArgs.TryParse(Base, out _)!.Value.ArmPower;
            Assert.Null(power.RestPoseDegrees);
            Assert.Equal(0.5, power.ParkSpeedRadPerSec);
            Assert.Equal(3, power.ToleranceDegrees);
            Assert.Equal(20, power.ParkTimeoutSeconds);
            Assert.Equal(0, power.RestContactDegrees);
        }

        [Fact]
        public void ArmPower_EveryValue_IsAnArgument()
        {
            var parsed = RobotHostArgs.TryParse(Base.Concat(new[]
            {
                "--rest-pose-deg", "-1.20,91.16,-78.48,-112.0,5.76", "--park-speed-rad-per-sec", "0.3", "--park-tolerance-deg", "2",
                "--park-timeout-seconds", "30", "--park-contact-deg", "10",
            }).ToArray(), out string? error);
            Assert.Null(error);
            var power = parsed!.Value.ArmPower;
            Assert.Equal(new[] { -1.20, 91.16, -78.48, -112.0, 5.76 }, power.RestPoseDegrees);
            Assert.Equal(0.3, power.ParkSpeedRadPerSec);
            Assert.Equal(2, power.ToleranceDegrees);
            Assert.Equal(30, power.ParkTimeoutSeconds);
            Assert.Equal(10, power.RestContactDegrees);
        }

        [Fact]
        public void ArmPower_AnEmptyRestPose_IsNone()
        {
            var parsed = RobotHostArgs.TryParse(Base.Concat(new[] { "--rest-pose-deg", "" }).ToArray(), out string? error);
            Assert.Null(error);
            Assert.Null(parsed!.Value.ArmPower.RestPoseDegrees);
        }

        [Theory]
        [InlineData("--rest-pose-deg", "1,two,3")]
        [InlineData("--rest-pose-deg", "0,400,0,0,0")]
        [InlineData("--park-speed-rad-per-sec", "0")]
        [InlineData("--park-speed-rad-per-sec", "10")]
        [InlineData("--park-tolerance-deg", "-1")]
        [InlineData("--park-timeout-seconds", "never")]
        [InlineData("--park-contact-deg", "90")]
        public void ArmPower_RefusesAnUnusableValue(string flag, string value)
        {
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { flag, value }).ToArray(), out string? error));
            Assert.NotNull(error);
        }
    }
}
