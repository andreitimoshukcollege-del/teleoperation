using System;
using System.Linq;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.Plant;
using Teleop.RobotArm.Collision;

namespace Teleop.RobotHost.Tests.Backend
{
    /// <summary>The FR3 Duo's host side (docs/adr/0027): the simulated backend, the plant, the start pose and the arguments.</summary>
    public class SerialChainHostTests
    {
        private const long Tps = 1_000_000;
        private const long Ms = Tps / 1000;

        private static (SerialChainProfile Profile, TrajectoryShaperConfig Shaper, Fr3SimBackend Backend) Sim(Fr3SimOptions? options = null)
        {
            SerialChainProfile profile = SerialChainTestProfile.Fr3Duo();
            TrajectoryShaperConfig shaper = SerialChainShaping.ForProfile(profile);
            return (profile, shaper, new Fr3SimBackend(shaper, SerialChainHost.StartPose(profile, shaper), Tps, options));
        }

        private static float Read(IArmBackend backend, long t, byte motor)
        {
            var output = new JointSample[32];
            int n = backend.ReadState(t, output, out _);
            for (int i = 0; i < n; i++) if (output[i].MotorId == motor) return output[i].Value;
            throw new Xunit.Sdk.XunitException($"motor {motor} not read");
        }

        [Fact]
        public void TheStartPoseIsReadyForTheLeftArmAndItsMirrorForTheRight_HandsOpen()
        {
            (SerialChainProfile profile, TrajectoryShaperConfig shaper, _) = Sim();
            double[] start = SerialChainHost.StartPose(profile, shaper);
            double[] left = Enumerable.Range(1, 7).Select(id => start[shaper.IndexOf((byte)id)]).ToArray();
            double[] right = Enumerable.Range(11, 7).Select(id => start[shaper.IndexOf((byte)id)]).ToArray();

            Assert.Equal(SerialChainHost.ReadyPose, left);
            Assert.Equal(new[] { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, -Math.PI / 4 }, right.Select(v => v + 0.0));
            Assert.Equal(1.0, start[shaper.IndexOf(8)]);
            Assert.Equal(1.0, start[shaper.IndexOf(18)]);
        }

        [Fact]
        public void TheSimulatedArmShapesToATargetOnTheHostsClock_AndTracksBehindTheShaper()
        {
            (_, _, Fr3SimBackend backend) = Sim();
            long t = 1_000 * Ms;
            backend.ReadState(t, new JointSample[32], out _);   // the clock starts here
            backend.Send(new[] { new JointSample(1, 0.5f, false), new JointSample(8, 0f, false) }, t);

            float early = Read(backend, t + 100 * Ms, 1);
            Assert.True(early > 0.001f && early < 0.5f, $"after 100 ms joint 1 is at {early}");
            for (long k = 2; k <= 30; k++) Read(backend, t + k * 100 * Ms, 1);   // the host's loop, well inside the catch-up limit
            Assert.Equal(0.5f, Read(backend, t + 3_000 * Ms, 1), 4);
            Assert.Equal(0f, Read(backend, t + 3_000 * Ms, 8));
            Assert.Equal((float)(-Math.PI / 4), Read(backend, t + 3_000 * Ms, 2), 5);   // untouched joints hold
            Assert.Equal(3_000, backend.Steps);
            Assert.Equal(0, backend.SkippedTicks);
        }

        [Fact]
        public void Disabled_TheArmComesToRestAndIgnoresTargets()
        {
            (_, _, Fr3SimBackend backend) = Sim();
            backend.ReadState(0, new JointSample[32], out _);
            backend.Send(new[] { new JointSample(1, 1.5f, false) }, 0);
            backend.SetEnabled(false, 300 * Ms);
            float stopped = Read(backend, 2_000 * Ms, 1);
            backend.Send(new[] { new JointSample(1, -1f, false) }, 2_000 * Ms);

            Assert.Equal(stopped, Read(backend, 4_000 * Ms, 1), 4);
            Assert.True(stopped > 0.05f && stopped < 1.5f, $"stopped at {stopped}");
            Assert.False(backend.Enabled);
        }

        [Fact]
        public void AStalledHostSkipsPastTheCatchUpLimit_AndCountsIt()
        {
            (_, _, Fr3SimBackend backend) = Sim(new Fr3SimOptions { MaxCatchUpSeconds = 0.5 });
            backend.ReadState(0, new JointSample[32], out _);
            backend.ReadState(3_000 * Ms, new JointSample[32], out long sensed);
            Assert.Equal(500, backend.Steps);
            Assert.Equal(2_500 * Ms, backend.SkippedTicks);
            Assert.Equal(3_000 * Ms, sensed);
        }

        [Fact]
        public void ReadingNoiseIsSeeded()
        {
            float[] Run(ulong seed)
            {
                (_, _, Fr3SimBackend backend) = Sim(new Fr3SimOptions { NoiseRad = 1e-4, Seed = seed });
                return Enumerable.Range(0, 20).Select(k => Read(backend, k * 10 * Ms, 3)).ToArray();
            }

            Assert.Equal(Run(5), Run(5));
            Assert.NotEqual(Run(5), Run(6));
            Assert.All(Run(5), v => Assert.True(Math.Abs(v) < 1e-3));
        }

        [Fact]
        public void TheOptionsAreChecked()
        {
            Assert.NotNull(new Fr3SimOptions { TrackHz = 200 }.Validate());
            Assert.NotNull(new Fr3SimOptions { TrackDamping = 0 }.Validate());
            Assert.NotNull(new Fr3SimOptions { NoiseRad = -1 }.Validate());
            Assert.Null(new Fr3SimOptions().Validate());
            Assert.Throws<ArgumentException>(() => Sim(new Fr3SimOptions { TrackHz = double.NaN }));
        }

        [Fact]
        public void ThePlantDropsStaleCommands_ClampsToTheSoftLimits_AndCountsUnknownMotors()
        {
            (SerialChainProfile profile, _, Fr3SimBackend backend) = Sim();
            var plant = new SerialChainPlant(profile, backend);

            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointTargets(new[]
            {
                new JointTarget(4, 0.5f, 0f),     // joint 4's range is below zero: clamped to its soft upper
                new JointTarget(8, 2f, 0f),       // a hand's fraction: clamped to 1
                new JointTarget(9, 0.1f, 0f),     // no such motor
                new JointTarget(11, 0.2f, 0f),
            }, captureTicks: 100, nowTicks: 0));
            Assert.Equal(JointCommandOutcome.Stale, plant.CommandJointTargets(new[] { new JointTarget(11, -0.2f, 0f) }, captureTicks: 100, nowTicks: 1));

            Assert.Equal(1, plant.AcceptedCommands);
            Assert.Equal(1, plant.StaleCommands);
            Assert.Equal(1, plant.UnknownTargets);
            Assert.Equal(2, plant.ClampedTargets);
            Assert.Equal(new byte[] { 4, 8, 11 }, plant.LastSent.ToArray().Select(s => s.MotorId));
            Assert.Equal((float)profile.Arms[0].Joints[3].Limits.SoftUpper, plant.LastSent[0].Value);
            Assert.Equal(1f, plant.LastSent[1].Value);

            plant.RestartJointCommandOrdering();
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointTargets(new[] { new JointTarget(11, -0.2f, 0f) }, captureTicks: 5, nowTicks: 2));
        }

        [Fact]
        public void WithACollisionModel_ThePlantRefusesACollidingCommand_AndTakesTheNextClearOne()
        {
            (SerialChainProfile profile, TrajectoryShaperConfig shaper, Fr3SimBackend backend) = Sim();
            var model = SerialChainProfileJson.LoadCollision(System.IO.Path.Combine(System.IO.Path.GetDirectoryName(SerialChainTestProfile.Path)!, "fr3-duo.collision.json"));
            double[] start = SerialChainHost.StartPose(profile, shaper);
            double[] byProfile = profile.Arms.SelectMany(a => a.Joints.Select(j => j.MotorId).Concat(a.HasGripper ? new[] { a.Gripper.MotorId } : new byte[0]))
                .Select(id => start[shaper.IndexOf(id)]).ToArray();
            var plant = new SerialChainPlant(profile, backend, model, byProfile);

            // Both arms swung toward each other: the forearms meet between the bases.
            JointTarget[] crash = new[] { -1.2, 0.6, 0, -1.6, 0, 2.0, 0.8 }.Select((q, j) => new JointTarget((byte)(j + 1), (float)q, 0f))
                .Concat(new[] { 1.2, 0.6, 0, -1.6, 0, 2.0, -0.8 }.Select((q, j) => new JointTarget((byte)(j + 11), (float)q, 0f))).ToArray();
            Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointTargets(crash, captureTicks: 10, nowTicks: 0));
            Assert.Equal(1, plant.RefusedCommands);
            Assert.Contains("mm apart", plant.DescribeLastRefusal());
            Assert.Equal(0, plant.LastSentCount);

            // A small clear move is taken, even with an older-looking stamp than the refused one: refused stamps do not count.
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointTargets(new[] { new JointTarget(1, 0.1f, 0f) }, captureTicks: 5, nowTicks: 1));
        }

        [Fact]
        public void ThePlantReportsEveryMotorWithItsSensedTime()
        {
            (SerialChainProfile profile, _, Fr3SimBackend backend) = Sim();
            var plant = new SerialChainPlant(profile, backend);
            var buffer = new JointSample[JointStateCodec.MaxJointsPerMessage];
            plant.ReadJointState(0, buffer);
            int n = plant.ReadJointState(250 * Ms + 400, buffer);

            Assert.Equal(16, n);
            Assert.Equal(250 * Ms, plant.LastFeedbackTicks);
            Assert.All(buffer.Take(n), s => Assert.True(s.Sensed));
        }

        [Fact]
        public void SixteenJointsFitOneCommandAndOneStateFrame_TheJetRoversBytesAreUnchanged()
        {
            Assert.Equal(256, JointDatagram.MaxBytes);
            Assert.Equal(26, JointCommandCodec.MaxJointsPerMessage);
            Assert.Equal(35, JointStateCodec.MaxJointsPerMessage);

            var sixteen = Enumerable.Range(0, 16).Select(i => new JointTarget((byte)(i + 1), 0.1f * i, 0f)).ToArray();
            var buffer = new byte[JointDatagram.MaxBytes];
            Assert.True(JointCommandCodec.TryEncode(7, 123, sixteen, buffer, out int written));
            Assert.Equal(14 + 16 * 9, written);
            Assert.True(JointStateCodec.TryEncode(1, 2, 3, 4, 5, Tps, sixteen.Select(t => new JointSample(t.MotorId, t.Angle, true)).ToArray(), buffer, out written));
            Assert.Equal(46 + 16 * 6, written);

            // A JetRover command is the same size it always was: the budget moved, the layout did not.
            Assert.True(JointCommandCodec.TryEncode(7, 123, sixteen.Take(6).ToArray(), buffer, out written));
            Assert.Equal(14 + 6 * 9, written);
        }

        [Theory]
        [InlineData(new[] { "--joint-local-port", "6002", "--backend", "sim" }, "--profile-path")]
        [InlineData(new[] { "--profile-path", "p.json", "--backend", "sim" }, "--joint-local-port")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002" }, "--backend is sim")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "fci" }, "--backend is sim")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "arm-server" }, "--arm-server-socket")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "arm-server", "--arm-server-socket", "/tmp/s", "--arm-server-timeout", "0" }, "--arm-server-timeout")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "sim", "--relay-socket", "x" }, "unknown argument")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "sim", "--sim-track-hz", "500" }, "tracking frequency")]
        [InlineData(new[] { "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "sim", "--auto-record" }, "--record-control")]
        public void BadArgumentsAreRefused(string[] args, string expected)
        {
            Assert.Null(SerialChainHostArgs.TryParse(args, out string error));
            Assert.Contains(expected, error);
        }

        [Fact]
        public void GoodArgumentsParse_AndAutoRecordingIsSessionV2FromTheSimHost()
        {
            SerialChainHostArgs? a = SerialChainHostArgs.TryParse(new[]
            {
                "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "sim", "--sim-track-hz", "30",
                "--record-control", "rec/.active", "--auto-record", "--auto-record-root", "rec/fr3",
            }, out string error);

            Assert.True(a != null, error);
            Assert.Equal(30, a!.Sim.TrackHz);
            Assert.Equal("teleop-session", a.AutoRecord!.SessionFormat);
            Assert.Equal(2, a.AutoRecord.SessionVersion);
            Assert.Equal("sim-host", a.AutoRecord.Source);
            Assert.Equal("p.json", a.AutoRecord.ProfilePath);
            Assert.Equal(0, a.AutoRecord.DriverWaitSeconds);
        }

        [Fact]
        public void TheArmServerBackend_Parses_WithItsOwnSocketBesideTheServers_AndIsaacSessionsSaySo()
        {
            SerialChainHostArgs? a = SerialChainHostArgs.TryParse(new[]
            {
                "--profile-path", "p.json", "--joint-local-port", "6002", "--backend", "arm-server", "--arm-server-socket", "/tmp/teleop-isaac.sock",
            }, out string error);

            Assert.True(a != null, error);
            Assert.Equal("/tmp/teleop-isaac.sock.robothost", a!.ArmServerClientSocket);
            Assert.Equal(60, a.ArmServerTimeoutSeconds);
            Assert.Equal("isaac", SerialChainHost.SourceOf(Teleop.RobotHost.ArmServer.ArmServerKind.Isaac));
            Assert.Equal("robot", SerialChainHost.SourceOf(Teleop.RobotHost.ArmServer.ArmServerKind.Fci));
        }

        [Fact]
        public void TheProfileKindDecidesTheHost()
        {
            Assert.True(SerialChainProfileJson.IsSerialChain(SerialChainTestProfile.Path));
            string jetrover = System.IO.Path.Combine(System.IO.Path.GetDirectoryName(SerialChainTestProfile.Path)!, "jetrover.json");
            Assert.False(SerialChainProfileJson.IsSerialChain(jetrover));
        }
    }
}
