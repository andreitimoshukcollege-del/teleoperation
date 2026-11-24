using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Threading;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Gate;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
using Teleop.RobotHost.Tests.Plant;
using Teleop.RobotHost.Tests.Relay;

namespace Teleop.RobotHost.Tests.Gate
{
    /// <summary>
    /// RobotHost's optional command gate (docs/adr/0038 N): off, the plant is as it was; on, a held command never reaches
    /// the relay and is logged; a check that misses its deadline lets the command through unless the gate fails closed;
    /// the park is never judged; and the flag rules keep it off the real robot unless asked for twice.
    /// </summary>
    public sealed class CommandGateTests
    {
        private static readonly GenericArmPlantConfig Config = GenericArmPlantConfig.Default;
        private static readonly RobotArmProfile JetRover = Config.Profile;

        private static byte Motor(JointRole role) => JetRover.Joints[JetRover.TryGetJointIndex(role)].MotorId;

        private static JointTarget[] Pose(ArmPose pose) => new[]
        {
            new JointTarget(Motor(JointRole.BaseYaw), pose.BaseYaw, 1f),
            new JointTarget(Motor(JointRole.Proximal), pose.Proximal, 1f),
            new JointTarget(Motor(JointRole.Distal), pose.Distal, 1f),
            new JointTarget(Motor(JointRole.Wrist), pose.Wrist, 1f),
            new JointTarget(Motor(JointRole.GripperRotate), pose.Roll, 1f),
        };

        private static JointTarget[] Home() => Pose(JetRover.Collision!.Home);

        private sealed class Clock : ITimeAuthority
        {
            public long Now;
            public long TicksPerSecond => 1_000_000_000;
            public long NowTicks => Now;
        }

        private sealed class RealClock : ITimeAuthority
        {
            public long TicksPerSecond => Stopwatch.Frequency;
            public long NowTicks => Stopwatch.GetTimestamp();
        }

        private sealed class FakeCheck : ICommandCheck
        {
            public CommandVerdict Verdict = CommandVerdict.Accept;
            public string Reason = "";
            public readonly List<uint> Asked = new List<uint>();

            public CommandVerdict Judge(uint sequence, long captureTicks, long nowTicks, ReadOnlySpan<JointTarget> targets, out string reason)
            {
                Asked.Add(sequence);
                reason = Verdict == CommandVerdict.Hold ? Reason : "";
                return Verdict;
            }
        }

        [Fact]
        public void Off_ThePlantAcceptsAsBefore()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            Assert.Null(plant.CommandGate);
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(), captureTicks: 10));
            Assert.Single(relay.SentCommands);
            Assert.Equal(0, plant.HeldJointCommands);
        }

        [Fact]
        public void AHold_NeverReachesTheRelay_AndIsLoggedWithItsSequenceAndReason()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            var check = new FakeCheck { Verdict = CommandVerdict.Hold, Reason = "near the mug" };
            var clock = new Clock { Now = 500 };
            var lines = new List<string>();
            var gate = new CommandGate(check, "table:20:-0.2", 5, failClosed: false, clock, lines.Add);
            plant.CommandGate = gate.ShouldHold;

            gate.Sequence = 41;
            Assert.Equal(JointCommandOutcome.Held, plant.CommandJointAngles(Home(), captureTicks: 10));
            Assert.Empty(relay.SentCommands);
            Assert.Equal(1, plant.HeldJointCommands);
            Assert.Equal(new[] { "GH|500|41|near_the_mug" }, lines);
            Assert.Equal('H', SessionLines.Outcome(JointCommandOutcome.Held));

            // Let through, the next one goes to the relay as before, and nothing is logged.
            check.Verdict = CommandVerdict.Accept;
            gate.Sequence = 42;
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(), captureTicks: 11));
            Assert.Single(relay.SentCommands);
            Assert.Single(lines);
            Assert.Equal("GC|500|table:20:-0.2|5|open", gate.ConfigLine(500));
        }

        [Fact]
        public void OnlyACommandThePlantWouldTake_IsPutToTheCheck_AndTheParkNever()
        {
            var relay = new FakeRelayClient();
            var plant = new GenericArmPlant(Config, relay);
            var check = new FakeCheck();
            var gate = new CommandGate(check, "socket:/x", 5, false, new Clock(), null);
            plant.CommandGate = gate.ShouldHold;

            gate.Sequence = 1;
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(), captureTicks: 10));
            gate.Sequence = 2;
            Assert.Equal(JointCommandOutcome.Stale, plant.CommandJointAngles(Home(), captureTicks: 10));
            gate.Sequence = 3;
            JointTarget[] nan = Home();
            nan[0] = new JointTarget(nan[0].MotorId, float.NaN, 1f);
            Assert.Equal(JointCommandOutcome.Refused, plant.CommandJointAngles(nan, captureTicks: 12));
            gate.Sequence = 4;
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointAngles(Home(), captureTicks: 13, askGate: false));
            Assert.Equal(new uint[] { 1 }, check.Asked);
        }

        [Fact]
        public void AMiss_LetsTheCommandThrough_UnlessTheGateFailsClosed()
        {
            var check = new FakeCheck { Verdict = CommandVerdict.NoAnswer };
            var clock = new Clock { Now = 7 };
            var lines = new List<string>();
            var open = new CommandGate(check, "socket:/x", 5, failClosed: false, clock, lines.Add) { Sequence = 9 };
            Assert.False(open.ShouldHold(Home(), 1));
            Assert.Equal(new[] { "GT|7|9|0" }, lines);
            Assert.Equal((0, 1), (open.Holds, open.Misses));

            lines.Clear();
            var closed = new CommandGate(check, "socket:/x", 5, failClosed: true, clock, lines.Add) { Sequence = 10 };
            Assert.True(closed.ShouldHold(Home(), 1));
            Assert.Equal(new[] { "GT|7|10|0", "GH|7|10|no-answer" }, lines);
            Assert.Equal("GC|7|socket:/x|5|closed", closed.ConfigLine(7));
        }

        [Fact]
        public void TheTableExample_HoldsAWristNearTheTable_AndLetsTheRestThrough()
        {
            ArmPose home = JetRover.Collision!.Home;
            float wristZ = ArmKinematics.Forward(JetRover, home.BaseYaw, home.Proximal, home.Distal).Z;
            var near = new TableClearanceCheck(JetRover, 0.020, wristZ - 0.010); // 10 mm above a table, within 20
            var far = new TableClearanceCheck(JetRover, 0.020, wristZ - 0.050);
            Assert.Equal(CommandVerdict.Hold, near.Judge(1, 0, 0, Home(), out string reason));
            Assert.Equal("table:20", reason);
            Assert.Equal(CommandVerdict.Accept, far.Judge(1, 0, 0, Home(), out reason));
            Assert.Equal("", reason);
            // A command that does not place the wrist (the gripper alone) is let through.
            Assert.Equal(CommandVerdict.Accept, near.Judge(1, 0, 0, new[] { new JointTarget(10, 1f, 0f) }, out _));
        }

        [Fact]
        public void TheWire_RoundTrips_AndAReasonIsCleaned()
        {
            var buffer = new byte[CommandGateWire.RequestBytes(CommandGateWire.MaxTargets)];
            int n = CommandGateWire.EncodeRequest(buffer, 77, -5, 123, 1_000_000_000, Home());
            var targets = new JointTarget[CommandGateWire.MaxTargets];
            Assert.True(CommandGateWire.TryDecodeRequest(buffer.AsSpan(0, n), out uint seq, out long capture, out long robot, out long tps, targets, out int count));
            Assert.Equal((77u, -5L, 123L, 1_000_000_000L, 5), (seq, capture, robot, tps, count));
            Assert.Equal(Home(), targets.Take(count).ToArray());

            var reply = new byte[64];
            int r = CommandGateWire.EncodeReply(reply, 77, true, "too|close;" + new string('x', 80));
            Assert.True(CommandGateWire.TryDecodeReply(reply.AsSpan(0, r), out seq, out bool hold, out string reason));
            Assert.Equal(77u, seq);
            Assert.True(hold);
            Assert.Equal(CommandGateWire.MaxReasonBytes, reason.Length);
            Assert.StartsWith("too_close_x", reason);
            Assert.False(CommandGateWire.TryDecodeReply(reply.AsSpan(0, r - 1), out _, out _, out _));
        }

        [LinuxOnlyFact]
        public void OverASocket_TheCheckAnswers_AndOneThatIsGoneIsAMissWithinTheDeadline()
        {
            string dir = Path.Combine(Path.GetTempPath(), "gate-" + Guid.NewGuid().ToString("N").Substring(0, 8));
            Directory.CreateDirectory(dir);
            string path = Path.Combine(dir, "check.sock");
            var clock = new RealClock();
            ArmPose home = JetRover.Collision!.Home;
            float wristZ = ArmKinematics.Forward(JetRover, home.BaseYaw, home.Proximal, home.Distal).Z;
            using (var server = new CommandCheckServer(path, new TableClearanceCheck(JetRover, 0.020, wristZ - 0.010)))
            using (var client = new UdsCommandCheck(path + ".robothost", path, 200, clock))
            {
                using var stop = new CancellationTokenSource();
                var serving = new Thread(() => { while (!stop.IsCancellationRequested) server.Serve(20_000); }) { IsBackground = true };
                serving.Start();
                Assert.Equal(CommandVerdict.Hold, client.Judge(5, 0, clock.NowTicks, Home(), out string reason));
                Assert.Equal("table:20", reason);
                Assert.Equal(CommandVerdict.Accept, client.Judge(6, 0, clock.NowTicks, new[] { new JointTarget(10, 1f, 0f) }, out reason));
                stop.Cancel();
                serving.Join();
            }

            // Nothing listening: a miss, and no longer than the deadline.
            using (var lonely = new UdsCommandCheck(path + ".robothost", path, 5, clock))
            {
                long t0 = clock.NowTicks;
                Assert.Equal(CommandVerdict.NoAnswer, lonely.Judge(7, 0, t0, Home(), out _));
                Assert.True((clock.NowTicks - t0) * 1000.0 / clock.TicksPerSecond < 100);
            }

            // A check that is there but slow: a miss after the deadline.
            using (var slowServer = new CommandCheckServer(path, new FakeCheck()))
            using (var hurried = new UdsCommandCheck(path + ".robothost", path, 5, clock))
            {
                long t0 = clock.NowTicks;
                Assert.Equal(CommandVerdict.NoAnswer, hurried.Judge(8, 0, t0, Home(), out _)); // nobody serves it
                double waited = (clock.NowTicks - t0) * 1000.0 / clock.TicksPerSecond;
                Assert.InRange(waited, 4.5, 100);
                // The late answer is read off and ignored: the next command gets its own.
                Assert.Equal(1, slowServer.Serve(10_000));
                var serving = new Thread(() => slowServer.Serve(500_000)) { IsBackground = true };
                serving.Start();
                Assert.Equal(CommandVerdict.Accept, new UdsCommandCheckProbe(hurried).Judge(9, clock));
                serving.Join();
            }

            foreach (string f in Directory.GetFiles(dir)) File.Delete(f);
            Directory.Delete(dir);
        }

        /// <summary>Asks with a generous wait, by retrying a short-deadline check until its answer comes.</summary>
        private sealed class UdsCommandCheckProbe
        {
            private readonly UdsCommandCheck _check;
            public UdsCommandCheckProbe(UdsCommandCheck check) => _check = check;

            public CommandVerdict Judge(uint seq, ITimeAuthority clock)
            {
                CommandVerdict v = CommandVerdict.NoAnswer;
                for (int i = 0; i < 100 && v == CommandVerdict.NoAnswer; i++) v = _check.Judge(seq, 0, clock.NowTicks, Home(), out _);
                return v;
            }
        }

        private static readonly string[] Base =
            { "--local-port", "6000", "--relay-socket", "/tmp/a.sock", "--local-relay-socket", "/tmp/b.sock",
              "--joint-local-port", "6002", "--record-control", "/r/.active" };

        [Fact]
        public void TheFlags_KeepTheGateOff_AndOffTheRealRobotUnlessAskedForTwice()
        {
            RobotHostArgs plain = RobotHostArgs.TryParse(Base, out string? error)!.Value;
            Assert.Null(error);
            Assert.Null(plain.CommandGate);

            // The twin: --auto-record --auto-record-source isaac.
            string[] twin = Base.Concat(new[] { "--auto-record", "--auto-record-source", "isaac" }).ToArray();
            RobotHostArgs on = RobotHostArgs.TryParse(twin.Concat(new[] { "--command-gate", "table:20:-0.1962", "--command-gate-deadline-ms", "3",
                "--command-gate-fail-closed" }).ToArray(), out error)!.Value;
            Assert.Null(error);
            Assert.Equal("table:20:-0.1962", on.CommandGate);
            Assert.Equal(3, on.CommandGateDeadlineMs);
            Assert.True(on.CommandGateFailClosed);
            Assert.False(on.CommandGateOnRealRobot);

            // Anything else (the real robot's service records as source robot) needs the explicit flag too.
            string[] robot = Base.Concat(new[] { "--auto-record" }).ToArray();
            Assert.Null(RobotHostArgs.TryParse(robot.Concat(new[] { "--command-gate", "socket:/tmp/c.sock" }).ToArray(), out error));
            Assert.Contains("--command-gate-on-real-robot", error);
            Assert.Null(RobotHostArgs.TryParse(Base.Concat(new[] { "--command-gate", "socket:/tmp/c.sock" }).ToArray(), out error));
            RobotHostArgs real = RobotHostArgs.TryParse(robot.Concat(new[] { "--command-gate", "socket:/tmp/c.sock", "--command-gate-on-real-robot" }).ToArray(),
                out error)!.Value;
            Assert.Null(error);
            Assert.True(real.CommandGateOnRealRobot);
            Assert.Equal(5, real.CommandGateDeadlineMs);

            // The gate's other flags without it, a malformed spec and a deadline out of range are refused.
            foreach (string[] bad in new[]
            {
                new[] { "--command-gate-fail-closed" },
                new[] { "--command-gate-on-real-robot" },
                new[] { "--command-gate-deadline-ms", "5" },
                new[] { "--command-gate", "table:20" },
                new[] { "--command-gate", "socket:" },
                new[] { "--command-gate", "table:20:-0.2", "--command-gate-deadline-ms", "0" },
                new[] { "--command-gate", "table:20:-0.2", "--command-gate-deadline-ms", "500" },
            })
            {
                Assert.Null(RobotHostArgs.TryParse(twin.Concat(bad).ToArray(), out error));
                Assert.NotNull(error);
            }
        }
    }
}
