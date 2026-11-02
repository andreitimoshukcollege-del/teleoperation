using Teleop.Core.Contracts;
using Teleop.Core.Prediction;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Tests.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The operator end of a serial-chain RobotHost's joint port (docs/adr/0029 decision 5).</summary>
    public class SerialChainJointLinkTests
    {
        private const long Tps = 10_000_000;

        /// <summary>The headset's socket, seen from the test: what the link sent, and what the robot sends back.</summary>
        private sealed class FakeSocket : ITransport
        {
            public readonly List<byte[]> Sent = new List<byte[]>();
            public readonly Queue<(byte[] Bytes, long Arrival)> Incoming = new Queue<(byte[], long)>();

            public int MaxPayloadBytes => JointDatagram.MaxBytes;

            public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
            {
                Sent.Add(payload.ToArray());
                return true;
            }

            public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
            {
                byteCount = 0;
                arrivalTicks = 0;
                if (Incoming.Count == 0 || Incoming.Peek().Arrival > nowTicks) return false;
                (byte[] bytes, long arrival) = Incoming.Dequeue();
                bytes.CopyTo(destination);
                byteCount = bytes.Length;
                arrivalTicks = arrival;
                return true;
            }

            public void Reset()
            {
                Sent.Clear();
                Incoming.Clear();
            }
        }

        private static readonly ClockSyncConfig Sync = new ClockSyncConfig(historyCapacity: 32, smoothingAlpha: 0.2f,
            maxAcceptableRttTicks: Tps, outlierRttMultiple: 4.0, minSamplesBeforeTrusted: 3);

        private static SerialChainJointLink Link(SerialChainProfile profile, FakeSocket socket, ManualClock clock)
        {
            JointPredictorConfig config = SerialChainJointLink.PredictorConfig(profile, Tps, maxHorizonMs: 300, modelSpeedRadPerSec: 2.5f);
            var presets = new[]
            {
                new JointPredictorPreset("hold-last", "hold-last", config),
                new JointPredictorPreset("franka-replay", "franka-replay", config),
            };
            return new SerialChainJointLink(socket, clock, profile, Sync, presets, activePreset: 0);
        }

        /// <summary>The robot's answer to the newest keepalive: a frame of every motor, its clock 5 s ahead, 2 ms each way.</summary>
        private static void Answer(SerialChainProfile profile, FakeSocket socket, long now, float[] values, uint sequence)
        {
            byte[] keepalive = socket.Sent.Last(b => JointStateCodec.TryDecodeSubscribe(b, out _, out _));
            JointStateCodec.TryDecodeSubscribe(keepalive, out long operatorSend, out _);
            long oneWay = Tps / 500, ahead = 5 * Tps;
            long robotNow = now + ahead + oneWay;
            var joints = new List<JointSample>();
            int k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint j in arm.Joints) joints.Add(new JointSample(j.MotorId, values[k++], true));
                if (arm.HasGripper) joints.Add(new JointSample(arm.Gripper.MotorId, values[k++], true));
            }

            var buffer = new byte[JointDatagram.MaxBytes];
            Assert.True(JointStateCodec.TryEncode(sequence, robotNow, robotNow - Tps / 1000, operatorSend, robotNow - 1, Tps, joints.ToArray(), buffer, out int n));
            socket.Incoming.Enqueue((buffer.AsSpan(0, n).ToArray(), now + 2 * oneWay));
        }

        [Fact]
        public void TheKeepalive_GoesAtOnce_AndEveryHalfSecond()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            for (int ms = 0; ms <= 1200; ms += 10)
            {
                clock.SetTicks(ms * Tps / 1000);
                link.Tick(clock.NowTicks);
            }

            Assert.Equal(3, socket.Sent.Count(b => JointStateCodec.TryDecodeSubscribe(b, out _, out _)));
            Assert.Equal(0, link.FramesReceived);
            Assert.True(double.IsPositiveInfinity(link.SecondsSinceLastFrame(clock.NowTicks)));
            Assert.True(link.Predicted(clock.NowTicks).IsEmpty);
        }

        [Fact]
        public void TheHeartbeat_SaysWhoAndWhichMode_OnceASecond_AndAtOnceOnAChange()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            var heard = new List<(long At, OperatorClientKind Kind, OperatorMotionMode Mode, ArmPowerRequest Request)>();
            for (int ms = 0; ms <= 2500; ms += 10)
            {
                clock.SetTicks(ms * Tps / 1000);
                if (ms == 1700) link.MotionOn = true;
                int before = socket.Sent.Count;
                link.Tick(clock.NowTicks);
                foreach (byte[] b in socket.Sent.Skip(before))
                {
                    if (ArmPowerCodec.TryDecodeRequest(b, out OperatorClientKind kind, out OperatorMotionMode mode, out ArmPowerRequest request, out _, out _))
                        heard.Add((ms, kind, mode, request));
                }
            }

            // At once, a second later, at the change, and then not again until a second after it (2.7 s).
            Assert.Equal(new long[] { 0, 1000, 1700 }, heard.Select(h => h.At).ToArray());
            Assert.All(heard, h => Assert.Equal(OperatorClientKind.Headset, h.Kind));
            Assert.All(heard, h => Assert.Equal(ArmPowerRequest.None, h.Request));
            Assert.Equal(OperatorMotionMode.Off, heard[1].Mode);
            Assert.Equal(OperatorMotionMode.On, heard[2].Mode);
        }

        [Fact]
        public void Frames_SyncTheClock_AndBecomeTheMeasuredAndPredictedArm()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            float[] values = Enumerable.Range(0, 16).Select(i => i % 8 == 7 ? 0.6f : -0.1f * (i % 8) - 0.05f).ToArray();
            uint sequence = 0;
            for (int ms = 0; ms <= 3000; ms += 20)
            {
                clock.SetTicks(ms * Tps / 1000);
                link.Tick(clock.NowTicks);
                if (ms % 500 == 0) Answer(profile, socket, clock.NowTicks, values, ++sequence);
            }

            Assert.True(link.IsSynced);
            Assert.True(link.FramesReceived >= 5);
            Assert.InRange(link.RoundTripMilliseconds, 3.0f, 5.0f);
            Assert.Equal(16, link.Measured.Length);

            var inOrder = new float[16];
            Assert.Equal(16, link.ToProfileOrder(link.Measured, inOrder));
            Assert.Equal(values, inOrder);

            ReadOnlySpan<JointSample> predicted = link.Predicted(clock.NowTicks);
            Assert.False(predicted.IsEmpty);
            var predictedInOrder = new float[16];
            Assert.Equal(16, link.ToProfileOrder(predicted, predictedInOrder));
            for (int i = 0; i < 16; i++) Assert.Equal(values[i], predictedInOrder[i], 5);
        }

        [Fact]
        public void ACommandSent_IsReplayedByFrankaReplay_TowardItsTarget()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            Assert.True(link.Bank.TrySetActive(1));
            float[] values = { 0, -0.785f, 0, -2.356f, 0, 1.571f, 0.785f, 1f, 0, -0.785f, 0, -2.356f, 0, 1.571f, -0.785f, 1f };
            uint sequence = 0;
            for (int ms = 0; ms <= 3000; ms += 20)
            {
                clock.SetTicks(ms * Tps / 1000);
                link.Tick(clock.NowTicks);
                if (ms % 100 == 0) Answer(profile, socket, clock.NowTicks, values, ++sequence);
            }

            Assert.True(link.IsSynced);

            // Joint 1 of the left arm commanded 0.3 rad further: 200 ms later the replay has it well on its way.
            var targets = new JointTarget[16];
            int k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint j in arm.Joints) targets[k] = new JointTarget(j.MotorId, values[k++], 0f);
                if (arm.HasGripper) targets[k] = new JointTarget(arm.Gripper.MotorId, values[k++], 0f);
            }

            targets[0] = new JointTarget(1, 0.3f, 0f);
            link.OnCommandSent(clock.NowTicks, targets);
            var inOrder = new float[16];
            link.ToProfileOrder(link.Predicted(clock.NowTicks + Tps / 5), inOrder);
            Assert.InRange(inOrder[0], 0.05f, 0.3f);
            Assert.Equal(values[1], inOrder[1], 4);
        }

        [Fact]
        public void AnAddedDelay_HoldsFramesBack_ButNotTheClock()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            link.AddedDelayMs = 200;
            float[] values = Enumerable.Repeat(0.25f, 16).ToArray();
            uint sequence = 0;
            for (int ms = 0; ms <= 2000; ms += 10)
            {
                clock.SetTicks(ms * Tps / 1000);
                link.Tick(clock.NowTicks);
                if (ms % 500 == 0) Answer(profile, socket, clock.NowTicks, values, ++sequence);
            }

            Assert.True(link.IsSynced); // the clock syncs on real arrivals

            // A new frame (other values) arrives 4 ms after it is answered: nothing changes for 200 ms, then it is the measurement.
            float[] next = Enumerable.Repeat(0.5f, 16).ToArray();
            long start = clock.NowTicks;
            Answer(profile, socket, start, next, ++sequence);
            var inOrder = new float[16];
            for (int ms = 0; ms <= 300; ms += 5)
            {
                clock.SetTicks(start + ms * Tps / 1000);
                link.Tick(clock.NowTicks);
                link.ToProfileOrder(link.Measured, inOrder);
                if (ms < 200) Assert.Equal(0.25f, inOrder[0]);
                else if (ms >= 210) Assert.Equal(0.5f, inOrder[0]);
            }

            link.AddedDelayMs = 5000;
            Assert.Equal(2000, link.AddedDelayMs); // clamped to 2 s
            link.AddedDelayMs = double.NaN;
            Assert.Equal(0, link.AddedDelayMs);
        }

        [Fact]
        public void EveryEnabledPreset_IsScoredLive_AndFreshReadingsAreTracked()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            Assert.True(double.IsPositiveInfinity(link.SecondsSinceFreshReading(0)));
            uint sequence = 0;
            var values = new float[16];
            for (int ms = 0; ms <= 6000; ms += 10)
            {
                clock.SetTicks(ms * Tps / 1000);
                // The arm moves: each joint drifts, so the presets' errors are not all zero.
                for (int i = 0; i < 16; i++) values[i] = i % 8 == 7 ? 1f : 0.1f * (float)Math.Sin(ms / 700.0 + i);
                link.Tick(clock.NowTicks);
                if (ms % 20 == 0) Answer(profile, socket, clock.NowTicks, values, ++sequence);
            }

            Assert.True(link.SecondsSinceFreshReading(clock.NowTicks) < 0.1);
            for (int i = 0; i < link.Bank.Count; i++)
            {
                Assert.True(link.Scorer.TryGetStats(i, out Teleop.Core.Metrics.LiveJointStats stats));
                Assert.True(stats.HasData, $"preset {link.Bank.LabelAt(i)} has no live numbers");
                Assert.True(stats.Readings > 100);
                Assert.False(float.IsNaN(stats.ErrorP95Deg));
            }
        }

        [Fact]
        public void TheHeadsetsConditions_GoAtOnceOnAChange_AndOnceASecond()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var socket = new FakeSocket();
            var clock = new ManualClock(Tps);
            SerialChainJointLink link = Link(profile, socket, clock);
            var heard = new List<(int Ms, HeadsetConditions Conditions)>();
            for (int ms = 0; ms <= 2500; ms += 10)
            {
                clock.SetTicks(ms * Tps / 1000);
                if (ms == 1300) link.SetHeadsetConditions(new HeadsetConditions(150f, 20f, 0.5f, 0f, 7UL));
                int before = socket.Sent.Count;
                link.Tick(clock.NowTicks);
                foreach (byte[] b in socket.Sent.Skip(before))
                {
                    if (HeadsetConditionsCodec.TryDecode(b, out HeadsetConditions c, out _, out _)) heard.Add((ms, c));
                }
            }

            // Off at once and a second later, the disturbance the moment it is set, then a second after that.
            Assert.Equal(new[] { 0, 1000, 1300, 2300 }, heard.Select(h => h.Ms).ToArray());
            Assert.Equal(0f, heard[1].Conditions.LagMs);
            Assert.Equal(150f, heard[2].Conditions.LagMs);
            Assert.Equal(7UL, heard[3].Conditions.Seed);
        }

        [Fact]
        public void TickingWithNothingArriving_AllocatesNothing()
        {
            SerialChainProfile profile = SerialChainKinematicsTests.LoadFr3Duo();
            var clock = new ManualClock(Tps);
            var socket = new FakeSocket();
            SerialChainJointLink link = Link(profile, socket, clock);
            link.Tick(0);
            long now = 1;
            // Inside one keepalive interval, so the fake socket's own list never grows.
            AllocationAssert.Zero(() => link.Tick(now++), iterations: 1000);
        }
    }
}
