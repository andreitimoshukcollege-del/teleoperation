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
