using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.Eval.ArmPower;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.Tests.ArmPower
{
    /// <summary>`arm-power` against a fake RobotHost on loopback: it answers each request as a script says.</summary>
    public class ArmPowerCommandTests
    {
        private sealed class FakeRobotHost : IDisposable
        {
            private readonly UdpClient _udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
            private readonly Thread _thread;
            private volatile bool _stop;

            public FakeRobotHost(Func<int, ArmPowerRequest, (ArmPowerState, ArmPowerReason, float)> answer)
            {
                _udp.Client.ReceiveTimeout = 100;
                _thread = new Thread(() =>
                {
                    int n = 0;
                    var buffer = new byte[ArmPowerCodec.StatusSize];
                    while (!_stop)
                    {
                        IPEndPoint from = new IPEndPoint(IPAddress.Any, 0);
                        byte[] data;
                        try { data = _udp.Receive(ref from); }
                        catch (SocketException) { continue; }
                        if (!ArmPowerCodec.TryDecodeRequest(data, out OperatorClientKind kind, out _, out ArmPowerRequest request, out uint seq, out long ticks)) continue;
                        Kinds |= kind == OperatorClientKind.Tool ? 1 : 2;
                        (ArmPowerState state, ArmPowerReason reason, float remaining) = answer(n++, request);
                        ArmPowerCodec.TryEncodeStatus(state, reason, seq, ticks, 0, remaining, buffer, out int written);
                        _udp.Send(buffer, written, from);
                    }
                }) { IsBackground = true };
                _thread.Start();
            }

            public int Port => ((IPEndPoint)_udp.Client.LocalEndPoint!).Port;

            /// <summary>Bit 0: a tool sent; bit 1: anything else did.</summary>
            public int Kinds;

            public void Dispose()
            {
                _stop = true;
                _thread.Join();
                _udp.Dispose();
            }
        }

        private static int Run(FakeRobotHost robot, string request) =>
            ArmPowerCommand.Run(new[] { "arm-power", "--remote-host", "127.0.0.1", "--remote-port", robot.Port.ToString(), "--request", request, "--timeout-seconds", "5" });

        [Fact]
        public void Relax_WaitsThroughTheParkUntilRelaxed_AsATool()
        {
            using var robot = new FakeRobotHost((n, r) => n < 3 ? (ArmPowerState.Parking, ArmPowerReason.None, 20f - 5f * n) : (ArmPowerState.Relaxed, ArmPowerReason.None, 0f));
            Assert.Equal(0, Run(robot, "relax"));
            Assert.Equal(1, robot.Kinds); // never as a headset: a tool opens no automatic recording
        }

        [Fact]
        public void Relax_Refused_ExitsNonZero()
        {
            using var robot = new FakeRobotHost((n, r) => (ArmPowerState.Holding, ArmPowerReason.BoardNotAnswering, float.NaN));
            Assert.Equal(1, Run(robot, "relax"));
        }

        [Fact]
        public void Hold_EndsWhenHolding()
        {
            using var robot = new FakeRobotHost((n, r) => r == ArmPowerRequest.Hold ? (ArmPowerState.Holding, ArmPowerReason.None, float.NaN) : (ArmPowerState.Relaxed, ArmPowerReason.None, 0f));
            Assert.Equal(0, Run(robot, "hold"));
        }

        [Fact]
        public void NoRobot_TimesOutNonZero()
        {
            using var robot = new FakeRobotHost((n, r) => (ArmPowerState.Parking, ArmPowerReason.None, 10f));
            Assert.Equal(1, ArmPowerCommand.Run(new[] { "arm-power", "--remote-host", "127.0.0.1", "--remote-port", robot.Port.ToString(), "--request", "relax", "--timeout-seconds", "1" }));
        }

        [Fact]
        public void ABadRequest_IsAUsageError()
        {
            Assert.Equal(64, ArmPowerCommand.Run(new[] { "arm-power", "--remote-host", "127.0.0.1", "--request", "park" }));
        }
    }
}
