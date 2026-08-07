using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotHost.Net;
using Teleop.RobotHost.Time;

namespace Teleop.RobotHost.Tests.Net
{
    public class UdpTransportTests
    {
        private static int GetFreeUdpPort()
        {
            using var probe = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            probe.Bind(new IPEndPoint(IPAddress.Loopback, 0));
            return ((IPEndPoint)probe.LocalEndPoint!).Port;
        }

        [Fact]
        public void SendAndReceive_RoundTripOverRealLoopbackSockets()
        {
            int portA = GetFreeUdpPort();
            int portB = GetFreeUdpPort();
            var clock = new MonotonicClock();

            using var transportA = new UdpTransport(portA, new IPEndPoint(IPAddress.Loopback, portB), maxPayloadBytes: 64, clock);
            using var transportB = new UdpTransport(portB, new IPEndPoint(IPAddress.Loopback, portA), maxPayloadBytes: 64, clock);

            byte[] payload = { 1, 2, 3, 4, 5 };
            Assert.True(transportA.Send(payload, clock.NowTicks));

            // Real network I/O, even over loopback, is not instantaneous -- poll briefly rather
            // than asserting delivery on the very first TryReceive call.
            Span<byte> destination = stackalloc byte[64];
            bool received = false;
            int byteCount = 0;
            long arrivalTicks = 0;
            for (int attempt = 0; attempt < 50 && !received; attempt++)
            {
                received = transportB.TryReceive(clock.NowTicks, destination, out byteCount, out arrivalTicks);
                if (!received)
                {
                    Thread.Sleep(10);
                }
            }

            Assert.True(received);
            Assert.Equal(payload.Length, byteCount);
            Assert.Equal(payload, destination.Slice(0, byteCount).ToArray());
            Assert.True(arrivalTicks > 0);
        }

        [Fact]
        public void TryReceive_ReturnsFalse_WhenNothingHasArrived()
        {
            int port = GetFreeUdpPort();
            var clock = new MonotonicClock();
            using var transport = new UdpTransport(port, new IPEndPoint(IPAddress.Loopback, GetFreeUdpPort()), maxPayloadBytes: 64, clock);

            Span<byte> destination = stackalloc byte[64];
            bool received = transport.TryReceive(clock.NowTicks, destination, out int byteCount, out long arrivalTicks);

            Assert.False(received);
            Assert.Equal(0, byteCount);
            Assert.Equal(0, arrivalTicks);
        }

        [Fact]
        public void Send_RejectsPayloadLargerThanMaxPayloadBytes()
        {
            int port = GetFreeUdpPort();
            var clock = new MonotonicClock();
            using var transport = new UdpTransport(port, new IPEndPoint(IPAddress.Loopback, GetFreeUdpPort()), maxPayloadBytes: 4, clock);

            bool sent = transport.Send(new byte[] { 1, 2, 3, 4, 5 }, clock.NowTicks);

            Assert.False(sent);
        }

        [Fact]
        public void Reset_DrainsPendingDatagrams()
        {
            int portA = GetFreeUdpPort();
            int portB = GetFreeUdpPort();
            var clock = new MonotonicClock();

            using var transportA = new UdpTransport(portA, new IPEndPoint(IPAddress.Loopback, portB), maxPayloadBytes: 64, clock);
            using var transportB = new UdpTransport(portB, new IPEndPoint(IPAddress.Loopback, portA), maxPayloadBytes: 64, clock);

            transportA.Send(new byte[] { 9, 9 }, clock.NowTicks);

            // Give the datagram a moment to actually arrive in transportB's receive buffer
            // before Reset is expected to drain it.
            Thread.Sleep(50);
            transportB.Reset();

            Span<byte> destination = stackalloc byte[64];
            bool received = transportB.TryReceive(clock.NowTicks, destination, out _, out _);
            Assert.False(received);
        }
    }
}
