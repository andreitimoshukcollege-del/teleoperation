using System;
using Teleop.Core.Transport;
using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    /// <summary>
    /// The robot-side uplink impairment (docs/adr/0021): a pass-through when off; Core's profile, on
    /// the receive side only, when on; a typo refused rather than recorded as an impaired session.
    /// </summary>
    public class UplinkImpairedReceiverTests
    {
        private const long Tps = 1000; // ticks are milliseconds

        private static (LoopbackTransport socket, UplinkImpairedReceiver receiver) Make()
        {
            var socket = new LoopbackTransport(64, 256);
            return (socket, new UplinkImpairedReceiver(socket, Tps, 64));
        }

        [Fact]
        public void Off_EverythingPassesStraightThrough()
        {
            var (socket, receiver) = Make();
            socket.Send(new byte[] { 1, 2, 3 }, 10);
            var buffer = new byte[64];
            Assert.True(receiver.TryReceive(10, buffer, out int n, out long read, out long delivered));
            Assert.Equal(3, n);
            Assert.Equal(new byte[] { 1, 2, 3 }, buffer.AsSpan(0, 3).ToArray());
            Assert.Equal(10, read);
            Assert.Equal(10, delivered);
            Assert.Equal("none", receiver.Profile);
        }

        [Fact]
        public void On_ADatagramWaitsForTheProfile_AndKeepsWhenItWasRead()
        {
            var (socket, receiver) = Make();
            receiver.SetProfile("delay-100ms", 7); // 100 ms, +-5 ms of jitter
            socket.Send(new byte[] { 9 }, 0);
            var buffer = new byte[64];
            Assert.False(receiver.TryReceive(50, buffer, out _, out _, out _));
            Assert.True(receiver.TryReceive(110, buffer, out int n, out long read, out long delivered));
            Assert.Equal(1, n);
            Assert.Equal(9, buffer[0]);
            Assert.Equal(0, read);
            Assert.Equal(110, delivered);
        }

        [Fact]
        public void ALossyProfile_DropsSomeCommands()
        {
            var (socket, receiver) = Make();
            receiver.SetProfile("loss-50pct", 3);
            var buffer = new byte[64];
            int received = 0;
            for (int i = 0; i < 200; i++)
            {
                socket.Send(new byte[] { (byte)i }, i);
                while (receiver.TryReceive(i, buffer, out _, out _, out _)) received++;
            }

            while (receiver.TryReceive(10_000, buffer, out _, out _, out _)) received++;
            Assert.InRange(received, 50, 150);
        }

        [Fact]
        public void AnUnknownProfile_IsRefused_AndTheOldOneStays()
        {
            var (_, receiver) = Make();
            receiver.SetProfile("50ms-5j", 1);
            Assert.Throws<ArgumentException>(() => receiver.SetProfile("50ms-5jj", 1));
            Assert.Equal("50ms-5j", receiver.Profile);
            receiver.SetProfile("lan", 1);
            Assert.Equal("none", receiver.Profile);
        }

        [Fact]
        public void SwitchingProfiles_IsAnOutageForWhatWasInFlight()
        {
            var (socket, receiver) = Make();
            receiver.SetProfile("delay-100ms", 1);
            socket.Send(new byte[] { 1 }, 0);
            var buffer = new byte[64];
            Assert.False(receiver.TryReceive(10, buffer, out _, out _, out _)); // now held by the emulator
            receiver.SetProfile("none", 1);
            Assert.False(receiver.TryReceive(500, buffer, out _, out _, out _));
        }
    }
}
