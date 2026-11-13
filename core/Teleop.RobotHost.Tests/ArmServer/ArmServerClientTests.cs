using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using System.Threading;
using Teleop.Core.Types;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Tests.Relay;

namespace Teleop.RobotHost.Tests.ArmServer
{
    /// <summary>RobotHost's side of the arm-server socket (docs/adr/0030 decision 4), against a scripted server.</summary>
    public class ArmServerClientTests
    {
        private static readonly byte[] Motors = { 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 13, 14, 15, 16, 17, 18 };
        private const long Tps = 1_000_000_000;

        /// <summary>An in-memory link whose far side is a scripted server: it answers a Hello with a Welcome and a State.</summary>
        private sealed class ScriptedServer : IArmServerLink
        {
            private readonly Queue<byte[]> _toClient = new Queue<byte[]>();
            public readonly List<byte[]> FromClient = new List<byte[]>();
            public ArmServerKind Kind = ArmServerKind.Isaac;
            public byte[] WelcomeMotors = Motors;
            public long ClockOffsetTicks;
            public Func<long> Now = () => 0;
            public bool Answers = true;

            public void Send(ReadOnlySpan<byte> datagram)
            {
                FromClient.Add(datagram.ToArray());
                if (!Answers || ArmServerWire.KindOf(datagram) != ArmServerWire.KindHello) return;
                var b = new byte[ArmServerWire.MaxBytes];
                ArmServerWire.TryEncodeWelcome(1, Kind, Now() + ClockOffsetTicks, WelcomeMotors, "scripted", b, out int n);
                _toClient.Enqueue(b.AsSpan(0, n).ToArray());
                PushState(Now(), WelcomeMotors.Select(m => m * 0.01f).ToArray());
            }

            public void PushState(long sensed, float[] positions, byte[]? ids = null)
            {
                ids ??= WelcomeMotors;
                var b = new byte[ArmServerWire.MaxBytes];
                var zeros = new float[ids.Length];
                ArmServerWire.TryEncodeState(2, new ArmServerStateHeader(sensed, sensed, ArmServerWire.FlagEnabled), ids, positions, zeros, positions, b, out int n);
                _toClient.Enqueue(b.AsSpan(0, n).ToArray());
            }

            public void PushHealth(float factor)
            {
                var b = new byte[ArmServerWire.MaxBytes];
                ArmServerWire.TryEncodeHealth(3, new ArmServerHealth(factor, 0.1f, 2, 5000), b, out int n);
                _toClient.Enqueue(b.AsSpan(0, n).ToArray());
            }

            public bool TryReceive(Span<byte> buffer, out int bytes)
            {
                bytes = 0;
                if (_toClient.Count == 0) return false;
                byte[] d = _toClient.Dequeue();
                d.CopyTo(buffer);
                bytes = d.Length;
                return true;
            }

            public void Dispose()
            {
            }
        }

        private static (ArmServerClient client, ScriptedServer server, Func<long> now) Connected(Action<ScriptedServer>? setUp = null)
        {
            long t = 5 * Tps;
            Func<long> now = () => t += 1_000_000;   // a millisecond per look at the clock
            var server = new ScriptedServer { Now = () => t };
            setUp?.Invoke(server);
            var client = new ArmServerClient(server, Motors, Tps);
            return (client, server, now);
        }

        [Fact]
        public void Connecting_TakesTheServersKindAndTheArmsPose()
        {
            var (client, server, now) = Connected();
            Assert.True(client.Connect(now, "fr3-duo", 1.0, null, out string error), error);
            Assert.Equal(ArmServerKind.Isaac, client.Kind);
            Assert.Equal("scripted", client.Description);
            Assert.Equal(Motors.Select(m => m * 0.01f), client.Positions.ToArray());
            Assert.True(ArmServerWire.TryDecodeHello(server.FromClient[0], out _, out long tps, out ReadOnlySpan<byte> name));
            Assert.Equal(Tps, tps);
            Assert.Equal("fr3-duo", System.Text.Encoding.UTF8.GetString(name));
        }

        [Fact]
        public void AServerWithOtherMotors_OrAnotherClock_OrNoAnswer_IsRefused()
        {
            var (client, _, now) = Connected(s => s.WelcomeMotors = Motors.Reverse().ToArray());
            Assert.False(client.Connect(now, "fr3-duo", 1.0, null, out string error));
            Assert.Contains("not this profile's", error);

            (client, _, now) = Connected(s => s.ClockOffsetTicks = Tps); // a second apart: another machine's clock
            Assert.False(client.Connect(now, "fr3-duo", 1.0, null, out error));
            Assert.Contains("clock", error);

            (client, _, now) = Connected(s => s.Answers = false);
            Assert.False(client.Connect(now, "fr3-duo", 0.2, null, out error));
            Assert.Contains("no arm server answered", error);
        }

        [Fact]
        public void Targets_GoOnlyWhileEnabled_AndStatesAreTheTruth()
        {
            var (client, server, now) = Connected();
            Assert.True(client.Connect(now, "fr3-duo", 1.0, null, out string error), error);
            var truth = new List<(long, double[])>();
            client.TruthSink = (sensed, q) => truth.Add((sensed, (double[])q.Clone()));

            int before = server.FromClient.Count;
            client.Send(new[] { new JointSample(1, 0.5f, false) }, now());
            Assert.Equal(ArmServerWire.KindTargets, ArmServerWire.KindOf(server.FromClient.Last()));
            client.SetEnabled(false, now());
            Assert.Equal(ArmServerWire.KindEnable, ArmServerWire.KindOf(server.FromClient.Last()));
            client.Send(new[] { new JointSample(1, 0.6f, false) }, now());
            Assert.Equal(before + 2, server.FromClient.Count); // the target after disabling was not sent

            float[] q = Motors.Select(m => -m * 0.02f).ToArray();
            server.PushState(123_000, q);
            server.PushHealth(3.5f);
            var output = new JointSample[Motors.Length];
            Assert.Equal(Motors.Length, client.ReadState(now(), output, out long sensed));
            Assert.Equal(123_000, sensed);
            Assert.Equal(q, output.Select(s => s.Value));
            Assert.Equal(Motors, output.Select(s => s.MotorId));
            Assert.Single(truth);
            Assert.Equal(q.Select(x => (double)x), truth[0].Item2);
            Assert.Equal(3.5f, client.LastHealth.RealTimeFactor);
            Assert.Contains("3.5x real time", client.Summary);
        }

        [Fact]
        public void AStateNamingAMotorThisHostLacks_IsRejected_AndChangesNothing()
        {
            var (client, server, now) = Connected();
            Assert.True(client.Connect(now, "fr3-duo", 1.0, null, out string error), error);
            float[] before = client.Positions.ToArray();
            server.PushState(1, new[] { 9f, 9f }, new byte[] { 1, 99 });
            client.Poll(now());
            Assert.Equal(1, client.Rejected);
            Assert.Equal(before, client.Positions.ToArray());
        }

        [LinuxOnlyFact]
        public void TheUnixSocketLink_CarriesDatagramsBothWays()
        {
            string serverPath = Path.Combine(Path.GetTempPath(), $"teleop-armserver-{Path.GetRandomFileName()}.sock");
            string clientPath = serverPath + ".robothost";
            try
            {
                using var serverSocket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
                serverSocket.Bind(new UnixDomainSocketEndPoint(serverPath));
                using var link = new UdsArmServerLink(serverPath, clientPath);
                link.Send(new byte[] { 1, 2, 3 });
                var buffer = new byte[16];
                System.Net.EndPoint from = new UnixDomainSocketEndPoint(clientPath);
                serverSocket.ReceiveTimeout = 2000;
                int n = serverSocket.ReceiveFrom(buffer, ref from);
                Assert.Equal(new byte[] { 1, 2, 3 }, buffer.Take(n));

                serverSocket.SendTo(new byte[] { 4, 5 }, new UnixDomainSocketEndPoint(clientPath));
                int got = 0;
                for (int i = 0; i < 200 && !link.TryReceive(buffer, out got); i++) Thread.Sleep(5);
                Assert.Equal(new byte[] { 4, 5 }, buffer.Take(got));
            }
            finally
            {
                if (File.Exists(serverPath)) File.Delete(serverPath);
                Assert.False(File.Exists(clientPath), "the link left its socket file behind");
            }
        }
    }
}
