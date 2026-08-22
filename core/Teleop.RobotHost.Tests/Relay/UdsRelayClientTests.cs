using System.IO;
using System.Net.Sockets;
using System.Threading;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Tests.Relay
{
    public class UdsRelayClientTests
    {
        private static string TempSocketPath() =>
            Path.Combine(Path.GetTempPath(), $"teleop-robothost-test-{Path.GetRandomFileName()}.sock");

        [LinuxOnlyFact]
        public void Send_DeliversCommandToPeerSocket()
        {
            string hostPath = TempSocketPath();
            string relayPath = TempSocketPath();

            try
            {
                using var relaySocket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
                relaySocket.Bind(new UnixDomainSocketEndPoint(relayPath));
                relaySocket.Blocking = false;

                using var client = new UdsRelayClient(hostPath, relayPath);
                client.Send(new[] { new JointTarget(motorId: 1, angle: 2.5f, speed: 300f) });

                byte[] buffer = new byte[RelayProtocol.CommandEncodedSize(RelayProtocol.MaxJointsPerMessage)];
                int received = PollReceive(relaySocket, buffer);

                Span<JointTarget> decodeBuffer = stackalloc JointTarget[RelayProtocol.MaxJointsPerMessage];
                Assert.True(RelayProtocol.TryDecodeCommand(buffer.AsSpan(0, received), decodeBuffer, out int targetCount));
                Assert.Equal(1, targetCount);
                Assert.Equal(2.5f, decodeBuffer[0].Angle);
            }
            finally
            {
                TryDelete(hostPath);
                TryDelete(relayPath);
            }
        }

        [LinuxOnlyFact]
        public void TryReceiveFeedback_ReturnsDecodedFeedbackSentByRelay()
        {
            string hostPath = TempSocketPath();
            string relayPath = TempSocketPath();

            try
            {
                using var relaySocket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
                relaySocket.Bind(new UnixDomainSocketEndPoint(relayPath));

                using var client = new UdsRelayClient(hostPath, relayPath);

                var sentFeedback = new[]
                {
                    new JointFeedbackEntry(motorId: 1, valid: true, pulse: 17f),
                    new JointFeedbackEntry(motorId: 2, valid: false, pulse: 0f),
                    new JointFeedbackEntry(motorId: 3, valid: true, pulse: -3f),
                };
                byte[] buffer = new byte[RelayProtocol.FeedbackEncodedSize(sentFeedback.Length)];
                RelayProtocol.EncodeFeedback(sentFeedback, buffer);
                relaySocket.SendTo(buffer, new UnixDomainSocketEndPoint(hostPath));

                bool received = false;
                Span<JointFeedbackEntry> entriesBuffer = new JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
                int entryCount = 0;
                for (int attempt = 0; attempt < 50 && !received; attempt++)
                {
                    received = client.TryReceiveFeedback(entriesBuffer, out entryCount);
                    if (!received)
                    {
                        Thread.Sleep(10);
                    }
                }

                Assert.True(received);
                Assert.Equal(3, entryCount);
                Assert.True(entriesBuffer[0].Valid);
                Assert.Equal(17f, entriesBuffer[0].Pulse);
                Assert.False(entriesBuffer[1].Valid);
                Assert.Equal(-3f, entriesBuffer[2].Pulse);
            }
            finally
            {
                TryDelete(hostPath);
                TryDelete(relayPath);
            }
        }

        [LinuxOnlyFact]
        public void Construction_RemovesStaleSocketFileLeftBehindByACrashedPreviousRun()
        {
            string hostPath = TempSocketPath();
            string relayPath = TempSocketPath();
            File.WriteAllText(hostPath, "stale, not a real socket");

            try
            {
                using var client = new UdsRelayClient(hostPath, relayPath);
                // Construction succeeding at all (no exception binding to hostPath) is the assertion.
            }
            finally
            {
                TryDelete(hostPath);
                TryDelete(relayPath);
            }
        }

        private static int PollReceive(Socket socket, byte[] buffer)
        {
            for (int attempt = 0; attempt < 50; attempt++)
            {
                if (socket.Available > 0)
                {
                    return socket.Receive(buffer);
                }
                Thread.Sleep(10);
            }

            throw new TimeoutException("No datagram arrived within the polling window.");
        }

        private static void TryDelete(string path)
        {
            try
            {
                File.Delete(path);
            }
            catch (IOException)
            {
            }
        }
    }
}
