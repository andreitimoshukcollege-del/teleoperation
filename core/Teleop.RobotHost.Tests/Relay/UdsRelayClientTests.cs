using System.IO;
using System.Net.Sockets;
using System.Threading;
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
                client.Send(new LocalArmCommand(baseDirection: 2.5f));

                byte[] buffer = new byte[RelayProtocol.ArmCommandEncodedSize];
                int received = PollReceive(relaySocket, buffer);

                Assert.True(RelayProtocol.TryDecodeCommand(buffer.AsSpan(0, received), out LocalArmCommand decoded));
                Assert.Equal(2.5f, decoded.BaseDirection);
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

                byte[] buffer = new byte[RelayProtocol.FeedbackEncodedSize];
                RelayProtocol.EncodeFeedback(new LocalFeedback(baseDegreesValid: true, baseDegrees: 17), buffer);
                relaySocket.SendTo(buffer, new UnixDomainSocketEndPoint(hostPath));

                bool received = false;
                LocalFeedback feedback = default;
                for (int attempt = 0; attempt < 50 && !received; attempt++)
                {
                    received = client.TryReceiveFeedback(out feedback);
                    if (!received)
                    {
                        Thread.Sleep(10);
                    }
                }

                Assert.True(received);
                Assert.True(feedback.BaseDegreesValid);
                Assert.Equal(17, feedback.BaseDegrees);
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
