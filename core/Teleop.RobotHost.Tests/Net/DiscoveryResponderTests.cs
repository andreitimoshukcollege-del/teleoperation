using System.Net;
using System.Net.Sockets;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    public class DiscoveryResponderTests
    {
        [Fact]
        public void TheFilter_KeepsOnlyAddressesTheLanReaches()
        {
            var kept = DiscoveryResponder.Filter(new[]
            {
                ("wlan0", IPAddress.Parse("10.188.58.137")),
                ("eth0", IPAddress.Parse("192.168.1.20")),
                ("lo", IPAddress.Loopback),
                ("tailscale0", IPAddress.Parse("100.112.90.72")),
                ("wg0", IPAddress.Parse("100.100.1.1")),
                ("docker0", IPAddress.Parse("172.17.0.1")),
                ("br-1a2b", IPAddress.Parse("172.18.0.1")),
                ("eth1", IPAddress.Parse("169.254.3.4")),
                ("wlan0", IPAddress.Parse("fe80::1")),
                ("wlan0", IPAddress.Parse("10.188.58.137")),
            });

            Assert.Equal(new[] { IPAddress.Parse("10.188.58.137"), IPAddress.Parse("192.168.1.20") }, kept);
        }

        [Fact]
        public void AQuery_IsAnsweredWithTheNameTheAddressesAndItsNonce()
        {
            var lan = new[] { IPAddress.Parse("10.188.58.137") };
            using var responder = new DiscoveryResponder(0, () => lan, "jetrover-sinrg");
            using var client = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp) { ReceiveTimeout = 2000 };
            client.Bind(new IPEndPoint(IPAddress.Loopback, 0));
            var query = new byte[RobotDiscoveryCodec.QuerySize];
            RobotDiscoveryCodec.TryEncodeQuery(77, query, out _);
            client.SendTo(query, new IPEndPoint(IPAddress.Loopback, responder.Port));
            client.SendTo(new byte[] { 1, 2, 3 }, new IPEndPoint(IPAddress.Loopback, responder.Port)); // ignored

            int answered = 0;
            for (int i = 0; i < 200 && answered == 0; i++)
            {
                answered = responder.Poll();
                if (answered == 0) System.Threading.Thread.Sleep(5);
            }

            Assert.Equal(1, answered);
            var reply = new byte[RobotDiscoveryCodec.MaxReplySize];
            int n = client.Receive(reply);
            var addresses = new uint[RobotDiscoveryCodec.MaxAddresses];
            Assert.True(RobotDiscoveryCodec.TryDecodeReply(reply.AsSpan(0, n), out uint nonce, addresses, out int count, out string name));
            Assert.Equal(77u, nonce);
            Assert.Equal("jetrover-sinrg", name);
            Assert.Equal(1, count);
            Assert.Equal(lan[0], RobotDiscoveryCodec.ToAddress(addresses[0]));
        }

        [Fact]
        public void TheRealInterfaces_CanBeRead()
        {
            Assert.All(DiscoveryResponder.LanAddresses(), a => Assert.False(IPAddress.IsLoopback(a)));
        }
    }
}
