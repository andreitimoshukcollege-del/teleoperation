using System;
using System.Net;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    public class RobotDiscoveryTests
    {
        private static readonly IPAddress Tailscale = IPAddress.Parse("100.112.90.72");
        private static readonly IPAddress Lan = IPAddress.Parse("10.188.58.137");
        private static readonly IPAddress OldLan = IPAddress.Parse("10.188.57.244");

        private static byte[] Reply(uint nonce, params IPAddress[] addresses)
        {
            var buffer = new byte[RobotDiscoveryCodec.MaxReplySize];
            Assert.True(RobotDiscoveryCodec.TryEncodeReply(nonce, addresses, "jetrover-sinrg", buffer, out int n));
            return buffer.AsSpan(0, n).ToArray();
        }

        [Fact]
        public void AQuery_RoundTrips()
        {
            var buffer = new byte[RobotDiscoveryCodec.QuerySize];
            Assert.True(RobotDiscoveryCodec.TryEncodeQuery(0xDEADBEEF, buffer, out int n));
            Assert.Equal(8, n);
            Assert.True(RobotDiscoveryCodec.TryDecodeQuery(buffer, out uint nonce));
            Assert.Equal(0xDEADBEEF, nonce);
            Assert.False(RobotDiscoveryCodec.TryDecodeReply(buffer, out _, new uint[8], out _, out _));
        }

        [Fact]
        public void AReply_RoundTrips_AndSkipsWhatItCannotCarry()
        {
            byte[] reply = Reply(42, Lan, IPAddress.IPv6Loopback, IPAddress.Parse("192.168.1.5"));
            var addresses = new uint[RobotDiscoveryCodec.MaxAddresses];
            Assert.True(RobotDiscoveryCodec.TryDecodeReply(reply, out uint nonce, addresses, out int count, out string name));
            Assert.Equal(42u, nonce);
            Assert.Equal(2, count);
            Assert.Equal(Lan, RobotDiscoveryCodec.ToAddress(addresses[0]));
            Assert.Equal(IPAddress.Parse("192.168.1.5"), RobotDiscoveryCodec.ToAddress(addresses[1]));
            Assert.Equal("jetrover-sinrg", name);
            Assert.False(RobotDiscoveryCodec.TryDecodeQuery(reply, out _));
        }

        [Fact]
        public void TruncatedOrForeignBytes_AreRefused()
        {
            byte[] reply = Reply(7, Lan);
            var addresses = new uint[RobotDiscoveryCodec.MaxAddresses];
            for (int length = 0; length < reply.Length; length++)
            {
                Assert.False(RobotDiscoveryCodec.TryDecodeReply(reply.AsSpan(0, length), out _, addresses, out _, out _), $"length {length}");
            }

            reply[0] ^= 0xFF;
            Assert.False(RobotDiscoveryCodec.TryDecodeReply(reply, out _, addresses, out _, out _));
        }

        /// <summary>Runs the resolver against a scripted robot: who answers, after how long, from where.</summary>
        private static RobotAddressResolver Run(IPAddress? tailscale, IPAddress? configured,
            Func<IPAddress, IPAddress?> answersFrom, Func<IPAddress, IPAddress[]>? reports = null, double replyDelay = 0.02,
            string? mdnsName = null, IPAddress? mdnsAnswer = null)
        {
            const uint nonce = 1234;
            var r = new RobotAddressResolver(tailscale, configured, mdnsName, nonce);
            var pending = new System.Collections.Generic.List<(double at, IPAddress from, byte[] bytes)>();
            for (double t = 0; t < 5 && r.Source == RobotAddressSource.Pending; t += 0.005)
            {
                while (r.NextQuery(t, out IPAddress to, out RobotQueryKind kind))
                {
                    if (kind == RobotQueryKind.Mdns)
                    {
                        if (mdnsAnswer != null) pending.Add((t + replyDelay, mdnsAnswer, MdnsCodecTests.Answer(r.MdnsId, mdnsName!, mdnsAnswer)));
                        continue;
                    }

                    IPAddress? from = answersFrom(to);
                    if (from != null) pending.Add((t + replyDelay, from, Reply(nonce, reports?.Invoke(from) ?? new[] { Lan })));
                }

                foreach (var p in pending.ToArray())
                {
                    if (p.at <= t)
                    {
                        r.OnDatagram(t, p.from, p.bytes);
                        pending.Remove(p);
                    }
                }
            }

            return r;
        }

        [Fact]
        public void OnTheSameNetwork_ItUsesTheLanAddressTailscaleReported()
        {
            RobotAddressResolver r = Run(Tailscale, OldLan, to => to.Equals(Tailscale) ? Tailscale : to.Equals(Lan) ? Lan : null);
            Assert.Equal(RobotAddressSource.Lan, r.Source);
            Assert.Equal(Lan, r.Address);
            Assert.Equal("jetrover-sinrg", r.RobotName);
            Assert.Equal(new[] { Lan }, r.ReportedLanAddresses);
        }

        [Fact]
        public void OnAnotherNetwork_ItUsesTailscale()
        {
            RobotAddressResolver r = Run(Tailscale, OldLan, to => to.Equals(Tailscale) ? Tailscale : null);
            Assert.Equal(RobotAddressSource.Tailscale, r.Source);
            Assert.Equal(Tailscale, r.Address);
        }

        [Fact]
        public void WithoutTailscale_MdnsFindsTheNewLease()
        {
            // The configured address is yesterday's lease; the robot's mDNS responder gives today's.
            RobotAddressResolver r = Run(Tailscale, OldLan, _ => null, mdnsName: "jetrover-sinrg.local", mdnsAnswer: Lan);
            Assert.Equal(RobotAddressSource.Lan, r.Source);
            Assert.Equal(Lan, r.Address);
            Assert.Contains(r.Log, line => line.Contains("mDNS"));
        }

        [Fact]
        public void AnMdnsAnswerWithDockersAddress_IsNotTaken()
        {
            RobotAddressResolver r = Run(null, OldLan, _ => null, mdnsName: "jetrover-sinrg.local", mdnsAnswer: IPAddress.Parse("172.17.0.1"));
            Assert.Equal(RobotAddressSource.Configured, r.Source);
            Assert.Equal(OldLan, r.Address);
        }

        [Fact]
        public void WithoutTailscaleOrBroadcast_TheConfiguredAddressStillAnswers()
        {
            RobotAddressResolver r = Run(Tailscale, Lan, to => to.Equals(Lan) ? Lan : null);
            Assert.Equal(RobotAddressSource.Lan, r.Source);
            Assert.Equal(Lan, r.Address);
        }

        [Fact]
        public void WhenNothingAnswers_ItFallsBackToTheConfiguredAddress_OrReportsNothing()
        {
            RobotAddressResolver r = Run(Tailscale, OldLan, _ => null);
            Assert.Equal(RobotAddressSource.Configured, r.Source);
            Assert.Equal(OldLan, r.Address);
            Assert.Contains(r.Log, line => line.Contains("no answer through Tailscale"));

            RobotAddressResolver none = Run(null, null, _ => null, mdnsName: "jetrover-sinrg.local");
            Assert.Equal(RobotAddressSource.NotFound, none.Source);
            Assert.Null(none.Address);
        }

        [Fact]
        public void ATailscaleAnswerWithNoLanAddress_IsTailscale()
        {
            RobotAddressResolver r = Run(Tailscale, null, to => to.Equals(Tailscale) ? Tailscale : null, _ => Array.Empty<IPAddress>());
            Assert.Equal(RobotAddressSource.Tailscale, r.Source);
        }

        [Fact]
        public void RepliesWithTheWrongNonce_OrFromElsewhere_AreIgnored()
        {
            var r = new RobotAddressResolver(Tailscale, null, null, nonce: 5);
            Assert.True(r.NextQuery(0, out IPAddress to, out _));
            Assert.Equal(Tailscale, to);
            r.OnDatagram(0.01, Tailscale, Reply(6, Lan));
            r.OnDatagram(0.01, Lan, Reply(5, Lan));
            Assert.Equal(RobotAddressSource.Pending, r.Source);
            r.OnDatagram(0.02, Tailscale, Reply(5, Lan));
            Assert.Equal(RobotAddressSource.Pending, r.Source); // now probing the LAN address
            Assert.True(r.NextQuery(0.03, out to, out _));
            Assert.Equal(Lan, to);
        }

        [Fact]
        public void TheWorstCase_TakesAboutTwoAndAHalfSeconds()
        {
            var r = new RobotAddressResolver(Tailscale, OldLan, "jetrover-sinrg.local", 1);
            double t = 0;
            for (; r.Source == RobotAddressSource.Pending; t += 0.005) r.NextQuery(t, out _, out _);
            Assert.InRange(t, RobotAddressResolver.TailscaleSeconds + RobotAddressResolver.LocalSearchSeconds,
                RobotAddressResolver.TailscaleSeconds + RobotAddressResolver.LocalSearchSeconds + 0.05);
        }

        [Fact]
        public void TailscaleRange_Is100_64Slash10()
        {
            Assert.True(RobotAddressResolver.IsTailscaleRange(Tailscale));
            Assert.True(RobotAddressResolver.IsTailscaleRange(IPAddress.Parse("100.64.0.1")));
            Assert.False(RobotAddressResolver.IsTailscaleRange(IPAddress.Parse("100.128.0.1")));
            Assert.False(RobotAddressResolver.IsTailscaleRange(Lan));
        }
    }
}
