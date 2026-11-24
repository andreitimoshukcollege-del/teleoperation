using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;
using Teleop.RobotHost.Net;

namespace Teleop.RobotHost.Tests.Net
{
    /// <summary>
    /// The twin's <c>stall:</c> uplink (docs/adr/0038 N): a base delay with jitter that keeps the link's order, stalls that end
    /// in a burst of everything they held, nothing lost, and delays that are a function of the seed and the time since the
    /// first datagram, so a replay meets its source's network.
    /// </summary>
    public class StallUplinkImpairmentTests
    {
        private const long Tps = 1000; // ticks are milliseconds
        private const string Profile = "stall:50:20:2:300:300";

        /// <summary>Sends a command every <paramref name="spacing"/> ms from <paramref name="start"/> for <paramref name="ms"/> and
        /// returns (index, read, delivered) for each, in the order the receiver gave them.</summary>
        private static List<(int index, long read, long delivered)> Run(string profile, ulong seed, long start = 0, long ms = 20_000, long spacing = 28)
        {
            var socket = new LoopbackTransport(64, 4096);
            var receiver = new UplinkImpairedReceiver(socket, Tps, 64);
            receiver.SetProfile(profile, seed);
            var got = new List<(int, long, long)>();
            var buffer = new byte[64];
            int sent = 0;
            for (long t = start; t <= start + ms + 2000; t++)
            {
                if (t <= start + ms && (t - start) % spacing == 0)
                {
                    socket.Send(new[] { (byte)(sent & 0xFF), (byte)(sent >> 8) }, t);
                    sent++;
                }

                while (receiver.TryReceive(t, buffer, out int n, out long read, out long delivered))
                {
                    Assert.Equal(2, n);
                    got.Add((buffer[0] | (buffer[1] << 8), read - start, delivered - start));
                }
            }

            Assert.Equal(sent, got.Count); // nothing lost
            return got;
        }

        [Fact]
        public void TheName_CarriesEverySetting_AndAMalformedOneIsRefused()
        {
            Assert.True(StallSettings.TryParse("stall:85:20.5:4:200:500", out StallSettings s, out string? error), error);
            Assert.Equal(85, s.BaseMs);
            Assert.Equal(20.5, s.JitterMs);
            Assert.Equal(4, s.MeanGapSeconds);
            Assert.Equal(200, s.MinStallMs);
            Assert.Equal(500, s.MaxStallMs);
            Assert.Equal("stall:85:20.5:4:200:500", s.Name);

            foreach (string bad in new[] { "stall:85:20:4:200", "stall:85:20:4:500:200", "stall:-1:20:4:200:500", "stall:85:x:4:200:500",
                                           "stall:85:20:0.1:200:500", "stall:85:20:4:200:9000", "stall:NaN:20:4:200:500", "50ms-5j" })
            {
                Assert.False(StallSettings.TryParse(bad, out _, out error), bad);
                Assert.NotNull(error);
            }

            Assert.True(UplinkProfiles.TryResolve(Profile, Tps, null, out INetworkImpairment[] impairments, out error), error);
            Assert.IsType<StallUplinkImpairment>(Assert.Single(impairments));
            Assert.False(UplinkProfiles.TryResolve("stall:1:2", Tps, null, out _, out _));
            Assert.True(UplinkProfiles.RestartsEachSession(Profile));
            Assert.False(UplinkProfiles.RestartsEachSession("50ms-5j"));

            // RobotHost refuses a malformed one by name, as it does a typo of a Core profile.
            var receiver = new UplinkImpairedReceiver(new LoopbackTransport(64, 16), Tps, 64);
            Assert.Throws<ArgumentException>(() => receiver.SetProfile("stall:85:20", 1));
            receiver.SetProfile(Profile, 1);
            Assert.Equal(Profile, receiver.Profile);
        }

        [Fact]
        public void TheLink_KeepsItsOrder_DelaysWithinTheBaseAndJitter_AndStallsEndInABurst()
        {
            List<(int index, long read, long delivered)> got = Run(Profile, 3);
            Assert.Equal(Enumerable.Range(0, got.Count), got.Select(g => g.index)); // in order: nothing for the staleness rule to refuse

            var bursts = got.GroupBy(g => g.delivered).Where(g => g.Count() >= 5).ToList();
            // About one stall every 2 s in 20 s, each holding about 300 / 28 commands.
            Assert.InRange(bursts.Count, 5, 16);
            Assert.All(bursts, b => Assert.InRange(b.Count(), 9, 12));
            var inBurst = new HashSet<int>(bursts.SelectMany(b => b.Select(g => g.index)));
            foreach ((int index, long read, long delivered) in got)
            {
                long delay = delivered - read;
                if (inBurst.Contains(index)) Assert.InRange(delay, 50, 50 + 300 + 1);
                else Assert.InRange(delay, 50, 70 + 1); // + the loop's own millisecond
            }

            // The jitter is really there: not every delay is the base.
            Assert.True(got.Where(g => !inBurst.Contains(g.index)).Select(g => g.delivered - g.read).Distinct().Count() > 10);
        }

        [Fact]
        public void TheDelays_AreTheSeedsAndTheTimeSinceTheFirstDatagram()
        {
            List<long> Delays(ulong seed, long start) => Run(Profile, seed, start, ms: 10_000).Select(g => g.delivered - g.read).ToList();

            List<long> a = Delays(5, 0);
            Assert.Equal(a, Delays(5, 0));
            // A take that starts later, after the profile was set afresh (Program does that at each session start), meets
            // the same network: a replay sees its source's delays.
            Assert.Equal(a, Delays(5, 123_457));
            Assert.NotEqual(a, Delays(6, 0));
        }
    }
}
