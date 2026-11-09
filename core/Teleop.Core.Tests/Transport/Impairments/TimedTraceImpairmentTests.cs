using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Transport;
using Teleop.Core.Transport.Impairments;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Transport.Impairments;

/// <summary>A recorded network replayed by time (docs/adr/0035 N4).</summary>
public class TimedTraceImpairmentTests
{
    private const long Ms = 10_000; // 10 MHz ticks

    private static TimedTraceImpairment Make(out long[] delays, out bool[] lost, bool bind = true)
    {
        // 10 samples 20 ms apart: 30 ms, a 450 ms stall decaying, then 30 ms; sample 7 lost.
        delays = new[] { 30 * Ms, 30 * Ms, 450 * Ms, 430 * Ms, 410 * Ms, 30 * Ms, 30 * Ms, 0, 30 * Ms, 30 * Ms };
        lost = new bool[10];
        lost[7] = true;
        var t = new TimedTraceImpairment(delays, lost, 20 * Ms);
        if (bind) t.Bind(1);
        return t;
    }

    private static (bool Dropped, long Delay) Fate(TimedTraceImpairment t, long sendTicks)
    {
        var f = new DatagramFate { SendTicks = sendTicks };
        t.ApplyOnSend(ref f);
        if (!f.Dropped) t.ApplyOnDeliver(ref f);
        return (f.Dropped, f.DelayTicks);
    }

    [Fact]
    public void Samples_AreLookedUpByTimeSinceTheFirstDatagram_NotByCount()
    {
        TimedTraceImpairment t = Make(out _, out _);
        long t0 = 1_000_000;
        Assert.Equal((false, 30 * Ms), Fate(t, t0));
        // Three datagrams inside the stall's 20 ms all get the stall, however many are sent.
        Assert.Equal((false, 450 * Ms), Fate(t, t0 + 40 * Ms));
        Assert.Equal((false, 450 * Ms), Fate(t, t0 + 45 * Ms));
        Assert.Equal((false, 450 * Ms), Fate(t, t0 + 59 * Ms));
        Assert.Equal((false, 430 * Ms), Fate(t, t0 + 60 * Ms));
        Assert.Equal((true, 0L), Fate(t, t0 + 141 * Ms)); // sample 7: lost
        Assert.Equal((false, 450 * Ms), Fate(t, t0 + 200 * Ms + 45 * Ms)); // 245 ms: sample 12, wrapped to 2
    }

    [Fact]
    public void ItWraps_AndResetStartsAgainFromTheNextDatagram()
    {
        TimedTraceImpairment t = Make(out long[] delays, out _);
        Fate(t, 5 * Ms);
        Assert.Equal(delays[2], Fate(t, 5 * Ms + 200 * Ms + 40 * Ms).Delay); // 240 ms later: sample 12 mod 10 = 2
        t.Reset();
        Assert.Equal(delays[0], Fate(t, 999 * Ms).Delay); // a new origin
        Assert.Equal(delays[3], Fate(t, 999 * Ms + 60 * Ms).Delay);
    }

    [Fact]
    public void ThroughAnEmulator_StallsKeepTheirLength_AndLossesDrop()
    {
        var loop = new LoopbackTransport(64, 256);
        var link = new EmulatedTransport(loop, new Teleop.Core.Contracts.INetworkImpairment[] { Make(out _, out _, bind: false) }, 7, 256);
        var buffer = new byte[64];
        var sends = new long[36];
        var arrivals = new List<(long Send, long Arrival)>();
        int sent = 0;
        long now = 0;
        void Drain()
        {
            while (link.TryReceive(now, buffer, out _, out long arrival)) arrivals.Add((sends[buffer[0]], arrival));
        }

        for (int i = 0; i < 36; i++) // 36 Hz for a second: 1.8x the trace's own rate
        {
            long send = i * 278 * Ms / 10;
            while (now < send)
            {
                now += Ms;
                Drain();
            }

            sends[i] = send;
            if (link.Send(new[] { (byte)i }, send)) sent++;
        }

        for (int k = 0; k < 1000; k++)
        {
            now += Ms;
            Drain();
        }

        Assert.Equal(sent, arrivals.Count);
        // Sent in [40, 60) ms (and again 200 ms later, wrapped): 450 ms late.
        Assert.Contains(arrivals, a => a.Send >= 40 * Ms && a.Send < 60 * Ms && a.Arrival - a.Send == 450 * Ms);
        Assert.True(sent < 36); // datagrams sent in a lost sample's 20 ms never arrive
    }

    [Fact]
    public void ApplyingAllocatesNothing_AndBadTracesAreRefused()
    {
        TimedTraceImpairment t = Make(out _, out _);
        long send = 0;
        AllocationAssert.Zero(() =>
        {
            var f = new DatagramFate { SendTicks = send };
            t.ApplyOnSend(ref f);
            t.ApplyOnDeliver(ref f);
            send += 7 * Ms;
        });
        Assert.Throws<ArgumentException>(() => new TimedTraceImpairment(new long[0], new bool[0], 1));
        Assert.Throws<ArgumentException>(() => new TimedTraceImpairment(new long[2], new bool[1], 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new TimedTraceImpairment(new long[1], new bool[1], 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new TimedTraceImpairment(new[] { -1L }, new bool[1], 1));
        Assert.Throws<InvalidOperationException>(() => t.Bind(2));
    }
}
