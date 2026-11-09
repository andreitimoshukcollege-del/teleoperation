using System.Text.Json;
using Teleop.Core.Contracts;
using Teleop.Core.Forecast;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Forecast;

/// <summary>
/// The causal network forecasters against the trainer's golden (docs/adr/0035 N3): 30 s of what a headset observes on a
/// bursty link, replayed here, must give the trainer's <c>persistence</c> and <c>rtt-half</c> forecasts at every issue, the
/// network block layout.py encodes from them, and forecast.effective_times' effective times. The golden is written by
/// `armpred golden-forecast`, never by hand; a failure means the headset would tell the model about a network in a form it
/// never trained on.
/// </summary>
public class NetworkForecasterTests
{
    /// <summary>A forecast value, ms: computed in double on both sides, stored as float.</summary>
    private const double ValueTolerance = 1e-4;

    /// <summary>A block value: asinh in float32 there, in double then rounded here (one float ulp at most).</summary>
    private const double BlockTolerance = 2e-6;

    private static readonly string Path_ = Path.Combine(AppContext.BaseDirectory, "testdata", "learned", "forecast-golden-v1.json");

    private static JsonElement Golden()
    {
        Assert.True(File.Exists(Path_), $"{Path_} is missing: `cd trainer && python -m armpred golden-forecast --out ../core/testdata/learned`");
        return JsonDocument.Parse(File.ReadAllText(Path_)).RootElement;
    }

    /// <summary>Every forecast the forecaster issues over the golden's observations, fed in time order before each issue.</summary>
    private static List<NetworkForecast> Replay(JsonElement g, INetworkForecaster forecaster)
    {
        var frames = g.GetProperty("frames").EnumerateArray().Select(e => (t: e[0].GetInt64(), s: e[1].GetInt64(), q: e[2].GetInt64())).ToList();
        var trips = g.GetProperty("roundTrips").EnumerateArray()
            .Select(e => (t: e[0].GetInt64(), rtt: e[1].GetInt64(), down: e[2].GetInt64(), unc: e[3].GetInt64(), min: e[4].GetInt64(), synced: e[5].GetInt32() != 0))
            .ToList();
        var out_ = new List<NetworkForecast>();
        int fi = 0, ri = 0;
        foreach (JsonElement issue in g.GetProperty("issues").EnumerateArray())
        {
            long t = issue.GetInt64();
            while (fi < frames.Count || ri < trips.Count)
            {
                bool frameNext = fi < frames.Count && (ri >= trips.Count || frames[fi].t <= trips[ri].t);
                long next = frameNext ? frames[fi].t : trips[ri].t;
                if (next > t) break;
                if (frameNext)
                {
                    forecaster.ObserveFrame(frames[fi].t, frames[fi].s, frames[fi].q);
                    fi++;
                }
                else
                {
                    var r = trips[ri++];
                    forecaster.ObserveRoundTrip(r.t, r.rtt, r.down, r.unc, r.min, r.synced);
                }
            }

            var f = new NetworkForecast();
            forecaster.Issue(t, f);
            out_.Add(f);
        }

        return out_;
    }

    private static void Near(double want, double got, double tol, string what) =>
        Assert.True(Math.Abs(got - want) <= tol * Math.Max(1.0, Math.Abs(want)), $"{what}: {got} vs the trainer's {want}");

    [Theory]
    [InlineData("persistence")]
    [InlineData("rtt-half")]
    public void EveryForecast_IsTheTrainers(string level)
    {
        JsonElement g = Golden();
        Assert.Equal("teleop-forecast-golden", g.GetProperty("format").GetString());
        Assert.Equal(Enumerable.Range(0, NetworkForecast.HorizonCount).Select(NetworkForecast.HorizonMs), g.GetProperty("horizonsMs").EnumerateArray().Select(e => e.GetInt32()));
        Assert.Equal(Enumerable.Range(0, NetworkForecast.QuantileCount).Select(NetworkForecast.Quantile), g.GetProperty("quantiles").EnumerateArray().Select(e => e.GetDouble()));
        INetworkForecaster forecaster = Registries.NetworkForecasters[level](g.GetProperty("tps").GetInt64());
        Assert.Equal(level, forecaster.Level);
        List<NetworkForecast> got = Replay(g, forecaster);
        JsonElement s = g.GetProperty("streams").GetProperty(level);
        int validDown = 0, validUp = 0;
        for (int i = 0; i < got.Count; i++)
        {
            NetworkForecast f = got[i];
            for (int d = 0; d < NetworkForecast.Directions; d++)
            {
                string at = $"issue {i}, direction {d}";
                Assert.True(s.GetProperty("validQ")[i][d].GetInt32() != 0 == f.ValidQ[d], $"{at}: validQ");
                Assert.True(s.GetProperty("validLoss")[i][d].GetInt32() != 0 == f.ValidLoss[d], $"{at}: validLoss");
                Assert.True(s.GetProperty("validOutage")[i][d].GetInt32() != 0 == f.ValidOutage[d], $"{at}: validOutage");
                Near(s.GetProperty("floorMs")[i][d].GetDouble(), f.FloorMs[d], ValueTolerance, $"{at}: floor");
                Near(s.GetProperty("outage")[i][d].GetDouble(), f.Outage[d], ValueTolerance, $"{at}: outage");
            }

            if (f.ValidQ[NetworkForecast.Down]) validDown++;
            if (f.ValidQ[NetworkForecast.Up]) validUp++;
            JsonElement q = s.GetProperty("qMs")[i], loss = s.GetProperty("loss")[i];
            for (int k = 0; k < f.QMs.Length; k++) Near(q[k].GetDouble(), f.QMs[k], ValueTolerance, $"issue {i}: q[{k}]");
            for (int k = 0; k < f.Loss.Length; k++) Near(loss[k].GetDouble(), f.Loss[k], ValueTolerance, $"issue {i}: loss[{k}]");
        }

        // The fixture exercises both sides, and the time before ClockSync syncs.
        Assert.InRange(validDown, 1, got.Count - 1);
        Assert.InRange(validUp, 1, got.Count - 1);
    }

    [Fact]
    public void TheNetworkBlock_IsTheTrainers()
    {
        JsonElement g = Golden();
        List<NetworkForecast> got = Replay(g, new PersistenceForecaster(g.GetProperty("tps").GetInt64()));
        JsonElement blocks = g.GetProperty("blocks");
        Assert.Equal("persistence", blocks.GetProperty("level").GetString());
        Assert.Equal(LearnedJointWindow.NetworkFeatures - 1, NetworkForecastBlock.Length);
        var block = new float[NetworkForecastBlock.Length];
        int b = 0;
        foreach (JsonElement index in blocks.GetProperty("index").EnumerateArray())
        {
            NetworkForecastBlock.Encode(got[index.GetInt32()], block);
            JsonElement want = blocks.GetProperty("values")[b++];
            Assert.Equal(block.Length, want.GetArrayLength());
            for (int k = 0; k < block.Length; k++) Near(want[k].GetDouble(), block[k], BlockTolerance, $"block of issue {index.GetInt32()}, value {k}");
        }
    }

    [Theory]
    [InlineData("persistence", "effPersistence")]
    [InlineData("rtt-half", "effRttHalf")]
    public void EffectiveTimes_AreTheTrainers(string level, string key)
    {
        JsonElement g = Golden();
        long tps = g.GetProperty("tps").GetInt64();
        List<NetworkForecast> forecasts = Replay(g, Registries.NetworkForecasters[level](tps));
        long[] issues = g.GetProperty("issues").EnumerateArray().Select(e => e.GetInt64()).ToArray();
        JsonElement c = g.GetProperty("commands");
        long[] send = c.GetProperty("send").EnumerateArray().Select(e => e.GetInt64()).ToArray();
        long[] fallback = c.GetProperty("effNoForecast").EnumerateArray().Select(e => e.GetInt64()).ToArray();
        long[] want = c.GetProperty(key).EnumerateArray().Select(e => e.GetInt64()).ToArray();
        var eff = new ForecastEffectiveTimes(tps);
        int k = -1, moved = 0;
        for (int i = 0; i < send.Length; i++)
        {
            while (k + 1 < issues.Length && issues[k + 1] <= send[i]) k++;
            long e = eff.Next(send[i], fallback[i], k >= 0 ? forecasts[k] : null);
            Assert.True(e == want[i], $"command {i}: {e} vs the trainer's {want[i]}");
            if (e != fallback[i]) moved++;
        }

        Assert.True(moved > 0, "no command's effective time came from the forecast");
    }

    [Fact]
    public void Persistence_IsInvalidBeforeSync_AndSeesAnOutage()
    {
        const long tps = 1_000_000_000, ms = tps / 1000;
        var p = new PersistenceForecaster(tps);
        var f = new NetworkForecast();
        for (int i = 0; i < 50; i++) p.ObserveFrame(i * 20 * ms + 15 * ms, -1, i); // unsynced: ignored
        p.Issue(1100 * ms, f);
        Assert.False(f.ValidQ[NetworkForecast.Down]);
        for (int i = 50; i < 100; i++) p.ObserveFrame(i * 20 * ms + 15 * ms, i * 20 * ms, i);
        p.Issue(1990 * ms, f);
        Assert.True(f.ValidQ[NetworkForecast.Down] && f.Outage[NetworkForecast.Down] == 0f);
        Assert.Equal(15f, f.QMs[NetworkForecast.QIndex(NetworkForecast.Down, NetworkForecast.H0, NetworkForecast.P50)], 3);
        p.Issue(2300 * ms, f); // 315 ms since the last frame
        Assert.Equal(1f, f.Outage[NetworkForecast.Down]);
    }

    [Fact]
    public void Reset_ReturnsItToAsConstructed()
    {
        JsonElement g = Golden();
        long tps = g.GetProperty("tps").GetInt64();
        foreach (string level in Registries.NetworkForecasters.Keys)
        {
            INetworkForecaster used = Registries.NetworkForecasters[level](tps);
            Replay(g, used);
            used.Reset();
            List<NetworkForecast> again = Replay(g, used), fresh = Replay(g, Registries.NetworkForecasters[level](tps));
            for (int i = 0; i < fresh.Count; i++)
            {
                Assert.Equal(fresh[i].QMs, again[i].QMs);
                Assert.Equal(fresh[i].ValidQ, again[i].ValidQ);
                Assert.Equal(fresh[i].FloorMs, again[i].FloorMs);
            }
        }
    }

    [Fact]
    public void ObservingAndIssuing_AllocateNothing()
    {
        const long tps = 1_000_000_000, ms = tps / 1000;
        var forecasters = Registries.NetworkForecasters.Values.Select(make => make(tps)).ToArray();
        var f = new NetworkForecast();
        var block = new float[NetworkForecastBlock.Length];
        var eff = new ForecastEffectiveTimes(tps);
        long i = 0;
        void Step()
        {
            foreach (INetworkForecaster p in forecasters)
            {
                p.ObserveFrame(i * 20 * ms + 14 * ms, i * 20 * ms, i);
                if (i % 50 == 0) p.ObserveRoundTrip(i * 20 * ms + 14 * ms, 30 * ms, 14 * ms, 15 * ms, 28 * ms, true);
                if (i % 5 == 0) p.Issue(i * 20 * ms + 15 * ms, f);
            }

            NetworkForecastBlock.Encode(f, block);
            eff.Next(i * 20 * ms, i * 20 * ms + 15 * ms, f);
            i++;
        }

        for (int k = 0; k < 2500; k++) Step(); // past the ring's capacity
        AllocationAssert.Zero(Step, 2000);
    }

    [Fact]
    public void TheBank_GivesTheForecast_AndItsEffectiveTimes_OnlyToTakers()
    {
        var taker = new Taker();
        var plain = new Plain();
        var bank = new JointPredictorBank(new[] { "taker", "plain" }, new ICommandAwarePredictor[] { taker, plain }, 0);
        Assert.True(bank.TakesForecast(0));
        Assert.False(bank.TakesForecast(1));
        var f = new NetworkForecast();
        f.Clear(123);
        bank.ObserveForecast(f);
        bank.ObserveCommand(1000, 1200, new[] { new JointSample(1, 0.1f, false) }, 0f);
        Assert.Equal((1, 1200L), (taker.Forecasts, taker.LastEffective));
        Assert.Equal((0, 1000L), (plain.Forecasts, plain.LastEffective));
        bank.ClearForecast();
        Assert.Equal(1, taker.Cleared);
        Assert.True(bank.TrySetEnabled(1, false));
        Assert.True(bank.TrySetActive(0));
    }

    [Fact]
    public void TheWindow_CarriesTheForecast_AndItsAge_ThenZerosWhenCleared()
    {
        JsonElement g = Golden();
        long tps = g.GetProperty("tps").GetInt64();
        List<NetworkForecast> got = Replay(g, new PersistenceForecaster(tps));
        NetworkForecast f = got.First(x => x.ValidQ[NetworkForecast.Down] && x.ValidQ[NetworkForecast.Up]);
        var config = new JointPredictorConfig(maxJoints: 8, commandHistoryCapacity: 64, maxJointSpeedRadPerSec: 6f, instantMotorIds: new byte[0],
            maxHorizonTicks: tps, ticksPerSecond: tps);
        var window = new LearnedJointWindow(config);
        var joints = LearnedJointWindow.Motors.Select(m => new JointSample(m, 0.1f, true)).ToArray();
        long t0 = f.IssuedTicks;
        window.ObserveCommand(t0 - tps / 10, joints);
        window.ObserveState(t0 - tps / 20, joints);
        var x = new float[LearnedJointWindow.InputLength];
        var prior = new double[LearnedJointWindow.JointCount];
        int network = LearnedJointWindow.JointCount * LearnedJointWindow.JointFeatures + LearnedJointWindow.BaseArmFeatures;
        Assert.True(window.TryBuild(t0, x, prior, out _));
        Assert.All(x.Skip(network).Take(LearnedJointWindow.NetworkFeatures), v => Assert.Equal(0f, v));

        window.ObserveForecast(f);
        long target = t0 + tps / 4;
        Assert.True(window.TryBuild(target, x, prior, out _));
        var block = new float[NetworkForecastBlock.Length];
        NetworkForecastBlock.Encode(f, block);
        Assert.Equal(block, x.Skip(network).Take(block.Length).ToArray());
        Assert.Equal(0.5f, x[network + LearnedJointWindow.NetworkFeatures - 1]); // 250 ms old / 0.5 s

        window.ClearForecast();
        Assert.True(window.TryBuild(target, x, prior, out _));
        Assert.All(x.Skip(network).Take(LearnedJointWindow.NetworkFeatures), v => Assert.Equal(0f, v));
    }

    /// <summary>A joint predictor that does not take the forecast: it only records its effective times.</summary>
    private class Plain : ICommandAwarePredictor
    {
        public int Forecasts { get; protected set; }
        public long LastEffective { get; private set; }

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec) => LastEffective = effectiveTicks;

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
        }

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count)
        {
            count = 0;
            return false;
        }

        public void Reset()
        {
        }

        public PredictorDiagnostics Diagnostics => default;
    }

    private sealed class Taker : Plain, INetworkForecastTaker
    {
        public int Cleared { get; private set; }

        public void ObserveForecast(NetworkForecast forecast) => Forecasts++;

        public void ClearForecast() => Cleared++;
    }
}
