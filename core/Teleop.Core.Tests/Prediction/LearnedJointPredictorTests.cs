using System.Text.Json;
using Teleop.Core.Contracts;
using Teleop.Core.Inference;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

/// <summary>
/// The learned predictor against the trainer's golden (docs/adr/0025): one real view's predictor calls,
/// replayed here, must give the trainer's windows, the trainer's model outputs, and the trainer's
/// predictions. The golden is written by `armpred golden-learned`, never by hand; a failure here means
/// the C# copy and the trainer disagree, and the model would see inputs it never trained on.
/// </summary>
public class LearnedJointPredictorTests
{
    /// <summary>The trainer's own parity tolerance for the exported graph, rad (export.py).</summary>
    private const double OutputTolerance = 1e-5;

    /// <summary>A window value: computed in double on both sides and stored as float.</summary>
    private const double WindowTolerance = 2e-6;

    private static readonly string Dir = Path.Combine(AppContext.BaseDirectory, "testdata", "learned");

    private static JsonElement Golden() => JsonDocument.Parse(File.ReadAllText(Path.Combine(Dir, "golden-v3.json"))).RootElement;

    private static MlpBackend Model(JsonElement golden)
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(Dir, golden.GetProperty("model").GetProperty("weights").GetString()!));
        Assert.True(MlpWeights.TryParse(bytes, out MlpWeights? weights, out string error), error);
        return new MlpBackend(weights!);
    }

    private static JointPredictorConfig Config(JsonElement golden, IInferenceBackend? model)
    {
        JsonElement c = golden.GetProperty("config");
        byte[] Bytes(string name) => c.GetProperty(name).EnumerateArray().Select(e => (byte)e.GetInt32()).ToArray();
        float[] Floats(string name) => c.GetProperty(name).EnumerateArray().Select(e => (float)e.GetDouble()).ToArray();
        return new JointPredictorConfig(
            maxJoints: c.GetProperty("maxJoints").GetInt32(), commandHistoryCapacity: c.GetProperty("commandHistoryCapacity").GetInt32(),
            maxJointSpeedRadPerSec: (float)c.GetProperty("maxJointSpeedRadPerSec").GetDouble(), instantMotorIds: new byte[0],
            maxHorizonTicks: c.GetProperty("maxHorizonTicks").GetInt64(), ticksPerSecond: c.GetProperty("ticksPerSecond").GetInt64(),
            scaledMotorIds: Bytes("scaledMotorIds"), speedScales: Floats("speedScales"),
            sensedAgeMotorIds: Bytes("sensedAgeMotorIds"), sensedAgeSeconds: Floats("sensedAgeSeconds"), model: model);
    }

    private static JointSample[] Samples(JsonElement e, bool withFlags)
    {
        int[] m = e.GetProperty("m").EnumerateArray().Select(x => x.GetInt32()).ToArray();
        float[] v = e.GetProperty("v").EnumerateArray().Select(x => (float)x.GetDouble()).ToArray();
        int[]? f = withFlags ? e.GetProperty("f").EnumerateArray().Select(x => x.GetInt32()).ToArray() : null;
        return m.Select((id, i) => new JointSample((byte)id, v[i], f != null && f[i] != 0)).ToArray();
    }

    [Fact]
    public void TheGoldenIsTheLayoutThisCopyBuilds()
    {
        JsonElement g = Golden();
        Assert.Equal("teleop-learned-golden", g.GetProperty("format").GetString());
        Assert.Equal(LearnedJointWindow.Motors.Select(m => (int)m), g.GetProperty("predictedMotors").EnumerateArray().Select(e => e.GetInt32()));
        Assert.Equal(LearnedJointWindow.GripperFractionPerRadian, g.GetProperty("gripperFractionPerRadian").GetDouble());
        Assert.Equal(LearnedJointWindow.InputLength, g.GetProperty("windows")[0].GetProperty("x").GetArrayLength());
    }

    [Fact]
    public void TheWeightsAreTheScoredModels()
    {
        JsonElement g = Golden();
        MlpBackend model = Model(g);
        Assert.Equal(g.GetProperty("model").GetProperty("onnxSha256").GetString(), model.Weights.OnnxSha256);
        Assert.Equal(g.GetProperty("model").GetProperty("modelId").GetString(), model.ModelId);
        Assert.Equal((LearnedJointWindow.InputLength, 12), (model.InputLength, model.OutputLength));
    }

    [Fact]
    public void TheBackend_ReproducesTheTrainersOutputs_OnItsWindows()
    {
        JsonElement g = Golden();
        MlpBackend model = Model(g);
        var y = new float[model.OutputLength];
        foreach (JsonElement w in g.GetProperty("windows").EnumerateArray())
        {
            float[] x = w.GetProperty("x").EnumerateArray().Select(e => (float)e.GetDouble()).ToArray();
            double[] expected = w.GetProperty("y").EnumerateArray().Select(e => e.GetDouble()).ToArray();
            Assert.True(model.TryRun(x, y));
            for (int i = 0; i < expected.Length; i++) Assert.InRange(y[i] - expected[i], -OutputTolerance, OutputTolerance);
        }
    }

    [Fact]
    public void TheWindow_And_ThePredictions_MatchTheTrainer_CallByCall()
    {
        JsonElement g = Golden();
        JointPredictorConfig config = Config(g, Model(g));
        var window = new LearnedJointWindow(config);
        var predictor = new LearnedJointPredictor(config);
        var x = new float[LearnedJointWindow.InputLength];
        var prior = new double[LearnedJointWindow.JointCount];
        var output = new JointSample[config.MaxJoints];
        JsonElement windows = g.GetProperty("windows");
        int predictions = 0, compared = 0, windowsChecked = 0, trainerFallbacks = 0;
        double worstWindow = 0, worstPrediction = 0;
        foreach (JsonElement e in g.GetProperty("events").EnumerateArray())
        {
            string kind = e.GetProperty("k").GetString()!;
            long t = e.GetProperty("t").GetInt64();
            if (kind == "c")
            {
                JointSample[] targets = Samples(e, withFlags: false);
                window.ObserveCommand(t, targets);
                predictor.ObserveCommand(t, targets, (float)e.GetProperty("s").GetDouble());
                continue;
            }

            if (kind == "s")
            {
                JointSample[] joints = Samples(e, withFlags: true);
                window.ObserveState(t, joints);
                predictor.ObserveState(t, joints);
                continue;
            }

            predictions++;
            if (e.TryGetProperty("w", out JsonElement wi))
            {
                JsonElement w = windows[wi.GetInt32()];
                double[] expected = w.GetProperty("x").EnumerateArray().Select(v => v.GetDouble()).ToArray();
                if (!window.TryBuild(t, x, prior, out bool valid))
                {
                    // No window yet: the trainer's is the all-zero invalid one.
                    Assert.False(w.GetProperty("valid").GetBoolean());
                    Assert.All(expected, v => Assert.Equal(0.0, v));
                    windowsChecked++;
                    goto predict;
                }

                Assert.Equal(w.GetProperty("valid").GetBoolean(), valid);
                for (int i = 0; i < expected.Length; i++)
                {
                    double d = Math.Abs(x[i] - expected[i]);
                    worstWindow = Math.Max(worstWindow, d);
                    Assert.True(d <= WindowTolerance + 1e-6 * Math.Abs(expected[i]),
                        $"window {wi.GetInt32()} value {i} (row {i / LearnedJointWindow.JointFeatures}, column {i % LearnedJointWindow.JointFeatures}): {x[i]} vs {expected[i]}");
                }

                windowsChecked++;
            }

        predict:
            bool ok = predictor.TryPredict(t, output, out int count);
            bool fallback = e.GetProperty("fb").GetBoolean();
            if (fallback) trainerFallbacks++;
            if (e.GetProperty("p").ValueKind == JsonValueKind.Null)
            {
                // The trainer had no prediction (no reading of every arm joint yet); this predictor
                // answers with its prior or not at all, and never with the model.
                Assert.True(fallback);
                continue;
            }

            Assert.True(ok);
            double?[] p = e.GetProperty("p").EnumerateArray().Select(v => v.ValueKind == JsonValueKind.Null ? (double?)null : v.GetDouble()).ToArray();
            for (int r = 0; r < LearnedJointWindow.JointCount; r++)
            {
                if (p[r] is not double want) continue;
                byte motor = LearnedJointWindow.Motors[r];
                int at = Array.FindIndex(output, 0, count, s => s.MotorId == motor);
                Assert.True(at >= 0, $"motor {motor} missing from the prediction");
                double got = LearnedJointWindow.ToModel(motor, output[at].Value);
                double d = Math.Abs(got - want);
                worstPrediction = Math.Max(worstPrediction, d);
                // float output, and the gripper converted to its 0..1 wire fraction and back.
                Assert.True(d <= OutputTolerance + 2e-6 * Math.Abs(want), $"prediction at {t}, motor {motor}: {got} vs {want}");
                compared++;
            }
        }

        Assert.Equal(g.GetProperty("windows").GetArrayLength(), windowsChecked);
        Assert.True(predictions > 500 && compared > 3000, $"{predictions} predictions, {compared} values compared");
        // The model ran exactly where the trainer's did; its fallbacks here came before any reading, when this
        // predictor has nothing to answer with (TryPredict false) rather than a prior to fall back to.
        Assert.Equal(predictions - trainerFallbacks, predictor.Inferences);
    }

    [Fact]
    public void ItIsRegistered_AndRefusesAConfigWithoutAModel()
    {
        JsonElement g = Golden();
        Assert.IsType<LearnedJointPredictor>(Registries.JointPredictors["learned"](Config(g, Model(g))));
        Assert.Throws<ArgumentException>(() => Registries.JointPredictors["learned"](Config(g, null)));
    }

    [Fact]
    public void AModelOfTheWrongShape_IsRefused()
    {
        JsonElement g = Golden();
        Assert.Throws<ArgumentException>(() => new LearnedJointPredictor(Config(g, new FixedModel(10, 12, ok: true))));
    }

    [Fact]
    public void AFailingOrNonFiniteModel_FallsBackToThePrior()
    {
        JsonElement g = Golden();
        foreach (FixedModel bad in new[] { new FixedModel(LearnedJointWindow.InputLength, 12, ok: false), new FixedModel(LearnedJointWindow.InputLength, 12, ok: true, value: float.NaN) })
        {
            var learned = new LearnedJointPredictor(Config(g, bad));
            var prior = new RelayReplayPredictor(Config(g, null));
            Feed(g, learned, prior, out long lastQuery);
            var a = new JointSample[16];
            var b = new JointSample[16];
            Assert.True(learned.TryPredict(lastQuery, a, out int na));
            Assert.True(prior.TryPredict(lastQuery, b, out int nb));
            Assert.Equal(b.Take(nb), a.Take(na));
            Assert.Equal(0, learned.Inferences);
        }
    }

    [Fact]
    public void AStaleReading_FallsBackToThePrior()
    {
        JsonElement g = Golden();
        JointPredictorConfig config = Config(g, Model(g));
        var learned = new LearnedJointPredictor(config);
        Feed(g, learned, null, out long lastQuery);
        long tps = config.TicksPerSecond;
        var output = new JointSample[16];
        long before = learned.Fallbacks;
        Assert.True(learned.TryPredict(lastQuery + 2 * tps, output, out _));
        Assert.Equal(before + 1, learned.Fallbacks);
    }

    [Fact]
    public void Reset_ReturnsItToAsConstructed()
    {
        JsonElement g = Golden();
        JointPredictorConfig config = Config(g, Model(g));
        var used = new LearnedJointPredictor(config);
        Feed(g, used, null, out _);
        used.Reset();
        var fresh = new LearnedJointPredictor(config);
        Feed(g, used, null, out long q1);
        Feed(g, fresh, null, out long q2);
        var a = new JointSample[16];
        var b = new JointSample[16];
        Assert.True(used.TryPredict(q1, a, out int na));
        Assert.True(fresh.TryPredict(q2, b, out int nb));
        Assert.Equal(b.Take(nb), a.Take(na));
        Assert.Equal((fresh.Inferences, fresh.Fallbacks), (used.Inferences, used.Fallbacks));
    }

    [Fact]
    public void TryPredict_AllocatesNothing()
    {
        JsonElement g = Golden();
        var learned = new LearnedJointPredictor(Config(g, Model(g)));
        Feed(g, learned, null, out long lastQuery);
        var output = new JointSample[16];
        AllocationAssert.Zero(() => learned.TryPredict(lastQuery, output, out _), iterations: 200);
    }

    [Fact]
    public void AMalformedWeightsFile_IsRefused()
    {
        JsonElement g = Golden();
        byte[] bytes = File.ReadAllBytes(Path.Combine(Dir, g.GetProperty("model").GetProperty("weights").GetString()!));
        Assert.False(MlpWeights.TryParse(bytes.AsSpan(0, bytes.Length - 1), out _, out _));
        byte[] wrongMagic = (byte[])bytes.Clone();
        wrongMagic[0] = (byte)'X';
        Assert.False(MlpWeights.TryParse(wrongMagic, out _, out _));
        byte[] wrongVersion = (byte[])bytes.Clone();
        wrongVersion[4] = 9;
        Assert.False(MlpWeights.TryParse(wrongVersion, out _, out string error));
        Assert.Contains("format", error);
    }

    // Replays the golden's calls (no queries) into the predictors; the last query's time comes back.
    private static void Feed(JsonElement g, ICommandAwarePredictor a, ICommandAwarePredictor? b, out long lastQuery)
    {
        lastQuery = 0;
        foreach (JsonElement e in g.GetProperty("events").EnumerateArray())
        {
            string kind = e.GetProperty("k").GetString()!;
            long t = e.GetProperty("t").GetInt64();
            if (kind == "c")
            {
                JointSample[] targets = Samples(e, withFlags: false);
                a.ObserveCommand(t, targets, 0f);
                b?.ObserveCommand(t, targets, 0f);
            }
            else if (kind == "s")
            {
                JointSample[] joints = Samples(e, withFlags: true);
                a.ObserveState(t, joints);
                b?.ObserveState(t, joints);
            }
            else
            {
                lastQuery = t;
            }
        }
    }

    private sealed class FixedModel : IInferenceBackend
    {
        private readonly bool _ok;
        private readonly float _value;

        public FixedModel(int input, int output, bool ok, float value = 0f)
        {
            InputLength = input;
            OutputLength = output;
            _ok = ok;
            _value = value;
        }

        public string ModelId => "fixed";
        public int InputLength { get; }
        public int OutputLength { get; }

        public bool TryRun(ReadOnlySpan<float> input, Span<float> output)
        {
            if (!_ok) return false;
            output.Slice(0, OutputLength).Fill(_value);
            return true;
        }

        public void Reset()
        {
        }
    }
}
