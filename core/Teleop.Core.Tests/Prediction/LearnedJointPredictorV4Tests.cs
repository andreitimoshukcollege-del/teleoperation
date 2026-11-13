using System.Text.Json;
using Teleop.Core.Contracts;
using Teleop.Core.Inference;
using Teleop.Core.Prediction;
using Teleop.Core.Registry;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;

namespace Teleop.Core.Tests.Prediction;

/// <summary>
/// The learned predictor's layout v4 (the FR3 Duo, docs/adr/0028) against the trainer's golden: one simulated view's
/// predictor calls, replayed here, must give the trainer's windows, outputs and predictions. The golden is written by
/// `armpred golden-learned-v4`, never by hand. Layout v3's golden (LearnedJointPredictorTests) is untouched.
/// </summary>
public class LearnedJointPredictorV4Tests
{
    private const double OutputTolerance = 1e-5;
    private const double WindowTolerance = 2e-6;

    private static readonly string Dir = Path.Combine(AppContext.BaseDirectory, "testdata", "learned");

    private static JsonElement Golden() => JsonDocument.Parse(File.ReadAllText(Path.Combine(Dir, "golden-v4.json"))).RootElement;

    private static MlpBackend Model(JsonElement golden)
    {
        byte[] bytes = File.ReadAllBytes(Path.Combine(Dir, golden.GetProperty("model").GetProperty("weights").GetString()!));
        Assert.True(MlpWeights.TryParse(bytes, out MlpWeights? weights, out string error), error);
        return new MlpBackend(weights!);
    }

    internal static TrajectoryShaperConfig Shaper(JsonElement golden)
    {
        JsonElement s = golden.GetProperty("shaper");
        var limits = s.GetProperty("motors").EnumerateArray().Select(m => m.GetProperty("isFinger").GetBoolean()
            ? ShapedJointLimits.Finger((byte)m.GetProperty("motorId").GetInt32(), m.GetProperty("fingerSpeed").GetDouble(), m.GetProperty("fingerDeadband").GetDouble())
            : ShapedJointLimits.Joint((byte)m.GetProperty("motorId").GetInt32(), m.GetProperty("lower").GetDouble(), m.GetProperty("upper").GetDouble(),
                m.GetProperty("softLower").GetDouble(), m.GetProperty("softUpper").GetDouble(), m.GetProperty("velocity").GetDouble(),
                m.GetProperty("acceleration").GetDouble(), m.GetProperty("jerk").GetDouble(), m.GetProperty("positionBasedDeceleration").GetDouble(),
                m.GetProperty("positionBasedVelocityOffset").GetDouble())).ToArray();
        return new TrajectoryShaperConfig(limits, s.GetProperty("stepSeconds").GetDouble(), s.GetProperty("velocityScale").GetDouble(),
            s.GetProperty("accelerationScale").GetDouble(), s.GetProperty("jerkScale").GetDouble(), s.GetProperty("brakingScale").GetDouble(),
            s.GetProperty("settleSeconds").GetDouble(), s.GetProperty("positionMargin").GetDouble());
    }

    internal static LearnedChainLayout Chain(JsonElement golden)
    {
        JsonElement c = golden.GetProperty("chain");
        return new LearnedChainLayout(
            c.GetProperty("motors").EnumerateArray().Select(e => (byte)e.GetInt32()).ToArray(),
            c.GetProperty("armOfRow").EnumerateArray().Select(e => e.GetInt32()).ToArray(),
            c.GetProperty("identity").EnumerateArray().Select(e => e.GetInt32()).ToArray(),
            c.GetProperty("gravity").EnumerateArray().Select(e => e.GetDouble()).ToArray());
    }

    private static JointPredictorConfig Config(JsonElement golden, IInferenceBackend? model, bool chain = true)
    {
        JsonElement c = golden.GetProperty("config");
        return new JointPredictorConfig(
            maxJoints: c.GetProperty("maxJoints").GetInt32(), commandHistoryCapacity: c.GetProperty("commandHistoryCapacity").GetInt32(),
            maxJointSpeedRadPerSec: 2f, instantMotorIds: c.GetProperty("instantMotorIds").EnumerateArray().Select(e => (byte)e.GetInt32()).ToArray(),
            maxHorizonTicks: c.GetProperty("maxHorizonTicks").GetInt64(), ticksPerSecond: c.GetProperty("ticksPerSecond").GetInt64(),
            model: model, shaper: Shaper(golden), chainLayout: chain ? Chain(golden) : null);
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
        Assert.Equal(2, g.GetProperty("version").GetInt32());
        Assert.Equal(LearnedJointWindowV4.LayoutId, g.GetProperty("layout").GetString());
        LearnedChainLayout chain = Chain(g);
        Assert.Equal(chain.Rows * LearnedJointWindowV4.JointFeatures + chain.Arms * LearnedJointWindowV4.ArmFeatures,
            g.GetProperty("windows")[0].GetProperty("x").GetArrayLength());
        Assert.Equal((253, 125, 249), (LearnedJointWindowV4.JointFeatures, LearnedJointWindowV4.ArmFeatures, LearnedJointWindowV4.HandFlagIndex));
    }

    [Fact]
    public void TheWeightsAreVersion2_WithTheLayoutsHead()
    {
        JsonElement g = Golden();
        MlpBackend model = Model(g);
        MlpWeights w = model.Weights;
        Assert.Equal((2, "v4", 16, 2), (w.FormatVersion, w.LayoutId, w.Joints, w.Arms));
        Assert.Equal(LearnedJointWindowV4.HandFlagIndex, w.FlagIndex);
        Assert.Equal((float)(3.0 * Math.PI / 180.0), w.Cap);
        Assert.Equal(0.5f, w.FlagCap);
        LearnedChainLayout chain = Chain(g);
        for (int r = 0; r < chain.Rows; r++) Assert.Equal(chain.ArmOf(r), w.ArmOf(r));
        Assert.Equal(g.GetProperty("model").GetProperty("onnxSha256").GetString(), w.OnnxSha256);
        Assert.Equal((4298, 32), (model.InputLength, model.OutputLength));
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
        var window = new LearnedJointWindowV4(config);
        var predictor = new LearnedJointPredictor(config);
        Assert.Equal("v4", predictor.Layout);
        var x = new float[window.InputLength];
        var prior = new double[window.Rows];
        var output = new JointSample[config.MaxJoints];
        JsonElement windows = g.GetProperty("windows");
        LearnedChainLayout chain = Chain(g);
        int predictions = 0, compared = 0, windowsChecked = 0, trainerFallbacks = 0;
        foreach (JsonElement e in g.GetProperty("events").EnumerateArray())
        {
            string kind = e.GetProperty("k").GetString()!;
            long t = e.GetProperty("t").GetInt64();
            if (kind == "c")
            {
                JointSample[] targets = Samples(e, withFlags: false);
                window.ObserveCommand(t, targets);
                predictor.ObserveCommand(t, targets, 0f);
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
                    // No window yet (no reading of every arm joint): the trainer's is the all-zero invalid one.
                    Assert.False(w.GetProperty("valid").GetBoolean());
                    Assert.All(expected, v => Assert.Equal(0.0, v));
                    windowsChecked++;
                    goto predict;
                }

                Assert.Equal(w.GetProperty("valid").GetBoolean(), valid);
                for (int i = 0; i < expected.Length; i++)
                {
                    double d = Math.Abs(x[i] - expected[i]);
                    bool inRows = i < chain.Rows * LearnedJointWindowV4.JointFeatures;
                    string where = inRows
                        ? $"row {i / LearnedJointWindowV4.JointFeatures}, column {i % LearnedJointWindowV4.JointFeatures}"
                        : $"arm block, column {(i - chain.Rows * LearnedJointWindowV4.JointFeatures) % LearnedJointWindowV4.ArmFeatures}";
                    Assert.True(d <= WindowTolerance + 1e-6 * Math.Abs(expected[i]), $"window {wi.GetInt32()} value {i} ({where}): {x[i]} vs {expected[i]}");
                }

                windowsChecked++;
            }

        predict:
            bool ok = predictor.TryPredict(t, output, out int count);
            if (e.GetProperty("fb").GetBoolean()) trainerFallbacks++;
            if (e.GetProperty("p").ValueKind == JsonValueKind.Null)
            {
                Assert.True(e.GetProperty("fb").GetBoolean());
                continue;
            }

            Assert.True(ok);
            double?[] p = e.GetProperty("p").EnumerateArray().Select(v => v.ValueKind == JsonValueKind.Null ? (double?)null : v.GetDouble()).ToArray();
            for (int r = 0; r < chain.Rows; r++)
            {
                if (p[r] is not double want) continue;
                byte motor = chain.MotorOf(r);
                int at = Array.FindIndex(output, 0, count, s => s.MotorId == motor);
                Assert.True(at >= 0, $"motor {motor} missing from the prediction");
                double d = Math.Abs(output[at].Value - want);
                Assert.True(d <= OutputTolerance + 2e-6 * Math.Abs(want), $"prediction at {t}, motor {motor}: {output[at].Value} vs {want}");
                compared++;
            }
        }

        Assert.Equal(windows.GetArrayLength(), windowsChecked);
        Assert.True(predictions > 300 && compared > 4000, $"{predictions} predictions, {compared} values compared");
        Assert.Equal(predictions - trainerFallbacks, predictor.Inferences);
    }

    [Fact]
    public void ALayoutV4Model_WithoutTheChain_IsRefused_AndALayoutV3Model_WithIt()
    {
        JsonElement g = Golden();
        Assert.Throws<ArgumentException>(() => new LearnedJointPredictor(Config(g, Model(g), chain: false)));
        Assert.Throws<ArgumentException>(() => new LearnedJointPredictor(Config(g, new FixedModel(LearnedJointWindow.InputLength, 12))));
        Assert.IsType<LearnedJointPredictor>(Registries.JointPredictors["learned"](Config(g, Model(g))));
    }

    [Fact]
    public void PredictingAllocatesNothing()
    {
        JsonElement g = Golden();
        JointPredictorConfig config = Config(g, Model(g));
        var predictor = new LearnedJointPredictor(config);
        long last = 0;
        foreach (JsonElement e in g.GetProperty("events").EnumerateArray())
        {
            string kind = e.GetProperty("k").GetString()!;
            long t = e.GetProperty("t").GetInt64();
            if (kind == "c") predictor.ObserveCommand(t, Samples(e, false), 0f);
            else if (kind == "s") predictor.ObserveState(t, Samples(e, true));
            else last = t;
        }

        var output = new JointSample[config.MaxJoints];
        AllocationAssert.Zero(() => predictor.TryPredict(last, output, out _), iterations: 200);
    }

    private sealed class FixedModel : IInferenceBackend
    {
        public FixedModel(int inputs, int outputs)
        {
            InputLength = inputs;
            OutputLength = outputs;
        }

        public string ModelId => "fixed";
        public int InputLength { get; }
        public int OutputLength { get; }
        public bool TryRun(ReadOnlySpan<float> input, Span<float> output) => true;
        public void Reset()
        {
        }
    }
}
