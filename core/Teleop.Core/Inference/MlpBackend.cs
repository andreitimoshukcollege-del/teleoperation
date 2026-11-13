using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Inference
{
    /// <summary>
    /// The learned joint model's exported graph in plain C# (docs/adr/0025): the trainer's
    /// <c>FlatExport</c> over architecture A. Input <c>[joints × jointFeatures | armFeatures]</c> (layout
    /// v3: six rows of 251, then 114); for each joint, its row and the arm row through the dense layers
    /// (ReLU between them), then the head: a correction <c>cap · tanh(raw₀)</c>, the cap 15° or, on the
    /// gripper's row (its identity flag), 90° (in servo radians), and a log-scale <c>raw₁</c> clamped to
    /// [ln 1e-4, ln 0.5]. Output <c>[joints × 2]</c>.
    ///
    /// <para><b>Weights version 2</b> (layout v4, docs/adr/0028): the input is <c>[rows × jointFeatures | arms ×
    /// armFeatures]</c>, each row paired with its own arm's block, and the head's caps and flag come from the weights
    /// (<c>cap + (flagCap − cap) · flag</c>). A version-1 file is one arm with v3's head, computed as before.</para>
    ///
    /// <para>Single precision throughout, as the exported graph is; summation order differs from
    /// onnxruntime's, within the trainer's 1e-5 rad parity tolerance (pinned by the golden test).
    /// Deterministic, stateless (<see cref="Reset"/> is a no-op), allocation-free after construction.</para>
    /// </summary>
    public sealed class MlpBackend : IInferenceBackend
    {
        /// <summary>Where a joint row's gripper identity flag sits: the identity block's last entry (layout v3).</summary>
        public const int GripperFlagIndex = 247;

        private static readonly float CapBase = (float)(15.0 * Math.PI / 180.0);
        private static readonly float CapGripperExtra = (float)(90.0 * Math.PI / 180.0 - 15.0 * Math.PI / 180.0);
        private static readonly float LogScaleMin = (float)Math.Log(1e-4);
        private static readonly float LogScaleMax = (float)Math.Log(0.5);

        private readonly MlpWeights _w;
        private readonly float[] _a;
        private readonly float[] _b;

        private readonly float _flagExtra;
        private readonly float _cap;

        public MlpBackend(MlpWeights weights)
        {
            _w = weights ?? throw new ArgumentNullException(nameof(weights));
            if (weights.FlagIndex >= weights.JointFeatures)
            {
                throw new ArgumentException($"a joint row of {weights.JointFeatures} has no identity block at {weights.FlagIndex}", nameof(weights));
            }

            int widest = weights.JointFeatures + weights.ArmFeatures;
            for (int k = 0; k < weights.LayerCount; k++) widest = Math.Max(widest, weights.Outputs[k]);
            _a = new float[widest];
            _b = new float[widest];
            InputLength = weights.Joints * weights.JointFeatures + weights.Arms * weights.ArmFeatures;
            OutputLength = weights.Joints * 2;
            // Version 1 keeps v3's constants exactly as they were computed; version 2's come from the file.
            if (weights.FormatVersion == MlpWeights.Version)
            {
                _cap = CapBase;
                _flagExtra = CapGripperExtra;
            }
            else
            {
                _cap = weights.Cap;
                _flagExtra = weights.FlagCap - weights.Cap;
            }
        }

        public string ModelId => _w.ModelId;
        public int InputLength { get; }
        public int OutputLength { get; }
        public MlpWeights Weights => _w;

        public bool TryRun(ReadOnlySpan<float> input, Span<float> output)
        {
            if (input.Length != InputLength || output.Length < OutputLength)
            {
                return false;
            }

            int jf = _w.JointFeatures, af = _w.ArmFeatures;
            for (int j = 0; j < _w.Joints; j++)
            {
                ReadOnlySpan<float> row = input.Slice(j * jf, jf);
                ReadOnlySpan<float> arm = input.Slice(_w.Joints * jf + _w.ArmOf(j) * af, af);
                row.CopyTo(_a);
                arm.CopyTo(new Span<float>(_a, jf, af));
                float[] h = _a, next = _b;
                for (int k = 0; k < _w.LayerCount; k++)
                {
                    float[] weights = _w.Weights[k], bias = _w.Biases[k];
                    int n = _w.Inputs[k], m = _w.Outputs[k];
                    bool relu = k < _w.LayerCount - 1;
                    for (int o = 0; o < m; o++)
                    {
                        float sum = 0f;
                        int at = o * n;
                        for (int i = 0; i < n; i++) sum += weights[at + i] * h[i];
                        sum += bias[o];
                        next[o] = relu && sum < 0f ? 0f : sum;
                    }

                    float[] t = h;
                    h = next;
                    next = t;
                }

                float cap = _w.FormatVersion == MlpWeights.Version
                    ? row[GripperFlagIndex] * CapGripperExtra + CapBase
                    : _cap + _flagExtra * row[_w.FlagIndex];
                output[2 * j] = cap * (float)Math.Tanh(h[0]);
                float logScale = h[1];
                output[2 * j + 1] = logScale < LogScaleMin ? LogScaleMin : logScale > LogScaleMax ? LogScaleMax : logScale;
            }

            return true;
        }

        /// <summary>Stateless: nothing carries between inferences.</summary>
        public void Reset()
        {
        }
    }
}
