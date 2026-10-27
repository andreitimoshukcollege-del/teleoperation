using System;
using System.Buffers.Binary;
using System.Text;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Inference
{
    /// <summary>
    /// An architecture-A (mlp) model's weights, parsed from the trainer's <c>model.weights</c>
    /// (trainer/contract/MODEL_FORMAT.md, <i>Weights</i>; docs/adr/0025). The host reads the file; Core
    /// parses the bytes, since it does no I/O. Little-endian:
    /// <c>"TLMW" | u32 version (1) | u32 jointFeatures | u32 armFeatures | u32 joints | u32 layers |
    /// layers × { u32 inputs | u32 outputs | f32 weight[outputs × inputs] (row-major, one row per output) |
    /// f32 bias[outputs] } | 32 bytes: the source .onnx's sha256 | u32 n | n bytes: the model id (UTF-8)</c>.
    /// </summary>
    public sealed class MlpWeights
    {
        public const int Version = 1;
        private static readonly byte[] Magic = { (byte)'T', (byte)'L', (byte)'M', (byte)'W' };

        private MlpWeights(int jointFeatures, int armFeatures, int joints, float[][] weights, float[][] biases, int[] inputs,
            int[] outputs, string onnxSha256, string modelId)
        {
            JointFeatures = jointFeatures;
            ArmFeatures = armFeatures;
            Joints = joints;
            Weights = weights;
            Biases = biases;
            Inputs = inputs;
            Outputs = outputs;
            OnnxSha256 = onnxSha256;
            ModelId = modelId;
        }

        public int JointFeatures { get; }
        public int ArmFeatures { get; }
        public int Joints { get; }
        public int LayerCount => Weights.Length;

        /// <summary>Per layer: <c>outputs × inputs</c>, row <c>o</c> at <c>o * inputs</c>.</summary>
        internal float[][] Weights { get; }
        internal float[][] Biases { get; }
        internal int[] Inputs { get; }
        internal int[] Outputs { get; }

        /// <summary>The scored <c>.onnx</c> these weights came from, as lowercase hex.</summary>
        public string OnnxSha256 { get; }

        /// <summary>The trainer's name for the model, e.g. <c>finetune-pilot2-early7 dcdb5669a970</c>.</summary>
        public string ModelId { get; }

        /// <summary>
        /// Parses a weights file. False, with a reason, for anything malformed: a wrong magic or version,
        /// a layer chain that does not connect, a size that does not match, or bytes left over.
        /// </summary>
        public static bool TryParse(ReadOnlySpan<byte> data, out MlpWeights? weights, out string error)
        {
            weights = null;
            int at = 0;
            if (data.Length < 24 || !data.Slice(0, 4).SequenceEqual(Magic))
            {
                error = "not a model.weights file";
                return false;
            }

            at = 4;
            int version = ReadInt(data, ref at);
            if (version != Version)
            {
                error = $"weights format {version}, this build reads {Version}";
                return false;
            }

            int jointFeatures = ReadInt(data, ref at), armFeatures = ReadInt(data, ref at), joints = ReadInt(data, ref at);
            int layers = ReadInt(data, ref at);
            if (jointFeatures <= 0 || armFeatures <= 0 || joints <= 0 || layers <= 0 || layers > 16)
            {
                error = "bad header";
                return false;
            }

            var w = new float[layers][];
            var b = new float[layers][];
            var ins = new int[layers];
            var outs = new int[layers];
            for (int k = 0; k < layers; k++)
            {
                if (data.Length - at < 8)
                {
                    error = $"layer {k}: cut off";
                    return false;
                }

                ins[k] = ReadInt(data, ref at);
                outs[k] = ReadInt(data, ref at);
                int expectedIn = k == 0 ? jointFeatures + armFeatures : outs[k - 1];
                if (ins[k] != expectedIn || outs[k] <= 0 || outs[k] > 4096)
                {
                    error = $"layer {k}: {ins[k]} -> {outs[k]} does not follow from {expectedIn}";
                    return false;
                }

                long floats = (long)ins[k] * outs[k] + outs[k];
                if (data.Length - at < floats * 4)
                {
                    error = $"layer {k}: cut off";
                    return false;
                }

                w[k] = ReadFloats(data, ref at, ins[k] * outs[k]);
                b[k] = ReadFloats(data, ref at, outs[k]);
            }

            if (outs[layers - 1] != 2)
            {
                error = $"the last layer has {outs[layers - 1]} outputs; the head takes 2 (a correction and a log-scale)";
                return false;
            }

            if (data.Length - at < 36)
            {
                error = "no provenance";
                return false;
            }

            var hex = new StringBuilder(64);
            for (int i = 0; i < 32; i++) hex.Append(data[at + i].ToString("x2", System.Globalization.CultureInfo.InvariantCulture));
            at += 32;
            int idLength = ReadInt(data, ref at);
            if (idLength < 0 || data.Length - at != idLength)
            {
                error = "the model id's length does not match the file";
                return false;
            }

            string id = Encoding.UTF8.GetString(data.Slice(at, idLength).ToArray());
            weights = new MlpWeights(jointFeatures, armFeatures, joints, w, b, ins, outs, hex.ToString(), id);
            error = "";
            return true;
        }

        private static int ReadInt(ReadOnlySpan<byte> data, ref int at)
        {
            int v = BinaryPrimitives.ReadInt32LittleEndian(data.Slice(at, 4));
            at += 4;
            return v;
        }

        private static float[] ReadFloats(ReadOnlySpan<byte> data, ref int at, int n)
        {
            var out_ = new float[n];
            for (int i = 0; i < n; i++)
            {
                out_[i] = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(data.Slice(at, 4)));
                at += 4;
            }

            return out_;
        }
    }
}
