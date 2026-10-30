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
    ///
    /// <para><b>Version 2</b> (layout v4, the FR3 Duo; docs/adr/0028 decision 2) is version 1's, except for two things.
    /// <c>joints</c> becomes the row count, followed by <c>u32 arms</c>. Before the layers it carries:
    /// <c>u32 n | n bytes: the layout id | rows bytes: each row's arm | u32 handFlagIndex | f32 cap | f32 handCap</c>.
    /// Each row is paired with its own arm's block, and the head's caps and flag come from the file rather than
    /// the layout. Version 1 is read as one arm, with v3's head.</para>
    /// </summary>
    public sealed class MlpWeights
    {
        public const int Version = 1;
        public const int Version2 = 2;
        private static readonly byte[] Magic = { (byte)'T', (byte)'L', (byte)'M', (byte)'W' };

        /// <summary>Layout v3's head, which a version-1 file implies: the gripper's flag at 247, caps of 15 and 90 degrees.</summary>
        public const int V3FlagIndex = 247;
        public static readonly float V3Cap = (float)(15.0 * Math.PI / 180.0);
        public static readonly float V3FlagCap = (float)(90.0 * Math.PI / 180.0);

        private MlpWeights(int version, string layoutId, int jointFeatures, int armFeatures, int joints, int arms, byte[] armOfRow,
            int flagIndex, float cap, float flagCap, float[][] weights, float[][] biases, int[] inputs, int[] outputs, string onnxSha256,
            string modelId)
        {
            FormatVersion = version;
            LayoutId = layoutId;
            JointFeatures = jointFeatures;
            ArmFeatures = armFeatures;
            Joints = joints;
            Arms = arms;
            _armOfRow = armOfRow;
            FlagIndex = flagIndex;
            Cap = cap;
            FlagCap = flagCap;
            Weights = weights;
            Biases = biases;
            Inputs = inputs;
            Outputs = outputs;
            OnnxSha256 = onnxSha256;
            ModelId = modelId;
        }

        private readonly byte[] _armOfRow;

        /// <summary>The file's format version: 1 (layout v3) or 2.</summary>
        public int FormatVersion { get; }

        /// <summary>The window the model takes: <c>v3</c> for a version-1 file, as its version 2 header says otherwise.</summary>
        public string LayoutId { get; }

        public int JointFeatures { get; }
        public int ArmFeatures { get; }

        /// <summary>Rows: one per predicted joint.</summary>
        public int Joints { get; }

        /// <summary>Arm blocks in the input: 1 for a version-1 file.</summary>
        public int Arms { get; }

        /// <summary>The arm block row <paramref name="row"/> is paired with.</summary>
        public int ArmOf(int row) => _armOfRow[row];

        /// <summary>The head: the identity entry that selects <see cref="FlagCap"/> over <see cref="Cap"/>.</summary>
        public int FlagIndex { get; }
        public float Cap { get; }
        public float FlagCap { get; }

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
            if (version != Version && version != Version2)
            {
                error = $"weights format {version}, this build reads {Version} and {Version2}";
                return false;
            }

            int jointFeatures = ReadInt(data, ref at), armFeatures = ReadInt(data, ref at), joints = ReadInt(data, ref at);
            int arms = version == Version2 ? ReadInt(data, ref at) : 1;
            int layers = ReadInt(data, ref at);
            if (jointFeatures <= 0 || armFeatures <= 0 || joints <= 0 || joints > 64 || arms <= 0 || arms > joints || layers <= 0 || layers > 16)
            {
                error = "bad header";
                return false;
            }

            string layoutId = "v3";
            var armOfRow = new byte[joints];
            int flagIndex = V3FlagIndex;
            float cap = V3Cap, flagCap = V3FlagCap;
            if (version == Version2)
            {
                if (data.Length - at < 4)
                {
                    error = "cut off before the layout id";
                    return false;
                }

                int n = ReadInt(data, ref at);
                if (n <= 0 || n > 32 || data.Length - at < n + joints + 12)
                {
                    error = "bad layout id";
                    return false;
                }

                layoutId = Encoding.UTF8.GetString(data.Slice(at, n).ToArray());
                at += n;
                for (int r = 0; r < joints; r++)
                {
                    armOfRow[r] = data[at++];
                    if (armOfRow[r] >= arms)
                    {
                        error = $"row {r} is paired with arm {armOfRow[r]} of {arms}";
                        return false;
                    }
                }

                flagIndex = ReadInt(data, ref at);
                cap = BitConverter.Int32BitsToSingle(ReadInt(data, ref at));
                flagCap = BitConverter.Int32BitsToSingle(ReadInt(data, ref at));
                if (flagIndex < 0 || flagIndex >= jointFeatures || !(cap > 0f) || !(flagCap > 0f))
                {
                    error = "bad head";
                    return false;
                }
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
            weights = new MlpWeights(version, layoutId, jointFeatures, armFeatures, joints, arms, armOfRow, flagIndex, cap, flagCap,
                w, b, ins, outs, hex.ToString(), id);
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
