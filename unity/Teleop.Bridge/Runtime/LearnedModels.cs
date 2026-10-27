using System.IO;
using Teleop.Core.Contracts;
using Teleop.Core.Inference;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Finds a learned joint model by name for a <c>learned</c> preset (docs/adr/0025): an override in
    /// <c>Application.persistentDataPath/models/&lt;name&gt;.weights</c> (pushed with
    /// <c>just push-galaxy-model</c>, no rebuild), else the build's
    /// <c>Resources/models/&lt;name&gt;.bytes</c>. Reading the file is all this does; the parse and the
    /// model are Core's (<see cref="MlpWeights"/>, <see cref="MlpBackend"/>). Called once per preset at
    /// start, never on the frame path.
    /// </summary>
    public static class LearnedModels
    {
        public static bool TryLoad(string name, out IInferenceBackend model, out string error)
        {
            model = null;
            if (string.IsNullOrEmpty(name) || name.IndexOfAny(new[] { '/', '\\', ':' }) >= 0 || name.Contains(".."))
            {
                error = $"'{name}' is not a model name";
                return false;
            }

            byte[] bytes;
            string source;
            string overridePath = Path.Combine(Application.persistentDataPath, "models", name + ".weights");
            if (File.Exists(overridePath))
            {
                bytes = File.ReadAllBytes(overridePath);
                source = overridePath;
            }
            else
            {
                TextAsset asset = Resources.Load<TextAsset>("models/" + name);
                if (asset == null)
                {
                    error = $"no model '{name}' (neither {overridePath} nor Resources/models/{name}.bytes)";
                    return false;
                }

                bytes = asset.bytes;
                source = "Resources/models/" + name + ".bytes";
            }

            if (!MlpWeights.TryParse(bytes, out MlpWeights weights, out string parseError))
            {
                error = $"{source}: {parseError}";
                return false;
            }

            model = new MlpBackend(weights);
            Debug.Log($"[twin] learned model {model.ModelId} from {source}");
            error = "";
            return true;
        }

        /// <summary>
        /// For the editor's checks, which see Bridge only: loads <paramref name="name"/> as a preset would
        /// and runs it once on an all-zero window. False, with the reason, if either fails.
        /// </summary>
        public static bool Check(string name, out string modelId, out string error)
        {
            modelId = "";
            if (!TryLoad(name, out IInferenceBackend model, out error)) return false;
            modelId = model.ModelId;
            var output = new float[model.OutputLength];
            if (!model.TryRun(new float[model.InputLength], output))
            {
                error = $"model {modelId} did not run";
                return false;
            }

            foreach (float v in output)
            {
                if (float.IsNaN(v) || float.IsInfinity(v))
                {
                    error = $"model {modelId} gave a non-finite output";
                    return false;
                }
            }

            return true;
        }
    }
}
