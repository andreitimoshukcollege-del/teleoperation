using System;
using System.IO;
using System.Text.Json;
using Teleop.RobotArm.Types;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// Loads a serial-chain robot profile (docs/adr/0026), and tells the two profile kinds apart. The document
    /// type is shared (<see cref="SerialChainProfileDocument"/>); only the file handling is this host's, a
    /// small copy of <c>Teleop.Eval</c>'s, as <see cref="RobotArmProfileJson"/> is.
    /// </summary>
    public static class SerialChainProfileJson
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions { IncludeFields = true };

        /// <summary>Whether the profile file at <paramref name="path"/> says <c>"kind": "serial-chain"</c>. A file without a kind is ADR 0011's planar profile.</summary>
        /// A file that is not JSON is not one either: the planar loader then reports what is wrong with it.
        public static bool IsSerialChain(string path)
        {
            try
            {
                using JsonDocument document = JsonDocument.Parse(File.ReadAllText(path));
                return document.RootElement.ValueKind == JsonValueKind.Object &&
                    document.RootElement.TryGetProperty("kind", out JsonElement kind) &&
                    kind.ValueKind == JsonValueKind.String &&
                    kind.GetString() == SerialChainProfile.Kind;
            }
            catch (JsonException)
            {
                return false;
            }
        }

        public static SerialChainProfile Load(string path)
        {
            try
            {
                SerialChainProfileDocument document = JsonSerializer.Deserialize<SerialChainProfileDocument>(File.ReadAllText(path), Options)
                    ?? throw new InvalidDataException($"robot profile at '{path}' deserialized to null");
                return document.ToProfile();
            }
            catch (FormatException e)
            {
                throw new InvalidDataException($"robot profile at '{path}': {e.Message}", e);
            }
        }
    }
}
