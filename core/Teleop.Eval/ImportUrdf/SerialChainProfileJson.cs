using System.IO;
using System.Text.Json;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.ImportUrdf
{
    /// <summary>
    /// Reads and writes a <see cref="SerialChainProfileDocument"/> (docs/adr/0026). The document type
    /// is shared with Unity and RobotHost; only the file handling is this host's. Output is
    /// byte-stable: indented, LF line endings on every platform, a final newline, and numbers in
    /// .NET's shortest round-trip form.
    /// </summary>
    internal static class SerialChainProfileJson
    {
        private static readonly JsonSerializerOptions Options = new JsonSerializerOptions
        {
            IncludeFields = true,
            WriteIndented = true,
        };

        public static string Serialize(SerialChainProfileDocument document) =>
            JsonSerializer.Serialize(document, Options).Replace("\r\n", "\n") + "\n";

        public static SerialChainProfileDocument Deserialize(string json) =>
            JsonSerializer.Deserialize<SerialChainProfileDocument>(json, Options)
            ?? throw new InvalidDataException("the profile deserialized to null");

        /// <summary>Loads and validates a serial-chain profile file.</summary>
        public static SerialChainProfile Load(string path)
        {
            try
            {
                return Deserialize(File.ReadAllText(path)).ToProfile();
            }
            catch (System.FormatException e)
            {
                throw new InvalidDataException($"robot profile at '{path}': {e.Message}", e);
            }
        }
    }
}
