using System;
using System.IO;
using Teleop.RobotArm.World;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// What both hosts (<c>Program</c>, the JetRover's, and <c>SerialChainHost</c>, the FR3 Duo's) start the same way: the
    /// automatic sessions (docs/adr/0023 decision 4), the RECORD button's takes (docs/adr/0033) with their disk thresholds,
    /// and the scene file (docs/adr/0031 amendment 2). One copy, so an option added for one robot reaches the other.
    /// </summary>
    internal static class HostRecording
    {
        /// <summary>Free space where the recordings go, or null when it cannot be read.</summary>
        internal static long? FreeBytes(string path)
        {
            try
            {
                return new DriveInfo(path).AvailableFreeSpace;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is ArgumentException)
            {
                return null;
            }
        }

        /// <summary>Automatic sessions into the recorder's auto root while commanded; null without <c>--auto-record</c>.</summary>
        internal static AutoSessionRecorder? StartAuto(AutoRecordOptions? options, SessionRecorder? recorder, long ticksPerSecond,
            Func<UplinkCondition> uplink, Action<string> log)
        {
            if (recorder == null || options == null) return null;
            string root = recorder.AutoRoot;
            Directory.CreateDirectory(root);
            var auto = new AutoSessionRecorder(options, recorder, ticksPerSecond, () => DateTime.UtcNow, () => FreeBytes(root), uplink, log);
            long? free = FreeBytes(root);
            log($"[record] automatic sessions into {root} while commanded ({options}); " +
                (free.HasValue ? $"{free.Value / 1073741824.0:0.00} GB free now" : "free space unknown"));
            return auto;
        }

        /// <summary>
        /// The RECORD button's takes under <c>--scenarios</c> (<paramref name="scenariosRoot"/>), with the free-disk
        /// thresholds when given (<c>--scenario-start-free-gb</c>, <c>--scenario-stop-free-gb</c>: docs/adr/0033 amendment 1,
        /// the Jetson's disk being smaller than the GPU machine's); null without <c>--scenarios</c>.
        /// </summary>
        internal static ScenarioRecorder? StartScenarios(string? scenariosRoot, double? startFreeGigabytes, double? stopFreeGigabytes,
            SessionRecorder? recorder, AutoSessionRecorder? auto, long ticksPerSecond, Func<UplinkCondition> uplink, Action<string> log)
        {
            if (recorder == null || scenariosRoot == null) return null;
            var folder = new ScenarioFolder(scenariosRoot);
            folder.EnsureExists();
            string root = Path.GetFullPath(scenariosRoot);
            var scenarios = new ScenarioRecorder(root, recorder, auto, folder, ticksPerSecond, () => DateTime.UtcNow, () => FreeBytes(root), uplink, log);
            if (startFreeGigabytes is double startFree) scenarios.StartFreeGigabytes = startFree;
            if (stopFreeGigabytes is double stopFree) scenarios.StopFreeGigabytes = stopFree;
            log($"[scenario] the headset's RECORD button records into {root} (the list: {Path.Combine(root, ScenarioList.FileName)}): " +
                $"a take starts with {scenarios.StartFreeGigabytes:0.#} GB free and stops below {scenarios.StopFreeGigabytes:0.#} GB");
            return scenarios;
        }

        /// <summary>
        /// The scene file, refused unless it is for <paramref name="profileName"/>, with its sha256 (what the arm server's
        /// WORLD_INFO must match). Throws what reading or parsing it throws.
        /// </summary>
        internal static WorldScene LoadScene(string path, string profileName, out byte[] sha256)
        {
            byte[] bytes = File.ReadAllBytes(path);
            WorldScene scene = System.Text.Json.JsonSerializer.Deserialize<SceneDocument>(bytes, new System.Text.Json.JsonSerializerOptions { IncludeFields = true })!.ToScene();
            if (scene.RobotProfile != profileName) throw new InvalidDataException($"the scene is for '{scene.RobotProfile}', the profile is '{profileName}'");
            sha256 = System.Security.Cryptography.SHA256.HashData(bytes);
            return scene;
        }

        /// <summary>The scene each session records in its session.json.</summary>
        internal static void NoteScene(AutoRecordOptions? options, string path, WorldScene scene, ulong seed)
        {
            if (options == null) return;
            options.ScenePath = path;
            options.SceneName = scene.Name;
            options.SceneSeed = seed;
        }
    }
}
