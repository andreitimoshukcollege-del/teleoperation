using System;
using System.IO;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Loads a plain config type the way Teleop/CLAUDE.md's Quest constraints require: no
    /// arbitrary filesystem paths. An override at
    /// <c>Application.persistentDataPath/&lt;overrideFileName&gt;</c> -- pushed with
    /// <c>adb push</c>, no rebuild -- wins if present, so recalibrating on-device never requires a
    /// new APK. Otherwise falls back to the <c>Resources</c> default shipped in the build.
    ///
    /// Never throws: a missing or malformed override/default falls through to the caller-supplied
    /// <paramref name="fallback"/> rather than blocking Play mode, since a wrong-but-present
    /// config is far easier to notice (a HUD number, or an arm that won't move, will look
    /// implausible) than a crash on startup.
    ///
    /// Generic since <see cref="DisplayCalibrationConfig"/> (Phase 4) and
    /// <c>JetRoverArmConfig</c> (the JetRover VR feature) both need the exact same
    /// override-then-Resources-then-fallback logic, just against different files.
    /// </summary>
    public static class ConfigLoader
    {
        private const string DisplayOverrideFileName = "display_calibration.json";
        private const string DisplayResourceName = "display_calibration";

        public static DisplayCalibrationConfig Load() =>
            Load(DisplayResourceName, DisplayOverrideFileName, new DisplayCalibrationConfig());

        public static T Load<T>(string resourceName, string overrideFileName, T fallback) where T : class
        {
            string overridePath = Path.Combine(Application.persistentDataPath, overrideFileName);
            if (File.Exists(overridePath))
            {
                if (TryParse(File.ReadAllText(overridePath), out T fromOverride))
                {
                    return fromOverride;
                }

                Debug.LogWarning($"Teleop: {overridePath} exists but failed to parse; falling back.");
            }

            TextAsset defaultAsset = Resources.Load<TextAsset>(resourceName);
            if (defaultAsset != null && TryParse(defaultAsset.text, out T fromResource))
            {
                return fromResource;
            }

            Debug.LogWarning($"Teleop: no {resourceName} config found; using the provided fallback.");
            return fallback;
        }

        private static bool TryParse<T>(string json, out T config) where T : class
        {
            try
            {
                config = JsonUtility.FromJson<T>(json);
                return config != null;
            }
            catch (Exception)
            {
                config = null;
                return false;
            }
        }
    }
}
