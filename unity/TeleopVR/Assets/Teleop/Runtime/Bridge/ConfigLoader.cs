using System;
using System.IO;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Loads <see cref="DisplayCalibrationConfig"/> the way Teleop/CLAUDE.md's Quest constraints
    /// require: no arbitrary filesystem paths. An override at
    /// <c>Application.persistentDataPath/display_calibration.json</c> -- pushed with
    /// <c>adb push</c>, no rebuild -- wins if present, so recalibrating on-device never requires a
    /// new APK. Otherwise falls back to the <c>Resources</c> default shipped in the build.
    ///
    /// Never throws: a missing or malformed override/default falls through to
    /// <c>new DisplayCalibrationConfig()</c>'s hardcoded placeholder rather than blocking Play
    /// mode, since a wrong-but-present calibration is far easier to notice (the HUD's number will
    /// look implausible) than a crash on startup.
    /// </summary>
    public static class ConfigLoader
    {
        private const string OverrideFileName = "display_calibration.json";
        private const string ResourceName = "display_calibration";

        public static DisplayCalibrationConfig Load()
        {
            string overridePath = Path.Combine(Application.persistentDataPath, OverrideFileName);
            if (File.Exists(overridePath))
            {
                if (TryParse(File.ReadAllText(overridePath), out DisplayCalibrationConfig fromOverride))
                {
                    return fromOverride;
                }

                Debug.LogWarning($"Teleop: {overridePath} exists but failed to parse; falling back.");
            }

            TextAsset defaultAsset = Resources.Load<TextAsset>(ResourceName);
            if (defaultAsset != null && TryParse(defaultAsset.text, out DisplayCalibrationConfig fromResource))
            {
                return fromResource;
            }

            Debug.LogWarning("Teleop: no display_calibration config found; using an uncalibrated placeholder.");
            return new DisplayCalibrationConfig();
        }

        private static bool TryParse(string json, out DisplayCalibrationConfig config)
        {
            try
            {
                config = JsonUtility.FromJson<DisplayCalibrationConfig>(json);
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
