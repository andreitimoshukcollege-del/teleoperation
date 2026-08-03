using System;

namespace Teleop.Bridge
{
    /// <summary>
    /// The calibrated constant behind <c>t_photon = t_render + DisplayOffset</c>
    /// (docs/metrics.md §1, <c>Types/LatencyTrace.WithPhotonTicks</c>'s doc). Plain
    /// <see cref="Serializable"/> data, not a ScriptableObject: it round-trips through
    /// <c>UnityEngine.JsonUtility</c> so <see cref="ConfigLoader"/> can read it from either a
    /// <c>Resources</c> default or an <c>Application.persistentDataPath</c> override pushed with
    /// <c>adb push</c> -- no rebuild required, per Teleop/CLAUDE.md's Quest constraints.
    ///
    /// <see cref="DisplayOffsetMilliseconds"/>'s shipped default is an admittedly invented
    /// placeholder, not a measurement. See docs/adr/0003-display-offset-calibration.md for the
    /// physical procedure that replaces it with a real number for a specific headset and refresh
    /// rate -- Gate 4 (docs/setup.md) is not closed until that happens.
    /// </summary>
    [Serializable]
    public sealed class DisplayCalibrationConfig
    {
        public float DisplayOffsetMilliseconds = 20f;
        public string HeadsetModel = "uncalibrated";
        public float RefreshRateHz = 90f;
    }
}
