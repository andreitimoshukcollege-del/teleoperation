using System;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Connection, safety, and algorithm-selection settings for <see cref="JetRoverOperatorBridge"/>
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). Plain <see cref="Serializable"/>
    /// data, same pattern as <see cref="DisplayCalibrationConfig"/>: round-trips through
    /// <c>UnityEngine.JsonUtility</c>, loaded via <see cref="ConfigLoader.Load{T}"/> from a
    /// <c>Resources</c> default overridable at <c>Application.persistentDataPath</c> (no rebuild
    /// needed to point at a different robot or try a different predictor).
    ///
    /// Defaults for the predictor/reconciler/clock-sync numeric fields match
    /// <c>core/Teleop.Eval/MoveArm/MoveArmCommand.cs</c>'s values exactly -- the only values ever
    /// proven against this real robot -- per Core's own "no invented numbers" rule for these
    /// config structs. They are overridable, not hardcoded, so this feature can actually be used
    /// to test a different predictor/network profile, not just move the arm.
    /// </summary>
    [Serializable]
    public sealed class JetRoverArmConfig
    {
        [Header("Connection -- must match an already-running Teleop.RobotHost")]
        public string RemoteHost = "100.112.90.72";
        public int RemotePort = 6000; // Teleop.RobotHost's --local-port (Cartesian/move-arm path)
        public int LocalPort = 6001; // must match Teleop.RobotHost's --remote-port
        public int JointRemotePort = 6002; // Teleop.RobotHost's --joint-local-port
        public int JointLocalPort = 0; // 0 = OS-assigned ephemeral port; this channel only ever sends

        [Header("Safety -- see JetRoverOperatorBridge's own doc for why these exist")]
        // MoveArmArgs.DefaultRateHz's own comment records the real incident this guards against:
        // the real servo's ~300ms per-move cooldown silently drops a correction sent faster than
        // that. Do not raise this without re-reading that incident.
        public double CommandRateHz = 2.0;

        // A human must deliberately arm real motion after confirming physical clearance -- this
        // feature's equivalent of MoveArmArgs.ConfirmHardwareMotion/--confirm-hardware-motion.
        // Connectivity/replies still work either way, so "is the Jetson responding" is visible
        // before arming motion.
        public bool ConfirmHardwareMotion = false;

        [Header("Algorithm selection -- Registries.Predictors/Reconcilers keys")]
        public string PredictorName = "none";
        public string ReconcilerName = "snap";

        [Header("Network impairment -- Teleop.Core.Transport.NetworkProfileCatalog name, empty = none")]
        public string NetworkProfileName = "lan";

        [Header("PredictorConfig (see Teleop.Core.Types.PredictorConfig for which predictor reads which field)")]
        public double MaxHorizonMs = 500.0;
        public double MaxObservationGapMs = 1000.0;
        public int PredictorHistoryCapacity = 16;
        public float SmoothingAlpha = 0.3f;
        public float SmoothingBeta = 0.1f;
        public float ProcessNoise = 0.01f;
        public float MeasurementNoise = 0.001f;
        public float MaxLinearSpeed = 10f;
        public float MaxAngularSpeed = 10f;

        [Header("ReconcilerConfig")]
        public float ConvergencePositionToleranceMeters = 0.005f;
        public float ConvergenceOrientationToleranceRadians = 0.017f;
        public double MaxTimeToConvergenceMs = 1000.0;
        public float MaxCorrectionLinearSpeedMetersPerSecond = 5f;
        public float MaxCorrectionAngularSpeedRadPerSecond = 10f;
        public int RollbackHistoryCapacity = 16;

        [Header("ClockSyncConfig")]
        public int ClockSyncHistoryCapacity = 32;
        public float ClockSyncSmoothingAlpha = 0.2f;
        public double MaxAcceptableRttMs = 2000.0;
        public double OutlierRttMultiple = 3.0;
        public int MinSamplesBeforeTrusted = 3;
    }
}
