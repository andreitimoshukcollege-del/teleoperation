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
        // MoveArmArgs.DefaultRateHz's own comment describes a real ~300ms-cooldown incident, but
        // that number was specific to move-arm/ClockSyncCheckCommand's one-shot-target use case at
        // the old, slower PulsesPerSecond -- it does NOT apply directly here. Ground-truthed
        // against the real servo_controller.py on the Jetson (2026-08-22): the actual per-servo
        // floor is `duration = max(0.05s, pulseDelta / PulsesPerSecond)`, and a setPos() call
        // arriving inside another move's cooldown is silently COALESCED (not queued, not lost
        // forever) -- explicit, documented behavior on that side, meant to absorb a fast burst
        // into one physical move rather than block on a redundant serial round trip per packet.
        //
        // That coalescing is safe here specifically because JetRoverOperatorBridge.Update()
        // unconditionally re-sends the current target every interval, even when stationary (see
        // below) -- an occasionally-coalesced intermediate write is corrected by the very next
        // tick once the servo's cooldown clears. move-arm/ClockSyncCheckCommand send a fixed
        // target once and stop, so the same reasoning does NOT transfer to their own 2Hz, which
        // stays unchanged.
        //
        // Raised 2.0 -> 40.0 -> 48.0 (2026-08-22, three rounds of operator feedback -- 40Hz felt
        // "much better" but still noticeably laggy on real hardware). 48Hz is 96% of
        // relay_node.py's own fixed 50Hz local-socket poll rate (_COMMAND_POLL_PERIOD_SECONDS),
        // which is the actual hard ceiling on this path: sending faster than that cannot be
        // noticed any sooner on the Jetson side, so this is close to the practical maximum this
        // knob alone can deliver. Breaking past 50Hz for real would mean raising
        // _COMMAND_POLL_PERIOD_SECONDS itself, on the Jetson's separate jetrover-teleop-ros repo --
        // not done here, a bigger cross-repo change that needs its own explicit go-ahead. Watch
        // the next real-hardware pass for jerkiness/buzzing/strain; fall back toward 40.0/20.0 if
        // it shows any of that.
        public double CommandRateHz = 48.0;

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
