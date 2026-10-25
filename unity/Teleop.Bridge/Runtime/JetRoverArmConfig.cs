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

        // Finding the robot at start (docs/adr/0022): ask through RobotTailscaleHost for its LAN addresses,
        // use the LAN when it answers there, Tailscale when not; when Tailscale is not running here, ask
        // for RemoteHostName over mDNS (docs/adr/0015); RemoteHost (what just push-galaxy-config writes)
        // when nothing answers. False uses RemoteHost alone, as before. An empty RemoteHostName skips
        // mDNS. DiscoveryPort is RobotHost's --discovery-port.
        public bool DiscoverRobot = true;
        public string RobotTailscaleHost = "100.112.90.72";
        public string RemoteHostName = "jetrover-sinrg.local";
        public int DiscoveryPort = 6005;

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
        // _COMMAND_POLL_PERIOD_SECONDS itself, in robot/ros (docs/adr/0019) --
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

        // Buffering (docs/adr/0012-playout-policy-wiring.md). Selected by name from
        // Registry/Registries.cs, the same way PredictorName and ReconcilerName already are --
        // this is the axis a real Tailscale link is most likely to reward, so leaving it the one
        // unselectable axis on the hardware path would be the wrong asymmetry.
        //
        // `immediate` is the default because it is the zero buffer, which is exactly what this
        // bridge did before a playout policy existed: an unedited jetrover_connection.json keeps
        // behaving as it did. `percentile` is the one worth trying over a real link -- see
        // docs/research-log/2026-09-09-percentile-tracking-decisions.md for where it wins and
        // where a tuned `fixed` budget still beats it.
        public string PlayoutPolicyName = "immediate";

        // `fixed`'s constant budget, and `percentile`'s warm-up value before its window fills.
        // Measured from capture, not from arrival: a value below the link's one-way delay buffers
        // nothing at all. Over Tailscale to the Jetson that means anything under ~60-110ms is a
        // no-op, so it is left at 0 rather than at a number that looks configured but is inert.
        public double PlayoutBudgetMs = 0.0;

        public double PlayoutMinBudgetMs = 0.0;
        public double PlayoutMaxBudgetMs = 400.0;

        // Must exceed PlayoutMaxBudgetMs / the command interval, or the buffer discards samples it
        // had room to hold and the late-arrival rate describes the capacity rather than the policy.
        // At 48 Hz (~21ms) a 400ms ceiling spans ~19 samples, so 64 is comfortable.
        public int PlayoutHistoryCapacity = 64;

        public double PlayoutTargetPercentile = 1.0;
        public int PlayoutDelayWindowSamples = 64;

        // Galaxy XR operator input (docs/adr/0017-galaxy-xr-clutch-and-gripper.md). Read by the
        // TeleopXR project's clutch driver; TeleopVR's controllers ignore them.
        //
        // Hand motion -> arm motion while the clutch is engaged: 0.5 means 10 cm of hand moves the
        // wrist 5 cm. The arm reaches about 26 cm, so 1:1 makes an ordinary hand movement a large arm
        // movement.
        public float ClutchScale = 0.5f;

        // The lowest the clutch may put the wrist, metres in the arm-base frame (Z up, 0 = the
        // base-yaw pivot). The arm has no floor limit of its own; tune this on the hardware.
        public float ClutchMinHeightMeters = -0.05f;

        // The gripper hand's thumb-to-index distance: this wide or wider is open (0), this narrow or
        // narrower is closed (1), linear in between.
        public float GripperPinchOpenMeters = 0.08f;
        public float GripperPinchClosedMeters = 0.015f;

        // The gripper hand's twist drives the wrist roll (motor 5), 1:1 from palm-down. Sign is +1
        // or -1, to match how the servo is mounted: flip it if twisting your hand turns the gripper
        // the other way. The limit keeps the command inside what a wrist can comfortably do.
        public float GripperRollSign = 1f;
        public float GripperRollLimitDegrees = 90f;

        // The joint-state twin (docs/adr/0018): which Registries.JointPredictors entry predicts the
        // robot's joints now. relay-replay (the default since 2026-10-20) models the JetRover relay's
        // paced servo writes; servo-replay assumed every command is obeyed at once and snapped back on
        // each fresh reading (core/Teleop.Core/Prediction/CLAUDE.md has the benchmark); hold-last is
        // no prediction. JointModelSpeedRadPerSec is servo-replay's servo speed, rad/s. The prediction
        // horizon is capped by MaxHorizonMs above. relay-replay-servo (docs/adr/0021 decision 15) is
        // relay-replay with a servo behind the gate (the JointServo* fields below). It is the default
        // since 2026-10-25: on the first recordings (7 pilot sessions, trainer/runs/eval-pilot1), with
        // the servo fitted by `armpred sysid`, its error p95 was 3.80 deg against relay-replay's 7.03,
        // and it ran 5 ms ahead of the arm against relay-replay's 100. It was worse on 300 ms bursty
        // jitter (8.16 against 4.08) and corrects a little more; set relay-replay to go back.
        public string JointPredictorName = "relay-replay-servo";
        public float JointModelSpeedRadPerSec = 6f;

        // relay-replay-servo's servo: a speed cap (rad/s), a first-order lag, a fixed pipeline delay
        // from RobotHost's socket to the driver's gate, and each servo's first move sized from its
        // measurement. Lag and pipeline delay are `armpred sysid`'s fit on the 2026-10-25 pilot
        // (trainer/runs/sysid-pilot1b/prior.json, relayReplayServo: 50 ms and 16.8 ms). The speed cap
        // is still Core's guess: no recording yet moved faster than the servo allows. Only
        // relay-replay-servo reads these.
        public float JointServoMaxSpeedRadPerSec = 5f;
        public float JointServoLagMs = 50f;
        public float JointServoPipelineDelayMs = 16.8f;
        public bool JointServoFirstMoveFromMeasured = true;

        // How much older than its frame's stamp each motor's reading is, ms, for relay-replay and
        // relay-replay-servo: `armpred sysid`'s fit on the 2026-10-25 pilot. The robot reads motors 1-5
        // (then the gripper) one after another, about 3 ms each, and the relay forwards each cycle at up
        // to 50 Hz. The 45/35/25/15/5 these replaced assumed 10 ms reads and a 20 Hz relay. Same length
        // and order as the ids.
        public int[] JointReadingAgeMotorIds = { 1, 2, 3, 4, 5 };
        public float[] JointReadingAgeMs = { 30.2f, 25.3f, 22.6f, 20.1f, 17.1f };

        // How fast the gripper's 0..1 fraction moves per radian of servo travel, so the predictor
        // moves it at servo speed like the other joints: RobotHost's PulsePerRadian (1000/240°,
        // 238.7) over its open-to-closed pulse span (645 - 116 = 529, measured 2026-10-25; it was
        // 666.7 and 0.358 before). Change it with GenericArmPlantConfig.GripperOpenPulse/GripperClosedPulse.
        public float GripperFractionPerRadian = 0.4513f;
    }
}
