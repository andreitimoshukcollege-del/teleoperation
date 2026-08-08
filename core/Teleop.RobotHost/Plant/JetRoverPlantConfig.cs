using System;
using Teleop.RobotHost.Kinematics;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// <see cref="JetRoverPlant"/>'s tunable knobs. Unlike <c>RigidBodyPlant</c> (Core's plant,
    /// deliberately config-free because it has no real research knobs), this plant drives real
    /// motors and has several genuinely safety- and calibration-relevant parameters -- a config
    /// struct is the right call here, not ceremony.
    /// </summary>
    public readonly struct JetRoverPlantConfig
    {
        /// <summary>Ruler-measured link lengths for <see cref="FourDofArmKinematics"/>.</summary>
        public readonly ArmLinkLengths Links;

        /// <summary>
        /// Pulse units per radian, for converting an IK joint-angle delta into the relay's
        /// "direction" units. Confirmed against Hiwonder's own published docs: the servo's full
        /// travel is 0-1000 pulse = 0-240 degrees (<b>not</b> the 180 degrees
        /// `ServoController.pulseToDeg` in the ported ROS SDK assumes -- that mismatch is a
        /// separate, pre-existing inaccuracy in that file, deliberately not touched here; this
        /// plant does its own pulse/radian conversion independently and only ever sends
        /// <c>direction</c> units over the wire, never degrees, for these four joints).
        /// </summary>
        public readonly float PulsePerRadian;

        /// <summary>
        /// Pulse units per degree using the ROS SDK's own (separate, pre-existing) 180-degree
        /// assumption -- <b>only</b> for reversing <c>ServoController.pulseToDeg</c>'s conversion
        /// when interpreting an incoming feedback reading (which that function already produced
        /// using that assumption before publishing it as a ROS topic value), so the round trip
        /// through degrees on the wire is numerically exact. Every other angle computation in
        /// this plant uses <see cref="PulsePerRadian"/> (the confirmed-correct 240-degree range)
        /// -- the two must not be conflated.
        /// </summary>
        public readonly float PulsePerDegreeAssumed180;

        /// <summary>
        /// <c>ServoController.setPos</c>'s own fixed step size (50 pulse per unit of
        /// "direction") -- needed to convert a desired pulse delta into the direction value that
        /// produces it: <c>direction = pulseDelta / StepSizePulses</c>.
        /// </summary>
        public readonly float StepSizePulses;

        /// <summary>
        /// Hard clamp on the magnitude of any single direction value sent to the relay,
        /// independent of the computed delta -- a safety backstop against a bug in the IK/
        /// tracking computation (or a bad <c>CommandFrame.Pose</c> from upstream) commanding an
        /// oversized single step.
        /// </summary>
        public readonly float MaxDirectionMagnitude;

        /// <summary>Servo pulse value corresponding to joint angle 0 -- the arm's own "center" (matches <c>resetArm</c>'s pulse 500 for every joint).</summary>
        public readonly int ZeroPulse;

        /// <summary>Full raw travel range, in pulse units (0-1000 on this hardware) -- joint-angle targets are clamped to this before being converted to a direction delta.</summary>
        public readonly int MinPulse;
        public readonly int MaxPulse;

        /// <summary>
        /// Per-joint override of <see cref="MaxPulse"/> for the lower-arm joint specifically --
        /// a real mechanical limit, not a research knob: on 2026-08-08 the lower arm collided
        /// with the robot's own base plate at a real physical target, straining the servo against
        /// the obstruction until the operator manually commanded it back. <see cref="MaxPulse"/>
        /// (1000, the servo's full electrical travel) does not reflect this robot's actual usable
        /// range once the plate is accounted for. Defaults to <see cref="MaxPulse"/> (no
        /// additional restriction) so a fresh robot/config with different mechanical clearance
        /// isn't silently limited by this one -- <c>Teleop.RobotHost</c>'s
        /// <c>--lower-arm-max-pulse</c> sets this explicitly for the real hardware, calibrated to
        /// 50 pulse (~9 degrees in <see cref="PulsePerDegreeAssumed180"/>'s space) by iterative
        /// real-hardware testing with a human confirming clearance at each step (see
        /// robot/README.md). An initial calibration pass landed on a higher, wrong value because
        /// the ROS-side servo's own 300ms per-move cooldown was silently dropping the final small
        /// correction at the sender's default rate -- the arm never actually reached the
        /// commanded pulse, so the "safe" value that pass converged on didn't reflect where the
        /// arm would land once that race was fixed. Recalibrated at a slower send rate afterward.
        /// </summary>
        public readonly int LowerArmMaxPulse;

        /// <summary>
        /// Gripper open/closed servo-degree values, in the *ROS SDK's own* assumed 0-180 degree
        /// space (<c>ServoController.degToPulse</c>) -- not the corrected 240-degree range above,
        /// since <see cref="Teleop.RobotHost.Relay.LocalArmCommand.GripperDegrees"/> passes
        /// through to that function unmodified. Defaults are a plausible half-open range, not yet
        /// calibrated against the real gripper's actual travel.
        /// </summary>
        public readonly float GripperOpenDegrees;
        public readonly float GripperClosedDegrees;

        public JetRoverPlantConfig(
            ArmLinkLengths links,
            float pulsePerRadian,
            float pulsePerDegreeAssumed180,
            float stepSizePulses,
            float maxDirectionMagnitude,
            int zeroPulse,
            int minPulse,
            int maxPulse,
            float gripperOpenDegrees,
            float gripperClosedDegrees,
            int? lowerArmMaxPulse = null)
        {
            Links = links;
            PulsePerRadian = pulsePerRadian;
            PulsePerDegreeAssumed180 = pulsePerDegreeAssumed180;
            StepSizePulses = stepSizePulses;
            MaxDirectionMagnitude = maxDirectionMagnitude;
            ZeroPulse = zeroPulse;
            MinPulse = minPulse;
            MaxPulse = maxPulse;
            GripperOpenDegrees = gripperOpenDegrees;
            GripperClosedDegrees = gripperClosedDegrees;
            LowerArmMaxPulse = lowerArmMaxPulse ?? maxPulse;
        }

        public static JetRoverPlantConfig Default => new JetRoverPlantConfig(
            links: ArmLinkLengths.Measured,
            pulsePerRadian: 1000f / (240f * MathF.PI / 180f),
            pulsePerDegreeAssumed180: 1000f / 180f,
            stepSizePulses: 50f,
            maxDirectionMagnitude: 5f,
            zeroPulse: 500,
            minPulse: 0,
            maxPulse: 1000,
            gripperOpenDegrees: 30f,
            gripperClosedDegrees: 150f);
    }
}
