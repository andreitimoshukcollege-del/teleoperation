using System;
using Teleop.RobotArm.Types;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// <see cref="GenericArmPlant"/>'s tunable knobs, generalized from <c>JetRoverPlantConfig</c>
    /// (docs/adr/0011-generic-robot-arm-profiles.md) to carry a <see cref="RobotArmProfile"/>
    /// instead of the old hardcoded <c>ArmLinkLengths</c>. Unlike <c>RigidBodyPlant</c> (Core's
    /// plant, deliberately config-free because it has no real research knobs), this plant drives
    /// real motors and has several genuinely safety- and calibration-relevant parameters -- a
    /// config struct is the right call here, not ceremony.
    ///
    /// <c>PulsePerDegreeAssumed180</c> and its <c>DegreesToPulse</c> conversion are gone
    /// entirely: feedback now carries raw pulse directly (docs/adr/0011), so there is no degree
    /// round-trip left to reverse.
    /// </summary>
    public readonly struct GenericArmPlantConfig
    {
        /// <summary>This robot's topology and geometry.</summary>
        public readonly RobotArmProfile Profile;

        /// <summary>
        /// Pulse units per radian, for converting an arm joint's IK angle into a pulse target.
        /// Confirmed against Hiwonder's own published docs for the JetRover: full travel is
        /// 0-1000 pulse = 0-240 degrees. A different robot's profile may need a different value --
        /// this stays a plant-wide scalar (not per-joint) since nothing so far needs otherwise.
        /// </summary>
        public readonly float PulsePerRadian;

        /// <summary>
        /// Move speed sent to the relay for every joint, pulses/second -- replaces the old
        /// Python-side hardcoded <c>PULSES_PER_SECOND</c> module constant
        /// (jetrover-teleop-ros/servo_controller.py) with one centrally configured value sent
        /// explicitly over the wire (docs/adr/0011), so the two independently-maintained
        /// deployments no longer have to be kept in sync by hand for this number.
        ///
        /// Raised 300 -> 900 -> 1500 -> 2200 (2026-08-22, three rounds of operator feedback
        /// chasing "faster"/"less laggy," each round tested live against real hardware and
        /// confirmed clean -- no jerkiness, buzzing, or strain reported at 1500 -- before pushing
        /// further). This is also the actual lever for the "slow" half of that complaint, not
        /// <c>JetRoverArmConfig.CommandRateHz</c>: the relay's real per-servo write cooldown is
        /// gated by the *previous move's own commanded duration* (<c>pulseDelta / PulsesPerSecond</c>,
        /// docs/adr/0010), so raising this both moves the arm faster and shortens the cooldown
        /// window for the next correction, instead of racing it. No documented real max speed
        /// spec for these servos exists anywhere in this project -- every increase past 900 has
        /// been genuinely exploratory, gated only on "did it look and sound clean" real-hardware
        /// checks, not a known safety margin. Keep confirming clean before pushing this further;
        /// unlike <c>MaxDirectionMagnitude</c> (a per-call jump-size safety backstop, deliberately
        /// held back from further increases as of 2026-08-22 once it reached 75% of the proximal
        /// joint's full travel in one call), this knob has no such structural ceiling of its own --
        /// its limit is whatever the real servo can mechanically take before straining.
        /// </summary>
        public readonly float PulsesPerSecond;

        /// <summary>
        /// The relay's own fixed step size (50 pulse per unit of "direction") -- needed to convert
        /// a desired pulse delta into the direction value that produces it:
        /// <c>direction = pulseDelta / StepSizePulses</c>.
        /// </summary>
        public readonly float StepSizePulses;

        /// <summary>
        /// Hard clamp on the magnitude of any single direction value applied per call,
        /// independent of the computed delta -- a safety backstop against a bug in the IK/
        /// tracking computation (or a bad <c>CommandFrame.Pose</c> from upstream) commanding an
        /// oversized single step. Shared across every non-gripper joint; the gripper is open-loop
        /// and never participates in this clamp (unchanged from the pre-generalization design).
        ///
        /// Raised 5 -> 10 -> 15 -> 20 (2026-08-22, alongside <see cref="PulsesPerSecond"/>'s
        /// increases) so a large single-frame drag-target jump commits more distance per
        /// <c>Command()</c>/<c>CommandJointAngles()</c> call instead of needing as many correction
        /// cycles to converge. <b>At 20 (1000 pulses/call) this backstop is no longer a real
        /// backstop for this profile</b> -- 1000 pulses spans this hardware's entire 0-1000 raw
        /// travel range, so no physically legal single-call target (even the proximal joint's
        /// full floor-to-ceiling swing) can exceed it anymore; the clamp is structurally a no-op
        /// here, not just "loosened." This was an explicit, informed operator choice made after
        /// being told exactly that (2026-08-22) -- not a default to raise further without the same
        /// conversation, since raising it beyond 20 has no additional effect for this profile (the
        /// per-joint pulse range clamp already bounds every result first) while removing the last
        /// trace of documentation that this was a deliberate tradeoff rather than an oversight.
        /// </summary>
        public readonly float MaxDirectionMagnitude;

        /// <summary>Servo pulse value corresponding to joint angle 0 -- each robot's own "center."</summary>
        public readonly int ZeroPulse;

        /// <summary>Full raw travel range, in pulse units (0-1000 on JetRover's hardware) -- the fallback for any joint with no per-joint override in its <see cref="RobotArmProfile.Joints"/> entry.</summary>
        public readonly int MinPulse;
        public readonly int MaxPulse;

        /// <summary>
        /// Gripper open/closed pulse values -- generalized from the old
        /// <c>GripperOpenDegrees</c>/<c>GripperClosedDegrees</c> (the ROS SDK's own 0-180 degree
        /// space) now that the gripper flows through the same pulse-unit wire as every other
        /// joint (docs/adr/0011): <c>degrees/180*1000</c>, applied once here rather than on the
        /// Python side. Defaults are a plausible half-open range, not yet calibrated against the
        /// real gripper's actual travel (unchanged from before).
        /// </summary>
        public readonly float GripperOpenPulse;
        public readonly float GripperClosedPulse;

        public GenericArmPlantConfig(
            RobotArmProfile profile,
            float pulsePerRadian,
            float pulsesPerSecond,
            float stepSizePulses,
            float maxDirectionMagnitude,
            int zeroPulse,
            int minPulse,
            int maxPulse,
            float gripperOpenPulse,
            float gripperClosedPulse)
        {
            Profile = profile;
            PulsePerRadian = pulsePerRadian;
            PulsesPerSecond = pulsesPerSecond;
            StepSizePulses = stepSizePulses;
            MaxDirectionMagnitude = maxDirectionMagnitude;
            ZeroPulse = zeroPulse;
            MinPulse = minPulse;
            MaxPulse = maxPulse;
            GripperOpenPulse = gripperOpenPulse;
            GripperClosedPulse = gripperClosedPulse;
        }

        /// <summary>
        /// The exact JetRover configuration this codebase has always run, expressed against
        /// <see cref="RobotArmProfile.JetRoverMeasuredDefault"/> instead of hardcoded per-joint
        /// fields -- <see cref="RobotArmProfile.JetRoverMeasuredDefault"/>'s own
        /// <c>MinAngleRadians</c> on the proximal joint already carries the old
        /// <c>LowerArmMinPulse</c> (50) safety floor, so nothing further needs to be re-applied
        /// here. <c>GripperOpenPulse</c>/<c>GripperClosedPulse</c> are the old 30/150-degree
        /// defaults converted once (<c>degrees/180*1000</c>).
        /// </summary>
        public static GenericArmPlantConfig Default => new GenericArmPlantConfig(
            profile: RobotArmProfile.JetRoverMeasuredDefault,
            pulsePerRadian: 1000f / (240f * MathF.PI / 180f),
            pulsesPerSecond: 2200f,
            stepSizePulses: 50f,
            maxDirectionMagnitude: 20f,
            zeroPulse: 500,
            minPulse: 0,
            maxPulse: 1000,
            gripperOpenPulse: 30f / 180f * 1000f,
            gripperClosedPulse: 150f / 180f * 1000f);
    }
}
