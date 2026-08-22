using System;
using System.Numerics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Kinematics
{
    /// <summary>
    /// Closed-form forward/inverse kinematics for a <see cref="RobotArmProfile"/>'s position-
    /// affecting chain: an optional rotating base yaw + a 2-link planar arm (proximal/distal
    /// joints) + zero or more wrist-pitch joints that don't move the target point, only
    /// orientation (docs/adr/0011-generic-robot-arm-profiles.md). Replaces the old JetRover-only
    /// <c>FourDofArmKinematics</c> -- same math, parametrized by profile data instead of hardcoded
    /// fields, so any profile sharing this topology (not just JetRover's own numbers) gets the
    /// same closed-form solve.
    ///
    /// <b>Deliberately not arbitrary-N position-solving joints</b> -- see
    /// <see cref="RobotArmProfile"/>'s own doc for why exactly 2 is the well-posed case for
    /// closed-form law-of-cosines IK. Roll and yaw of the commanded end-effector orientation are
    /// still dropped entirely, same as before -- only pitch is honored, via
    /// <see cref="TryInverse"/>'s <c>wristPitchesOut</c>.
    /// </summary>
    public static class ArmKinematics
    {
        /// <summary>
        /// Forward kinematics: given the position-affecting joint angles (radians), returns the
        /// wrist position in the arm's local frame (origin at the base-yaw axis if
        /// <see cref="RobotArmProfile.HasRotatingBase"/>, Z up, X-forward -- Core's ROS
        /// convention). Used only to round-trip-test <see cref="TryInverse"/>;
        /// <c>GenericArmPlant</c> does not call this directly (its own believed-position tracking
        /// comes from sensed joint angles, not from re-deriving a position it already commanded).
        /// </summary>
        public static Vector3 Forward(in RobotArmProfile profile, float baseYaw, float proximalPitch, float distalPitch)
        {
            float l1Angle = proximalPitch;
            float elbowLocalX = profile.ProximalLinkLength * MathF.Cos(l1Angle);
            float elbowLocalZ = profile.BaseHeight + profile.ProximalLinkLength * MathF.Sin(l1Angle);

            float l2Angle = proximalPitch + distalPitch;
            float wristLocalX = elbowLocalX + profile.DistalLinkLength * MathF.Cos(l2Angle);
            float wristLocalZ = elbowLocalZ + profile.DistalLinkLength * MathF.Sin(l2Angle);

            // A profile without a rotating base only ever operates in one fixed vertical plane --
            // forcing yaw to 0 here (rather than trusting the caller never to pass a nonzero one)
            // makes that honest limitation structural, not a documentation-only promise.
            float effectiveYaw = profile.HasRotatingBase ? baseYaw : 0f;
            float x = wristLocalX * MathF.Cos(effectiveYaw);
            float y = wristLocalX * MathF.Sin(effectiveYaw);
            return new Vector3(x, y, wristLocalZ);
        }

        /// <summary>
        /// Inverse kinematics for the position-affecting joints, plus the wrist-pitch closure.
        /// Always succeeds (returns true): a target outside the arm's physical reach is clamped to
        /// the nearest reachable point on the boundary of the working envelope rather than
        /// rejected, same as the model this replaces -- <paramref name="wasClamped"/> reports
        /// whether that happened. "Elbow-up" solution chosen arbitrarily, same as before.
        ///
        /// <paramref name="wristPitchesOut"/> must have length &gt;=
        /// <see cref="RobotArmProfile.WristJointCount"/> (a caller-supplied buffer, typically
        /// <c>stackalloc</c>, keeping this allocation-free). When there is more than one wrist
        /// joint, only <c>wristPitchesOut[0]</c> absorbs the commanded pitch and every later index
        /// is held at exactly 0 -- a documented, deterministic redundancy-resolution choice, the
        /// same spirit as the elbow-up-only choice above.
        /// </summary>
        public static bool TryInverse(
            in RobotArmProfile profile,
            Vector3 targetPosition,
            float desiredAbsolutePitchRadians,
            out float baseYaw,
            out float proximalPitch,
            out float distalPitch,
            Span<float> wristPitchesOut,
            out bool wasClamped)
        {
            if (wristPitchesOut.Length < profile.WristJointCount)
            {
                throw new ArgumentException(
                    $"wristPitchesOut must have length >= profile.WristJointCount ({profile.WristJointCount}), got {wristPitchesOut.Length}.",
                    nameof(wristPitchesOut));
            }

            float r = MathF.Sqrt(targetPosition.X * targetPosition.X + targetPosition.Y * targetPosition.Y);

            // baseYaw is physically meaningless (and Atan2 is numerically ill-conditioned) when the
            // target sits almost directly above the base-yaw axis -- see FourDofArmKinematics's
            // original 2026-08-13 fix, ported unchanged. A profile with no rotating base never
            // computes a nonzero yaw at all, matching Forward's own honesty about that limitation.
            const float minYawStabilizationRadius = 0.01f;
            baseYaw = (profile.HasRotatingBase && r > minYawStabilizationRadius)
                ? MathF.Atan2(targetPosition.Y, targetPosition.X)
                : 0f;

            float dz = targetPosition.Z - profile.BaseHeight;
            float reach = MathF.Sqrt(r * r + dz * dz);

            float maxReach = profile.ProximalLinkLength + profile.DistalLinkLength;
            float minReach = MathF.Abs(profile.ProximalLinkLength - profile.DistalLinkLength);
            // Keep strictly inside the boundary -- exactly at either extreme makes the law-of-
            // cosines denominators well-defined but the solution numerically singular.
            const float epsilon = 1e-4f;
            float clampedReach = Math.Clamp(reach, minReach + epsilon, maxReach - epsilon);
            wasClamped = clampedReach != reach;
            reach = clampedReach;

            float cosGamma = (profile.ProximalLinkLength * profile.ProximalLinkLength +
                    profile.DistalLinkLength * profile.DistalLinkLength - reach * reach)
                / (2f * profile.ProximalLinkLength * profile.DistalLinkLength);
            float gamma = MathF.Acos(Math.Clamp(cosGamma, -1f, 1f));
            distalPitch = MathF.PI - gamma;

            float phi = MathF.Atan2(dz, r);
            float cosAlpha = (profile.ProximalLinkLength * profile.ProximalLinkLength + reach * reach -
                    profile.DistalLinkLength * profile.DistalLinkLength)
                / (2f * profile.ProximalLinkLength * reach);
            float alpha = MathF.Acos(Math.Clamp(cosAlpha, -1f, 1f));
            proximalPitch = phi - alpha;

            if (profile.WristJointCount > 0)
            {
                wristPitchesOut[0] = desiredAbsolutePitchRadians - (proximalPitch + distalPitch);
                for (int i = 1; i < profile.WristJointCount; i++)
                {
                    wristPitchesOut[i] = 0f;
                }
            }

            return true;
        }

        /// <summary>
        /// Extracts "pitch" from a commanded orientation the same gimbal-lock-free way regardless
        /// of roll or yaw -- unchanged from <c>FourDofArmKinematics</c>, this math has no per-robot
        /// dependency at all.
        /// </summary>
        public static float ExtractPitchRadians(Quaternion rotation)
        {
            Vector3 forward = Vector3.Transform(Vector3.UnitX, rotation);
            float horizontal = MathF.Sqrt(forward.X * forward.X + forward.Y * forward.Y);
            return MathF.Atan2(forward.Z, horizontal);
        }

        /// <summary>
        /// Turns <see cref="TryInverse"/>'s output into one <see cref="JointTarget"/> per joint in
        /// <paramref name="profile"/>, in <see cref="RobotArmProfile.Joints"/>'s own order -- the
        /// step that makes IK output wire-ready for either hop (a caller converts angle/fraction to
        /// this joint's actual wire unit afterward; see <see cref="JointTarget"/>'s own doc).
        ///
        /// <b><see cref="JointTarget.Angle"/>'s meaning here is role-dependent, not uniformly
        /// radians</b>: <see cref="JointRole.BaseYaw"/>/<see cref="JointRole.Proximal"/>/
        /// <see cref="JointRole.Distal"/>/<see cref="JointRole.Wrist"/> are radians;
        /// <see cref="JointRole.GripperMain"/> is <paramref name="gripperFraction"/> (0=open,
        /// 1=closed) -- the same two-unit-space split <c>JetRoverPlant.ApplyJointTargets</c> always
        /// had (gripper was never in the same angle space as the arm joints), just keyed by role
        /// now instead of by which named field it was.
        /// <see cref="JointRole.GripperRotate"/> is always 0 -- no operator input yet computes an
        /// independent gripper-rotation command (no <c>CommandFrame</c>/<c>JointCommandFrame</c>
        /// field exists for it); the profile can describe a rotating gripper structurally, but
        /// driving one end to end is a follow-up, not part of this generalization.
        /// <see cref="JointTarget.Speed"/> is always 0 here -- filled in by whichever codec/plant
        /// step actually knows the configured speed for this hop.
        /// </summary>
        public static int MapAnglesToJointTargets(
            in RobotArmProfile profile,
            float baseYaw, float proximalPitch, float distalPitch,
            ReadOnlySpan<float> wristPitches, float gripperFraction,
            Span<JointTarget> output)
        {
            if (output.Length < profile.JointCount)
            {
                throw new ArgumentException(
                    $"output must have length >= profile.JointCount ({profile.JointCount}), got {output.Length}.",
                    nameof(output));
            }

            int count = 0;
            foreach (JointHardwareSpec joint in profile.Joints)
            {
                float angle = joint.Role switch
                {
                    JointRole.BaseYaw => baseYaw,
                    JointRole.Proximal => proximalPitch,
                    JointRole.Distal => distalPitch,
                    JointRole.Wrist => wristPitches[joint.WristIndex],
                    JointRole.GripperMain => gripperFraction,
                    JointRole.GripperRotate => 0f,
                    _ => 0f,
                };
                output[count++] = new JointTarget(joint.MotorId, angle, speed: 0f);
            }

            return count;
        }
    }
}
