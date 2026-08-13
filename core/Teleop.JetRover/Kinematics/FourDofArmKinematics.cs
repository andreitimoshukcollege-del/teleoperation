using System;
using System.Numerics;

namespace Teleop.JetRover.Kinematics
{
    /// <summary>
    /// Closed-form forward/inverse kinematics for the JetRover arm's position-affecting chain:
    /// base yaw + a 2-link planar arm (lower/middle joints), plus a wrist-pitch joint (upper)
    /// that does not move the target point, only orientation. This is a position-and-pitch-only
    /// model of a physically richer arm -- see the class-level caveats below for exactly what it
    /// drops and why, before trusting it for anything beyond what
    /// docs/adr/0007-jetrover-plant-and-robot-host.md's Phase 2 needs.
    ///
    /// Lives in <c>Teleop.JetRover</c>, a shared package compiled by both <c>Teleop.RobotHost</c>
    /// and Unity (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) -- moved here from
    /// <c>Teleop.RobotHost</c> once the operator side (Unity) needed to run this same computation
    /// for real, not just approximate it for visualization. Deliberately not part of
    /// <c>Teleop.Core</c>: this is one specific robot's ruler-measured hardware geometry, not a
    /// general research technique (<c>core/Teleop.Core/Plant/CLAUDE.md</c>).
    ///
    /// <b>Link lengths are ruler-measured, not from a datasheet or URDF</b> -- no such
    /// specification exists for this JetRover variant (checked the Jetson filesystem and the
    /// full `SINRG-Lab/industryxr-robot` GitHub repo; Hiwonder's own published docs give a
    /// wingspan and per-servo angle range but no inter-joint link lengths). Expect a few
    /// millimeters to a centimeter of real-world error until an actual calibration pass happens.
    ///
    /// <b>Target point is the wrist (end of the "upper" link), not the gripper fingertip.</b> No
    /// measurement exists from the upper-arm joint to wherever the gripper actually grips --
    /// asking for it was judged not worth another round of back-and-forth for a Phase 2 whose own
    /// scope is "prove Cartesian control works," not "reach a specific point to the millimeter."
    /// A future calibration pass should add that offset here rather than trying to fold it into
    /// the existing link lengths.
    ///
    /// <b>Roll and yaw of the commanded end-effector orientation are dropped entirely</b> -- only
    /// pitch (elevation of the commanded forward direction) is honored, via <see cref="InverseUpperPitch"/>.
    /// A 4-DOF chain (base yaw, 2 planar joints, 1 wrist pitch) is exactly determined for
    /// position + one orientation angle; it cannot also represent roll or a yaw independent of
    /// where the base already points. This mirrors how <c>RawPoseCodec</c>'s own doc is explicit
    /// about what a wire format does and doesn't preserve -- same discipline, applied to a
    /// physical linkage instead of a codec.
    /// </summary>
    public static class FourDofArmKinematics
    {
        /// <summary>
        /// Forward kinematics: given the three position-affecting joint angles (radians), returns
        /// the wrist position in the arm's local frame (origin at the base-yaw axis, Z up,
        /// X-forward when <paramref name="baseYaw"/> is zero -- Core's ROS convention). Used only
        /// to round-trip-test <see cref="TryInverse"/>; <c>JetRoverPlant</c> does not call this
        /// directly (its own believed-position tracking comes from sensed joint angles, not
        /// from re-deriving a position it already commanded).
        /// </summary>
        public static Vector3 Forward(ArmLinkLengths links, float baseYaw, float lowerPitch, float middlePitch)
        {
            float l1Angle = lowerPitch;
            float elbowLocalX = links.Lower * MathF.Cos(l1Angle);
            float elbowLocalZ = links.Base + links.Lower * MathF.Sin(l1Angle);

            float l2Angle = lowerPitch + middlePitch;
            float wristLocalX = elbowLocalX + links.Middle * MathF.Cos(l2Angle);
            float wristLocalZ = elbowLocalZ + links.Middle * MathF.Sin(l2Angle);

            // wristLocalX is the horizontal reach before yaw is applied; rotate it into the
            // X/Y plane by baseYaw (Z-up, right-handed).
            float x = wristLocalX * MathF.Cos(baseYaw);
            float y = wristLocalX * MathF.Sin(baseYaw);
            return new Vector3(x, y, wristLocalZ);
        }

        /// <summary>
        /// Inverse kinematics for the position-affecting joints. Always succeeds (returns true):
        /// a target outside the arm's physical reach is clamped to the nearest reachable point
        /// on the boundary of the working envelope (documented behavior, not silently wrong
        /// behavior) rather than rejected, since <c>IRobotPlant.Command</c> has no failure channel
        /// to report an unreachable target through. <paramref name="wasClamped"/> reports whether
        /// that happened, so a caller close enough to the wire to matter (e.g. a VR operator
        /// dragging past reach) can show it -- added when this became operator-side, authoritative
        /// computation rather than a robot-side-only detail; see this file's own class doc and
        /// docs/adr/0009-jetrover-operator-side-inverse-kinematics.md.
        ///
        /// "Elbow-up" solution chosen arbitrarily (the other real solution, "elbow-down", is
        /// never produced) -- there is no basis yet to prefer one over the other without an
        /// actual calibration pass against the real arm's joint limits and any self-collision
        /// geometry, and picking consistently is what makes the chosen one at least testable.
        /// </summary>
        public static bool TryInverse(
            ArmLinkLengths links,
            Vector3 targetPosition,
            out float baseYaw,
            out float lowerPitch,
            out float middlePitch,
            out bool wasClamped)
        {
            float r = MathF.Sqrt(targetPosition.X * targetPosition.X + targetPosition.Y * targetPosition.Y);

            // baseYaw is physically meaningless (and Atan2 is numerically ill-conditioned) when
            // the target sits almost directly above the base-yaw axis -- a real bug found via the
            // JetRover VR drag feature (2026-08-13): a sub-millimeter jitter in X or Y this close
            // to the axis, well within real controller tracking noise, can swing the raw Atan2
            // result by up to 180 degrees despite the target barely moving, which showed up as
            // the Unity arm rig visibly flashing between two poses while dragging a target that
            // looked perfectly still. Below this radius, any yaw reaches the same point, so pin
            // it to a fixed value instead of computing an unstable one. The threshold is
            // comfortably above real tracking jitter (millimeters) and small relative to this
            // arm's ~0.26m max reach.
            const float minYawStabilizationRadius = 0.01f;
            baseYaw = r > minYawStabilizationRadius ? MathF.Atan2(targetPosition.Y, targetPosition.X) : 0f;

            float dz = targetPosition.Z - links.Base;
            float reach = MathF.Sqrt(r * r + dz * dz);

            float maxReach = links.Lower + links.Middle;
            float minReach = MathF.Abs(links.Lower - links.Middle);
            // Keep strictly inside the boundary -- exactly at either extreme makes the law-of-
            // cosines denominators well-defined but the solution numerically singular (an
            // infinitesimal input change would flip the elbow-up/down branch).
            const float epsilon = 1e-4f;
            float clampedReach = Math.Clamp(reach, minReach + epsilon, maxReach - epsilon);
            wasClamped = clampedReach != reach;
            reach = clampedReach;

            float cosGamma = (links.Lower * links.Lower + links.Middle * links.Middle - reach * reach)
                / (2f * links.Lower * links.Middle);
            float gamma = MathF.Acos(Math.Clamp(cosGamma, -1f, 1f));
            middlePitch = MathF.PI - gamma;

            float phi = MathF.Atan2(dz, r);
            float cosAlpha = (links.Lower * links.Lower + reach * reach - links.Middle * links.Middle)
                / (2f * links.Lower * reach);
            float alpha = MathF.Acos(Math.Clamp(cosAlpha, -1f, 1f));
            lowerPitch = phi - alpha;

            return true;
        }

        /// <summary>
        /// The wrist-pitch (upper) joint's angle: whatever is needed so the cumulative pitch of
        /// the chain (<paramref name="lowerPitch"/> + <paramref name="middlePitch"/> +
        /// upperPitch) equals <paramref name="desiredAbsolutePitchRadians"/> -- "upper" is the
        /// joint that closes the chain to hit the commanded orientation's pitch, per the class
        /// doc's caveat that roll/yaw are dropped entirely.
        /// </summary>
        public static float InverseUpperPitch(float lowerPitch, float middlePitch, float desiredAbsolutePitchRadians)
        {
            return desiredAbsolutePitchRadians - (lowerPitch + middlePitch);
        }

        /// <summary>
        /// Extracts "pitch" from a commanded orientation the same gimbal-lock-free way
        /// regardless of roll or yaw: transforms the local +X (forward) axis by
        /// <paramref name="rotation"/> and returns that direction's elevation angle above the
        /// X/Y plane. Deliberately not a standard Euler decomposition (order-dependent and prone
        /// to gimbal lock) -- this chain only ever wants "how far up or down is the commanded
        /// forward direction pointing," not a full 3-angle breakdown.
        /// </summary>
        public static float ExtractPitchRadians(Quaternion rotation)
        {
            Vector3 forward = Vector3.Transform(Vector3.UnitX, rotation);
            float horizontal = MathF.Sqrt(forward.X * forward.X + forward.Y * forward.Y);
            return MathF.Atan2(forward.Z, horizontal);
        }
    }

    /// <summary>
    /// Ruler-measured link lengths for <see cref="FourDofArmKinematics"/>, in metres (Core's
    /// convention). See that class's doc for how approximate these are and what they don't
    /// account for.
    /// </summary>
    public readonly struct ArmLinkLengths
    {
        /// <summary>Height of the shoulder (lower-arm joint) pivot above the base-yaw axis.</summary>
        public readonly float Base;

        /// <summary>Length of the link between the lower-arm and middle-arm joints ("bicep").</summary>
        public readonly float Lower;

        /// <summary>Length of the link between the middle-arm and upper-arm joints ("forearm").</summary>
        public readonly float Middle;

        public ArmLinkLengths(float baseHeight, float lower, float middle)
        {
            Base = baseHeight;
            Lower = lower;
            Middle = middle;
        }

        /// <summary>
        /// Ruler-measured (not calipers) against the physical robot: base pivot ~3.5cm off the
        /// mount, lower-to-middle and middle-to-upper links ~13cm each. See the class doc on
        /// <see cref="FourDofArmKinematics"/> for why no more precise source exists.
        /// </summary>
        public static ArmLinkLengths Measured => new ArmLinkLengths(baseHeight: 0.035f, lower: 0.13f, middle: 0.13f);
    }
}
