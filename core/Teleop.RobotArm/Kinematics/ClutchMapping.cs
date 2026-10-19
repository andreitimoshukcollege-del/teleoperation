using System;
using System.Numerics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Kinematics
{
    /// <summary>
    /// Clutch control (docs/adr/0017-galaxy-xr-clutch-and-gripper.md): while engaged, the arm's
    /// target follows the operator's hand <i>relative to where both were at engage</i>, scaled, and
    /// is kept inside the reachable workspace; while released, it holds. Re-engaging anywhere never
    /// jumps the arm, because the anchor is re-taken from the current hand and target.
    ///
    /// <para>All positions are in the arm's base frame (Core convention: Z up, X forward), the same
    /// frame <see cref="ArmKinematics.TryInverse"/> takes. Pitch is absolute end-effector pitch in
    /// radians, and follows the hand's pitch change 1:1 (it is an angle, so scaling it would make a
    /// level hand produce a tilted gripper). The stored target is always the projected, reachable
    /// one, so pushing past the edge does nothing until the hand comes back.</para>
    ///
    /// <para>A plain struct: no allocations, no clock, no I/O. The host owns when to engage.</para>
    /// </summary>
    public struct ClutchMapping
    {
        private Vector3 _handAtEngage;
        private Vector3 _targetAtEngage;
        private float _handPitchAtEngage;
        private float _targetPitchAtEngage;

        /// <summary>Hand motion → target motion. 0.5 means 10 cm of hand moves the target 5 cm.</summary>
        public float Scale { get; }

        /// <summary>The lowest the target may go, Z in the base frame (m).</summary>
        public float MinHeight { get; }

        public bool IsEngaged { get; private set; }

        /// <summary>The current target: projected and reachable, held while released.</summary>
        public Vector3 Target { get; private set; }

        /// <summary>The current absolute end-effector pitch (radians), held while released.</summary>
        public float Pitch { get; private set; }

        public ClutchMapping(float scale, float minHeight, Vector3 initialTarget, float initialPitch)
        {
            if (!(scale > 0f) || float.IsInfinity(scale))
            {
                throw new ArgumentOutOfRangeException(nameof(scale), scale, "scale must be a positive, finite number");
            }

            Scale = scale;
            MinHeight = minHeight;
            Target = initialTarget;
            Pitch = initialPitch;
            IsEngaged = false;
            _handAtEngage = default;
            _targetAtEngage = default;
            _handPitchAtEngage = 0f;
            _targetPitchAtEngage = 0f;
        }

        /// <summary>
        /// Starts following the hand from where it is now. Projects the held target first, so an
        /// initial target outside the workspace is pulled in before any motion is applied.
        /// </summary>
        public void Engage(in RobotArmProfile profile, Vector3 handPosition, float handPitch)
        {
            Target = ArmKinematics.ProjectToWorkspace(profile, Target, MinHeight);
            _handAtEngage = handPosition;
            _handPitchAtEngage = handPitch;
            _targetAtEngage = Target;
            _targetPitchAtEngage = Pitch;
            IsEngaged = true;
        }

        /// <summary>
        /// Moves the target with the hand. Does nothing while released. A NaN
        /// <paramref name="handPitch"/> leaves the pitch where it was at engage (the host's "don't
        /// follow pitch" option).
        /// </summary>
        public void Update(in RobotArmProfile profile, Vector3 handPosition, float handPitch)
        {
            if (!IsEngaged)
            {
                return;
            }

            Vector3 desired = _targetAtEngage + Scale * (handPosition - _handAtEngage);
            Target = ArmKinematics.ProjectToWorkspace(profile, desired, MinHeight);
            Pitch = float.IsNaN(handPitch)
                ? _targetPitchAtEngage
                : _targetPitchAtEngage + WrapAngle(handPitch - _handPitchAtEngage);
        }

        /// <summary>
        /// Replaces the held target and pitch while released: what the arm actually reached, when the
        /// collision-aware IK stopped short of the last target (docs/adr/0020). The next
        /// <see cref="Engage"/> then starts from there instead of from inside the obstacle. Ignored
        /// while engaged, where the hand decides.
        /// </summary>
        public void Rebase(Vector3 target, float pitch)
        {
            if (IsEngaged)
            {
                return;
            }

            Target = target;
            Pitch = pitch;
        }

        /// <summary>Stops following. The target and pitch hold where they are.</summary>
        public void Release()
        {
            IsEngaged = false;
        }

        private static float WrapAngle(float radians)
        {
            const float twoPi = 2f * MathF.PI;
            radians %= twoPi;
            if (radians > MathF.PI) radians -= twoPi;
            else if (radians < -MathF.PI) radians += twoPi;
            return radians;
        }
    }

    /// <summary>
    /// The off hand's pinch as a gripper command (docs/adr/0017-galaxy-xr-clutch-and-gripper.md):
    /// the thumb–index distance maps linearly from <c>openAt</c> (0, open) to <c>closedAt</c>
    /// (1, closed), clamped. What to do when the hand is not tracked is the host's decision, and
    /// the ADR's is to hold the last value.
    /// </summary>
    public static class GripperMapping
    {
        public static float FromPinchDistance(float distanceMeters, float openAtMeters, float closedAtMeters)
        {
            if (!(openAtMeters > closedAtMeters))
            {
                throw new ArgumentException($"openAtMeters ({openAtMeters}) must be greater than closedAtMeters ({closedAtMeters})");
            }

            // NaN stays NaN (Math.Clamp passes it through): an untracked hand has no distance, and
            // turning that into "open" would drop whatever the gripper holds. The host holds instead.
            float t = (openAtMeters - distanceMeters) / (openAtMeters - closedAtMeters);
            return Math.Clamp(t, 0f, 1f);
        }
    }

    /// <summary>
    /// The off hand's twist as the wrist-roll command (docs/adr/0017): the hand's roll about its
    /// pointing direction, in the arm-base frame (<see cref="ArmKinematics.ExtractRollRadians"/>),
    /// times <c>sign</c> (+1 or -1, matching the servo's mounting), clamped to ±<c>limit</c>. Palm
    /// down is 0, jaws side by side. NaN stays NaN, so the host can hold the last value.
    /// </summary>
    public static class WristRollMapping
    {
        public static float FromHandRoll(float handRollRadians, float sign, float limitRadians)
        {
            if (!(limitRadians >= 0f))
            {
                throw new ArgumentOutOfRangeException(nameof(limitRadians), limitRadians, "limit must be zero or positive");
            }

            float roll = (sign < 0f ? -1f : 1f) * handRollRadians;
            return float.IsNaN(roll) ? float.NaN : Math.Clamp(roll, -limitRadians, limitRadians);
        }

        /// <summary>
        /// The relative form, used while the clutch is engaged: the wrist roll at engage plus how far
        /// the hand has twisted since (wrapped to ±π, times <c>sign</c>), clamped to ±<c>limit</c>.
        /// Relative, so how the hand happens to be held, and how the hand-tracking runtime defines a
        /// palm's "up", do not matter. Only the twist does. NaN stays NaN.
        /// </summary>
        public static float FromHandRollDelta(float handRollRadians, float handRollAtEngageRadians, float rollAtEngageRadians,
            float sign, float limitRadians)
        {
            if (!(limitRadians >= 0f))
            {
                throw new ArgumentOutOfRangeException(nameof(limitRadians), limitRadians, "limit must be zero or positive");
            }

            float delta = handRollRadians - handRollAtEngageRadians;
            if (float.IsNaN(delta) || float.IsNaN(rollAtEngageRadians))
            {
                return float.NaN;
            }

            const float twoPi = 2f * MathF.PI;
            delta %= twoPi;
            if (delta > MathF.PI) delta -= twoPi;
            else if (delta < -MathF.PI) delta += twoPi;
            return Math.Clamp(rollAtEngageRadians + (sign < 0f ? -1f : 1f) * delta, -limitRadians, limitRadians);
        }
    }
}
