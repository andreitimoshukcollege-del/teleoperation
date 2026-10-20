using System;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// Clutch control for the JetRover arm (docs/adr/0017-galaxy-xr-clutch-and-gripper.md), as a
    /// thin Unity adapter over Core's <see cref="ClutchMapping"/>. An input component calls
    /// <see cref="Engage"/>, <see cref="Follow"/> and <see cref="Release"/> with a hand pose in world
    /// space; this converts it into the arm-base frame, lets <see cref="ClutchMapping"/> decide the
    /// target, and writes that target to the bridge's drag target Transform. The bridge then does
    /// what it always did with that Transform: IK, the rig, the command stream.
    ///
    /// <para>It lives in Bridge, not in the XR project, because it is where Core types meet Unity
    /// types (unity/CLAUDE.md: the XR assemblies reference Bridge only). Plain C#, no
    /// MonoBehaviour, no allocations after construction.</para>
    /// </summary>
    public sealed class JetRoverClutch
    {
        private readonly Transform _armBase;
        private readonly Transform _target;
        private readonly RobotArmProfile _profile;
        private ClutchMapping _mapping;

        public JetRoverClutch(Transform armBase, Transform target, in RobotArmProfile profile, float scale, float minHeightMeters)
        {
            _armBase = armBase ?? throw new ArgumentNullException(nameof(armBase));
            _target = target ?? throw new ArgumentNullException(nameof(target));
            _profile = profile;

            // Start from wherever the target is, pulled into the workspace, so the arm and the
            // target agree before anyone touches anything.
            ToBase(_target.position, _target.rotation, out var position, out float pitch);
            _mapping = new ClutchMapping(scale, minHeightMeters,
                ArmKinematics.ProjectToWorkspace(_profile, position, minHeightMeters), pitch);
            WriteTarget();
        }

        public bool IsEngaged => _mapping.IsEngaged;
        public float Scale => _mapping.Scale;

        /// <summary>
        /// Starts following the hand from its current pose. The target first takes up wherever the
        /// drag target is now: the bridge moves it to what the arm reached when the IK stopped short
        /// (docs/adr/0020), and the clutch must start from there, not from inside the obstacle.
        /// </summary>
        public void Engage(Vector3 handPosition, Quaternion handRotation)
        {
            // Read near the clutch's own pitch and roll: past straight down, the plain reading is the
            // same orientation with the roll half a turn round (ArmKinematics.ExtractPitchRollNear).
            var held = _armBase.InverseTransformPoint(_target.position).ToCore();
            ArmKinematics.ExtractPitchRollNear((Quaternion.Inverse(_armBase.rotation) * _target.rotation).ToCore(),
                _mapping.Pitch, Roll, out float heldPitch, out float heldRoll);
            _mapping.Rebase(held, heldPitch);
            Roll = heldRoll;
            ToBase(handPosition, handRotation, out var position, out float pitch);
            _mapping.Engage(_profile, position, pitch);
            WriteTarget();
        }

        /// <summary>
        /// Moves the target with the hand, while engaged. <paramref name="followPitch"/> false keeps
        /// the wrist's pitch where it was at engage.
        /// </summary>
        public void Follow(Vector3 handPosition, Quaternion handRotation, bool followPitch = true)
        {
            if (!_mapping.IsEngaged)
            {
                return;
            }

            ToBase(handPosition, handRotation, out var position, out float pitch);
            _mapping.Update(_profile, position, followPitch ? pitch : float.NaN);
            WriteTarget();
        }

        public void Release() => _mapping.Release();

        /// <summary>The wrist roll in the target (radians, Core convention), held until set again.</summary>
        public float Roll { get; private set; }

        // The gripper hand's rotation (arm base frame, Core convention) when the roll was anchored.
        private System.Numerics.Quaternion _handAtAnchor;
        private bool _hasRollAnchor;
        private float _rollAtAnchor;

        /// <summary>
        /// Starts relative wrist roll from the gripper hand's current rotation: from now on
        /// <see cref="FollowRoll"/> turns the wrist by how far that hand twists. Call it when the
        /// clutch engages, or on the first tracked frame after.
        /// </summary>
        public void AnchorRoll(Quaternion handRotation)
        {
            _handAtAnchor = HandInBase(handRotation);
            _hasRollAnchor = true;
            _rollAtAnchor = Roll;
        }

        public bool HasRollAnchor => _hasRollAnchor;

        /// <summary>Forgets the roll anchor (the clutch released); the roll holds.</summary>
        public void ReleaseRoll() => _hasRollAnchor = false;

        /// <summary>
        /// The wrist roll follows the gripper hand's twist since <see cref="AnchorRoll"/> (docs/adr/0017
        /// amendment 2026-10-12): relative, so the hand-tracking runtime's idea of a palm's "up" does
        /// not matter, only the twist does.
        /// </summary>
        public void FollowRoll(Quaternion handRotation, float sign, float limitDegrees)
        {
            if (!HasRollAnchor)
            {
                return;
            }

            // The twist about the hand's own pointing axis since the anchor: no trouble when the hand
            // points straight down, unlike a difference of two absolute roll readings.
            float twist = WristRollMapping.TwistRadians(_handAtAnchor, HandInBase(handRotation));
            float roll = WristRollMapping.FromHandRollDelta(twist, 0f, _rollAtAnchor,
                sign, limitDegrees * (MathF.PI / 180f));
            if (!float.IsNaN(roll))
            {
                Roll = roll;
                WriteTarget();
            }
        }

        private System.Numerics.Quaternion HandInBase(Quaternion handRotation) =>
            (Quaternion.Inverse(_armBase.rotation) * handRotation).ToCore();

        /// <summary>
        /// A thumb-to-index distance (m) as a gripper command, 0 open to 1 closed, by the config's
        /// <c>GripperPinchOpenMeters</c> and <c>GripperPinchClosedMeters</c>. NaN stays NaN, which
        /// <see cref="JetRoverOperatorBridge.GripperFraction"/> ignores, so an untracked hand holds.
        /// </summary>
        public static float GripperFromPinchDistance(float distanceMeters, JetRoverArmConfig config) =>
            GripperMapping.FromPinchDistance(distanceMeters, config.GripperPinchOpenMeters, config.GripperPinchClosedMeters);

        private void ToBase(Vector3 worldPosition, Quaternion worldRotation, out System.Numerics.Vector3 position, out float pitch)
        {
            position = _armBase.InverseTransformPoint(worldPosition).ToCore();
            pitch = ArmKinematics.ExtractPitchRadians((Quaternion.Inverse(_armBase.rotation) * worldRotation).ToCore());
        }

        private void WriteTarget()
        {
            _target.position = _armBase.TransformPoint(_mapping.Target.ToUnity());
            // Pitch, then roll; yaw is the base's job. Core pitch tilts +X (Unity +Z, forward) toward
            // +Z (Unity +Y, up): a negative turn about Unity's +X. Core roll lifts +Y (Unity's left,
            // -X) toward up: a negative turn about Unity's +Z. ArmKinematics.ExtractPitchRadians and
            // ExtractRollRadians give both back, so the bridge and the robot read what was written.
            _target.rotation = _armBase.rotation
                * Quaternion.AngleAxis(-_mapping.Pitch * Mathf.Rad2Deg, Vector3.right)
                * Quaternion.AngleAxis(-Roll * Mathf.Rad2Deg, Vector3.forward);
        }
    }
}
