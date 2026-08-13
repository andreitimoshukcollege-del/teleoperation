using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// Applies already-computed JetRover joint angles to a 4-segment visual rig
    /// (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md). Deliberately dumb: this class
    /// has no Unity lifecycle methods of its own and no Core dependency -- it is called from
    /// <see cref="JetRoverOperatorBridge"/> with the exact angles that were just computed
    /// host-side and are about to be sent to the real robot, not re-derived from a round-tripped
    /// pose estimate. That means this rig always shows what was actually commanded, with none of
    /// the staleness or elbow-up/down ambiguity a re-derivation would risk.
    ///
    /// <b>The real middle-arm servo (ID 3) is not confirmed dead -- do not describe it that way.</b>
    /// The actual finding (robot/README.md, <see cref="Teleop.RobotHost.Plant.JetRoverPlant"/>'s own
    /// doc) is narrower: writes to it work fine (it visibly moves, and a human can feel it holding
    /// torque), but position-*read* requests to it never succeed, confirmed independently of ROS by
    /// calling the board SDK directly. That means this rig's displayed angle for that joint may be
    /// ahead of or behind the real arm's actual position (no sensed feedback ever corrects it), but
    /// the real joint can very plausibly still be moving underneath. Do not "fix" this by hiding or
    /// freezing the middle segment -- that would hide a real, still only partially diagnosed,
    /// hardware fact this rig should keep surfacing, not paper over.
    ///
    /// <b>Axis/sign mapping below is empirically calibrated against the visual rig in the Unity
    /// Editor (2026-08-13), not yet against live hardware.</b> Core's convention (right-handed,
    /// Z-up, X-forward) and Unity's (left-handed, Y-up, Z-forward) relate through
    /// <see cref="CoordConversion"/>'s basis vectors, but this class applies angles directly to
    /// pivot Transforms rather than converting a Pose, so it needed its own reasoning about which
    /// Unity axis each Core rotation corresponds to -- the pitch signs' original first-pass guess
    /// (+1) was confirmed backwards by dragging the target in Play mode with no robot connected
    /// (enabled by fixing <see cref="JetRoverOperatorBridge"/> to drive this rig from the raw drag
    /// target instead of a round-tripped robot-state estimate): the arm drooped away from a target
    /// placed above it instead of reaching up toward it, the signature of Core's "positive pitch
    /// tilts up" being applied as "tilt down" in Unity's opposite rotation sense about the same
    /// physical axis. The four `*Sign` fields still exist so a human can flip a remaining backwards
    /// axis from the Inspector, no code change needed -- this pass got close, not exact (per
    /// docs/adr/0009's own verification steps, an actual live-hardware pass is still needed for a
    /// final check, but that no longer blocks Unity-side kinematics/rig work at all).
    /// </summary>
    public sealed class JetRoverArmRig : MonoBehaviour
    {
        [Header("Pivots -- see docs/adr/0009-jetrover-operator-side-inverse-kinematics.md's scene assembly steps")]
        [SerializeField] private Transform baseYawPivot;
        [SerializeField] private Transform lowerPitchPivot;
        [SerializeField] private Transform middlePitchPivot;
        [SerializeField] private Transform upperPitchPivot;

        [Header("Reach-clamp visibility -- recolored when the last commanded target was outside the arm's reach")]
        [SerializeField] private Renderer reachWarningRenderer;
        [SerializeField] private Color normalColor = Color.white;
        [SerializeField] private Color clampedColor = Color.red;

        [Header("Axis sign flips -- flip via Inspector after empirically checking against the real arm, not by editing code")]
        [SerializeField] private float baseYawSign = -1f; // Core's right-handed +Z yaw vs. Unity's left-handed +Y rotation
        [SerializeField] private float lowerPitchSign = -1f; // empirically confirmed 2026-08-13 -- +1 made the arm droop away from a target instead of reaching toward it
        [SerializeField] private float middlePitchSign = -1f;
        [SerializeField] private float upperPitchSign = -1f;

        private MaterialPropertyBlock _propertyBlock;

        /// <summary>
        /// Applies one set of joint angles (radians, Core convention) to the rig's pivots and
        /// updates the reach-clamp warning color. Called from <see cref="JetRoverOperatorBridge"/>
        /// right after it computes these same angles for the command it's about to send.
        /// </summary>
        public void ApplyAngles(float baseYaw, float lowerPitch, float middlePitch, float upperPitch, bool wasClamped)
        {
            if (baseYawPivot != null)
            {
                baseYawPivot.localRotation = Quaternion.AngleAxis(baseYaw * baseYawSign * Mathf.Rad2Deg, Vector3.up);
            }

            if (lowerPitchPivot != null)
            {
                lowerPitchPivot.localRotation = Quaternion.AngleAxis(lowerPitch * lowerPitchSign * Mathf.Rad2Deg, Vector3.right);
            }

            if (middlePitchPivot != null)
            {
                middlePitchPivot.localRotation = Quaternion.AngleAxis(middlePitch * middlePitchSign * Mathf.Rad2Deg, Vector3.right);
            }

            if (upperPitchPivot != null)
            {
                upperPitchPivot.localRotation = Quaternion.AngleAxis(upperPitch * upperPitchSign * Mathf.Rad2Deg, Vector3.right);
            }

            if (reachWarningRenderer != null)
            {
                _propertyBlock ??= new MaterialPropertyBlock();
                reachWarningRenderer.GetPropertyBlock(_propertyBlock);
                _propertyBlock.SetColor("_Color", wasClamped ? clampedColor : normalColor);
                reachWarningRenderer.SetPropertyBlock(_propertyBlock);
            }
        }
    }
}
