using Teleop.Bridge;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Shows the gripper command on the virtual JetRover: slides the two finger groups inward by
    /// <see cref="JetRoverOperatorBridge.GripperFraction"/>, and turns the gripper by the wrist roll
    /// (<see cref="JetRoverOperatorBridge.WristRollRadians"/>, motor 5). It shows what is <i>sent</i>; the real
    /// gripper is open-loop (docs/adr/0011, docs/adr/0017), so nothing here knows whether it got there.
    /// Display only. Wired by <c>XrControlSetup</c>. With no bridge assigned it shows whatever
    /// <see cref="Show"/> was last given: the predicted and measured arms are driven that way
    /// (docs/adr/0018).
    /// </summary>
    public sealed class TwinGripper : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge bridge;
        [Tooltip("Left finger first; each slides toward the other along its parent's X axis.")]
        [SerializeField] private Transform leftFinger;
        [SerializeField] private Transform rightFinger;
        [Tooltip("How far each finger moves from open to closed, metres.")]
        [SerializeField] private float travelMeters = 0.0165f;
        [Tooltip("The group past the roll servo; it turns about its local Z (the link's axis).")]
        [SerializeField] private Transform rollPivot;
        [Header("Hinged jaws (the CAD gripper's four-bar linkage), used instead of the sliding fingers")]
        [Tooltip("Hinged links (their origins are the hinges): each inner one turns about its local X by up to Jaw Closed Degrees, and its outer one turns back by the same, which keeps the finger pad parallel.")]
        [SerializeField] private Transform leftJaw;
        [SerializeField] private Transform leftJawOuter;
        [SerializeField] private Transform rightJaw;
        [SerializeField] private Transform rightJawOuter;
        [Tooltip("How far each inner link turns from open to closed. The CAD jaws meet at about 37°.")]
        [SerializeField] private float jawClosedDegrees = 37f;
        [Tooltip("+1 or -1: which way about the links' X axis closes.")]
        [SerializeField] private float jawClosingSign = -1f;

        private float _gripper, _roll;

        /// <summary>
        /// Shows or hides everything past the roll servo. The measured arm hides it when the robot
        /// does not measure the roll, rather than drawing a commanded value as if it were measured.
        /// </summary>
        public void SetGripperVisible(bool visible)
        {
            if (rollPivot != null && rollPivot.gameObject.activeSelf != visible)
            {
                rollPivot.gameObject.SetActive(visible);
            }
        }

        /// <summary>Sets what an arm with no bridge shows: the gripper fraction and the wrist roll (radians).</summary>
        public void Show(float gripperFraction, float wristRollRadians)
        {
            _gripper = Mathf.Clamp01(gripperFraction);
            _roll = wristRollRadians;
            Apply();
        }

        private void LateUpdate()
        {
            if (bridge != null)
            {
                _gripper = bridge.GripperFraction;
                _roll = bridge.WristRollRadians;
                Apply();
            }
        }

        private Quaternion _leftJaw0, _leftOuter0, _rightJaw0, _rightOuter0;
        private bool _jawsCaptured;

        private void Apply()
        {
            if (leftJaw != null || rightJaw != null)
            {
                // The jaws as built are open; their rest rotations are taken the first time.
                if (!_jawsCaptured)
                {
                    _jawsCaptured = true;
                    if (leftJaw != null) _leftJaw0 = leftJaw.localRotation;
                    if (leftJawOuter != null) _leftOuter0 = leftJawOuter.localRotation;
                    if (rightJaw != null) _rightJaw0 = rightJaw.localRotation;
                    if (rightJawOuter != null) _rightOuter0 = rightJawOuter.localRotation;
                }

                float angle = _gripper * jawClosedDegrees;
                if (leftJaw != null) leftJaw.localRotation = _leftJaw0 * Quaternion.AngleAxis(-angle * jawClosingSign, Vector3.right);
                if (leftJawOuter != null) leftJawOuter.localRotation = _leftOuter0 * Quaternion.AngleAxis(angle * jawClosingSign, Vector3.right);
                if (rightJaw != null) rightJaw.localRotation = _rightJaw0 * Quaternion.AngleAxis(angle * jawClosingSign, Vector3.right);
                if (rightJawOuter != null) rightJawOuter.localRotation = _rightOuter0 * Quaternion.AngleAxis(-angle * jawClosingSign, Vector3.right);
            }

            float inward = _gripper * travelMeters;
            if (leftFinger != null) leftFinger.localPosition = new Vector3(inward, 0f, 0f);
            if (rightFinger != null) rightFinger.localPosition = new Vector3(-inward, 0f, 0f);
            // Core roll lifts the gripper's left side: a negative turn about Unity's forward axis, the
            // same convention JetRoverClutch writes the target with.
            if (rollPivot != null) rollPivot.localRotation = Quaternion.AngleAxis(-_roll * Mathf.Rad2Deg, Vector3.forward);
        }
    }
}
