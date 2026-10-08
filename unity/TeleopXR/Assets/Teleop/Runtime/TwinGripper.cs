using Teleop.Bridge;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Shows the gripper command on the virtual JetRover: slides the two finger groups inward by
    /// <see cref="JetRoverOperatorBridge.GripperFraction"/>. It shows what is <i>sent</i>; the real
    /// gripper is open-loop (docs/adr/0011, docs/adr/0017), so nothing here knows whether it got there.
    /// Display only. Wired by <c>XrControlSetup</c>.
    /// </summary>
    public sealed class TwinGripper : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge bridge;
        [Tooltip("Left finger first; each slides toward the other along its parent's X axis.")]
        [SerializeField] private Transform leftFinger;
        [SerializeField] private Transform rightFinger;
        [Tooltip("How far each finger moves from open to closed, metres.")]
        [SerializeField] private float travelMeters = 0.0165f;

        private void LateUpdate()
        {
            if (bridge == null)
            {
                return;
            }

            float inward = bridge.GripperFraction * travelMeters;
            if (leftFinger != null) leftFinger.localPosition = new Vector3(inward, 0f, 0f);
            if (rightFinger != null) rightFinger.localPosition = new Vector3(-inward, 0f, 0f);
        }
    }
}
