using TMPro;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// A world-space TextMeshPro label showing whether <see cref="JetRoverOperatorBridge"/> has
    /// ever heard back from the real robot, and if so, whether that contact is still fresh --
    /// same shape as <see cref="LatencyHud"/> (reads, never computes; refreshed every
    /// <c>Update</c>). Exists because <see cref="JetRoverOperatorBridge.HasReceivedAnyState"/> was
    /// previously computed but never displayed anywhere -- the only way to tell whether Unity had
    /// actually reached <c>Teleop.RobotHost</c> was watching the arm move or reading Jetson-side
    /// logs over SSH.
    /// </summary>
    public sealed class JetRoverConnectionHud : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge operatorBridge;
        [SerializeField] private TMP_Text label;

        private void Update()
        {
            if (label == null || operatorBridge == null)
            {
                return;
            }

            label.text = operatorBridge.Status switch
            {
                JetRoverOperatorBridge.ConnectionStatus.Connected => "Robot: connected",
                JetRoverOperatorBridge.ConnectionStatus.Stale => "Robot: connection lost",
                _ => "Robot: no connection yet",
            };
        }
    }
}
