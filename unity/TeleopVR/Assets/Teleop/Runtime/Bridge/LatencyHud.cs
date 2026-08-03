using TMPro;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// The in-VR latency HUD docs/setup.md's Phase 4 description calls for: a world-space
    /// TextMeshPro label, parented under the XR rig so it renders in-headset (unlike legacy
    /// <c>OnGUI</c>, which does not appear in a VR view), refreshed every <c>Update</c> from
    /// <see cref="TeleopOperatorBridge"/>'s metric sink. Reads, never computes -- every number
    /// shown here already exists in Core/Bridge; this class is display only.
    /// </summary>
    public sealed class LatencyHud : MonoBehaviour
    {
        [SerializeField] private TeleopOperatorBridge operatorBridge;
        [SerializeField] private TMP_Text label;

        private void Update()
        {
            if (label == null || operatorBridge == null)
            {
                return;
            }

            string m2p = FormatLatest("m2p_ms", "M2P");
            string uplink = FormatLatest("owd_uplink_ms", "uplink OWD");
            string downlink = FormatLatest("owd_downlink_ms", "downlink OWD");

            label.text = $"{m2p}\n{uplink}\n{downlink}";
        }

        private string FormatLatest(string metricName, string displayName)
        {
            return operatorBridge.MetricSink.TryGetLatest(metricName, out double value, out _)
                ? $"{displayName}: {value:F1} ms"
                : $"{displayName}: --";
        }
    }
}
