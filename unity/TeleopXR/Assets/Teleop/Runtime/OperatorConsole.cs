using System.Text;
using Teleop.Bridge;
using TMPro;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The operator console on the bench (docs/adr/0023): which joint predictor draws the twin, which
    /// others run beside it, a network disturbance on the joint link, and every running predictor's
    /// live numbers. A thin adapter: the bank, the scoring and the disturbance are Bridge's and Core's
    /// (<see cref="JetRoverJointStateLink"/>, <see cref="JetRoverOperatorBridge.TryApplyDisturbance"/>);
    /// this only maps presses to calls and numbers to text.
    /// <list type="bullet">
    /// <item><b>Predictor row:</b> pressing a predictor's big cap draws the twin with it (turning it
    /// on first if it was off, so it warms up from the next reading). The small cap beside it turns it
    /// on or off; the drawing one cannot be turned off.</item>
    /// <item><b>Network row:</b> one cap per disturbance, applied to the joint socket in both
    /// directions; the first is off.</item>
    /// <item><b>The panel:</b> per running predictor, error p50/p95, correction p95, jitter p95 and the
    /// gripper's error p95 over the last 30 s, refreshed four times a second. Diagnostics, never
    /// results.</item>
    /// </list>
    /// </summary>
    public sealed class OperatorConsole : MonoBehaviour
    {
        public const int MaxPredictors = 6;

        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private TMP_Text panel;

        /// <summary>The default network row: Core <c>NetworkProfileCatalog</c> names ("lan" is off), and what the caps say.</summary>
        public static readonly string[] DefaultNetworkProfiles = { "lan", "50ms-5j", "150ms-20j-0.5loss", "300ms-60j-2loss-bursty", "delay-500ms", "loss-10pct" };
        public static readonly string[] DefaultNetworkLabels = { "NET OFF", "50 ms", "150 ms", "300 ms", "+500 ms", "10% LOSS" };

        [SerializeField] private string[] networkProfiles = (string[])DefaultNetworkProfiles.Clone();
        [SerializeField] private string[] networkLabels = (string[])DefaultNetworkLabels.Clone();

        private static readonly Color Active = new Color(0.2f, 0.9f, 0.4f);    // the predicted arm's green
        private static readonly Color Running = new Color(0.75f, 0.78f, 0.82f);
        private static readonly Color Off = new Color(0.13f, 0.13f, 0.15f);
        private static readonly Color Network = new Color(1f, 0.62f, 0.15f);

        private readonly StringBuilder _text = new StringBuilder(1024);
        private int _network;
        private string _networkError;
        private float _nextRefresh;

        public bool IsVisible(ConsoleButton.Kind kind, int index)
        {
            if (kind == ConsoleButton.Kind.Network) return index < networkProfiles.Length;
            JetRoverJointStateLink link = Link;
            return link != null && index < link.PredictorCount;
        }

        public string LabelFor(ConsoleButton.Kind kind, int index)
        {
            if (kind == ConsoleButton.Kind.Network) return index < networkLabels.Length ? networkLabels[index] : "";
            if (kind == ConsoleButton.Kind.TogglePredictor) return "";
            JetRoverJointStateLink link = Link;
            return link != null && index < link.PredictorCount ? link.PredictorLabel(index) : "";
        }

        public Color ColorFor(ConsoleButton.Kind kind, int index)
        {
            if (kind == ConsoleButton.Kind.Network) return index == _network ? (index == 0 ? Running : Network) : Off;
            JetRoverJointStateLink link = Link;
            if (link == null || index >= link.PredictorCount) return Off;
            if (kind == ConsoleButton.Kind.TogglePredictor) return link.IsPredictorEnabled(index) ? Running : Off;
            if (index == link.ActivePredictorIndex) return Active;
            return link.IsPredictorEnabled(index) ? Running * 0.6f : Off;
        }

        public void OnPressed(ConsoleButton.Kind kind, int index)
        {
            JetRoverJointStateLink link = Link;
            switch (kind)
            {
                case ConsoleButton.Kind.SelectPredictor:
                    if (link == null || index >= link.PredictorCount) return;
                    if (!link.IsPredictorEnabled(index)) link.TrySetPredictorEnabled(index, true);
                    if (link.TrySelectPredictor(index)) Debug.Log($"[console] the twin is drawn by '{link.PredictorLabel(index)}'");
                    break;
                case ConsoleButton.Kind.TogglePredictor:
                    if (link == null || index >= link.PredictorCount) return;
                    link.TrySetPredictorEnabled(index, !link.IsPredictorEnabled(index));
                    break;
                case ConsoleButton.Kind.Network:
                    if (bridge == null || index >= networkProfiles.Length) return;
                    if (bridge.TryApplyDisturbance(networkProfiles[index], out string error))
                    {
                        _network = index;
                        _networkError = null;
                    }
                    else
                    {
                        _networkError = error;
                        Debug.LogWarning($"[console] {networkProfiles[index]}: {error}");
                    }

                    break;
            }

            _nextRefresh = 0f;
        }

        private JetRoverJointStateLink Link => bridge != null ? bridge.JointState : null;

        private void Update()
        {
            if (panel == null || Time.unscaledTime < _nextRefresh)
            {
                return;
            }

            _nextRefresh = Time.unscaledTime + 0.25f;
            JetRoverJointStateLink link = Link;
            _text.Clear();
            if (link == null)
            {
                _text.Append("no joint link");
                panel.SetText(_text);
                return;
            }

            // Monospaced, so the columns line up in TextMeshPro's proportional font.
            _text.Append("<mspace=0.55em><b>PREDICTOR          ERR p50   p95   CORR p95  JIT p95  GRIP p95     n</b>\n");
            for (int i = 0; i < link.PredictorCount; i++)
            {
                if (!link.IsPredictorEnabled(i)) continue;
                bool active = i == link.ActivePredictorIndex;
                _text.Append(active ? "<color=#33E666>> " : "  ");
                Pad(link.PredictorLabel(i), 16);
                if (link.IsPredictorWarmingUp(i))
                {
                    _text.Append("   warming up");
                }
                else if (link.TryGetPredictorStats(i, out JetRoverPredictorStats s) && s.HasData)
                {
                    Number(s.ErrorP50Deg, 7);
                    Number(s.ErrorP95Deg, 6);
                    Number(s.CorrectionP95Deg, 9);
                    Number(s.JitterP95Deg, 9);
                    Number(s.GripperErrorP95ServoDeg, 9);
                    _text.Append(' ', 2).Append(s.Readings);
                }
                else
                {
                    _text.Append("   no fresh readings yet");
                }

                if (active) _text.Append("</color>");
                _text.Append('\n');
            }

            JointLinkDisturbance disturbance = bridge.JointDisturbance;
            _text.Append(disturbance != null && disturbance.IsActive ? "<color=#FF9E26>NETWORK DISTURBED: " : "network: ");
            _text.Append(disturbance != null ? disturbance.Description : "off");
            if (disturbance != null && disturbance.IsActive) _text.Append("  (seed ").Append(disturbance.Seed).Append(")</color>");
            if (_networkError != null) _text.Append("\n<color=#FF5050>").Append(_networkError).Append("</color>");
            _text.Append("\nlast 30 s, degrees (gripper: servo degrees); diagnostics, not results");
            _text.Append("\n<color=#A0A6AE>front row: predictors (cap draws the twin, dot turns it on/off)  ·  back row: network</color>");
            panel.SetText(_text);
        }

        private void Pad(string s, int width)
        {
            if (s.Length > width) s = s.Substring(0, width);
            _text.Append(s);
            _text.Append(' ', width - s.Length);
        }

        /// <summary>Right-aligned, one decimal, '-' for none.</summary>
        private void Number(float value, int width)
        {
            if (float.IsNaN(value))
            {
                _text.Append(' ', width - 1).Append('-');
                return;
            }

            int tenths = Mathf.RoundToInt(value * 10f);
            int digits = tenths >= 1000 ? 5 : tenths >= 100 ? 4 : 3; // "12.3" or "1.2"
            _text.Append(' ', Mathf.Max(0, width - digits));
            _text.Append(tenths / 10).Append('.').Append(tenths % 10);
        }
    }
}
