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
    /// (<see cref="OperatorBridgeBase"/>, either robot's: the JetRover's or the FR3 Duo's, docs/adr/0029
    /// amendment); this only maps presses to calls and numbers to text.
    /// <list type="bullet">
    /// <item><b>Predictor row:</b> pressing a predictor's big cap draws the twin with it (turning it
    /// on first if it was off, so it warms up from the next reading). The small cap beside it turns it
    /// on or off; the drawing one cannot be turned off.</item>
    /// <item><b>Network sliders:</b> lag, jitter and loss, applied together to the joint socket in
    /// both directions once they have stopped moving for a moment; all at zero is off.</item>
    /// <item><b>The panel:</b> per running predictor, error p50/p95, correction p95, jitter p95 and the
    /// gripper's error p95 over the last 30 s, refreshed four times a second. Diagnostics, never
    /// results.</item>
    /// </list>
    /// </summary>
    public sealed class OperatorConsole : MonoBehaviour
    {
        public const int MaxPredictors = 6;

        [SerializeField] private OperatorBridgeBase bridge;
        [SerializeField] private TMP_Text panel;

        private static readonly Color Active = new Color(0.2f, 0.9f, 0.4f);    // the predicted arm's green
        private static readonly Color Running = new Color(0.75f, 0.78f, 0.82f);
        private static readonly Color Off = new Color(0.13f, 0.13f, 0.15f);

        private readonly StringBuilder _text = new StringBuilder(2048);
        private float _nextRefresh;

        // The network sliders' values, applied together once they have settled (docs/adr/0023 decision 3).
        private float _lagMs, _jitterMs, _lossPercent;
        private float _appliedLag = -1f, _appliedJitter = -1f, _appliedLoss = -1f;
        private float _changedAt = -1f;
        private const float SettleSeconds = 0.4f;

        /// <summary>A network slider moved: remembered now, applied once the sliders have settled.</summary>
        public void SetNetwork(SpeedSlider.Setting setting, float value)
        {
            float before = setting == SpeedSlider.Setting.NetworkLag ? _lagMs : setting == SpeedSlider.Setting.NetworkJitter ? _jitterMs : _lossPercent;
            if (before == value) return;
            if (setting == SpeedSlider.Setting.NetworkLag) _lagMs = value;
            else if (setting == SpeedSlider.Setting.NetworkJitter) _jitterMs = value;
            else _lossPercent = value;
            _changedAt = Time.unscaledTime;
        }

        public bool IsVisible(ConsoleButton.Kind kind, int index)
        {
            IOperatorJointLink link = Link;
            return link != null && index < link.PredictorCount;
        }

        public string LabelFor(ConsoleButton.Kind kind, int index)
        {
            if (kind == ConsoleButton.Kind.TogglePredictor) return "";
            IOperatorJointLink link = Link;
            return link != null && index < link.PredictorCount ? link.PredictorLabel(index) : "";
        }

        public Color ColorFor(ConsoleButton.Kind kind, int index)
        {
            IOperatorJointLink link = Link;
            if (link == null || index >= link.PredictorCount) return Off;
            if (kind == ConsoleButton.Kind.TogglePredictor) return link.IsPredictorEnabled(index) ? Running : Off;
            if (index == link.ActivePredictorIndex) return Active;
            return link.IsPredictorEnabled(index) ? Running * 0.6f : Off;
        }

        public void OnPressed(ConsoleButton.Kind kind, int index)
        {
            IOperatorJointLink link = Link;
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
            }

            _nextRefresh = 0f;
        }

        private IOperatorJointLink Link => bridge != null ? bridge.Link : null;

        private void Update()
        {
            ApplyNetworkWhenSettled();
            if (panel == null || Time.unscaledTime < _nextRefresh)
            {
                return;
            }

            _nextRefresh = Time.unscaledTime + 0.25f;
            IOperatorJointLink link = Link;
            _text.Clear();
            if (link == null)
            {
                _text.Append("no joint link");
                panel.SetText(_text);
                return;
            }

            // The servo board going silent is the one thing on this panel that needs a person.
            if (link.BoardSilent)
            {
                _text.Append("<color=#FF5050><b>SERVO BOARD NOT ANSWERING</b> for ").Append(Mathf.RoundToInt(link.SecondsSinceFreshReading))
                    .Append(" s: frames arrive, readings do not. Power-cycle the robot.</color>\n");
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
            _text.Append("\nlast 30 s, degrees (gripper: ").Append(bridge.GripperErrorUnit).Append("); diagnostics, not results\n");
            _text.Append("</mspace><color=#A0A6AE>");
            for (int i = 0; i < link.PredictorCount; i++)
            {
                string description = link.PredictorDescription(i);
                if (description.Length == 0) continue;
                _text.Append('\n').Append(link.PredictorLabel(i)).Append(": ").Append(description);
            }

            _text.Append("\ncaps: draw the twin with one  ·  dots: turn one on or off  ·  sliders: lag, jitter, loss</color>");
            panel.SetText(_text);
        }

        private void ApplyNetworkWhenSettled()
        {
            if (_changedAt < 0f || Time.unscaledTime - _changedAt < SettleSeconds || bridge == null) return;
            _changedAt = -1f;
            if (_lagMs == _appliedLag && _jitterMs == _appliedJitter && _lossPercent == _appliedLoss) return;
            _appliedLag = _lagMs;
            _appliedJitter = _jitterMs;
            _appliedLoss = _lossPercent;
            bridge.ApplyCustomDisturbance(_lagMs, _jitterMs, _lossPercent);
            _nextRefresh = 0f;
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
