using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// One fingertip cap of the <see cref="OperatorConsole"/> (docs/adr/0023): it reports its press to
    /// the console and shows the colour the console gives it. The same press as the motion button and
    /// the twin switches (<see cref="FingertipPress"/>): only a fingertip on the cap counts.
    /// </summary>
    public sealed class ConsoleButton : MonoBehaviour
    {
        public enum Kind { SelectPredictor, TogglePredictor }

        [SerializeField] private OperatorConsole console;
        [SerializeField] private Kind kind;
        [SerializeField] private int index;
        [SerializeField] private XROrigin origin;
        [SerializeField] private Renderer cap;
        [SerializeField] private TMP_Text label;

        private FingertipPress _press;
        private MaterialPropertyBlock _block;
        private Color _shown = new Color(-1f, -1f, -1f);
        private string _shownLabel;

        public Kind ButtonKind => kind;

        public int Index => index;

        private void Update()
        {
            if (console == null || cap == null)
            {
                return;
            }

            if (!console.IsVisible(kind, index))
            {
                SetShown(false);
                return;
            }

            SetShown(true);
            _press ??= new FingertipPress(0.018f, 0.012f, 0.04f, 0.4f);
            if (_press.Update(cap, origin))
            {
                console.OnPressed(kind, index);
            }

            Color color = console.ColorFor(kind, index);
            if (color != _shown)
            {
                _shown = color;
                _block ??= new MaterialPropertyBlock();
                cap.GetPropertyBlock(_block);
                _block.SetColor("_BaseColor", color);
                _block.SetColor("_Color", color);
                cap.SetPropertyBlock(_block);
            }

            if (label != null)
            {
                string text = console.LabelFor(kind, index);
                if (!ReferenceEquals(text, _shownLabel))
                {
                    _shownLabel = text;
                    label.SetText(text);
                }
            }
        }

        /// <summary>Hides the cap and its label for a slot the config leaves empty.</summary>
        private void SetShown(bool shown)
        {
            if (cap.enabled != shown) cap.enabled = shown;
            if (label != null && label.enabled != shown) label.enabled = shown;
        }
    }
}
