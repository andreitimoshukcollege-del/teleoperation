using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// A fingertip switch on the bench that shows or hides one of the twin's arms (docs/adr/0018).
    /// The same press as the motion button (<see cref="FingertipPress"/>): only a fingertip on the cap
    /// counts. Lit in that arm's colour when the arm is shown, dim when hidden; the colours are the
    /// twin's code (commanded blue, predicted green, measured amber).
    /// </summary>
    public sealed class TwinVisibilityButton : MonoBehaviour
    {
        public enum Arm { Predicted, Commanded, Measured }

        [SerializeField] private TwinArmsDriver arms;
        [SerializeField] private Arm arm;
        [SerializeField] private XROrigin origin;
        [SerializeField] private Renderer cap;
        [SerializeField] private TMP_Text label;
        [SerializeField] private Color onColor = Color.white;
        [SerializeField] private Color offColor = new Color(0.12f, 0.12f, 0.14f);

        private FingertipPress _press;
        private int _shown = -1;
        private MaterialPropertyBlock _block;

        private bool Visible
        {
            get => arm == Arm.Predicted ? arms.ShowPredicted : arm == Arm.Measured ? arms.ShowMeasured : arms.ShowCommanded;
            set
            {
                if (arm == Arm.Predicted) arms.ShowPredicted = value;
                else if (arm == Arm.Measured) arms.ShowMeasured = value;
                else arms.ShowCommanded = value;
            }
        }

        private void Update()
        {
            if (arms == null || cap == null)
            {
                return;
            }

            _press ??= new FingertipPress(0.022f, 0.012f, 0.05f, 0.4f);
            if (_press.Update(cap, origin))
            {
                Visible = !Visible;
            }

            int state = Visible ? 1 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = Visible ? onColor : offColor;
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null)
            {
                label.alpha = Visible ? 1f : 0.45f;
            }
        }
    }
}
