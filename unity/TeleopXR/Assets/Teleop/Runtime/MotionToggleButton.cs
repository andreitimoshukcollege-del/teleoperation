using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The robot's start/stop switch (docs/adr/0017): a button on the bench that you press down with
    /// an index fingertip. Each press toggles <see cref="JetRoverOperatorBridge.MotionEnabled"/>. The
    /// scene starts with it off, so nothing reaches the robot until someone presses it. The virtual arm
    /// follows the controls either way.
    ///
    /// <para>It reads the fingertips from XR Hands (<see cref="FingertipPress"/>) rather than being an
    /// XRI interactable, so no ray or pinch can press it from a distance; a clutch pinch elsewhere in
    /// the room cannot start the robot.</para>
    /// </summary>
    public sealed class MotionToggleButton : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private XROrigin origin;
        [Tooltip("The cap that is pressed, sitting level; its top face is the button's surface. Recoloured with the state.")]
        [SerializeField] private Renderer cap;
        [SerializeField] private TMP_Text label;
        [SerializeField] private float radiusMeters = 0.03f;
        [Tooltip("How far below the cap's top a fingertip counts as pressing.")]
        [SerializeField] private float pressDepthMeters = 0.012f;
        [Tooltip("How far away a fingertip must go before the next press counts.")]
        [SerializeField] private float rearmDistanceMeters = 0.07f;
        [SerializeField] private float cooldownSeconds = 0.6f;
        [SerializeField] private Color offColor = new Color(0.9f, 0.12f, 0.1f);
        [SerializeField] private Color onColor = new Color(0.15f, 0.95f, 0.35f);
        [SerializeField] private Color lockedColor = new Color(0.35f, 0.36f, 0.4f);

        private FingertipPress _press;
        private int _shown = -1;
        private MaterialPropertyBlock _block;

        private void Update()
        {
            if (bridge == null || cap == null)
            {
                return;
            }

            _press ??= new FingertipPress(radiusMeters, pressDepthMeters, rearmDistanceMeters, cooldownSeconds);
            if (_press.Update(cap, origin) && bridge.HardwareMotionAllowed)
            {
                bridge.MotionEnabled = !bridge.MotionEnabled;
                Debug.Log($"[xr] robot motion {(bridge.MotionEnabled ? "STARTED" : "STOPPED")} by the bench button");
            }

            Show();
        }

        private void Show()
        {
            int state = !bridge.HardwareMotionAllowed ? 2 : bridge.MotionEnabled ? 1 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 2 ? lockedColor : state == 1 ? onColor : offColor;
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null)
            {
                label.text = state == 2 ? "ROBOT LOCKED\n<size=60%>config: ConfirmHardwareMotion off</size>"
                    : state == 1 ? "ROBOT ON\n<size=60%>press to stop</size>"
                    : "ROBOT OFF\n<size=60%>press to start</size>";
                label.color = color;
            }
        }
    }
}
