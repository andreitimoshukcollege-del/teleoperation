using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The FR3 Duo's buttons (docs/adr/0029 decision 7 and amendment): <b>Power</b>, the motion switch, and <b>Recentre</b>,
    /// which anchors the robot to where you stand and face now (the absolute mapping). Each is a button you press down with a fingertip
    /// (<see cref="FingertipPress"/>, so no ray, pinch or grab elsewhere can press it). It sets
    /// <see cref="SerialChainOperatorBridge.MotionEnabled"/>: off sends nothing, and on first takes the robot's measured
    /// pose as the command and lets go of both arms, so nothing jumps to where the hands were. The scene starts off.
    /// The serial-chain host has no arm power to park or relax (docs/adr/0027 decision 5): off only stops commanding,
    /// and the robot's own shaper brings it to rest at its last target.
    /// </summary>
    public sealed class Fr3MotionButton : MonoBehaviour
    {
        public enum Role { Power, Recentre }

        [SerializeField] private Role role = Role.Power;
        [SerializeField] private SerialChainOperatorBridge bridge;
        [SerializeField] private XROrigin origin;
        [Tooltip("The cap that is pressed, sitting level; its top face is the button's surface. Recoloured with the state.")]
        [SerializeField] private Renderer cap;
        [SerializeField] private TMP_Text label;
        [SerializeField] private float radiusMeters = 0.03f;
        [SerializeField] private float pressDepthMeters = 0.012f;
        [SerializeField] private float rearmDistanceMeters = 0.07f;
        [SerializeField] private float cooldownSeconds = 0.6f;
        [SerializeField] private Color offColor = new Color(0.9f, 0.12f, 0.1f);
        [SerializeField] private Color onColor = new Color(0.15f, 0.95f, 0.35f);
        [SerializeField] private Color lockedColor = new Color(0.35f, 0.36f, 0.4f);

        private FingertipPress _press;
        private MaterialPropertyBlock _block;
        private int _shown = -1;
        private float _flashUntil;

        public Role ButtonRole => role;

        private void Update()
        {
            if (bridge == null || cap == null || !bridge.IsReady)
            {
                return;
            }

            _press ??= new FingertipPress(radiusMeters, pressDepthMeters, rearmDistanceMeters, cooldownSeconds);
            bool pressed = _press.Update(cap, origin);
            if (role == Role.Recentre)
            {
                if (pressed && bridge.IsAbsolute)
                {
                    bridge.Recentre();
                    _flashUntil = Time.unscaledTime + 0.6f;
                }

                ShowRecentre();
                return;
            }

            if (pressed && bridge.MotionAllowed)
            {
                bool before = bridge.MotionEnabled;
                bridge.MotionEnabled = !before;
                Debug.Log($"[fr3] robot motion {(before ? "on" : "off")} -> {(bridge.MotionEnabled ? "on" : "off")} by the button");
            }

            Show();
        }

        private void ShowRecentre()
        {
            int state = !bridge.IsAbsolute ? 2 : Time.unscaledTime < _flashUntil ? 1 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 1 ? onColor : state == 0 ? new Color(0.25f, 0.55f, 1f) : lockedColor;
            string text = state == 2 ? "RECENTRE\n<size=60%>relative mapping: not used</size>" : "RECENTRE\n<size=60%>stand comfortably, press</size>";
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }

        private void OnApplicationFocus(bool focused)
        {
            // Builds only: the editor loses focus whenever another window is clicked. (Pausing is the bridge's.)
            if (role == Role.Power && !focused && !Application.isEditor && bridge != null) bridge.MotionEnabled = false;
        }

        private void Show()
        {
            int state = !bridge.MotionAllowed ? 2 : bridge.MotionEnabled ? 1 : bridge.FramesReceived == 0 ? 3 : 0;
            if (state == _shown)
            {
                return;
            }

            _shown = state;
            Color color = state == 1 ? onColor : state == 0 ? offColor : lockedColor;
            string text = state == 1 ? "ROBOT ON\n<size=60%>press to stop commanding</size>"
                : state == 2 ? "MOTION LOCKED\n<size=60%>config: AllowMotion off</size>"
                : state == 3 ? "ROBOT OFF\n<size=60%>waiting for the robot's state</size>"
                : "ROBOT OFF\n<size=60%>press to start</size>";
            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null) label.SetText(text);
        }
    }
}
