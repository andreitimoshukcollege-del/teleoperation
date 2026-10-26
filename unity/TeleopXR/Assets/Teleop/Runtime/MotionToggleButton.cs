using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The robot's power and pause switches on the bench (docs/adr/0017, docs/adr/0023 decision 5): buttons
    /// you press down with an index fingertip. They set <see cref="JetRoverOperatorBridge.MotionMode"/>:
    /// <list type="bullet">
    /// <item><b>Power</b> (the big button): off or paused -> on; on -> off. Off parks the arm at its rest
    /// pose and switches its torque off, so it rests unpowered; on switches torque back on first, and no
    /// command is sent until RobotHost says the arm is holding.</item>
    /// <item><b>Pause</b> (the small one beside it): on -> paused; paused -> on. Paused sends nothing and
    /// keeps torque on, so the arm holds where it is. Pausing never powers a relaxed arm.</item>
    /// </list>
    /// The scene starts off, so nothing reaches the robot until someone presses power. The virtual arm
    /// follows the controls either way. The labels show what RobotHost reports: parking, relaxed, a
    /// refused park and why, or waiting for torque.
    ///
    /// <para>They read the fingertips from XR Hands (<see cref="FingertipPress"/>) rather than being XRI
    /// interactables, so no ray or pinch can press them from a distance; a clutch pinch elsewhere in the
    /// room cannot start the robot.</para>
    /// </summary>
    public sealed class MotionToggleButton : MonoBehaviour
    {
        public enum Role { Power, Pause }

        [SerializeField] private Role role = Role.Power;
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
        [SerializeField] private Color pausedColor = new Color(1f, 0.62f, 0.15f);
        [SerializeField] private Color lockedColor = new Color(0.35f, 0.36f, 0.4f);

        private FingertipPress _press;
        private int _shown = -1;
        private MaterialPropertyBlock _block;

        public Role ButtonRole => role;

        private void Update()
        {
            if (bridge == null || cap == null)
            {
                return;
            }

            _press ??= new FingertipPress(radiusMeters, pressDepthMeters, rearmDistanceMeters, cooldownSeconds);
            if (_press.Update(cap, origin) && bridge.HardwareMotionAllowed)
            {
                JetRoverMotionMode before = bridge.MotionMode;
                JetRoverMotionMode after = role == Role.Power
                    ? (before == JetRoverMotionMode.On ? JetRoverMotionMode.Off : JetRoverMotionMode.On)
                    : (before == JetRoverMotionMode.On ? JetRoverMotionMode.Paused : before == JetRoverMotionMode.Paused ? JetRoverMotionMode.On : before);
                if (after != before)
                {
                    bridge.MotionMode = after;
                    Debug.Log($"[xr] robot motion {before} -> {after} by the {role} button");
                }
            }

            Show();
        }

        private void Show()
        {
            JetRoverJointStateLink link = bridge.JointState;
            JetRoverArmPower power = link != null ? link.ArmPower : JetRoverArmPower.Unknown;
            JetRoverMotionMode mode = bridge.MotionMode;
            int remaining = link != null ? Mathf.RoundToInt(link.ArmPowerRemainingDeg) : 0;
            string reason = link != null ? link.ArmPowerReasonText : "";
            int key = !bridge.HardwareMotionAllowed ? -2
                : (((int)mode * 10 + (int)power) * 2 + (bridge.WaitingForHold ? 1 : 0)) * 1000 + Mathf.Clamp(remaining, 0, 999)
                  + (reason.Length > 0 ? 1_000_000 + reason.Length : 0);
            if (key == _shown)
            {
                return;
            }

            _shown = key;
            Color color;
            string text;
            if (!bridge.HardwareMotionAllowed)
            {
                color = lockedColor;
                text = role == Role.Power ? "ROBOT LOCKED\n<size=60%>config: ConfirmHardwareMotion off</size>" : "PAUSE";
            }
            else if (role == Role.Pause)
            {
                bool paused = mode == JetRoverMotionMode.Paused;
                color = paused ? pausedColor : mode == JetRoverMotionMode.On ? onColor * 0.7f : lockedColor;
                text = paused ? "PAUSED\n<size=60%>holding · press to resume</size>"
                    : mode == JetRoverMotionMode.On ? "PAUSE\n<size=60%>hold in place</size>" : "PAUSE";
            }
            else if (mode == JetRoverMotionMode.On)
            {
                color = onColor;
                text = bridge.WaitingForHold ? "STARTING\n<size=60%>torque on…</size>" : "ROBOT ON\n<size=60%>press to park and relax</size>";
            }
            else if (mode == JetRoverMotionMode.Paused)
            {
                color = pausedColor;
                text = "PAUSED\n<size=60%>press to start · torque held</size>";
            }
            else
            {
                color = offColor;
                text = power == JetRoverArmPower.Parking ? $"PARKING\n<size=60%>{remaining}° to the rest pose</size>"
                    : power == JetRoverArmPower.Relaxed ? "ROBOT OFF\n<size=60%>relaxed · press to start</size>"
                    : reason.Length > 0 ? $"ROBOT OFF\n<size=60%>not parked: {reason}</size>"
                    : "ROBOT OFF\n<size=60%>press to start</size>";
            }

            _block ??= new MaterialPropertyBlock();
            cap.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            cap.SetPropertyBlock(_block);
            if (label != null)
            {
                label.text = text;
                label.color = color;
            }
        }
    }
}
