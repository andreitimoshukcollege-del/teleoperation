using System.Collections.Generic;
using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.Hands;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Galaxy XR's operator input (docs/adr/0017-galaxy-xr-clutch-and-gripper.md): one hand clutches
    /// the arm, the other drives the gripper. A thin adapter: it reads hand joints from XR Hands and
    /// passes them to the Bridge's <see cref="JetRoverClutch"/> and
    /// <see cref="JetRoverOperatorBridge.GripperFraction"/>. The mapping itself is Core's.
    /// <list type="bullet">
    /// <item><b>Clutch hand</b> (right by default): a pinch engages and the arm follows the hand's
    /// motion from there. It engages under <see cref="engageBelowMeters"/> thumb-to-index distance and
    /// releases over <see cref="releaseAboveMeters"/>, so a borderline pinch doesn't chatter. Releasing,
    /// or losing tracking of that hand, disengages, and the arm holds.</item>
    /// <item><b>Gripper hand</b>: how tightly it pinches is how far the gripper closes, and how far it
    /// twists from palm-down is the wrist roll. Losing tracking of that hand holds both; the gripper
    /// never opens on its own.</item>
    /// </list>
    /// Runs before <see cref="JetRoverOperatorBridge"/> each frame, so the target the bridge sends is
    /// this frame's.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public sealed class HandClutchDriver : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private JetRoverArmRig armRig;
        [Tooltip("The XR Origin whose tracking space the hand joints are reported in.")]
        [SerializeField] private XROrigin origin;
        [SerializeField] private Handedness clutchHand = Handedness.Right;
        [SerializeField] private float engageBelowMeters = 0.015f;
        [SerializeField] private float releaseAboveMeters = 0.03f;
        [Tooltip("Off: the wrist keeps the pitch it had when the clutch engaged.")]
        [SerializeField] private bool followPitch = true;
        [Tooltip("Optional: a line of text that shows whether the clutch is on and the gripper command.")]
        [SerializeField] private TMP_Text statusLabel;

        private static readonly List<XRHandSubsystem> s_Subsystems = new List<XRHandSubsystem>();
        private XRHandSubsystem _hands;
        private bool _logged;
        private int _shownState = -1;

        public bool IsClutched => bridge != null && bridge.Clutch != null && bridge.Clutch.IsEngaged;

        private void Update()
        {
            JetRoverClutch clutch = bridge != null ? bridge.Clutch : null;
            if (clutch == null)
            {
                return;
            }

            if (!_logged)
            {
                _logged = true;
                Debug.Log($"[xr] input: clutch on the {clutchHand} hand, scale {clutch.Scale:0.###}, pitch {(followPitch ? "follows" : "held")}; gripper on the other hand's pinch");
            }

            if (_hands == null || !_hands.running)
            {
                _hands = FindHands();
            }

            Transform space = origin != null ? (origin.TrackablesParent != null ? origin.TrackablesParent : origin.transform) : null;
            if (_hands == null || space == null)
            {
                if (clutch.IsEngaged) clutch.Release();
                Show(clutch);
                return;
            }

            XRHand driving = clutchHand == Handedness.Right ? _hands.rightHand : _hands.leftHand;
            XRHand gripping = clutchHand == Handedness.Right ? _hands.leftHand : _hands.rightHand;

            if (TryPinch(driving, space, out Vector3 point, out Quaternion rotation, out _, out float distance))
            {
                if (!clutch.IsEngaged && distance < engageBelowMeters)
                {
                    clutch.Engage(point, rotation);
                }
                else if (clutch.IsEngaged && distance > releaseAboveMeters)
                {
                    clutch.Release();
                }

                clutch.Follow(point, rotation, followPitch);
            }
            else if (clutch.IsEngaged)
            {
                clutch.Release();
            }

            if (TryPinch(gripping, space, out _, out Quaternion gripRotation, out bool hasPalm, out float gripDistance))
            {
                bridge.GripperFraction = JetRoverClutch.GripperFromPinchDistance(gripDistance, bridge.Config);
                if (hasPalm)
                {
                    clutch.SetRollFromHand(gripRotation, bridge.Config.GripperRollSign, bridge.Config.GripperRollLimitDegrees);
                }
            }

            if (armRig != null)
            {
                armRig.Highlighted = clutch.IsEngaged;
            }

            Show(clutch);
        }

        /// <summary>The pinch point (midway between thumb and index tips), the palm's rotation, and the tip distance, in world space.</summary>
        private static bool TryPinch(XRHand hand, Transform space, out Vector3 point, out Quaternion rotation, out bool hasPalm, out float distance)
        {
            point = default;
            rotation = Quaternion.identity;
            hasPalm = false;
            distance = float.NaN;
            if (!hand.isTracked
                || !hand.GetJoint(XRHandJointID.ThumbTip).TryGetPose(out Pose thumb)
                || !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose index))
            {
                return false;
            }

            distance = Vector3.Distance(thumb.position, index.position) * space.lossyScale.x;
            point = space.TransformPoint((thumb.position + index.position) * 0.5f);
            if (hand.GetJoint(XRHandJointID.Palm).TryGetPose(out Pose palm))
            {
                // XR Hands' palm: forward toward the fingertips, up out of the back of the hand.
                rotation = space.rotation * palm.rotation;
                hasPalm = true;
            }

            return true;
        }

        private void Show(JetRoverClutch clutch)
        {
            if (statusLabel == null)
            {
                return;
            }

            int percent = Mathf.RoundToInt(bridge.GripperFraction * 100f);
            int degrees = Mathf.RoundToInt(bridge.WristRollRadians * Mathf.Rad2Deg);
            int motion = !bridge.HardwareMotionAllowed ? 2 : bridge.MotionEnabled ? 1 : 0;
            int state = ((motion * 2 + (clutch.IsEngaged ? 1 : 0)) * 1000 + percent) * 1000 + degrees + 500;
            if (state == _shownState)
            {
                return;
            }

            _shownState = state;
            // Allocation-free SetText needs a fixed format string per case.
            switch (motion * 2 + (clutch.IsEngaged ? 1 : 0))
            {
                case 0: statusLabel.SetText("ROBOT OFF  ·  pinch to drive  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
                case 1: statusLabel.SetText("ROBOT OFF  ·  CLUTCH ON  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
                case 2: statusLabel.SetText("ROBOT ON  ·  pinch to drive  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
                case 3: statusLabel.SetText("ROBOT ON  ·  CLUTCH ON  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
                case 4: statusLabel.SetText("ROBOT LOCKED (config)  ·  pinch to drive  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
                default: statusLabel.SetText("ROBOT LOCKED (config)  ·  CLUTCH ON  ·  grip {0:0}%  roll {1:0}°", (float)percent, (float)degrees); break;
            }
        }

        private static XRHandSubsystem FindHands()
        {
            SubsystemManager.GetSubsystems(s_Subsystems);
            foreach (XRHandSubsystem subsystem in s_Subsystems)
            {
                if (subsystem.running)
                {
                    return subsystem;
                }
            }

            return null;
        }
    }
}
