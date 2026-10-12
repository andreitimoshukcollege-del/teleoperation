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
        [Tooltip("Optional: the camera display. While robot motion is off, a pinch that starts with the hand's ray on it moves the display instead of the arm.")]
        [SerializeField] private DisplayMover displayMover;
        [Tooltip("Optional: drawn from the hand to the display while the ray is on it.")]
        [SerializeField] private LineRenderer pointer;
        [Tooltip("Where the hand's ray starts relative to the head, before the pinch point: about the shoulder (x is mirrored for the left hand).")]
        [SerializeField] private Vector3 shoulderFromHead = new Vector3(0.17f, -0.17f, 0f);
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

            bool moving = displayMover != null && displayMover.IsMoving;
            // The display is only offered while the robot cannot move, so a pinch meant for it can
            // never become a clutch on a live arm, and the reverse.
            bool displayOffered = displayMover != null && !(bridge.MotionEnabled && bridge.HardwareMotionAllowed);
            bool pointing = false;
            Ray ray = default;
            float hitDistance = 0f;
            if (TryPinch(driving, space, out Vector3 point, out Quaternion rotation, out _, out float distance))
            {
                ray = HandRay(point);
                pointing = displayOffered && !clutch.IsEngaged && displayMover.TryHit(ray, out hitDistance);

                if (!clutch.IsEngaged && !moving && distance < engageBelowMeters)
                {
                    // Where the pinch starts decides what it holds: the display under the ray, or the arm.
                    if (pointing)
                    {
                        displayMover.Begin(ray, hitDistance);
                        moving = true;
                    }
                    else
                    {
                        clutch.Engage(point, rotation);
                    }
                }
                else if ((clutch.IsEngaged || moving) && distance > releaseAboveMeters)
                {
                    clutch.Release();
                    if (displayMover != null) displayMover.End();
                    moving = false;
                }

                if (moving && displayOffered)
                    displayMover.Drag(ray);
                else if (moving)
                {
                    // Motion was switched on mid-move: let go of the display.
                    displayMover.End();
                    moving = false;
                }
                else
                    clutch.Follow(point, rotation, followPitch);
            }
            else
            {
                if (clutch.IsEngaged) clutch.Release();
                if (moving) displayMover.End();
                moving = false;
            }

            if (displayMover != null) displayMover.Hovered = pointing || moving;
            ShowPointer(pointing || moving, point, ray, moving ? -1f : hitDistance);

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

        /// <summary>
        /// The hand's pointing ray: from about the shoulder through the pinch point, the way Android
        /// XR and Quest aim hand rays. It is steadier than the hand's own orientation, and it is
        /// where people expect a pinch to point.
        /// </summary>
        private Ray HandRay(Vector3 pinchPoint)
        {
            Camera eye = Camera.main;
            if (eye == null)
            {
                return new Ray(pinchPoint, Vector3.forward);
            }

            Transform head = eye.transform;
            Vector3 flatRight = Vector3.ProjectOnPlane(head.right, Vector3.up).normalized;
            float side = clutchHand == Handedness.Right ? 1f : -1f;
            Vector3 shoulder = head.position + flatRight * (shoulderFromHead.x * side) + Vector3.up * shoulderFromHead.y;
            Vector3 direction = pinchPoint - shoulder;
            return new Ray(pinchPoint, direction.sqrMagnitude > 1e-6f ? direction.normalized : head.forward);
        }

        private void ShowPointer(bool visible, Vector3 from, Ray ray, float distance)
        {
            if (pointer == null)
            {
                return;
            }

            if (pointer.enabled != visible) pointer.enabled = visible;
            if (!visible)
            {
                return;
            }

            // While holding, the line runs to the display's grab point; while aiming, to where the ray hits it.
            Vector3 to = distance >= 0f ? ray.GetPoint(distance) : displayMover.GrabPoint(ray);
            pointer.SetPosition(0, from);
            pointer.SetPosition(1, to);
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
