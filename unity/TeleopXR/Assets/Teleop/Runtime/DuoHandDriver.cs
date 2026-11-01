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
    /// The FR3 Duo's operator input (docs/adr/0029): each hand drives its own arm, the left hand the left arm. A thin
    /// adapter: it reads hand joints from XR Hands and passes each hand to
    /// <see cref="SerialChainOperatorBridge.SetHand"/>, which decides (Teleop.RobotArm's <c>GrabGesture</c>):
    /// <list type="bullet">
    /// <item><b>Grab</b> (middle, ring and little fingers curled into the palm): that arm follows the palm's motion
    /// and rotation from there, the deadman of the clutch. Open the hand and the arm holds; grab again anywhere and it
    /// carries on from where it is.</item>
    /// <item><b>While grabbed, the thumb-index gap is that arm's gripper</b>: 8 cm or more open, 1.5 cm or less closed.
    /// Letting go holds the gripper; it never opens on its own.</item>
    /// <item><b>Losing tracking of a hand</b> lets go of its arm.</item>
    /// </list>
    /// What this measures is geometry only: three fingers' tip-to-wrist and knuckle-to-wrist distances and the tip gap.
    /// Runs before the bridge each frame, so the command it sends is this frame's.
    /// </summary>
    [DefaultExecutionOrder(-50)]
    public sealed class DuoHandDriver : MonoBehaviour
    {
        [SerializeField] private SerialChainOperatorBridge bridge;
        [Tooltip("The XR Origin whose tracking space the hand joints are reported in.")]
        [SerializeField] private XROrigin origin;
        [Tooltip("Optional: the motion mode and each arm's state and gripper (the display's caption line, as the JetRover's clutch line).")]
        [SerializeField] private TMP_Text statusLabel;
        [Tooltip("Optional: whether the robot's state is arriving (the line above the twin switches, as the JetRover's).")]
        [SerializeField] private TMP_Text linkLabel;

        private static readonly List<XRHandSubsystem> s_Subsystems = new List<XRHandSubsystem>();
        private XRHandSubsystem _hands;
        private long _shown = long.MinValue;
        private int _shownLink = int.MinValue;

        private void Update()
        {
            if (bridge == null || !bridge.IsReady)
            {
                return;
            }

            if (_hands == null || !_hands.running)
            {
                _hands = FindHands();
            }

            Transform space = origin != null ? (origin.TrackablesParent != null ? origin.TrackablesParent : origin.transform) : null;
            if (_hands == null || space == null)
            {
                bridge.SetHand(true, false, default, Quaternion.identity, float.NaN, float.NaN);
                bridge.SetHand(false, false, default, Quaternion.identity, float.NaN, float.NaN);
            }
            else
            {
                Feed(_hands.leftHand, true, space);
                Feed(_hands.rightHand, false, space);
            }

            Show();
        }

        private void Feed(XRHand hand, bool left, Transform space)
        {
            if (!hand.isTracked
                || !TryPosition(hand, XRHandJointID.Wrist, out Vector3 wrist)
                || !hand.GetJoint(XRHandJointID.Palm).TryGetPose(out Pose palm)
                || !TryPosition(hand, XRHandJointID.ThumbTip, out Vector3 thumb)
                || !TryPosition(hand, XRHandJointID.IndexTip, out Vector3 index)
                || !TryPosition(hand, XRHandJointID.MiddleProximal, out Vector3 middleKnuckle)
                || !TryPosition(hand, XRHandJointID.MiddleTip, out Vector3 middleTip)
                || !TryPosition(hand, XRHandJointID.RingProximal, out Vector3 ringKnuckle)
                || !TryPosition(hand, XRHandJointID.RingTip, out Vector3 ringTip)
                || !TryPosition(hand, XRHandJointID.LittleProximal, out Vector3 littleKnuckle)
                || !TryPosition(hand, XRHandJointID.LittleTip, out Vector3 littleTip))
            {
                bridge.SetHand(left, false, default, Quaternion.identity, float.NaN, float.NaN);
                return;
            }

            // Ratios do not care about the tracking space's scale; the pinch gap does.
            float curl = SerialChainOperatorBridge.CurlRatio(
                Vector3.Distance(middleTip, wrist), Vector3.Distance(middleKnuckle, wrist),
                Vector3.Distance(ringTip, wrist), Vector3.Distance(ringKnuckle, wrist),
                Vector3.Distance(littleTip, wrist), Vector3.Distance(littleKnuckle, wrist));
            float pinch = Vector3.Distance(thumb, index) * space.lossyScale.x;
            bridge.SetHand(left, true, space.TransformPoint(palm.position), space.rotation * palm.rotation, curl, pinch);
        }

        private static bool TryPosition(XRHand hand, XRHandJointID id, out Vector3 position)
        {
            bool ok = hand.GetJoint(id).TryGetPose(out Pose pose);
            position = pose.position;
            return ok;
        }

        private void Show()
        {
            ShowLink();
            if (statusLabel == null)
            {
                return;
            }

            // 4: locked by a double curl but holding, because the hand is lost.
            int l = (int)bridge.ArmState(true), r = (int)bridge.ArmState(false);
            if (bridge.IsLocked(true) && l == (int)DuoArmState.Idle) l = 4;
            if (bridge.IsLocked(false) && r == (int)DuoArmState.Idle) r = 4;
            int lp = Mathf.RoundToInt(bridge.Opening(true) * 100f), rp = Mathf.RoundToInt(bridge.Opening(false) * 100f);
            int motion = !bridge.MotionAllowed ? 3 : bridge.MotionMode == JetRoverMotionMode.On ? 1 : bridge.MotionMode == JetRoverMotionMode.Paused ? 2 : 0;
            long key = (((l * 5 + r) * 101L + lp) * 101 + rp) * 4 + motion;
            if (key == _shown)
            {
                return;
            }

            // A new string only when something shown has changed (a few times a second at most).
            _shown = key;
            string motionText = motion == 3 ? "MOTION LOCKED (config)" : motion == 1 ? "ROBOT ON" : motion == 2 ? "PAUSED" : "ROBOT OFF";
            statusLabel.SetText($"{motionText}  ·  LEFT {Arm(l)} grip {lp}%  ·  RIGHT {Arm(r)} grip {rp}%");
        }

        /// <summary>The line above the twin switches: whether the robot's state is arriving, and a disturbance when one is on.</summary>
        private void ShowLink()
        {
            if (linkLabel == null)
            {
                return;
            }

            // 0 no frames, 1 stale, 2 live (unsynced), 3 live; the RTT and a disturbance change the text too.
            int state = bridge.FramesReceived == 0 ? 0 : bridge.SecondsSinceLastFrame > 1f ? 1 : bridge.IsSynced ? 3 : 2;
            int rtt = Mathf.Clamp(Mathf.RoundToInt(bridge.RoundTripMilliseconds), 0, 9999);
            bool disturbed = bridge.JointDisturbance != null && bridge.JointDisturbance.IsActive;
            int key = (disturbed ? 100000 : 0) + state * 10000 + rtt;
            if (key == _shownLink)
            {
                return;
            }

            _shownLink = key;
            switch (state)
            {
                case 0: linkLabel.SetText("NO ROBOT STATE  ·  start just fr3-sim-host"); break;
                case 1: linkLabel.SetText("ROBOT STATE STALE  ·  no frame for over a second"); break;
                case 2: linkLabel.SetText("robot state live  ·  syncing clocks…"); break;
                default:
                    if (disturbed) linkLabel.SetText("robot state live  ·  rtt {0:0} ms  ·  NETWORK DISTURBED", rtt);
                    else linkLabel.SetText("robot state live  ·  rtt {0:0} ms", rtt);
                    break;
            }
        }

        private string Arm(int state) =>
            state == 4 ? "<color=#33BFFF>LOCKED · hand lost</color>"
            : state == (int)DuoArmState.Following ? (bridge.UsesToggle ? "<color=#33BFFF>LOCKED</color>" : "<color=#33BFFF>HELD</color>")
            : state == (int)DuoArmState.StoppedShort ? "<color=#FF9919>OUT OF REACH</color>"
            : state == (int)DuoArmState.Blocked ? "<color=#FF3333>BLOCKED</color>"
            : "free";

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
