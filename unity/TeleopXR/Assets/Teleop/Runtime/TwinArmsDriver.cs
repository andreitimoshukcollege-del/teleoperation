using Teleop.Bridge;
using TMPro;
using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Drives the twin's predicted and last-measured arms from the joint-state downlink, and shows
    /// or hides each of the three arms (docs/adr/0018).
    /// <list type="bullet">
    /// <item><b>Predicted</b> (solid): the predictor's estimate of the robot's joints at this frame,
    /// applied in <c>Application.onBeforeRender</c>, the last hook before rendering
    /// (unity/CLAUDE.md).</item>
    /// <item><b>Measured</b> (faint): the newest frame as received, about one network trip
    /// old.</item>
    /// <item><b>Commanded</b> (ghost): the existing arm the clutch drives; this only shows or hides
    /// it.</item>
    /// </list>
    /// The predicted and measured arms stay hidden until the robot's first frame arrives. A thin
    /// adapter: every decision about the robot's motion is the Core predictor's.
    /// </summary>
    public sealed class TwinArmsDriver : MonoBehaviour
    {
        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private JetRoverArmRig predictedRig;
        [SerializeField] private TwinGripper predictedGripper;
        [SerializeField] private JetRoverArmRig measuredRig;
        [SerializeField] private TwinGripper measuredGripper;
        [Tooltip("The commanded arm's visuals (its TwinVisual groups), shown or hidden as one.")]
        [SerializeField] private GameObject[] commandedVisuals = new GameObject[0];
        [Tooltip("Optional: one line about the robot state stream.")]
        [SerializeField] private TMP_Text statusLabel;
        [SerializeField] private bool showPredicted = true;
        [SerializeField] private bool showMeasured = true;
        [SerializeField] private bool showCommanded = true;

        private int _shownStatus = -1;

        public bool ShowPredicted { get => showPredicted; set => showPredicted = value; }
        public bool ShowMeasured { get => showMeasured; set => showMeasured = value; }

        public bool ShowCommanded
        {
            get => showCommanded;
            set
            {
                showCommanded = value;
                foreach (GameObject visual in commandedVisuals)
                {
                    if (visual != null) visual.SetActive(value);
                }
            }
        }

        private void OnEnable() => Application.onBeforeRender += HandleBeforeRender;
        private void OnDisable() => Application.onBeforeRender -= HandleBeforeRender;

        private void Start() => ShowCommanded = showCommanded;

        private void HandleBeforeRender()
        {
            JetRoverJointStateLink link = bridge != null ? bridge.JointState : null;
            bool hasState = link != null && link.HasState;

            bool predicted = false, measured = false;
            if (hasState && link.TryGetPredicted(bridge.NowTicks, out JetRoverJointPose predictedPose))
            {
                predicted = Drive(predictedRig, predictedGripper, predictedPose);
            }

            if (hasState && link.TryGetMeasured(out JetRoverJointPose measuredPose))
            {
                measured = Drive(measuredRig, measuredGripper, measuredPose);
                // Unless the relay reads the wrist roll or the gripper back (just robot-feedback-ids), the
                // measured arm shows no gripper rather than the last command dressed as a reading.
                if (measuredGripper != null) measuredGripper.SetGripperVisible(measuredPose.RollSensed || measuredPose.GripperSensed);
            }

            SetVisible(predictedRig, showPredicted && predicted);
            SetVisible(measuredRig, showMeasured && measured);
            ShowStatus(link);
        }

        private static bool Drive(JetRoverArmRig rig, TwinGripper gripper, in JetRoverJointPose pose)
        {
            if (rig == null)
            {
                return false;
            }

            rig.ApplyAngles(pose.BaseYaw, pose.Proximal, pose.Distal, pose.WristPitch, wasClamped: false);
            if (gripper != null) gripper.Show(pose.Gripper, pose.WristRoll);
            return true;
        }

        private static void SetVisible(JetRoverArmRig rig, bool visible)
        {
            if (rig != null && rig.gameObject.activeSelf != visible)
            {
                rig.gameObject.SetActive(visible);
            }
        }

        private void ShowStatus(JetRoverJointStateLink link)
        {
            if (statusLabel == null)
            {
                return;
            }

            // 0 no link, 1 no frames, 2 stale, 3 live (unsynced), 4 live; the RTT changes the text too.
            int state;
            if (link == null) state = 0;
            else if (link.FramesReceived == 0) state = 1;
            else if (link.SecondsSinceLastFrame > 1f) state = 2;
            else state = link.IsSynced ? 4 : 3;
            int rtt = link != null ? Mathf.RoundToInt(link.RoundTripMilliseconds) : 0;
            int key = state * 10000 + Mathf.Clamp(rtt, 0, 9999);
            if (key == _shownStatus)
            {
                return;
            }

            _shownStatus = key;
            switch (state)
            {
                case 0:
                case 1: statusLabel.SetText("NO ROBOT STATE  ·  RobotHost needs the joint-state stream (ADR 0018)"); break;
                case 2: statusLabel.SetText("ROBOT STATE STALE  ·  no frame for over a second"); break;
                case 3: statusLabel.SetText("robot state live  ·  syncing clocks…"); break;
                default: statusLabel.SetText("robot state live  ·  rtt {0:0} ms", rtt); break;
            }
        }
    }
}
