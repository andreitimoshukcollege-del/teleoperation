using Teleop.Bridge;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.Hands;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// The arm-speed slider on the bench (docs/adr/0018): touch the track with an index fingertip and
    /// slide along it. Near you is slow, far is fast. It sets
    /// <see cref="JetRoverOperatorBridge.SpeedRadPerSec"/>, which rides in every joint command; RobotHost
    /// caps it at the speed it was deployed with, and the predictor replays at the same speed. Like
    /// the buttons it reads fingertips from XR Hands, so no ray or pinch can move it.
    /// </summary>
    public sealed class SpeedSlider : MonoBehaviour
    {
        /// <summary>What the slider sets.</summary>
        public enum Setting
        {
            /// <summary>The arm's joint speed, rad/s (<see cref="JetRoverOperatorBridge.SpeedRadPerSec"/>).</summary>
            ArmSpeed,

            /// <summary>An artificial delay on the robot's state reports, ms (<see cref="JetRoverJointStateLink.AddedDelayMs"/>), to see the prediction work on a fast LAN.</summary>
            AddedDelay,

            /// <summary>The console's network disturbance (docs/adr/0023): one-way lag, ms, both ways.</summary>
            NetworkLag,

            /// <summary>The console's network disturbance: uniform jitter, ms, both ways.</summary>
            NetworkJitter,

            /// <summary>The console's network disturbance: random loss, percent, both ways.</summary>
            NetworkLoss,
        }

        [SerializeField] private Setting setting = Setting.ArmSpeed;
        [SerializeField] private JetRoverOperatorBridge bridge;
        [Tooltip("The operator console the network settings go to (they are applied together, there).")]
        [SerializeField] private OperatorConsole console;
        [SerializeField] private XROrigin origin;
        [Tooltip("The slow end of the track (its top surface, centre line).")]
        [SerializeField] private Transform trackStart;
        [Tooltip("The fast end of the track.")]
        [SerializeField] private Transform trackEnd;
        [SerializeField] private Transform knob;
        [SerializeField] private TMP_Text label;
        [Tooltip("The value at the near end, the far end, and at start (rad/s for speed, ms for delay).")]
        [SerializeField] private float minRadPerSec = 0.5f;
        [SerializeField] private float maxRadPerSec = 6f;
        [SerializeField] private float startRadPerSec = 3.8f;
        [Tooltip("How far beside the track a fingertip still counts, metres.")]
        [SerializeField] private float touchRadius = 0.025f;

        private XRHandSubsystem _hands;
        private float _value;
        private int _shown = -1;
        private bool _course;
        private bool _wasTest;

        private void Start()
        {
            _value = Mathf.Clamp(startRadPerSec, minRadPerSec, maxRadPerSec);
            Apply();
        }

        private void Update()
        {
            if (bridge == null || trackStart == null || trackEnd == null || origin == null)
            {
                return;
            }

            // The training course sets the arm speed while it runs (docs/adr/0024 decision 4): the knob
            // follows it and the slider cannot be moved.
            if (setting == Setting.ArmSpeed && bridge.Training != null && bridge.Training.Running)
            {
                _value = Mathf.Clamp(bridge.Training.SpeedRadPerSec, minRadPerSec, maxRadPerSec);
                _course = true;
                Apply();
                return;
            }

            // Training data must be real-network data (docs/adr/0024 decision 8): selecting TRAIN puts
            // the disturbance and the added delay back to zero, and a running course holds them there.
            if (setting != Setting.ArmSpeed && bridge.Training != null)
            {
                bool test = bridge.Training.Test;
                if (_wasTest && !test) _value = 0f;
                _wasTest = test;
                if (bridge.Training.Running)
                {
                    _value = 0f;
                    _course = true;
                    Apply();
                    return;
                }
            }

            _course = false;
            if (_hands == null || !_hands.running)
            {
                _hands = FingertipPress.FindHands();
            }

            Transform space = origin.TrackablesParent != null ? origin.TrackablesParent : origin.transform;
            if (_hands != null && (TryTouch(_hands.rightHand, space, out float t) || TryTouch(_hands.leftHand, space, out t)))
            {
                // Tenths of a rad/s, so the value settles instead of trembling with the fingertip.
                float step = setting == Setting.ArmSpeed ? 0.1f : setting == Setting.NetworkLoss ? 1f : setting == Setting.NetworkJitter ? 5f : 10f;
                _value = Mathf.Round(Mathf.Lerp(minRadPerSec, maxRadPerSec, t) / step) * step;
            }

            Apply();
        }

        private bool TryTouch(XRHand hand, Transform space, out float t)
        {
            t = 0f;
            if (!hand.isTracked || !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose tip))
            {
                return false;
            }

            Vector3 p = space.TransformPoint(tip.position);
            Vector3 a = trackStart.position, b = trackEnd.position, ab = b - a;
            float along = Vector3.Dot(p - a, ab) / ab.sqrMagnitude;
            if (along < -0.05f || along > 1.05f)
            {
                return false;
            }

            Vector3 closest = a + Mathf.Clamp01(along) * ab;
            Vector3 off = p - closest;
            if (off.y < -0.012f || off.y > 0.02f || new Vector2(off.x, off.z).magnitude > touchRadius)
            {
                return false;
            }

            t = Mathf.Clamp01(along);
            return true;
        }

        private void Apply()
        {
            if (bridge != null)
            {
                if (setting == Setting.ArmSpeed)
                    bridge.SpeedRadPerSec = _value;
                else if (setting == Setting.AddedDelay)
                {
                    if (bridge.JointState != null) bridge.JointState.AddedDelayMs = _value;
                }
                else if (console != null)
                    console.SetNetwork(setting, _value);
            }

            int shown = Mathf.RoundToInt(_value * 10f) * 2 + (_course ? 1 : 0);
            if (shown == _shown)
            {
                return;
            }

            _shown = shown;
            if (knob != null && trackStart != null && trackEnd != null)
            {
                float t = Mathf.InverseLerp(minRadPerSec, maxRadPerSec, _value);
                knob.position = Vector3.Lerp(trackStart.position, trackEnd.position, t);
            }

            if (label != null)
            {
                switch (setting)
                {
                    case Setting.ArmSpeed:
                        label.SetText(_course ? "ARM SPEED  {0:1} rad/s · COURSE" : "ARM SPEED  {0:1} rad/s", _value);
                        break;
                    case Setting.AddedDelay: label.SetText(_course ? "ADDED DELAY  {0:0} ms · COURSE" : "ADDED DELAY  {0:0} ms", _value); break;
                    case Setting.NetworkLag: label.SetText(_course ? "LAG  {0:0} ms · COURSE" : "LAG  {0:0} ms", _value); break;
                    case Setting.NetworkJitter: label.SetText(_course ? "JITTER  {0:0} ms · COURSE" : "JITTER  {0:0} ms", _value); break;
                    default: label.SetText(_course ? "LOSS  {0:0} % · COURSE" : "LOSS  {0:0} %", _value); break;
                }
            }
        }
    }
}
