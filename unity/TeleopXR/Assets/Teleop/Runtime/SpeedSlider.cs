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
        [SerializeField] private JetRoverOperatorBridge bridge;
        [SerializeField] private XROrigin origin;
        [Tooltip("The slow end of the track (its top surface, centre line).")]
        [SerializeField] private Transform trackStart;
        [Tooltip("The fast end of the track.")]
        [SerializeField] private Transform trackEnd;
        [SerializeField] private Transform knob;
        [SerializeField] private TMP_Text label;
        [SerializeField] private float minRadPerSec = 0.5f;
        [SerializeField] private float maxRadPerSec = 6f;
        [SerializeField] private float startRadPerSec = 3.8f;
        [Tooltip("How far beside the track a fingertip still counts, metres.")]
        [SerializeField] private float touchRadius = 0.025f;

        private XRHandSubsystem _hands;
        private float _value;
        private int _shown = -1;

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

            if (_hands == null || !_hands.running)
            {
                _hands = FingertipPress.FindHands();
            }

            Transform space = origin.TrackablesParent != null ? origin.TrackablesParent : origin.transform;
            if (_hands != null && (TryTouch(_hands.rightHand, space, out float t) || TryTouch(_hands.leftHand, space, out t)))
            {
                // Tenths of a rad/s, so the value settles instead of trembling with the fingertip.
                _value = Mathf.Round(Mathf.Lerp(minRadPerSec, maxRadPerSec, t) * 10f) / 10f;
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
                bridge.SpeedRadPerSec = _value;
            }

            int shown = Mathf.RoundToInt(_value * 10f);
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
                label.SetText("ARM SPEED  {0:1} rad/s", _value);
            }
        }
    }
}
