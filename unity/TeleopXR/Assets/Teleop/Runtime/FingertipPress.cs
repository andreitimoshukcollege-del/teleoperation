using System.Collections.Generic;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.Hands;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// A press-from-above detector for a level button cap, read from XR Hands fingertips (no XRI, so
    /// no ray or far pinch can press it). <see cref="Update"/> returns true once per press: a fingertip
    /// enters the cap's top within <c>radius</c>, the finger must then leave the cap's surroundings
    /// before the next press counts, and presses closer than <c>cooldown</c> are ignored. Shared by
    /// the robot's motion button and the twin's visibility switches.
    /// <para><b>A pointer click counts as a press</b> (docs/adr/0031 amendment 5): a hand pointer
    /// (<see cref="Fr3HandPointer"/>) that pinches with its ray on a cap calls <see cref="PointerClick"/>, and that cap's
    /// next <see cref="Update"/> returns true, under the same cooldown. Every cap a press watches is listed in
    /// <see cref="Caps"/>, so the pointer knows what it can click.</para>
    /// </summary>
    public sealed class FingertipPress
    {
        private static readonly List<XRHandSubsystem> s_Subsystems = new List<XRHandSubsystem>();
        private static readonly HashSet<Renderer> s_Caps = new HashSet<Renderer>();
        private static Renderer s_Clicked;
        private static int s_ClickedFrame = -10;

        /// <summary>Every cap a press has watched (destroyed ones read as null: skip them).</summary>
        public static IEnumerable<Renderer> Caps => s_Caps;

        /// <summary>A pointer clicked <paramref name="cap"/>: its press reports it at its next update (this frame or the next).</summary>
        public static void PointerClick(Renderer cap)
        {
            s_Clicked = cap;
            s_ClickedFrame = Time.frameCount;
        }
        private readonly float _radius, _depth, _rearm, _cooldown;
        private XRHandSubsystem _hands;
        private bool _armed = true;
        private float _lastPress = -10f;

        public FingertipPress(float radiusMeters, float pressDepthMeters, float rearmDistanceMeters, float cooldownSeconds)
        {
            _radius = radiusMeters;
            _depth = pressDepthMeters;
            _rearm = rearmDistanceMeters;
            _cooldown = cooldownSeconds;
        }

        public bool Update(Renderer cap, XROrigin origin)
        {
            if (cap == null || origin == null)
            {
                return false;
            }

            s_Caps.Add(cap);
            if (ReferenceEquals(s_Clicked, cap) && Time.frameCount - s_ClickedFrame <= 1)
            {
                s_Clicked = null;
                if (Time.unscaledTime - _lastPress > _cooldown)
                {
                    _lastPress = Time.unscaledTime;
                    return true;
                }
            }

            if (_hands == null || !_hands.running)
            {
                _hands = FindHands();
            }

            Transform space = origin.TrackablesParent != null ? origin.TrackablesParent : origin.transform;
            if (_hands == null)
            {
                return false;
            }

            bool pressing = false, near = false;
            Probe(_hands.leftHand, space, cap, ref pressing, ref near);
            Probe(_hands.rightHand, space, cap, ref pressing, ref near);

            if (pressing && _armed && Time.unscaledTime - _lastPress > _cooldown)
            {
                _lastPress = Time.unscaledTime;
                _armed = false;
                return true;
            }

            if (!near)
            {
                _armed = true;
            }

            return false;
        }

        private void Probe(XRHand hand, Transform space, Renderer cap, ref bool pressing, ref bool near)
        {
            if (!hand.isTracked || !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose tip))
            {
                return;
            }

            // The cap sits level: its top face is the top of its bounds.
            Vector3 world = space.TransformPoint(tip.position);
            Bounds bounds = cap.bounds;
            Vector3 offset = world - new Vector3(bounds.center.x, bounds.max.y, bounds.center.z);
            float radial = new Vector2(offset.x, offset.z).magnitude;
            if (radial < _radius && offset.y < 0.005f && offset.y > -_depth)
            {
                pressing = true;
            }

            if (offset.magnitude < _rearm)
            {
                near = true;
            }
        }

        internal static XRHandSubsystem FindHands()
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
