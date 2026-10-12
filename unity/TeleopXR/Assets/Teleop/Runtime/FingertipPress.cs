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
    /// </summary>
    public sealed class FingertipPress
    {
        private static readonly List<XRHandSubsystem> s_Subsystems = new List<XRHandSubsystem>();
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
