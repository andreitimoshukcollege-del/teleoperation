using Teleop.Bridge;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.Hands;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Pointing from a distance while the robot is off (docs/adr/0031 amendment 5): each tracked hand casts a ray, from about
    /// its shoulder through the pinch point between its thumb and index, and a pinch is a click.
    /// <list type="bullet">
    /// <item><b>On a button</b> (any cap a <see cref="FingertipPress"/> watches: power, the console, the switches, RECORD, ...)
    /// the pinch presses it, exactly as a fingertip would.</item>
    /// <item><b>On a scene object</b> the pinch grabs it, and the ray drags it across the table; letting go asks RobotHost to put
    /// it there (<see cref="OperatorSideControls.UpdatePointer"/>). The twin shows its ghost meanwhile, and the solid object
    /// moves once the world frames say so. A fallen object is stood up by a click on it.</item>
    /// <item><b>On a camera display</b> the pinch grabs it, and it follows the ray at the distance it was grabbed at, up and
    /// down too, turning to face you (<see cref="DisplayMover"/>).</item>
    /// </list>
    /// <para><b>Only while the robot is off</b>, so a pinch never does two things: with the robot on, the hands drive the arms and
    /// the rays are hidden (the fingertips still press buttons). A ray whose hit is closer than <see cref="nearMeters"/> is
    /// hidden too: that close, the fingertip is the way to press.</para>
    /// </summary>
    public sealed class Fr3HandPointer : MonoBehaviour
    {
        [Tooltip("Either robot's bridge: the FR3 Duo's, or the JetRover's (docs/adr/0034 J6).")]
        [SerializeField] private OperatorBridgeBase bridge;
        [SerializeField] private XROrigin origin;
        [SerializeField] private Material rayMaterial;
        [Tooltip("The camera displays the pointer can move.")]
        [SerializeField] private DisplayMover[] displays = new DisplayMover[0];
        [SerializeField] private float maxMeters = 3f;
        [SerializeField] private float nearMeters = 0.15f;
        [Tooltip("Thumb tip to index tip: a pinch below this, released above releaseMeters.")]
        [SerializeField] private float pinchMeters = 0.02f;
        [SerializeField] private float releaseMeters = 0.035f;
        [Tooltip("How far beside a cap's edge the ray may pass and still click it: the console's smallest caps are 1.6 cm.")]
        [SerializeField] private float capSlackMeters = 0.02f;
        [SerializeField] private Color idleColor = new Color(1f, 1f, 1f, 0.35f);
        [SerializeField] private Color hoverColor = new Color(0.3f, 0.85f, 1f, 0.9f);
        [SerializeField] private Color activeColor = new Color(0.2f, 1f, 0.45f, 1f);

        private readonly LineRenderer[] _rays = new LineRenderer[2];
        private readonly Transform[] _dots = new Transform[2];
        private readonly bool[] _pinched = new bool[2];
        private readonly bool[] _onCap = new bool[2];
        private readonly DisplayMover[] _moving = new DisplayMover[2];
        private readonly bool[] _aiming = new bool[2];
        private readonly bool[] _holding = new bool[2];

        /// <summary>
        /// Whether a hand's pinch is this pointer's (0 left, 1 right): its ray is on a button, an object or a display, or a pinch
        /// it took is still held. The JetRover's clutch is the right hand's pinch too, so <c>HandClutchDriver</c> leaves such a
        /// pinch alone (docs/adr/0034 J6). As of the last frame: the two scripts' order is not fixed.
        /// </summary>
        public bool Owns(int hand) => hand >= 0 && hand < 2 && (_aiming[hand] || _holding[hand]);
        private XRHandSubsystem _hands;
        private MaterialPropertyBlock _block;

        private void Awake()
        {
            for (int h = 0; h < 2; h++)
            {
                var ray = new GameObject(h == 0 ? "LeftRay" : "RightRay");
                ray.transform.SetParent(transform, false);
                LineRenderer line = ray.AddComponent<LineRenderer>();
                line.positionCount = 2;
                line.useWorldSpace = true;
                line.widthMultiplier = 0.003f;
                line.numCapVertices = 2;
                line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                line.receiveShadows = false;
                if (rayMaterial != null) line.sharedMaterial = rayMaterial;
                line.enabled = false;
                _rays[h] = line;

                GameObject dot = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                dot.name = h == 0 ? "LeftDot" : "RightDot";
                Destroy(dot.GetComponent<Collider>());
                dot.transform.SetParent(transform, false);
                dot.transform.localScale = Vector3.one * 0.015f;
                if (rayMaterial != null) dot.GetComponent<Renderer>().sharedMaterial = rayMaterial;
                dot.SetActive(false);
                _dots[h] = dot.transform;
            }
        }

        private void Update()
        {
            OperatorSideControls side = bridge != null ? bridge.SideControls : null;
            bool active = side != null && !RobotOn && origin != null && Camera.main != null;
            if (_hands == null || !_hands.running) _hands = FingertipPress.FindHands();
            Transform space = origin != null && origin.TrackablesParent != null ? origin.TrackablesParent : origin != null ? origin.transform : null;
            for (int h = 0; h < 2; h++)
            {
                XRHand hand = _hands == null ? default : h == 0 ? _hands.leftHand : _hands.rightHand;
                Vector3 from = Vector3.zero, dir = Vector3.zero;
                float pinch = float.PositiveInfinity;
                bool tracked = active && _hands != null && space != null && TryRay(hand, h, space, out from, out dir, out pinch);

                bool was = _pinched[h];
                _pinched[h] = tracked && (was ? pinch < releaseMeters : pinch < pinchMeters);
                bool rising = _pinched[h] && !was;

                Renderer cap = null;
                DisplayMover display = null;
                float capAt = float.PositiveInfinity, objectAt = -1f, displayAt = float.PositiveInfinity;
                var ray = new Ray(from, dir);
                if (tracked)
                {
                    cap = NearestCap(from, dir, out capAt);
                    objectAt = side.PointerHit(from, dir);
                    display = NearestDisplay(ray, out displayAt);
                }

                // A display being moved keeps the pinch until it is let go.
                if (_moving[h] != null)
                {
                    if (tracked && _pinched[h])
                    {
                        _hovered[h] = _moving[h];
                        _moving[h].Drag(ray);
                        side.UpdatePointer(h, tracked, false, from, dir);
                        Show(h, true, from, dir, Vector3.Distance(from, _moving[h].GrabPoint(ray)), activeColor);
                        _aiming[h] = false;
                        _holding[h] = true;
                        continue;
                    }

                    _moving[h].End();
                    _moving[h] = null;
                }

                bool displayFirst = display != null && displayAt < capAt && (objectAt < 0f || displayAt < objectAt);
                bool objectFirst = !displayFirst && objectAt >= 0f && objectAt < capAt;
                float hitAt = displayFirst ? displayAt : objectFirst ? objectAt : cap != null ? capAt : -1f;
                bool near = hitAt >= 0f && hitAt < nearMeters;
                bool dragging = side != null && side.PointerDragging;
                _hovered[h] = displayFirst && !near ? display : null;
                if (rising && displayFirst && !near)
                {
                    display.Begin(ray, displayAt);
                    _moving[h] = display;
                    side.UpdatePointer(h, tracked, false, from, dir);
                    Show(h, true, from, dir, displayAt, activeColor);
                    _aiming[h] = false;
                    _holding[h] = true;
                    continue;
                }

                // A pinch that starts on a button clicks it, and is not also a drag for as long as it is held.
                if (rising) _onCap[h] = cap != null && !objectFirst && !displayFirst && !near;
                if (!_pinched[h]) _onCap[h] = false;
                if (rising && _onCap[h])
                {
                    FingertipPress.PointerClick(cap);
                    Debug.Log($"[pointer] {(h == 0 ? "left" : "right")} hand clicked {cap.transform.parent?.name ?? cap.name}");
                }

                bool wasDragging = side != null && side.PointerDragging;
                if (side != null) side.UpdatePointer(h, tracked && !near, _pinched[h] && !_onCap[h] && !displayFirst, from, dir);
                _aiming[h] = tracked && !near && hitAt >= 0f;
                if (rising) _holding[h] = _aiming[h];
                if (!_pinched[h]) _holding[h] = false;
                if (!wasDragging && side != null && side.PointerDragging) _holding[h] = true;
                Show(h, tracked && !near, from, dir, hitAt, _pinched[h] || (dragging && side.PointerDragging) ? activeColor : hitAt >= 0f ? hoverColor : idleColor);
            }
        }

        private bool TryRay(XRHand hand, int h, Transform space, out Vector3 from, out Vector3 dir, out float pinch)
        {
            from = dir = Vector3.zero;
            pinch = float.PositiveInfinity;
            if (!hand.isTracked ||
                !hand.GetJoint(XRHandJointID.IndexProximal).TryGetPose(out Pose indexBase) ||
                !hand.GetJoint(XRHandJointID.ThumbProximal).TryGetPose(out Pose thumbBase) ||
                !hand.GetJoint(XRHandJointID.IndexTip).TryGetPose(out Pose indexTip) ||
                !hand.GetJoint(XRHandJointID.ThumbTip).TryGetPose(out Pose thumbTip))
            {
                return false;
            }

            // From about the shoulder through the pinch point: steady while the fingers pinch, unlike the fingers' own direction.
            Transform head = Camera.main.transform;
            Vector3 right = Vector3.ProjectOnPlane(head.right, Vector3.up).normalized;
            Vector3 shoulder = head.position + (h == 0 ? -0.17f : 0.17f) * right + 0.2f * Vector3.down;
            from = space.TransformPoint(0.5f * (indexBase.position + thumbBase.position));
            dir = (from - shoulder).normalized;
            pinch = Vector3.Distance(space.TransformPoint(indexTip.position), space.TransformPoint(thumbTip.position));
            return dir.sqrMagnitude > 0.5f;
        }

        private void LateUpdate()
        {
            // Lit while a hand's ray is on it (or moving it); the pointer is off with the robot on.
            foreach (DisplayMover d in displays)
            {
                if (d != null) d.Hovered = false;
            }

            if (bridge == null || RobotOn) return;
            for (int h = 0; h < 2; h++)
            {
                if (_rays[h].enabled && _hovered[h] != null) _hovered[h].Hovered = true;
            }
        }

        private readonly DisplayMover[] _hovered = new DisplayMover[2];

        private bool RobotOn => bridge != null && bridge.MotionMode == JetRoverMotionMode.On;

        private DisplayMover NearestDisplay(Ray ray, out float distance)
        {
            distance = float.PositiveInfinity;
            DisplayMover best = null;
            foreach (DisplayMover d in displays)
            {
                if (d == null || !d.gameObject.activeInHierarchy || !d.TryHit(ray, out float at) || at <= 0f || at > maxMeters + 3f || at >= distance) continue;
                distance = at;
                best = d;
            }

            return best;
        }

        /// <summary>The nearest watched cap the ray passes over, through its level top, within its radius and the slack.</summary>
        private Renderer NearestCap(Vector3 from, Vector3 dir, out float distance)
        {
            distance = float.PositiveInfinity;
            Renderer best = null;
            foreach (Renderer cap in FingertipPress.Caps)
            {
                if (cap == null || !cap.gameObject.activeInHierarchy) continue;
                Bounds b = cap.bounds;
                if (Mathf.Abs(dir.y) < 1e-3f) continue;
                float t = (b.max.y - from.y) / dir.y;
                if (t <= 0f || t > maxMeters || t >= distance) continue;
                Vector3 p = from + dir * t;
                float radius = Mathf.Max(b.extents.x, b.extents.z) + capSlackMeters;
                if (new Vector2(p.x - b.center.x, p.z - b.center.z).sqrMagnitude > radius * radius) continue;
                distance = t;
                best = cap;
            }

            return best;
        }

        private void Show(int h, bool visible, Vector3 from, Vector3 dir, float hitAt, Color color)
        {
            LineRenderer line = _rays[h];
            line.enabled = visible;
            _dots[h].gameObject.SetActive(visible && hitAt >= 0f);
            if (!visible) return;
            Vector3 end = from + dir * (hitAt >= 0f ? hitAt : maxMeters);
            line.SetPosition(0, from);
            line.SetPosition(1, end);
            _dots[h].position = end;
            _block ??= new MaterialPropertyBlock();
            Tint(line, color);
            Tint(_dots[h].GetComponent<Renderer>(), color);
        }

        private void Tint(Renderer r, Color color)
        {
            // URP's materials ignore a line's vertex colours: the colour goes through the material's properties.
            r.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            r.SetPropertyBlock(_block);
        }
    }
}
