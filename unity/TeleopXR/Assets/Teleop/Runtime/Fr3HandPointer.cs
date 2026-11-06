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
    /// it there (<see cref="SerialChainOperatorBridge.UpdatePointer"/>). The twin shows its ghost meanwhile, and the solid object
    /// moves once the world frames say so. A fallen object is stood up by a click on it.</item>
    /// </list>
    /// <para><b>Only while the robot is off</b>, so a pinch never does two things: with the robot on, the hands drive the arms and
    /// the rays are hidden (the fingertips still press buttons). A ray whose hit is closer than <see cref="nearMeters"/> is
    /// hidden too: that close, the fingertip is the way to press.</para>
    /// </summary>
    public sealed class Fr3HandPointer : MonoBehaviour
    {
        [SerializeField] private SerialChainOperatorBridge bridge;
        [SerializeField] private XROrigin origin;
        [SerializeField] private Material rayMaterial;
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
            bool active = bridge != null && bridge.IsReady && !bridge.MotionEnabled && origin != null && Camera.main != null;
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
                float capAt = float.PositiveInfinity, objectAt = -1f;
                if (tracked)
                {
                    cap = NearestCap(from, dir, out capAt);
                    objectAt = bridge.PointerHit(from, dir);
                }

                bool objectFirst = objectAt >= 0f && objectAt < capAt;
                float hitAt = objectFirst ? objectAt : cap != null ? capAt : -1f;
                bool near = hitAt >= 0f && hitAt < nearMeters;
                bool dragging = bridge != null && bridge.PointerDragging;

                // A pinch that starts on a button clicks it, and is not also a drag for as long as it is held.
                if (rising) _onCap[h] = cap != null && !objectFirst && !near;
                if (!_pinched[h]) _onCap[h] = false;
                if (rising && _onCap[h])
                {
                    FingertipPress.PointerClick(cap);
                    Debug.Log($"[pointer] {(h == 0 ? "left" : "right")} hand clicked {cap.transform.parent?.name ?? cap.name}");
                }

                if (bridge != null) bridge.UpdatePointer(h, tracked && !near, _pinched[h] && !_onCap[h], from, dir);
                Show(h, tracked && !near, from, dir, hitAt, _pinched[h] || (dragging && bridge.PointerDragging) ? activeColor : hitAt >= 0f ? hoverColor : idleColor);
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
