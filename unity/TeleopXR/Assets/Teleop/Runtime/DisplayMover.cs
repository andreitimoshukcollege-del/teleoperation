using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Lets the operator move the camera display (on its floor stand) by pointing at it and pinching,
    /// near or far. <see cref="HandClutchDriver"/> casts the hand's ray, and at the start of each pinch
    /// decides whether it grabs the display or engages the clutch; it only offers the display while
    /// robot motion is off, so moving it can never move the robot.
    ///
    /// <para>While held, the point that was grabbed stays at the same distance along the hand's ray,
    /// so sweeping the hand sweeps the display around you; it slides over the floor (its height stays)
    /// and keeps turning to face you.</para>
    /// </summary>
    public sealed class DisplayMover : MonoBehaviour
    {
        [Tooltip("The bar on top of the display: lit while the hand's ray is on the display.")]
        [SerializeField] private Renderer handle;
        [SerializeField] private Color idleColor = new Color(0.35f, 0.85f, 1f);
        [SerializeField] private Color hoverColor = new Color(1f, 0.85f, 0.25f);
        [SerializeField] private Color heldColor = new Color(0.3f, 1f, 0.45f);
        [Tooltip("How much bigger than the display itself the grab target is, metres.")]
        [SerializeField] private float grabMargin = 0.05f;

        private Renderer[] _renderers;
        private Vector3 _offset;
        private float _distance;
        private MaterialPropertyBlock _block;
        private int _shown = -1;

        public bool IsMoving { get; private set; }

        /// <summary>Set by the hand driver each frame: whether its ray is on the display.</summary>
        public bool Hovered { get; set; }

        private void Awake() => _renderers = GetComponentsInChildren<Renderer>(true);

        /// <summary>Where the ray first meets the display's bounds, if it does.</summary>
        public bool TryHit(Ray ray, out float distance)
        {
            distance = 0f;
            if (_renderers == null || _renderers.Length == 0)
            {
                return false;
            }

            Bounds bounds = _renderers[0].bounds;
            for (int i = 1; i < _renderers.Length; i++)
            {
                if (_renderers[i].enabled) bounds.Encapsulate(_renderers[i].bounds);
            }

            bounds.Expand(grabMargin * 2f);
            return bounds.IntersectRay(ray, out distance);
        }

        public void Begin(Ray ray, float distance)
        {
            _distance = distance;
            _offset = transform.position - ray.GetPoint(distance);
            IsMoving = true;
        }

        public void Drag(Ray ray)
        {
            if (!IsMoving)
            {
                return;
            }

            Vector3 target = ray.GetPoint(_distance) + _offset;
            transform.position = new Vector3(target.x, transform.position.y, target.z);

            Camera eye = Camera.main;
            if (eye != null)
            {
                // The display's forward points away from the viewer, as the lab builds it.
                Vector3 away = transform.position - eye.transform.position;
                away.y = 0f;
                if (away.sqrMagnitude > 1e-4f)
                {
                    transform.rotation = Quaternion.LookRotation(away.normalized, Vector3.up);
                }
            }
        }

        /// <summary>The grabbed point while held: where the ray is, at the distance it was grabbed at.</summary>
        public Vector3 GrabPoint(Ray ray) => ray.GetPoint(_distance);

        public void End() => IsMoving = false;

        private void LateUpdate()
        {
            int state = IsMoving ? 2 : Hovered ? 1 : 0;
            if (state == _shown || handle == null)
            {
                return;
            }

            _shown = state;
            Color color = state == 2 ? heldColor : state == 1 ? hoverColor : idleColor;
            _block ??= new MaterialPropertyBlock();
            handle.GetPropertyBlock(_block);
            _block.SetColor("_BaseColor", color);
            _block.SetColor("_Color", color);
            handle.SetPropertyBlock(_block);
        }
    }
}
