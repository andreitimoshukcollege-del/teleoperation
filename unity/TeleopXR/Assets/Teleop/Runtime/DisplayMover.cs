using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Lets the operator move the camera display (on its floor stand): pinch the bar on top of it with
    /// the clutch hand and walk it. <see cref="HandClutchDriver"/> decides at the start of each pinch
    /// whether it grabs this bar or engages the clutch, so moving the display never moves the robot.
    /// The display slides over the floor (its height stays) and keeps turning to face you.
    /// </summary>
    public sealed class DisplayMover : MonoBehaviour
    {
        [Tooltip("The bar on top of the display that is grabbed.")]
        [SerializeField] private Renderer handle;
        [Tooltip("How far from the bar a pinch still grabs it, metres.")]
        [SerializeField] private float grabMargin = 0.05f;

        private Vector3 _rootAtGrab, _handAtGrab;

        public bool IsMoving { get; private set; }

        public bool Contains(Vector3 point)
        {
            if (handle == null)
            {
                return false;
            }

            Bounds bounds = handle.bounds;
            bounds.Expand(grabMargin * 2f);
            return bounds.Contains(point);
        }

        public void Begin(Vector3 hand)
        {
            _rootAtGrab = transform.position;
            _handAtGrab = hand;
            IsMoving = true;
        }

        public void Drag(Vector3 hand)
        {
            if (!IsMoving)
            {
                return;
            }

            Vector3 delta = hand - _handAtGrab;
            delta.y = 0f;
            transform.position = _rootAtGrab + delta;

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

        public void End() => IsMoving = false;
    }
}
