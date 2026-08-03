using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Drives this Transform from the XR controller's pointer (aim) pose, via Input System actions
/// created inline with explicit binding paths -- not via the project's "XRI Default Input
/// Actions" asset (Assets/Samples/XR Interaction Toolkit/2.6.5/Starter Assets/), which fails to
/// import under the installed Input System version. Creating the actions in code with a literal
/// binding path sidesteps that broken asset file entirely: there is nothing here for it to fail
/// to parse.
///
/// Pointer pose, not device (grip) pose: Touch controllers report the two separately, tilted
/// relative to each other by design, and ray interactors are built expecting the pointer pose.
/// Feeding them the grip pose (an earlier version of this script did) is why the ray pointed
/// higher than it should have.
///
/// The binding path is relative to the tracking origin, not to whatever this GameObject's actual
/// Unity parent is. <see cref="origin"/> makes that explicit: the pose is computed relative to
/// that Transform and applied in world space, so this works correctly regardless of where in the
/// hierarchy the GameObject actually sits (e.g. under "Camera Offset", which already carries its
/// own offset -- assigning as local position/rotation would double-count it).
/// </summary>
public class XRNodePoseDriver : MonoBehaviour
{
    public enum Hand
    {
        Left,
        Right,
    }

    public Hand hand = Hand.Right;

    [Tooltip("Tracking-origin Transform (e.g. XR Origin) the pointer pose is relative to.")]
    public Transform origin;

    private InputAction _positionAction;
    private InputAction _rotationAction;

    private void OnEnable()
    {
        string usage = hand == Hand.Right ? "RightHand" : "LeftHand";
        _positionAction = new InputAction(binding: $"<XRController>{{{usage}}}/pointerPosition");
        _rotationAction = new InputAction(binding: $"<XRController>{{{usage}}}/pointerRotation");
        _positionAction.Enable();
        _rotationAction.Enable();
    }

    private void OnDisable()
    {
        _positionAction?.Disable();
        _rotationAction?.Disable();
    }

    private void OnDestroy()
    {
        _positionAction?.Dispose();
        _rotationAction?.Dispose();
    }

    private void Update()
    {
        if (origin == null)
        {
            return;
        }

        Vector3 position = _positionAction.ReadValue<Vector3>();
        if (IsFinite(position))
        {
            transform.position = origin.TransformPoint(position);
        }

        Quaternion rotation = _rotationAction.ReadValue<Quaternion>();
        if (IsValidRotation(rotation))
        {
            transform.rotation = origin.rotation * rotation;
        }
    }

    // ReadValue can return garbage (NaN, or a zero/degenerate quaternion) during device
    // connect/reconnect, before the runtime has a real pose to report yet. Skipping the
    // assignment on a bad frame is harmless -- the Transform just keeps last frame's value --
    // whereas writing a NaN pose sticks around and breaks every downstream consumer of this
    // Transform (the ray interactor's UI raycasting, in an earlier version of this bug).
    private static bool IsFinite(Vector3 v) =>
        !float.IsNaN(v.x) && !float.IsNaN(v.y) && !float.IsNaN(v.z) &&
        !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);

    private static bool IsValidRotation(Quaternion q) =>
        !float.IsNaN(q.x) && !float.IsNaN(q.y) && !float.IsNaN(q.z) && !float.IsNaN(q.w) &&
        (q.x != 0f || q.y != 0f || q.z != 0f || q.w != 0f);
}
