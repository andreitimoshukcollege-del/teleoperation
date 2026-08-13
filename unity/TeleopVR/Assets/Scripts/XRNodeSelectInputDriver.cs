using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;

/// <summary>
/// Feeds a controller's Select input via an Input System action created inline with an explicit
/// binding path -- the same workaround <see cref="XRNodePoseDriver"/> already uses for
/// Position/Rotation, applied to Select instead, since the project's "XRI Default Input Actions"
/// asset fails to import under the installed Input System version (Teleop/CLAUDE.md's
/// "Known-broken" section) and that asset is what <c>XR Controller (Action-based)</c>'s Select
/// Action Value field would otherwise need to bind to.
///
/// Needed for the JetRover VR drag feature (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md):
/// without a working Select input, an <c>XRGrabInteractable</c> can never actually be grabbed --
/// silently, with no console error, since the interactor's controller simply never reports select
/// as active.
///
/// <b>Targets <see cref="ActionBasedController.selectAction"/>, not <c>selectActionValue</c></b> --
/// confirmed against the installed XR Interaction Toolkit 2.6.5 package source (an earlier version
/// of this doc comment flagged this as unverified and guessed wrong): <c>ActionBasedController</c>'s
/// <c>UpdateInput</c> determines whether select is actually pressed via
/// <c>IsPressed(m_SelectAction.action)</c> -- <c>selectActionValue</c> only supplies the
/// continuous magnitude for an already-pressed selection, and its own doc comment says Unity
/// falls back to <c>selectAction</c> when it's unset, never the other way around. Driving only
/// <c>selectActionValue</c> (this script's original approach) left <c>selectAction</c> at its
/// default, unbound action, so <c>IsPressed</c> was always false and a grab could never actually
/// trigger -- silently, with no console error, exactly the failure mode this class's own doc
/// above already warned about, just from the opposite field.
/// </summary>
public class XRNodeSelectInputDriver : MonoBehaviour
{
    public enum Hand
    {
        Left,
        Right,
    }

    public Hand hand = Hand.Right;

    [Tooltip("The ActionBasedController whose Select Action this feeds -- the same component whose Position/Rotation Action fields XRNodePoseDriver works around instead of using.")]
    public ActionBasedController controller;

    private InputAction _selectAction;

    private void OnEnable()
    {
        string usage = hand == Hand.Right ? "RightHand" : "LeftHand";
        // Explicit Button type, not the constructor's default Value type: a Value action's phase
        // only pulses to Performed on the frame the control's value changes, reverting to Started
        // while held steady with no further change -- ActionBasedController.IsPressed checks
        // exactly `phase == Performed`, so a Value-typed action would make a held trigger register
        // as "pressed" for one frame only. Button type's default Press interaction keeps phase at
        // Performed for the entire duration the control is actuated, which is what a sustained
        // grab-and-hold needs.
        _selectAction = new InputAction(
            binding: $"<XRController>{{{usage}}}/triggerPressed", type: InputActionType.Button);
        _selectAction.Enable();

        if (controller != null)
        {
            controller.selectAction = new InputActionProperty(_selectAction);
        }
    }

    private void OnDisable()
    {
        _selectAction?.Disable();
    }

    private void OnDestroy()
    {
        _selectAction?.Dispose();
    }
}
