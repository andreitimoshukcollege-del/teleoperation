using System.Collections.Generic;
using Teleop.Bridge;
using UnityEngine;
using UnityEngine.XR;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Turns the robot off when the headset comes off (the user's choice, 2026-10-27): ROBOT OFF, so the
    /// arm parks at its rest pose and relaxes (docs/adr/0023 decision 5). Putting the headset back on
    /// never turns it on again; that is the power button, so the arm cannot jump to wherever the hands
    /// happen to be.
    /// <list type="bullet">
    /// <item><b>The app paused or lost focus</b> (Android suspends it when the headset is taken off): at
    /// once, and the relax request goes out before the app stops (<see cref="OperatorBridgeBase.TurnOffNow"/>).
    /// Only in a build: the editor loses focus whenever another window is clicked.</item>
    /// <item><b>User presence</b> (the head device's <c>userPresence</c>, where the runtime reports it):
    /// after <see cref="absentSeconds"/> without the user, for a runtime that keeps the app running with
    /// the headset off.</item>
    /// </list>
    /// A safety setting: <c>just unity-check-xr</c> requires it present, wired and enabled.
    /// </summary>
    public sealed class HeadsetRemovalGuard : MonoBehaviour
    {
        [SerializeField] private OperatorBridgeBase bridge;
        [Tooltip("How long the headset must report no user before the robot is turned off, seconds.")]
        [SerializeField] private float absentSeconds = 0.5f;

        private readonly List<InputDevice> _heads = new List<InputDevice>();
        private float _absentFor;
        private bool _loggedPresence;

        private void Update()
        {
            if (bridge == null)
            {
                return;
            }

            if (TryReadPresence(out bool present))
            {
                if (!_loggedPresence)
                {
                    _loggedPresence = true;
                    Debug.Log("[xr] the headset reports user presence: taking it off turns the robot off");
                }

                _absentFor = present ? 0f : _absentFor + Time.unscaledDeltaTime;
                if (_absentFor >= absentSeconds)
                {
                    bridge.TurnOffNow("the headset was taken off");
                }
            }
        }

        private void OnApplicationPause(bool paused)
        {
            if (paused && !Application.isEditor && bridge != null)
            {
                bridge.TurnOffNow("the app was paused (headset taken off)");
            }
        }

        private void OnApplicationFocus(bool focused)
        {
            if (!focused && !Application.isEditor && bridge != null)
            {
                bridge.TurnOffNow("the app lost focus (headset taken off)");
            }
        }

        private bool TryReadPresence(out bool present)
        {
            present = true;
            _heads.Clear();
            InputDevices.GetDevicesAtXRNode(XRNode.Head, _heads);
            for (int i = 0; i < _heads.Count; i++)
            {
                if (_heads[i].isValid && _heads[i].TryGetFeatureValue(CommonUsages.userPresence, out present))
                {
                    return true;
                }
            }

            return false;
        }
    }
}
