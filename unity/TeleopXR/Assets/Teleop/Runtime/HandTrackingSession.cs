using UnityEngine;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.XR
{
    /// <summary>
    /// Two jobs at session start, both once:
    /// <list type="bullet">
    /// <item><b>Asks for hand tracking.</b> Android XR gates hand data behind a runtime permission.
    /// The Unity Android XR package adds the manifest entry when its hand-tracking feature is
    /// enabled; the user still has to grant it. Until they do, the hands simply do not appear and
    /// nothing can grab the target, so the robot cannot be commanded at all.</item>
    /// <item><b>Records the input modality.</b> Hand tracking reaches the robot through the same IK and
    /// command stream as controller motion, but with its own jitter and dropouts, so a session's
    /// numbers depend on it (docs/adr/0016 §5). One log line states it, and that no smoothing is
    /// applied to the pinch.</item>
    /// </list>
    /// No Bridge or Core reference: this is device plumbing, not teleoperation logic.
    /// </summary>
    public sealed class HandTrackingSession : MonoBehaviour
    {
        /// <summary>Android XR's runtime permission for hand-tracking data.</summary>
        public const string HandTrackingPermission = "android.permission.HAND_TRACKING";

        private void Start()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(HandTrackingPermission))
            {
                UnityEngine.Android.Permission.RequestUserPermission(HandTrackingPermission);
            }
#endif
            Debug.Log("[xr] input modality: hands (pinch-grab, near and far); hand smoothing off, target smoothing off");
        }
    }
}
