using UnityEngine;
using CoreVec = System.Numerics.Vector3;
using CoreQuat = System.Numerics.Quaternion;
using CorePose = Teleop.Core.Types.Pose;

namespace Teleop.Bridge
{
    /// <summary>
    /// Phase 1 gate only — DELETE once Gate 1 passes.
    ///
    /// Proves the whole chain works with a tiny payload:
    /// core/*.cs -> asmdef -> managed DLL -> IL2CPP -> C++ -> NDK -> APK -> Quest.
    /// If something in that chain is misconfigured you find out in a five-minute build
    /// rather than after three weeks of algorithm work.
    ///
    /// Attach to any GameObject in a scene, press Play, read the Console. Then build an
    /// APK, sideload it, and read `adb logcat -s Unity`.
    /// </summary>
    public sealed class SmokeTest : MonoBehaviour
    {
        void Start()
        {
            // 1. Construct a Core type. Proves Unity compiled the local UPM package.
            var pose = new CorePose(new CoreVec(1f, 2f, 3f), CoreQuat.Identity);
            Debug.Log($"[SmokeTest] Core type constructed: {pose}");

            // 2. Convert. Proves the Bridge assembly links against Core.
            Vector3 unityPos = pose.Position.ToUnity();
            Debug.Log($"[SmokeTest] ROS (1,2,3) -> Unity {unityPos}  (expect (-2.0, 3.0, 1.0))");

            // 3. Position round-trip. Proves the conversion is self-consistent.
            CoreVec posBack = unityPos.ToCore();
            bool posOk = Mathf.Abs(posBack.X - 1f) < 1e-5f
                      && Mathf.Abs(posBack.Y - 2f) < 1e-5f
                      && Mathf.Abs(posBack.Z - 3f) < 1e-5f;
            Debug.Log($"[SmokeTest] position round-trip: {(posOk ? "PASS" : "FAIL")} " +
                      $"({posBack.X:F3}, {posBack.Y:F3}, {posBack.Z:F3})");

            // 4. Rotation round-trip: 90 degrees yaw about the ROS Z (up) axis.
            //    Two quaternions represent the same rotation when |dot| == 1.
            CoreQuat yaw = CoreQuat.CreateFromAxisAngle(new CoreVec(0f, 0f, 1f), Mathf.PI / 2f);
            CoreQuat rotBack = yaw.ToUnity().ToCore();
            float dot = Mathf.Abs(yaw.X * rotBack.X + yaw.Y * rotBack.Y
                                + yaw.Z * rotBack.Z + yaw.W * rotBack.W);
            Debug.Log($"[SmokeTest] rotation round-trip: {(dot > 0.99999f ? "PASS" : "FAIL")} " +
                      $"|dot|={dot:F6}");
        }
    }
}
