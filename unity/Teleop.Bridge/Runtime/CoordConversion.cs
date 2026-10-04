using UnityEngine;
using CoreVec = System.Numerics.Vector3;
using CoreQuat = System.Numerics.Quaternion;
using CorePose = Teleop.Core.Types.Pose;

namespace Teleop.Bridge
{
    /// <summary>
    /// The ONLY place in the repository where coordinate handedness is converted.
    ///
    /// Core  = ROS:   right-handed, Z-up,  X-forward, Y-left
    /// Unity = Unity: left-handed,  Y-up,  Z-forward, X-right
    ///
    /// A second conversion site produces bugs that look exactly like prediction error.
    /// If you need a conversion somewhere else, call into here instead.
    /// </summary>
    public static class CoordConversion
    {
        // ---- ROS -> Unity ----

        public static Vector3 ToUnity(this CoreVec v) => new Vector3(-v.Y, v.Z, v.X);

        public static Quaternion ToUnity(this CoreQuat q) =>
            new Quaternion(q.Y, -q.Z, -q.X, q.W);

        // ---- Unity -> ROS ----

        public static CoreVec ToCore(this Vector3 v) => new CoreVec(v.z, -v.x, v.y);

        public static CoreQuat ToCore(this Quaternion q) =>
            new CoreQuat(-q.z, q.x, -q.y, q.w);

        // ---- Pose helpers ----

        public static CorePose ToCorePose(this Transform t) =>
            new CorePose(t.position.ToCore(), t.rotation.ToCore());

        public static void ApplyTo(this CorePose p, Transform t) =>
            t.SetPositionAndRotation(p.Position.ToUnity(), p.Rotation.ToUnity());
    }
}
