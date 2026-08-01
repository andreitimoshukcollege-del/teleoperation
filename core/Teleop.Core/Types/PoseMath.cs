using System;
using System.Numerics;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// Shared pose-error definitions, used by every reconciler's correction-magnitude metric and
    /// by any offline predictor scorer's position/orientation error (docs/metrics.md §4, §5).
    /// Written once, here, specifically so those two never silently disagree on what "error"
    /// means -- both need the identical definition, not two implementations that happen to
    /// usually produce close numbers.
    ///
    /// Units are metres and radians (ROS convention, matching <see cref="Pose"/>); the
    /// millimetres and degrees docs/metrics.md reports are a reporting-time conversion, not
    /// something this type does.
    /// </summary>
    public static class PoseMath
    {
        /// <summary>Euclidean distance between two positions, metres. Allocation-free.</summary>
        public static float PositionErrorMeters(in Pose a, in Pose b) =>
            Vector3.Distance(a.Position, b.Position);

        /// <summary>
        /// Geodesic angle between two orientations, radians, via <c>2 * acos(|dot(q1, q2)|)</c>.
        /// The absolute value of the dot product handles quaternion double-cover: <c>q</c> and
        /// <c>-q</c> represent the identical rotation, and without it two bit-different
        /// quaternions describing the same orientation would report a large spurious error. The
        /// dot product is clamped to <c>[-1, 1]</c> before <c>acos</c> because floating-point
        /// rounding can push it fractionally outside that domain even for exactly-equal
        /// orientations, which would otherwise produce <c>NaN</c>. Allocation-free.
        /// </summary>
        public static float OrientationErrorRadians(in Pose a, in Pose b)
        {
            float dot = Quaternion.Dot(a.Rotation, b.Rotation);
            float clampedAbsDot = Math.Min(Math.Abs(dot), 1f);
            return 2f * MathF.Acos(clampedAbsDot);
        }
    }
}
