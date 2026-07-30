using System.Numerics;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Types
{
    /// <summary>
    /// A rigid-body pose in the canonical Core convention: ROS, right-handed, Z-up,
    /// X-forward, metres. Uses System.Numerics so that both Unity and headless hosts can
    /// consume it. Conversion to UnityEngine types happens in exactly one place:
    /// unity/.../Bridge/CoordConversion.cs.
    /// </summary>
    public readonly struct Pose
    {
        public readonly Vector3 Position;
        public readonly Quaternion Rotation;

        public Pose(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public static Pose Identity => new Pose(Vector3.Zero, Quaternion.Identity);

        public override string ToString() =>
            $"Pose(pos=({Position.X:F3},{Position.Y:F3},{Position.Z:F3}), " +
            $"rot=({Rotation.X:F3},{Rotation.Y:F3},{Rotation.Z:F3},{Rotation.W:F3}))";
    }
}
