// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// One object's pose as it travels (docs/adr/0031 amendment 2): its scene id, flags, and its pose in the robot's root
    /// frame (metres; quaternion w, x, y, z), in float32 as the wires carry it.
    /// </summary>
    public readonly struct ObjectPose
    {
        /// <summary>Isaac moves it as a kinematic body (bit 0).</summary>
        public const byte FlagKinematic = 1;

        public readonly byte Id;
        public readonly byte Flags;
        public readonly float X;
        public readonly float Y;
        public readonly float Z;
        public readonly float Qw;
        public readonly float Qx;
        public readonly float Qy;
        public readonly float Qz;

        public ObjectPose(byte id, byte flags, float x, float y, float z, float qw, float qx, float qy, float qz)
        {
            Id = id;
            Flags = flags;
            X = x;
            Y = y;
            Z = z;
            Qw = qw;
            Qx = qx;
            Qy = qy;
            Qz = qz;
        }

        public static ObjectPose Of(byte id, byte flags, in Types.RigidTransform p) =>
            new ObjectPose(id, flags, (float)p.X, (float)p.Y, (float)p.Z, (float)p.Qw, (float)p.Qx, (float)p.Qy, (float)p.Qz);

        public Types.RigidTransform ToTransform() => new Types.RigidTransform(X, Y, Z, Qw, Qx, Qy, Qz);

        /// <summary>Whether the pose is finite with a unit quaternion (within 1e-3): what a placement must be.</summary>
        public bool IsValid()
        {
            float s = X + Y + Z + Qw + Qx + Qy + Qz;
            if (float.IsNaN(s) || float.IsInfinity(s)) return false;
            float n = Qw * Qw + Qx * Qx + Qy * Qy + Qz * Qz;
            return n > 0.998f && n < 1.002f;
        }

        public override string ToString() => $"{Id}:[{X:F4}, {Y:F4}, {Z:F4} | {Qw:F4}, {Qx:F4}, {Qy:F4}, {Qz:F4}]";
    }
}
