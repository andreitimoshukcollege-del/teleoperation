using System;

namespace Teleop.RobotArm.Types
{
    /// <summary>
    /// A rigid transform in double precision: a translation (metres) and a unit quaternion. It maps a
    /// point in a child frame into its parent's (<c>parent = Translation + Rotation · child</c>).
    ///
    /// <para><b>Double, not System.Numerics.</b> <c>Vector3</c>/<c>Quaternion</c> are single precision.
    /// A serial chain composes a dozen of these per arm, and the FR3's encoders resolve far below a
    /// single-precision step at a metre of reach (docs/adr/0026 decision 4).</para>
    /// </summary>
    public readonly struct RigidTransform
    {
        public readonly double X;
        public readonly double Y;
        public readonly double Z;
        public readonly double Qw;
        public readonly double Qx;
        public readonly double Qy;
        public readonly double Qz;

        public RigidTransform(double x, double y, double z, double qw, double qx, double qy, double qz)
        {
            X = x;
            Y = y;
            Z = z;
            Qw = qw;
            Qx = qx;
            Qy = qy;
            Qz = qz;
        }

        public static RigidTransform Identity => new RigidTransform(0, 0, 0, 1, 0, 0, 0);

        /// <summary>
        /// A URDF <c>&lt;origin xyz rpy&gt;</c>: the translation, then the fixed-axis roll about X, pitch
        /// about Y and yaw about Z (rotation matrix Rz(yaw) · Ry(pitch) · Rx(roll), as the URDF spec
        /// defines it).
        /// </summary>
        public static RigidTransform FromXyzRpy(double x, double y, double z, double roll, double pitch, double yaw)
        {
            double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
            double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
            double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);

            return new RigidTransform(
                x, y, z,
                cy * cp * cr + sy * sp * sr,
                cy * cp * sr - sy * sp * cr,
                cy * sp * cr + sy * cp * sr,
                sy * cp * cr - cy * sp * sr);
        }

        /// <summary>A rotation of <paramref name="angle"/> radians about the unit axis (<paramref name="ax"/>, <paramref name="ay"/>, <paramref name="az"/>), no translation.</summary>
        public static RigidTransform Rotation(double ax, double ay, double az, double angle)
        {
            double s = Math.Sin(angle * 0.5);
            return new RigidTransform(0, 0, 0, Math.Cos(angle * 0.5), ax * s, ay * s, az * s);
        }

        /// <summary>A translation, no rotation.</summary>
        public static RigidTransform Translation(double x, double y, double z) => new RigidTransform(x, y, z, 1, 0, 0, 0);

        /// <summary><c>this · child</c>: <paramref name="child"/>'s frame expressed in this transform's parent.</summary>
        public RigidTransform Compose(in RigidTransform child)
        {
            Rotate(child.X, child.Y, child.Z, out double rx, out double ry, out double rz);
            return new RigidTransform(
                X + rx, Y + ry, Z + rz,
                Qw * child.Qw - Qx * child.Qx - Qy * child.Qy - Qz * child.Qz,
                Qw * child.Qx + Qx * child.Qw + Qy * child.Qz - Qz * child.Qy,
                Qw * child.Qy - Qx * child.Qz + Qy * child.Qw + Qz * child.Qx,
                Qw * child.Qz + Qx * child.Qy - Qy * child.Qx + Qz * child.Qw);
        }

        /// <summary>The transform that undoes this one: <c>Inverse().Compose(this)</c> is the identity.</summary>
        public RigidTransform Inverse()
        {
            var conjugate = new RigidTransform(0, 0, 0, Qw, -Qx, -Qy, -Qz);
            conjugate.Rotate(X, Y, Z, out double x, out double y, out double z);
            return new RigidTransform(-x, -y, -z, Qw, -Qx, -Qy, -Qz);
        }

        /// <summary>Rotates the vector (<paramref name="vx"/>, <paramref name="vy"/>, <paramref name="vz"/>) by this transform's rotation only.</summary>
        public void Rotate(double vx, double vy, double vz, out double x, out double y, out double z)
        {
            // v + 2w(q × v) + 2 q × (q × v), q the vector part.
            double tx = 2 * (Qy * vz - Qz * vy);
            double ty = 2 * (Qz * vx - Qx * vz);
            double tz = 2 * (Qx * vy - Qy * vx);
            x = vx + Qw * tx + (Qy * tz - Qz * ty);
            y = vy + Qw * ty + (Qz * tx - Qx * tz);
            z = vz + Qw * tz + (Qx * ty - Qy * tx);
        }

        /// <summary>Maps the point (<paramref name="px"/>, <paramref name="py"/>, <paramref name="pz"/>) from the child frame into the parent.</summary>
        public void TransformPoint(double px, double py, double pz, out double x, out double y, out double z)
        {
            Rotate(px, py, pz, out x, out y, out z);
            x += X;
            y += Y;
            z += Z;
        }

        /// <summary>
        /// The pose a fraction <paramref name="t"/> (0..1) of the way from <paramref name="a"/> to <paramref name="b"/>: the position
        /// along the straight line, the rotation along the shorter arc (slerp).
        /// </summary>
        public static RigidTransform Interpolate(in RigidTransform a, in RigidTransform b, double t)
        {
            t = t < 0 ? 0 : t > 1 ? 1 : t;
            double bw = b.Qw, bx = b.Qx, by = b.Qy, bz = b.Qz;
            double dot = a.Qw * bw + a.Qx * bx + a.Qy * by + a.Qz * bz;
            if (dot < 0)
            {
                dot = -dot;
                bw = -bw; bx = -bx; by = -by; bz = -bz;
            }

            double wa, wb;
            if (dot > 0.9995)
            {
                wa = 1 - t;
                wb = t;
            }
            else
            {
                double theta = Math.Acos(dot), sin = Math.Sin(theta);
                wa = Math.Sin((1 - t) * theta) / sin;
                wb = Math.Sin(t * theta) / sin;
            }

            double qw = wa * a.Qw + wb * bw, qx = wa * a.Qx + wb * bx, qy = wa * a.Qy + wb * by, qz = wa * a.Qz + wb * bz;
            double n = Math.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            return new RigidTransform(a.X + t * (b.X - a.X), a.Y + t * (b.Y - a.Y), a.Z + t * (b.Z - a.Z), qw / n, qx / n, qy / n, qz / n);
        }

        public override string ToString() => $"[{X:F6}, {Y:F6}, {Z:F6} | {Qw:F6}, {Qx:F6}, {Qy:F6}, {Qz:F6}]";
    }
}
