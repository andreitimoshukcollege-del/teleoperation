using System.IO;
using System.Text.Json;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Kinematics
{
    /// <summary>
    /// Forward kinematics of the committed FR3 Duo profile (docs/adr/0026 decision 4), checked against
    /// two oracles that share none of its code. Both are written here with plain 4×4 matrices:
    /// <list type="bullet">
    /// <item>Franka's published Craig DH table for the FR3 arm, plus the Franka Hand's published flange-to-TCP
    /// offset (0.1034 m, rotated −45° about z);</item>
    /// <item>upstream's mount <c>kinematics.yaml</c> (<c>franka_description</c> @ 7aeeddc4,
    /// <c>accessories/fr3_duo_mount_v0_3</c>).</item>
    /// </list>
    /// </summary>
    public class SerialChainKinematicsTests
    {
        private const double Tolerance = 1e-9;

        // Franka's FR3 Craig DH table: per joint i, a(i-1), d(i), alpha(i-1). T(i) = RotX(alpha) TransX(a) RotZ(q) TransZ(d).
        private static readonly double[] DhA = { 0, 0, 0, 0.0825, -0.0825, 0, 0.088 };
        private static readonly double[] DhD = { 0.333, 0, 0.316, 0, 0.384, 0, 0 };
        private static readonly double[] DhAlpha = { 0, -Math.PI / 2, Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2, Math.PI / 2 };
        private const double FlangeD = 0.107;
        private const double HandTcpD = 0.1034;
        private const double HandYaw = -Math.PI / 4;

        internal static SerialChainProfile LoadFr3Duo()
        {
            string path = Path.Combine(RepoRoot(), "core", "RobotProfiles", "fr3-duo.json");
            var document = JsonSerializer.Deserialize<SerialChainProfileDocument>(
                File.ReadAllText(path), new JsonSerializerOptions { IncludeFields = true });
            Assert.NotNull(document);
            return document!.ToProfile();
        }

        [Fact]
        public void TheProfileIsTwoSevenJointArmsWithHandsAndTheAgreedMotorIds()
        {
            SerialChainProfile p = LoadFr3Duo();

            Assert.Equal("fr3-duo", p.Name);
            Assert.Equal(new[] { "left", "right" }, p.Arms.Select(a => a.Name));
            Assert.Equal(new byte[] { 1, 2, 3, 4, 5, 6, 7 }, p.Arms[0].Joints.Select(j => j.MotorId));
            Assert.Equal(new byte[] { 11, 12, 13, 14, 15, 16, 17 }, p.Arms[1].Joints.Select(j => j.MotorId));
            Assert.Equal(8, p.Arms[0].Gripper.MotorId);
            Assert.Equal(18, p.Arms[1].Gripper.MotorId);
            Assert.Equal(16, p.MotorCount);
            Assert.All(p.Arms, a => Assert.True(a.HasGripper && a.Gripper.SecondFingerMimics && a.Gripper.FingerTravel == 0.04));
            Assert.All(p.Arms.SelectMany(a => a.Joints), j => Assert.Equal(ChainJointType.Revolute, j.Type));

            Assert.True(p.TryFindMotor(13, out int arm, out int joint));
            Assert.Equal((1, 2), (arm, joint));
            Assert.True(p.TryFindMotor(8, out arm, out joint));
            Assert.Equal((0, -1), (arm, joint));
            Assert.False(p.TryFindMotor(9, out _, out _));
        }

        [Fact]
        public void EveryJointFrameAndTheToolMatchFrankasDhTable()
        {
            SerialChainProfile p = LoadFr3Duo();
            var rng = new SeededRng(20261028);
            Span<double> q = stackalloc double[7];
            Span<RigidTransform> frames = stackalloc RigidTransform[8];

            foreach (ChainArm arm in p.Arms)
            {
                RigidTransform toArmBase = arm.Mount.Inverse();
                for (int sample = 0; sample < 200; sample++)
                {
                    for (int j = 0; j < 7; j++)
                    {
                        ChainJointLimits l = arm.Joints[j].Limits;
                        q[j] = l.Lower + (l.Upper - l.Lower) * rng.NextDouble();
                    }

                    SerialChainKinematics.ComputeFrames(arm, q, frames);

                    double[,] dh = Matrix.Identity();
                    for (int j = 0; j < 7; j++)
                    {
                        dh = Matrix.Multiply(dh, Matrix.Multiply(Matrix.Multiply(Matrix.RotX(DhAlpha[j]), Matrix.Trans(DhA[j], 0, 0)),
                            Matrix.Multiply(Matrix.RotZ(q[j]), Matrix.Trans(0, 0, DhD[j]))));
                        AssertSame(dh, toArmBase.Compose(frames[j]), $"{arm.Name} joint {j + 1}, sample {sample}");
                    }

                    double[,] tcp = Matrix.Multiply(Matrix.Multiply(dh, Matrix.Trans(0, 0, FlangeD)),
                        Matrix.Multiply(Matrix.RotZ(HandYaw), Matrix.Trans(0, 0, HandTcpD)));
                    AssertSame(tcp, toArmBase.Compose(frames[7]), $"{arm.Name} tool, sample {sample}");
                    AssertSame(Matrix.Of(frames[7]), SerialChainKinematics.Tool(arm, q), $"{arm.Name} Tool(), sample {sample}");
                }
            }
        }

        [Fact]
        public void EachMountMatchesUpstreamsMountKinematics()
        {
            // fr3_duo_mount_v0_3/kinematics.yaml: the root link `base` is the mount's mounting point,
            // which is (-0.037, 0, -0.05) from the mount origin; the arms are at y = +-0.05018 with
            // rpy (-+0.89334809, -0.17456074, -+0.46334506) from the mount origin.
            SerialChainProfile p = LoadFr3Duo();
            double[,] origin = Matrix.Trans(0.037, 0, 0.05);

            double[,] left = Matrix.Multiply(origin, Matrix.Multiply(Matrix.Trans(0, 0.05018, 0), Matrix.Rpy(-0.89334809, -0.17456074, -0.46334506)));
            double[,] right = Matrix.Multiply(origin, Matrix.Multiply(Matrix.Trans(0, -0.05018, 0), Matrix.Rpy(0.89334809, -0.17456074, 0.46334506)));

            AssertSame(left, p.Arms[0].Mount, "left mount");
            AssertSame(right, p.Arms[1].Mount, "right mount");
        }

        [Fact]
        public void TheArmsAreMirrorImagesAcrossTheRobotsXzPlane()
        {
            // The mount mirrors the arms across y = 0 (y, roll and yaw negated). A mirrored
            // configuration negates every joint about an axis that the mirror reverses: joints 1, 3, 5
            // and 7. So the right arm at that configuration puts its TCP at the left TCP's mirror
            // image.
            SerialChainProfile p = LoadFr3Duo();
            var rng = new SeededRng(7);
            Span<double> qLeft = stackalloc double[7];
            Span<double> qRight = stackalloc double[7];

            for (int sample = 0; sample < 50; sample++)
            {
                for (int j = 0; j < 7; j++)
                {
                    ChainJointLimits l = p.Arms[0].Joints[j].Limits;
                    // Joint 4's and 6's ranges are not symmetric about 0; mirrored joints stay inside both arms' ranges.
                    double lo = j % 2 == 0 ? -Math.Min(-l.Lower, l.Upper) : l.Lower;
                    double hi = j % 2 == 0 ? Math.Min(-l.Lower, l.Upper) : l.Upper;
                    qLeft[j] = lo + (hi - lo) * rng.NextDouble();
                    qRight[j] = j % 2 == 0 ? -qLeft[j] : qLeft[j];
                }

                RigidTransform l7 = SerialChainKinematics.Tool(p.Arms[0], qLeft);
                RigidTransform r7 = SerialChainKinematics.Tool(p.Arms[1], qRight);
                Assert.True(Math.Abs(l7.X - r7.X) < Tolerance && Math.Abs(l7.Y + r7.Y) < Tolerance && Math.Abs(l7.Z - r7.Z) < Tolerance,
                    $"sample {sample}: left TCP {l7}, right TCP {r7}");
            }
        }

        [Fact]
        public void ForwardKinematicsAllocatesNothing()
        {
            ChainArm arm = LoadFr3Duo().Arms[0];
            double[] q = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
            var frames = new RigidTransform[SerialChainKinematics.FrameCount(arm)];
            double sink = 0;

            AllocationAssert.Zero(() =>
            {
                SerialChainKinematics.ComputeFrames(arm, q, frames);
                sink += SerialChainKinematics.Tool(arm, q).Z;
            });

            Assert.True(sink > 0);
        }

        [Fact]
        public void WrongSpanLengthsAreRefused()
        {
            ChainArm arm = LoadFr3Duo().Arms[0];
            Assert.Throws<ArgumentException>(() => SerialChainKinematics.ComputeFrames(arm, new double[6], new RigidTransform[8]));
            Assert.Throws<ArgumentException>(() => SerialChainKinematics.ComputeFrames(arm, new double[7], new RigidTransform[7]));
            Assert.Throws<ArgumentException>(() => SerialChainKinematics.Tool(arm, new double[8]));
        }

        [Fact]
        public void APrismaticJointSlidesAlongItsAxis()
        {
            var limits = new ChainJointLimits(0, 0.1, 0, 0.1, 0.2, 1, 10, 10);
            var arm = new ChainArm("slide", RigidTransform.Translation(1, 0, 0),
                new[] { new ChainJoint(1, "slide", ChainJointType.Prismatic, RigidTransform.FromXyzRpy(0, 0, 0.5, 0, 0, Math.PI / 2), 1, 0, 0, limits) },
                RigidTransform.Identity, null);

            RigidTransform t = SerialChainKinematics.Tool(arm, new[] { 0.05 });

            // The joint frame is yawed 90 degrees, so its +x is the root's +y.
            Assert.Equal(1.0, t.X, 12);
            Assert.Equal(0.05, t.Y, 12);
            Assert.Equal(0.5, t.Z, 12);
        }

        private static void AssertSame(double[,] expected, in RigidTransform actual, string what)
        {
            double[,] m = Matrix.Of(actual);
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    Assert.True(Math.Abs(expected[r, c] - m[r, c]) < Tolerance,
                        $"{what}: [{r},{c}] expected {expected[r, c]}, got {m[r, c]} ({actual})");
                }
            }
        }

        internal static string RepoRoot()
        {
            for (DirectoryInfo? dir = new DirectoryInfo(AppContext.BaseDirectory); dir != null; dir = dir.Parent)
            {
                if (Directory.Exists(Path.Combine(dir.FullName, "core", "RobotProfiles")) &&
                    Directory.Exists(Path.Combine(dir.FullName, "robot", "franka")))
                {
                    return dir.FullName;
                }
            }

            throw new InvalidOperationException($"no repo root (core/RobotProfiles and robot/franka) above {AppContext.BaseDirectory}");
        }

        /// <summary>The oracle's arithmetic: homogeneous 4×4 matrices, nothing shared with <see cref="RigidTransform"/>.</summary>
        private static class Matrix
        {
            public static double[,] Identity() => new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

            public static double[,] Trans(double x, double y, double z) => new double[,] { { 1, 0, 0, x }, { 0, 1, 0, y }, { 0, 0, 1, z }, { 0, 0, 0, 1 } };

            public static double[,] RotX(double a) => new double[,]
            {
                { 1, 0, 0, 0 }, { 0, Math.Cos(a), -Math.Sin(a), 0 }, { 0, Math.Sin(a), Math.Cos(a), 0 }, { 0, 0, 0, 1 },
            };

            public static double[,] RotY(double a) => new double[,]
            {
                { Math.Cos(a), 0, Math.Sin(a), 0 }, { 0, 1, 0, 0 }, { -Math.Sin(a), 0, Math.Cos(a), 0 }, { 0, 0, 0, 1 },
            };

            public static double[,] RotZ(double a) => new double[,]
            {
                { Math.Cos(a), -Math.Sin(a), 0, 0 }, { Math.Sin(a), Math.Cos(a), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 },
            };

            /// <summary>The URDF convention: Rz(yaw) · Ry(pitch) · Rx(roll).</summary>
            public static double[,] Rpy(double roll, double pitch, double yaw) => Multiply(RotZ(yaw), Multiply(RotY(pitch), RotX(roll)));

            public static double[,] Multiply(double[,] a, double[,] b)
            {
                var m = new double[4, 4];
                for (int r = 0; r < 4; r++)
                    for (int c = 0; c < 4; c++)
                        for (int k = 0; k < 4; k++)
                            m[r, c] += a[r, k] * b[k, c];
                return m;
            }

            /// <summary>A rigid transform as a matrix, from its quaternion by the textbook formula.</summary>
            public static double[,] Of(in RigidTransform t)
            {
                double w = t.Qw, x = t.Qx, y = t.Qy, z = t.Qz;
                return new double[,]
                {
                    { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), t.X },
                    { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), t.Y },
                    { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), t.Z },
                    { 0, 0, 0, 1 },
                };
            }
        }
    }
}
