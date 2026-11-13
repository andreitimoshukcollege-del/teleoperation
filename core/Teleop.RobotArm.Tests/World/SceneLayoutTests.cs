using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>Seeded layouts (docs/adr/0031 amendment 2): repeatable, on the table, apart, and within each object's jitter.</summary>
    public class SceneLayoutTests
    {
        private static readonly WorldScene Scene = SceneDocumentTests.LoadShipped();

        private static RigidTransform[] Draw(ulong seed, out int nominal)
        {
            var poses = new RigidTransform[Scene.Objects.Length];
            nominal = SceneLayout.Draw(Scene, seed, poses);
            return poses;
        }

        private static double Yaw(in RigidTransform p) => 2 * Math.Atan2(p.Qz, p.Qw);

        [Fact]
        public void TheSameSeed_GivesTheSameLayout_AndAnotherSeedAnother()
        {
            RigidTransform[] a = Draw(7, out _), b = Draw(7, out _), c = Draw(8, out _);
            for (int i = 0; i < a.Length; i++)
            {
                Assert.Equal(a[i].ToString(), b[i].ToString());
            }

            Assert.Contains(Enumerable.Range(0, a.Length), i => Math.Abs(a[i].X - c[i].X) > 1e-6);
        }

        [Fact]
        public void ALayoutIsFrozen_SoASeedNamesTheSameLayoutOnEveryRuntime()
        {
            // SeededRng is pure integer arithmetic, so a seed names one layout everywhere. A change to the draw changes what
            // every recorded seed means: core/Scenes/testdata/layout-seed1.txt holds seed 1's, rewritten only on purpose
            // (`just gen-isaac-goldens`).
            RigidTransform[] p = Draw(1, out int nominal);
            Assert.Equal(0, nominal);
            string[] actual = p.Select(x => FormattableString.Invariant($"{x.X:F6} {x.Y:F6} {Yaw(x):F6}")).ToArray();
            string path = Path.Combine(Kinematics.SerialChainKinematicsTests.RepoRoot(), "core", "Scenes", "testdata", "layout-seed1.txt");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, string.Join("\n", actual) + "\n", new System.Text.UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            Assert.Equal(File.ReadAllText(path).Replace("\r\n", "\n").TrimEnd('\n').Split('\n'), actual);
        }

        [Fact]
        public void AThousandSeeds_KeepEveryObjectOnTheTable_ApartAndWithinItsJitter()
        {
            int fallbacks = 0;
            for (ulong seed = 0; seed < 1000; seed++)
            {
                RigidTransform[] p = Draw(seed, out int nominal);
                fallbacks += nominal;
                for (int i = 0; i < p.Length; i++)
                {
                    SceneObject o = Scene.Objects[i];
                    Assert.True(Scene.FitsOnTable(i, p[i].X, p[i].Y), $"seed {seed}: '{o.Name}' off the table");
                    Assert.InRange(Math.Abs(p[i].X - o.NominalX), 0, o.JitterX + 1e-12);
                    Assert.InRange(Math.Abs(p[i].Y - o.NominalY), 0, o.JitterY + 1e-12);
                    Assert.Equal(Scene.Table.Top + o.SizeZ / 2, p[i].Z, 12);
                    Assert.Equal(0, p[i].Qx, 12);
                    Assert.Equal(0, p[i].Qy, 12);
                    double turn = Math.Abs(Math.IEEERemainder(Yaw(p[i]) - o.NominalYaw, 2 * Math.PI));
                    Assert.InRange(turn, 0, o.JitterYaw + 1e-9);
                    for (int k = 0; k < i && nominal == 0; k++)
                    {
                        Assert.False(Scene.Overlaps(i, p[i].X, p[i].Y, k, p[k].X, p[k].Y), $"seed {seed}: '{o.Name}' overlaps '{Scene.Objects[k].Name}'");
                    }
                }
            }

            Assert.InRange(fallbacks, 0, 10); // of 4000 placements
        }

        [Fact]
        public void Drawing_AllocatesNothing()
        {
            var poses = new RigidTransform[Scene.Objects.Length];
            ulong s = 0;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() => SceneLayout.Draw(Scene, s++, poses), iterations: 1000);
        }

        [Fact]
        public void TooFewPoses_IsRefused()
        {
            Assert.Throws<ArgumentException>(() => SceneLayout.Draw(Scene, 1, new RigidTransform[1]));
        }
    }
}
