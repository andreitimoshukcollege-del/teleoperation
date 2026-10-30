using System.IO;
using System.Text.Json;
using Teleop.Core.Tests.TestSupport;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Tests.Kinematics
{
    /// <summary>The FR3 Duo's inverse kinematics and collision model (docs/adr/0026 amendment).</summary>
    public class SerialChainSolverTests
    {
        private static readonly double[] Ready = { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4 };

        internal static SerialChainCollisionModel Collision()
        {
            string path = Path.Combine(SerialChainKinematicsTests.RepoRoot(), "core", "RobotProfiles", "fr3-duo.collision.json");
            var doc = JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(path), new JsonSerializerOptions { IncludeFields = true })!;
            return doc.ToModel();
        }

        private static double[] RandomPose(ref SeededRng rng, ChainArm arm, double[] around, double spread)
        {
            var q = new double[arm.Joints.Length];
            for (int j = 0; j < q.Length; j++)
            {
                ChainJointLimits l = arm.Joints[j].Limits;
                double v = around[j] + spread * (2 * rng.NextDouble() - 1);
                q[j] = Math.Min(Math.Max(v, l.SoftLower + 0.05), l.SoftUpper - 0.05);
            }

            return q;
        }

        [Fact]
        public void TargetsNearWhereTheArmIs_AreReachedToAMillimetreAndAFewTenthsOfADegree()
        {
            // A teleoperation step: the hand moves the target a few centimetres from where the arm is.
            ChainArm arm = SerialChainKinematicsTests.LoadFr3Duo().Arms[0];
            var solver = new SerialChainSolver(arm);
            var rng = new SeededRng(11);
            var solution = new double[7];
            int converged = 0, trials = 300;
            for (int k = 0; k < trials; k++)
            {
                double[] start = RandomPose(ref rng, arm, Ready, 0.8);
                double[] goal = RandomPose(ref rng, arm, start, 0.15);
                RigidTransform target = SerialChainKinematics.Tool(arm, goal);
                ChainSolveResult r = solver.Solve(target, start, solution);
                RigidTransform reached = SerialChainKinematics.Tool(arm, solution);
                if (r.Converged)
                {
                    converged++;
                    double d = Math.Sqrt(Sq(reached.X - target.X) + Sq(reached.Y - target.Y) + Sq(reached.Z - target.Z));
                    Assert.True(d <= 1.1e-4, $"trial {k}: converged but {d} m off");
                }
            }

            Assert.True(converged >= trials * 0.97, $"{converged} of {trials} converged");
        }

        [Fact]
        public void FarTargets_FromTheReadyPose_MostlyConverge_AndNeverLeaveTheLimits()
        {
            ChainArm arm = SerialChainKinematicsTests.LoadFr3Duo().Arms[1];
            var solver = new SerialChainSolver(arm, new SerialChainSolverOptions { Iterations = 150 });
            var rng = new SeededRng(12);
            var solution = new double[7];
            double[] ready = Ready.Select((q, j) => j % 2 == 0 ? -q : q).ToArray();
            int converged = 0, trials = 200;
            for (int k = 0; k < trials; k++)
            {
                double[] goal = RandomPose(ref rng, arm, ready, 0.9);
                ChainSolveResult r = solver.Solve(SerialChainKinematics.Tool(arm, goal), ready, solution);
                if (r.Converged) converged++;
                for (int j = 0; j < 7; j++)
                {
                    ChainJointLimits l = arm.Joints[j].Limits;
                    Assert.InRange(solution[j], l.SoftLower, l.SoftUpper);
                }
            }

            Assert.True(converged >= trials * 0.95, $"{converged} of {trials} converged");
        }

        [Fact]
        public void TheArmsCurrentPose_IsItsOwnSolution_TheNullSpaceDoesNotDrift()
        {
            ChainArm arm = SerialChainKinematicsTests.LoadFr3Duo().Arms[0];
            var solver = new SerialChainSolver(arm);
            var solution = new double[7];
            ChainSolveResult r = solver.Solve(SerialChainKinematics.Tool(arm, Ready), Ready, solution);
            Assert.True(r.Converged);
            Assert.Equal(0, r.Iterations);
            for (int j = 0; j < 7; j++) Assert.Equal(Ready[j], solution[j], 12);
        }

        [Fact]
        public void AnUnreachableTarget_ReportsItAndStaysInsideTheLimits()
        {
            ChainArm arm = SerialChainKinematicsTests.LoadFr3Duo().Arms[0];
            var solver = new SerialChainSolver(arm);
            var solution = new double[7];
            ChainSolveResult r = solver.Solve(RigidTransform.Translation(2.0, 0, 0.5), Ready, solution);
            Assert.False(r.Converged);
            Assert.True(r.PositionError > 0.5 && double.IsFinite(r.PositionError));
            for (int j = 0; j < 7; j++) Assert.InRange(solution[j], arm.Joints[j].Limits.SoftLower, arm.Joints[j].Limits.SoftUpper);
        }

        [Fact]
        public void SolvingIsDeterministic_AndAllocatesNothing()
        {
            ChainArm arm = SerialChainKinematicsTests.LoadFr3Duo().Arms[0];
            var solver = new SerialChainSolver(arm);
            double[] goal = Ready.Select((q, j) => q + 0.1 * (j % 3 - 1)).ToArray();
            RigidTransform target = SerialChainKinematics.Tool(arm, goal);
            var a = new double[7];
            var b = new double[7];
            solver.Solve(target, Ready, a);
            solver.Solve(target, Ready, b);
            Assert.Equal(a.Select(BitConverter.DoubleToInt64Bits), b.Select(BitConverter.DoubleToInt64Bits));
            AllocationAssert.Zero(() => solver.Solve(target, Ready, a), iterations: 500);
        }

        [Fact]
        public void TheReadyPose_IsCollisionFree()
        {
            SerialChainProfile p = SerialChainKinematicsTests.LoadFr3Duo();
            double[] pose = Ready.Concat(Ready.Select((q, j) => j % 2 == 0 ? -q : q)).ToArray();
            ChainCollisionHit hit = SerialChainCollision.Check(p, Collision(), pose);
            Assert.False(hit.IsHit, hit.IsHit ? Describe(hit) : "");
        }

        [Fact]
        public void ArmsSwungIntoEachOther_Collide()
        {
            // Each arm's first joint turned toward the other arm, shoulders forward: the forearms meet between the bases.
            SerialChainProfile p = SerialChainKinematicsTests.LoadFr3Duo();
            SerialChainCollisionModel model = Collision();
            bool anyArmToArm = false;
            foreach (double swing in new[] { 0.8, 1.0, 1.2, 1.4 })
            {
                double[] left = { -swing, 0.6, 0, -1.6, 0, 2.0, 0.8 };
                double[] right = { swing, 0.6, 0, -1.6, 0, 2.0, -0.8 };
                ChainCollisionHit hit = SerialChainCollision.Check(p, model, left.Concat(right).ToArray());
                if (hit.IsHit && hit.CapsuleA >= 0 && model.Capsules[hit.CapsuleA].Arm != model.Capsules[hit.CapsuleB].Arm &&
                    model.Capsules[hit.CapsuleA].Arm >= 0 && model.Capsules[hit.CapsuleB].Arm >= 0)
                {
                    anyArmToArm = true;
                }
            }

            Assert.True(anyArmToArm, "no swing of the arms toward each other was caught as arm-to-arm");
        }

        [Fact]
        public void AJointOutsideItsSoftRange_IsTheHit()
        {
            SerialChainProfile p = SerialChainKinematicsTests.LoadFr3Duo();
            double[] pose = Ready.Concat(Ready.Select((q, j) => j % 2 == 0 ? -q : q)).ToArray();
            pose[3] = 0.5; // joint 4's range is entirely below zero
            ChainCollisionHit hit = SerialChainCollision.Check(p, Collision(), pose);
            Assert.True(hit.IsHit);
            Assert.Equal((0, 3), (hit.Arm, hit.Joint));
        }

        [Fact]
        public void CheckingAllocatesNothing()
        {
            SerialChainProfile p = SerialChainKinematicsTests.LoadFr3Duo();
            SerialChainCollisionModel model = Collision();
            double[] pose = Ready.Concat(Ready.Select((q, j) => j % 2 == 0 ? -q : q)).ToArray();
            AllocationAssert.Zero(() => SerialChainCollision.Check(p, model, pose), iterations: 500);
        }

        [Fact]
        public void SegmentDistance_MatchesKnownCases()
        {
            Assert.Equal(1.0, Math.Sqrt(SerialChainCollision.SegmentDistanceSquared(new double[] { 0, 0, 0, 1, 0, 0 }, new double[] { 0.5, 1, 0, 0.5, 2, 0 })), 12);
            Assert.Equal(0.0, SerialChainCollision.SegmentDistanceSquared(new double[] { -1, 0, 0, 1, 0, 0 }, new double[] { 0, -1, 0, 0, 1, 0 }), 12);
            Assert.Equal(Math.Sqrt(2), Math.Sqrt(SerialChainCollision.SegmentDistanceSquared(new double[] { 0, 0, 0, 0, 0, 0 }, new double[] { 1, 1, 0, 1, 1, 0 })), 12);
        }

        private static string Describe(ChainCollisionHit hit)
        {
            SerialChainCollisionModel m = Collision();
            return hit.CapsuleA >= 0 ? $"{m.Capsules[hit.CapsuleA].Name} / {m.Capsules[hit.CapsuleB].Name} at {hit.Distance} m" : $"arm {hit.Arm} joint {hit.Joint}";
        }

        private static double Sq(double x) => x * x;
    }
}
