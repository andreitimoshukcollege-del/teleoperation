using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using Teleop.Core.Tests.TestSupport;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Tests.Collision;
using Teleop.RobotArm.Types;
using Xunit.Abstractions;
using static Teleop.RobotArm.Tests.Collision.TestArm;

namespace Teleop.RobotArm.Tests.Kinematics
{
    public class ArmSolverTests
    {
        private static readonly ArmCollisionModel Model = TestArm.Model();
        private static readonly RobotArmProfile Arm = Profile(Model);
        private readonly ITestOutputHelper _output;

        public ArmSolverTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void FreeTarget_IsReachedExactly_WithTheClosedFormSolution()
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            var target = new Vector3(0.15f, 0.03f, 0.10f);
            ArmSolution s = solver.Solve(Arm, target, 0f, 0f);

            Assert.Equal(ArmSolveStatus.Exact, s.Status);
            Vector3 reached = ArmKinematics.Forward(Arm, s.Pose.BaseYaw, s.Pose.Proximal, s.Pose.Distal);
            Assert.True(Vector3.Distance(target, reached) < 1e-4f, $"{reached}");
            Assert.Equal(0f, s.Pose.Proximal + s.Pose.Distal + s.Pose.Wrist, 4);
        }

        [Fact]
        public void TargetInsideTheDeck_StopsAtAValidPoseShortOfIt()
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            ArmSolution start = solver.Solve(Arm, new Vector3(0.15f, 0f, 0.10f), -0.5f, 0f);
            Assert.NotEqual(ArmSolveStatus.Blocked, start.Status);

            ArmSolution s = solver.Solve(Arm, new Vector3(0.13f, 0f, -0.06f), -0.5f, 0f);

            Assert.Equal(ArmSolveStatus.Blocked, s.Status);
            Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose));
            Assert.True(s.Position.Z > TestArm.Deck.Max.Z, $"stopped at {s.Position}, inside the deck");
            Assert.True(s.Position.Z < start.Position.Z, "it should still have moved toward the target");
        }

        [Fact]
        public void TargetReachableOnlyTilted_IsReachedWithTheTilt()
        {
            // Gripper straight down over the deck would push its 9 cm into the deck top. The target is
            // 14 cm from the shoulder: the elbow's ±120° keeps the arm from anything nearer than 13.
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            var target = new Vector3(0.14f, 0f, 0.07f);
            ArmSolution s = solver.Solve(Arm, target, -90f * Deg, 0f);

            Assert.Equal(ArmSolveStatus.PitchAdjusted, s.Status);
            Assert.True(Vector3.Distance(target, s.Position) < 1e-5f);
            Assert.True(s.Pitch > -90f * Deg + 1e-3f, $"pitch {s.Pitch}");
            Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose));
        }

        [Fact]
        public void TargetBehindTheRobot_StopsWithoutSwinging()
        {
            // Behind the robot is out of reach without swinging the base half round or flipping the
            // elbow (the shoulder's and wrist's ±120 degrees rule out the rest). Neither may happen
            // on its own: the arm stops, valid, with the base and the elbow where they were.
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            ArmSolution s = default;
            for (int i = 0; i < 40; i++)
            {
                s = solver.Solve(Arm, new Vector3(-0.06f, 0.0f, 0.20f), 0.5f, 0f);
            }

            Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose));
            Assert.True(MathF.Abs(s.Pose.BaseYaw) < 30f * Deg, $"the base swung to {s.Pose.BaseYaw / Deg:0} deg");
            Assert.True(s.Pose.Distal < 0f, "the elbow flipped");
        }

        [Fact]
        public void EveryResult_IsValid_ForRandomTargetsAndPaths()
        {
            var random = new Random(42);
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            var stopwatch = new Stopwatch();
            var times = new double[10_000];
            int blocked = 0;
            var target = new Vector3(0.15f, 0f, 0.08f);
            for (int i = 0; i < 10_000; i++)
            {
                // Mostly a random walk (a hand moving), sometimes a jump anywhere.
                target = i % 50 == 0
                    ? new Vector3(Next(random, 0.3f), Next(random, 0.3f), Next(random, 0.3f))
                    : target + new Vector3(Next(random, 0.02f), Next(random, 0.02f), Next(random, 0.02f));
                stopwatch.Restart();
                ArmSolution s = solver.Solve(Arm, target, Next(random, 1.5f), Next(random, 1.5f));
                times[i] = stopwatch.Elapsed.TotalMilliseconds;
                if (s.Status == ArmSolveStatus.Blocked) blocked++;
                Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose),
                    $"step {i}: {ArmCollision.Check(Arm, Model, s.Pose).Describe(Model)}");
            }

            // Timing is logged, never asserted (a shared CI box is no benchmark). The first 100
            // solves are left out: they include the JIT.
            double[] warm = times[100..];
            Array.Sort(warm);
            _output.WriteLine($"solve: mean {warm.Average():0.000} ms, p99 {warm[(int)(warm.Length * 0.99)]:0.000} ms, " +
                $"max {warm[^1]:0.000} ms; {blocked} of 10000 blocked");
        }

        [Fact]
        public void SmallTargetSteps_NeverFlipTheBranch()
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            ArmSolution previous = solver.Solve(Arm, new Vector3(0.15f, 0f, 0.10f), 0f, 0f);
            for (int i = 1; i <= 400; i++)
            {
                float angle = i * 0.01f;
                var target = new Vector3(0.15f * MathF.Cos(angle), 0.15f * MathF.Sin(angle), 0.10f + 0.03f * MathF.Sin(3f * angle));
                ArmSolution s = solver.Solve(Arm, target, 0f, 0f);
                if (s.Status == ArmSolveStatus.Exact && previous.Status == ArmSolveStatus.Exact)
                {
                    Assert.True(previous.Branch == s.Branch, $"step {i}: branch {previous.Branch} -> {s.Branch} ({previous.Status} -> {s.Status}), " +
                        $"prox {previous.Pose.Proximal:0.00}->{s.Pose.Proximal:0.00} dist {previous.Pose.Distal:0.00}->{s.Pose.Distal:0.00} " +
                        $"wrist {previous.Pose.Wrist:0.00}->{s.Pose.Wrist:0.00} pitch {previous.Pitch:0.00}->{s.Pitch:0.00}");
                    Assert.True(previous.Pose.MaxJointDelta(s.Pose) < 0.2f, $"step {i}: jump {previous.Pose.MaxJointDelta(s.Pose)}");
                }

                previous = s;
            }
        }

        [Fact]
        public void AHeldTarget_FarAway_IsReachedOverSeveralSolves_NotLeftHalfway()
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            // The base turns about 80 degrees: more than one solve's 30, within the 90 per joint
            // the solver travels to on its own.
            var target = new Vector3(0.03f, 0.17f, 0.10f);
            ArmSolution s = default;
            int solves = 0;
            do
            {
                s = solver.Solve(Arm, target, 0f, 0f);
                solves++;
            }
            while ((s.Status == ArmSolveStatus.Blocked || s.Status == ArmSolveStatus.Approaching) && solves < 40);

            Assert.True(s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted, $"ended {s.Status}");
            Assert.True(solves > 1, "the test should need more than one solve to mean anything");
        }

        [Fact]
        public void SameInputs_GiveSameOutputs()
        {
            var a = new ArmSolver();
            var b = new ArmSolver();
            a.Reset(Arm, Model.Home);
            b.Reset(Arm, Model.Home);
            var random = new Random(3);
            for (int i = 0; i < 500; i++)
            {
                var target = new Vector3(Next(random, 0.3f), Next(random, 0.3f), Next(random, 0.3f));
                float pitch = Next(random, 1.5f), roll = Next(random, 1f);
                ArmSolution x = a.Solve(Arm, target, pitch, roll);
                ArmSolution y = b.Solve(Arm, target, pitch, roll);
                Assert.Equal(x.Pose.BaseYaw, y.Pose.BaseYaw);
                Assert.Equal(x.Pose.Proximal, y.Pose.Proximal);
                Assert.Equal(x.Pose.Distal, y.Pose.Distal);
                Assert.Equal(x.Pose.Wrist, y.Pose.Wrist);
                Assert.Equal(x.Status, y.Status);
            }
        }

        [Fact]
        public void WithoutACollisionModel_ItIsThePlainClosedFormSolve()
        {
            RobotArmProfile plain = Profile(null);
            var solver = new ArmSolver();
            var target = new Vector3(0.12f, -0.05f, 0.02f);
            ArmSolution s = solver.Solve(plain, target, 0.3f, 0f);
            Span<float> wrists = stackalloc float[1];
            ArmKinematics.TryInverse(plain, target, 0.3f, out float yaw, out float proximal, out float distal, wrists, out _);
            Assert.Equal(yaw, s.Pose.BaseYaw, 5);
            Assert.Equal(proximal, s.Pose.Proximal, 5);
            Assert.Equal(distal, s.Pose.Distal, 5);
            Assert.Equal(wrists[0], s.Pose.Wrist, 5);
            Assert.Equal(ArmSolveStatus.Exact, s.Status);
        }

        [Fact]
        public void Solve_AllocatesNothing_EvenWhenBlocked()
        {
            var holder = new SolverHolder();
            holder.Solver.Reset(Arm, Model.Home);
            float x = 0f;
            RobotArmProfile arm = Arm;
            AllocationAssert.Zero(() =>
            {
                x += 0.001f;
                // Alternates between a free target and one inside the deck.
                Vector3 target = (int)(x * 1000) % 2 == 0 ? new Vector3(0.15f, 0f, 0.08f) : new Vector3(0.13f, 0f, -0.06f);
                holder.Solver.Solve(arm, target, -0.4f, 0f);
            }, iterations: 2000);
        }

        private sealed class SolverHolder
        {
            public ArmSolver Solver;
        }

        private static float Next(Random random, float magnitude) => magnitude * (2f * (float)random.NextDouble() - 1f);
    }
}
