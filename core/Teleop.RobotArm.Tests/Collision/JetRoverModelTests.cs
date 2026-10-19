using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Xunit.Abstractions;

namespace Teleop.RobotArm.Tests.Collision
{
    /// <summary>
    /// The generated JetRover model (docs/adr/0020) against what the robot is known to do. If one
    /// of these fails after rerunning <c>just robot-collision-model</c>, the fit changed in a way
    /// that matters on hardware: look at the previews before relaxing anything here.
    /// </summary>
    public class JetRoverModelTests
    {
        private static readonly RobotArmProfile Arm = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Arm.Collision!;
        private static readonly float Deg = MathF.PI / 180f;
        private readonly ITestOutputHelper _output;

        public JetRoverModelTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void TheDefaultProfile_CarriesTheGeneratedModel()
        {
            Assert.NotNull(Arm.Collision);
            Assert.Same(JetRoverCollisionModel.Instance, ArmCollisionModels.ForProfile("jetrover"));
            Assert.Null(ArmCollisionModels.ForProfile("some-other-arm"));
            // The hand-tuned shoulder floor is gone: the model does its job now.
            Assert.All(Arm.Joints, j => Assert.Null(j.MinAngleRadians));
        }

        [Fact]
        public void Home_IsValid() => Assert.True(ArmCollision.IsValid(Arm, Model, Model.Home), ArmCollision.Check(Arm, Model, Model.Home).Describe(Model));

        [Theory]
        [InlineData(0.15f, 0f, 0.08f)]   // the validated hardware target (robot/README.md)
        [InlineData(0.18f, 0.05f, 0.10f)]
        [InlineData(0.20f, -0.08f, 0.06f)]
        public void EverydayTargets_AreReachedExactly(float x, float y, float z)
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            ArmSolution s = solver.Solve(Arm, new Vector3(x, y, z), 0f, 0f);
            Assert.Equal(ArmSolveStatus.Exact, s.Status);
            Assert.False(s.ReachClamped);
        }

        [Fact]
        public void ShoulderDrivenIntoTheDeck_IsCaught()
        {
            // The case the old hand-tuned floor existed for: the shoulder pitched down into the plate.
            CollisionHit hit = ArmCollision.Check(Arm, Model, new ArmPose(0f, -100f * Deg, 60f * Deg, 0f, 0f));
            Assert.True(hit.IsHit);
        }

        [Fact]
        public void TargetInsideTheChassis_StopsAboveIt()
        {
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            solver.Solve(Arm, new Vector3(0.15f, 0f, 0.08f), -0.6f, 0f);
            ArmSolution s = solver.Solve(Arm, new Vector3(0.05f, 0f, -0.08f), -0.6f, 0f);
            Assert.Equal(ArmSolveStatus.Blocked, s.Status);
            Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose));
        }

        [Fact]
        public void EveryResult_IsValid_AndTheCostIsLogged()
        {
            var random = new Random(9);
            var solver = new ArmSolver();
            solver.Reset(Arm, Model.Home);
            var times = new double[5000];
            var stopwatch = new Stopwatch();
            var target = new Vector3(0.15f, 0f, 0.08f);
            int blocked = 0;
            for (int i = 0; i < times.Length; i++)
            {
                target = i % 40 == 0
                    ? new Vector3(Next(random, 0.3f), Next(random, 0.3f), Next(random, 0.25f))
                    : target + new Vector3(Next(random, 0.01f), Next(random, 0.01f), Next(random, 0.01f));
                stopwatch.Restart();
                ArmSolution s = solver.Solve(Arm, target, Next(random, 1.2f), Next(random, 1.2f));
                times[i] = stopwatch.Elapsed.TotalMilliseconds;
                if (s.Status == ArmSolveStatus.Blocked) blocked++;
                Assert.True(ArmCollision.IsValid(Arm, Model, s.Pose), $"step {i}: {ArmCollision.Check(Arm, Model, s.Pose).Describe(Model)}");
            }

            double[] warm = times[100..];
            Array.Sort(warm);
            _output.WriteLine($"jetrover solve: mean {warm.Average():0.000} ms, p99 {warm[(int)(warm.Length * 0.99)]:0.000} ms, " +
                $"max {warm[^1]:0.000} ms; {blocked} of {times.Length} blocked");
        }

        private static float Next(Random random, float magnitude) => magnitude * (2f * (float)random.NextDouble() - 1f);
    }
}
