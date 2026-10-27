using System;
using System.Numerics;
using Teleop.Core.Types;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Training
{
    public class TrainingCourseTests
    {
        private const float Floor = -0.05f; // the headset's ClutchMinHeightMeters
        private const double Frame = 1.0 / 90.0;
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;

        [Fact]
        public void EverySampledTarget_IsReachedByTheHeadsetsSolver_AboveTheFloor()
        {
            var sampler = new TrainingTargetSampler(Profile, Floor);
            var rng = new SeededRng(7);
            for (int i = 0; i < 60; i++)
            {
                Assert.True(sampler.TrySample(ref rng, out Vector3 t));
                Assert.True(t.Z >= Floor + TrainingTargetSampler.FloorClearanceMeters);
                Assert.True(MathF.Sqrt(t.X * t.X + t.Y * t.Y) >= TrainingTargetSampler.MinRadiusMeters);
                Assert.True(t.X > 0f); // in front: yaw within 75 degrees of straight ahead

                // Independently: the solver, from home, ends at the target.
                var solver = new ArmSolver();
                solver.Reset(Profile, Profile.Collision!.Home);
                ArmSolution s = default;
                for (int k = 0; k < TrainingTargetSampler.MaxSolvesPerCheck && (k == 0 || s.Status == ArmSolveStatus.Approaching); k++)
                {
                    s = solver.Solve(Profile, t, 0f, 0f);
                }

                Assert.True(s.Status == ArmSolveStatus.Exact || s.Status == ArmSolveStatus.PitchAdjusted, $"{t}: {s.Status}");
                Assert.True(Vector3.Distance(s.Position, t) <= TrainingTargetSampler.ReachToleranceMeters);
            }
        }

        [Fact]
        public void APointInsideTheBody_OrBelowTheFloor_IsNotReachable()
        {
            var sampler = new TrainingTargetSampler(Profile, Floor);
            Assert.False(sampler.IsReachable(new Vector3(0.02f, 0f, 0.05f)));    // the arm's own column
            Assert.False(sampler.IsReachable(new Vector3(0.15f, 0f, Floor)));    // on the floor
            Assert.False(sampler.IsReachable(new Vector3(0.60f, 0f, 0.10f)));    // out of reach
        }

        [Fact]
        public void TheSameCourseId_GivesTheSameCourse()
        {
            TrainingCourse a = Started(42), b = Started(42), c = Started(43);
            Assert.Equal(a.TargetCount, b.TargetCount);
            for (int i = 0; i < a.TargetCount; i++)
            {
                Assert.Equal(a.Target(i), b.Target(i));
                Assert.Equal(a.Goal(i), b.Goal(i));
            }

            Assert.NotEqual(a.Target(0), c.Target(0));
        }

        [Fact]
        public void ARound_IsAnnounced_ThenRuns_ThenShowsItsResult_ThenTheNextIsAnnounced()
        {
            TrainingCourse course = Started(1);
            Assert.Equal((TrainingTask.Reach, TrainingPhase.Idle, 1u), (course.Task, course.Phase, course.Round));
            Assert.Equal(TrainingCourse.ReachTargets, course.TargetCount);

            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            Assert.Equal(TrainingPhase.Active, course.Phase);

            now = Run(course, now, TrainingCourse.ReachRoundSeconds + 0.5, Vector3.Zero, ready: true);
            Assert.Equal(TrainingPhase.Done, course.Phase);
            Assert.Equal(0f, course.Score);

            now = Run(course, now, TrainingCourse.ResultSeconds + 0.05, Vector3.Zero, ready: true);
            Assert.Equal((TrainingTask.Trace, TrainingPhase.Idle, 2u), (course.Task, course.Phase, course.Round));
            Assert.Equal(TrainingCourse.TracePoints, course.PathCount);
        }

        [Fact]
        public void Reach_CountsATarget_AfterTheHold_AndOnlyWithTheGripperItAsks()
        {
            TrainingCourse course = Started(3);
            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            int reached = 0;
            for (int i = 0; i < course.TargetCount; i++)
            {
                Assert.Equal(i, course.ActiveTarget);
                Vector3 t = course.Target(i);
                GripperGoal goal = course.Goal(i);
                if (goal != GripperGoal.Any)
                {
                    // The wrong gripper never counts.
                    float wrong = goal == GripperGoal.Open ? 1f : 0f;
                    now = Run(course, now, 1.0, t, ready: true, gripper: wrong);
                    Assert.Equal(i, course.ActiveTarget);
                }

                float right = goal == GripperGoal.Closed ? 1f : 0f;
                now = Run(course, now, TrainingCourse.ReachHoldSeconds * 0.5, t, ready: true, gripper: right);
                Assert.Equal(i, course.ActiveTarget); // not held long enough yet
                now = Run(course, now, TrainingCourse.ReachHoldSeconds * 0.6, t, ready: true, gripper: right);
                reached++;
                if (reached < course.TargetCount) Assert.Equal(reached, (int)course.Score);
            }

            Assert.Equal(TrainingPhase.Done, course.Phase);
            Assert.Equal(TrainingCourse.ReachTargets, (int)course.Score);
        }

        [Fact]
        public void TheClock_StopsWhileTheRobotIsNotReady()
        {
            TrainingCourse course = Started(4);
            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: false);
            Assert.Equal(TrainingPhase.Suspended, course.Phase);
            now = Run(course, now, 200.0, Vector3.Zero, ready: false);
            Assert.Equal(TrainingPhase.Suspended, course.Phase);
            Assert.Equal(TrainingCourse.ReachRoundSeconds, course.SecondsLeft, 3);
            Assert.Equal(0f, course.CoverageSeconds(TrainingPurpose.Train, course.Level));

            Run(course, now, 10.0, Vector3.Zero, ready: true);
            Assert.Equal(TrainingPhase.Active, course.Phase);
            Assert.InRange(course.CoverageSeconds(TrainingPurpose.Train, course.Level), 9.9f, 10.1f);
        }

        [Fact]
        public void Trace_ScoresTheShareOfTimeOnTheBead_AndEveryPointOfItsCurveIsFollowable()
        {
            TrainingCourse course = Started(5);
            double now = SkipToTask(course, TrainingTask.Trace, 0);
            var sampler = new TrainingTargetSampler(Profile, Floor);
            var points = new Vector3[course.PathCount];
            for (int i = 0; i < points.Length; i++) points[i] = course.PathPoint(i);
            Assert.True(sampler.IsFollowable(points));

            now = Run(course, now, TrainingCourse.CountdownSeconds + 0.05, course.PathPoint(0), ready: true);
            // Ride the bead for half the round, then sit still.
            double half = TrainingCourse.TraceRoundSeconds / 2;
            for (double t = 0; t < half; t += Frame)
            {
                course.Update(now, course.Bead, 0f, true, false);
                now += Frame;
            }

            now = Run(course, now, half - 1.0, new Vector3(0.3f, 0.3f, 0.3f), ready: true);
            Assert.InRange(course.Score, 45f, 56f);
        }

        [Fact]
        public void Shuttle_CountsAlternateTouches_OnlyAlternately()
        {
            TrainingCourse course = Started(6);
            double now = SkipToTask(course, TrainingTask.Shuttle, 0);
            Assert.Equal(2, course.TargetCount);
            float apart = Vector3.Distance(course.Target(0), course.Target(1));
            Assert.InRange(apart, TrainingCourse.ShuttleMinApart, TrainingCourse.ShuttleMaxApart);

            now = Run(course, now, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            now = Run(course, now, 1.0, course.Target(0), ready: true);
            now = Run(course, now, 1.0, course.Target(0), ready: true); // staying on the same one does not count again
            Assert.Equal(1f, course.Score);
            now = Run(course, now, 1.0, course.Target(1), ready: true);
            Run(course, now, 1.0, course.Target(0), ready: true);
            Assert.Equal(3f, course.Score);
        }

        [Fact]
        public void EachRound_TakesTheLeastCoveredLevel_ForItsPurpose()
        {
            var course = new TrainingCourse(Profile, Floor);
            var counts = new float[TrainingCourse.PurposeCount * TrainingCourse.LevelCount];
            counts[0] = 100; counts[1] = 50; counts[2] = 300; counts[3] = 10; counts[4] = 20; // train
            counts[5] = 0; counts[6] = 0; counts[7] = 0; counts[8] = 0; counts[9] = 0;       // test
            course.LoadCoverage(counts);
            course.Start(9, 0);
            Assert.Equal(3, course.Level);
            Assert.Equal(6.0f, course.SpeedRadPerSec);

            course.Stop();
            course.Purpose = TrainingPurpose.Test;
            course.Start(10, 0);
            Assert.Equal(0, course.Level);
            Assert.Equal(TrainingPurpose.Test, course.State.Purpose);
        }

        [Fact]
        public void AVariedRound_ChangesItsSpeed_AndReportsNaN()
        {
            var course = new TrainingCourse(Profile, Floor);
            var counts = new float[TrainingCourse.PurposeCount * TrainingCourse.LevelCount];
            for (int i = 0; i < 4; i++) counts[i] = 100;
            course.LoadCoverage(counts);
            course.Start(11, 0);
            Assert.Equal(TrainingCourse.VariedLevel, course.Level);
            Assert.True(float.IsNaN(course.State.SpeedLevel));

            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            var seen = new System.Collections.Generic.HashSet<float>();
            for (double t = 0; t < 60; t += Frame)
            {
                course.Update(now, Vector3.Zero, 0f, true, false);
                seen.Add(course.SpeedRadPerSec);
                now += Frame;
            }

            Assert.True(seen.Count >= 3, string.Join(",", seen));
            foreach (float s in seen) Assert.Contains(s, TrainingCourse.SpeedLevels);
        }

        [Fact]
        public void TheDisturbedFlag_SticksForTheRound()
        {
            TrainingCourse course = Started(12);
            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            Assert.False(course.State.Disturbed);
            course.Update(now, Vector3.Zero, 0f, true, true);
            now += Frame;
            now = Run(course, now, 1.0, Vector3.Zero, ready: true);
            Assert.True(course.State.Disturbed);
        }

        [Fact]
        public void Stopped_SendsNoSpeed_AndReportsIdle()
        {
            TrainingCourse course = Started(13);
            course.Stop();
            Assert.True(float.IsNaN(course.SpeedRadPerSec));
            Assert.Equal((TrainingTask.None, TrainingPhase.Idle), (course.State.Task, course.State.Phase));
            Assert.Equal(0, course.TargetCount);
        }

        [Fact]
        public void Update_AllocatesNothing()
        {
            TrainingCourse course = Started(14);
            double now = Run(course, 0, TrainingCourse.CountdownSeconds + 0.05, Vector3.Zero, ready: true);
            for (int i = 0; i < 500; i++) // warm up: the first calls JIT, which counts as allocation
            {
                now += Frame;
                course.Update(now, new Vector3(0.15f, 0f, 0.08f), 0.5f, true, false);
                _ = course.State;
            }

            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 2000; i++)
            {
                now += Frame;
                course.Update(now, new Vector3(0.15f, 0f, 0.08f), 0.5f, true, false);
                _ = course.State;
            }

            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }

        private static TrainingCourse Started(uint id)
        {
            var course = new TrainingCourse(Profile, Floor);
            course.Start(id, 0);
            return course;
        }

        private static double Run(TrainingCourse course, double now, double seconds, Vector3 wrist, bool ready, float gripper = 0f)
        {
            double end = now + seconds;
            for (; now < end; now += Frame) course.Update(now, wrist, gripper, ready, false);
            course.Update(now, wrist, gripper, ready, false);
            return now;
        }

        // Plays rounds (doing nothing) until the given task is announced.
        private static double SkipToTask(TrainingCourse course, TrainingTask task, double now)
        {
            for (int guard = 0; guard < 10 && !(course.Task == task && course.Phase == TrainingPhase.Idle); guard++)
            {
                now = Run(course, now, 1.0, Vector3.Zero, ready: true);
                while (course.Phase != TrainingPhase.Idle) now = Run(course, now, 1.0, Vector3.Zero, ready: true);
            }

            Assert.Equal(task, course.Task);
            return now;
        }
    }
}
