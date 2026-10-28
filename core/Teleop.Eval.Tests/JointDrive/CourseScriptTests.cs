using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>joint-drive's course script: the robot driving the training course itself, within the same safety rules as every other script.</summary>
    public class CourseScriptTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);

        private static DrivePlan Build(ulong seed, double seconds = 600)
        {
            Assert.True(DrivePlan.TryBuild(DriveScript.Course, Profile, Model.Home, 6f, seconds, seed, Envelope, out DrivePlan? plan, out string? error), error);
            return plan!;
        }

        [Fact]
        public void EveryLeg_IsCollisionFree_InsideTheEnvelope_AndNoFasterThanTheSlidersTop()
        {
            DrivePlan plan = Build(3);
            foreach (DriveLeg leg in plan.Legs)
            {
                Assert.True(DrivePlan.PathValid(Profile, Model, leg.Phase == LegPhase.Approach ? null : Envelope, leg.From, leg.To),
                    $"{leg.Start:0.0} s: {DrivePlan.Describe(leg.From)} -> {DrivePlan.Describe(leg.To)}");
                if (leg.Duration > 0)
                {
                    // A minimum-jerk leg peaks at 15/8 of its mean speed; a linear one is its mean.
                    double peak = leg.From.MaxJointDelta(leg.To) / leg.Duration * (leg.Linear ? 1.0 : 1.875);
                    Assert.True(peak <= CourseScript.MaxPeakRadPerSec + 1e-3, $"{leg.Start:0.0} s: {peak:0.00} rad/s");
                }
            }

            Assert.InRange(plan.TotalSeconds, 600, 700);
        }

        [Fact]
        public void TheGripper_OpensAndCloses_ButNeverPastItsCap_NorClosedForLong()
        {
            DrivePlan plan = Build(4);
            var fractions = new HashSet<float>();
            double closedSince = double.NaN, longestClosed = 0;
            for (double t = 0; t < plan.TotalSeconds; t += 0.02)
            {
                float g = plan.GripperAt(t);
                if (float.IsNaN(g)) continue;
                Assert.InRange(g, CourseScript.GripperOpen, CourseScript.GripperClosed);
                fractions.Add((float)Math.Round(g, 2));
                bool closed = g >= CourseScript.GripperClosed - 1e-4f;
                if (closed && double.IsNaN(closedSince)) closedSince = t;
                if (!closed && !double.IsNaN(closedSince))
                {
                    longestClosed = Math.Max(longestClosed, t - closedSince);
                    closedSince = double.NaN;
                }
            }

            Assert.Contains(CourseScript.GripperOpen, fractions);
            Assert.Contains(CourseScript.GripperClosed, fractions);
            // Fully closed at most for its hold, plus the next move's ramp away.
            Assert.True(longestClosed <= CourseScript.MaxClosedSeconds + CourseScript.GripperRampSeconds + 0.05, $"{longestClosed:0.00} s closed");
        }

        [Fact]
        public void ItCoversEveryLevel_AndVariesThem()
        {
            DrivePlan plan = Build(5, seconds: 1800);
            var speeds = plan.Legs.Where(l => l.Phase == LegPhase.Script && !float.IsNaN(l.Speed)).Select(l => l.Speed).ToHashSet();
            foreach (float level in CourseScript.Levels) Assert.Contains(level, speeds);
            Assert.All(speeds, s => Assert.Contains(s, CourseScript.Levels));
            Assert.Contains(plan.Legs, l => l.Linear); // traces
        }

        [Fact]
        public void APlan_IsDeterministic()
        {
            DrivePlan a = Build(9, 300), b = Build(9, 300), c = Build(10, 300);
            Assert.Equal(a.Legs.Count, b.Legs.Count);
            for (int i = 0; i < a.Legs.Count; i++)
            {
                Assert.Equal(a.Legs[i].To.BaseYaw, b.Legs[i].To.BaseYaw);
                Assert.Equal(a.Legs[i].Start, b.Legs[i].Start);
                Assert.Equal(a.Legs[i].GripperTo, b.Legs[i].GripperTo);
            }

            Assert.NotEqual(a.Legs[5].To.BaseYaw, c.Legs[5].To.BaseYaw);
        }

        [Fact]
        public void ItEndsBackHome()
        {
            DrivePlan plan = Build(6, 300);
            Assert.True(plan.Legs[^1].To.MaxJointDelta(Model.Home) < 0.01f);
            Assert.Equal(LegPhase.Return, plan.Legs[^1].Phase);
        }
    }
}
