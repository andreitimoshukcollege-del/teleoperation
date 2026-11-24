using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Teleop.Eval.JointDrive;
using Teleop.Eval.StopLatency;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;

namespace Teleop.Eval.Tests.StopLatency
{
    public class StopLatencyTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);
        private static readonly int[] AllJoints = { 0, 1, 2, 3 };
        private static readonly float[] DefaultSpeeds = { 0.5f, 1f, 2f };

        private static StopLatencyPlan Build(IReadOnlyList<int>? joints = null, IReadOnlyList<float>? speeds = null, int repeats = 2,
            double runUp = 0.35, double hold = 1.5)
        {
            Assert.True(StopLatencyPlan.TryBuild(Profile, Model, Envelope, Model.Home, joints ?? AllJoints, speeds ?? DefaultSpeeds, repeats, runUp, hold,
                out StopLatencyPlan? plan, out string? error), error);
            return plan!;
        }

        [Fact]
        public void EveryCommandedPose_IsInsideTheEnvelope_AndFree()
        {
            StopLatencyPlan plan = Build();
            Assert.NotEmpty(plan.Trials);
            for (double t = 0; t <= plan.TotalSeconds; t += 0.01)
            {
                ArmPose p = plan.Sample(t, out _, out _);
                Assert.True(Envelope.Contains(p), $"t={t:0.00}: {DrivePlan.Describe(p)} leaves the envelope");
                Assert.True(ArmCollision.IsValid(Profile, Model, p), $"t={t:0.00}: {DrivePlan.Describe(p)} collides");
            }
        }

        [Fact]
        public void ThePlan_IsDeterministic_AndCoversEveryJointAndSpeed()
        {
            StopLatencyPlan a = Build(), b = Build();
            Assert.Equal(a.Trials.Select(t => (t.Joint, t.SpeedRadPerSec, t.Direction, t.StopAt)), b.Trials.Select(t => (t.Joint, t.SpeedRadPerSec, t.Direction, t.StopAt)));
            foreach (float s in DefaultSpeeds)
            {
                foreach (int j in AllJoints)
                {
                    Assert.Equal(2, a.Trials.Count(t => t.Joint == j && t.SpeedRadPerSec == s) + a.Skipped.Count(x => x.StartsWith(DriveEnvelope.JointName(j)) && x.Contains(FormattableString.Invariant($"{s:0.##} rad/s"))));
                }
            }
        }

        [Fact]
        public void TheCommand_MovesAtItsSpeed_ThenStopsDead_AndHolds()
        {
            StopLatencyPlan plan = Build(new[] { 2 }, new[] { 1f }, repeats: 1);
            StopTrialPlan trial = plan.Trials[0];
            float start = DrivePlan.Joint(plan.StartPose, 2);
            Assert.Equal(StopLatencyPlan.Excursion(1f, 0.35), trial.ExcursionRadians, 5);
            Assert.Equal(1f * (0.05f + 0.35f), trial.ExcursionRadians, 5);

            // At full speed just before the stop: 1 rad/s over the last 10 ms.
            float before = DrivePlan.Joint(plan.Sample(trial.StopAt - 0.02, out StopPhase p1, out _), 2);
            float justBefore = DrivePlan.Joint(plan.Sample(trial.StopAt - 0.01, out _, out _), 2);
            Assert.Equal(StopPhase.Move, p1);
            Assert.Equal(0.01f, MathF.Abs(justBefore - before), 4);

            // From the stop on, the command is exactly the held pose until the hold ends.
            for (double t = trial.StopAt; t < trial.HoldEnd; t += 0.01)
            {
                ArmPose p = plan.Sample(t, out StopPhase phase, out _);
                Assert.Equal(StopPhase.Hold, phase);
                Assert.Equal(DrivePlan.Joint(trial.Held, 2), DrivePlan.Joint(p, 2));
            }

            Assert.Equal(start + trial.Direction * trial.ExcursionRadians, DrivePlan.Joint(trial.Held, 2), 5);
            // Only the trial's joint moves.
            foreach (int other in new[] { 0, 1, 3, 4 })
            {
                Assert.Equal(DrivePlan.Joint(plan.StartPose, other), DrivePlan.Joint(trial.Held, other));
            }

            // Back at the start after the return, and still there through the rest.
            ArmPose back = plan.Sample(trial.ReturnEnd + 0.01, out StopPhase rest, out _);
            Assert.Equal(StopPhase.Rest, rest);
            Assert.Equal(start, DrivePlan.Joint(back, 2), 5);
        }

        [Fact]
        public void Repeats_AlternateDirection_WhereBothFit()
        {
            StopLatencyPlan plan = Build(new[] { 0 }, new[] { 0.5f }, repeats: 2);
            Assert.Equal(new[] { 1f, -1f }, plan.Trials.Select(t => t.Direction));
        }

        [Fact]
        public void ASwingOverTheLimit_IsLeftOut_AndAPlanWithNothingLeftIsRefused()
        {
            // 3 rad/s for a 1 s run-up is 3.15 rad, over the 60-degree limit.
            Assert.False(StopLatencyPlan.TryBuild(Profile, Model, Envelope, Model.Home, new[] { 0 }, new[] { 3f }, 1, 1.0, 1.5, out _, out string? error));
            Assert.Contains("limit", error);
            Assert.True(StopLatencyPlan.TryBuild(Profile, Model, Envelope, Model.Home, new[] { 0 }, new[] { 0.5f, 3f }, 1, 1.0, 1.5, out StopLatencyPlan? plan, out _));
            Assert.Single(plan!.Trials);
            Assert.Single(plan.Skipped);
        }

        [Theory]
        [InlineData(new[] { 4 }, new[] { 1f }, 1, 0.35, 1.5)]     // the roll is not a trial joint
        [InlineData(new[] { 0 }, new[] { 3.5f }, 1, 0.35, 1.5)]   // faster than the limit
        [InlineData(new[] { 0 }, new[] { 1f }, 0, 0.35, 1.5)]     // no repeats
        [InlineData(new[] { 0 }, new[] { 1f }, 1, 0.1, 1.5)]      // too short a run-up to trail steadily
        [InlineData(new[] { 0 }, new[] { 1f }, 1, 0.35, 0.5)]     // too short a hold to see it stop
        public void BadSettings_AreRefused(int[] joints, float[] speeds, int repeats, double runUp, double hold)
        {
            Assert.False(StopLatencyPlan.TryBuild(Profile, Model, Envelope, Model.Home, joints, speeds, repeats, runUp, hold, out _, out string? error));
            Assert.False(string.IsNullOrEmpty(error));
        }

        /// <summary>An arm that follows its command <paramref name="delay"/> late, read every 20 ms, from the plan's start.</summary>
        private static StopMeter Follow(StopLatencyPlan plan, double delay, double overshootRad = 0, double overshootSeconds = 0.1)
        {
            var meter = new StopMeter();
            for (double t = 0; t <= plan.TotalSeconds; t += 0.02)
            {
                ArmPose p = plan.Sample(Math.Max(0, t - delay), out StopPhase phase, out int trial);
                if (overshootRad > 0 && trial >= 0)
                {
                    StopTrialPlan tr = plan.Trials[trial];
                    double since = t - delay - tr.StopAt;
                    if (since >= 0 && since < overshootSeconds)
                        p = DrivePlan.WithJoint(p, tr.Joint, DrivePlan.Joint(p, tr.Joint) + tr.Direction * (float)(overshootRad * Math.Sin(Math.PI * since / overshootSeconds)));
                }

                meter.Add(t, p);
            }

            return meter;
        }

        [Fact]
        public void TheMeter_SeesADelayedArmKeepMovingForItsDelay_AndAsFarAsItTrailed()
        {
            StopLatencyPlan plan = Build(new[] { 2 }, new[] { 1f }, repeats: 1);
            StopTrialPlan trial = plan.Trials[0];
            StopMeter meter = Follow(plan, delay: 0.2);
            Assert.True(meter.TryMeasure(Profile, trial, trial.StopAt, trial.HoldEnd, out StopResult r));
            // Readings every 20 ms: the last moving one is within a reading of the delay.
            Assert.InRange(r.StopMs, 160, 220);
            // At 1 rad/s, 0.2 s behind: about 11.5 degrees to go when the command stopped, and it goes that far.
            Assert.InRange(r.ErrorAtStopDeg, 10.0, 13.0);
            Assert.InRange(r.TravelDeg, 10.0, 13.0);
            Assert.Equal(0.0, r.PastTargetDeg, 3);
            Assert.True(r.Settled);
            Assert.True(r.ToolTravelCm > 1.0);
            Assert.Equal(trial.Joint, r.Joint);
        }

        [Fact]
        public void TheMeter_SeesAnOvershoot()
        {
            StopLatencyPlan plan = Build(new[] { 0 }, new[] { 1f }, repeats: 1);
            StopTrialPlan trial = plan.Trials[0];
            StopMeter meter = Follow(plan, delay: 0.2, overshootRad: 2.0 * Math.PI / 180, overshootSeconds: 0.2);
            Assert.True(meter.TryMeasure(Profile, trial, trial.StopAt, trial.HoldEnd, out StopResult r));
            Assert.InRange(r.PastTargetDeg, 1.8, 2.01);
            Assert.InRange(r.StopMs, 300, 420);
        }

        [Fact]
        public void TheMeter_WithNoReadingBeforeTheStop_MeasuresNothing()
        {
            StopLatencyPlan plan = Build(new[] { 0 }, new[] { 1f }, repeats: 1);
            var meter = new StopMeter();
            meter.Add(plan.Trials[0].StopAt + 0.1, plan.StartPose);
            Assert.False(meter.TryMeasure(Profile, plan.Trials[0], plan.Trials[0].StopAt, plan.Trials[0].HoldEnd, out _));
        }

        [Fact]
        public void TheToolPoint_IsTheWristPlusTheGripperAxis()
        {
            // At home the wrist pitch closes to -0 (60 - 90 + 30): the gripper points level, straight ahead.
            ArmPose home = Model.Home;
            System.Numerics.Vector3 wrist = Teleop.RobotArm.Kinematics.ArmKinematics.Forward(Profile, home.BaseYaw, home.Proximal, home.Distal);
            System.Numerics.Vector3 tool = StopMeter.Tool(Profile, home);
            Assert.Equal(wrist.X + StopMeter.ToolOffsetMeters, tool.X, 3);
            Assert.Equal(wrist.Z, tool.Z, 3);
        }

        [Fact]
        public void TheCommandLine_RefusesToMoveWithoutTheConfirmation()
        {
            Assert.Equal(64, StopLatencyCommand.Run(new[] { "stop-latency", "--remote-host", "127.0.0.1" }));
            Assert.Equal(64, StopLatencyCommand.Run(new[] { "stop-latency", "--dry-run", "--speeds", "4", "--slider", "3.8" }));
            Assert.Equal(64, StopLatencyCommand.Run(new[] { "stop-latency", "--dry-run", "--joints", "elbow" }));
            Assert.Equal(0, StopLatencyCommand.Run(new[] { "stop-latency", "--dry-run" }));
        }
    }
}
