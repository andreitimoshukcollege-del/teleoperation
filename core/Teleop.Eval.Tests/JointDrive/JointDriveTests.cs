using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.Tests.JointDrive
{
    public class JointDriveTests
    {
        private static readonly RobotArmProfile Profile = RobotArmProfile.JetRoverMeasuredDefault;
        private static readonly ArmCollisionModel Model = Profile.Collision!;
        private static readonly DriveEnvelope Envelope = DriveEnvelope.FromModel(Model, 15f * MathF.PI / 180f);

        private static DrivePlan Build(DriveScript script, float speed, ulong seed, double seconds = 300, ArmPose? start = null)
        {
            Assert.True(DrivePlan.TryBuild(script, Profile, start ?? Model.Home, speed, seconds, seed, Envelope, out DrivePlan? plan, out string? error), error);
            return plan!;
        }

        [Fact]
        public void TheEnvelope_IsTheModelsRangesLessTheMargin()
        {
            float m = 15f * MathF.PI / 180f;
            Assert.Equal(Model.Proximal.Min + m, Envelope[1].Min, 6);
            Assert.Equal(Model.Proximal.Max - m, Envelope[1].Max, 6);
            Assert.Equal(Model.BaseYaw.Max - m, Envelope[0].Max, 6);
            Assert.Throws<ArgumentException>(() => DriveEnvelope.FromModel(Model, 2f));
        }

        [Theory]
        [InlineData(DriveScript.Waypoints)]
        [InlineData(DriveScript.Steps)]
        public void APlan_IsDeterministic(DriveScript script)
        {
            DrivePlan a = Build(script, 2.5f, 7), b = Build(script, 2.5f, 7), c = Build(script, 2.5f, 8);
            Assert.Equal(a.Legs.Select(l => (l.Start, l.To)), b.Legs.Select(l => (l.Start, l.To)));
            Assert.NotEqual(a.Legs.Select(l => l.To), c.Legs.Select(l => l.To));
        }

        [Theory]
        [InlineData(DriveScript.Waypoints, 1.0f, 1UL)]
        [InlineData(DriveScript.Waypoints, 6.0f, 2UL)]
        [InlineData(DriveScript.Steps, 2.5f, 3UL)]
        [InlineData(DriveScript.Steps, 6.0f, 4UL)]
        public void EveryCommandedPose_IsInsideTheEnvelope_AndFree(DriveScript script, float speed, ulong seed)
        {
            DrivePlan plan = Build(script, speed, seed, seconds: 600);
            int checkedPoses = 0;
            foreach (DriveLeg leg in plan.Legs)
            {
                // The path a leg covers, at the check's own spacing: what the arm passes through.
                Assert.True(DrivePlan.PathValid(Profile, Model, Envelope, leg.From, leg.To), $"leg at {leg.Start:0.0} s leaves the envelope or collides");
                checkedPoses++;
            }

            for (double t = 0; t <= plan.TotalSeconds; t += 1 / 48.0)
            {
                ArmPose p = plan.Sample(t);
                Assert.True(Envelope.Contains(p), $"{DrivePlan.Describe(p)} at {t:0.00} s is outside the envelope");
                Assert.True(ArmCollision.IsValid(Profile, Model, p), $"{DrivePlan.Describe(p)} at {t:0.00} s collides");
            }

            Assert.True(checkedPoses > 100);
        }

        [Theory]
        [InlineData(1.0f)]
        [InlineData(4.0f)]
        public void SmoothMotion_NeverExceedsTheSpeed(float speed)
        {
            DrivePlan plan = Build(DriveScript.Waypoints, speed, 5);
            const double dt = 0.002;
            ArmPose previous = plan.Sample(0);
            for (double t = dt; t <= plan.TotalSeconds; t += dt)
            {
                ArmPose p = plan.Sample(t);
                Assert.True(previous.MaxJointDelta(p) / dt <= speed * 1.001, $"{previous.MaxJointDelta(p) / dt:0.00} rad/s at {t:0.000} s");
                previous = p;
            }
        }

        [Fact]
        public void Steps_MoveOneJointAtATime_ByABoundedAmount()
        {
            DrivePlan plan = Build(DriveScript.Steps, 3f, 9);
            foreach (DriveLeg leg in plan.Legs.Where(l => l.Phase == LegPhase.Script))
            {
                Assert.Equal(0, leg.Duration);
                int moved = Enumerable.Range(0, DriveEnvelope.JointCount).Count(j => DrivePlan.Joint(leg.From, j) != DrivePlan.Joint(leg.To, j));
                Assert.Equal(1, moved);
                Assert.InRange(leg.From.MaxJointDelta(leg.To), 0.075f - 1e-6f, 0.8f + 1e-6f);
                Assert.InRange(leg.Hold, 1.2, 1.8);
            }
        }

        [Fact]
        public void APlan_StartsFromTheArm_AndEndsAtHome()
        {
            var start = new ArmPose(0.3f, 1.2f, -1.2f, 0.2f, 0.1f);
            DrivePlan plan = Build(DriveScript.Waypoints, 2f, 1, 60, start);
            Assert.Equal(LegPhase.Approach, plan.Legs[0].Phase);
            Assert.Equal(start.BaseYaw, plan.Sample(0).BaseYaw);
            Assert.Equal(LegPhase.Return, plan.Legs[plan.Legs.Count - 1].Phase);
            Assert.True(plan.Sample(plan.TotalSeconds + 5).MaxJointDelta(Model.Home) < 1e-6f);
            Assert.True(plan.TotalSeconds >= 60);
        }

        [Fact]
        public void AStartThatCannotReachHomeFreely_IsRefused()
        {
            // Proximal far below its range: the model's own range check fails at the first pose.
            var start = new ArmPose(0f, -1.2f, 0f, 0f, 0f);
            Assert.False(DrivePlan.TryBuild(DriveScript.Waypoints, Profile, start, 2f, 60, 1, Envelope, out DrivePlan? plan, out string? error));
            Assert.Null(plan);
            Assert.Contains("move-arm", error);
        }

        [Fact]
        public void TheWatchdog_StopsAStalledJoint_NotAMovingOne()
        {
            var commanded = new ArmPose(0f, 1.0f, -1.5f, 0.5f, 0f);
            var moving = new DriveWatchdog();
            var stalled = new DriveWatchdog();
            // Frames at RobotHost's 50 Hz, a fresh reading in two of every five: the robot's 20 Hz.
            for (int i = 0; i <= 150; i++)
            {
                double t = i * 0.02;
                moving.OnFrame(t);
                stalled.OnFrame(t);
                if (i % 5 != 0 && i % 5 != 2) continue;
                // Far behind but closing at 0.3 rad/s, against stuck 0.6 rad short.
                moving.OnReading(t, new ArmPose(0f, (float)(0.0 + 0.3 * t), -1.5f, 0.5f, 0f), commanded);
                stalled.OnReading(t, new ArmPose(0f, 0.4f, -1.5f, 0.5f, 0f), commanded);
            }

            Assert.Equal(DriveFault.None, moving.Check(3.0));
            Assert.Equal(DriveFault.Stall, stalled.Check(3.0));
            Assert.Equal(1, stalled.StalledJoint);
        }

        [Fact]
        public void AShortGap_OnlyPauses_AndTheStreamComingBackResumes()
        {
            var w = new DriveWatchdog();
            // Frames every 20 ms, except a 1.5 s Wi-Fi stall from 2 s, as on 2026-10-22's first recorded drive.
            for (double t = 0; t < 12; t += 0.01)
            {
                bool stalled = t >= 2.0 && t < 3.5;
                if (!stalled && Math.Round(t * 100) % 2 == 0) w.OnFrame(t);
                bool healthy = w.IsHealthy(t);
                if (t > 2.3 && t < 3.5) Assert.False(healthy);
                // Healthy again once the last second holds 30 frames: 0.6 s after they return.
                if (t > 4.15) Assert.True(healthy);
                Assert.Equal(DriveFault.None, w.Check(t));
            }
        }

        [Fact]
        public void ALongGap_ALongThinStream_OrAStreamThatKeepsDropping_Stops()
        {
            var gap = new DriveWatchdog();
            for (double t = 0; t < 1; t += 0.02) gap.OnFrame(t);
            for (double t = 1.3; t < 4.5; t += 0.01) Assert.False(gap.IsHealthy(t)); // frames stopped at 0.98 s
            Assert.Equal(DriveFault.FrameGap, gap.Check(4.5));

            // 10 Hz: the Wi-Fi losing most frames.
            var thin = new DriveWatchdog();
            for (double t = 0; t < 5; t += 0.01)
            {
                if (Math.Round(t * 100) % 10 == 0) thin.OnFrame(t);
                thin.IsHealthy(t);
            }

            Assert.Equal(DriveFault.StreamDegraded, thin.Check(5.0));

            // A second subscriber: the stream alternates between the two every half second.
            var shared = new DriveWatchdog();
            DriveFault fault = DriveFault.None;
            double at = 0;
            for (double t = 0; t < 12 && fault == DriveFault.None; t += 0.01)
            {
                if ((int)(t / 0.5) % 2 == 0 && Math.Round(t * 100) % 2 == 0) shared.OnFrame(t);
                shared.IsHealthy(t);
                fault = shared.Check(t);
                at = t;
            }

            Assert.Equal(DriveFault.StreamDegraded, fault);
            // Alternating halves average 25 Hz, under 30 throughout once the 1 s rate window fills: 3 s later it stops.
            Assert.InRange(at, 3.0, 5.0);

            var full = new DriveWatchdog();
            for (double t = 0; t < 12; t += 0.01)
            {
                if (Math.Round(t * 100) % 2 == 0) full.OnFrame(t);
                Assert.True(full.IsHealthy(t));
                Assert.Equal(DriveFault.None, full.Check(t));
            }
        }

        [Fact]
        public void TheOperatorLog_HasTheSessionFormatsHeaderRecordsAndTrailer()
        {
            var stream = new MemoryStream();
            using (var log = new OperatorLog(new NonClosing(stream), 10_000_000, "20261022T120000Z-drive-1"))
            {
                log.Command(123, 0, 120, new[] { new JointTarget(1, 0.25f, 2.5f), new JointTarget(10, 0f, 2.5f) });
                log.Command(456, 1, 450, new[] { new JointTarget(1, -0.1f, 2.5f) });
            }

            string text = Encoding.ASCII.GetString(stream.ToArray());
            string[] lines = text.Split('\n');
            Assert.Equal("JSES|1|operator|10000000|OPERATOR|20261022T120000Z-drive-1", lines[0]);
            Assert.Equal("OC|123|0|120|1:0.25:2.5;10:0:2.5", lines[1]);
            Assert.Equal("OC|456|1|450|1:-0.1:2.5", lines[2]);
            string body = lines[0] + "\n" + lines[1] + "\n" + lines[2] + "\n";
            string sha = Convert.ToHexString(SHA256.HashData(Encoding.ASCII.GetBytes(body))).ToLowerInvariant();
            Assert.Equal($"END|2|0|{sha}", lines[3]);
            Assert.Equal("", lines[4]);
        }

        [Fact]
        public void ACommand_CarriesEveryJointAtTheRequestedSpeed_InTheProfilesOrder()
        {
            var targets = new JointTarget[Profile.JointCount];
            int count = JointDriveCommand.Targets(Profile, new ArmPose(0.1f, 0.2f, 0.3f, 0.4f, 0.5f), 0.7f, 3f, targets);
            Assert.Equal(new byte[] { 1, 2, 3, 4, 10, 5 }, targets.Take(count).Select(t => t.MotorId));
            Assert.Equal(new[] { 0.1f, 0.2f, 0.3f, 0.4f, 0.7f, 0.5f }, targets.Take(count).Select(t => t.Angle));
            Assert.All(targets.Take(count), t => Assert.Equal(3f, t.Speed));
        }

        private sealed class NonClosing : Stream
        {
            private readonly Stream _inner;
            public NonClosing(Stream inner) => _inner = inner;
            public override bool CanRead => false;
            public override bool CanSeek => false;
            public override bool CanWrite => true;
            public override long Length => _inner.Length;
            public override long Position { get => _inner.Position; set => throw new NotSupportedException(); }
            public override void Flush() => _inner.Flush();
            public override int Read(byte[] buffer, int offset, int count) => throw new NotSupportedException();
            public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
            public override void SetLength(long value) => throw new NotSupportedException();
            public override void Write(byte[] buffer, int offset, int count) => _inner.Write(buffer, offset, count);
        }
    }
}
