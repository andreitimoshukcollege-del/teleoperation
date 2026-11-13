using System;
using Teleop.RobotArm.Operator;
using Xunit;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The live link health the headset shows (docs/metrics.md §3, "Live link health").</summary>
    public sealed class LiveLinkMeterTests
    {
        private const long Tps = 1_000_000, RobotTps = 1_000_000_000;

        /// <summary>Frames every 20 ms for <paramref name="n"/> frames, each <paramref name="owdMs"/> on the way, the robot's clock 7 s ahead.</summary>
        private static void Feed(LiveLinkMeter m, int n, double owdMs, Func<int, bool>? lost = null, Func<int, double>? extraMs = null)
        {
            for (int i = 0; i < n; i++)
            {
                if (lost != null && lost(i)) continue;
                long sendOp = i * 20 * Tps / 1000;
                long arrival = sendOp + (long)((owdMs + (extraMs?.Invoke(i) ?? 0)) * Tps / 1000);
                m.OnFrame((uint)(1000 + i), (sendOp + 7 * Tps) * (RobotTps / Tps), RobotTps, arrival, sendOp);
            }
        }

        [Fact]
        public void ASteadyLink_HasItsDelay_NoJitter_NoLoss()
        {
            var m = new LiveLinkMeter(Tps);
            Feed(m, 200, 35);
            m.Trim(200 * 20 * Tps / 1000);
            Assert.Equal(35, m.LagMs, 3);
            Assert.Equal(0, m.JitterMs, 6);
            Assert.Equal(0, m.LossFraction, 6);
        }

        [Fact]
        public void LostFrames_AreCountedBySequence_OverTheLastFiveSeconds()
        {
            var m = new LiveLinkMeter(Tps);
            Feed(m, 500, 10, lost: i => i % 10 == 3); // 10 s, one frame in ten lost
            m.Trim(500 * 20 * Tps / 1000);
            Assert.InRange(m.Frames, 220, 230); // the last 5 s only
            Assert.Equal(0.1, m.LossFraction, 2);
        }

        [Fact]
        public void Jitter_IsRfc3550s_AndALateOrDuplicateFrameIsNotCountedTwice()
        {
            var m = new LiveLinkMeter(Tps);
            Feed(m, 400, 10, extraMs: i => i % 2 == 0 ? 0 : 4); // alternating 0 / 4 ms extra: |D| = 4 ms every frame
            Assert.Equal(4, m.JitterMs, 1);
            int before = m.Frames;
            m.OnFrame(1000 + 398, 0, RobotTps, 1, long.MinValue); // a repeat of an old sequence
            Assert.Equal(before, m.Frames);
        }

        [Fact]
        public void ARestartedRobot_RestartsTheWindow_AndLagWaitsForSync()
        {
            var m = new LiveLinkMeter(Tps);
            Feed(m, 50, 10);
            m.OnFrame(5, 0, RobotTps, 2 * Tps, long.MinValue); // the sequence starts again: unsynced
            Assert.Equal(1, m.Frames);
            Assert.True(double.IsNaN(m.LagMs));
        }

        [Fact]
        public void Frames_AllocateNothing()
        {
            var m = new LiveLinkMeter(Tps);
            Feed(m, 300, 10);
            int i = 300;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() =>
            {
                m.OnFrame((uint)(1000 + i), i * 20_000_000L, RobotTps, i * 20_000L + 10_000, i * 20_000L);
                m.Trim(i * 20_000L);
                _ = m.LagMs + m.LossFraction + m.JitterMs;
                i++;
            }, iterations: 1000);
        }
    }
}
