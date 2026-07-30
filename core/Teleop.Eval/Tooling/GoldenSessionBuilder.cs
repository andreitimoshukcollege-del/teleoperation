using System.Numerics;
using Teleop.Core.Time;
using Teleop.Core.Types;
using Teleop.Eval.Recording;

namespace Teleop.Eval.Tooling
{
    /// <summary>
    /// Builds the committed golden <c>.tlog</c> fixture deterministically. Never hand-author
    /// this file -- generate it here, inspect the diff, and commit the result. Uses
    /// <see cref="ManualClock"/> throughout, never a wall clock, so the fixture's content never
    /// depends on how fast this machine happens to run.
    /// </summary>
    public static class GoldenSessionBuilder
    {
        private const ulong Seed = 12345UL;

        public static void Build(string path)
        {
            var clock = new ManualClock(ticksPerSecond: 10_000_000, startTicks: 0);
            using var writer = new TlogFileWriter(path);

            writer.WriteHeader(clock.TicksPerSecond, Seed);

            // Fifty frames of a steady, ordinary command sequence, each paired with a ground-
            // truth pose sample -- the normal case a predictor is scored against.
            for (uint i = 0; i < 50; i++)
            {
                clock.AdvanceTicks(100_000); // 10ms per frame

                var pose = new Pose(new Vector3(i * 0.01f, 0f, 1.0f), Quaternion.Identity);

                var frame = new CommandFrame(
                    sequence: i,
                    ackSequence: i == 0 ? 0u : i - 1,
                    captureTicks: clock.NowTicks,
                    pose: pose,
                    linearVelocity: new Vector3(0.1f, 0f, 0f),
                    angularVelocity: Vector3.Zero,
                    gripper: 0f);
                writer.WriteCommandFrame(frame);

                writer.WriteStampedPose(new Stamped<Pose>(clock.NowTicks, pose));
            }

            // A sequence near uint.MaxValue -- exercises the full uint range, not just small
            // counters, and a nonzero gripper value.
            clock.AdvanceTicks(100_000);
            writer.WriteCommandFrame(new CommandFrame(
                sequence: uint.MaxValue - 1,
                ackSequence: uint.MaxValue - 2,
                captureTicks: clock.NowTicks,
                pose: Pose.Identity,
                linearVelocity: Vector3.Zero,
                angularVelocity: Vector3.Zero,
                gripper: 0.5f));

            long t = clock.NowTicks;

            // Eight LatencyTrace records, each leaving exactly one optional field unset --
            // exercises TryGet's "false" path independently for every field the format carries.
            writer.WriteLatencyTrace(LatencyTrace.ForSequence(1)
                // CaptureTicks unset.
                .WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20).WithDownlinkSendTicks(t + 25)
                .WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45).WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(2)
                .WithCaptureTicks(t)
                // UplinkSendTicks unset.
                .WithRobotRecvTicks(t + 20).WithDownlinkSendTicks(t + 25)
                .WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45).WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(3)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10)
                // RobotRecvTicks unset.
                .WithDownlinkSendTicks(t + 25)
                .WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45).WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(4)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                // DownlinkSendTicks unset.
                .WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45).WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(5)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25)
                // OperatorRecvTicks unset.
                .WithPlayoutTicks(t + 45).WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(6)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25).WithOperatorRecvTicks(t + 40)
                // PlayoutTicks unset.
                .WithRenderTicks(t + 50)
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(7)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25).WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45)
                // RenderTicks unset.
                .WithPhotonTicks(t + 58).WithClockSync(100, 5));

            writer.WriteLatencyTrace(LatencyTrace.ForSequence(8)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25).WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45)
                .WithRenderTicks(t + 50)
                // PhotonTicks unset.
                .WithClockSync(100, 5));

            // Fully populated -- every field present.
            writer.WriteLatencyTrace(LatencyTrace.ForSequence(9)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25).WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45)
                .WithRenderTicks(t + 50).WithPhotonTicks(t + 58).WithClockSync(100, 5));

            // A negative ClockOffsetTicks -- the robot's clock is behind the operator's.
            writer.WriteLatencyTrace(LatencyTrace.ForSequence(10)
                .WithCaptureTicks(t).WithUplinkSendTicks(t + 10).WithRobotRecvTicks(t + 20)
                .WithDownlinkSendTicks(t + 25).WithOperatorRecvTicks(t + 40).WithPlayoutTicks(t + 45)
                .WithRenderTicks(t + 50).WithPhotonTicks(t + 58).WithClockSync(-500, 15));

            writer.WriteEndOfSession();
        }
    }
}
