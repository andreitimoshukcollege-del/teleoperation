using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using Teleop.Core.Types;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Tests.Backend;
using Teleop.RobotHost.World;

namespace Teleop.RobotHost.Tests.World
{
    /// <summary>RobotHost's world (docs/adr/0031 amendment 2): layouts, placements, edits, what reaches the headset, and the records.</summary>
    public class WorldHostTests
    {
        private const long Tps = 1_000_000;
        private const long Ms = Tps / 1000;

        private static readonly byte[] SceneBytes = File.ReadAllBytes(Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "..", "Scenes", "fr3-duo-bench.json"));
        private static readonly WorldScene Scene = JsonSerializer.Deserialize<SceneDocument>(SceneBytes, new JsonSerializerOptions { IncludeFields = true })!.ToScene();
        private static readonly byte[] Sha = SHA256.HashData(SceneBytes);

        private sealed class Rig
        {
            public readonly List<string> Records = new List<string>();
            public readonly List<byte[]> Sent = new List<byte[]>();
            public readonly WorldHost Host;
            public readonly IWorldBackend Backend;
            public long Now;

            public Rig(IWorldBackend? backend = null, ulong seed = 7)
            {
                Backend = backend ?? new StaticWorld(Scene, Sha, Tps);
                Host = new WorldHost(Scene, Sha, Backend, Tps, seed, Records.Add);
            }

            public void Tick(double ms, bool subscribed = true, bool motionOff = true)
            {
                Now += (long)(ms * Ms);
                (Backend as StaticWorld)?.Poll(Now);
                Host.Tick(Now, subscribed, motionOff, (b, n) => Sent.Add(b.AsSpan(0, n).ToArray()));
            }

            public WorldStatus LastStatus()
            {
                byte[] d = Sent.Last(x => x[0] == WorldCodec.StatusTag);
                Assert.True(WorldCodec.TryDecodeStatus(d, out WorldStatus s, new byte[32], out _));
                return s;
            }

            public ObjectPose[] LastFrame()
            {
                byte[] d = Sent.Last(x => x[0] == WorldCodec.FrameTag);
                var poses = new ObjectPose[8];
                Assert.True(WorldCodec.TryDecodeFrame(d, out _, out _, out _, out _, poses, out int n));
                return poses.Take(n).ToArray();
            }
        }

        [Fact]
        public void Layout1_IsTheSeedsLayout_AndReachesTheHeadsetAt50Hz()
        {
            var rig = new Rig(seed: 7);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            var expected = new RigidTransform[Scene.Objects.Length];
            SceneLayout.Draw(Scene, 7, expected);
            ObjectPose[] frame = rig.LastFrame();
            Assert.Equal(Scene.Objects.Length, frame.Length);
            for (int i = 0; i < frame.Length; i++)
            {
                Assert.Equal(expected[i].X, frame[i].X, 5);
                Assert.Equal(expected[i].Y, frame[i].Y, 5);
                Assert.Equal(expected[i].Qz, frame[i].Qz, 5);
            }

            for (int i = 0; i < 1000; i++) rig.Tick(1);
            int frames = rig.Sent.Count(x => x[0] == WorldCodec.FrameTag);
            Assert.InRange(frames, 52, 54); // 1.05 s at 50 Hz (one every 20 ms, the first at once), one part each for four objects
            WorldStatus s = rig.LastStatus();
            Assert.True(s.Running && s.EditsAllowed && !s.Physics);
            Assert.Equal((1u, 7UL, (byte)4), (s.LayoutNumber, s.LayoutSeed, s.ObjectCount));
            Assert.Contains(rig.Records, r => r.StartsWith("WL|", StringComparison.Ordinal) && r.EndsWith("|1|7", StringComparison.Ordinal));
            Assert.Equal(4, rig.Records.Count(r => r.StartsWith("WP|", StringComparison.Ordinal) && r.EndsWith("|applied", StringComparison.Ordinal)));
            Assert.Contains(rig.Records, r => r.StartsWith("WO|", StringComparison.Ordinal));
            Assert.Contains(rig.Records, r => r.StartsWith("WF|", StringComparison.Ordinal));
        }

        [Fact]
        public void NothingIsSent_WithoutASubscriber()
        {
            var rig = new Rig();
            for (int i = 0; i < 100; i++) rig.Tick(1, subscribed: false);
            Assert.Empty(rig.Sent);
        }

        [Fact]
        public void NewLayout_OnlyWithMotionOff_AndOncePerSequence()
        {
            var rig = new Rig(seed: 7);
            rig.Tick(1);
            rig.Host.HandleEdit(WorldEditOp.NewLayout, 5, default, motionOff: false, rig.Now);
            rig.Tick(30, motionOff: false);
            Assert.Equal((1u, 5u, (byte)PlaceOutcome.Busy), (rig.LastStatus().LayoutNumber, rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));

            rig.Host.HandleEdit(WorldEditOp.NewLayout, 6, default, motionOff: true, rig.Now);
            rig.Host.HandleEdit(WorldEditOp.NewLayout, 6, default, motionOff: true, rig.Now); // re-sent: applied once
            rig.Tick(30);
            WorldStatus s = rig.LastStatus();
            Assert.Equal((2u, 8UL, 6u, (byte)PlaceOutcome.Applied), (s.LayoutNumber, s.LayoutSeed, s.EditSequence, s.EditOutcome));
            var expected = new RigidTransform[Scene.Objects.Length];
            SceneLayout.Draw(Scene, 8, expected);
            Assert.Equal(expected[0].X, rig.LastFrame()[0].X, 5);
        }

        [Fact]
        public void AMove_IsAppliedOnTheTable_AndRefusedOffIt()
        {
            var rig = new Rig();
            rig.Tick(1);
            var on = new ObjectPose(4, 0, 0.45f, -0.05f, 0.5f, 1, 0, 0, 0);
            rig.Host.HandleEdit(WorldEditOp.Move, 1, on, motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.Equal((1u, (byte)PlaceOutcome.Applied), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            ObjectPose can = rig.LastFrame().Single(p => p.Id == 4);
            Assert.Equal((0.45f, 0.10f), (can.X, can.Z)); // set upright on the table, whatever height was asked

            rig.Host.HandleEdit(WorldEditOp.Move, 2, new ObjectPose(4, 0, 1.5f, 0f, 0.1f, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.Equal((2u, (byte)PlaceOutcome.OutsideRegion), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            Assert.Contains(rig.Records, r => r.StartsWith("WE|", StringComparison.Ordinal) && r.EndsWith("|outside-region", StringComparison.Ordinal));
        }

        /// <summary>A backend that refuses each object's first placement as overlapping an arm, then applies.</summary>
        private sealed class Refusing : IWorldBackend
        {
            private readonly Queue<(uint, PlaceOutcome, ObjectPose)> _placed = new Queue<(uint, PlaceOutcome, ObjectPose)>();
            private readonly HashSet<byte> _refused = new HashSet<byte>();
            public readonly List<ObjectPose> Asked = new List<ObjectPose>();
            public bool HasWorld => true;
            public ReadOnlySpan<byte> SceneSha256 => Sha;
            public bool Physics => true;

            public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
            {
                Asked.Add(pose);
                _placed.Enqueue((request, _refused.Add(pose.Id) ? PlaceOutcome.OverlapsArm : PlaceOutcome.Applied, pose));
            }

            public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision)
            {
                count = 0;
                sensedTicks = 0;
                revision = 0;
                return false;
            }

            public bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose)
            {
                request = 0;
                outcome = 0;
                pose = default;
                if (_placed.Count == 0) return false;
                (request, outcome, pose) = _placed.Dequeue();
                return true;
            }
        }

        [Fact]
        public void ARefusedPlacement_IsRedrawnWithinTheObjectsJitter()
        {
            var backend = new Refusing();
            var rig = new Rig(backend);
            rig.Tick(1);
            rig.Tick(1);
            Assert.Equal(8, backend.Asked.Count); // four refused, four redrawn
            for (int i = 0; i < 4; i++)
            {
                ObjectPose second = backend.Asked[4 + i];
                SceneObject o = Scene.Objects[Scene.IndexOf(second.Id)];
                Assert.True(Scene.FitsOnTable(Scene.IndexOf(second.Id), second.X, second.Y));
                Assert.InRange(Math.Abs(second.X - o.NominalX), 0, o.JitterX + 1e-6);
            }

            Assert.Equal(4, rig.Host.PlacementsRefused);
        }

        [Fact]
        public void ABackendRunningAnotherScene_KeepsTheWorldOff()
        {
            var other = new StaticWorld(Scene, new byte[32], Tps);
            var host = new WorldHost(Scene, Sha, other, Tps, 1);
            var sent = new List<byte[]>();
            for (long t = 1; t < 100; t++)
            {
                other.Poll(t * Ms);
                host.Tick(t * Ms, true, true, (b, n) => sent.Add(b.AsSpan(0, n).ToArray()));
            }

            Assert.False(host.Running);
            Assert.Equal(0u, host.LayoutNumber);
            Assert.DoesNotContain(sent, d => d[0] == WorldCodec.FrameTag);
            Assert.True(WorldCodec.TryDecodeStatus(sent.Last(), out WorldStatus s, new byte[32], out _));
            Assert.False(s.Running);
        }

        /// <summary>The static world, counting what it is asked to place.</summary>
        private sealed class Counting : IWorldBackend
        {
            private readonly StaticWorld _inner = new StaticWorld(Scene, Sha, Tps);
            public readonly List<(PlaceCause Cause, ObjectPose Pose)> Asked = new List<(PlaceCause, ObjectPose)>();
            public bool HasWorld => true;
            public ReadOnlySpan<byte> SceneSha256 => Sha;
            public bool Physics => false;
            public void Poll(long now) => _inner.Poll(now);

            public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
            {
                Asked.Add((cause, pose));
                _inner.Place(request, cause, pose, hostTicks);
            }

            public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision) =>
                _inner.TryTakeSnapshot(poses, out count, out sensedTicks, out revision);

            public bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose) => _inner.TryTakePlaced(out request, out outcome, out pose);
        }

        [Fact]
        public void AMove_ReSentUntilEchoed_IsPlacedOnce_AndNotWhileMotionIsOn()
        {
            var backend = new Counting();
            var rig = new Rig(backend);
            rig.Tick(1);
            backend.Poll(rig.Now);
            int layout = backend.Asked.Count;
            var move = new ObjectPose(4, 0, 0.45f, -0.05f, 0.1f, 1, 0, 0, 0);
            for (int k = 0; k < 5; k++)
            {
                rig.Host.HandleEdit(WorldEditOp.Move, 9, move, motionOff: true, rig.Now); // the headset re-sends until echoed
                rig.Tick(5);
            }

            Assert.Single(backend.Asked.Skip(layout), a => a.Cause == PlaceCause.Headset);
            Assert.Equal((9u, (byte)PlaceOutcome.Applied), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));

            rig.Host.HandleEdit(WorldEditOp.Move, 10, new ObjectPose(4, 0, 0.50f, 0.05f, 0.1f, 1, 0, 0, 0), motionOff: false, rig.Now);
            rig.Tick(30, motionOff: false);
            Assert.Equal((10u, (byte)PlaceOutcome.Busy), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            Assert.Single(backend.Asked.Skip(layout), a => a.Cause == PlaceCause.Headset); // nothing more was placed
        }

        /// <summary>A backend that has the world and never answers a placement.</summary>
        private sealed class Silent : IWorldBackend
        {
            public bool HasWorld => true;
            public ReadOnlySpan<byte> SceneSha256 => Sha;
            public bool Physics => true;
            public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks) { }

            public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision)
            {
                count = 0;
                sensedTicks = 0;
                revision = 0;
                return false;
            }

            public bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose)
            {
                request = 0;
                outcome = 0;
                pose = default;
                return false;
            }
        }

        [Fact]
        public void AMoveTheBackendNeverAnswers_TimesOut()
        {
            var rig = new Rig(new Silent());
            rig.Tick(1);
            rig.Tick(WorldHost.PlaceTimeoutSeconds * 1000 + 50); // the layout's placements time out first
            rig.Host.HandleEdit(WorldEditOp.Move, 3, new ObjectPose(4, 0, 0.45f, -0.05f, 0.1f, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(WorldHost.PlaceTimeoutSeconds * 500);
            Assert.NotEqual((3u, (byte)PlaceOutcome.TimedOut), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            rig.Tick(WorldHost.PlaceTimeoutSeconds * 600);
            Assert.Equal((3u, (byte)PlaceOutcome.TimedOut), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            Assert.Contains(rig.Records, r => r.StartsWith("WE|", StringComparison.Ordinal) && r.EndsWith("|timed-out", StringComparison.Ordinal));
        }

        [Fact]
        public void WithAScene_ACommandIntoAnObstacleIsAccepted()
        {
            // The plant (and the headset's IK) know the robot's body, never the world's objects: an arm meets an obstacle in
            // Isaac's physics, and the contact shows as command against measured (docs/adr/0031 amendment 2).
            SerialChainProfile profile = SerialChainTestProfile.Fr3Duo();
            TrajectoryShaperConfig shaper = SerialChainShaping.ForProfile(profile);
            var backend = new Fr3SimBackend(shaper, SerialChainHost.StartPose(profile, shaper), Tps);
            var model = SerialChainProfileJson.LoadCollision(Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "fr3-duo.collision.json"));
            double[] start = SerialChainHost.StartPose(profile, shaper);
            double[] byProfile = profile.Arms.SelectMany(a => a.Joints.Select(j => j.MotorId).Concat(a.HasGripper ? new[] { a.Gripper.MotorId } : new byte[0]))
                .Select(id => start[shaper.IndexOf(id)]).ToArray();
            var plant = new SerialChainPlant(profile, backend, model, byProfile);
            var world = new WorldHost(Scene, Sha, new StaticWorld(Scene, Sha, Tps), Tps, 1);

            // A pose that puts the left gripper down onto the table in front of it, where layouts put obstacles.
            double[] q = { 0.3, 0.55, 0, -2.0, 0, 2.55, 0.785 };
            RigidTransform tool = SerialChainKinematics.Tool(profile.Arms[0], q);
            Assert.InRange(tool.X, Scene.Table.XMin, Scene.Table.XMax);
            Assert.InRange(tool.Z, Scene.Table.Top - 0.05, Scene.Table.Top + 0.30);
            JointTarget[] command = q.Select((v, j) => new JointTarget((byte)(j + 1), (float)v, 0f)).ToArray();
            Assert.Equal(JointCommandOutcome.Accepted, plant.CommandJointTargets(command, captureTicks: 1, nowTicks: 1));
            Assert.NotNull(world);
        }

        [Fact]
        public void AResize_IsAppliedByTheBackend_ReachesTheHeadsetAsSizes_AndIsRecorded()
        {
            var rig = new Rig();
            rig.Tick(1);
            int can = Scene.IndexOf(4);
            rig.Host.HandleEdit(WorldEditOp.Resize, 1, new ObjectPose(4, 0, 1.25f, 0, 0, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.Equal((1u, (byte)PlaceOutcome.Applied), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            Assert.Equal(1.25f, rig.Host.ScaleOf(can));
            byte[] d = rig.Sent.Last(x => x[0] == WorldCodec.SizesTag);
            var ids = new byte[24];
            var scales = new float[24];
            Assert.True(WorldCodec.TryDecodeSizes(d, out uint revision, ids, scales, out int n));
            Assert.Equal((1u, 4), (revision, n));
            Assert.Equal(1.25f, scales[Array.IndexOf(ids, (byte)4, 0, n)]);
            ObjectPose stood = rig.LastFrame().Single(p => p.Id == 4);
            Assert.Equal(Scene.Table.Top + Scene.Objects[can].SizeZ * 1.25 / 2, stood.Z, 5); // stood up at its new height
            Assert.Contains(rig.Records, r => r.StartsWith("WZ|", StringComparison.Ordinal) && r.Contains("4:1.25", StringComparison.Ordinal));
            Assert.Contains(rig.Records, r => r.StartsWith("WE|", StringComparison.Ordinal) && r.Contains("|resize|", StringComparison.Ordinal) && r.EndsWith("|applied", StringComparison.Ordinal));
        }

        [Fact]
        public void AResize_IsRefusedOutOfRange_WithMotionOn_AndWhereItWouldReachAnotherObject()
        {
            var rig = new Rig();
            rig.Tick(1);
            rig.Host.HandleEdit(WorldEditOp.Resize, 1, new ObjectPose(4, 0, 5f, 0, 0, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.Equal((byte)PlaceOutcome.InvalidPose, rig.LastStatus().EditOutcome);
            rig.Host.HandleEdit(WorldEditOp.Resize, 2, new ObjectPose(4, 0, 1.5f, 0, 0, 1, 0, 0, 0), motionOff: false, rig.Now);
            rig.Tick(30, motionOff: false);
            Assert.Equal((2u, (byte)PlaceOutcome.Busy), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome));
            rig.Host.HandleEdit(WorldEditOp.Resize, 3, new ObjectPose(4, 0, (float)WorldScene.MaxObjectScale, 0, 0, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.True(rig.LastStatus().EditOutcome is (byte)PlaceOutcome.Applied or (byte)PlaceOutcome.OverlapsObject or (byte)PlaceOutcome.OutsideRegion); // at the top step: wherever it fits, or refused
            rig.Host.HandleEdit(WorldEditOp.Resize, 4, new ObjectPose(4, 0, 1.5f, 0, 0, 1, 0, 0, 0), motionOff: true, rig.Now);
            rig.Tick(30);
            Assert.Equal((4u, (byte)PlaceOutcome.InvalidPose), (rig.LastStatus().EditSequence, rig.LastStatus().EditOutcome)); // not a step
            Assert.Equal(rig.LastStatus().EditOutcome == (byte)PlaceOutcome.InvalidPose && rig.Records.Exists(r => r.StartsWith("WE|", StringComparison.Ordinal) && r.Contains("|resize|", StringComparison.Ordinal) && r.EndsWith("|applied", StringComparison.Ordinal)) ? (float)WorldScene.MaxObjectScale : 1f,
                rig.Host.ScaleOf(Scene.IndexOf(4))); // the invalid one changed nothing; the top step only if it was applied
        }
    }
}
