using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

namespace Teleop.RobotArm.Tests.World
{
    /// <summary>The world on the joint port (docs/adr/0031 amendment 2), and the headset's view of it.</summary>
    public class WorldWireTests
    {
        private static ObjectPose P(byte id, float x) => new ObjectPose(id, 0, x, 0.1f, 0.15f, 1, 0, 0, 0);

        [Fact]
        public void AFullFrame_FitsTheJointPortAndRoundTrips()
        {
            var objects = Enumerable.Range(1, WorldCodec.MaxObjectsPerFrame).Select(i => P((byte)i, 0.05f * i)).ToArray();
            var buffer = new byte[JointDatagram.MaxBytes];
            Assert.True(WorldCodec.TryEncodeFrame(7, 123456, 1, 3, objects, buffer, out int n));
            Assert.Equal(249, n);
            Assert.True(n <= JointDatagram.MaxBytes);
            var back = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
            Assert.True(WorldCodec.TryDecodeFrame(buffer.AsSpan(0, n), out uint snap, out long sensed, out byte part, out byte parts, back, out int count));
            Assert.Equal((7u, 123456L, (byte)1, (byte)3, 8), (snap, sensed, part, parts, count));
            Assert.Equal(objects[7].ToString(), back[7].ToString());
            Assert.False(WorldCodec.TryDecodeFrame(buffer.AsSpan(0, n - 1), out _, out _, out _, out _, back, out _));
            Assert.False(WorldCodec.TryEncodeFrame(1, 1, 0, 1, new ObjectPose[9], buffer, out _));
        }

        [Fact]
        public void AnEdit_AndAStatus_RoundTrip_AndOthersAreRefused()
        {
            var buffer = new byte[JointDatagram.MaxBytes];
            Assert.True(WorldCodec.TryEncodeEdit(WorldEditOp.Move, 3, 9, -5, P(3, 0.6f), buffer, out int n));
            Assert.Equal(WorldCodec.EditBytes, n);
            Assert.True(WorldCodec.TryDecodeEdit(buffer.AsSpan(0, n), out WorldEditOp op, out uint seq, out long ticks, out ObjectPose pose));
            Assert.Equal((WorldEditOp.Move, 9u, -5L, (byte)3, 0.6f), (op, seq, ticks, pose.Id, pose.X));
            Assert.False(WorldCodec.TryEncodeEdit(WorldEditOp.Move, 3, 0, 0, P(3, 0), buffer, out _)); // sequence 0 is "none"

            byte[] sha = Enumerable.Range(0, 32).Select(i => (byte)i).ToArray();
            var status = new WorldStatus(WorldStatus.FlagRunning | WorldStatus.FlagPhysics, 4, 2, 0xABCDEF, 9, 1, 3);
            Assert.True(WorldCodec.TryEncodeStatus(status, sha, 77, buffer, out n));
            Assert.Equal(WorldCodec.StatusBytes, n);
            var shaBack = new byte[32];
            Assert.True(WorldCodec.TryDecodeStatus(buffer.AsSpan(0, n), out WorldStatus got, shaBack, out long sent));
            Assert.Equal(status, got);
            Assert.Equal(sha, shaBack);
            Assert.Equal(77L, sent);
            buffer[1] = 2;
            Assert.False(WorldCodec.TryDecodeStatus(buffer.AsSpan(0, n), out _, shaBack, out _));
        }

        [Fact]
        public void TheState_KeepsTheNewestPose_AndIgnoresLateAndForeignOnes()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var sha = new byte[32];
            var state = new WorldState(scene, sha);
            Assert.False(state.TryGet(1, out _, out _));
            Assert.True(state.Apply(P(1, 0.5f), 100));
            Assert.False(state.Apply(P(1, 0.4f), 90)); // a reordered, older part
            Assert.True(state.TryGet(1, out ObjectPose pose, out long sensed));
            Assert.Equal((0.5f, 100L), (pose.X, sensed));
            Assert.False(state.Apply(P(200, 0.5f), 120));
            Assert.Equal((1L, 1L), (state.Superseded, state.UnknownIds));

            Assert.False(state.SceneMismatch);
            var other = new byte[32];
            other[5] = 1;
            state.ApplyStatus(new WorldStatus(WorldStatus.FlagRunning, 4, 1, 1, 0, 0, 0), other);
            Assert.True(state.SceneMismatch);
            state.Reset();
            Assert.False(state.TryGet(1, out _, out _));
            Assert.False(state.HasStatus);
        }

        [Fact]
        public void ApplyingPoses_AllocatesNothing()
        {
            var state = new WorldState(SceneDocumentTests.LoadShipped(), new byte[32]);
            ObjectPose p = P(2, 0.5f);
            for (int i = 0; i < 200; i++) state.Apply(p, i);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 200; i < 5000; i++) state.Apply(p, i);
            Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        }

        [Fact]
        public void Sizes_RoundTrip_AndAResizeEdit_CarriesItsFactorInX()
        {
            byte[] ids = { 1, 4 };
            float[] scales = { 1f, 1.75f };
            var buffer = new byte[256];
            Assert.True(WorldCodec.TryEncodeSizes(9, ids, scales, buffer, out int n));
            Assert.Equal(WorldCodec.SizesBytes(2), n);
            var gotIds = new byte[24];
            var gotScales = new float[24];
            Assert.True(WorldCodec.TryDecodeSizes(buffer.AsSpan(0, n), out uint revision, gotIds, gotScales, out int count));
            Assert.Equal((9u, 2, (byte)4, 1.75f), (revision, count, gotIds[1], gotScales[1]));
            Assert.False(WorldCodec.TryDecodeSizes(buffer.AsSpan(0, n - 1), out _, gotIds, gotScales, out _));

            Assert.True(WorldCodec.TryEncodeEdit(WorldEditOp.Resize, 4, 3, 77, new ObjectPose(4, 0, 1.5f, 0, 0, 1, 0, 0, 0), buffer, out n));
            Assert.True(WorldCodec.TryDecodeEdit(buffer.AsSpan(0, n), out WorldEditOp op, out uint sequence, out _, out ObjectPose pose));
            Assert.Equal((WorldEditOp.Resize, 3u, (byte)4, 1.5f), (op, sequence, pose.Id, pose.X));
        }

        [Fact]
        public void TheState_TakesSizes_IgnoringForeignIdsAndFactorsOutOfRange()
        {
            WorldScene scene = SceneDocumentTests.LoadShipped();
            var world = new WorldState(scene, new byte[32]);
            Assert.Equal(1f, world.Scale(4));
            world.ApplySizes(3, new byte[] { 4, 2, 200 }, new[] { 2f, 9f, 2f });
            Assert.Equal((2f, 1f, 1f, 3u), (world.Scale(4), world.Scale(2), world.Scale(200), world.SizesRevision));
            world.Reset();
            Assert.Equal(1f, world.Scale(4));
        }
    }
}
