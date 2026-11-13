using System;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;

namespace Teleop.RobotHost.Tests.ArmServer
{
    /// <summary>The arm-server wire's world kinds (docs/adr/0031 amendment 2): round trips, and what is refused.</summary>
    public class ArmServerWorldTests
    {
        private readonly byte[] _buffer = new byte[ArmServerWire.MaxBytes];

        private static ObjectPose Pose(byte id, float x) => new ObjectPose(id, ObjectPose.FlagKinematic, x, -0.25f, 0.15f, 0.5f, 0.5f, -0.5f, 0.5f);

        [Fact]
        public void EveryWorldKind_RoundTrips()
        {
            byte[] sha = new byte[32];
            sha[0] = 9;
            Assert.True(ArmServerWorld.TryEncodeWorldInfo(1, sha, new byte[] { 1, 2, 3 }, _buffer, out int n));
            Assert.Equal(ArmServerWorld.KindWorldInfo, ArmServerWire.KindOf(_buffer.AsSpan(0, n)));
            Assert.True(ArmServerWorld.TryDecodeWorldInfo(_buffer.AsSpan(0, n), out ReadOnlySpan<byte> gotSha, out ReadOnlySpan<byte> ids));
            Assert.Equal(9, gotSha[0]);
            Assert.Equal(new byte[] { 1, 2, 3 }, ids.ToArray());

            var objects = new ObjectPose[ArmServerWorld.MaxObjectsPerWorld];
            for (int i = 0; i < objects.Length; i++) objects[i] = Pose((byte)(i + 1), 0.01f * i);
            Assert.True(ArmServerWorld.TryEncodeWorld(2, 77, 5, 6, 1, 2, objects, _buffer, out n));
            Assert.True(n <= ArmServerWire.MaxBytes);
            var back = new ObjectPose[ArmServerWorld.MaxObjectsPerWorld];
            Assert.True(ArmServerWorld.TryDecodeWorld(_buffer.AsSpan(0, n), out long sensed, out uint snap, out uint rev, out byte part, out byte parts, back, out int count));
            Assert.Equal((77L, 5u, 6u, (byte)1, (byte)2, 16), (sensed, snap, rev, part, parts, count));
            Assert.Equal(objects[15].ToString(), back[15].ToString());
            Assert.Equal(ObjectPose.FlagKinematic, back[3].Flags);

            Assert.True(ArmServerWorld.TryEncodePlace(3, 88, 12, PlaceCause.Layout, Pose(4, 0.5f), _buffer, out n));
            Assert.True(ArmServerWorld.TryDecodePlace(_buffer.AsSpan(0, n), out long host, out uint request, out PlaceCause cause, out ObjectPose placed));
            Assert.Equal((88L, 12u, PlaceCause.Layout, (byte)4, 0.5f), (host, request, cause, placed.Id, placed.X));

            Assert.True(ArmServerWorld.TryEncodePlaced(4, 12, PlaceOutcome.Applied, 9, Pose(4, 0.5f), _buffer, out n));
            Assert.True(ArmServerWorld.TryDecodePlaced(_buffer.AsSpan(0, n), out request, out PlaceOutcome outcome, out rev, out placed));
            Assert.Equal((12u, PlaceOutcome.Applied, 9u, (byte)4), (request, outcome, rev, placed.Id));

            var contacts = new[] { new ArmServerContact(1, 254, 3, 7.5f) };
            Assert.True(ArmServerWorld.TryEncodeContacts(5, 99, contacts, _buffer, out n));
            var cback = new ArmServerContact[4];
            Assert.True(ArmServerWorld.TryDecodeContacts(_buffer.AsSpan(0, n), out sensed, cback, out count));
            Assert.Equal((99L, 1, (byte)254, 7.5f), (sensed, count, cback[0].Link, cback[0].ForceNewtons));
        }

        [Fact]
        public void BadWorldDatagrams_AreRefused()
        {
            var objects = new[] { Pose(1, 0.5f) };
            Assert.False(ArmServerWorld.TryEncodeWorld(1, 0, 0, 0, 2, 2, objects, _buffer, out _)); // part >= parts
            Assert.False(ArmServerWorld.TryEncodeWorld(1, 0, 0, 0, 0, 1, new ObjectPose[ArmServerWorld.MaxObjectsPerWorld + 1], _buffer, out _));
            Assert.True(ArmServerWorld.TryEncodeWorld(1, 0, 0, 0, 0, 1, objects, _buffer, out int n));
            var back = new ObjectPose[4];
            Assert.False(ArmServerWorld.TryDecodeWorld(_buffer.AsSpan(0, n - 1), out _, out _, out _, out _, out _, back, out _));
            Assert.False(ArmServerWorld.TryDecodePlace(_buffer.AsSpan(0, n), out _, out _, out _, out _)); // another kind
            Assert.True(ArmServerWorld.TryEncodePlace(1, 0, 1, PlaceCause.Reset, objects[0], _buffer, out n));
            _buffer[8 + 12] = 4; // no such cause
            Assert.False(ArmServerWorld.TryDecodePlace(_buffer.AsSpan(0, n), out _, out _, out _, out _));
            Assert.False(ArmServerWorld.TryEncodeWorldInfo(1, new byte[31], new byte[1], _buffer, out _));
        }

        [Fact]
        public void APoseIsValid_OnlyFiniteWithAUnitQuaternion()
        {
            Assert.True(new ObjectPose(1, 0, 0.5f, 0, 0.1f, 1, 0, 0, 0).IsValid());
            Assert.False(new ObjectPose(1, 0, float.NaN, 0, 0.1f, 1, 0, 0, 0).IsValid());
            Assert.False(new ObjectPose(1, 0, 0.5f, 0, 0.1f, 0.5f, 0, 0, 0).IsValid());
        }
    }
}
