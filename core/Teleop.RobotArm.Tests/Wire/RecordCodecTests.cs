using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    /// <summary>Scenario recording on the joint port (docs/adr/0033): the three datagrams, and the headset's state from them.</summary>
    public class RecordCodecTests
    {
        [Fact]
        public void ARequest_RoundTrips_AndABadOneIsRefused()
        {
            var b = new byte[RecordCodec.RequestBytes];
            Assert.True(RecordCodec.TryEncodeRequest(RecordOp.Start, 3, 7, 42, 123456789L, b, out int n));
            Assert.Equal(RecordCodec.RequestBytes, n);
            Assert.True(RecordCodec.TryDecodeRequest(b, out RecordOp op, out byte scenario, out ushort revision, out uint sequence, out long ticks));
            Assert.Equal((RecordOp.Start, (byte)3, (ushort)7, 42u, 123456789L), (op, scenario, revision, sequence, ticks));

            Assert.False(RecordCodec.TryEncodeRequest(RecordOp.Stop, 0, 0, 0, 0, b, out _)); // sequence 0 is "none"
            byte[] bad = (byte[])b.Clone();
            bad[2] = 9;
            Assert.False(RecordCodec.TryDecodeRequest(bad, out _, out _, out _, out _, out _));
            bad = (byte[])b.Clone();
            bad[1] = 2;
            Assert.False(RecordCodec.TryDecodeRequest(bad, out _, out _, out _, out _, out _));
            Assert.False(RecordCodec.TryDecodeRequest(b.AsSpan(0, 19), out _, out _, out _, out _, out _));
            // Another joint-port datagram is not mistaken for one.
            Assert.False(RecordCodec.TryDecodeRequest(new byte[] { WorldCodec.EditTag, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, out _, out _, out _, out _, out _));
        }

        [Fact]
        public void AStatus_RoundTrips()
        {
            var s = new RecordStatus(RecordState.Recording, RecordOutcome.Started, 9, 2, 5, 3, 61_234, 12, 11, 412);
            var b = new byte[RecordCodec.StatusBytes];
            Assert.True(RecordCodec.TryEncodeStatus(s, 777, b, out int n));
            Assert.Equal(32, n);
            Assert.True(RecordCodec.TryDecodeStatus(b, out RecordStatus back, out long sent));
            Assert.Equal(s, back);
            Assert.Equal(777, sent);
            b[2] = 7;
            Assert.False(RecordCodec.TryDecodeStatus(b, out _, out _));
        }

        [Theory]
        [InlineData("free-play", true)]
        [InlineData("knock_over.2", true)]
        [InlineData("a", true)]
        [InlineData("", false)]
        [InlineData("has space", false)]
        [InlineData("_discarded", false)]
        [InlineData(".hidden", false)]
        [InlineData("..", false)]
        [InlineData("slash/name", false)]
        [InlineData("abcdefghijklmnopqrstuvwxy", false)] // 25
        public void AScenarioName_NamesAFolderSafely(string name, bool valid) => Assert.Equal(valid, RecordCodec.IsValidName(name));

        [Fact]
        public void AListOf12_ComesInTwoParts_AndTheHeadsetAssemblesIt_InAnyOrder()
        {
            string[] names = Enumerable.Range(1, 12).Select(i => $"scenario-{i}").ToArray();
            var p0 = new byte[256];
            var p1 = new byte[256];
            Assert.True(RecordCodec.TryEncodeList(4, 0, 2, names, 0, 8, p0, out int n0));
            Assert.True(RecordCodec.TryEncodeList(4, 1, 2, names, 8, 4, p1, out int n1));
            Assert.True(n0 <= 256 && n1 <= 256);

            var state = new RecordingState();
            Assert.True(state.ApplyListPart(p1.AsSpan(0, n1)));
            Assert.False(state.HasList); // half a list is no list
            Assert.True(state.ApplyListPart(p0.AsSpan(0, n0)));
            Assert.True(state.HasList);
            Assert.Equal(4, state.ListRevision);
            Assert.Equal(names, Enumerable.Range(0, state.Count).Select(state.Name).ToArray());

            // A new revision replaces it once whole.
            Assert.True(RecordCodec.TryEncodeList(5, 0, 1, new[] { "only" }, 0, 1, p0, out n0));
            Assert.True(state.ApplyListPart(p0.AsSpan(0, n0)));
            Assert.Equal(1, state.Count);
            Assert.Equal("only", state.Name(0));
            Assert.Equal("", state.Name(1));
        }

        [Fact]
        public void TheLongestPart_FitsAJointDatagram()
        {
            string[] names = Enumerable.Range(0, 8).Select(i => new string((char)('a' + i), RecordCodec.MaxNameLength)).ToArray();
            var b = new byte[JointDatagram.MaxBytes];
            Assert.True(RecordCodec.TryEncodeList(1, 0, 2, names, 0, 8, b, out int n));
            Assert.Equal(RecordCodec.ListHeaderBytes + 8 * 25, n);
            Assert.False(RecordCodec.TryEncodeList(1, 0, 1, new[] { "bad name" }, 0, 1, b, out _));
        }

        [Fact]
        public void TheListAgain_AtTheSameRevision_AllocatesNothing()
        {
            var b = new byte[256];
            Assert.True(RecordCodec.TryEncodeList(2, 0, 1, new[] { "a", "b" }, 0, 2, b, out int n));
            var state = new RecordingState();
            state.ApplyListPart(b.AsSpan(0, n));
            var status = new RecordStatus(RecordState.Idle, RecordOutcome.None, 0, 255, 2, 2, 0, 0, 0, 100);
            long i = 0;
            Teleop.Core.Tests.TestSupport.AllocationAssert.Zero(() =>
            {
                state.ApplyListPart(b.AsSpan(0, n));
                state.ApplyStatus(status, i++);
            }, iterations: 1000);
            Assert.True(state.ListCurrent);
        }
    }
}
