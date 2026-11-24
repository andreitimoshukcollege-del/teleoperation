using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotArm.Tests.Wire
{
    /// <summary>
    /// A mixed take's planned moves and replay groups, joint-drive to RobotHost (docs/adr/0038 decisions 1 and 2), which records
    /// them as MV and RP.
    /// </summary>
    public class ScriptMoveCodecTests
    {
        [Fact]
        public void AMove_RoundTripsExactly()
        {
            var m = new ScriptMove(7, 23, ScriptMoveKind.Aim, 41, -12.5f, -9.25f, -0.0625f, 0.5f, 0.75f, 1.25f);
            var buffer = new byte[ScriptMoveCodec.Size];
            Assert.True(ScriptMoveCodec.TryEncode(m, 0xDEADBEEF, -123456789012, buffer, out int written));
            Assert.Equal(44, written);
            Assert.True(ScriptMoveCodec.IsScriptMove(buffer));
            Assert.True(ScriptMoveCodec.TryDecode(buffer, out ScriptMove back, out uint seq, out long ticks));
            Assert.Equal(m, back);
            Assert.Equal((0xDEADBEEFu, -123456789012L), (seq, ticks));
            // Nothing else on the joint port takes it for one of its own.
            Assert.False(ScriptEventCodec.TryDecode(buffer, out _, out _, out _));
            Assert.False(ReplayCodec.TryDecode(buffer, out _, out _, out _));
            Assert.False(JointCommandCodec.TryDecode(buffer, out _, out _, new JointTarget[JointCommandCodec.MaxJointsPerMessage], out _));
            Assert.False(TrainingTaskCodec.TryDecode(buffer, out _, out _, out _));
            Assert.False(RecordCodec.TryDecodeRequest(buffer, out _, out _, out _, out _, out _));
        }

        [Fact]
        public void TheMoveLayout_IsFixed()
        {
            var m = new ScriptMove(258, 3, ScriptMoveKind.Pass, 9, 1f, 0f, 0f, 0f, 0f, 0f);
            var buffer = new byte[ScriptMoveCodec.Size];
            Assert.True(ScriptMoveCodec.TryEncode(m, 5, 6, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x8D, 1, 3, 9, 2, 1, 3, 0,
                5, 0, 0, 0,
                0x00, 0x00, 0x80, 0x3F, // 1.0f
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                6, 0, 0, 0, 0, 0, 0, 0,
            }, buffer);
        }

        [Fact]
        public void AFreeMove_CarriesNaNForWhatItHasNot()
        {
            var m = new ScriptMove(1, 1, ScriptMoveKind.Free, 0, float.NaN, float.NaN, 0f, float.NaN, float.NaN, 0.8f);
            var buffer = new byte[ScriptMoveCodec.Size];
            Assert.True(ScriptMoveCodec.TryEncode(m, 1, 1, buffer, out _));
            Assert.True(ScriptMoveCodec.TryDecode(buffer, out ScriptMove back, out _, out _));
            Assert.True(float.IsNaN(back.OffsetMm) && float.IsNaN(back.ContactSeconds));
            Assert.Equal(m, back);
            Assert.Equal(new[] { "free", "aim", "pass", "pause", "reverse" },
                new[] { ScriptMoveKind.Free, ScriptMoveKind.Aim, ScriptMoveKind.Pass, ScriptMoveKind.Pause, ScriptMoveKind.Reverse }.Select(ScriptMoveCodec.Name));
        }

        [Fact]
        public void AnAccident_RoundTripsWithItsObject_AndSaysItsKind()
        {
            var kinds = new[] { ScriptMoveKind.Jerk, ScriptMoveKind.Overshoot, ScriptMoveKind.WrongWay, ScriptMoveKind.Slip, ScriptMoveKind.Rush };
            Assert.Equal(new[] { "jerk", "overshoot", "wrong-way", "slip", "rush" }, kinds.Select(ScriptMoveCodec.Name));
            Assert.Equal(new byte[] { 6, 7, 8, 9, 10 }, kinds.Select(k => (byte)k));
            var buffer = new byte[ScriptMoveCodec.Size];
            foreach (ScriptMoveKind k in kinds)
            {
                Assert.True(ScriptMoveCodec.IsAccident(k));
                var m = new ScriptMove(4, 19, k, k == ScriptMoveKind.Rush ? (byte)33 : (byte)0, -8f, -11.5f, -0.25f, 0.375f, 0.5f, 0.875f);
                Assert.True(ScriptMoveCodec.TryEncode(m, 9, 10, buffer, out _));
                Assert.Equal((byte)k, buffer[2]);
                Assert.True(ScriptMoveCodec.TryDecode(buffer, out ScriptMove back, out _, out _));
                Assert.Equal(m, back);
            }

            foreach (ScriptMoveKind k in new[] { ScriptMoveKind.Free, ScriptMoveKind.Aim, ScriptMoveKind.Pass, ScriptMoveKind.Pause, ScriptMoveKind.Reverse }) Assert.False(ScriptMoveCodec.IsAccident(k));
        }

        [Theory]
        [InlineData(2, 0)]    // no kind
        [InlineData(2, 11)]   // a kind this build does not define (6-10 are the accidents, docs/adr/0038 "M, motion")
        [InlineData(1, 2)]    // another version
        [InlineData(0, 0x8C)] // another message
        public void UnknownMoveBytes_AreRefused(int index, byte value)
        {
            var buffer = new byte[ScriptMoveCodec.Size];
            Assert.True(ScriptMoveCodec.TryEncode(new ScriptMove(1, 1, ScriptMoveKind.Free, 0, 0f, 0f, 0f, 0f, 0f, 0f), 0, 0, buffer, out _));
            buffer[index] = value;
            Assert.False(ScriptMoveCodec.TryDecode(buffer, out _, out _, out _));
        }

        [Fact]
        public void AShortMove_IsRefused_AndNothingIsWrittenIntoAShortBuffer()
        {
            var buffer = new byte[ScriptMoveCodec.Size];
            Assert.True(ScriptMoveCodec.TryEncode(new ScriptMove(1, 1, ScriptMoveKind.Free, 0, 0f, 0f, 0f, 0f, 0f, 0f), 0, 0, buffer, out _));
            Assert.False(ScriptMoveCodec.TryDecode(buffer.AsSpan(0, ScriptMoveCodec.Size - 1), out _, out _, out _));
            var shortBuffer = new byte[ScriptMoveCodec.Size - 1];
            Assert.False(ScriptMoveCodec.TryEncode(default, 0, 0, shortBuffer, out int written));
            Assert.Equal(ScriptMoveCodec.Size, written);
            Assert.All(shortBuffer, b => Assert.Equal(0, b));
        }

        [Fact]
        public void AReplayRecord_RoundTripsExactly_AndSaysItsEdit()
        {
            var records = new[]
            {
                ReplayRecord.Source(100003),
                new ReplayRecord(ReplayRole.Replay, 100003, ReplayEdit.Remove, 33),
                new ReplayRecord(ReplayRole.Replay, 100003, ReplayEdit.Shift, 34, 0.025f, -0.0125f),
                new ReplayRecord(ReplayRole.Replay, ulong.MaxValue, ReplayEdit.Move, 35, 0.25f, -0.125f, 1.5f),
                new ReplayRecord(ReplayRole.Replay, 100003, ReplayEdit.Add, 36, 0.21f, 0.0625f, -0.5f),
            };
            string[] said = { "none", "remove:33", "shift:34:0.025:-0.0125", "move:35:0.25:-0.125:1.5", "add:36:0.21:0.0625:-0.5" };
            var buffer = new byte[ReplayCodec.Size];
            for (int i = 0; i < records.Length; i++)
            {
                Assert.True(ReplayCodec.TryEncode(records[i], 77u + (uint)i, 42, buffer, out int written));
                Assert.Equal(40, written);
                Assert.True(ReplayCodec.IsReplay(buffer));
                Assert.True(ReplayCodec.TryDecode(buffer, out ReplayRecord back, out uint seq, out long ticks));
                Assert.Equal(records[i], back);
                Assert.Equal((77u + (uint)i, 42L), (seq, ticks));
                Assert.Equal(said[i], ReplayCodec.Describe(back));
                Assert.False(ScriptMoveCodec.TryDecode(buffer, out _, out _, out _));
                Assert.False(ScriptEventCodec.TryDecode(buffer, out _, out _, out _));
            }

            Assert.Equal(("source", "replay"), (ReplayCodec.Name(ReplayRole.Source), ReplayCodec.Name(ReplayRole.Replay)));
        }

        [Fact]
        public void TheReplayLayout_IsFixed()
        {
            var r = new ReplayRecord(ReplayRole.Replay, 0x0102030405060708UL, ReplayEdit.Shift, 9, 1f, 0f, 0f);
            var buffer = new byte[ReplayCodec.Size];
            Assert.True(ReplayCodec.TryEncode(r, 3, 4, buffer, out _));
            Assert.Equal(new byte[]
            {
                0x8E, 1, 2, 2, 9, 0, 0, 0,
                3, 0, 0, 0,
                8, 7, 6, 5, 4, 3, 2, 1,
                0x00, 0x00, 0x80, 0x3F, // 1.0f
                0, 0, 0, 0,
                0, 0, 0, 0,
                4, 0, 0, 0, 0, 0, 0, 0,
            }, buffer);
        }

        [Theory]
        [InlineData(2, 0)]    // no role
        [InlineData(2, 3)]    // a role this build does not define
        [InlineData(3, 5)]    // an edit this build does not define (4, add, since docs/adr/0038's add)
        [InlineData(1, 2)]    // another version
        [InlineData(0, 0x8D)] // another message
        public void UnknownReplayBytes_AreRefused(int index, byte value)
        {
            var buffer = new byte[ReplayCodec.Size];
            Assert.True(ReplayCodec.TryEncode(new ReplayRecord(ReplayRole.Replay, 1, ReplayEdit.Remove, 2), 0, 0, buffer, out _));
            buffer[index] = value;
            Assert.False(ReplayCodec.TryDecode(buffer, out _, out _, out _));
        }

        [Fact]
        public void ASourceWithAnEdit_OrAShortRecord_IsRefused()
        {
            var buffer = new byte[ReplayCodec.Size];
            Assert.True(ReplayCodec.TryEncode(new ReplayRecord(ReplayRole.Source, 1, ReplayEdit.Remove, 2), 0, 0, buffer, out _));
            Assert.False(ReplayCodec.TryDecode(buffer, out _, out _, out _));
            Assert.True(ReplayCodec.TryEncode(ReplayRecord.Source(1), 0, 0, buffer, out _));
            Assert.False(ReplayCodec.TryDecode(buffer.AsSpan(0, ReplayCodec.Size - 1), out _, out _, out _));
            var shortBuffer = new byte[ReplayCodec.Size - 1];
            Assert.False(ReplayCodec.TryEncode(ReplayRecord.Source(1), 0, 0, shortBuffer, out _));
            Assert.All(shortBuffer, b => Assert.Equal(0, b));
        }
    }
}
