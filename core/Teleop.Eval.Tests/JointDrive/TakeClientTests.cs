using System;
using System.Collections.Generic;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.Tests.JointDrive
{
    /// <summary>
    /// joint-drive's RECORD button (<see cref="TakeClient"/>, docs/adr/0033): it reads RobotHost's list and status as the
    /// headset does, and its requests are the headset's, each with a new sequence, resent unchanged until echoed.
    /// </summary>
    public class TakeClientTests
    {
        private readonly List<byte[]> _sent = new();

        private TakeClient Client(uint first = 41) => new((bytes, n) => _sent.Add(bytes.AsSpan(0, n).ToArray()), first);

        private static byte[] List(ushort revision, params string[] names)
        {
            var buffer = new byte[256];
            Assert.True(RecordCodec.TryEncodeList(revision, 0, 1, names, 0, names.Length, buffer, out int n));
            return buffer.AsSpan(0, n).ToArray();
        }

        private static byte[] Status(RecordState state, RecordOutcome outcome, uint sequence, ushort revision, ushort take = 0)
        {
            var buffer = new byte[RecordCodec.StatusBytes];
            var status = new RecordStatus(state, outcome, sequence, 0, 3, revision, 0, take, 0, 120);
            Assert.True(RecordCodec.TryEncodeStatus(status, 123, buffer, out int n));
            return buffer.AsSpan(0, n).ToArray();
        }

        [Fact]
        public void ItFindsTheScenarioInRobotHostsList()
        {
            TakeClient client = Client();
            Assert.Equal(-1, client.IndexOf("knock-over")); // no list yet
            Assert.True(client.OnDatagram(List(7, "free-play", "knock-over", "push-to-edge")));
            Assert.Equal(1, client.IndexOf("knock-over"));
            Assert.Equal(-1, client.IndexOf("Knock-Over"));
            Assert.Equal("free-play, knock-over, push-to-edge", client.Names());
            Assert.Equal(7, client.State.ListRevision);
        }

        [Fact]
        public void ARequestIsTheHeadsetsAndIsResentUnchangedUntilEchoed()
        {
            TakeClient client = Client(first: 41);
            uint sequence = client.Begin(RecordOp.Start, 1, 7, 555);
            Assert.Equal(42u, sequence);
            Assert.True(RecordCodec.TryDecodeRequest(_sent[0], out RecordOp op, out byte scenario, out ushort revision, out uint sent, out long ticks));
            Assert.Equal((RecordOp.Start, (byte)1, (ushort)7, 42u, 555L), (op, scenario, revision, sent, ticks));

            Assert.False(client.Answered(sequence));
            client.Resend();
            Assert.Equal(_sent[0], _sent[1]); // RobotHost ignores a sequence it has answered: resending is safe

            client.OnDatagram(Status(RecordState.Recording, RecordOutcome.Started, 41, 7)); // an older request's echo
            Assert.False(client.Answered(sequence));
            client.OnDatagram(Status(RecordState.Recording, RecordOutcome.Started, 42, 7, take: 3));
            Assert.True(client.Answered(sequence));
            Assert.Equal((RecordOutcome.Started, (ushort)3), (client.State.Status.Outcome, client.State.Status.Take));

            uint stop = client.Begin(RecordOp.Stop, RecordStatus.NoScenario, 7, 600);
            Assert.Equal(43u, stop);
            Assert.True(RecordCodec.TryDecodeRequest(_sent[2], out RecordOp stopOp, out _, out _, out uint stopSent, out _));
            Assert.Equal((RecordOp.Stop, 43u), (stopOp, stopSent));
        }

        [Fact]
        public void JointStateAndOtherDatagramsAreNotItsConcern()
        {
            TakeClient client = Client();
            Assert.False(client.OnDatagram(new byte[] { 0x01, 0x02, 0x03 }));
            Assert.False(client.OnDatagram(ReadOnlySpan<byte>.Empty));
            Assert.False(client.State.HasStatus || client.State.HasList);
        }

        [Fact]
        public void ASequenceNeverWrapsToZero()
        {
            TakeClient client = Client(first: uint.MaxValue);
            Assert.Equal(1u, client.Begin(RecordOp.Start, 0, 1, 0));
            Assert.InRange(TakeClient.FirstSequence(), 0x4000_0000u, 0x7FFF_FFFFu);
        }
    }
}
