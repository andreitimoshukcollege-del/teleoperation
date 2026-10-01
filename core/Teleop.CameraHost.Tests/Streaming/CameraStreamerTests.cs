using System.Net;
using Teleop.CameraHost.Streaming;
using Teleop.Core.Camera;
using Teleop.Core.Types;

namespace Teleop.CameraHost.Tests.Streaming
{
    /// <summary>
    /// <see cref="CameraStreamer"/>, <see cref="SubscriberTracker"/> and <see cref="FramePacer"/>: who
    /// gets frames, how many, and that what is sent is exactly what Core's reassembler rebuilds.
    /// </summary>
    public class CameraStreamerTests
    {
        private const long Tps = 1_000_000_000; // the Jetson's Stopwatch rate
        private const long Ms = Tps / 1000;
        private static readonly IPEndPoint Viewer = new(IPAddress.Parse("10.188.60.46"), 6004);
        private static readonly IPEndPoint OtherViewer = new(IPAddress.Parse("10.188.60.99"), 6004);

        private sealed class RecordingSender : IDatagramSender
        {
            public readonly List<(byte[] Datagram, EndPoint Target)> Sent = new();
            public int FailAfter = int.MaxValue;

            public bool TrySend(ReadOnlySpan<byte> datagram, EndPoint target)
            {
                if (Sent.Count >= FailAfter)
                {
                    return false;
                }

                Sent.Add((datagram.ToArray(), target));
                return true;
            }
        }

        private static byte[] Subscribe(ushort maxFps = 0)
        {
            var buffer = new byte[CameraSubscribeCodec.EncodedSize];
            new CameraSubscribeCodec().TryEncode(maxFps, 0, buffer, out _);
            return buffer;
        }

        private static byte[] Jpeg(int seed, int length = 42_000) =>
            Enumerable.Range(0, length).Select(i => unchecked((byte)(i * 13 + seed * 101))).ToArray();

        private static (CameraStreamer Streamer, RecordingSender Sender, Func<long> Clock, Action<long> SetClock) Make(uint firstId = 5000)
        {
            var sender = new RecordingSender();
            long clock = 0;
            var streamer = new CameraStreamer(sender, () => clock, Tps, subscriberTimeoutTicks: 3000 * Ms, firstFrameId: firstId);
            return (streamer, sender, () => clock, t => clock = t);
        }

        [Fact]
        public void WithoutASubscriber_NothingIsSent()
        {
            var (streamer, sender, _, _) = Make();
            Assert.Equal(FrameOutcome.NoSubscriber, streamer.OfferFrame(Jpeg(1), 100, 640, 480, nowTicks: 0));
            Assert.Empty(sender.Sent);
            Assert.Equal(1, streamer.FramesWithoutSubscriber);
        }

        [Fact]
        public void ASubscribedFrame_IsSentInChunksThatRebuildItExactly_WithOneSharedStamp()
        {
            var (streamer, sender, _, setClock) = Make(firstId: 5000);
            Assert.True(streamer.HandleDatagram(Subscribe(), Viewer, nowTicks: 0));

            setClock(777 * Ms);
            byte[] jpeg = Jpeg(1);
            Assert.Equal(FrameOutcome.Sent, streamer.OfferFrame(jpeg, captureTicks: 700 * Ms, 640, 480, nowTicks: 760 * Ms));

            Assert.Equal(CameraChunkCodec.ChunkCountFor(jpeg.Length), sender.Sent.Count);
            Assert.All(sender.Sent, s => Assert.Equal(Viewer, s.Target));

            var reassembler = new CameraFrameReassembler(3, 256 * 1024);
            foreach (var (datagram, _) in sender.Sent)
            {
                Assert.NotEqual(CameraChunkOutcome.Inconsistent, reassembler.Accept(datagram, 0));
            }

            Assert.True(reassembler.TryTakeLatest(out CameraFrameInfo info, out ReadOnlySpan<byte> rebuilt));
            Assert.Equal(jpeg, rebuilt.ToArray());
            Assert.Equal(5000u, info.Frame.FrameId);
            Assert.Equal(700 * Ms, info.Frame.CaptureTicks);
            Assert.Equal(777 * Ms, info.Frame.SendTicks); // read once, before the first chunk
            Assert.Equal(Tps, info.Frame.TicksPerSecond);
            Assert.Equal((ushort)640, info.Frame.Width);
            Assert.Equal(sender.Sent.Sum(s => s.Datagram.Length), streamer.BytesSent);
        }

        [Fact]
        public void AnUnsetCaptureStamp_IsSentUnset_NotReplaced()
        {
            var (streamer, sender, _, _) = Make();
            streamer.HandleDatagram(Subscribe(), Viewer, 0);
            Assert.Equal(FrameOutcome.Sent, streamer.OfferFrame(Jpeg(2, 500), LatencyTrace.Unset, 640, 480, nowTicks: 10 * Ms));

            Assert.True(new CameraChunkCodec().TryDecode(sender.Sent[0].Datagram, out CameraChunkHeader header, out _));
            Assert.Equal(LatencyTrace.Unset, header.Frame.CaptureTicks);
        }

        [Fact]
        public void FrameIds_StartWhereTheHostSaysAndAdvancePerFrame()
        {
            var (streamer, sender, _, _) = Make(firstId: uint.MaxValue);
            streamer.HandleDatagram(Subscribe(), Viewer, 0);
            streamer.OfferFrame(Jpeg(1, 100), 1 * Ms, 640, 480, 1 * Ms);
            streamer.OfferFrame(Jpeg(2, 100), 40 * Ms, 640, 480, 40 * Ms);

            var codec = new CameraChunkCodec();
            codec.TryDecode(sender.Sent[0].Datagram, out CameraChunkHeader first, out _);
            codec.TryDecode(sender.Sent[1].Datagram, out CameraChunkHeader second, out _);
            Assert.Equal(uint.MaxValue, first.Frame.FrameId);
            Assert.Equal(0u, second.Frame.FrameId); // wraps, which the reassembler treats as newer
        }

        [Fact]
        public void TheSubscriptionLapsesAfterTheTimeout_AndAKeepaliveRenewsIt()
        {
            var (streamer, sender, _, _) = Make();
            streamer.HandleDatagram(Subscribe(), Viewer, nowTicks: 0);
            Assert.Equal(FrameOutcome.Sent, streamer.OfferFrame(Jpeg(1, 100), 2_900 * Ms, 640, 480, nowTicks: 2_900 * Ms));
            Assert.Equal(FrameOutcome.NoSubscriber, streamer.OfferFrame(Jpeg(2, 100), 3_100 * Ms, 640, 480, nowTicks: 3_100 * Ms));

            Assert.True(streamer.HandleDatagram(Subscribe(), Viewer, nowTicks: 3_200 * Ms)); // returning counts as a change
            Assert.Equal(FrameOutcome.Sent, streamer.OfferFrame(Jpeg(3, 100), 3_250 * Ms, 640, 480, nowTicks: 3_250 * Ms));
            Assert.Equal(2, sender.Sent.Count);
        }

        [Fact]
        public void TheNewestSubscriberTakesTheStreamOver()
        {
            var (streamer, sender, _, _) = Make();
            Assert.True(streamer.HandleDatagram(Subscribe(), Viewer, 0));
            Assert.False(streamer.HandleDatagram(Subscribe(), Viewer, 1_000 * Ms)); // same viewer: no change
            Assert.True(streamer.HandleDatagram(Subscribe(), OtherViewer, 1_500 * Ms));

            streamer.OfferFrame(Jpeg(1, 100), 1_600 * Ms, 640, 480, 1_600 * Ms);
            Assert.Equal(OtherViewer, sender.Sent.Single().Target);
        }

        [Fact]
        public void ARequestedRate_ThinsFramesByCaptureTime_ToleratingCameraJitter()
        {
            var (streamer, sender, _, _) = Make();
            streamer.HandleDatagram(Subscribe(maxFps: 15), Viewer, 0);

            // 30 fps with the JetRover camera's 32-36 ms jitter: every other frame should go.
            long[] captures = { 0, 32, 68, 100, 136, 168, 204, 236, 272, 304 };
            int sent = captures.Count(c => streamer.OfferFrame(Jpeg((int)c, 100), c * Ms, 640, 480, c * Ms) == FrameOutcome.Sent);

            Assert.Equal(5, sent);
            Assert.Equal(5, streamer.FramesPaced);
        }

        [Fact]
        public void RequestingTheCamerasOwnRate_KeepsEveryFrameDespiteJitter()
        {
            var (streamer, _, _, _) = Make();
            streamer.HandleDatagram(Subscribe(maxFps: 30), Viewer, 0);
            long[] captures = { 0, 32, 68, 100, 136, 168 };
            Assert.All(captures, c => Assert.Equal(FrameOutcome.Sent, streamer.OfferFrame(Jpeg((int)c, 100), c * Ms, 640, 480, c * Ms)));
        }

        [Fact]
        public void ASendFailingMidFrame_StopsThatFrame_AndStillConsumesItsId()
        {
            var (streamer, sender, _, _) = Make(firstId: 10);
            streamer.HandleDatagram(Subscribe(), Viewer, 0);
            sender.FailAfter = 3;
            Assert.Equal(FrameOutcome.SendFailed, streamer.OfferFrame(Jpeg(1), 1 * Ms, 640, 480, 1 * Ms));
            Assert.Equal(3, sender.Sent.Count);

            sender.FailAfter = int.MaxValue;
            streamer.OfferFrame(Jpeg(2, 100), 40 * Ms, 640, 480, 40 * Ms);
            new CameraChunkCodec().TryDecode(sender.Sent[^1].Datagram, out CameraChunkHeader header, out _);
            Assert.Equal(11u, header.Frame.FrameId);
            Assert.Equal(1, streamer.FramesFailed);
        }

        [Fact]
        public void NonKeepaliveDatagrams_AreIgnored()
        {
            var (streamer, sender, _, _) = Make();
            Assert.False(streamer.HandleDatagram(new byte[] { 1, 2, 3 }, Viewer, 0));
            Assert.Equal(1, streamer.DatagramsIgnored);
            Assert.Equal(FrameOutcome.NoSubscriber, streamer.OfferFrame(Jpeg(1, 100), 0, 640, 480, 0));
            Assert.Empty(sender.Sent);
        }

        [Fact]
        public void AnEmptyFrame_IsNotSendable()
        {
            var (streamer, sender, _, _) = Make();
            streamer.HandleDatagram(Subscribe(), Viewer, 0);
            Assert.Equal(FrameOutcome.Unsendable, streamer.OfferFrame(ReadOnlySpan<byte>.Empty, 0, 640, 480, 0));
            Assert.Empty(sender.Sent);
        }
    }
}
