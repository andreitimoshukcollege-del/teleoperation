using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// Decides when to send a joint-state frame and what goes in it (docs/adr/0018). The operator
    /// subscribes with a keepalive on the joint port; while a keepalive is fresh, a frame goes out
    /// every <see cref="IntervalTicks"/>, echoing the latest keepalive's ticks so the operator can
    /// run its own <c>ClockSync</c>. No I/O here, so it is testable: the host reads the plant and
    /// sends the bytes.
    /// </summary>
    public sealed class JointStateStreamer
    {
        private readonly long _ticksPerSecond;
        private readonly JointSample[] _joints = new JointSample[JointStateCodec.MaxJointsPerMessage];
        private long _lastSubscribeRecvTicks = long.MinValue;
        private long _echoOperatorTicks;
        private long _echoRobotRecvTicks;
        private long _lastSendTicks = long.MinValue;
        private uint _sequence;

        public long IntervalTicks { get; }
        public long SubscriptionTimeoutTicks { get; }

        public JointStateStreamer(long ticksPerSecond, double rateHz = 50.0, double subscriptionTimeoutSeconds = 3.0)
        {
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            if (!(rateHz > 0)) throw new ArgumentOutOfRangeException(nameof(rateHz));
            _ticksPerSecond = ticksPerSecond;
            IntervalTicks = (long)(ticksPerSecond / rateHz);
            SubscriptionTimeoutTicks = (long)(ticksPerSecond * subscriptionTimeoutSeconds);
        }

        /// <summary>True while a keepalive has arrived within the timeout.</summary>
        public bool IsSubscribed(long nowTicks) =>
            _lastSubscribeRecvTicks != long.MinValue && nowTicks - _lastSubscribeRecvTicks <= SubscriptionTimeoutTicks;

        /// <summary>Handles a received datagram if it is a keepalive; returns false for anything else.</summary>
        public bool TryHandleSubscribe(ReadOnlySpan<byte> datagram, long arrivalTicks)
        {
            if (!JointStateCodec.TryDecodeSubscribe(datagram, out long operatorTicks, out _))
            {
                return false;
            }

            _lastSubscribeRecvTicks = arrivalTicks;
            _echoOperatorTicks = operatorTicks;
            _echoRobotRecvTicks = arrivalTicks;
            return true;
        }

        /// <summary>
        /// Encodes the next frame into <paramref name="destination"/> when one is due. The plant's
        /// joints are read through <paramref name="readJoints"/> only then.
        /// </summary>
        public bool TryBuildFrame(long nowTicks, Func<JointSample[], int> readJoints, long sensedTicks,
            Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = 0;
            if (!IsSubscribed(nowTicks) || (_lastSendTicks != long.MinValue && nowTicks - _lastSendTicks < IntervalTicks))
            {
                return false;
            }

            int count = readJoints(_joints);
            if (!JointStateCodec.TryEncode(_sequence, nowTicks, sensedTicks, _echoOperatorTicks, _echoRobotRecvTicks,
                    _ticksPerSecond, _joints.AsSpan(0, count), destination, out bytesWritten))
            {
                return false;
            }

            _sequence++;
            _lastSendTicks = nowTicks;
            return true;
        }
    }
}
