using System.Net;

namespace Teleop.CameraHost.Streaming
{
    /// <summary>
    /// Who the camera is streaming to (docs/adr/0014-camera-frame-downlink.md §2): the source of the
    /// most recent keepalive, until none has arrived for the timeout. The newest keepalive wins, so a
    /// second viewer takes the stream over rather than sharing it, the same reply-to-sender model the
    /// pose path uses.
    /// </summary>
    internal sealed class SubscriberTracker
    {
        private readonly long _timeoutTicks;
        private EndPoint? _target;
        private long _lastSeenTicks;
        private ushort _requestedMaxFps;

        public SubscriberTracker(long timeoutTicks)
        {
            if (timeoutTicks <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(timeoutTicks), timeoutTicks, "Timeout must be positive.");
            }

            _timeoutTicks = timeoutTicks;
        }

        /// <summary>
        /// Records a keepalive. Returns true when the stream's target changed: a first subscriber, a
        /// different one, or the same one returning after timing out.
        /// </summary>
        public bool Observe(EndPoint from, ushort requestedMaxFps, long nowTicks)
        {
            bool changed = _target is null || !_target.Equals(from) || nowTicks - _lastSeenTicks > _timeoutTicks;
            _target = from;
            _lastSeenTicks = nowTicks;
            _requestedMaxFps = requestedMaxFps;
            return changed;
        }

        public bool TryGetActive(long nowTicks, out EndPoint target, out ushort requestedMaxFps)
        {
            if (_target is null || nowTicks - _lastSeenTicks > _timeoutTicks)
            {
                target = null!;
                requestedMaxFps = 0;
                return false;
            }

            target = _target;
            requestedMaxFps = _requestedMaxFps;
            return true;
        }

        /// <summary>The last subscriber, active or not, for logging.</summary>
        public EndPoint? LastTarget => _target;
    }
}
