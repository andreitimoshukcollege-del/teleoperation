namespace Teleop.CameraHost.Streaming
{
    /// <summary>
    /// Thins the camera's frame rate down to a viewer's requested maximum (docs/adr/0014 §2). Decides
    /// on capture time, not send time, so that the frames kept are evenly spaced in the scene rather
    /// than in the sender's scheduling. A tenth of a period of slack absorbs the camera's own interval
    /// jitter (32-36 ms at 30 fps on the JetRover), which would otherwise drop every other frame when
    /// the requested rate equals the camera's.
    /// </summary>
    internal sealed class FramePacer
    {
        private readonly long _ticksPerSecond;
        private bool _hasSent;
        private long _lastSentTicks;

        public FramePacer(long ticksPerSecond)
        {
            _ticksPerSecond = ticksPerSecond;
        }

        /// <param name="requestedMaxFps">0 means no limit: send every frame.</param>
        public bool ShouldSend(long frameTicks, ushort requestedMaxFps)
        {
            if (!_hasSent || requestedMaxFps == 0)
            {
                return true;
            }

            long period = _ticksPerSecond / requestedMaxFps;
            return frameTicks - _lastSentTicks >= period - (period / 10);
        }

        public void MarkSent(long frameTicks)
        {
            _hasSent = true;
            _lastSentTicks = frameTicks;
        }

        public void Reset() => _hasSent = false;
    }
}
