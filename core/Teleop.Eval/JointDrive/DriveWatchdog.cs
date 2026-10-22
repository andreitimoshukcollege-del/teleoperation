using System;
using Teleop.RobotArm.Collision;

namespace Teleop.Eval.JointDrive
{
    /// <summary>Why <c>joint-drive</c> stopped the arm early.</summary>
    public enum DriveFault
    {
        None,

        /// <summary>A joint is far from its command and not moving toward it: it has hit something, or something holds it.</summary>
        Stall,

        /// <summary>No joint-state frame for <see cref="DriveWatchdog.MaxDegradedSeconds"/>: the robot, the network or RobotHost is gone.</summary>
        FrameGap,

        /// <summary>
        /// Frames kept arriving, but the stream was unhealthy for <see cref="DriveWatchdog.MaxDegradedSeconds"/> of the last
        /// <see cref="DriveWatchdog.DegradedWindowSeconds"/>: the Wi-Fi is losing them, or someone else (a headset, joint-probe)
        /// is taking the stream, and so is on the joint port too. A second subscriber makes the stream alternate between the
        /// two every keepalive, so it is never unhealthy for long at a stretch, only often.
        /// The robot's log tells the two apart (one <c>SS</c> line means one subscriber).
        /// </summary>
        StreamDegraded,
    }

    /// <summary>
    /// Stops <c>joint-drive</c> when the arm is not doing what it is told (the stall watchdog docs/adr/0021
    /// lists among its data-collection tools). Pure: it sees the time, each frame's arrival and each
    /// fresh reading with the pose commanded at that moment, and says whether to stop. Times are seconds
    /// on any one clock.
    ///
    /// <para>A stall is a joint more than <see cref="StallErrorRadians"/> from its command whose
    /// reading has moved less than <see cref="StallSpeedRadPerSec"/> over the last half second, for
    /// <see cref="StallSeconds"/> in a row. A moving arm is never a stall, however far behind, and an
    /// arm that has arrived is never far, so the limits need not know the speed or the latency.</para>
    ///
    /// <para><b>A degraded stream pauses; only a long one stops.</b> <see cref="IsHealthy"/> is false
    /// while the newest frame is over <see cref="PauseGapSeconds"/> old or the last second carried
    /// under <see cref="MinFrameRateHz"/> frames, and <c>joint-drive</c> then holds its plan where it
    /// is. Lab Wi-Fi stalls for a second or two in both directions now and then (2026-10-22's first
    /// recorded drive: commands reached the robot up to 341 ms apart while both ends sent steadily),
    /// and the robot holds its last target meanwhile, which is safe. It is a fault when the stream
    /// has been unhealthy for <see cref="MaxDegradedSeconds"/> in all within the last
    /// <see cref="DegradedWindowSeconds"/>, whether in one stretch or in many short ones: a second
    /// subscriber (the headset left on) toggles it every half second, and must stop the drive.</para>
    /// </summary>
    public sealed class DriveWatchdog
    {
        public const double StallErrorRadians = 0.25;
        public const double StallSpeedRadPerSec = 0.1;
        public const double StallSeconds = 1.0;
        public const double PauseGapSeconds = 0.25;
        public const double MinFrameRateHz = 30.0;
        public const double FrameRateWindowSeconds = 1.0;
        public const double MaxDegradedSeconds = 3.0;
        public const double DegradedWindowSeconds = 10.0;

        /// <summary>The degraded time is kept in buckets this long.</summary>
        private const double BucketSeconds = 0.1;

        /// <summary>How far back a reading's speed is measured.</summary>
        private const double SpeedWindowSeconds = 0.5;

        private const int ReadingCapacity = 64;

        private readonly double[] _readingTimes = new double[ReadingCapacity];
        private readonly ArmPose[] _readings = new ArmPose[ReadingCapacity];
        private int _readingCount;
        private int _readingNext;

        private readonly double[] _stallSince = new double[DriveEnvelope.JointCount];
        private readonly double[] _frameTimes = new double[256];
        private int _frameCount;
        private int _frameNext;
        private double _firstFrame = double.NaN;
        private double _lastFrame = double.NaN;
        private readonly double[] _degradedBuckets = new double[(int)(DegradedWindowSeconds / BucketSeconds)];
        private long _bucket = long.MinValue;
        private double _lastHealthCheck = double.NaN;

        public DriveWatchdog()
        {
            Array.Fill(_stallSince, double.NaN);
        }

        /// <summary>The joint that stalled, after <see cref="DriveFault.Stall"/>; otherwise -1.</summary>
        public int StalledJoint { get; private set; } = -1;

        public void OnFrame(double t)
        {
            if (double.IsNaN(_firstFrame)) _firstFrame = t;
            _lastFrame = t;
            _frameTimes[_frameNext] = t;
            _frameNext = (_frameNext + 1) % _frameTimes.Length;
            _frameCount = Math.Min(_frameCount + 1, _frameTimes.Length);
        }

        /// <summary>A fresh servo reading taken at <paramref name="sensedAt"/>, and the pose commanded then.</summary>
        public void OnReading(double sensedAt, in ArmPose measured, in ArmPose commanded)
        {
            bool haveOld = TryReadingBefore(sensedAt - SpeedWindowSeconds, out double oldAt, out ArmPose old);
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                double error = Math.Abs(DrivePlan.Joint(commanded, j) - DrivePlan.Joint(measured, j));
                bool still = haveOld && Math.Abs(DrivePlan.Joint(measured, j) - DrivePlan.Joint(old, j)) / (sensedAt - oldAt) < StallSpeedRadPerSec;
                if (error > StallErrorRadians && still)
                {
                    if (double.IsNaN(_stallSince[j])) _stallSince[j] = sensedAt;
                    else if (sensedAt - _stallSince[j] >= StallSeconds && StalledJoint < 0) StalledJoint = j;
                }
                else
                {
                    _stallSince[j] = double.NaN;
                }
            }

            _readingTimes[_readingNext] = sensedAt;
            _readings[_readingNext] = measured;
            _readingNext = (_readingNext + 1) % ReadingCapacity;
            _readingCount = Math.Min(_readingCount + 1, ReadingCapacity);
        }

        /// <summary>
        /// Whether the stream is good enough to move on: a frame within <see cref="PauseGapSeconds"/>,
        /// and at least <see cref="MinFrameRateHz"/> over the last <see cref="FrameRateWindowSeconds"/>
        /// (once that much has passed). Call it once per loop: it also adds up how long the stream has
        /// been degraded, for <see cref="Check"/>.
        /// </summary>
        public bool IsHealthy(double now)
        {
            bool healthy = !double.IsNaN(_lastFrame) && now - _lastFrame <= PauseGapSeconds;
            if (healthy && now - _firstFrame >= FrameRateWindowSeconds)
            {
                int recent = 0;
                for (int i = 0; i < _frameCount; i++)
                {
                    if (_frameTimes[i] > now - FrameRateWindowSeconds) recent++;
                }

                healthy = recent >= MinFrameRateHz * FrameRateWindowSeconds;
            }

            double dt = double.IsNaN(_lastHealthCheck) ? 0 : Math.Max(0, now - _lastHealthCheck);
            _lastHealthCheck = now;
            AdvanceBuckets(now);
            if (!healthy) _degradedBuckets[(int)(_bucket % _degradedBuckets.Length)] += dt;
            return healthy;
        }

        public DriveFault Check(double now)
        {
            if (StalledJoint >= 0) return DriveFault.Stall;
            AdvanceBuckets(now);
            double degraded = 0;
            foreach (double d in _degradedBuckets) degraded += d;
            if (degraded < MaxDegradedSeconds) return DriveFault.None;
            return double.IsNaN(_lastFrame) || now - _lastFrame >= MaxDegradedSeconds ? DriveFault.FrameGap : DriveFault.StreamDegraded;
        }

        /// <summary>Moves the bucket ring to <paramref name="now"/>, emptying the buckets it passes.</summary>
        private void AdvanceBuckets(double now)
        {
            long bucket = (long)Math.Floor(now / BucketSeconds);
            if (_bucket == long.MinValue)
            {
                _bucket = bucket;
                return;
            }

            for (long b = _bucket + 1; b <= bucket && b <= _bucket + _degradedBuckets.Length; b++)
            {
                _degradedBuckets[(int)(b % _degradedBuckets.Length)] = 0;
            }

            if (bucket > _bucket) _bucket = bucket;
        }

        /// <summary>The newest reading taken at or before <paramref name="t"/>.</summary>
        private bool TryReadingBefore(double t, out double at, out ArmPose pose)
        {
            at = double.NegativeInfinity;
            pose = default;
            for (int i = 0; i < _readingCount; i++)
            {
                if (_readingTimes[i] <= t && _readingTimes[i] > at)
                {
                    at = _readingTimes[i];
                    pose = _readings[i];
                }
            }

            return !double.IsNegativeInfinity(at);
        }
    }
}
