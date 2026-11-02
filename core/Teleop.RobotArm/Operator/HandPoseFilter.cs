using System;
using Teleop.RobotArm.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The hand filter's settings (docs/adr/0029 amendment 7). Defaults are first guesses, tuned on the headset.</summary>
    public sealed class HandFilterConfig
    {
        /// <summary>The cutoff while the hand is still (Hz): lower is smoother, and lags more on slow motion.</summary>
        public double MinCutoffHz { get; set; } = 1.0;

        /// <summary>How fast the cutoff rises with the hand's speed (Hz per m/s): higher lags less when moving and lets more tremor through.</summary>
        public double PositionBeta { get; set; } = 80.0;

        /// <summary>The same for the palm's turning (Hz per rad/s).</summary>
        public double RotationBeta { get; set; } = 16.0;

        /// <summary>The cutoff of the speed estimates that set the two cutoffs (Hz).</summary>
        public double DerivativeCutoffHz { get; set; } = 1.0;

        /// <summary>Samples further apart than this (s) start the filter again from the hand: a gap is not motion.</summary>
        public double MaxGapSeconds { get; set; } = 0.25;

        public string? Validate()
        {
            if (!(MinCutoffHz > 0) || !(DerivativeCutoffHz > 0) || double.IsInfinity(MinCutoffHz + DerivativeCutoffHz))
            {
                return $"the hand filter's cutoffs must be positive and finite (min {MinCutoffHz} Hz, derivative {DerivativeCutoffHz} Hz)";
            }

            if (!(PositionBeta >= 0) || !(RotationBeta >= 0) || double.IsInfinity(PositionBeta + RotationBeta))
            {
                return $"the hand filter's betas must be 0 or positive, and finite (position {PositionBeta}, rotation {RotationBeta})";
            }

            if (!(MaxGapSeconds > 0) || double.IsInfinity(MaxGapSeconds)) return $"MaxGapSeconds must be positive and finite ({MaxGapSeconds})";
            return null;
        }
    }

    /// <summary>
    /// Takes the tracking jitter out of a hand that is held still, and stays out of the way of one that moves (docs/adr/0029
    /// amendment 7): the absolute mapping makes every millimetre of tracking noise a millimetre of tool, and the IK turned a
    /// still hand's noise into joints visibly shaking between poses. A One Euro filter (Casiez, Roussel and Vogel, CHI 2012):
    /// a low-pass whose cutoff is <see cref="HandFilterConfig.MinCutoffHz"/> while the hand is still and rises with its speed,
    /// so a still hand is smoothed hard and a moving one barely.
    /// <list type="bullet">
    /// <item><b>Position:</b> one cutoff for all three axes, from the speed of the velocity (the difference of consecutive
    /// samples, low-passed at <see cref="HandFilterConfig.DerivativeCutoffHz"/>), so the filter never bends the direction of
    /// a motion.</item>
    /// <item><b>Orientation:</b> the same on the palm's turning: the angular velocity (world frame) of consecutive samples,
    /// low-passed, sets the cutoff, and the filtered rotation moves that share of the way to the hand's along the shorter
    /// arc.</item>
    /// <item><b>The first sample</b>, and the first after a gap longer than <see cref="HandFilterConfig.MaxGapSeconds"/>,
    /// passes through and starts the filter there. A sample no later than the last teaches nothing and returns the filtered
    /// pose unchanged; a non-finite one passes through and restarts the filter.</item>
    /// </list>
    /// One per hand, fed every tracked frame (not only while the hand drives its arm), so the filter has settled by the
    /// time a grab takes it, and <see cref="Reset"/> when tracking is lost. A plain struct with no clock of its own: the host
    /// passes the time, in seconds, from its injected clock. Nothing allocated.
    /// </summary>
    public struct HandPoseFilter
    {
        private bool _started;
        private double _t;
        private double _x, _y, _z, _qw, _qx, _qy, _qz;
        private double _rawX, _rawY, _rawZ, _rawQw, _rawQx, _rawQy, _rawQz;
        private double _vx, _vy, _vz, _wx, _wy, _wz;

        /// <summary>The hand's speed and the palm's turning rate as the filter estimates them (m/s, rad/s): what sets its cutoffs.</summary>
        public double Speed => Math.Sqrt(_vx * _vx + _vy * _vy + _vz * _vz);

        public double TurnRate => Math.Sqrt(_wx * _wx + _wy * _wy + _wz * _wz);

        /// <summary>The filtered pose for <paramref name="hand"/>, sampled at <paramref name="nowSeconds"/>.</summary>
        public RigidTransform Filter(HandFilterConfig config, in RigidTransform hand, double nowSeconds)
        {
            double n = Math.Sqrt(hand.Qw * hand.Qw + hand.Qx * hand.Qx + hand.Qy * hand.Qy + hand.Qz * hand.Qz);
            if (!(n > 1e-9) || double.IsNaN(hand.X + hand.Y + hand.Z + n + nowSeconds) || double.IsInfinity(hand.X + hand.Y + hand.Z + n + nowSeconds))
            {
                this = default;
                return hand;
            }

            double qw = hand.Qw / n, qx = hand.Qx / n, qy = hand.Qy / n, qz = hand.Qz / n;
            double dt = nowSeconds - _t;
            if (!_started || dt > config.MaxGapSeconds)
            {
                this = default;
                _started = true;
                _t = nowSeconds;
                _x = _rawX = hand.X;
                _y = _rawY = hand.Y;
                _z = _rawZ = hand.Z;
                _qw = _rawQw = qw;
                _qx = _rawQx = qx;
                _qy = _rawQy = qy;
                _qz = _rawQz = qz;
                return new RigidTransform(hand.X, hand.Y, hand.Z, qw, qx, qy, qz);
            }

            if (!(dt > 0))
            {
                return new RigidTransform(_x, _y, _z, _qw, _qx, _qy, _qz);
            }

            // The speeds, from consecutive raw samples: a still hand's noise mostly cancels in them, where a difference from
            // the filtered pose would read the filter's own lag as speed.
            double ad = Alpha(dt, config.DerivativeCutoffHz);
            _vx += ad * ((hand.X - _rawX) / dt - _vx);
            _vy += ad * ((hand.Y - _rawY) / dt - _vy);
            _vz += ad * ((hand.Z - _rawZ) / dt - _vz);
            AngularVelocity(qw, qx, qy, qz, _rawQw, _rawQx, _rawQy, _rawQz, dt, out double wx, out double wy, out double wz);
            _wx += ad * (wx - _wx);
            _wy += ad * (wy - _wy);
            _wz += ad * (wz - _wz);

            double a = Alpha(dt, config.MinCutoffHz + config.PositionBeta * Speed);
            _x += a * (hand.X - _x);
            _y += a * (hand.Y - _y);
            _z += a * (hand.Z - _z);
            RigidTransform turned = RigidTransform.Interpolate(
                new RigidTransform(0, 0, 0, _qw, _qx, _qy, _qz), new RigidTransform(0, 0, 0, qw, qx, qy, qz),
                Alpha(dt, config.MinCutoffHz + config.RotationBeta * TurnRate));
            _qw = turned.Qw;
            _qx = turned.Qx;
            _qy = turned.Qy;
            _qz = turned.Qz;

            _t = nowSeconds;
            _rawX = hand.X;
            _rawY = hand.Y;
            _rawZ = hand.Z;
            _rawQw = qw;
            _rawQx = qx;
            _rawQy = qy;
            _rawQz = qz;
            return new RigidTransform(_x, _y, _z, _qw, _qx, _qy, _qz);
        }

        /// <summary>Forgets the hand: the next sample passes through and starts the filter again.</summary>
        public void Reset() => this = default;

        /// <summary>The share of the way to the new sample a low-pass at <paramref name="cutoffHz"/> moves in <paramref name="seconds"/>.</summary>
        private static double Alpha(double seconds, double cutoffHz)
        {
            double r = 2 * Math.PI * cutoffHz * seconds;
            return r / (1 + r);
        }

        /// <summary>The world-frame angular velocity that turns <c>p</c> into <c>q</c> in <paramref name="seconds"/>, the shorter way.</summary>
        private static void AngularVelocity(double qw, double qx, double qy, double qz, double pw, double px, double py, double pz,
            double seconds, out double wx, out double wy, out double wz)
        {
            // q · conj(p): the turn applied after p.
            double w = qw * pw + qx * px + qy * py + qz * pz;
            double x = -qw * px + qx * pw - qy * pz + qz * py;
            double y = -qw * py + qx * pz + qy * pw - qz * px;
            double z = -qw * pz - qx * py + qy * px + qz * pw;
            if (w < 0)
            {
                w = -w;
                x = -x;
                y = -y;
                z = -z;
            }

            double s = Math.Sqrt(x * x + y * y + z * z);
            double perSine = s > 1e-12 ? 2 * Math.Atan2(s, w) / s : 2;
            wx = x * perSine / seconds;
            wy = y * perSine / seconds;
            wz = z * perSine / seconds;
        }
    }
}
