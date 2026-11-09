using System;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The middle-finger tap's thresholds (docs/adr/0029 amendment 10). First guesses, tuned on the headset.</summary>
    public sealed class MiddleTapConfig
    {
        /// <summary>The thumb and middle fingertips closer than this (m) are touching.</summary>
        public double TouchMeters { get; set; } = 0.015;

        /// <summary>A touch ends when they are this far apart (m): wider than <see cref="TouchMeters"/>, so it does not chatter.</summary>
        public double ReleaseMeters { get; set; } = 0.03;

        /// <summary>A touch counts only while the thumb and index fingertips are at least this far apart (m): closing the gripper brings the middle finger along.</summary>
        public double IndexApartMeters { get; set; } = 0.035;

        /// <summary>A touch must last this long (s) before it toggles: shorter is tracking flicker.</summary>
        public double MinTouchSeconds { get; set; } = 0.03;

        public string? Validate()
        {
            if (!(TouchMeters > 0) || !(ReleaseMeters > TouchMeters) || double.IsInfinity(ReleaseMeters))
                return $"the middle tap's touch distance must be positive and below its release distance ({TouchMeters} / {ReleaseMeters} m)";
            if (!(IndexApartMeters > 0) || double.IsInfinity(IndexApartMeters)) return $"MiddleTapIndexApartMeters must be positive ({IndexApartMeters})";
            if (!(MinTouchSeconds >= 0) || double.IsInfinity(MinTouchSeconds)) return $"MiddleTapMinSeconds must be zero or more ({MinTouchSeconds})";
            return null;
        }
    }

    /// <summary>
    /// The lock as a single tap of the thumb on the middle fingertip (docs/adr/0029 amendment 10, the user's choice over the
    /// double curl, which hand tracking often missed). It is read the way the gripper's pinch is, from two fingertips' gap,
    /// which tracking measures far more steadily than a three-finger fold.
    /// <list type="bullet">
    /// <item><b>A tap toggles the lock:</b> once the thumb and middle fingertips have touched (closer than
    /// <see cref="MiddleTapConfig.TouchMeters"/>) for <see cref="MiddleTapConfig.MinTouchSeconds"/>, while the index is apart. It
    /// answers then, without waiting for the release. Holding the touch toggles nothing more; the next tap needs a release
    /// (wider than <see cref="MiddleTapConfig.ReleaseMeters"/>).</item>
    /// <item><b>The gripper never toggles it:</b> a touch while the thumb and index are closer than
    /// <see cref="MiddleTapConfig.IndexApartMeters"/> counts for nothing, and so does one during which the index closes.</item>
    /// <item><b>An untracked frame says nothing</b> (or a gap that is not a number): it neither starts nor ends a touch, and
    /// losing the hand never unlocks the arm (the host holds it while the hand is gone).</item>
    /// </list>
    /// A plain struct with no clock of its own: the host passes the time, in seconds, from its injected clock.
    /// </summary>
    public struct MiddleTapToggle
    {
        private bool _touching, _armed;
        private double _touchStart;

        public bool IsLocked { get; private set; }

        /// <summary>Feeds one frame: the thumb-middle and thumb-index gaps (m). Returns whether the arm is locked after it.</summary>
        public bool Update(MiddleTapConfig config, bool tracked, double middleMeters, double indexMeters, double nowSeconds)
        {
            if (!tracked || double.IsNaN(middleMeters) || double.IsNaN(indexMeters) || double.IsNaN(nowSeconds)) return IsLocked;
            bool touching = _touching ? middleMeters < config.ReleaseMeters : middleMeters < config.TouchMeters;
            bool indexApart = indexMeters >= config.IndexApartMeters;
            if (touching && !_touching)
            {
                _touchStart = nowSeconds;
                _armed = indexApart;
            }

            if (touching && !indexApart) _armed = false; // the gripper closing, not a tap
            if (touching && _armed && nowSeconds - _touchStart >= config.MinTouchSeconds)
            {
                IsLocked = !IsLocked;
                _armed = false; // spent: holding on toggles nothing more
            }

            _touching = touching;
            return IsLocked;
        }

        /// <summary>Unlocks and forgets any touch in progress.</summary>
        public void Reset()
        {
            IsLocked = false;
            _touching = _armed = false;
        }
    }
}
