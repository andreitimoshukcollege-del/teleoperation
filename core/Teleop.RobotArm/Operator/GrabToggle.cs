using System;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The double-curl toggle's timing (docs/adr/0029 amendment). Defaults are first guesses, tuned on the headset.</summary>
    public sealed class GrabToggleConfig
    {
        /// <summary>A click is a curl held at least this long (s): shorter is tracking flicker.</summary>
        public double MinPressSeconds { get; set; } = 0.04;

        /// <summary>A click is a curl held at most this long (s), and the second click must start within this long of the first one's end.</summary>
        public double DoubleClickSeconds { get; set; } = 0.6;

        public string? Validate()
        {
            if (!(MinPressSeconds >= 0) || !(DoubleClickSeconds > MinPressSeconds) || double.IsInfinity(DoubleClickSeconds))
            {
                return $"a click must be shorter than the double-click window (min {MinPressSeconds} s, window {DoubleClickSeconds} s)";
            }

            return null;
        }
    }

    /// <summary>
    /// A double curl locks an arm onto its hand, and another double curl unlocks it (docs/adr/0029 amendment, the user's
    /// choice): hand tracking does not read a curled hand as curled at every angle, so holding the curl to stay engaged
    /// let go whenever the hand turned. Fed the <see cref="GrabGesture"/>'s held state each frame:
    /// <list type="bullet">
    /// <item><b>A click</b> is a curl that lasts between <see cref="GrabToggleConfig.MinPressSeconds"/> and
    /// <see cref="GrabToggleConfig.DoubleClickSeconds"/>.</item>
    /// <item><b>The lock toggles</b> when a second curl starts within the window after a click ended: on its start, so it
    /// answers at once. That curl is spent: its end starts no new click.</item>
    /// <item><b>An untracked frame says nothing</b>: it neither ends a curl nor starts one, so a tracking dropout cannot
    /// fake a double click, and losing the hand never unlocks the arm (the host holds it while the hand is gone).</item>
    /// </list>
    /// A plain struct with no clock of its own: the host passes the time, in seconds, from its injected clock.
    /// </summary>
    public struct GrabToggle
    {
        private bool _wasHeld, _spent, _pending;
        private double _pressStart, _clickEnd;

        public bool IsLocked { get; private set; }

        /// <summary>Feeds one frame; returns whether the arm is locked after it.</summary>
        public bool Update(GrabToggleConfig config, bool tracked, bool held, double nowSeconds)
        {
            if (!tracked || double.IsNaN(nowSeconds))
            {
                return IsLocked;
            }

            if (_pending && nowSeconds - _clickEnd > config.DoubleClickSeconds)
            {
                _pending = false;
            }

            if (held && !_wasHeld)
            {
                _pressStart = nowSeconds;
                _spent = false;
                if (_pending)
                {
                    IsLocked = !IsLocked;
                    _pending = false;
                    _spent = true;
                }
            }
            else if (!held && _wasHeld && !_spent)
            {
                double length = nowSeconds - _pressStart;
                _pending = length >= config.MinPressSeconds && length <= config.DoubleClickSeconds;
                _clickEnd = nowSeconds;
            }

            _wasHeld = held;
            return IsLocked;
        }

        /// <summary>Unlocks and forgets any click in progress.</summary>
        public void Reset()
        {
            IsLocked = false;
            _wasHeld = _spent = _pending = false;
        }
    }
}
