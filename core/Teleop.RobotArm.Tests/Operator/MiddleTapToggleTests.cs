using Teleop.RobotArm.Operator;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The lock as a thumb-to-middle tap (docs/adr/0029 amendment 10): taps toggle it, the gripper never does.</summary>
    public class MiddleTapToggleTests
    {
        private static readonly MiddleTapConfig Config = new MiddleTapConfig();
        private const double Frame = 1.0 / 90;
        private const double Open = 0.08, Apart = 0.06;

        /// <summary>Feeds <paramref name="seconds"/> of frames with these gaps, from <paramref name="t"/>; returns the time after.</summary>
        private static double Run(ref MiddleTapToggle toggle, double t, double seconds, double middle, double index, bool tracked = true)
        {
            for (double end = t + seconds; t < end; t += Frame) toggle.Update(Config, tracked, middle, index, t);
            return t;
        }

        [Fact]
        public void ATap_Locks_AndTheNextTapUnlocks()
        {
            var toggle = new MiddleTapToggle();
            double t = Run(ref toggle, 0, 0.2, Open, Apart);
            Assert.False(toggle.IsLocked);
            t = Run(ref toggle, t, 0.1, 0.005, Apart); // a quick tap
            Assert.True(toggle.IsLocked);
            t = Run(ref toggle, t, 0.3, Open, Apart);
            Assert.True(toggle.IsLocked);
            t = Run(ref toggle, t, 0.1, 0.005, Apart);
            Assert.False(toggle.IsLocked);
        }

        [Fact]
        public void HoldingTheTouch_TogglesOnce_AndItAnswersBeforeTheRelease()
        {
            var toggle = new MiddleTapToggle();
            double t = Run(ref toggle, 0, 0.05, 0.005, Apart);
            Assert.True(toggle.IsLocked); // within 50 ms of touching
            Run(ref toggle, t, 2.0, 0.005, Apart);
            Assert.True(toggle.IsLocked);
        }

        [Fact]
        public void AFlicker_ShorterThanTheMinimum_TogglesNothing()
        {
            var toggle = new MiddleTapToggle();
            double t = Run(ref toggle, 0, 0.1, Open, Apart);
            toggle.Update(Config, true, 0.005, Apart, t); // one frame
            Run(ref toggle, t + Frame, 0.3, Open, Apart);
            Assert.False(toggle.IsLocked);
        }

        [Fact]
        public void ClosingTheGripper_NeverToggles_EvenWhenTheMiddleFingerComesAlong()
        {
            var toggle = new MiddleTapToggle();
            double t = Run(ref toggle, 0, 0.1, Open, Apart);
            t = Run(ref toggle, t, 1.0, 0.01, 0.005); // a fist-like close: index and middle both on the thumb
            Assert.False(toggle.IsLocked);
            t = Run(ref toggle, t, 0.2, Open, Apart);
            // A touch during which the index closes before it toggles: the gripper too.
            toggle.Update(Config, true, 0.005, Apart, t);
            t = Run(ref toggle, t + Frame, 0.5, 0.005, 0.01);
            Assert.False(toggle.IsLocked);
        }

        [Fact]
        public void LosingTracking_NeitherUnlocksNorFakesATap()
        {
            var toggle = new MiddleTapToggle();
            double t = Run(ref toggle, 0, 0.1, 0.005, Apart);
            Assert.True(toggle.IsLocked);
            t = Run(ref toggle, t, 0.3, Open, Apart);
            t = Run(ref toggle, t, 1.0, 0.005, Apart, tracked: false); // lost: nothing counts
            Assert.True(toggle.IsLocked);
            t = Run(ref toggle, t, 0.2, double.NaN, Apart);
            Assert.True(toggle.IsLocked);
            toggle.Reset();
            Assert.False(toggle.IsLocked);
        }

        [Fact]
        public void TheConfig_RefusesARelease_BelowTheTouch()
        {
            Assert.Null(new MiddleTapConfig().Validate());
            Assert.NotNull(new MiddleTapConfig { TouchMeters = 0.03, ReleaseMeters = 0.02 }.Validate());
            Assert.NotNull(new MiddleTapConfig { IndexApartMeters = 0 }.Validate());
        }
    }
}
