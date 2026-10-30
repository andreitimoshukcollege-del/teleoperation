using Teleop.RobotArm.Operator;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>The double-curl lock (docs/adr/0029 amendment).</summary>
    public class GrabToggleTests
    {
        private static readonly GrabToggleConfig Config = new GrabToggleConfig();

        /// <summary>Plays a held/open script at 90 Hz: each (held, seconds) pair holds that state that long. Returns the lock after each segment.</summary>
        private static bool[] Play(ref GrabToggle toggle, ref double t, params (bool Held, double Seconds)[] script)
        {
            var locks = new bool[script.Length];
            for (int i = 0; i < script.Length; i++)
            {
                for (double end = t + script[i].Seconds; t < end - 1e-9; t += 1 / 90.0) toggle.Update(Config, true, script[i].Held, t);
                locks[i] = toggle.IsLocked;
            }

            return locks;
        }

        [Fact]
        public void ADoubleCurl_Locks_AndAnotherUnlocks()
        {
            var g = new GrabToggle();
            double t = 0;
            bool[] l = Play(ref g, ref t, (false, 0.5), (true, 0.15), (false, 0.15), (true, 0.15), (false, 1.0));
            Assert.Equal(new[] { false, false, false, true, true }, l);

            l = Play(ref g, ref t, (true, 0.15), (false, 0.15), (true, 0.15), (false, 1.0));
            Assert.Equal(new[] { true, true, false, false }, l);
        }

        [Fact]
        public void ASingleCurl_OrOneHeld_DoesNothing()
        {
            var g = new GrabToggle();
            double t = 0;
            Assert.All(Play(ref g, ref t, (false, 0.3), (true, 0.2), (false, 2.0)), locked => Assert.False(locked));
            Assert.All(Play(ref g, ref t, (true, 3.0), (false, 0.2), (true, 3.0), (false, 1.0)), locked => Assert.False(locked));
        }

        [Fact]
        public void CurlsTooFarApart_OrFlickers_AreNotADoubleClick()
        {
            var g = new GrabToggle();
            double t = 0;
            Assert.All(Play(ref g, ref t, (true, 0.15), (false, 0.9), (true, 0.15), (false, 1.0)), locked => Assert.False(locked));

            // A one-frame flicker is shorter than a click, so it cannot be the first of two.
            var f = new GrabToggle();
            double s = 0;
            f.Update(Config, true, false, s);
            f.Update(Config, true, true, s += 0.011);
            f.Update(Config, true, false, s += 0.011);
            f.Update(Config, true, true, s += 0.1);
            Assert.False(f.IsLocked);
        }

        [Fact]
        public void UntrackedFrames_SayNothing_AndNeverUnlock()
        {
            var g = new GrabToggle();
            double t = 0;
            Play(ref g, ref t, (true, 0.15), (false, 0.15), (true, 0.15), (false, 0.3));
            Assert.True(g.IsLocked);

            // The hand is lost for two seconds: still locked, and nothing it "did" counts.
            for (double end = t + 2; t < end; t += 1 / 90.0) g.Update(Config, false, (int)(t * 10) % 2 == 0, t);
            Assert.True(g.IsLocked);

            // Back, and a deliberate double curl unlocks.
            Play(ref g, ref t, (false, 0.3), (true, 0.15), (false, 0.15), (true, 0.15));
            Assert.False(g.IsLocked);
        }

        [Fact]
        public void TheSecondCurl_IsSpent_ItsEndStartsNoNewClick()
        {
            // Lock with curls 1+2; curl 3 right after must not pair with curl 2's end to unlock again.
            var g = new GrabToggle();
            double t = 0;
            bool[] l = Play(ref g, ref t, (true, 0.15), (false, 0.15), (true, 0.15), (false, 0.15), (true, 0.15), (false, 1.0));
            Assert.Equal(new[] { false, false, true, true, true, true }, l);
        }

        [Fact]
        public void TheConfig_IsChecked()
        {
            Assert.Null(Config.Validate());
            Assert.NotNull(new GrabToggleConfig { MinPressSeconds = 0.5, DoubleClickSeconds = 0.4 }.Validate());
        }
    }
}
