using Teleop.RobotArm.Operator;

namespace Teleop.RobotArm.Tests.Operator
{
    /// <summary>One hand's grab and its gripper (docs/adr/0029 decision 2).</summary>
    public class GrabGestureTests
    {
        private static readonly GrabGestureConfig Config = new GrabGestureConfig();

        [Fact]
        public void TheGrab_EngagesBelowTheEngageCurl_HoldsInTheBand_AndLetsGoAboveTheReleaseCurl()
        {
            var g = new GrabGesture();
            Assert.False(g.Update(Config, true, 2.0f));
            Assert.False(g.Update(Config, true, 1.4f)); // in the band, not yet held: stays released
            Assert.True(g.Update(Config, true, 1.0f));
            Assert.True(g.Update(Config, true, 1.4f)); // in the band, held: stays held
            Assert.True(g.Update(Config, true, 1.55f));
            Assert.False(g.Update(Config, true, 1.56f));
        }

        [Fact]
        public void LosingTracking_OrANonNumber_LetsGo()
        {
            var g = new GrabGesture();
            Assert.True(g.Update(Config, true, 1.0f));
            Assert.False(g.Update(Config, false, 1.0f));
            Assert.True(g.Update(Config, true, 1.0f));
            Assert.False(g.Update(Config, true, float.NaN));
            Assert.True(g.Update(Config, true, 1.0f));
            Assert.False(g.Update(Config, true, float.PositiveInfinity));
        }

        [Fact]
        public void TheCurlRatio_ReadsAFlatHandAsOpen_AndAFistAsAGrab()
        {
            // Knuckles 9 cm from the wrist; a flat finger's tip 18 cm, a curled one's 8 cm.
            float flat = GrabGesture.CurlRatio(0.18f, 0.09f, 0.17f, 0.085f, 0.15f, 0.08f);
            float fist = GrabGesture.CurlRatio(0.08f, 0.09f, 0.075f, 0.085f, 0.07f, 0.08f);
            Assert.InRange(flat, 1.8f, 2.1f);
            Assert.InRange(fist, 0.8f, 1.0f);
            Assert.False(new GrabGesture().Update(Config, true, flat));
            Assert.True(new GrabGesture().Update(Config, true, fist));
            Assert.True(float.IsNaN(GrabGesture.CurlRatio(0.1f, 0f, 0.1f, 0.1f, 0.1f, 0.1f)));
        }

        [Fact]
        public void TheOpening_IsLinearBetweenTheTwoPinches_AndClamped()
        {
            Assert.Equal(1f, GrabGesture.Opening(Config, 0.08f), 5);
            Assert.Equal(1f, GrabGesture.Opening(Config, 0.12f), 5);
            Assert.Equal(0f, GrabGesture.Opening(Config, 0.015f), 5);
            Assert.Equal(0f, GrabGesture.Opening(Config, 0.0f), 5);
            Assert.Equal(0.5f, GrabGesture.Opening(Config, (0.08f + 0.015f) / 2), 5);
            Assert.True(float.IsNaN(GrabGesture.Opening(Config, float.NaN)));
        }

        [Fact]
        public void AConfig_ThatWouldChatterOrInvert_IsRefused()
        {
            Assert.Null(Config.Validate());
            Assert.NotNull(new GrabGestureConfig { EngageBelowCurl = 1.5f, ReleaseAboveCurl = 1.5f }.Validate());
            Assert.NotNull(new GrabGestureConfig { PinchOpenMeters = 0.01f, PinchClosedMeters = 0.02f }.Validate());
        }
    }
}
