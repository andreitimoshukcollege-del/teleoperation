using System;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>The grab gesture's thresholds (docs/adr/0029 decision 2). Every default is a first guess, to be tuned on the headset.</summary>
    public sealed class GrabGestureConfig
    {
        /// <summary>The grab holds once the curl ratio (<see cref="GrabGesture.CurlRatio"/>) falls below this.</summary>
        public float EngageBelowCurl { get; set; } = 1.25f;

        /// <summary>The grab lets go once the curl ratio rises above this. Above <see cref="EngageBelowCurl"/>, so a borderline fist does not chatter.</summary>
        public float ReleaseAboveCurl { get; set; } = 1.55f;

        /// <summary>Thumb and index tips this far apart, or further, is the gripper fully open (m).</summary>
        public float PinchOpenMeters { get; set; } = 0.08f;

        /// <summary>Thumb and index tips this close, or closer, is the gripper fully closed (m).</summary>
        public float PinchClosedMeters { get; set; } = 0.015f;

        /// <summary>Null when the thresholds are usable; otherwise what is wrong with them.</summary>
        public string? Validate()
        {
            if (!(EngageBelowCurl > 0f) || !(ReleaseAboveCurl > EngageBelowCurl) || float.IsInfinity(ReleaseAboveCurl))
            {
                return $"the grab must engage below a positive curl and release above it (engage {EngageBelowCurl}, release {ReleaseAboveCurl})";
            }

            if (!(PinchClosedMeters >= 0f) || !(PinchOpenMeters > PinchClosedMeters) || float.IsInfinity(PinchOpenMeters))
            {
                return $"the gripper's open pinch must be wider than its closed pinch (open {PinchOpenMeters}, closed {PinchClosedMeters})";
            }

            return null;
        }
    }

    /// <summary>
    /// One hand's grab (docs/adr/0029 decision 2): the middle, ring and little fingers curled into the palm hold that
    /// hand's arm, the deadman of the two-arm clutch, and the thumb-index gap meanwhile sets that arm's gripper. A plain
    /// struct with hysteresis and no clock: the host feeds it every frame and owns what holding means.
    ///
    /// <para><b>The curl ratio</b> is each finger's tip-to-wrist distance over its knuckle-to-wrist distance, averaged
    /// over the three fingers. It does not depend on the hand's size or where it is: about 2 for a flat hand, about 1 for
    /// a fist. Losing tracking, or any reading that is not a number, lets go.</para>
    /// </summary>
    public struct GrabGesture
    {
        public bool IsHeld { get; private set; }

        /// <summary>Feeds one frame; returns whether the grab holds after it.</summary>
        public bool Update(GrabGestureConfig config, bool tracked, float curlRatio)
        {
            if (!tracked || float.IsNaN(curlRatio) || float.IsInfinity(curlRatio))
            {
                IsHeld = false;
            }
            else if (!IsHeld && curlRatio < config.EngageBelowCurl)
            {
                IsHeld = true;
            }
            else if (IsHeld && curlRatio > config.ReleaseAboveCurl)
            {
                IsHeld = false;
            }

            return IsHeld;
        }

        public void Reset() => IsHeld = false;

        /// <summary>
        /// The curl ratio from three fingers' tip-to-wrist and knuckle-to-wrist distances (m); NaN when a knuckle
        /// distance is not positive.
        /// </summary>
        public static float CurlRatio(float middleTip, float middleKnuckle, float ringTip, float ringKnuckle, float littleTip, float littleKnuckle)
        {
            if (!(middleKnuckle > 0f) || !(ringKnuckle > 0f) || !(littleKnuckle > 0f))
            {
                return float.NaN;
            }

            return (middleTip / middleKnuckle + ringTip / ringKnuckle + littleTip / littleKnuckle) / 3f;
        }

        /// <summary>The gripper's opening fraction (1 open, 0 closed) from the thumb-index gap, linear between the config's two distances.</summary>
        public static float Opening(GrabGestureConfig config, float pinchMeters)
        {
            if (float.IsNaN(pinchMeters))
            {
                return float.NaN;
            }

            float t = (pinchMeters - config.PinchClosedMeters) / (config.PinchOpenMeters - config.PinchClosedMeters);
            return Math.Min(1f, Math.Max(0f, t));
        }
    }
}
