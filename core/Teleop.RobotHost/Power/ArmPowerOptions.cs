using System;
using System.Globalization;

namespace Teleop.RobotHost.Power
{
    /// <summary>
    /// "Park, then relax" (docs/adr/0023 decision 5): the rest pose and the thresholds, from
    /// <c>--rest-pose-deg</c>, <c>--park-speed-rad-per-sec</c>, <c>--park-tolerance-deg</c> and
    /// <c>--park-timeout-seconds</c>.
    /// </summary>
    public sealed class ArmPowerOptions
    {
        /// <summary>
        /// The rest pose, one angle per arm joint (every joint but the gripper) in the profile's order,
        /// degrees, in the joint-state stream's convention (the profile's zero offsets applied, as
        /// <c>just joint-probe</c> prints them). Null: no rest pose, and every relax is refused.
        /// </summary>
        public double[]? RestPoseDegrees { get; set; }

        /// <summary>How fast the furthest joint moves while parking, rad/s.</summary>
        public double ParkSpeedRadPerSec { get; set; } = 0.5;

        /// <summary>Every arm joint's fresh reading within this of the rest pose means the arm is there, degrees.</summary>
        public double ToleranceDegrees { get; set; } = 3.0;

        /// <summary>A park that has not arrived after this long ends back in holding, refused, seconds.</summary>
        public double ParkTimeoutSeconds { get; set; } = 20.0;

        /// <summary>
        /// <c>--park-contact-deg</c>: 0 (the default) checks the whole park path strictly, the rest pose
        /// included, as decided. A rest pose the arm lies <i>on</i> something in touches the model's body
        /// boxes (with its 1 cm margin) by definition; above 0, the samples within this many degrees of the
        /// rest pose (every joint) may touch exactly the fixed boxes the rest pose itself touches, with the
        /// same links, and nothing else. Every sample further away is still checked strictly.
        /// </summary>
        public double RestContactDegrees { get; set; }

        /// <summary>The arm must stay at the rest pose this long, with no command sent, before the torque goes off, seconds.</summary>
        public double SettleSeconds { get; set; } = 0.3;

        /// <summary>A reading older than this is not fresh, seconds.</summary>
        public double FreshSeconds { get; set; } = 0.5;

        /// <summary>No fresh reading of an arm joint for this long while parking: the board is not answering, seconds.</summary>
        public double BoardSilentSeconds { get; set; } = 2.0;

        /// <summary>How often a park waypoint is sent while moving, seconds (the relay's minimum move is 50 ms).</summary>
        public double WaypointSeconds { get; set; } = 0.05;

        /// <summary>How often the status repeats while not holding, seconds.</summary>
        public double StatusRepeatSeconds { get; set; } = 1.0;

        /// <summary>The collision check's sample spacing along the park path, degrees.</summary>
        public double PathStepDegrees { get; set; } = 2.0;

        /// <summary>The reason these options cannot be used, or null.</summary>
        public string? Validate()
        {
            if (!(ParkSpeedRadPerSec > 0) || ParkSpeedRadPerSec > 3) return "--park-speed-rad-per-sec must be above 0 and at most 3";
            if (!(ToleranceDegrees > 0) || ToleranceDegrees > 20) return "--park-tolerance-deg must be above 0 and at most 20";
            if (!(ParkTimeoutSeconds > 0) || ParkTimeoutSeconds > 600) return "--park-timeout-seconds must be above 0 and at most 600";
            if (!(RestContactDegrees >= 0) || RestContactDegrees > 45) return "--park-contact-deg must be 0 to 45";
            if (RestPoseDegrees != null)
            {
                foreach (double d in RestPoseDegrees)
                {
                    if (double.IsNaN(d) || double.IsInfinity(d) || Math.Abs(d) > 180) return "--rest-pose-deg values must be numbers within ±180";
                }
            }

            return null;
        }

        /// <summary>
        /// Parses <c>--rest-pose-deg</c>: comma-separated degrees, e.g. <c>-1.20,91.16,-78.48,-112.0,5.76</c>.
        /// Empty (or only spaces) is no rest pose. False, with a reason, for anything else that is not a
        /// list of numbers.
        /// </summary>
        public static bool TryParseRestPose(string text, out double[]? degrees, out string? error)
        {
            degrees = null;
            error = null;
            if (string.IsNullOrWhiteSpace(text)) return true;
            string[] parts = text.Split(',');
            var values = new double[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                if (!double.TryParse(parts[i].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]))
                {
                    error = $"--rest-pose-deg: '{parts[i]}' is not a number (expected comma-separated degrees, one per arm joint)";
                    return false;
                }
            }

            degrees = values;
            return true;
        }

        public override string ToString() =>
            (RestPoseDegrees == null ? "no rest pose" : "rest pose " + string.Join(", ", Array.ConvertAll(RestPoseDegrees,
                d => d.ToString("0.##", CultureInfo.InvariantCulture))) + " deg") +
            $", park at {ParkSpeedRadPerSec.ToString("0.##", CultureInfo.InvariantCulture)} rad/s, within " +
            $"{ToleranceDegrees.ToString("0.##", CultureInfo.InvariantCulture)} deg, timeout " +
            $"{ParkTimeoutSeconds.ToString("0.##", CultureInfo.InvariantCulture)} s, " +
            (RestContactDegrees > 0
                ? $"the rest pose's own contacts allowed within {RestContactDegrees.ToString("0.##", CultureInfo.InvariantCulture)} deg of it"
                : "the whole path checked strictly");
    }
}
