using Teleop.Core.Contracts;

namespace Teleop.Bridge
{
    /// <summary>
    /// The source of <c>DisplayOffset</c> for <c>t_photon = t_render + DisplayOffset</c>.
    /// Teleop/CLAUDE.md's "Time" section says this "comes from OpenXR <c>predictedDisplayTime</c>
    /// where available, otherwise a per-headset calibrated constant measured with the photodiode
    /// rig."
    ///
    /// <b>Only the calibrated-constant half is implemented here.</b> The live OpenXR
    /// predicted-display-time path (<c>UnityEngine.XR.XRDisplaySubsystem.TryGetDisplayTiming</c>,
    /// broadly) needs verification against the exact OpenXR/XR-SDK package versions actually
    /// installed, which requires a Unity Editor to compile-check -- not available to the agent
    /// that wrote this. Wiring it in is a deliberate follow-up, not an oversight: guessing at the
    /// API surface risks a Bridge file that does not compile at all, which is strictly worse than
    /// shipping the constant-only path now. The constant path is not a stub -- it returns a real,
    /// usable value -- so this is still "done," pending refinement.
    /// </summary>
    public sealed class XrDisplayTimeProvider
    {
        /// <summary>
        /// <see cref="DisplayCalibrationConfig.DisplayOffsetMilliseconds"/> converted to ticks on
        /// <paramref name="clock"/>'s timebase.
        /// </summary>
        public long GetDisplayOffsetTicks(ITimeAuthority clock, DisplayCalibrationConfig config)
        {
            return (long)(config.DisplayOffsetMilliseconds / 1000.0 * clock.TicksPerSecond);
        }
    }
}
