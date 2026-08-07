using Xunit;

namespace Teleop.RobotHost.Tests.Relay
{
    /// <summary>
    /// Windows' AF_UNIX implementation supports only <c>SOCK_STREAM</c>, not the
    /// <c>SOCK_DGRAM</c> this project's local relay channel uses (Linux supports both) --
    /// <see cref="UdsRelayClient"/> only ever runs on the Jetson (Linux) in production, but this
    /// repo's whole toolchain builds and tests through the Windows .NET SDK (root CLAUDE.md's
    /// Environment section). Skipping here is honest, not a fake pass: these tests genuinely
    /// cannot exercise real AF_UNIX+SOCK_DGRAM behavior on this host OS, and the actual
    /// functionality is verified for real against the physical robot as part of the Phase 1
    /// end-to-end hardware smoke test (docs/adr/0007-jetrover-plant-and-robot-host.md).
    /// </summary>
    public sealed class LinuxOnlyFactAttribute : FactAttribute
    {
        public LinuxOnlyFactAttribute()
        {
            if (!OperatingSystem.IsLinux())
            {
                Skip = "AF_UNIX + SOCK_DGRAM is not supported on this OS (Windows only supports SOCK_STREAM); " +
                       "verified for real against the Jetson instead.";
            }
        }
    }
}
