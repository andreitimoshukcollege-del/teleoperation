using System;
using System.Net;
using System.Threading;
using Teleop.Core.Pipeline;
using Teleop.Core.Transport;
using Teleop.RobotHost.Net;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;
using Teleop.RobotHost.Time;

namespace Teleop.RobotHost
{
    /// <summary>
    /// The robot-side host process (docs/adr/0007-jetrover-plant-and-robot-host.md): the actual
    /// <see cref="RobotEndpoint"/> (Core, unmodified) end of a real <see cref="UdpTransport"/>
    /// link, driving a real <see cref="JetRoverPlant"/> through a local relay to the JetRover's
    /// ROS 2 node. Runs forever, unattended -- unlike <c>Teleop.Eval</c>, which is a CLI that
    /// runs once and exits (root CLAUDE.md invariant 10's exit-code contract), this is a
    /// long-running server, which is why it is its own project rather than a new
    /// <c>Teleop.Eval</c> verb.
    /// </summary>
    internal static class Program
    {
        private const int MaxDatagramBytes = 128;

        private static int Main(string[] args)
        {
            RobotHostArgs? parsed = RobotHostArgs.TryParse(args, out string? error);
            if (parsed is null)
            {
                Console.Error.WriteLine(error);
                Console.Error.WriteLine(RobotHostArgs.Usage);
                return 1;
            }

            RobotHostArgs a = parsed.Value;

            var clock = new MonotonicClock();
            var remoteEndPoint = new IPEndPoint(a.RemoteHost, a.RemotePort);
            var transport = new UdpTransport(a.LocalPort, remoteEndPoint, MaxDatagramBytes, clock);

            using var relay = new UdsRelayClient(a.LocalRelaySocketPath, a.RelaySocketPath);
            JetRoverPlantConfig plantConfig = a.MaxDirectionMagnitude is float overrideMagnitude
                ? new JetRoverPlantConfig(
                    links: JetRoverPlantConfig.Default.Links,
                    pulsePerRadian: JetRoverPlantConfig.Default.PulsePerRadian,
                    pulsePerDegreeAssumed180: JetRoverPlantConfig.Default.PulsePerDegreeAssumed180,
                    stepSizePulses: JetRoverPlantConfig.Default.StepSizePulses,
                    maxDirectionMagnitude: overrideMagnitude,
                    zeroPulse: JetRoverPlantConfig.Default.ZeroPulse,
                    minPulse: JetRoverPlantConfig.Default.MinPulse,
                    maxPulse: JetRoverPlantConfig.Default.MaxPulse,
                    gripperOpenDegrees: JetRoverPlantConfig.Default.GripperOpenDegrees,
                    gripperClosedDegrees: JetRoverPlantConfig.Default.GripperClosedDegrees)
                : JetRoverPlantConfig.Default;
            var plant = new JetRoverPlant(plantConfig, relay);

            var endpoint = new RobotEndpoint(
                plant,
                new RawPoseCodec(),
                new RobotStateFrameCodec(),
                uplinkTransport: transport,
                downlinkTransport: transport,
                robotClock: clock);

            Console.WriteLine(
                $"Teleop.RobotHost listening on UDP :{a.LocalPort}, replying to " +
                $"{remoteEndPoint}, relay socket {a.RelaySocketPath}. Ctrl+C to stop.");
            Console.WriteLine(
                $"[clock] TicksPerSecond={clock.TicksPerSecond}, stamped on every RobotStateFrame reply so " +
                "the operator can normalize for a mismatched rate automatically " +
                "(docs/adr/0008-clocksync-cross-rate-normalization.md).");
            Console.WriteLine($"[plant] MaxDirectionMagnitude={plantConfig.MaxDirectionMagnitude:0.##}");

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            while (!stop.IsSet)
            {
                endpoint.Step(clock.NowTicks);
                Thread.Sleep(5);
            }

            Console.WriteLine("Stopped.");
            return 0;
        }
    }
}
