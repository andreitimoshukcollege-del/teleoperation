using System;
using System.Net;
using System.Threading;
using Teleop.Core.Pipeline;
using Teleop.Core.Transport;
using Teleop.JetRover.Wire;
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

            // Always a custom config, never JetRoverPlantConfig.Default directly: LowerArmMinPulse
            // is a real safety limit (see its own doc comment) that must always be applied, not
            // just when --max-direction-magnitude happens to also be overridden.
            var defaults = JetRoverPlantConfig.Default;
            var plantConfig = new JetRoverPlantConfig(
                links: defaults.Links,
                pulsePerRadian: defaults.PulsePerRadian,
                pulsePerDegreeAssumed180: defaults.PulsePerDegreeAssumed180,
                stepSizePulses: defaults.StepSizePulses,
                maxDirectionMagnitude: a.MaxDirectionMagnitude ?? defaults.MaxDirectionMagnitude,
                zeroPulse: defaults.ZeroPulse,
                minPulse: defaults.MinPulse,
                maxPulse: defaults.MaxPulse,
                gripperOpenDegrees: defaults.GripperOpenDegrees,
                gripperClosedDegrees: defaults.GripperClosedDegrees,
                lowerArmMinPulse: a.LowerArmMinPulse);
            var plant = new JetRoverPlant(plantConfig, relay);

            var endpoint = new RobotEndpoint(
                plant,
                new RawPoseCodec(),
                new RobotStateFrameCodec(),
                uplinkTransport: transport,
                downlinkTransport: transport,
                robotClock: clock);

            // Optional second, JetRover-specific listener for pre-computed joint-angle commands
            // (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) -- not a RobotEndpoint,
            // since that class is hardwired to one ICommandCodec/CommandFrame shape. Uplink-only:
            // it never replies, since a caller sending these already gets robot state feedback
            // through its own separate Cartesian connection above. Shares the same plant instance,
            // so JetRoverPlant.CommandJointAngles's calls interleave with Command's on the one
            // belief/staleness tracker -- see the ADR's documented limitation about running both
            // paths against the same robot process at the same time.
            using var jointTransport = a.JointLocalPort.HasValue
                ? new UdpTransport(a.JointLocalPort.Value, remoteEndPoint, MaxDatagramBytes, clock)
                : null;
            byte[]? jointRecvBuffer = jointTransport != null ? new byte[MaxDatagramBytes] : null;

            Console.WriteLine(
                $"Teleop.RobotHost listening on UDP :{a.LocalPort}, replying to " +
                $"{remoteEndPoint}, relay socket {a.RelaySocketPath}. Ctrl+C to stop.");
            Console.WriteLine(
                $"[clock] TicksPerSecond={clock.TicksPerSecond}, stamped on every RobotStateFrame reply so " +
                "the operator can normalize for a mismatched rate automatically " +
                "(docs/adr/0008-clocksync-cross-rate-normalization.md).");
            Console.WriteLine(
                $"[plant] MaxDirectionMagnitude={plantConfig.MaxDirectionMagnitude:0.##} " +
                $"LowerArmMinPulse={plantConfig.LowerArmMinPulse}");
            if (jointTransport != null)
            {
                Console.WriteLine(
                    $"[joint] listening on UDP :{a.JointLocalPort} for pre-computed joint-angle " +
                    "commands (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md), uplink-only.");
            }

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            while (!stop.IsSet)
            {
                if (jointTransport != null)
                {
                    while (jointTransport.TryReceive(clock.NowTicks, jointRecvBuffer!, out int byteCount, out long _))
                    {
                        if (JointCommandCodec.TryDecode(jointRecvBuffer.AsSpan(0, byteCount), out JointCommandFrame frame))
                        {
                            plant.CommandJointAngles(
                                frame.BaseYaw, frame.LowerPitch, frame.MiddlePitch, frame.UpperPitch,
                                frame.Gripper, frame.CaptureTicks);
                        }
                    }
                }

                endpoint.Step(clock.NowTicks);
                Thread.Sleep(5);
            }

            Console.WriteLine("Stopped.");
            return 0;
        }
    }
}
