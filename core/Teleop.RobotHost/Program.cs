using System;
using System.Net;
using System.Threading;
using Teleop.Core.Pipeline;
using Teleop.Core.Transport;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Net;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;
using Teleop.RobotHost.Time;

namespace Teleop.RobotHost
{
    /// <summary>
    /// The robot-side host process (docs/adr/0007-jetrover-plant-and-robot-host.md): the actual
    /// <see cref="RobotEndpoint"/> (Core, unmodified) end of a real <see cref="UdpTransport"/>
    /// link, driving a real <see cref="GenericArmPlant"/> through a local relay to the robot's
    /// ROS 2 node. Runs forever, unattended -- unlike <c>Teleop.Eval</c>, which is a CLI that
    /// runs once and exits (root CLAUDE.md invariant 10's exit-code contract), this is a
    /// long-running server, which is why it is its own project rather than a new
    /// <c>Teleop.Eval</c> verb.
    /// </summary>
    internal static class Program
    {
        private const int MaxDatagramBytes = 128;

        /// <summary>
        /// Separate size budget for the joint-angle listener, distinct from the Cartesian path's
        /// own <see cref="MaxDatagramBytes"/> -- each <see cref="UdpTransport"/> owns its own
        /// fixed receive buffer, so there is no reason the two hops must share one constant
        /// (docs/adr/0011-generic-robot-arm-profiles.md). Numerically identical to
        /// <see cref="MaxDatagramBytes"/> today only because nothing has needed to diverge yet.
        /// </summary>
        private const int MaxJointDatagramBytes = 128;

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

            // a.ProfilePath omitted falls back to RobotArmProfile.JetRoverMeasuredDefault, the
            // exact configuration this codebase has always run -- see RobotHostArgs.ProfilePath's
            // own doc comment.
            RobotArmProfile profile = a.ProfilePath != null
                ? RobotArmProfileJson.Load(a.ProfilePath)
                : RobotArmProfile.JetRoverMeasuredDefault;

            var defaults = GenericArmPlantConfig.Default;
            var plantConfig = new GenericArmPlantConfig(
                profile: profile,
                pulsePerRadian: defaults.PulsePerRadian,
                pulsesPerSecond: defaults.PulsesPerSecond,
                stepSizePulses: defaults.StepSizePulses,
                maxDirectionMagnitude: a.MaxDirectionMagnitude ?? defaults.MaxDirectionMagnitude,
                zeroPulse: defaults.ZeroPulse,
                minPulse: defaults.MinPulse,
                maxPulse: defaults.MaxPulse,
                gripperOpenPulse: defaults.GripperOpenPulse,
                gripperClosedPulse: defaults.GripperClosedPulse);
            var plant = new GenericArmPlant(plantConfig, relay);

            var endpoint = new RobotEndpoint(
                plant,
                new RawPoseCodec(),
                new RobotStateFrameCodec(),
                uplinkTransport: transport,
                downlinkTransport: transport,
                robotClock: clock);

            // Optional second listener for pre-computed joint-angle commands
            // (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md) -- not a RobotEndpoint,
            // since that class is hardwired to one ICommandCodec/CommandFrame shape. Uplink-only:
            // it never replies, since a caller sending these already gets robot state feedback
            // through its own separate Cartesian connection above. Shares the same plant instance,
            // so GenericArmPlant.CommandJointAngles's calls interleave with Command's on the one
            // belief/staleness tracker -- see the ADR's documented limitation about running both
            // paths against the same robot process at the same time.
            using var jointTransport = a.JointLocalPort.HasValue
                ? new UdpTransport(a.JointLocalPort.Value, remoteEndPoint, MaxJointDatagramBytes, clock)
                : null;
            byte[]? jointRecvBuffer = jointTransport != null ? new byte[MaxJointDatagramBytes] : null;
            JointTarget[]? jointTargetsBuffer = jointTransport != null
                ? new JointTarget[JointCommandCodec.MaxJointsPerMessage]
                : null;

            Console.WriteLine(
                $"Teleop.RobotHost listening on UDP :{a.LocalPort}, replying to " +
                $"{remoteEndPoint}, relay socket {a.RelaySocketPath}. Ctrl+C to stop.");
            Console.WriteLine(
                $"[clock] TicksPerSecond={clock.TicksPerSecond}, stamped on every RobotStateFrame reply so " +
                "the operator can normalize for a mismatched rate automatically " +
                "(docs/adr/0008-clocksync-cross-rate-normalization.md).");
            Console.WriteLine(
                $"[plant] profile={profile.Name} JointCount={profile.JointCount} " +
                $"MaxDirectionMagnitude={plantConfig.MaxDirectionMagnitude:0.##}");
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
                        if (JointCommandCodec.TryDecode(
                            jointRecvBuffer.AsSpan(0, byteCount), out uint _, out long captureTicks,
                            jointTargetsBuffer!, out int targetCount))
                        {
                            plant.CommandJointAngles(jointTargetsBuffer!.AsSpan(0, targetCount), captureTicks);
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
