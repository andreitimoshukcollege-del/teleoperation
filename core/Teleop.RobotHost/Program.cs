using System;
using System.Net;
using System.Threading;
using Teleop.Core.Pipeline;
using Teleop.Core.Transport;
using Teleop.RobotArm.Types;
using Teleop.Core.Types;
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
            IPEndPoint? remoteEndPoint = a.RemoteHost != null
                ? new IPEndPoint(a.RemoteHost, a.RemotePort!.Value)
                : null;
            var transport = new UdpTransport(
                a.LocalPort, remoteEndPoint, MaxDatagramBytes, clock, replyToSender: a.ReplyToSender);

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
                pulsesPerSecond: a.PulsesPerSecond ?? defaults.PulsesPerSecond,
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
                // Uplink-only, so it never sends; reply-to-sender mode just lets it run without a
                // configured peer.
                ? new UdpTransport(a.JointLocalPort.Value, remoteEndPoint, MaxJointDatagramBytes, clock, replyToSender: true)
                : null;
            byte[]? jointRecvBuffer = jointTransport != null ? new byte[MaxJointDatagramBytes] : null;
            JointTarget[]? jointTargetsBuffer = jointTransport != null
                ? new JointTarget[JointCommandCodec.MaxJointsPerMessage]
                : null;
            // The joint-state downlink (docs/adr/0018): streams to whoever last sent a keepalive on
            // this same port, at 50 Hz, whether or not commands are arriving.
            var jointStreamer = new JointStateStreamer(clock.TicksPerSecond);
            byte[] jointStateBuffer = new byte[MaxJointDatagramBytes];
            Func<JointSample[], int> readJoints = buffer => plant.ReadJointState(buffer);
            bool loggedSubscriber = false;

            string replyDescription = a.ReplyToSender
                ? $"the sender of each command (before the first one: {remoteEndPoint?.ToString() ?? "nobody"})"
                : remoteEndPoint!.ToString();
            Console.WriteLine(
                $"Teleop.RobotHost listening on UDP :{a.LocalPort}, replying to " +
                $"{replyDescription}, relay socket {a.RelaySocketPath}. Ctrl+C to stop.");
            Console.WriteLine(
                $"[clock] TicksPerSecond={clock.TicksPerSecond}, stamped on every RobotStateFrame reply so " +
                "the operator can normalize for a mismatched rate automatically " +
                "(docs/adr/0008-clocksync-cross-rate-normalization.md).");
            Console.WriteLine(
                $"[plant] profile={profile.Name} JointCount={profile.JointCount} " +
                $"MaxDirectionMagnitude={plantConfig.MaxDirectionMagnitude:0.##} PulsesPerSecond={plantConfig.PulsesPerSecond:0.##}");
            if (jointTransport != null)
            {
                Console.WriteLine(
                    $"[joint] listening on UDP :{a.JointLocalPort} for pre-computed joint-angle " +
                    "commands (docs/adr/0009-jetrover-operator-side-inverse-kinematics.md), and streaming " +
                    "joint state at 50 Hz to whoever sends it a keepalive (docs/adr/0018).");
            }

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };

            EndPoint? loggedReplyTarget = transport.CurrentRemoteEndPoint;
            EndPoint? jointCommander = null;
            int loggedRefusals = 0;
            long lastRefusalLogTicks = long.MinValue;
            while (!stop.IsSet)
            {
                if (jointTransport != null)
                {
                    while (jointTransport.TryReceive(clock.NowTicks, jointRecvBuffer!, out int byteCount, out long arrivalTicks))
                    {
                        if (jointStreamer.TryHandleSubscribe(jointRecvBuffer.AsSpan(0, byteCount), arrivalTicks))
                        {
                            if (!loggedSubscriber)
                            {
                                Console.WriteLine($"[joint] streaming joint state to {jointTransport.CurrentRemoteEndPoint}");
                                loggedSubscriber = true;
                            }

                            continue;
                        }

                        if (JointCommandCodec.TryDecode(
                            jointRecvBuffer.AsSpan(0, byteCount), out uint _, out long captureTicks,
                            jointTargetsBuffer!, out int targetCount))
                        {
                            // A new sender's capture stamps are another clock's: start the
                            // staleness check over, or a rebooted headset is ignored for good.
                            EndPoint? sender = jointTransport.CurrentRemoteEndPoint;
                            if (!Equals(sender, jointCommander))
                            {
                                plant.RestartJointCommandOrdering();
                                Console.WriteLine($"[joint] commands from {sender} (was {jointCommander?.ToString() ?? "nobody"}); staleness check restarted");
                                jointCommander = sender;
                            }

                            plant.CommandJointAngles(jointTargetsBuffer!.AsSpan(0, targetCount), captureTicks);
                        }
                    }
                }

                endpoint.Step(clock.NowTicks);

                // Refused joint commands (a pose that would collide, docs/adr/0020), at most one line
                // every 2 s while they keep coming.
                if (plant.RefusedJointCommands != loggedRefusals && clock.NowTicks - lastRefusalLogTicks >= 2 * clock.TicksPerSecond)
                {
                    Console.WriteLine($"[plant] refused {plant.RefusedJointCommands - loggedRefusals} joint command(s): the pose would collide " +
                        $"({plant.DescribeLastRefusal()}); the arm holds");
                    loggedRefusals = plant.RefusedJointCommands;
                    lastRefusalLogTicks = clock.NowTicks;
                }

                if (jointTransport != null)
                {
                    long now = clock.NowTicks;
                    if (jointStreamer.TryBuildFrame(now, readJoints, plant.LastFeedbackTicks, jointStateBuffer, out int stateBytes))
                    {
                        jointTransport.Send(jointStateBuffer.AsSpan(0, stateBytes), now);
                    }
                    else if (loggedSubscriber && !jointStreamer.IsSubscribed(now))
                    {
                        Console.WriteLine("[joint] keepalive stopped; joint-state stream paused");
                        loggedSubscriber = false;
                    }
                }

                // Logged once per change, not per datagram: a moved operator is worth a line in
                // the service journal, a steady one is not.
                EndPoint? replyTarget = transport.CurrentRemoteEndPoint;
                if (!Equals(replyTarget, loggedReplyTarget))
                {
                    // A new Cartesian sender: as on the joint path, its stamps are another clock's.
                    // Its first command was judged against the old sender's and may have been
                    // dropped; the next one is not.
                    plant.RestartCartesianCommandOrdering();
                    Console.WriteLine($"[net] replying to {replyTarget} (was {loggedReplyTarget?.ToString() ?? "nobody"}); staleness check restarted");
                    loggedReplyTarget = replyTarget;
                }

                Thread.Sleep(5);
            }

            Console.WriteLine("Stopped.");
            return 0;
        }
    }
}
