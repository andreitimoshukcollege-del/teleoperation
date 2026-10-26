using System;
using System.IO;
using System.Net;
using System.Runtime.InteropServices;
using System.Threading;
using Teleop.Core.Pipeline;
using Teleop.Core.Transport;
using Teleop.RobotArm.Types;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Net;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
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
            if (args.Length == 1 && args[0] == "--clock-echo")
            {
                return ClockEcho();
            }

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

            // Recording (docs/adr/0021): robothost.jlog into the session directory the control file
            // names, while it exists. Off without --record-control; costs nothing while no session is open.
            // With --auto-record, RobotHost also opens sessions itself while it is commanded (docs/adr/0023).
            using var recorder = a.RecordControlPath != null
                ? new SessionRecorder(a.RecordControlPath, clock.TicksPerSecond, auto: a.AutoRecord)
                : null;
            // What this session's file already says about the subscriber, the sender and the uplink.
            var sessionState = recorder != null ? new SessionStateLines(recorder.Record) : null;
            IRelayClient plantRelay = recorder != null ? new RecordingRelayClient(relay, recorder, clock) : relay;

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
            // The headset sends its IK's joint angles and, for timing, the Cartesian target; only the
            // joint angles move the arm while they arrive (docs/adr/0020).
            var plant = new GenericArmPlant(plantConfig, plantRelay) { JointPrecedenceTicks = clock.TicksPerSecond / 2 };

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
            // A physically impaired uplink for recorded sessions (docs/adr/0021): off unless the uplink
            // control file names a profile. Receive side only, so frames to the headset are untouched.
            var uplink = jointTransport != null ? new UplinkImpairedReceiver(jointTransport, clock.TicksPerSecond, MaxJointDatagramBytes) : null;
            string? uplinkRequest = null;
            long nextUplinkPollTicks = long.MinValue;
            // A schedule in the control file (docs/adr/0021 decision 13): switched here, on time,
            // rather than by whoever edits the file.
            UplinkSchedule? uplinkSchedule = null;
            long uplinkScheduleStart = 0;
            int uplinkScheduleIndex = -1;
            ulong uplinkSeedNow = 0;
            JointTarget[]? jointTargetsBuffer = jointTransport != null
                ? new JointTarget[JointCommandCodec.MaxJointsPerMessage]
                : null;
            // The joint-state downlink (docs/adr/0018): streams to whoever last sent a keepalive on
            // this same port, at 50 Hz, whether or not commands are arriving.
            var jointStreamer = new JointStateStreamer(clock.TicksPerSecond);
            byte[] jointStateBuffer = new byte[MaxJointDatagramBytes];
            Func<JointSample[], int> readJoints = buffer => plant.ReadJointState(buffer);
            bool loggedSubscriber = false;
            JointSample[] recordedJoints = new JointSample[JointStateCodec.MaxJointsPerMessage];
            EndPoint? currentSubscriber = null;
            string? currentSubscriberText = null;
            bool subscriberLapsed = false;
            string? jointCommanderText = null;
            long lastJointCommandTicks = long.MinValue;

            // Recording whenever the robot is commanded (docs/adr/0023 decision 4).
            AutoSessionRecorder? auto = null;
            if (recorder != null && a.AutoRecord != null && jointTransport != null)
            {
                string autoRoot = recorder.AutoRoot;
                Directory.CreateDirectory(autoRoot);
                auto = new AutoSessionRecorder(a.AutoRecord, recorder, clock.TicksPerSecond, () => DateTime.UtcNow,
                    () => FreeBytes(autoRoot),
                    () => uplinkSchedule != null
                        ? new UplinkCondition(uplink!.Profile, uplinkSchedule.Seed, uplinkSchedule.Spec)
                        : new UplinkCondition(uplink!.Profile, uplinkSeedNow),
                    Console.WriteLine);
                long? freeNow = FreeBytes(autoRoot);
                Console.WriteLine($"[record] automatic sessions into {autoRoot} while commanded ({a.AutoRecord}); " +
                    (freeNow.HasValue ? $"{freeNow.Value / 1073741824.0:0.00} GB free now" : "free space unknown"));
            }

            // The start of every session's file: the state it would otherwise only learn on a change.
            void StartSessionState(long now)
            {
                sessionState!.Reset();
                if (currentSubscriberText != null && jointStreamer.IsSubscribed(now)) sessionState.Subscriber(now, currentSubscriberText);
                if (jointCommanderText != null && lastJointCommandTicks != long.MinValue && now - lastJointCommandTicks <= 3 * clock.TicksPerSecond)
                {
                    sessionState.Sender(now, jointCommanderText);
                }

                sessionState.Uplink(now, uplink?.Profile ?? "none", uplinkSeedNow);
            }

            // Discovery (docs/adr/0022): the headset asks here where the robot is, so a new DHCP lease
            // needs no config push. Its own port: a query on the joint port would take the joint stream.
            using var discovery = a.DiscoveryPort.HasValue ? new DiscoveryResponder(a.DiscoveryPort.Value) : null;

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

            if (discovery != null)
            {
                Console.WriteLine(
                    $"[discovery] answering on UDP :{discovery.Port} with this machine's LAN addresses " +
                    $"(now {string.Join(", ", DiscoveryResponder.LanAddresses())}) (docs/adr/0022)");
            }

            using var stop = new ManualResetEventSlim(initialState: false);
            Console.CancelKeyPress += (_, cancelArgs) =>
            {
                cancelArgs.Cancel = true;
                stop.Set();
            };
            // systemctl stop/restart: leave the loop as Ctrl+C does, so an open session gets its trailer.
            PosixSignalRegistration? sigterm = null;
            try
            {
                sigterm = PosixSignalRegistration.Create(PosixSignal.SIGTERM, context =>
                {
                    context.Cancel = true;
                    stop.Set();
                });
            }
            catch (PlatformNotSupportedException)
            {
            }

            EndPoint? loggedReplyTarget = transport.CurrentRemoteEndPoint;
            EndPoint? jointCommander = null;
            int loggedRefusals = 0;
            bool loggedPrecedence = false;
            long lastRefusalLogTicks = long.MinValue;
            while (!stop.IsSet)
            {
                auto?.Tick(clock.NowTicks);
                if (recorder != null && recorder.TakePreambleOwed()) StartSessionState(clock.NowTicks);

                if (jointTransport != null)
                {
                    if (a.UplinkControlPath != null && clock.NowTicks >= nextUplinkPollTicks)
                    {
                        nextUplinkPollTicks = clock.NowTicks + clock.TicksPerSecond / 2;
                        string? request = File.Exists(a.UplinkControlPath) ? File.ReadAllText(a.UplinkControlPath).Trim() : null;
                        if (request?.Length == 0) request = null;
                        if (request != uplinkRequest && UplinkSchedule.IsSchedule(request))
                        {
                            uplinkRequest = request;
                            if (UplinkSchedule.TryParse(request!, clock.TicksPerSecond, out UplinkSchedule? schedule, out string? why))
                            {
                                uplinkSchedule = schedule;
                                uplinkScheduleStart = clock.NowTicks;
                                uplinkScheduleIndex = -1;
                                Console.WriteLine($"[uplink] schedule '{schedule!.Spec}' (seed {schedule.Seed}) from now");
                            }
                            else
                            {
                                Console.Error.WriteLine($"[uplink] ignoring '{request}': {why}; still '{uplink!.Profile}'");
                            }
                        }
                        else if (request != uplinkRequest)
                        {
                            uplinkRequest = request;
                            uplinkSchedule = null;
                            string[] parts = (request ?? "none 0").Split(' ', StringSplitOptions.RemoveEmptyEntries);
                            ulong uplinkSeed = parts.Length > 1 && ulong.TryParse(parts[1], out ulong parsedSeed) ? parsedSeed : 1UL;
                            try
                            {
                                uplink!.SetProfile(parts[0], uplinkSeed);
                                uplinkSeedNow = uplinkSeed;
                                Console.WriteLine($"[uplink] joint commands now through '{uplink.Profile}' (seed {uplinkSeed})");
                                if (recorder != null && recorder.IsActive) sessionState!.Uplink(clock.NowTicks, uplink.Profile, uplinkSeed);
                            }
                            catch (ArgumentException e)
                            {
                                Console.Error.WriteLine($"[uplink] ignoring '{request}': {e.Message}; still '{uplink!.Profile}'");
                            }
                        }
                    }

                    if (uplinkSchedule != null)
                    {
                        int index = uplinkSchedule.IndexAt((clock.NowTicks - uplinkScheduleStart) / (double)clock.TicksPerSecond);
                        if (index != uplinkScheduleIndex)
                        {
                            uplinkScheduleIndex = index;
                            ulong segmentSeed = uplinkSchedule.Seed + (ulong)index;
                            uplink!.SetProfile(uplinkSchedule.ProfileOf(index), segmentSeed, drainPrevious: true);
                            uplinkSeedNow = segmentSeed;
                            Console.WriteLine($"[uplink] schedule segment {index}: joint commands now through '{uplink.Profile}' (seed {segmentSeed})");
                            if (recorder != null && recorder.IsActive) sessionState!.Uplink(clock.NowTicks, uplink.Profile, segmentSeed);
                        }
                    }

                    while (uplink!.TryReceive(clock.NowTicks, jointRecvBuffer!, out int byteCount, out long socketTicks, out long arrivalTicks))
                    {
                        if (jointStreamer.TryHandleSubscribe(jointRecvBuffer.AsSpan(0, byteCount), arrivalTicks))
                        {
                            EndPoint? subscriber = jointTransport.CurrentRemoteEndPoint;
                            if (!Equals(subscriber, currentSubscriber) || subscriberLapsed)
                            {
                                // A new subscriber splits an automatic session (docs/adr/0023): one per session.
                                currentSubscriber = subscriber;
                                currentSubscriberText = subscriber?.ToString() ?? "?";
                                subscriberLapsed = false;
                                auto?.OnSubscriber(arrivalTicks, currentSubscriberText);
                            }

                            if (recorder != null && recorder.IsActive &&
                                JointStateCodec.TryDecodeSubscribe(jointRecvBuffer.AsSpan(0, byteCount), out long operatorTicks, out long operatorTps))
                            {
                                sessionState!.Subscriber(arrivalTicks, currentSubscriberText);
                                recorder.Record(SessionLines.Keepalive(arrivalTicks, operatorTicks, operatorTps));
                            }

                            if (!loggedSubscriber)
                            {
                                Console.WriteLine($"[joint] streaming joint state to {jointTransport.CurrentRemoteEndPoint}");
                                loggedSubscriber = true;
                            }

                            continue;
                        }

                        if (JointCommandCodec.TryDecode(
                            jointRecvBuffer.AsSpan(0, byteCount), out uint commandSequence, out long captureTicks,
                            jointTargetsBuffer!, out int targetCount))
                        {
                            // A new sender's capture stamps are another clock's: start the
                            // staleness check over, or a rebooted headset is ignored for good.
                            EndPoint? sender = jointTransport.CurrentRemoteEndPoint;
                            if (!Equals(sender, jointCommander) || jointCommanderText == null)
                            {
                                plant.RestartJointCommandOrdering();
                                Console.WriteLine($"[joint] commands from {sender} (was {jointCommander?.ToString() ?? "nobody"}); staleness check restarted");
                                jointCommander = sender;
                                jointCommanderText = sender?.ToString() ?? "?";
                            }

                            // Before the plant takes it: the command may open the session it belongs in
                            // (docs/adr/0023), and its relay output (RO) is recorded inside the plant.
                            lastJointCommandTicks = arrivalTicks;
                            auto?.OnJointCommand(arrivalTicks, jointCommanderText);
                            if (recorder != null)
                            {
                                if (recorder.TakePreambleOwed()) StartSessionState(arrivalTicks);
                                if (recorder.IsActive) sessionState!.Sender(arrivalTicks, jointCommanderText);
                            }

                            JointCommandOutcome outcome = plant.CommandJointAngles(jointTargetsBuffer!.AsSpan(0, targetCount), captureTicks);
                            if (recorder != null && recorder.IsActive)
                            {
                                // Read from the socket at socketTicks; reached the plant now (later only when the uplink is impaired).
                                recorder.Record(SessionLines.JointCommand(socketTicks, arrivalTicks, commandSequence, captureTicks, outcome,
                                    jointTargetsBuffer.AsSpan(0, targetCount)));
                            }
                        }
                    }
                }

                endpoint.Step(clock.NowTicks);
                discovery?.Poll();

                if (!loggedPrecedence && plant.IgnoredCartesianCommands > 0)
                {
                    Console.WriteLine("[plant] joint commands are arriving: they alone move the arm; Cartesian commands only get state replies");
                    loggedPrecedence = true;
                }

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
                        // The frame as it went out, decoded from its own bytes.
                        if (recorder != null && recorder.IsActive &&
                            JointStateCodec.TryDecode(jointStateBuffer.AsSpan(0, stateBytes), out uint frameSequence, out long robotSend,
                                out long sensed, out long echoOperator, out long echoRobotRecv, out _, recordedJoints, out int frameJoints))
                        {
                            recorder.Record(SessionLines.Frame(now, frameSequence, robotSend, sensed, echoOperator, echoRobotRecv,
                                recordedJoints.AsSpan(0, frameJoints)));
                        }
                    }
                    else if (loggedSubscriber && !jointStreamer.IsSubscribed(now))
                    {
                        Console.WriteLine("[joint] keepalive stopped; joint-state stream paused");
                        loggedSubscriber = false;
                        subscriberLapsed = true;
                        auto?.OnSubscriber(now, null);
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

            sigterm?.Dispose();
            Console.WriteLine("Stopped.");
            return 0;
        }

        /// <summary>Free space where the recordings go, or null when it cannot be read.</summary>
        private static long? FreeBytes(string path)
        {
            try
            {
                return new DriveInfo(path).AvailableFreeSpace;
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is ArgumentException)
            {
                return null;
            }
        }

        /// <summary>
        /// <c>--clock-echo</c> (<c>just jetson-clock-check</c>, docs/adr/0021): for each line on stdin,
        /// print this process's clock (<c>Stopwatch</c> ticks and their rate) and flush. A caller that
        /// reads CLOCK_MONOTONIC just before writing and just after reading brackets the reply within
        /// a fraction of a millisecond, which proves (or disproves) that RobotHost and the driver
        /// stamp their logs on the same clock.
        /// </summary>
        private static int ClockEcho()
        {
            string? line;
            while ((line = Console.In.ReadLine()) != null)
            {
                long ticks = System.Diagnostics.Stopwatch.GetTimestamp();
                Console.Out.WriteLine($"{ticks} {System.Diagnostics.Stopwatch.Frequency}");
                Console.Out.Flush();
            }

            return 0;
        }
    }
}
