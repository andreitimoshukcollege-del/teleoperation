using System;
using System.IO;
using System.Net;
using System.Runtime.InteropServices;
using System.Threading;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.Net;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Recording;
using Teleop.RobotHost.Time;

namespace Teleop.RobotHost
{
    /// <summary>
    /// RobotHost for a serial-chain robot, such as the FR3 Duo (docs/adr/0027 decision 5). It is chosen by the
    /// profile's kind and runs instead of <see cref="Program"/>'s JetRover loop, which it leaves untouched. Only the
    /// joint port runs:
    /// <list type="bullet">
    /// <item>joint commands, through <see cref="SerialChainPlant"/> to an <see cref="IArmBackend"/>;</item>
    /// <item>joint-state frames at 50 Hz to whoever sends a keepalive (docs/adr/0018);</item>
    /// <item>the operator's heartbeat, the headset's conditions and its training course, recorded as on the JetRover.</item>
    /// </list>
    /// There is no Cartesian path, no relay and no arm power: a relax or hold request is recorded and refused
    /// with a log line until the real arm's ADR defines them.
    ///
    /// <para>The backend is <see cref="Fr3SimBackend"/>, the simulated arm, until the robot arrives. Sessions
    /// are <c>teleop-session</c> v2 (trainer/contract/SESSION_FORMAT.md). Besides the records the JetRover's
    /// sessions carry, they hold the targets sent to the backend (<c>AO</c>), and every 4 ms of the backend's
    /// own steps, the arm's measured state (<c>QS</c>), which is the truth.</para>
    /// </summary>
    internal static class SerialChainHost
    {
        /// <summary>Franka's "ready" pose for the left arm; the right arm is its mirror image (joints 1, 3, 5 and 7 negated).</summary>
        public static readonly double[] ReadyPose = { 0, -Math.PI / 4, 0, -3 * Math.PI / 4, 0, Math.PI / 2, Math.PI / 4 };

        /// <summary>The truth's spacing: every this many of the backend's 1 ms steps.</summary>
        public const int TruthEverySteps = 4;

        public static int Run(string[] args)
        {
            SerialChainHostArgs? a = SerialChainHostArgs.TryParse(args, out string error);
            if (a == null)
            {
                Console.Error.WriteLine(error);
                Console.Error.WriteLine(SerialChainHostArgs.Usage);
                return 1;
            }

            SerialChainProfile profile;
            Teleop.RobotArm.Collision.SerialChainCollisionModel? collision = null;
            try
            {
                profile = SerialChainProfileJson.Load(a.ProfilePath);
                if (a.CollisionModelPath != null)
                {
                    collision = SerialChainProfileJson.LoadCollision(a.CollisionModelPath);
                    if (collision.Profile != profile.Name) throw new InvalidDataException($"the collision model is for '{collision.Profile}', the profile is '{profile.Name}'");
                }
            }
            catch (Exception e) when (e is IOException || e is InvalidDataException || e is UnauthorizedAccessException || e is System.Text.Json.JsonException)
            {
                Console.Error.WriteLine($"[profile] {e.Message}");
                return 1;
            }

            if (a.AutoRecord != null) a.AutoRecord.ProfileName = profile.Name;

            var clock = new MonotonicClock();
            TrajectoryShaperConfig shaper = SerialChainShaping.ForProfile(profile);
            double[] start = StartPose(profile, shaper);
            var backend = new Fr3SimBackend(shaper, start, clock.TicksPerSecond, a.Sim);
            var plant = new SerialChainPlant(profile, backend, collision, ProfileOrder(profile, shaper, start));

            using var transport = new UdpTransport(a.JointLocalPort, null, JointDatagram.MaxBytes, clock, replyToSender: true);
            var uplink = new UplinkImpairedReceiver(transport, clock.TicksPerSecond, JointDatagram.MaxBytes);
            var streamer = new JointStateStreamer(clock.TicksPerSecond);
            using var recorder = a.RecordControlPath != null
                ? new SessionRecorder(a.RecordControlPath, clock.TicksPerSecond, auto: a.AutoRecord)
                : null;
            var sessionState = recorder != null ? new SessionStateLines(recorder.Record) : null;
            using var discovery = a.DiscoveryPort.HasValue ? new DiscoveryResponder(a.DiscoveryPort.Value) : null;

            ulong uplinkSeed = 0;
            AutoSessionRecorder? auto = null;
            if (recorder != null && a.AutoRecord != null)
            {
                string root = recorder.AutoRoot;
                Directory.CreateDirectory(root);
                auto = new AutoSessionRecorder(a.AutoRecord, recorder, clock.TicksPerSecond, () => DateTime.UtcNow,
                    () => FreeBytes(root), () => new UplinkCondition(uplink.Profile, uplinkSeed), Console.WriteLine);
                Console.WriteLine($"[record] automatic sessions into {root} while the headset commands ({a.AutoRecord})");
            }

            // The truth, from the backend's own steps rather than this loop's, so its spacing is exact.
            var motorIds = new byte[shaper.Count];
            for (int m = 0; m < shaper.Count; m++) motorIds[m] = shaper[m].MotorId;
            if (recorder != null)
            {
                backend.TruthEverySteps = TruthEverySteps;
                backend.TruthSink = (sensed, tracked) =>
                {
                    if (recorder.IsActive) recorder.Record(SessionLines.ControllerState(clock.NowTicks, sensed, motorIds, tracked));
                };
            }

            byte[] receive = new byte[JointDatagram.MaxBytes];
            byte[] frame = new byte[JointDatagram.MaxBytes];
            var targets = new JointTarget[JointCommandCodec.MaxJointsPerMessage];
            var recorded = new JointSample[JointStateCodec.MaxJointsPerMessage];
            Func<JointSample[], int> readJoints = buffer => plant.ReadJointState(clock.NowTicks, buffer);

            string? subscriberText = null, commanderText = null, operatorText = null, operatorKind = null, operatorMode = null;
            EndPoint? subscriber = null, commander = null;
            bool streaming = false, subscriberLapsed = false, loggedPowerRefusal = false;
            long lastCommandTicks = long.MinValue, lastOperatorTicks = long.MinValue, lastTrainingTicks = long.MinValue, lastHeadsetTicks = long.MinValue;
            TrainingState? lastTraining = null;
            HeadsetConditions? lastHeadset = null;
            string? uplinkRequest = null;
            long nextUplinkPoll = long.MinValue;

            void StartSessionState(long now)
            {
                sessionState!.Reset();
                if (subscriberText != null && streamer.IsSubscribed(now)) sessionState.Subscriber(now, subscriberText);
                if (commanderText != null && now - lastCommandTicks <= 3 * clock.TicksPerSecond) sessionState.Sender(now, commanderText);
                sessionState.Uplink(now, uplink.Profile, uplinkSeed);
                if (operatorText != null && now - lastOperatorTicks <= 3 * clock.TicksPerSecond) sessionState.Operator(now, operatorKind!, operatorMode!, operatorText);
                if (lastTraining.HasValue && now - lastTrainingTicks <= 3 * clock.TicksPerSecond) sessionState.Training(now, lastTraining.Value);
                if (lastHeadset.HasValue && now - lastHeadsetTicks <= 3 * clock.TicksPerSecond) sessionState.Headset(now, lastHeadset.Value);
            }

            Console.WriteLine(
                $"Teleop.RobotHost (serial chain): profile {profile.Name} ({a.ProfilePath}), {profile.MotorCount} motors in {profile.Arms.Length} arm(s); " +
                $"joint port UDP :{a.JointLocalPort}, frames at 50 Hz to whoever sends a keepalive (docs/adr/0027).");
            Console.WriteLine(
                $"[backend] SIMULATED arm (Fr3SimBackend): the shaper at {1 / shaper.StepSeconds:0} Hz, tracking {a.Sim.TrackHz:0.##} Hz damping {a.Sim.TrackDamping:0.##}, " +
                $"reading noise {a.Sim.NoiseRad} rad, seed {a.Sim.Seed}; starting at the ready pose. Nothing real moves.");
            Console.WriteLine(collision != null
                ? $"[collision] {collision.Capsules.Length} capsules, margin {collision.Margin * 1000:0} mm ({a.CollisionModelPath}): a colliding command is refused"
                : "[collision] no collision model (--collision-model): nothing is checked; a simulated arm only");
            if (discovery != null) Console.WriteLine($"[discovery] answering on UDP :{discovery.Port} (docs/adr/0022)");
            int loggedRefusals = 0;
            long lastRefusalLog = long.MinValue;
            Console.WriteLine("Enter (or Ctrl+C) stops.");

            // Windows sleeps in 15.6 ms ticks unless asked for 1 ms: without this, frames leave at about 36 Hz, not 50.
            bool raisedTimer = OperatingSystem.IsWindows() && TimeBeginPeriod(1) == 0;

            using var stop = new ManualResetEventSlim(false);
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                stop.Set();
            };
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

            // From WSL, Ctrl+C does not reach a Windows dotnet: Enter on stdin stops it too. End of input does not.
            var stdin = new Thread(() =>
            {
                try
                {
                    if (Console.In.ReadLine() != null) stop.Set();
                }
                catch (IOException)
                {
                }
            }) { IsBackground = true, Name = "stdin-stop" };
            stdin.Start();

            while (!stop.IsSet)
            {
                long now = clock.NowTicks;
                auto?.Tick(now);
                if (recorder != null && recorder.TakePreambleOwed()) StartSessionState(now);

                if (a.UplinkControlPath != null && now >= nextUplinkPoll)
                {
                    nextUplinkPoll = now + clock.TicksPerSecond / 2;
                    string? request = File.Exists(a.UplinkControlPath) ? File.ReadAllText(a.UplinkControlPath).Trim() : null;
                    if (request?.Length == 0) request = null;
                    if (request != uplinkRequest)
                    {
                        uplinkRequest = request;
                        string[] parts = (request ?? "none 0").Split(' ', StringSplitOptions.RemoveEmptyEntries);
                        ulong seed = parts.Length > 1 && ulong.TryParse(parts[1], out ulong s) ? s : 1UL;
                        try
                        {
                            uplink.SetProfile(parts[0], seed);
                            uplinkSeed = seed;
                            Console.WriteLine($"[uplink] joint commands now through '{uplink.Profile}' (seed {seed})");
                            if (recorder != null && recorder.IsActive) sessionState!.Uplink(now, uplink.Profile, seed);
                        }
                        catch (ArgumentException e)
                        {
                            Console.Error.WriteLine($"[uplink] ignoring '{request}': {e.Message}; still '{uplink.Profile}'");
                        }
                    }
                }

                while (uplink.TryReceive(clock.NowTicks, receive, out int bytes, out long socketTicks, out long arrival))
                {
                    ReadOnlySpan<byte> datagram = receive.AsSpan(0, bytes);
                    if (streamer.TryHandleSubscribe(datagram, arrival))
                    {
                        EndPoint? from = transport.CurrentRemoteEndPoint;
                        if (!Equals(from, subscriber) || subscriberLapsed)
                        {
                            subscriber = from;
                            subscriberText = from?.ToString() ?? "?";
                            subscriberLapsed = false;
                            auto?.OnSubscriber(arrival, subscriberText);
                        }

                        if (recorder != null && recorder.IsActive && JointStateCodec.TryDecodeSubscribe(datagram, out long opTicks, out long opTps))
                        {
                            sessionState!.Subscriber(arrival, subscriberText);
                            recorder.Record(SessionLines.Keepalive(arrival, opTicks, opTps));
                        }

                        if (!streaming)
                        {
                            Console.WriteLine($"[joint] streaming joint state to {from}");
                            streaming = true;
                        }

                        continue;
                    }

                    if (ArmPowerCodec.TryDecodeRequest(datagram, out OperatorClientKind kind, out OperatorMotionMode mode,
                            out ArmPowerRequest request, out uint powerSequence, out long powerOperatorTicks))
                    {
                        string text = transport.CurrentRemoteEndPoint?.ToString() ?? "?";
                        string kindName = SessionLines.KindName(kind), modeName = SessionLines.ModeName(mode);
                        if (kind == OperatorClientKind.Headset) auto?.OnHeadset(arrival, text);
                        if (text != operatorText || kindName != operatorKind || modeName != operatorMode) Console.WriteLine($"[operator] {kindName} at {text}, motion {modeName}");
                        operatorText = text;
                        operatorKind = kindName;
                        operatorMode = modeName;
                        lastOperatorTicks = arrival;
                        if (recorder != null && recorder.IsActive) sessionState!.Operator(arrival, kindName, modeName, text);
                        if (request != ArmPowerRequest.None)
                        {
                            if (recorder != null && recorder.IsActive) recorder.Record(SessionLines.PowerRequest(arrival, powerSequence, request, powerOperatorTicks));
                            if (!loggedPowerRefusal)
                            {
                                Console.WriteLine($"[power] {SessionLines.RequestName(request)} request ignored: a serial-chain robot has no arm power yet (docs/adr/0027 decision 5)");
                                loggedPowerRefusal = true;
                            }
                        }

                        continue;
                    }

                    if (HeadsetConditionsCodec.TryDecode(datagram, out HeadsetConditions headset, out _, out _))
                    {
                        lastHeadset = headset;
                        lastHeadsetTicks = arrival;
                        if (recorder != null && recorder.IsActive) sessionState!.Headset(arrival, headset);
                        continue;
                    }

                    if (TrainingTaskCodec.TryDecode(datagram, out TrainingState training, out _, out _))
                    {
                        lastTraining = training;
                        lastTrainingTicks = arrival;
                        if (recorder != null && recorder.IsActive) sessionState!.Training(arrival, training);
                        continue;
                    }

                    if (JointCommandCodec.TryDecode(datagram, out uint sequence, out long capture, targets, out int count))
                    {
                        EndPoint? sender = transport.CurrentRemoteEndPoint;
                        if (!Equals(sender, commander) || commanderText == null)
                        {
                            plant.RestartJointCommandOrdering();
                            Console.WriteLine($"[joint] commands from {sender} (was {commander?.ToString() ?? "nobody"}); staleness check restarted");
                            commander = sender;
                            commanderText = sender?.ToString() ?? "?";
                        }

                        lastCommandTicks = arrival;
                        auto?.OnJointCommand(arrival, commanderText);
                        if (recorder != null)
                        {
                            if (recorder.TakePreambleOwed()) StartSessionState(arrival);
                            if (recorder.IsActive) sessionState!.Sender(arrival, commanderText);
                        }

                        JointCommandOutcome outcome = plant.CommandJointTargets(targets.AsSpan(0, count), capture, arrival);
                        if (recorder != null && recorder.IsActive)
                        {
                            recorder.Record(SessionLines.JointCommand(socketTicks, arrival, sequence, capture, outcome, targets.AsSpan(0, count)));
                            if (outcome == JointCommandOutcome.Accepted) recorder.Record(SessionLines.ArmOut(arrival, plant.LastSent));
                        }
                    }
                }

                discovery?.Poll();
                now = clock.NowTicks;
                if (plant.RefusedCommands != loggedRefusals && now - lastRefusalLog >= 2 * clock.TicksPerSecond)
                {
                    Console.WriteLine($"[plant] refused {plant.RefusedCommands - loggedRefusals} joint command(s): the pose would collide ({plant.DescribeLastRefusal()}); the arm holds");
                    loggedRefusals = plant.RefusedCommands;
                    lastRefusalLog = now;
                }

                backend.ReadTruth(now, Span<double>.Empty, out _);   // the simulation, and its truth records, up to now

                if (streamer.TryBuildFrame(now, readJoints, plant.LastFeedbackTicks, frame, out int frameBytes))
                {
                    transport.Send(frame.AsSpan(0, frameBytes), now);
                    if (recorder != null && recorder.IsActive &&
                        JointStateCodec.TryDecode(frame.AsSpan(0, frameBytes), out uint frameSequence, out long robotSend, out long frameSensed,
                            out long echoOperator, out long echoRobotRecv, out _, recorded, out int frameJoints))
                    {
                        recorder.Record(SessionLines.Frame(now, frameSequence, robotSend, frameSensed, echoOperator, echoRobotRecv, recorded.AsSpan(0, frameJoints)));
                    }
                }
                else if (streaming && !streamer.IsSubscribed(now))
                {
                    Console.WriteLine("[joint] keepalive stopped; joint-state stream paused");
                    streaming = false;
                    subscriberLapsed = true;
                    auto?.OnSubscriber(now, null);
                }

                Thread.Sleep(1);
            }

            sigterm?.Dispose();
            if (raisedTimer) TimeEndPeriod(1);
            Console.WriteLine($"Stopped. {plant.AcceptedCommands} joint command(s) taken, {plant.StaleCommands} stale, {plant.RefusedCommands} refused (collision), {plant.UnknownTargets} target(s) for unknown motors, " +
                $"{plant.ClampedTargets} clamped; simulated {backend.Steps} ms" + (backend.SkippedTicks > 0 ? $", skipped {backend.SkippedTicks * 1000.0 / clock.TicksPerSecond:0} ms of a stalled loop" : "") + ".");
            return 0;
        }

        /// <summary>Every motor's start, in the shaper's order: the ready pose for each arm (mirrored for every arm after the first), hands open.</summary>
        public static double[] StartPose(SerialChainProfile profile, TrajectoryShaperConfig shaper)
        {
            var start = new double[shaper.Count];
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainArm arm = profile.Arms[a];
                for (int j = 0; j < arm.Joints.Length; j++)
                {
                    int m = shaper.IndexOf(arm.Joints[j].MotorId);
                    double q = j < ReadyPose.Length ? ReadyPose[j] : 0;
                    if (a % 2 == 1 && j % 2 == 0) q = -q;
                    ChainJointLimits l = arm.Joints[j].Limits;
                    start[m] = Math.Min(Math.Max(q, l.SoftLower), l.SoftUpper);
                }

                if (arm.HasGripper) start[shaper.IndexOf(arm.Gripper.MotorId)] = 1.0;
            }

            return start;
        }

        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint milliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint milliseconds);

        /// <summary>The shaper's per-motor values in the profile's motor order (each arm's joints, then its hand), as the plant takes them.</summary>
        private static double[] ProfileOrder(SerialChainProfile profile, TrajectoryShaperConfig shaper, double[] byShaper)
        {
            var out_ = new double[profile.MotorCount];
            int k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint j in arm.Joints) out_[k++] = byShaper[shaper.IndexOf(j.MotorId)];
                if (arm.HasGripper) out_[k++] = byShaper[shaper.IndexOf(arm.Gripper.MotorId)];
            }

            return out_;
        }

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
    }
}
