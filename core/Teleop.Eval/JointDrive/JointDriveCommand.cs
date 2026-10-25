using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// <c>joint-drive</c> (docs/adr/0021's data-collection tool): drives the real JetRover through a
    /// scripted session of joint moves, the way the headset does, with no headset and no operator.
    /// It sends joint commands to RobotHost's joint port at the headset's 48 Hz, sends the keepalive
    /// on the same socket so the joint-state stream comes back to it, and writes every command it
    /// sends to <c>operator.jlog</c>. <c>just record-drive</c> runs it inside a recorded session.
    ///
    /// <para><b>It moves the real arm.</b> Someone watches the whole time, with the space around the
    /// arm clear: the collision model knows the robot's own body and the floor, nothing else. The
    /// limits are the IK's (the collision model's joint ranges, docs/adr/0020) less
    /// <c>--margin-deg</c>, and every pose is checked before anything moves (<see cref="DrivePlan"/>).
    /// RobotHost also refuses any command whose pose would collide.</para>
    ///
    /// <para><b>It stops the arm</b>, by commanding the last measured pose, on Enter or Ctrl+C, on a stall,
    /// or when the joint-state stream has been missing or below 30 Hz for 3 s of any 10 s
    /// (<see cref="DriveWatchdog"/>): a lost robot, or a second subscriber such as the headset app. Less
    /// than that only pauses the plan: Wi-Fi stalls happen, and the robot holds its last target
    /// through them. Nothing is sent before every arm joint has a
    /// sensed reading. The first move is the approach from the measured pose to the model's home at
    /// 0.5 rad/s, and the session ends back there. Enter is the stop to rely on: under WSL the Windows
    /// <c>dotnet</c> does not receive a signal sent to it (checked 2026-10-22: a SIGINT left it running),
    /// while its standard input arrives through the pipe like any other.</para>
    ///
    /// <para><c>--stop-file</c> names a file whose appearance stops it as Enter does, for a run with no
    /// terminal (<c>robot/pilot/onboard.py</c> on the robot, stopped by <c>just pilot-stop</c>); it
    /// refuses to start while that file exists.</para>
    ///
    /// <para><c>--dry-run</c> plans from home, prints the envelope and what the session would do, and
    /// sends nothing; with <c>--operator-log</c> it writes the commands it would send, stamped on a
    /// simulated clock, to check the format.</para>
    /// </summary>
    public static class JointDriveCommand
    {
        /// <summary>The headset slider's top (TeleopXR's SpeedSlider).</summary>
        public const float MaxSpeedRadPerSec = 6f;

        private const int MaxDatagramBytes = 128;

        private const string Usage =
            "usage: joint-drive --script waypoints|steps --speed <rad/s> [--seconds 300] [--seed 1] [--margin-deg 15] [--rate-hz 48]\n" +
            "                   (--dry-run | --remote-host <ip> [--remote-port 6002] --confirm-hardware-motion)\n" +
            "                   [--operator-log <path> [--session-id <id>]] [--stop-file <path>]";

        public static int Run(string[] args)
        {
            DriveScript? script = null;
            float speed = 0f;
            double seconds = 300, marginDeg = 15, rateHz = 48;
            ulong seed = 1;
            string? host = null, logPath = null, sessionId = null, stopFile = null;
            int port = 6002;
            bool dryRun = false, confirmed = false;
            try
            {
                for (int i = 1; i < args.Length; i++)
                {
                    string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                    double Number() => double.Parse(Next(), NumberStyles.Float, CultureInfo.InvariantCulture);
                    switch (args[i])
                    {
                        case "--script":
                            string name = Next();
                            script = name == "waypoints" ? DriveScript.Waypoints : name == "steps" ? DriveScript.Steps
                                : throw new ArgumentException($"unknown script '{name}' (waypoints or steps)");
                            break;
                        case "--speed": speed = (float)Number(); break;
                        case "--seconds": seconds = Number(); break;
                        case "--seed": seed = ulong.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--margin-deg": marginDeg = Number(); break;
                        case "--rate-hz": rateHz = Number(); break;
                        case "--remote-host": host = Next(); break;
                        case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--operator-log": logPath = Next(); break;
                        case "--session-id": sessionId = Next(); break;
                        case "--stop-file": stopFile = Next(); break;
                        case "--dry-run": dryRun = true; break;
                        case "--confirm-hardware-motion": confirmed = true; break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }

                if (script == null) throw new ArgumentException("--script is required");
                if (!(speed > 0f) || speed > MaxSpeedRadPerSec) throw new ArgumentException($"--speed must be in (0, {MaxSpeedRadPerSec}] rad/s, the headset slider's range");
                if (!(seconds > 0)) throw new ArgumentException("--seconds must be positive");
                if (!(marginDeg >= 5)) throw new ArgumentException("--margin-deg must be at least 5");
                if (!(rateHz >= 10 && rateHz <= 100)) throw new ArgumentException("--rate-hz must be in [10, 100]");
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"joint-drive: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var envelope = DriveEnvelope.FromModel(model, (float)(marginDeg * Math.PI / 180));
            PrintEnvelope(envelope, marginDeg);

            if (dryRun)
            {
                return DryRun(script.Value, profile, envelope, speed, seconds, seed, rateHz, logPath, sessionId ?? "joint-drive-dry-run");
            }

            if (!confirmed)
            {
                Console.Error.WriteLine("joint-drive: this moves the real arm. Pass --confirm-hardware-motion, with someone watching it.");
                return 64;
            }

            if (host == null || !IPAddress.TryParse(host, out IPAddress? address))
            {
                Console.Error.WriteLine("joint-drive: --remote-host <ip> is required");
                return 64;
            }

            if (stopFile != null && File.Exists(stopFile))
            {
                Console.Error.WriteLine($"joint-drive: the stop file {stopFile} exists; nothing was sent. Remove it to run.");
                return 130;
            }

            return Drive(script.Value, profile, envelope, speed, seconds, seed, rateHz, new IPEndPoint(address, port), logPath, sessionId ?? "joint-drive", stopFile);
        }

        private static void PrintEnvelope(in DriveEnvelope envelope, double marginDeg)
        {
            Console.WriteLine($"[joint-drive] envelope: the IK's joint ranges (collision model, docs/adr/0020) less {marginDeg:0.#}° at each end:");
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                AngleRange r = envelope[j];
                Console.WriteLine(FormattableString.Invariant($"  {DriveEnvelope.JointName(j),-9} {r.Min * 180 / Math.PI,7:0.0}° .. {r.Max * 180 / Math.PI,6:0.0}°"));
            }
        }

        private static void PrintPlan(DrivePlan plan, double rateHz)
        {
            int steps = 0, script = 0;
            double maxSmoothSpeed = 0;
            var min = new double[DriveEnvelope.JointCount];
            var max = new double[DriveEnvelope.JointCount];
            Array.Fill(min, double.MaxValue);
            Array.Fill(max, double.MinValue);
            foreach (DriveLeg leg in plan.Legs)
            {
                if (leg.Phase != LegPhase.Script) continue;
                script++;
                if (leg.Duration <= 0) steps++;
                for (int j = 0; j < DriveEnvelope.JointCount; j++)
                {
                    min[j] = Math.Min(min[j], DrivePlan.Joint(leg.To, j));
                    max[j] = Math.Max(max[j], DrivePlan.Joint(leg.To, j));
                }
            }

            double dt = 1 / rateHz;
            ArmPose previous = plan.Sample(0);
            for (double t = dt; t <= plan.TotalSeconds; t += dt)
            {
                ArmPose p = plan.Sample(t);
                if (plan.LegAt(t).Duration > 0) maxSmoothSpeed = Math.Max(maxSmoothSpeed, previous.MaxJointDelta(p) / dt);
                previous = p;
            }

            Console.WriteLine(
                FormattableString.Invariant($"[joint-drive] plan: {plan.Script.ToString().ToLowerInvariant()} at {plan.SpeedRadPerSec:0.##} rad/s, {script} moves ({steps} steps), ") +
                FormattableString.Invariant($"{plan.TotalSeconds:0} s with the approach and return; fastest smooth motion commanded {maxSmoothSpeed:0.00} rad/s"));
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                if (min[j] > max[j]) continue;
                Console.WriteLine(FormattableString.Invariant($"  {DriveEnvelope.JointName(j),-9} targets from {min[j] * 180 / Math.PI,7:0.0}° to {max[j] * 180 / Math.PI,6:0.0}°"));
            }
        }

        private static int DryRun(DriveScript script, RobotArmProfile profile, DriveEnvelope envelope, float speed, double seconds, ulong seed,
            double rateHz, string? logPath, string sessionId)
        {
            if (!DrivePlan.TryBuild(script, profile, profile.Collision!.Home, speed, seconds, seed, envelope, out DrivePlan? plan, out string? error))
            {
                Console.Error.WriteLine($"joint-drive: {error}");
                return 1;
            }

            PrintPlan(plan!, rateHz);
            for (int i = 0; i < Math.Min(8, plan!.Legs.Count); i++)
            {
                DriveLeg leg = plan.Legs[i];
                Console.WriteLine(FormattableString.Invariant($"  {leg.Start,6:0.0} s  {leg.Phase,-8} {(leg.Duration > 0 ? $"{leg.Duration:0.00} s to" : "step to"),-10} {DrivePlan.Describe(leg.To)}"));
            }

            if (logPath != null)
            {
                const long tps = 1_000_000_000;
                using var log = new OperatorLog(File.Create(logPath), tps, sessionId);
                Span<JointTarget> targets = stackalloc JointTarget[profile.JointCount];
                uint sequence = 0;
                for (double t = 0; t <= plan.TotalSeconds; t += 1 / rateHz)
                {
                    long ticks = (long)(t * tps);
                    int count = Targets(profile, plan.Sample(t), 0f, speed, targets);
                    log.Command(ticks, sequence++, ticks, targets.Slice(0, count));
                }

                Console.WriteLine($"[joint-drive] wrote {sequence} commands, on a simulated clock, to {logPath}");
            }

            Console.WriteLine("[joint-drive] dry run: nothing was sent");
            return 0;
        }

        private static int Drive(DriveScript script, RobotArmProfile profile, DriveEnvelope envelope, float speed, double seconds, ulong seed,
            double rateHz, IPEndPoint robot, string? logPath, string sessionId, string? stopFile)
        {
            long tps = Stopwatch.Frequency;
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            var link = new Link(socket, robot, profile);

            using var stop = new ManualResetEventSlim(false);
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                stop.Set();
            };
            // Any line on standard input stops it too; a closed input (no terminal) does not.
            var input = new Thread(() =>
            {
                if (Console.In.ReadLine() != null) stop.Set();
            }) { IsBackground = true };
            input.Start();
            // And so does the stop file appearing: the stop for a run with no terminal (just pilot-stop).
            if (stopFile != null)
            {
                new Thread(() =>
                {
                    while (!stop.IsSet)
                    {
                        if (File.Exists(stopFile)) stop.Set();
                        Thread.Sleep(100);
                    }
                }) { IsBackground = true }.Start();
            }

            // Nothing moves before every arm joint has been read from its servo.
            Console.WriteLine($"[joint-drive] subscribing to {robot}'s joint-state stream; waiting for a sensed reading of every arm joint");
            long waitUntil = Stopwatch.GetTimestamp() + 5 * tps;
            while (!link.HasFullReading)
            {
                if (stop.IsSet)
                {
                    Console.Error.WriteLine("joint-drive: stopped by hand before anything was sent.");
                    return 130;
                }

                if (Stopwatch.GetTimestamp() > waitUntil)
                {
                    Console.Error.WriteLine("joint-drive: no frame with every arm joint sensed in 5 s (is RobotHost running with --joint-local-port, " +
                        "and is the board answering? just joint-probe shows what the stream carries). Nothing was sent.");
                    return 1;
                }

                link.Pump(Stopwatch.GetTimestamp(), default);
            }

            ArmPose start = link.Measured;
            Console.WriteLine($"[joint-drive] the arm is at {DrivePlan.Describe(start)}");
            if (!DrivePlan.TryBuild(script, profile, start, speed, seconds, seed, envelope, out DrivePlan? plan, out string? error))
            {
                Console.Error.WriteLine($"joint-drive: {error}");
                return 1;
            }

            PrintPlan(plan!, rateHz);
            Console.WriteLine("[joint-drive] the arm moves in 3 s. Press Enter to stop it where it is.");
            for (long until = Stopwatch.GetTimestamp() + 3 * tps; Stopwatch.GetTimestamp() < until;)
            {
                if (stop.IsSet) return 130;
                link.Pump(Stopwatch.GetTimestamp(), default);
                Thread.Sleep(5);
            }

            using OperatorLog? log = logPath != null ? new OperatorLog(File.Create(logPath), tps, sessionId) : null;
            var watchdog = new DriveWatchdog();
            var targets = new JointTarget[profile.JointCount];
            var datagram = new byte[MaxDatagramBytes];
            long interval = (long)(tps / rateHz);
            long t0 = Stopwatch.GetTimestamp();
            long nextSend = t0, nextReport = t0 + 10 * tps;
            uint sequence = 0;
            ArmPose commanded = plan!.Sample(0);
            double maxError = 0;
            DriveFault fault = DriveFault.None;
            float gripper = link.Gripper;

            void Send(long now, in ArmPose pose)
            {
                int count = Targets(profile, pose, gripper, speed, targets);
                if (JointCommandCodec.TryEncode(sequence, now, targets.AsSpan(0, count), datagram, out int written))
                {
                    socket.SendTo(datagram, 0, written, SocketFlags.None, robot);
                    log?.Command(now, sequence, now, targets.AsSpan(0, count));
                    sequence++;
                }
            }

            // The plan's own clock: it runs only while the stream is healthy, so a Wi-Fi stall holds the
            // commanded pose where it is and the plan resumes from there, without a jump.
            double planTime = 0;
            long lastLoop = t0;
            bool paused = false;
            int pauses = 0;
            double pausedSeconds = 0;
            while (true)
            {
                long now = Stopwatch.GetTimestamp();
                double dt = (now - lastLoop) / (double)tps;
                lastLoop = now;
                link.Pump(now, watchdog, commanded);
                maxError = Math.Max(maxError, link.Measured.MaxJointDelta(commanded));
                bool healthy = watchdog.IsHealthy(now / (double)tps);
                fault = watchdog.Check(now / (double)tps);
                if (fault != DriveFault.None || stop.IsSet || planTime > plan.TotalSeconds + 0.5) break;

                if (healthy) planTime += dt;
                else pausedSeconds += dt;
                if (healthy == paused)
                {
                    paused = !healthy;
                    if (paused) pauses++;
                    Console.WriteLine(paused
                        ? FormattableString.Invariant($"[joint-drive] {planTime,5:0} s: joint-state frames are not arriving (Wi-Fi?); holding until they do (it stops after {DriveWatchdog.MaxDegradedSeconds:0} s of this in {DriveWatchdog.DegradedWindowSeconds:0} s)")
                        : FormattableString.Invariant($"[joint-drive] {planTime,5:0} s: frames are back; going on"));
                }

                if (now >= nextSend)
                {
                    commanded = plan.Sample(planTime);
                    Send(now, commanded);
                    nextSend += interval;
                    if (nextSend < now) nextSend = now + interval;
                }

                if (now >= nextReport)
                {
                    Console.WriteLine(FormattableString.Invariant(
                        $"[joint-drive] {planTime,5:0} / {plan.TotalSeconds:0} s, {link.Frames} frames, largest gap commanded-measured so far {maxError * 180 / Math.PI:0} deg"));
                    nextReport += 10 * tps;
                }

                Thread.Sleep(1);
            }

            if (fault != DriveFault.None || stop.IsSet)
            {
                // Hold where the arm is, not where it was told to go: a stalled joint stops pushing.
                ArmPose hold = link.Measured;
                for (int i = 0; i < 15; i++)
                {
                    Send(Stopwatch.GetTimestamp(), hold);
                    Thread.Sleep(20);
                }
            }

            log?.Close();
            Console.WriteLine($"[joint-drive] sent {sequence} commands, received {link.Frames} frames" +
                (pauses > 0 ? FormattableString.Invariant($", paused {pauses} time(s) for {pausedSeconds:0.0} s in all") : "") +
                (logPath != null ? $"; commands logged to {logPath}" : ""));
            switch (fault)
            {
                case DriveFault.Stall:
                    Console.Error.WriteLine($"joint-drive: STOPPED: the {DriveEnvelope.JointName(watchdog.StalledJoint)} joint stalled " +
                        $"(over {DriveWatchdog.StallErrorRadians} rad from its command and not moving for {DriveWatchdog.StallSeconds} s); " +
                        "it now holds where it is. Check what it touched before going on.");
                    return 2;
                case DriveFault.FrameGap:
                    Console.Error.WriteLine($"joint-drive: STOPPED: no joint-state frame for {DriveWatchdog.MaxDegradedSeconds} s (the robot, RobotHost or the network is gone); " +
                        "sent a hold to the last measured pose.");
                    return 3;
                case DriveFault.StreamDegraded:
                    Console.Error.WriteLine($"joint-drive: STOPPED: the joint-state stream was missing or below {DriveWatchdog.MinFrameRateHz} Hz for {DriveWatchdog.MaxDegradedSeconds} s " +
                        $"of the last {DriveWatchdog.DegradedWindowSeconds} s. " +
                        "Either the Wi-Fi is losing frames, or something else (the headset app, joint-probe) is on the joint port: " +
                        "two SS lines in the session's robothost.jlog mean the second. Sent a hold to the last measured pose.");
                    return 3;
            }

            if (stop.IsSet)
            {
                Console.Error.WriteLine("joint-drive: stopped by hand; the arm holds where it is.");
                return 130;
            }

            Console.WriteLine("[joint-drive] done: the arm is back at home.");
            return 0;
        }

        /// <summary>One command's targets, in the profile's order, every joint carrying the requested speed as the headset's do.</summary>
        internal static int Targets(in RobotArmProfile profile, in ArmPose pose, float gripperFraction, float speed, Span<JointTarget> output)
        {
            int count = 0;
            foreach (JointHardwareSpec joint in profile.Joints)
            {
                float angle = joint.Role switch
                {
                    JointRole.BaseYaw => pose.BaseYaw,
                    JointRole.Proximal => pose.Proximal,
                    JointRole.Distal => pose.Distal,
                    JointRole.Wrist => pose.Wrist,
                    JointRole.GripperRotate => pose.Roll,
                    JointRole.GripperMain => gripperFraction,
                    _ => 0f,
                };
                output[count++] = new JointTarget(joint.MotorId, angle, speed);
            }

            return count;
        }

        /// <summary>The socket's receive side: keepalives out, frames in, the latest measured pose.</summary>
        private sealed class Link
        {
            private readonly Socket _socket;
            private readonly EndPoint _robot;
            private readonly int[] _jointOfMotor = new int[256];
            private readonly byte _gripperMotor;
            private readonly byte[] _buffer = new byte[MaxDatagramBytes * 2];
            private readonly JointSample[] _joints = new JointSample[JointStateCodec.MaxJointsPerMessage];
            private readonly float[] _measured = new float[DriveEnvelope.JointCount];
            private readonly bool[] _sensed = new bool[DriveEnvelope.JointCount];
            private long _nextKeepalive;
            private long _lastSensedTicks;

            public Link(Socket socket, EndPoint robot, in RobotArmProfile profile)
            {
                _socket = socket;
                _socket.Blocking = false;
                _robot = robot;
                Array.Fill(_jointOfMotor, -1);
                foreach (JointHardwareSpec joint in profile.Joints)
                {
                    switch (joint.Role)
                    {
                        case JointRole.BaseYaw: _jointOfMotor[joint.MotorId] = 0; break;
                        case JointRole.Proximal: _jointOfMotor[joint.MotorId] = 1; break;
                        case JointRole.Distal: _jointOfMotor[joint.MotorId] = 2; break;
                        case JointRole.Wrist when joint.WristIndex == 0: _jointOfMotor[joint.MotorId] = 3; break;
                        case JointRole.GripperRotate: _jointOfMotor[joint.MotorId] = 4; break;
                        case JointRole.GripperMain: _gripperMotor = joint.MotorId; break;
                    }
                }
            }

            public bool HasFullReading => Array.TrueForAll(_sensed, s => s);

            public ArmPose Measured => new ArmPose(_measured[0], _measured[1], _measured[2], _measured[3], _measured[4]);

            /// <summary>The gripper as the stream reports it (the commanded fraction: it is never read back), held unchanged.</summary>
            public float Gripper { get; private set; }

            public long Frames { get; private set; }

            /// <summary>Sends the keepalive when due and takes in every frame waiting; with a watchdog, feeds it each frame and fresh reading.</summary>
            public void Pump(long now, DriveWatchdog? watchdog, in ArmPose commanded = default)
            {
                if (now >= _nextKeepalive)
                {
                    JointStateCodec.TryEncodeSubscribe(now, Stopwatch.Frequency, _buffer, out int n);
                    _socket.SendTo(_buffer, 0, n, SocketFlags.None, _robot);
                    _nextKeepalive = now + Stopwatch.Frequency / 2;
                }

                while (_socket.Available > 0)
                {
                    int received;
                    try
                    {
                        received = _socket.Receive(_buffer);
                    }
                    catch (SocketException)
                    {
                        break;
                    }

                    if (!JointStateCodec.TryDecode(_buffer.AsSpan(0, received), out _, out _, out long sensedTicks, out _, out _,
                            out long robotTps, _joints, out int count))
                    {
                        continue;
                    }

                    Frames++;
                    watchdog?.OnFrame(Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency);
                    bool fresh = sensedTicks != 0 && sensedTicks != _lastSensedTicks;
                    for (int i = 0; i < count; i++)
                    {
                        JointSample s = _joints[i];
                        if (s.MotorId == _gripperMotor) Gripper = s.Value;
                        int j = _jointOfMotor[s.MotorId];
                        if (j < 0 || !s.Sensed) continue;
                        _measured[j] = s.Value;
                        _sensed[j] = true;
                    }

                    if (fresh)
                    {
                        _lastSensedTicks = sensedTicks;
                        if (HasFullReading) watchdog?.OnReading(sensedTicks / (double)robotTps, Measured, commanded);
                    }
                }
            }
        }
    }
}
