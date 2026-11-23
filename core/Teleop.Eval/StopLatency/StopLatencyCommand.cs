using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text.Json;
using System.Threading;
using Teleop.Eval.JointDrive;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.StopLatency
{
    /// <summary>
    /// <c>stop-latency</c> (docs/adr/0038 workstream R, item 32): how long and how far the JetRover's arm keeps moving after
    /// its command stops mid-move, per joint and at the tool point, at a few speeds, on the real robot or the Isaac twin.
    ///
    /// <para>It talks to RobotHost's joint port as the headset does (joint commands at 48 Hz, every joint carrying the slider's
    /// speed, the joint-state stream back). It switches torque on (<c>arm-power</c> hold), moves the arm to the collision
    /// model's home the way <c>joint-drive</c> does (<see cref="DrivePlan.TryApproach"/>, at 0.5 rad/s), and then runs
    /// <see cref="StopLatencyPlan"/>: each trial swings one joint from home at a constant speed and stops its command dead,
    /// holding it where it stopped, as the headset's command does when the clutch is let go. Every commanded pose is inside
    /// the IK's ranges less <c>--margin-deg</c> and free by the collision model, checked before anything moves. It stops the
    /// arm and exits non-zero on Enter, a stall or a lost stream, as <c>joint-drive</c> does (<see cref="DriveWatchdog"/>).</para>
    ///
    /// <para>Each trial's stop is measured live from the stream (<see cref="StopMeter"/>, on this machine's clock, so it
    /// includes a reading's way back), printed, and written with <c>--json</c>. Record the run as a session and
    /// <c>scripts/gpu/arm_response.py --only stops</c> measures the same stops on the robot's clock (its relay targets and
    /// readings). <c>--dry-run</c> prints the trials and sends nothing.</para>
    /// </summary>
    public static class StopLatencyCommand
    {
        private const string Usage =
            "usage: stop-latency (--dry-run | --remote-host <ip> [--remote-port 6002] --confirm-hardware-motion [--simulated-robot])\n" +
            "                    [--joints yaw,proximal,distal,wrist] [--speeds 0.5,1,2] [--repeats 2] [--run-up 0.35] [--hold 1.5]\n" +
            "                    [--slider 3.8] [--margin-deg 15] [--json <path>]";

        private const double RateHz = 48;
        private static readonly string[] JointNames = { "yaw", "proximal", "distal", "wrist" };

        public static int Run(string[] args)
        {
            string? host = null, jsonPath = null;
            int port = 6002, repeats = 2;
            bool dryRun = false, confirmed = false, simulated = false;
            double runUp = 0.35, hold = 1.5, marginDeg = 15;
            float slider = 3.8f;
            var joints = new List<int> { 0, 1, 2, 3 };
            var speeds = new List<float> { 0.5f, 1f, 2f };
            try
            {
                for (int i = 1; i < args.Length; i++)
                {
                    string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                    double Number() => double.Parse(Next(), NumberStyles.Float, CultureInfo.InvariantCulture);
                    switch (args[i])
                    {
                        case "--remote-host": host = Next(); break;
                        case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--confirm-hardware-motion": confirmed = true; break;
                        case "--simulated-robot": simulated = true; break;
                        case "--dry-run": dryRun = true; break;
                        case "--joints": joints = ParseJoints(Next()); break;
                        case "--speeds": speeds = Next().Split(',').Select(s => float.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture)).ToList(); break;
                        case "--repeats": repeats = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--run-up": runUp = Number(); break;
                        case "--hold": hold = Number(); break;
                        case "--slider": slider = (float)Number(); break;
                        case "--margin-deg": marginDeg = Number(); break;
                        case "--json": jsonPath = Next(); break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }

                if (!dryRun && host == null) throw new ArgumentException("--remote-host is required (or --dry-run)");
                if (!dryRun && !confirmed) throw new ArgumentException("this moves the arm: pass --confirm-hardware-motion once its workspace is clear");
                if (!(slider > 0f) || slider > JointDriveCommand.MaxSpeedRadPerSec) throw new ArgumentException($"--slider must be in (0, {JointDriveCommand.MaxSpeedRadPerSec}] rad/s");
                if (speeds.Any(s => s > slider)) throw new ArgumentException("every --speeds entry must be at most --slider: the servo is told to move no faster than the slider");
                if (!(marginDeg >= 5)) throw new ArgumentException("--margin-deg must be at least 5");
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"stop-latency: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var envelope = DriveEnvelope.FromModel(model, (float)(marginDeg * Math.PI / 180));
            if (!StopLatencyPlan.TryBuild(profile, model, envelope, model.Home, joints, speeds, repeats, runUp, hold, out StopLatencyPlan? plan, out string? error))
            {
                Console.Error.WriteLine($"stop-latency: {error}");
                return 64;
            }

            Describe(profile, plan!, slider);
            if (dryRun) return 0;

            IPAddress address;
            try
            {
                address = Dns.GetHostAddresses(host!).First(a => a.AddressFamily == AddressFamily.InterNetwork);
            }
            catch (Exception e) when (e is SocketException || e is InvalidOperationException || e is ArgumentException)
            {
                Console.Error.WriteLine($"stop-latency: cannot resolve {host}: {e.Message}");
                return 64;
            }

            var robot = new IPEndPoint(address, port);
            if (ArmPower.ArmPowerCommand.Request(robot, ArmPowerRequest.Hold, 10, out bool answered) != 0)
            {
                if (answered)
                {
                    Console.Error.WriteLine("stop-latency: RobotHost did not switch torque on (above); nothing was sent");
                    return 1;
                }

                Console.WriteLine("[stop-latency] RobotHost reports no arm power: going on without asking for torque");
            }

            return Drive(profile, model, robot, plan!, slider, simulated, jsonPath);
        }

        private static List<int> ParseJoints(string text)
        {
            var out_ = new List<int>();
            foreach (string name in text.Split(','))
            {
                int j = Array.IndexOf(JointNames, name.Trim().ToLowerInvariant());
                if (j < 0) throw new ArgumentException($"unknown joint '{name}' (yaw, proximal, distal, wrist)");
                if (!out_.Contains(j)) out_.Add(j);
            }

            return out_;
        }

        private static void Describe(in RobotArmProfile profile, StopLatencyPlan plan, float slider)
        {
            Console.WriteLine(FormattableString.Invariant(
                $"[stop-latency] {plan.Trials.Count} trials from home ({DrivePlan.Describe(plan.StartPose)}), {plan.TotalSeconds:0} s; every command carries the slider's {slider:0.##} rad/s"));
            foreach (StopTrialPlan t in plan.Trials)
            {
                double sweep = (StopMeter.Tool(profile, t.Held) - StopMeter.Tool(profile, plan.StartPose)).Length() * 100.0;
                Console.WriteLine(FormattableString.Invariant(
                    $"  trial {t.Index + 1,2}: {DriveEnvelope.JointName(t.Joint),-9} {(t.Direction > 0 ? "+" : "-")}{t.ExcursionRadians * 180 / Math.PI:0} degrees at {t.SpeedRadPerSec:0.##} rad/s, stops at {t.StopAt:0.0} s; the tool point moves {sweep:0.0} cm"));
            }

            foreach (string s in plan.Skipped) Console.WriteLine($"  left out: {s}");
        }

        private static int Drive(RobotArmProfile profile, ArmCollisionModel model, IPEndPoint robot, StopLatencyPlan plan, float slider, bool simulated, string? jsonPath)
        {
            long tps = Stopwatch.Frequency;
            double Now() => Stopwatch.GetTimestamp() / (double)tps;
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            var link = new Link(socket, robot, profile);
            var meter = new StopMeter();
            using var stop = new ManualResetEventSlim(false);
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                stop.Set();
            };
            new Thread(() =>
            {
                if (Console.In.ReadLine() != null) stop.Set();
            }) { IsBackground = true }.Start();

            double waitUntil = Now() + 5;
            while (!link.HasFullReading)
            {
                if (stop.IsSet) return 130;
                if (Now() > waitUntil)
                {
                    Console.Error.WriteLine("stop-latency: no frame with every arm joint sensed in 5 s (is RobotHost running, and the board answering? just joint-probe). Nothing was sent.");
                    return 1;
                }

                link.Pump(null);
                Thread.Sleep(2);
            }

            // To home first, as joint-drive goes there: straight when clear, else a joint at a time; a resting arm is freed first.
            if (!DrivePlan.TryApproach(profile, model, link.Measured, model.Home, out List<(ArmPose To, bool Step)> via, out string? why))
            {
                Console.Error.WriteLine($"stop-latency: {why}");
                return 1;
            }

            double countdown = JointDriveCommand.CountdownSeconds(simulated);
            Console.WriteLine(countdown > 0
                ? FormattableString.Invariant($"[stop-latency] the arm moves in {countdown:0} s, first to home. Press Enter to stop it where it is.")
                : "[stop-latency] simulated robot: no countdown, the arm moves now. Press Enter to stop it where it is.");
            for (double until = Now() + countdown; Now() < until;)
            {
                if (stop.IsSet) return 130;
                link.Pump(null);
                Thread.Sleep(5);
            }

            var watchdog = new DriveWatchdog(simulated ? 3.0 : DriveWatchdog.StallSeconds);
            var targets = new JointTarget[profile.JointCount];
            var datagram = new byte[JointDatagram.MaxBytes];
            uint sequence = 0;
            float gripper = link.Gripper;
            ArmPose commanded = link.Measured;

            void Send(in ArmPose pose)
            {
                int count = JointDriveCommand.Targets(profile, pose, gripper, slider, targets);
                if (JointCommandCodec.TryEncode(sequence, Stopwatch.GetTimestamp(), targets.AsSpan(0, count), datagram, out int written))
                {
                    socket.SendTo(datagram, 0, written, SocketFlags.None, robot);
                    sequence++;
                }
            }

            // Runs a timeline at 48 Hz until it ends; false on a stop, a stall or a lost stream (the arm then holds).
            DriveFault fault = DriveFault.None;
            bool RunTimeline(double seconds, Func<double, ArmPose> pose, Action<double, double>? onTick)
            {
                double start = Now(), next = start;
                while (true)
                {
                    double now = Now();
                    link.Pump(watchdog, commanded, meter);
                    fault = watchdog.Check(now);
                    if (fault != DriveFault.None || stop.IsSet) return false;
                    double t = now - start;
                    if (t > seconds) return true;
                    if (now >= next)
                    {
                        commanded = pose(t);
                        Send(commanded);
                        onTick?.Invoke(t, now);
                        next += 1.0 / RateHz;
                        if (next < now) next = now + 1.0 / RateHz;
                    }

                    Thread.Sleep(1);
                }
            }

            bool ok = true;
            ArmPose from = link.Measured;
            foreach ((ArmPose to, bool step) in via)
            {
                ArmPose a = from, b = to;
                double d = step ? 0.5 : DrivePlan.SmoothDuration(a.MaxJointDelta(b), DrivePlan.ApproachSpeedRadPerSec) + 0.3;
                ok = RunTimeline(d, t => step ? b : ArmPose.Lerp(a, b, (float)HandModel.MinimumJerk(t / Math.Max(1e-6, d - 0.3))), null);
                if (!ok) break;
                from = to;
            }

            var stopAt = new double?[plan.Trials.Count];
            var results = new List<StopResult>();
            int measured = 0;
            if (ok)
            {
                Console.WriteLine("[stop-latency] at home: the trials begin");
                ok = RunTimeline(plan.TotalSeconds, t => plan.Sample(t, out _, out _), (t, now) =>
                {
                    plan.Sample(t, out StopPhase phase, out int trial);
                    if (trial < 0) return;
                    if (phase == StopPhase.Hold && stopAt[trial] == null) stopAt[trial] = now;
                    // A trial is measured once its hold has ended, from the readings this side took in it.
                    for (int i = measured; i < plan.Trials.Count; i++)
                    {
                        StopTrialPlan p = plan.Trials[i];
                        if (stopAt[i] == null || now < stopAt[i]!.Value + (p.HoldEnd - p.StopAt)) break;
                        if (meter.TryMeasure(profile, p, stopAt[i]!.Value, stopAt[i]!.Value + (p.HoldEnd - p.StopAt), out StopResult r))
                        {
                            results.Add(r);
                            Console.WriteLine(FormattableString.Invariant(
                                $"  trial {p.Index + 1,2}: {DriveEnvelope.JointName(p.Joint),-9} at {p.SpeedRadPerSec:0.##} rad/s: kept moving {r.StopMs:0} ms and {r.TravelDeg:0.0} degrees " +
                                $"(it trailed by {r.ErrorAtStopDeg:0.0} at the stop, went {r.PastTargetDeg:0.0} past it); tool point {r.ToolTravelCm:0.0} cm{(r.Settled ? "" : " (NOT settled by the hold's end)")}"));
                        }
                        else
                        {
                            Console.WriteLine($"  trial {p.Index + 1,2}: too few readings to measure");
                        }

                        measured = i + 1;
                    }
                });
            }

            // However it ended, the arm holds where it is.
            ArmPose here = link.Measured;
            for (int i = 0; i < 5; i++)
            {
                Send(here);
                Thread.Sleep(20);
            }

            WriteSummary(results, plan, jsonPath, simulated);
            if (fault != DriveFault.None)
            {
                Console.Error.WriteLine($"stop-latency: STOPPED ({fault}): the arm holds where it is. Check it before going on.");
                return fault == DriveFault.Stall ? 2 : 3;
            }

            if (stop.IsSet)
            {
                Console.Error.WriteLine("stop-latency: stopped by hand; the arm holds where it is.");
                return 130;
            }

            if (results.Count == 0)
            {
                Console.Error.WriteLine("stop-latency: no trial could be measured");
                return 1;
            }

            Console.WriteLine($"[stop-latency] done: {results.Count} of {plan.Trials.Count} trials measured; the arm is at home.");
            return 0;
        }

        private static void WriteSummary(List<StopResult> results, StopLatencyPlan plan, string? jsonPath, bool simulated)
        {
            static double Median(IEnumerable<double> v)
            {
                var s = v.OrderBy(x => x).ToList();
                return s.Count == 0 ? double.NaN : s.Count % 2 == 1 ? s[s.Count / 2] : 0.5 * (s[s.Count / 2 - 1] + s[s.Count / 2]);
            }

            var groups = results.GroupBy(r => (r.Joint, r.SpeedRadPerSec)).OrderBy(g => g.Key.Joint).ThenBy(g => g.Key.SpeedRadPerSec).ToList();
            if (groups.Count > 0) Console.WriteLine("[stop-latency] medians (this machine's clock: includes a reading's way back, about one 20 ms driver cycle):");
            foreach (var g in groups)
            {
                Console.WriteLine(FormattableString.Invariant(
                    $"  {DriveEnvelope.JointName(g.Key.Joint),-9} {g.Key.SpeedRadPerSec,4:0.##} rad/s: {Median(g.Select(r => r.StopMs)),4:0} ms, {Median(g.Select(r => r.TravelDeg)),5:0.0} degrees, " +
                    $"{Median(g.Select(r => r.PastTargetDeg)),4:0.0} past the command; tool point {Median(g.Select(r => r.ToolTravelCm)),5:0.0} cm ({g.Count()} trials)"));
            }

            if (jsonPath == null) return;
            var doc = new
            {
                format = "stop-latency",
                version = 1,
                source = simulated ? "isaac" : "robot",
                clock = "tool (receive times of fresh readings; the recorded session on the robot's clock is the reference)",
                settleRadians = StopMeter.SettleRadians,
                toolOffsetMeters = StopMeter.ToolOffsetMeters,
                runUpSeconds = plan.RunUpSeconds,
                trials = results.Select(r => new
                {
                    trial = r.Trial + 1,
                    joint = DriveEnvelope.JointName(r.Joint),
                    speedRadPerSec = r.SpeedRadPerSec,
                    samples = r.Samples,
                    stopMs = Math.Round(r.StopMs, 1),
                    travelDeg = Math.Round(r.TravelDeg, 2),
                    errorAtStopDeg = Math.Round(r.ErrorAtStopDeg, 2),
                    pastTargetDeg = Math.Round(r.PastTargetDeg, 2),
                    toolTravelCm = Math.Round(r.ToolTravelCm, 2),
                    toolErrorAtStopCm = Math.Round(r.ToolErrorAtStopCm, 2),
                    settled = r.Settled,
                }),
                skipped = plan.Skipped,
            };
            File.WriteAllText(jsonPath, JsonSerializer.Serialize(doc, new JsonSerializerOptions { WriteIndented = true }) + "\n");
            Console.WriteLine($"[stop-latency] written to {jsonPath}");
        }

        /// <summary>The joint port's receive side: keepalives out, joint-state frames in, the newest measured pose; each fresh
        /// reading goes to the watchdog and the meter.</summary>
        private sealed class Link
        {
            private readonly Socket _socket;
            private readonly EndPoint _robot;
            private readonly int[] _jointOfMotor = new int[256];
            private readonly byte _gripperMotor;
            private readonly byte[] _buffer = new byte[JointDatagram.MaxBytes * 2];
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
            public float Gripper { get; private set; }

            public void Pump(DriveWatchdog? watchdog, in ArmPose commanded = default, StopMeter? meter = null)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= _nextKeepalive)
                {
                    JointStateCodec.TryEncodeSubscribe(now, Stopwatch.Frequency, _buffer, out int n);
                    _socket.SendTo(_buffer, 0, n, SocketFlags.None, _robot);
                    _nextKeepalive = now + Stopwatch.Frequency / 2;
                }

                while (_socket.Poll(0, SelectMode.SelectRead))
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

                    double arrival = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    watchdog?.OnFrame(arrival);
                    for (int i = 0; i < count; i++)
                    {
                        JointSample s = _joints[i];
                        if (s.MotorId == _gripperMotor) Gripper = s.Value;
                        int j = _jointOfMotor[s.MotorId];
                        if (j < 0 || !s.Sensed) continue;
                        _measured[j] = s.Value;
                        _sensed[j] = true;
                    }

                    if (sensedTicks != 0 && sensedTicks != _lastSensedTicks && HasFullReading)
                    {
                        _lastSensedTicks = sensedTicks;
                        watchdog?.OnReading(sensedTicks / (double)robotTps, Measured, commanded);
                        meter?.Add(arrival, Measured);
                    }
                }
            }
        }
    }
}
