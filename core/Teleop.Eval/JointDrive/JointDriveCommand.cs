using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Teleop.Core.Types;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

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
    /// <para><c>--stall-seconds</c> (with <c>--simulated-robot</c>, which it needs above 1 s) lengthens the stall stop for the
    /// JetRover's Isaac twin on a deliberately bad uplink (docs/adr/0035 N4, <c>just isaac-jetrover-record-drive</c>): RobotHost
    /// refuses the commands the uplink made stale and the arm holds short of its command, which a 1 s window takes for a
    /// stall. <c>--stream-seconds</c> (likewise) lengthens the degraded-stream stop: the uplink delays the keepalives too,
    /// and RobotHost pauses the joint-state stream until one arrives. A real arm always keeps the 1 s stall stop and the
    /// 3 s degraded-stream stop.</para>
    ///
    /// <para><c>--take &lt;scenario&gt;</c> records the drive as a take of that scenario, as the headset's RECORD button
    /// does (docs/adr/0033, <see cref="TakeClient"/>), so it becomes a LeRobot episode (docs/adr/0036) with nobody at the
    /// headset. The take starts where the script does (docs/adr/0036 amendment 5): after the 3 s countdown (none with
    /// <c>--simulated-robot</c>, docs/adr/0037 amendment 7) and the slow move
    /// home when there is one, as the arm begins to move, so neither is in the recording. It is stopped and kept on every way
    /// out after that, a fault or Enter included. A scenario RobotHost does not record leaves the arm unmoved; should RobotHost
    /// then refuse the start, the arm holds where the move home left it.</para>
    ///
    /// <para>A safety take (<c>--scene</c>) plans 1-3 events (<c>--events</c>) at random points of the take
    /// (<see cref="SafetyTake"/>), sends each to RobotHost as the arm leaves free movement for its object, which records it
    /// (<c>SE</c>), and after a knock plans the next once the objects have settled, while the arm keeps moving.</para>
    ///
    /// <para><c>--dry-run</c> plans from home, prints the envelope and what the session would do, and
    /// sends nothing; with <c>--operator-log</c> it writes the commands it would send, stamped on a
    /// simulated clock, to check the format.</para>
    /// </summary>
    public static class JointDriveCommand
    {
        /// <summary>The headset slider's top (TeleopXR's SpeedSlider).</summary>
        public const float MaxSpeedRadPerSec = 6f;

        /// <summary>The joint port's budget (docs/adr/0026 decision 6): this tool only talks on the joint port.</summary>
        private const int MaxDatagramBytes = Teleop.RobotArm.Wire.JointDatagram.MaxBytes;

        private const string Usage =
            "usage: joint-drive --script waypoints|steps|course --speed <rad/s> [--seconds 300] [--seed 1] [--margin-deg 15] [--rate-hz 48]\n" +
            "                   (--dry-run | --remote-host <ip> [--remote-port 6002] --confirm-hardware-motion)\n" +
            "                   [--operator-log <path> [--session-id <id>]] [--stop-file <path>] [--take <scenario>]\n" +
            "                   [--simulated-robot [--stall-seconds 1] [--stream-seconds 3]] [--scene <file> [--new-layout] [--touch-depth-mm 8] [--events 1-3]]\n" +
            "                   [--script mixed [--replays 0] [--max-joint-speed <rad/s>[,x4]] [--max-joint-accel <rad/s2>[,x4]] [--max-wrist-speed <m/s>]\n" +
            "                                   [--max-peak-speed <m/s>] [--no-caps] [--no-style] [--offset-mm -40,60]] [--dry-run --layouts 1-200 [--rows]]";

        public static int Run(string[] args)
        {
            DriveScript? script = null;
            float speed = 0f;
            double seconds = 300, marginDeg = 15, rateHz = 48;
            ulong seed = 1;
            string? host = null, logPath = null, sessionId = null, stopFile = null, take = null;
            int port = 6002;
            bool dryRun = false, confirmed = false, simulated = false;
            double stallSeconds = DriveWatchdog.StallSeconds, streamSeconds = DriveWatchdog.MaxDegradedSeconds;
            string? scenePath = null;
            bool newLayout = false;
            var safetyOptions = new SafetyOptions();
            // docs/adr/0038: the mixed script's options, its replays, and the dry run over RobotHost's layouts.
            var mixedOptions = new MixedOptions();
            int replays = 0;
            int layoutsFrom = 0, layoutsTo = -1;
            bool rows = false;
            MotionCaps Caps() => mixedOptions.Caps ??= MixedOptions.DefaultCaps();
            void PerJoint(string text, float[] into, string option)
            {
                string[] v = text.Split(',');
                if (v.Length != 1 && v.Length != into.Length) throw new ArgumentException($"{option} is one number for every joint, or five (yaw, shoulder, elbow, wrist pitch, roll)");
                for (int j = 0; j < into.Length; j++) into[j] = float.Parse(v[v.Length == 1 ? 0 : j], NumberStyles.Float, CultureInfo.InvariantCulture);
            }
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
                            script = name switch
                            {
                                "waypoints" => DriveScript.Waypoints,
                                "steps" => DriveScript.Steps,
                                "course" => DriveScript.Course,
                                "reach_no_obstacle" => DriveScript.ReachNoObstacle,
                                "stop_before_collision" => DriveScript.StopBeforeCollision,
                                "collide_and_recover" => DriveScript.CollideAndRecover,
                                "collide_with_obstacle" => DriveScript.CollideWithObstacle,
                                "mixed" => DriveScript.Mixed,
                                _ => throw new ArgumentException($"unknown script '{name}' (waypoints, steps, course, reach_no_obstacle, " +
                                                                 "stop_before_collision, collide_and_recover, collide_with_obstacle or mixed)"),
                            };
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
                        case "--take": take = Next(); break;
                        case "--simulated-robot": simulated = true; break;
                        case "--stall-seconds": stallSeconds = Number(); break;
                        case "--stream-seconds": streamSeconds = Number(); break;
                        case "--scene": scenePath = Next(); break;
                        case "--new-layout": newLayout = true; break;
                        case "--touch-depth-mm": safetyOptions.TouchDepthMeters = (float)(Number() / 1000.0); break;
                        case "--events": ParseEvents(Next(), safetyOptions); break;
                        case "--replays": replays = int.Parse(Next(), NumberStyles.None, CultureInfo.InvariantCulture); break;
                        case "--layouts": ParseRange(Next(), "--layouts", out layoutsFrom, out layoutsTo); break;
                        case "--rows": rows = true; break;
                        case "--max-joint-speed": PerJoint(Next(), Caps().JointSpeed, "--max-joint-speed"); break;
                        case "--max-joint-accel": PerJoint(Next(), Caps().JointAcceleration, "--max-joint-accel"); break;
                        case "--no-style": mixedOptions.Style = null; break;
                        case "--max-wrist-speed": Caps().MaxWristSpeed = (float)Number(); break;
                        case "--max-peak-speed": Caps().MaxPeakSpeed = Number(); break;
                        case "--no-caps": mixedOptions.Caps = null; break;
                        case "--offset-mm":
                            string[] mm = Next().Split(',');
                            if (mm.Length != 2) throw new ArgumentException("--offset-mm is two numbers, like -40,60");
                            mixedOptions.OffsetMinMeters = (float)(double.Parse(mm[0], NumberStyles.Float, CultureInfo.InvariantCulture) / 1000.0);
                            mixedOptions.OffsetMaxMeters = (float)(double.Parse(mm[1], NumberStyles.Float, CultureInfo.InvariantCulture) / 1000.0);
                            break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }

                if (script == null) throw new ArgumentException("--script is required");
                bool safety = SafetyScript.IsSafety(script.Value);
                if (safety && scenePath == null) throw new ArgumentException($"--script {SafetyScript.Category(script.Value)} plans around the objects: it needs --scene <core/Scenes/*.json>");
                if (!safety && (scenePath != null || newLayout)) throw new ArgumentException("--scene and --new-layout are for the safety scripts");
                if (newLayout && dryRun) throw new ArgumentException("--new-layout asks RobotHost for a layout; a dry run sends nothing");
                if (!(safetyOptions.TouchDepthMeters > 0f && safetyOptions.TouchDepthMeters <= 0.06f)) throw new ArgumentException("--touch-depth-mm must be in (0, 60]");
                safetyOptions.Validate();
                mixedOptions.Validate();
                if (replays < 0 || replays > MaxReplays) throw new ArgumentException($"--replays must be in [0, {MaxReplays}]");
                if (replays > 0 && script != DriveScript.Mixed) throw new ArgumentException("--replays is for --script mixed (docs/adr/0038 decision 2)");
                if (replays > 0 && (take == null || dryRun)) throw new ArgumentException("--replays records more takes: it needs --take, and a dry run records none");
                if (layoutsTo >= layoutsFrom && (!dryRun || !safety)) throw new ArgumentException("--layouts plans a safety script on RobotHost's layouts of --scene: a dry run");
                if (replays > 0 && !simulated) throw new ArgumentException("--replays has RobotHost move the objects, which only the Isaac twin can: it needs --simulated-robot");
                if (!(speed > 0f) || speed > MaxSpeedRadPerSec) throw new ArgumentException($"--speed must be in (0, {MaxSpeedRadPerSec}] rad/s, the headset slider's range");
                if (!(seconds > 0)) throw new ArgumentException("--seconds must be positive");
                if (!(marginDeg >= 5)) throw new ArgumentException("--margin-deg must be at least 5");
                if (!(rateHz >= 10 && rateHz <= 100)) throw new ArgumentException("--rate-hz must be in [10, 100]");
                if (take != null && (take.Length == 0 || take.Length > RecordCodec.MaxNameLength)) throw new ArgumentException("--take names a scenario");
                if (take != null && dryRun) throw new ArgumentException("--take records a real drive; a dry run sends nothing");
                if (!(stallSeconds >= DriveWatchdog.StallSeconds && stallSeconds <= DriveWatchdog.MaxStallSeconds))
                    throw new ArgumentException($"--stall-seconds must be in [{DriveWatchdog.StallSeconds}, {DriveWatchdog.MaxStallSeconds}]");
                // The real robot keeps the 1 s stall stop: a longer one is for the Isaac twin, said out loud.
                if (stallSeconds > DriveWatchdog.StallSeconds && !simulated)
                    throw new ArgumentException($"--stall-seconds over {DriveWatchdog.StallSeconds} needs --simulated-robot: a real arm stops on a {DriveWatchdog.StallSeconds} s stall");
                if (!(streamSeconds >= DriveWatchdog.MaxDegradedSeconds && streamSeconds <= DriveWatchdog.MaxStreamSeconds))
                    throw new ArgumentException($"--stream-seconds must be in [{DriveWatchdog.MaxDegradedSeconds}, {DriveWatchdog.MaxStreamSeconds}]");
                if (streamSeconds > DriveWatchdog.MaxDegradedSeconds && !simulated)
                    throw new ArgumentException($"--stream-seconds over {DriveWatchdog.MaxDegradedSeconds} needs --simulated-robot: a real arm stops after {DriveWatchdog.MaxDegradedSeconds} s of a degraded stream");
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"joint-drive: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            // The safety scripts' scene (docs/adr/0036 amendment 1): the file RobotHost runs, checked against its world status.
            WorldScene? scene = null;
            byte[] sceneSha = Array.Empty<byte>();
            if (scenePath != null)
            {
                try
                {
                    scene = WorldView.LoadScene(scenePath, out sceneSha);
                    safetyOptions.KnockMaxKg = scene.KnockMaxMass; // the scene's roles (docs/adr/0037 P4)
                }
                catch (Exception e) when (e is IOException || e is UnauthorizedAccessException || e is System.Text.Json.JsonException || e is InvalidDataException || e is ArgumentException)
                {
                    Console.Error.WriteLine($"joint-drive: --scene {scenePath}: {e.Message}");
                    return 64;
                }
            }

            RobotArmProfile profile = RobotArmProfile.JetRoverMeasuredDefault;
            ArmCollisionModel model = profile.Collision!;
            var envelope = DriveEnvelope.FromModel(model, (float)(marginDeg * Math.PI / 180));
            PrintEnvelope(envelope, marginDeg);

            if (dryRun)
            {
                // A safety script's dry run plans around the scene's objects where the file puts them.
                List<SafetyObject>? nominal = scene == null ? null : scene.Objects.Select(o => WorldView.Bounds(o, 1.0, o.Nominal.X, o.Nominal.Y,
                    o.Nominal.Z, o.Nominal.Qw, o.Nominal.Qx, o.Nominal.Qy, o.Nominal.Qz, WorldView.ShapesOf(scene))).ToList();
                if (layoutsTo >= layoutsFrom)
                {
                    return MixedDryRun.Run(script.Value, scene!, profile, envelope, speed, seconds, seed, layoutsFrom, layoutsTo, safetyOptions, mixedOptions, rows);
                }

                return DryRun(script.Value, profile, envelope, speed, seconds, seed, rateHz, logPath, sessionId ?? "joint-drive-dry-run", nominal, safetyOptions, mixedOptions);
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

            // Torque on first (docs/adr/0023 decision 5): a relaxed arm ignores every command, and a parked
            // one would stay parked. The arm stiffens where it rests; nothing moves.
            var robotEndPoint = new IPEndPoint(address, port);
            if (ArmPower.ArmPowerCommand.Request(robotEndPoint, ArmPowerRequest.Hold, 10, out bool answered) != 0)
            {
                if (answered)
                {
                    Console.Error.WriteLine("joint-drive: RobotHost did not switch torque on (above); nothing was sent");
                    return 1;
                }

                Console.WriteLine("[joint-drive] RobotHost reports no arm power (older than park and relax): driving without asking for torque");
            }

            return Drive(script.Value, profile, envelope, speed, seconds, seed, rateHz, robotEndPoint, logPath, sessionId ?? "joint-drive", stopFile, take,
                stallSeconds, streamSeconds, scene, sceneSha, newLayout, safetyOptions, simulated, mixedOptions, replays);
        }

        /// <summary>
        /// The pause between the plan and the first move, in which Enter stops the arm before it moves: 3 s for a real arm, so
        /// whoever watches it can step back or stop it, none for the Isaac twin (<c>--simulated-robot</c>), where it only cost
        /// each safety take 3 s (docs/adr/0037 amendment 7). Not part of the take either way.
        /// </summary>
        internal static double CountdownSeconds(bool simulatedRobot) => simulatedRobot ? 0 : 3;

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
                FormattableString.Invariant($"[joint-drive] plan: {(SafetyScript.IsSafety(plan.Script) ? SafetyScript.Category(plan.Script) : plan.Script.ToString().ToLowerInvariant())} at {plan.SpeedRadPerSec:0.##} rad/s, {script} moves ({steps} steps), ") +
                FormattableString.Invariant($"{plan.TotalSeconds:0} s with the approach and return; fastest smooth motion commanded {maxSmoothSpeed:0.00} rad/s"));
            for (int j = 0; j < DriveEnvelope.JointCount; j++)
            {
                if (min[j] > max[j]) continue;
                Console.WriteLine(FormattableString.Invariant($"  {DriveEnvelope.JointName(j),-9} targets from {min[j] * 180 / Math.PI,7:0.0}° to {max[j] * 180 / Math.PI,6:0.0}°"));
            }

            if (plan.Moves.Count > 0)
            {
                // docs/adr/0038 decision 1: the mixed take's moves, each sent to RobotHost (MV) as it begins.
                Console.WriteLine(FormattableString.Invariant(
                    $"[joint-drive] {plan.Moves.Count} moves ({plan.Moves.Count(m => m.Kind == ScriptMoveKind.Aim)} aims, {plan.Moves.Count(m => m.Kind == ScriptMoveKind.Pass)} passes); the take runs {plan.TotalSeconds - plan.ScriptStart:0.0} s from {plan.ScriptStart:0.0} s; {plan.PlanningReport}"));
                foreach (PlannedMove m in plan.Moves.Where(x => x.ObjectId != 0)) Console.WriteLine("  " + m);
            }

            if (plan.EventsDrawn > 0)
            {
                // docs/adr/0036 amendment 5: events anywhere in the take, the take starting at ScriptStart.
                Console.WriteLine(FormattableString.Invariant(
                    $"[joint-drive] {plan.Events.Count} of {plan.EventsDrawn} event(s) drawn planned now{(plan.Pending != null ? ", the next after the knock's objects settle" : "")}; the take runs {plan.TotalSeconds - plan.ScriptStart:0.0} s from {plan.ScriptStart:0.0} s:"));
                foreach (SafetyEvent e in plan.Events) Console.WriteLine("  " + e);
            }
        }

        private static int DryRun(DriveScript script, RobotArmProfile profile, DriveEnvelope envelope, float speed, double seconds, ulong seed,
            double rateHz, string? logPath, string sessionId, IReadOnlyList<SafetyObject>? objects, SafetyOptions? safetyOptions = null, MixedOptions? mixedOptions = null)
        {
            if (!DrivePlan.TryBuild(script, profile, profile.Collision!.Home, speed, seconds, seed, envelope, out DrivePlan? plan, out string? error, objects, safetyOptions,
                    mixedOptions))
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
                    float gripper = plan.GripperAt(t); // NaN: the plan holds it as reported, and a dry run has no report
                    int count = Targets(profile, plan.Sample(t), float.IsNaN(gripper) ? 0f : gripper, plan.SpeedAt(t), targets);
                    log.Command(ticks, sequence++, ticks, targets.Slice(0, count));
                }

                Console.WriteLine($"[joint-drive] wrote {sequence} commands, on a simulated clock, to {logPath}");
            }

            Console.WriteLine("[joint-drive] dry run: nothing was sent");
            return 0;
        }

        private static int Drive(DriveScript script, RobotArmProfile profile, DriveEnvelope envelope, float speed, double seconds, ulong seed,
            double rateHz, IPEndPoint robot, string? logPath, string sessionId, string? stopFile, string? takeScenario, double stallSeconds, double streamSeconds,
            WorldScene? scene, byte[] sceneSha, bool newLayout, SafetyOptions safetyOptions, bool simulatedRobot, MixedOptions mixedOptions, int replays)
        {
            long tps = Stopwatch.Frequency;
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            var link = new Link(socket, robot, profile);
            if (scene != null) link.World = new WorldState(scene, sceneSha);
            TakeClient? take = takeScenario == null ? null
                : new TakeClient((bytes, n) => socket.SendTo(bytes, 0, n, SocketFlags.None, robot), TakeClient.FirstSequence());
            link.Take = take;

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

            // A safety script plans around the objects where RobotHost says they are (docs/adr/0036 amendment 1), after a new
            // layout when asked: every object posed, and at rest.
            List<SafetyObject>? objects = null;
            if (link.World != null && !PrepareWorld(link, socket, robot, newLayout, stop, out objects))
            {
                return 1; // nothing moved
            }

            ArmPose start = link.Measured;
            Console.WriteLine($"[joint-drive] the arm is at {DrivePlan.Describe(start)}");
            // A take with several events takes seconds to plan (docs/adr/0036 amendment 5): the link is kept up meanwhile, so
            // RobotHost goes on streaming the joints and the world.
            Task<(bool Built, DrivePlan? Plan, string? Error)> planning = Task.Run(() =>
            {
                bool built = DrivePlan.TryBuild(script, profile, start, speed, seconds, seed, envelope, out DrivePlan? p, out string? why, objects, safetyOptions, mixedOptions);
                return (built, p, why);
            });
            while (!planning.IsCompleted)
            {
                if (stop.IsSet)
                {
                    Console.Error.WriteLine("joint-drive: stopped by hand before anything moved.");
                    return 130;
                }

                link.Pump(Stopwatch.GetTimestamp(), default);
                Thread.Sleep(5);
            }

            (bool planned, DrivePlan? plan, string? error) = planning.Result;
            if (!planned)
            {
                Console.Error.WriteLine($"joint-drive: {error}");
                return 1;
            }

            PrintPlan(plan!, rateHz);
            // A take of a scenario RobotHost does not record leaves the arm unmoved: that is known before anything moves.
            int scenarioIndex = -1;
            if (take != null && !AwaitScenario(take, takeScenario!, link, stop, out scenarioIndex))
            {
                return 1; // nothing moved
            }

            // The countdown comes before the take (docs/adr/0036 amendment 5): it is a stop for the real arm, not part of the
            // recording, which starts where the script does (the plan's ScriptStart), as the arm begins to move. The Isaac twin
            // (--simulated-robot) has nobody beside it to step back, so it goes at once (docs/adr/0037 amendment 7).
            double countdown = CountdownSeconds(simulatedRobot);
            Console.WriteLine(countdown > 0
                ? FormattableString.Invariant($"[joint-drive] the arm moves in {countdown:0} s. Press Enter to stop it where it is.")
                : "[joint-drive] simulated robot: no countdown, the arm moves now. Press Enter to stop it where it is.");
            for (long until = Stopwatch.GetTimestamp() + (long)(countdown * tps); Stopwatch.GetTimestamp() < until;)
            {
                if (stop.IsSet)
                {
                    Console.Error.WriteLine("joint-drive: stopped by hand before anything moved.");
                    return 130;
                }

                link.Pump(Stopwatch.GetTimestamp(), default);
                Thread.Sleep(5);
            }

            using OperatorLog? log = logPath != null ? new OperatorLog(File.Create(logPath), tps, sessionId) : null;
            var watchdog = new DriveWatchdog(stallSeconds, streamSeconds);
            if (stallSeconds > DriveWatchdog.StallSeconds)
                Console.WriteLine(FormattableString.Invariant($"[joint-drive] simulated robot: a joint stalls after {stallSeconds:0.#} s far from its command, not {DriveWatchdog.StallSeconds:0.#} s"));
            if (streamSeconds > DriveWatchdog.MaxDegradedSeconds)
                Console.WriteLine(FormattableString.Invariant($"[joint-drive] simulated robot: it stops after {streamSeconds:0.#} s of a degraded stream in {watchdog.DegradedWindowLength:0.#} s, not {DriveWatchdog.MaxDegradedSeconds:0.#} s in {DriveWatchdog.DegradedWindowSeconds:0.#} s"));
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
            // What each command carries: the plan's leg speed and gripper (the course script sets both),
            // else the session's speed and the gripper as the stream reported it.
            float sendSpeed = speed, sendGripper = gripper;

            void Send(long now, in ArmPose pose)
            {
                int count = Targets(profile, pose, sendGripper, sendSpeed, targets);
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
            bool recording = false, notRecorded = false;
            // A safety take's events (docs/adr/0036 amendment 5): each sent to RobotHost, which records it (SE), as the arm leaves
            // free movement for its object; and after a knock, the next one planned once the objects have settled.
            var sentEvents = new HashSet<int>();
            uint eventSequence = (uint)(Stopwatch.GetTimestamp() & 0x3FFFFFFF) | 1u;
            var eventDatagram = new byte[ScriptEventCodec.Size];
            var replan = new Replanning(plan.Pending);
            // A mixed take (docs/adr/0038): its replay group (RP) as it starts, each planned move (MV) as it begins, and every command
            // sent while it records, for its replays to send again.
            var tape = new List<SentCommand>();
            int movesSent = 0;
            uint recordSequence = (uint)(Stopwatch.GetTimestamp() & 0x3FFFFFFF) | 2u;
            var recordDatagram = new byte[Math.Max(ScriptMoveCodec.Size, ReplayCodec.Size)];
            bool mixed = plan.Script == DriveScript.Mixed;
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
                        ? FormattableString.Invariant($"[joint-drive] {planTime,5:0} s: joint-state frames are not arriving (Wi-Fi?); holding until they do (it stops after {watchdog.DegradedLimitSeconds:0} s of this in {watchdog.DegradedWindowLength:0} s)")
                        : FormattableString.Invariant($"[joint-drive] {planTime,5:0} s: frames are back; going on"));
                }

                if (take != null && !recording && planTime >= plan.ScriptStart)
                {
                    // The take starts where the script does: the arm still (at home after the move there, or where it was),
                    // and its first command goes out as soon as RobotHost records.
                    planTime = plan.ScriptStart;
                    if (!StartTake(take, takeScenario!, scenarioIndex, link, stop, watchdog, commanded))
                    {
                        notRecorded = true;
                        break;
                    }

                    recording = true;
                    if (mixed) SendReplay(socket, robot, ReplayRecord.Source(seed), ref recordSequence, recordDatagram);
                    // The replays keep this take's look (the renderer keeps it while .keep-look stands beside its control file when a
                    // session closes; the batch makes and removes the file on these lines, docs/adr/0038 decision 2).
                    if (mixed && replays > 0) Console.WriteLine("[joint-drive] keep-look: on");
                    lastLoop = Stopwatch.GetTimestamp(); // the wait for RobotHost is not plan time
                    nextSend = lastLoop;
                    continue;
                }

                foreach (SafetyEvent e in plan.Events)
                {
                    if (planTime < e.ApproachStart || !sentEvents.Add(e.Index)) continue;
                    var record = new ScriptEvent((byte)e.Index, (byte)plan.EventsDrawn, e.Kind, e.ObjectId,
                        (float)(e.FinalStart - e.ApproachStart), (float)(e.Contact - e.ApproachStart), (float)(e.End - e.ApproachStart), (float)(e.Clear - e.ApproachStart));
                    eventSequence++;
                    for (int copy = 0; copy < 3 && ScriptEventCodec.TryEncode(record, eventSequence, now, eventDatagram, out int n); copy++)
                    {
                        socket.SendTo(eventDatagram, 0, n, SocketFlags.None, robot); // a few copies: UDP; RobotHost records one per sequence
                    }

                    Console.WriteLine(FormattableString.Invariant(
                        $"[joint-drive] {planTime,5:0.0} s: event {e.Index} of {plan.EventsDrawn}, {ScriptEventCodec.Name(e.Kind)} object {e.ObjectId}: contact planned at {e.Contact:0.0} s, clear at {e.Clear:0.0} s"));
                }

                while (mixed && recording && movesSent < plan.Moves.Count && plan.Moves[movesSent].Start <= planTime)
                {
                    SendMove(socket, robot, plan.Moves[movesSent].Record(planTime, plan.Moves.Count), ref recordSequence, recordDatagram);
                    movesSent++;
                }

                DrivePlan? next = replan.Step(planTime, now / (double)tps, link.World);
                if (next != null) plan = next;

                if (now >= nextSend)
                {
                    commanded = plan.Sample(planTime);
                    sendSpeed = plan.SpeedAt(planTime);
                    float plannedGripper = plan.GripperAt(planTime);
                    sendGripper = float.IsNaN(plannedGripper) ? gripper : plannedGripper;
                    Send(now, commanded);
                    if (recording) tape.Add(new SentCommand(planTime - plan.ScriptStart, commanded, sendGripper, sendSpeed));
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

            replan.Abandon();
            if (fault != DriveFault.None || stop.IsSet || notRecorded)
            {
                // Hold where the arm is, not where it was told to go: a stalled joint stops pushing.
                ArmPose hold = link.Measured;
                for (int i = 0; i < 15; i++)
                {
                    Send(Stopwatch.GetTimestamp(), hold);
                    Thread.Sleep(20);
                }
            }

            if (notRecorded)
            {
                log?.Close();
                Console.Error.WriteLine($"joint-drive: RobotHost did not start the take; the arm holds at {DrivePlan.Describe(link.Measured)}.");
                return 1;
            }

            log?.Close();
            Console.WriteLine($"[joint-drive] sent {sequence} commands, received {link.Frames} frames" +
                (pauses > 0 ? FormattableString.Invariant($", paused {pauses} time(s) for {pausedSeconds:0.0} s in all") : "") +
                (logPath != null ? $"; commands logged to {logPath}" : ""));
            switch (fault)
            {
                case DriveFault.Stall:
                    Console.Error.WriteLine($"joint-drive: STOPPED: the {DriveEnvelope.JointName(watchdog.StalledJoint)} joint stalled " +
                        $"(over {DriveWatchdog.StallErrorRadians} rad from its command and not moving for {watchdog.StallWindowSeconds} s); " +
                        "it now holds where it is. Check what it touched before going on.");
                    return FinishTake(take, link, 2);
                case DriveFault.FrameGap:
                    Console.Error.WriteLine($"joint-drive: STOPPED: no joint-state frame for {watchdog.DegradedLimitSeconds} s (the robot, RobotHost or the network is gone); " +
                        "sent a hold to the last measured pose.");
                    return FinishTake(take, link, 3);
                case DriveFault.StreamDegraded:
                    Console.Error.WriteLine($"joint-drive: STOPPED: the joint-state stream was missing or below {DriveWatchdog.MinFrameRateHz} Hz for {watchdog.DegradedLimitSeconds} s " +
                        $"of the last {watchdog.DegradedWindowLength} s. " +
                        "Either the Wi-Fi is losing frames, or something else (the headset app, joint-probe) is on the joint port: " +
                        "two SS lines in the session's robothost.jlog mean the second. Sent a hold to the last measured pose.");
                    return FinishTake(take, link, 3);
            }

            if (stop.IsSet)
            {
                Console.Error.WriteLine("joint-drive: stopped by hand; the arm holds where it is.");
                return FinishTake(take, link, 130);
            }

            Console.WriteLine(SafetyScript.IsSafety(plan.Script)
                ? "[joint-drive] done: the arm carries on from here next take (docs/adr/0036 amendment 3)."
                : "[joint-drive] done: the arm is back at home.");
            int code = FinishTake(take, link, 0);
            if (!mixed || take == null) return code;
            Console.WriteLine($"[joint-drive] take recorded: source of group {seed}, ended {code}");
            if (replays == 0 || code != 0 || objects == null || tape.Count == 0) return code;

            // docs/adr/0038 decision 2: the same commands again, on the source's layout with one edit each.
            var touched = new HashSet<byte>();
            if (WorldView.TryObjects(link.World!, out List<SafetyObject> after))
            {
                foreach (SafetyObject o in objects)
                {
                    int k = after.FindIndex(x => x.Id == o.Id);
                    if (k < 0 || Vector3Distance(o, after[k]) > 0.01f || Turned(o, after[k]) > 10f * MathF.PI / 180f) touched.Add(o.Id);
                }
            }

            // The operator log holds the source's commands; each replay sends the same again.
            var replayed = new ReplayContext(socket, robot, link, take, takeScenario!, scenarioIndex, profile, plan, tape, objects, touched, seed, stop, rateHz, stallSeconds,
                streamSeconds, mixedOptions, null);
            return Replay(replayed, replays, ref sequence, ref recordSequence);
        }

        private static float Vector3Distance(in SafetyObject a, in SafetyObject b) => System.Numerics.Vector3.Distance(a.Center, b.Center);

        private static float Turned(in SafetyObject a, in SafetyObject b) =>
            2f * MathF.Acos(MathF.Min(1f, MathF.Abs(System.Numerics.Quaternion.Dot(a.Rotation, b.Rotation))));

        /// <summary>Sends one planned move (MV) a few times under one sequence: UDP, and RobotHost records one per sequence.</summary>
        private static void SendMove(Socket socket, IPEndPoint robot, in ScriptMove m, ref uint sequence, byte[] buffer)
        {
            sequence++;
            for (int copy = 0; copy < 3 && ScriptMoveCodec.TryEncode(m, sequence, Stopwatch.GetTimestamp(), buffer, out int n); copy++) socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
        }

        /// <summary>Sends the take's replay group (RP) a few times under one sequence.</summary>
        private static void SendReplay(Socket socket, IPEndPoint robot, in ReplayRecord r, ref uint sequence, byte[] buffer)
        {
            sequence++;
            for (int copy = 0; copy < 3 && ReplayCodec.TryEncode(r, sequence, Stopwatch.GetTimestamp(), buffer, out int n); copy++) socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
        }

        /// <summary>What the replays of a mixed take need from its drive (docs/adr/0038 decision 2).</summary>
        private sealed class ReplayContext
        {
            public ReplayContext(Socket socket, IPEndPoint robot, Link link, TakeClient take, string scenario, int scenarioIndex, RobotArmProfile profile, DrivePlan plan,
                List<SentCommand> tape, List<SafetyObject> objects, HashSet<byte> touched, ulong group, ManualResetEventSlim stop, double rateHz, double stallSeconds,
                double streamSeconds, MixedOptions options, OperatorLog? log)
            {
                Socket = socket;
                Robot = robot;
                Link = link;
                Take = take;
                Scenario = scenario;
                ScenarioIndex = scenarioIndex;
                Profile = profile;
                Plan = plan;
                Tape = tape;
                Objects = objects;
                Touched = touched;
                Group = group;
                Stop = stop;
                RateHz = rateHz;
                StallSeconds = stallSeconds;
                StreamSeconds = streamSeconds;
                Options = options;
                Log = log;
            }

            public Socket Socket { get; }
            public IPEndPoint Robot { get; }
            public Link Link { get; }
            public TakeClient Take { get; }
            public string Scenario { get; }
            public int ScenarioIndex { get; }
            public RobotArmProfile Profile { get; }
            public DrivePlan Plan { get; }
            public List<SentCommand> Tape { get; }
            public List<SafetyObject> Objects { get; }
            public HashSet<byte> Touched { get; }
            public ulong Group { get; }
            public ManualResetEventSlim Stop { get; }
            public double RateHz { get; }
            public double StallSeconds { get; }
            public double StreamSeconds { get; }
            public MixedOptions Options { get; }
            public OperatorLog? Log { get; }
        }

        /// <summary>A world edit's outcome on the wire (RobotHost's placement codes): applied, and busy (motion not off yet).</summary>
        private const byte EditApplied = 0, EditBusy = 7;

        /// <summary>
        /// The replays of a mixed take (docs/adr/0038 decision 2): for each, the source's layout put back (reset-layout, checked against
        /// where the objects stood at the source's start), the arm moved slowly back to where the source began (a setup move, not in
        /// any take), one edit drawn and applied (park, or move; a placement RobotHost refuses drawn again), the objects at rest, and
        /// then a new take of the scenario that sends the source's commands again at the same times from its start, with its
        /// replay group (RP) and the source's moves (MV). Each says how it ended; the first that fails ends the replays. The exit
        /// code: 0, or the first failing replay's.
        /// </summary>
        private static int Replay(ReplayContext c, int replays, ref uint sequence, ref uint recordSequence)
        {
            // The look is kept until the last replay is recording (the renderer decides as each session closes), or the replays end.
            try
            {
                return Replays(c, replays, ref sequence, ref recordSequence);
            }
            finally
            {
                if (!_lookReleased) Console.WriteLine("[joint-drive] keep-look: off");
                _lookReleased = false;
            }
        }

        private static bool _lookReleased;

        private static int Replays(ReplayContext c, int replays, ref uint sequence, ref uint recordSequence)
        {
            for (int r = 1; r <= replays; r++)
            {
                if (c.Stop.IsSet) return 130;
                Console.WriteLine($"[joint-drive] replay {r} of {replays}: putting the source's layout back");
                if (!RestoreLayout(c))
                {
                    Console.Error.WriteLine("joint-drive: the source's layout could not be put back; no more replays");
                    return 0;
                }

                // Edits pause the operator's motion (RobotHost takes them only then): on again for the move back.
                if (ArmPower.ArmPowerCommand.Request(c.Robot, ArmPowerRequest.Hold, 10, out _) != 0)
                {
                    Console.Error.WriteLine("joint-drive: RobotHost did not switch motion back on after the reset; no more replays");
                    return 0;
                }

                ArmPose start = c.Plan.Sample(c.Plan.ScriptStart);
                if (!SetupMove(c, start, ref sequence))
                {
                    Console.Error.WriteLine($"joint-drive: no clear way back to where the source began ({DrivePlan.Describe(start)}); no more replays");
                    return c.Stop.IsSet ? 130 : 0;
                }

                ReplayEditDraw edit = default;
                bool applied = false;
                WorldScene scene = c.Link.World!.Scene;
                for (int attempt = 0; attempt < 6 && !applied && !c.Stop.IsSet; attempt++)
                {
                    edit = ReplayEdits.Draw(c.Group, r, attempt, c.Plan, c.Objects, scene, c.Touched, c.Profile);
                    if (edit.Pose.Id == 0) break;
                    if (edit.After.HasValue && !ReplayEdits.KeepsQuiet(c.Plan, edit.After.Value, c.Profile, c.Profile.Collision!, c.Options))
                    {
                        Console.WriteLine($"[joint-drive] replay {r}: {ReplayCodec.Describe(edit.Record)} would meet the take's quiet start or end; another");
                        continue;
                    }

                    byte outcome = EditWorld(c, edit.Op, edit.Pose);
                    applied = outcome == EditApplied;
                    if (!applied) Console.WriteLine($"[joint-drive] replay {r}: RobotHost refused {ReplayCodec.Describe(edit.Record)} (placement code {outcome}); another");
                }

                if (!applied || !WaitAtRest(c, 20))
                {
                    Console.Error.WriteLine($"joint-drive: replay {r} found no edit RobotHost takes, or the objects did not settle; no more replays");
                    return c.Stop.IsSet ? 130 : 0;
                }

                if (ArmPower.ArmPowerCommand.Request(c.Robot, ArmPowerRequest.Hold, 10, out _) != 0)
                {
                    Console.Error.WriteLine("joint-drive: RobotHost did not switch motion back on after the edit; no more replays");
                    return 0;
                }

                int code = ReplayTake(c, r, edit.Record, ref sequence, ref recordSequence, r == replays);
                Console.WriteLine($"[joint-drive] take recorded: replay {r} of {replays} of group {c.Group}, {ReplayCodec.Describe(edit.Record)}, ended {code}");
                if (code != 0 && code != 4) return code;
            }

            return 0;
        }

        private static uint _editSequence = (uint)(Stopwatch.GetTimestamp() & 0x3FFFFFFF) | 1u;

        /// <summary>One world edit, motion paused meanwhile (RobotHost takes edits only then): sent until the status answers it; its outcome (255 stopped, 254 no answer in 10 s).</summary>
        private static byte EditWorld(ReplayContext c, WorldEditOp op, in ObjectPose pose)
        {
            WorldState world = c.Link.World!;
            var buffer = new byte[256];
            uint heartbeat = 0, edit = ++_editSequence;
            long tps = Stopwatch.Frequency, next = 0, until = Stopwatch.GetTimestamp() + 10 * tps;
            while (true)
            {
                long now = Stopwatch.GetTimestamp();
                if (c.Stop.IsSet) return 255;
                if (now > until) return 254;
                if (world.Status.EditSequence == edit)
                {
                    if (world.Status.EditOutcome != EditBusy) return world.Status.EditOutcome;
                    edit = ++_editSequence; // busy (motion not off yet): a refused sequence is spent, so a new one
                }

                if (now >= next)
                {
                    ArmPowerCodec.TryEncodeRequest(OperatorClientKind.Tool, OperatorMotionMode.Paused, ArmPowerRequest.None, ++heartbeat, now, buffer, out int n);
                    c.Socket.SendTo(buffer, 0, n, SocketFlags.None, c.Robot);
                    WorldCodec.TryEncodeEdit(op, pose.Id, edit, now, pose, buffer, out n);
                    c.Socket.SendTo(buffer, 0, n, SocketFlags.None, c.Robot);
                    next = now + tps * 3 / 10;
                }

                c.Link.Pump(now, null);
                Thread.Sleep(5);
            }
        }

        /// <summary>Waits until every object has stayed put for <see cref="SettleSeconds"/> (<see cref="RestWatch"/>), at most <paramref name="seconds"/>.</summary>
        private static bool WaitAtRest(ReplayContext c, double seconds)
        {
            var rest = new RestWatch(SettleSeconds);
            long tps = Stopwatch.Frequency, until = Stopwatch.GetTimestamp() + (long)(seconds * tps);
            while (!c.Stop.IsSet && Stopwatch.GetTimestamp() < until)
            {
                long now = Stopwatch.GetTimestamp();
                c.Link.Pump(now, null);
                if (rest.Still(c.Link.World!, now / (double)tps)) return true;
                Thread.Sleep(20);
            }

            return false;
        }

        /// <summary>
        /// The source's layout back: reset-layout (RobotHost puts each object where the layout drew it), the objects at rest, and every
        /// object within 1 cm and 5 degrees of where it stood when the source began (a knocked object lying on another's spot keeps
        /// that one from going back: tried three times). False when it is not.
        /// </summary>
        private static bool RestoreLayout(ReplayContext c)
        {
            string why = "";
            for (int attempt = 0; attempt < 3 && !c.Stop.IsSet; attempt++)
            {
                byte outcome = EditWorld(c, WorldEditOp.ResetLayout, default);
                if (outcome != EditApplied)
                {
                    why = $"reset-layout refused (code {outcome})";
                    continue;
                }

                if (!WaitAtRest(c, 20) || !WorldView.TryObjects(c.Link.World!, out List<SafetyObject> now))
                {
                    why = "the objects did not settle";
                    continue;
                }

                why = "";
                foreach (SafetyObject o in c.Objects)
                {
                    int k = now.FindIndex(x => x.Id == o.Id);
                    if (k < 0) why += FormattableString.Invariant($" {o.Id} is off the table;");
                    else if (Vector3Distance(o, now[k]) > 0.01f || Turned(o, now[k]) > 5f * MathF.PI / 180f)
                        why += FormattableString.Invariant($" {o.Id} is {Vector3Distance(o, now[k]) * 1000:0} mm and {Turned(o, now[k]) * 180 / MathF.PI:0} degrees from where it stood;");
                }

                if (now.Count != c.Objects.Count) why += $" {now.Count} objects on the table, not {c.Objects.Count};";
                if (why.Length == 0) return true;
                Console.WriteLine($"[joint-drive] the layout is not the source's yet:{why} again");
            }

            Console.Error.WriteLine($"joint-drive: {why}");
            return false;
        }

        /// <summary>
        /// Slowly (<see cref="DrivePlan.ApproachSpeedRadPerSec"/>) from where the arm is to <paramref name="to"/>, on a path clear of the
        /// objects as they now stand by 1 cm (straight, or a joint at a time), then still for half a second: a setup move, not in any
        /// take. False when there is no such path, or the drive stops.
        /// </summary>
        private static bool SetupMove(ReplayContext c, in ArmPose to, ref uint sequence)
        {
            ArmCollisionModel model = c.Profile.Collision!;
            if (!WorldView.TryObjects(c.Link.World!, out List<SafetyObject> now)) return false;
            ArmPose from = c.Link.Measured;
            if (!DrivePlan.TryApproach(c.Profile, SafetyScript.WithObjects(model, now, 0.01f), from, to, out List<(ArmPose To, bool Step)> via, out string? error))
            {
                Console.Error.WriteLine($"joint-drive: {error}");
                return false;
            }

            var points = new List<(double At, ArmPose Pose)> { (0, from) };
            double t = 0;
            ArmPose p = from;
            foreach ((ArmPose next, bool _) in via)
            {
                t += DrivePlan.SmoothDuration(p.MaxJointDelta(next), DrivePlan.ApproachSpeedRadPerSec);
                points.Add((t, next));
                p = next;
            }

            ArmPose Sample(double at)
            {
                for (int i = 1; i < points.Count; i++)
                {
                    if (at > points[i].At) continue;
                    double u = (at - points[i - 1].At) / Math.Max(1e-9, points[i].At - points[i - 1].At);
                    return ArmPose.Lerp(points[i - 1].Pose, points[i].Pose, (float)HandModel.MinimumJerk(u));
                }

                return points[points.Count - 1].Pose;
            }

            var watchdog = new DriveWatchdog(c.StallSeconds, c.StreamSeconds);
            var targets = new JointTarget[c.Profile.JointCount];
            var datagram = new byte[MaxDatagramBytes];
            long tps = Stopwatch.Frequency, t0 = Stopwatch.GetTimestamp(), nextSend = t0, interval = (long)(tps / c.RateHz);
            float gripper = c.Tape[0].Gripper, speed = DrivePlan.ApproachSpeedRadPerSec;
            ArmPose commanded = from;
            while (true)
            {
                long nowTicks = Stopwatch.GetTimestamp();
                double at = (nowTicks - t0) / (double)tps;
                c.Link.Pump(nowTicks, watchdog, commanded);
                if (c.Stop.IsSet || watchdog.Check(nowTicks / (double)tps) != DriveFault.None) return false;
                if (at > t + 0.5) return true;
                if (nowTicks >= nextSend)
                {
                    commanded = Sample(at);
                    int count = Targets(c.Profile, commanded, gripper, speed, targets);
                    if (JointCommandCodec.TryEncode(sequence, nowTicks, targets.AsSpan(0, count), datagram, out int written))
                    {
                        c.Socket.SendTo(datagram, 0, written, SocketFlags.None, c.Robot);
                        c.Log?.Command(nowTicks, sequence, nowTicks, targets.AsSpan(0, count));
                        sequence++;
                    }

                    nextSend += interval;
                    if (nextSend < nowTicks) nextSend = nowTicks + interval;
                }

                Thread.Sleep(1);
            }
        }

        /// <summary>
        /// One replay take: started as the source was (RECORD), its replay group (RP) sent, then the source's commands at their times
        /// from the take's start (the clock held while the joint-state stream is unhealthy, as the source's was) and its moves (MV) at
        /// theirs, the watchdog as for any drive; then stopped and kept. Its exit code, as a drive's.
        /// </summary>
        private static int ReplayTake(ReplayContext c, int replay, in ReplayRecord record, ref uint sequence, ref uint recordSequence, bool last)
        {
            var watchdog = new DriveWatchdog(c.StallSeconds, c.StreamSeconds);
            ArmPose commanded = c.Tape[0].Pose;
            if (!StartTake(c.Take, c.Scenario, c.ScenarioIndex, c.Link, c.Stop, watchdog, commanded))
            {
                Console.Error.WriteLine($"joint-drive: RobotHost did not start replay {replay}'s take");
                return 1;
            }

            var recordDatagram = new byte[Math.Max(ScriptMoveCodec.Size, ReplayCodec.Size)];
            SendReplay(c.Socket, c.Robot, record, ref recordSequence, recordDatagram);
            if (last)
            {
                Console.WriteLine("[joint-drive] keep-look: off");
                _lookReleased = true;
            }
            var targets = new JointTarget[c.Profile.JointCount];
            var datagram = new byte[MaxDatagramBytes];
            long tps = Stopwatch.Frequency, lastLoop = Stopwatch.GetTimestamp();
            double clock = 0;
            int moves = 0;
            DriveFault fault = DriveFault.None;
            List<SentCommand> tape = c.Tape;
            var player = new TapePlayer(tape);
            while (true)
            {
                long now = Stopwatch.GetTimestamp();
                double dt = (now - lastLoop) / (double)tps;
                lastLoop = now;
                c.Link.Pump(now, watchdog, commanded);
                bool healthy = watchdog.IsHealthy(now / (double)tps);
                fault = watchdog.Check(now / (double)tps);
                if (fault != DriveFault.None || c.Stop.IsSet) break;
                if (healthy) clock += dt;
                double planTime = c.Plan.ScriptStart + clock;
                while (moves < c.Plan.Moves.Count && c.Plan.Moves[moves].Start <= planTime)
                {
                    SendMove(c.Socket, c.Robot, c.Plan.Moves[moves].Record(planTime, c.Plan.Moves.Count), ref recordSequence, recordDatagram);
                    moves++;
                }

                while (player.TryNext(clock, out SentCommand s))
                {
                    commanded = s.Pose;
                    int count = Targets(c.Profile, s.Pose, s.Gripper, s.Speed, targets);
                    if (JointCommandCodec.TryEncode(sequence, now, targets.AsSpan(0, count), datagram, out int written))
                    {
                        c.Socket.SendTo(datagram, 0, written, SocketFlags.None, c.Robot);
                        c.Log?.Command(now, sequence, now, targets.AsSpan(0, count));
                        sequence++;
                    }
                }

                if (player.Done(clock, 0.5)) break;
                Thread.Sleep(1);
            }

            if (fault != DriveFault.None || c.Stop.IsSet)
            {
                ArmPose hold = c.Link.Measured;
                SentCommand lastSent = tape[Math.Max(0, player.Played - 1)];
                for (int i = 0; i < 15; i++)
                {
                    int count = Targets(c.Profile, hold, lastSent.Gripper, lastSent.Speed, targets);
                    if (JointCommandCodec.TryEncode(sequence, Stopwatch.GetTimestamp(), targets.AsSpan(0, count), datagram, out int written))
                    {
                        c.Socket.SendTo(datagram, 0, written, SocketFlags.None, c.Robot);
                        sequence++;
                    }

                    Thread.Sleep(20);
                }
            }

            int code = fault == DriveFault.Stall ? 2 : fault != DriveFault.None ? 3 : c.Stop.IsSet ? 130 : 0;
            if (code != 0) Console.Error.WriteLine($"joint-drive: replay {replay} STOPPED ({fault}{(c.Stop.IsSet ? ", by hand" : "")}); the arm holds where it is");
            return FinishTake(c.Take, c.Link, code);
        }

        /// <summary>How long a RECORD request is sent again before it counts as unanswered (the headset gives up after 2 s).</summary>
        private const double RequestSeconds = 5;

        /// <summary>
        /// The world the safety scripts plan in (docs/adr/0036 amendment 1): RobotHost's world status (its scene must be ours)
        /// and every object's pose, from the joint port. With <paramref name="newLayout"/>, first asks RobotHost for a new random
        /// layout, which it takes only while the operator's motion is not on: a heartbeat with motion paused, the edit re-sent
        /// until the status answers it, then torque on again (the hold request, motion on). Either way the objects must then
        /// be at rest (no move over 1 mm or 1 degree for <see cref="SettleSeconds"/>). False, said why, if any of it fails.
        /// </summary>
        private static bool PrepareWorld(Link link, Socket socket, IPEndPoint robot, bool newLayout, ManualResetEventSlim stop, out List<SafetyObject>? objects)
        {
            objects = null;
            WorldState world = link.World!;
            long tps = Stopwatch.Frequency;
            long until = Stopwatch.GetTimestamp() + 10 * tps;
            while (!world.HasStatus || !WorldView.TryObjects(world, out _))
            {
                if (stop.IsSet) return false;
                if (Stopwatch.GetTimestamp() > until)
                {
                    Console.Error.WriteLine("joint-drive: RobotHost sent no world in 10 s (it needs --scene, and the world frames go to a subscribed client). Nothing moved.");
                    return false;
                }

                link.Pump(Stopwatch.GetTimestamp(), null);
                Thread.Sleep(5);
            }

            if (world.SceneMismatch)
            {
                Console.Error.WriteLine("joint-drive: RobotHost runs another scene file than --scene (their sha256 differ). Nothing moved.");
                return false;
            }

            if (newLayout)
            {
                uint before = world.Status.LayoutNumber;
                var buffer = new byte[256];
                uint heartbeat = 0, edit = (uint)(Stopwatch.GetTimestamp() & 0x3FFFFFFF) | 1u;
                long next = 0;
                until = Stopwatch.GetTimestamp() + 10 * tps;
                Console.WriteLine($"[joint-drive] asking RobotHost for a new layout (now layout {before}); motion paused meanwhile");
                while (true)
                {
                    long now = Stopwatch.GetTimestamp();
                    if (stop.IsSet) return false;
                    if (now > until)
                    {
                        Console.Error.WriteLine($"joint-drive: RobotHost did not apply a new layout in 10 s (status: edit {world.Status.EditSequence}, outcome {world.Status.EditOutcome}). Nothing moved.");
                        return false;
                    }

                    if (world.Status.EditSequence == edit)
                    {
                        if (world.Status.EditOutcome == 0 && world.Status.LayoutNumber != before) break; // applied
                        if (world.Status.EditOutcome != 0) edit++; // busy (motion not off yet): a refused sequence is spent, so a new one
                    }

                    if (now >= next)
                    {
                        ArmPowerCodec.TryEncodeRequest(OperatorClientKind.Tool, OperatorMotionMode.Paused, ArmPowerRequest.None, ++heartbeat, now, buffer, out int n);
                        socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                        WorldCodec.TryEncodeEdit(WorldEditOp.NewLayout, 0, edit, now, default, buffer, out n);
                        socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                        next = now + tps * 3 / 10;
                    }

                    link.Pump(now, null);
                    Thread.Sleep(5);
                }

                Console.WriteLine($"[joint-drive] layout {world.Status.LayoutNumber} (seed {world.Status.LayoutSeed}); waiting for the objects to settle");
            }

            // At rest: every object's newest pose within 1 mm and 1 degree of where it was SettleSeconds ago.
            var scene = world.Scene;
            var anchor = new ObjectPose[scene.Objects.Length];
            var restless = new string[scene.Objects.Length];
            long anchorAt = 0;
            until = Stopwatch.GetTimestamp() + 20 * tps;
            while (true)
            {
                long now = Stopwatch.GetTimestamp();
                if (stop.IsSet) return false;
                if (now > until)
                {
                    string which = string.Join("; ", restless.Where(r => r != null));
                    Console.Error.WriteLine($"joint-drive: the objects did not come to rest in 20 s ({(which.Length > 0 ? which : "never all posed")}). Nothing moved.");
                    return false;
                }

                link.Pump(now, null);
                bool still = anchorAt != 0;
                for (int i = 0; i < anchor.Length; i++)
                {
                    world.TryGet(scene.Objects[i].Id, out ObjectPose p, out _);
                    if (anchorAt == 0) continue;
                    float moved = new System.Numerics.Vector3(p.X - anchor[i].X, p.Y - anchor[i].Y, p.Z - anchor[i].Z).Length();
                    float turned = 2f * MathF.Acos(MathF.Min(1f, MathF.Abs(p.Qw * anchor[i].Qw + p.Qx * anchor[i].Qx + p.Qy * anchor[i].Qy + p.Qz * anchor[i].Qz)));
                    if (moved > 0.001f || turned > MathF.PI / 180f)
                    {
                        still = false;
                        restless[i] = FormattableString.Invariant($"{scene.Objects[i].Name} moved {moved * 1000:0.#} mm, turned {turned * 180 / MathF.PI:0.#} deg at ({p.X:0.000}, {p.Y:0.000}, {p.Z:0.000})");
                    }
                    else restless[i] = null!;
                }

                if (still && now - anchorAt >= (long)(SettleSeconds * tps)) break;
                if (!still)
                {
                    for (int i = 0; i < anchor.Length; i++) world.TryGet(scene.Objects[i].Id, out anchor[i], out _);
                    anchorAt = now;
                }

                Thread.Sleep(20);
            }

            WorldView.TryObjects(world, out List<SafetyObject> found);
            objects = found;
            Console.WriteLine("[joint-drive] objects at rest: " + string.Join("; ", found.Select(o =>
                FormattableString.Invariant($"{o.Id} at ({o.Center.X:0.000}, {o.Center.Y:0.000}, {o.Center.Z:0.000})"))));
            if (newLayout && ArmPower.ArmPowerCommand.Request(robot, ArmPowerRequest.Hold, 10, out _) != 0)
            {
                Console.Error.WriteLine("joint-drive: RobotHost did not switch motion back on after the new layout. Nothing moved.");
                return false;
            }

            return true;
        }

        /// <summary>How long every object must stay put before a safety script plans around it, seconds.</summary>
        public const double SettleSeconds = 1.0;

        /// <summary>
        /// Waits for RobotHost's scenario list and finds <paramref name="scenario"/> on it, before anything moves. False, with the
        /// reason printed, when the list never comes or the scenario is not on it.
        /// </summary>
        private static bool AwaitScenario(TakeClient take, string scenario, Link link, ManualResetEventSlim stop, out int index)
        {
            index = -1;
            long tps = Stopwatch.Frequency;
            for (long until = Stopwatch.GetTimestamp() + 10 * tps; !take.State.HasList || !take.State.HasStatus;)
            {
                if (stop.IsSet) return false;
                if (Stopwatch.GetTimestamp() > until)
                {
                    Console.Error.WriteLine("joint-drive: RobotHost sent no scenario list in 10 s: it records no takes (it needs --scenarios; " +
                        "for Isaac, just isaac-jetrover-host with a scene). Nothing moved.");
                    return false;
                }

                link.Pump(Stopwatch.GetTimestamp(), default);
                Thread.Sleep(5);
            }

            index = take.IndexOf(scenario);
            if (index < 0)
            {
                Console.Error.WriteLine($"joint-drive: RobotHost has no scenario '{scenario}'; it has: {take.Names()}. Nothing moved.");
                return false;
            }

            return true;
        }

        /// <summary>
        /// Starts the take: sends Start until RobotHost's status echoes it, the joint-state stream still feeding
        /// <paramref name="watchdog"/> against <paramref name="commanded"/>. False, with the reason printed, when RobotHost refuses
        /// or does not answer.
        /// </summary>
        private static bool StartTake(TakeClient take, string scenario, int index, Link link, ManualResetEventSlim stop, DriveWatchdog watchdog, in ArmPose commanded)
        {
            if (stop.IsSet) return false;
            RecordOutcome outcome = Request(take, link, RecordOp.Start, (byte)index, take.State.ListRevision, watchdog, commanded);
            if (outcome != RecordOutcome.Started || take.State.Status.State != RecordState.Recording)
            {
                Console.Error.WriteLine($"joint-drive: RobotHost did not start a take of '{scenario}' ({Describe(outcome)}).");
                return false;
            }

            Console.WriteLine($"[joint-drive] recording take {take.State.Status.Take} of '{scenario}'");
            return true;
        }

        /// <summary>
        /// After a knock (docs/adr/0036 amendment 5): watches the objects until they are at rest while the arm goes on along the
        /// plan's runway (free movement away from the knocked object). Then the plan made at the start for after the knock takes
        /// over if the world is as it assumed (<see cref="SafetyContinuation.Holds"/>); otherwise the next event is planned on a
        /// thread of its own against the objects as they now are, and the new plan handed back if it comes in time to take over at
        /// its switch point. Either plan is the same as the one driven before that point, so nothing jumps; without one the runway
        /// ends the take.
        /// </summary>
        private sealed class Replanning
        {
            /// <summary>How long every object must stay put before the next event is planned against them, seconds.</summary>
            public const double StillSeconds = 0.6;

            /// <summary>
            /// The planner's time to plan the next event, seconds of the runway (half as much again for a second try). Measured on
            /// the GPU machine with the twin and other tests running (2026-11-21): a contact event plans in 0.1-3 s, and a
            /// knock planned against the objects a knock left in 4-11 s, mostly on its knocked object again after the others
            /// failed.
            /// </summary>
            public const double BudgetSeconds = 6.0;

            /// <summary>How long after a knock the objects may take to settle before the runway ends the take, seconds.</summary>
            public const double SettleSeconds = 6.0;

            private SafetyContinuation? _pending;
            private RestWatch? _rest;
            private Task<(DrivePlan Plan, string Outcome)>? _task;
            private double _switchAt = double.NaN;
            private int _tries;

            public Replanning(SafetyContinuation? pending)
            {
                _pending = pending;
            }

            /// <summary>One turn of the drive loop at <paramref name="planTime"/> (<paramref name="now"/>: seconds on a rising clock): the plan to take over, or null.</summary>
            public DrivePlan? Step(double planTime, double now, WorldState? world)
            {
                if (_task != null)
                {
                    if (!_task.IsCompleted) return null;
                    Task<(DrivePlan Plan, string Outcome)> done = _task;
                    _task = null;
                    if (done.IsFaulted || done.IsCanceled)
                    {
                        Console.Error.WriteLine($"joint-drive: planning after the knock failed ({done.Exception?.GetBaseException().Message}); the runway ends the take");
                        _pending = null;
                        return null;
                    }

                    (DrivePlan plan, string outcome) = done.Result;
                    if (planTime < _switchAt - HandModel.TickSeconds)
                    {
                        Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: after the knock, {outcome}; it takes over at {_switchAt:0.0} s"));
                        _pending = plan.Pending;
                        _rest = null;
                        _tries = 0;
                        return plan;
                    }

                    if (++_tries >= 2)
                    {
                        Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: planning after the knock came in late again; the runway ends the take"));
                        _pending = null;
                        return null;
                    }

                    Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: planning after the knock came in late (for {_switchAt:0.00} s); again, with more time"));
                }

                if (_pending == null || world == null || planTime < _pending.KnockEnd) return null;
                _rest ??= new RestWatch(StillSeconds);
                if (!_rest.Still(world, now))
                {
                    if (planTime > _pending.KnockEnd + SettleSeconds)
                    {
                        Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: the objects did not settle within {SettleSeconds:0} s of the knock; the runway ends the take"));
                        _pending = null;
                    }

                    return null;
                }

                if (!WorldView.TryObjects(world, out List<SafetyObject> objects))
                {
                    Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: the objects are not all posed; the runway ends the take"));
                    _pending = null;
                    return null;
                }

                // What was planned at the start for after this knock, if the world is as it assumed: no planning, nothing to wait for.
                if (_task == null && _tries == 0 && _pending.Assumed != null && planTime < _pending.AssumedAt - HandModel.TickSeconds)
                {
                    if (_pending.Holds(objects, out string why))
                    {
                        DrivePlan assumed = _pending.Assumed;
                        Console.WriteLine(FormattableString.Invariant(
                            $"[joint-drive] {planTime,5:0.0} s: the objects are at rest as the plan assumed ({why}); event {_pending.NextIndex} as planned, from {_pending.AssumedAt:0.0} s"));
                        _pending = assumed.Pending;
                        _rest = null;
                        return assumed;
                    }

                    Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: the plan made for after the knock no longer holds ({why})"));
                }

                _switchAt = _pending.SwitchAfter(planTime + BudgetSeconds * (1 + 0.5 * _tries));
                if (double.IsNaN(_switchAt))
                {
                    Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: no time left on the runway to plan another event; it ends the take"));
                    _pending = null;
                    return null;
                }

                SafetyContinuation c = _pending;
                double at = _switchAt;
                Console.WriteLine(FormattableString.Invariant($"[joint-drive] {planTime,5:0.0} s: the objects are at rest; planning event {c.NextIndex} against them, to take over at {at:0.0} s"));
                _task = Task.Run(() =>
                {
                    DrivePlan p = c.Continue(objects, at, out string outcome);
                    return (p, outcome);
                });
                return null;
            }

            /// <summary>The drive is over: nothing more is planned (a plan being made is left to finish on its own).</summary>
            public void Abandon() => _pending = null;
        }

        /// <summary>The most replays a mixed take records after itself (docs/adr/0038 decision 2).</summary>
        public const int MaxReplays = 8;

        /// <summary>A count (2) or a range (1-200) of non-negative integers, for <paramref name="option"/>.</summary>
        internal static void ParseRange(string text, string option, out int min, out int max)
        {
            string[] parts = text.Split('-');
            if (parts.Length > 2 || !int.TryParse(parts[0], NumberStyles.None, CultureInfo.InvariantCulture, out min) ||
                !int.TryParse(parts[parts.Length - 1], NumberStyles.None, CultureInfo.InvariantCulture, out max) || max < min)
            {
                throw new ArgumentException($"{option} is a count or a range, like 2 or 1-200, not '{text}'");
            }
        }

        /// <summary>--events: how many events a safety take draws, a count (2) or a range (1-3) (docs/adr/0036 amendment 5).</summary>
        internal static void ParseEvents(string text, SafetyOptions options)
        {
            string[] parts = text.Split('-');
            if (parts.Length > 2 || !int.TryParse(parts[0], NumberStyles.None, CultureInfo.InvariantCulture, out int min) ||
                !int.TryParse(parts[parts.Length - 1], NumberStyles.None, CultureInfo.InvariantCulture, out int max))
            {
                throw new ArgumentException($"--events is a count or a range, like 2 or 1-3, not '{text}'");
            }

            options.MinEvents = min;
            options.MaxEvents = max;
        }

        /// <summary>Stops and keeps the take, if one is going; the drive's own exit code stands unless that fails (exit 4).</summary>
        private static int FinishTake(TakeClient? take, Link link, int code)
        {
            if (take == null || !take.State.HasStatus || take.State.Status.State != RecordState.Recording) return code;
            RecordOutcome outcome = Request(take, link, RecordOp.Stop, RecordStatus.NoScenario, take.State.ListRevision);
            if (outcome == RecordOutcome.Saved)
            {
                Console.WriteLine($"[joint-drive] take {take.State.Status.Take} saved ({take.State.Status.TakesKept} kept of this scenario); " +
                    "RobotHost completes it, and the LeRobot converter adds it about a minute later");
                return code;
            }

            Console.Error.WriteLine($"joint-drive: the take was not stopped cleanly ({Describe(outcome)}); RobotHost stops it by itself 10 s after the last command.");
            return code == 0 ? 4 : code;
        }

        /// <summary>
        /// One RECORD request, sent every 0.25 s until RobotHost's status echoes it; its outcome, or None unanswered. With
        /// <paramref name="watchdog"/>, the stream feeds it meanwhile, against <paramref name="commanded"/>.
        /// </summary>
        private static RecordOutcome Request(TakeClient take, Link link, RecordOp op, byte scenario, ushort revision, DriveWatchdog? watchdog = null,
            ArmPose commanded = default)
        {
            long tps = Stopwatch.Frequency;
            long now = Stopwatch.GetTimestamp();
            uint sequence = take.Begin(op, scenario, revision, now);
            long nextSend = now + tps / 4, until = now + (long)(RequestSeconds * tps);
            while (!take.Answered(sequence))
            {
                now = Stopwatch.GetTimestamp();
                if (now > until) return RecordOutcome.None;
                if (now >= nextSend)
                {
                    take.Resend();
                    nextSend = now + tps / 4;
                }

                link.Pump(now, watchdog, commanded);
                Thread.Sleep(5);
            }

            return take.State.Status.Outcome;
        }

        private static string Describe(RecordOutcome outcome) => outcome == RecordOutcome.None ? "no answer" : outcome.ToString();

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

            /// <summary>Takes the joint port's RECORD status and scenario list, when the drive records a take.</summary>
            public TakeClient? Take { get; set; }

            /// <summary>Takes RobotHost's world status, frames and sizes, when a safety script plans around the objects.</summary>
            public WorldState? World { get; set; }

            private readonly byte[] _worldSha = new byte[WorldCodec.Sha256Bytes];
            private readonly ObjectPose[] _worldObjects = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
            private readonly byte[] _sizeIds = new byte[256];
            private readonly float[] _sizeScales = new float[256];

            private bool TakeWorld(ReadOnlySpan<byte> datagram)
            {
                if (World == null) return false;
                if (WorldCodec.TryDecodeStatus(datagram, out WorldStatus status, _worldSha, out _))
                {
                    World.ApplyStatus(status, _worldSha);
                    return true;
                }

                if (WorldCodec.TryDecodeFrame(datagram, out _, out long sensed, out _, out _, _worldObjects, out int count))
                {
                    for (int i = 0; i < count; i++) World.Apply(_worldObjects[i], sensed);
                    return true;
                }

                if (WorldCodec.TryDecodeSizes(datagram, out uint revision, _sizeIds, _sizeScales, out int sizes))
                {
                    World.ApplySizes(revision, _sizeIds.AsSpan(0, sizes), _sizeScales.AsSpan(0, sizes));
                    return true;
                }

                return false;
            }

            /// <summary>Sends the keepalive when due and takes in every frame waiting; with a watchdog, feeds it each frame and fresh reading.</summary>
            public void Pump(long now, DriveWatchdog? watchdog, in ArmPose commanded = default)
            {
                if (now >= _nextKeepalive)
                {
                    JointStateCodec.TryEncodeSubscribe(now, Stopwatch.Frequency, _buffer, out int n);
                    _socket.SendTo(_buffer, 0, n, SocketFlags.None, _robot);
                    _nextKeepalive = now + Stopwatch.Frequency / 2;
                }

                while (_socket.Poll(0, SelectMode.SelectRead)) // Poll, not Available: an empty datagram reads as Available 0 and would block the queue (B-01)
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
                        if (!TakeWorld(_buffer.AsSpan(0, received))) Take?.OnDatagram(_buffer.AsSpan(0, received));
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
