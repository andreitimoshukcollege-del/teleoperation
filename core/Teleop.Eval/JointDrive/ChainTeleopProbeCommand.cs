using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Threading;
using Teleop.Core.Prediction;
using Teleop.Core.Types;
using Teleop.Eval.Net;
using Teleop.Eval.Time;
using Teleop.RobotArm.Collision;
using Teleop.RobotArm.Kinematics;
using Teleop.RobotArm.Operator;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// <c>chain-teleop-probe</c> (<c>just fr3-teleop-probe</c>): the headset's two-hand operator (docs/adr/0029) with
    /// scripted hands instead of tracked ones, against a running serial-chain RobotHost. It is the same
    /// <see cref="SerialChainJointLink"/> and <see cref="SerialChainTeleop"/> the Galaxy XR runs, over a real socket,
    /// so the whole path the headset uses is exercised headlessly: keepalive, ClockSync, the robot's pose taken as
    /// the first command, clutch, IK, the collision check, 48 Hz commands, and the predictor drawing the twin.
    /// <list type="number">
    /// <item>Waits up to 5 s for frames and a synced clock, then takes the robot's pose.</item>
    /// <item>Grabs with both hands: over 2 s the left hand moves <c>--reach-m</c> forward and turns 15° about the
    /// vertical, and the right hand rises <c>--reach-m</c>; the left gripper closes. Then both hold still for 2 s.</item>
    /// <item>Passes when each arm's <i>measured</i> tool is within <c>--tolerance-mm</c> of its clutch target, the left
    /// gripper reads closed, no step was blocked, and the twin was right: each frame's joints against the estimate the
    /// predictor had made, before that frame arrived, for the instant it was read (the nearest loop tick), within 1° at p95 (<c>joint_prediction_error_deg</c>'s shape, docs/metrics.md §4,
    /// here a diagnostic).</item>
    /// </list>
    /// Simulated robots only (<c>--simulated-robot</c>), as <c>chain-drive</c>. Exit 0 pass, 1 fail, 2 no robot, 64 usage.
    /// </summary>
    public static class ChainTeleopProbeCommand
    {
        private const string Usage =
            "Usage: Teleop.Eval chain-teleop-probe --simulated-robot [--remote-host 127.0.0.1] [--remote-port 6002] [--local-port 6013]\n" +
            "  [--profile RobotProfiles/fr3-duo.json] [--reach-m 0.08] [--tolerance-mm 3] [--predictor franka-replay] [--as-headset]\n" +
            "  --as-headset: the heartbeat says Headset, so RobotHost records the session as it would the Galaxy XR's (to check that path).";

        public static int Run(string[] args)
        {
            string host = "127.0.0.1", profilePath = Path.Combine("RobotProfiles", "fr3-duo.json"), predictor = "franka-replay";
            int port = 6002, localPort = 6013;
            double reach = 0.08, toleranceMm = 3;
            bool simulated = false, asHeadset = false;
            try
            {
                for (int i = 1; i < args.Length; i++)
                {
                    string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                    switch (args[i])
                    {
                        case "--remote-host": host = Next(); break;
                        case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--local-port": localPort = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--profile": profilePath = Next(); break;
                        case "--reach-m": reach = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--tolerance-mm": toleranceMm = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--predictor": predictor = Next(); break;
                        case "--simulated-robot": simulated = true; break;
                        case "--as-headset": asHeadset = true; break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }

                if (!(reach > 0 && reach <= 0.2)) throw new ArgumentException("--reach-m must be in (0, 0.2]");
                if (!IPAddress.TryParse(host, out _)) throw new ArgumentException($"--remote-host '{host}' is not an IP address");
                if (predictor != "franka-replay" && predictor != "hold-last") throw new ArgumentException("--predictor is franka-replay or hold-last");
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"chain-teleop-probe: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            if (!simulated)
            {
                Console.Error.WriteLine("chain-teleop-probe: only a simulated robot (RobotHost --backend sim) may be driven (docs/adr/0029). Pass --simulated-robot.");
                return 64;
            }

            SerialChainProfile profile = ImportUrdf.SerialChainProfileJson.Load(profilePath);
            string collisionPath = Path.ChangeExtension(profilePath, null) + ".collision.json";
            var doc = System.Text.Json.JsonSerializer.Deserialize<SerialChainCollisionDocument>(File.ReadAllText(collisionPath),
                new System.Text.Json.JsonSerializerOptions { IncludeFields = true })!;
            SerialChainCollisionModel collision = doc.ToModel();

            var clock = new MonotonicClock();
            long tps = clock.TicksPerSecond;
            using var udp = new UdpTransport(localPort, new IPEndPoint(IPAddress.Parse(host), port), JointDatagram.MaxBytes, clock);
            JointPredictorConfig config = SerialChainJointLink.PredictorConfig(profile, tps, 300, 2.5f);
            var presets = new[] { new JointPredictorPreset(predictor, predictor, config) };
            var sync = new ClockSyncConfig(historyCapacity: 32, smoothingAlpha: 0.2f, maxAcceptableRttTicks: 2 * tps, outlierRttMultiple: 3.0, minSamplesBeforeTrusted: 3);
            // A tool, not a headset, unless asked: RobotHost records only a headset's sessions automatically.
            var link = new SerialChainJointLink(udp, clock, profile, sync, presets, 0, asHeadset ? OperatorClientKind.Headset : OperatorClientKind.Tool) { MotionOn = true };
            var teleop = new SerialChainTeleop(profile, collision);
            int left = teleop.ArmIndex("left"), right = teleop.ArmIndex("right");

            // 1. Frames, a synced clock, the robot's pose.
            long start = clock.NowTicks;
            while (!(link.IsSynced && teleop.HasPose))
            {
                long now = clock.NowTicks;
                if (now - start > 5 * tps)
                {
                    Console.Error.WriteLine($"chain-teleop-probe: no synced joint-state stream from {host}:{port} in 5 s ({link.FramesReceived} frames). Is `just fr3-sim-host` running?");
                    return 2;
                }

                link.Tick(now);
                if (link.FramesReceived > 0) teleop.TryTakePose(link.Measured);
                Thread.Sleep(5);
            }

            Console.WriteLine($"[probe] {profile.Name}: synced after {(clock.NowTicks - start) * 1000 / tps} ms, rtt {link.RoundTripMilliseconds:0.0} ms; predictor {predictor}");

            // 2. Both hands grab, move for 2 s, hold for 2 s.
            RigidTransform hand = RigidTransform.Identity;
            teleop.Engage(left, hand);
            teleop.Engage(right, hand);
            var targets = new JointTarget[teleop.MotorCount];
            var buffer = new byte[JointDatagram.MaxBytes];
            var errors = new System.Collections.Generic.List<double>();
            var lastFrames = link.FramesReceived;
            var inOrder = new float[profile.MotorCount];
            var predicted = new float[profile.MotorCount];
            const int History = 256; // about 1.5 s of loop ticks: longer than any reading's age here
            var shownAt = Enumerable.Repeat(long.MinValue, History).ToArray();
            var shown = new float[History, profile.MotorCount];
            long shownCount = 0;
            uint sequence = 0;
            bool blocked = false;
            long begin = clock.NowTicks, lastSend = long.MinValue, lastStep = begin, sendInterval = tps / 48;
            while (clock.NowTicks - begin < 4 * tps)
            {
                long now = clock.NowTicks;
                link.Tick(now);
                if (link.FramesReceived != lastFrames && link.MeasuredOperatorTicks != long.MinValue)
                {
                    // Every new frame against what the twin showed for the instant it was read, from before it arrived.
                    lastFrames = link.FramesReceived;
                    link.ToProfileOrder(link.Measured, inOrder);
                    int best = -1;
                    long bestGap = tps / 100; // within 10 ms of a tick, or not scored
                    for (int h = 0; h < History; h++)
                    {
                        long gap = Math.Abs(shownAt[h] - link.MeasuredOperatorTicks);
                        if (shownAt[h] != long.MinValue && gap < bestGap) { best = h; bestGap = gap; }
                    }

                    if (best >= 0)
                    {
                        for (int m = 0; m < inOrder.Length; m++)
                        {
                            if (!IsHand(profile, m)) errors.Add(Math.Abs(shown[best, m] - inOrder[m]) * 180 / Math.PI);
                        }
                    }
                }

                // What the twin shows now, kept for the frames that will report this instant.
                ReadOnlySpan<JointSample> p = link.Predicted(now);
                if (!p.IsEmpty && link.ToProfileOrder(p, predicted) == predicted.Length)
                {
                    int slot = (int)(shownCount++ % History);
                    shownAt[slot] = now;
                    for (int m = 0; m < predicted.Length; m++) shown[slot, m] = predicted[m];
                }

                double s = Math.Min(1.0, (now - begin) / (2.0 * tps));
                RigidTransform turn = RigidTransform.Rotation(0, 0, 1, s * Math.PI / 12);
                teleop.Follow(left, new RigidTransform(reach * s, 0, 0, turn.Qw, turn.Qx, turn.Qy, turn.Qz));
                teleop.Follow(right, RigidTransform.Translation(0, 0, reach * s));
                teleop.SetOpening(left, 1 - s);
                teleop.Step((now - lastStep) / (double)tps); // the loop's real period, as the headset steps by its frame's
                lastStep = now;
                blocked |= teleop.Status(left) == ChainArmStatus.Blocked || teleop.Status(right) == ChainArmStatus.Blocked;
                if (lastSend == long.MinValue || now - lastSend >= sendInterval)
                {
                    int n = teleop.WriteTargets(targets);
                    if (JointCommandCodec.TryEncode(++sequence, now, targets.AsSpan(0, n), buffer, out int written))
                    {
                        udp.Send(buffer.AsSpan(0, written), now);
                        link.OnCommandSent(now, targets.AsSpan(0, n));
                    }

                    lastSend = now;
                }

                // About 500 Hz: Thread.Sleep would round to Windows' 15.6 ms timer and send at 32 Hz, not 48.
                while (clock.NowTicks - now < tps / 500) Thread.SpinWait(64);
            }

            // 3. Where the robot says each tool is, against where its hand asked.
            link.ToProfileOrder(link.Measured, inOrder);
            bool pass = !blocked;
            int k = 0;
            for (int a = 0; a < profile.Arms.Length; a++)
            {
                ChainArm arm = profile.Arms[a];
                double[] q = inOrder.Skip(k).Take(arm.Joints.Length).Select(v => (double)v).ToArray();
                RigidTransform measured = SerialChainKinematics.Tool(arm, q);
                RigidTransform target = teleop.Target(a);
                double mm = 1000 * Math.Sqrt(Sq(measured.X - target.X) + Sq(measured.Y - target.Y) + Sq(measured.Z - target.Z));
                double deg = 2 * Math.Acos(Math.Min(1, Math.Abs(measured.Qw * target.Qw + measured.Qx * target.Qx + measured.Qy * target.Qy + measured.Qz * target.Qz))) * 180 / Math.PI;
                float grip = arm.HasGripper ? inOrder[k + arm.Joints.Length] : float.NaN;
                bool ok = mm <= toleranceMm && deg <= 1.0;
                if (a == left && arm.HasGripper) ok &= grip < 0.05f;
                Console.WriteLine(FormattableString.Invariant($"[probe] {arm.Name}: measured tool {mm:0.00} mm / {deg:0.00} deg from its target, gripper {grip:0.00}, status {teleop.Status(a)} -> {(ok ? "ok" : "FAIL")}"));
                pass &= ok;
                k += arm.MotorCount;
            }

            double p95 = errors.Count > 0 ? errors.OrderBy(e => e).ElementAt((int)Math.Min(errors.Count - 1, Math.Ceiling(0.95 * errors.Count) - 1)) : double.NaN;
            bool predictionOk = errors.Count > 100 && p95 <= 1.0;
            Console.WriteLine(FormattableString.Invariant($"[probe] {predictor}: the twin at each reading's instant vs the reading, p95 {p95:0.000} deg over {errors.Count} joint readings -> {(predictionOk ? "ok" : "FAIL")}"));
            if (blocked) Console.WriteLine("[probe] FAIL: a step was blocked by the collision model; the scripted moves should be clear");
            pass &= predictionOk;

            teleop.Release(left);
            teleop.Release(right);
            Console.WriteLine(pass ? "[probe] PASS" : "[probe] FAIL");
            return pass ? 0 : 1;
        }

        private static bool IsHand(SerialChainProfile profile, int index)
        {
            int k = 0;
            foreach (ChainArm arm in profile.Arms)
            {
                k += arm.Joints.Length;
                if (arm.HasGripper && index == k) return true;
                if (arm.HasGripper) k++;
            }

            return false;
        }

        private static double Sq(double x) => x * x;
    }
}
