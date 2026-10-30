using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// <c>chain-drive</c>: drives a serial-chain robot's RobotHost (docs/adr/0027), such as the simulated
    /// FR3 Duo, through seeded joint-space waypoint moves at the headset's 48 Hz, with no headset. It is the
    /// FR3's counterpart of <c>joint-drive</c>:
    /// <list type="bullet">
    /// <item>the keepalive on the same socket brings the joint-state stream back to it;</item>
    /// <item>every command it sends goes to <c>operator.jlog</c>;</item>
    /// <item>Enter stops it, ending back at the pose it started from.</item>
    /// </list>
    ///
    /// <para><b>Simulated robots only, for now.</b> A real FR3 needs the collision model of docs/adr/0026
    /// decision 5 before anything drives it: the two arms' bases are 10 cm apart. So it refuses to run
    /// without <c>--simulated-robot</c>, which says the RobotHost it talks to runs <c>--backend sim</c>.</para>
    ///
    /// <para><b>The moves:</b> every arm joint wanders within ±<c>--amplitude-rad</c> of where it started,
    /// inside its soft limits less 0.1 rad, on legs of 0.5–1.5 s with linear motion between waypoints. Each
    /// hand opens or closes every few seconds. The robot's shaper turns the 48 Hz targets into its
    /// trajectory.</para>
    /// </summary>
    public static class ChainDriveCommand
    {
        private const string Usage =
            "usage: chain-drive --simulated-robot [--profile RobotProfiles/fr3-duo.json] [--remote-host 127.0.0.1] [--remote-port 6002]\n" +
            "                   [--seconds 60] [--seed 1] [--amplitude-rad 0.5] [--rate-hz 48] [--operator-log <path> [--session-id <id>]] [--dry-run]";

        public static int Run(string[] args)
        {
            string profilePath = "RobotProfiles/fr3-duo.json", host = "127.0.0.1";
            int port = 6002;
            double seconds = 60, amplitude = 0.5, rateHz = 48;
            ulong seed = 1;
            string? logPath = null, sessionId = null;
            bool simulated = false, dryRun = false;
            try
            {
                for (int i = 1; i < args.Length; i++)
                {
                    string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                    double Number() => double.Parse(Next(), NumberStyles.Float, CultureInfo.InvariantCulture);
                    switch (args[i])
                    {
                        case "--profile": profilePath = Next(); break;
                        case "--remote-host": host = Next(); break;
                        case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--seconds": seconds = Number(); break;
                        case "--seed": seed = ulong.Parse(Next(), CultureInfo.InvariantCulture); break;
                        case "--amplitude-rad": amplitude = Number(); break;
                        case "--rate-hz": rateHz = Number(); break;
                        case "--operator-log": logPath = Next(); break;
                        case "--session-id": sessionId = Next(); break;
                        case "--simulated-robot": simulated = true; break;
                        case "--dry-run": dryRun = true; break;
                        default: throw new ArgumentException($"unknown argument '{args[i]}'");
                    }
                }

                if (!(seconds > 0)) throw new ArgumentException("--seconds must be positive");
                if (!(amplitude > 0 && amplitude <= 1.5)) throw new ArgumentException("--amplitude-rad must be in (0, 1.5]");
                if (!(rateHz >= 10 && rateHz <= 100)) throw new ArgumentException("--rate-hz must be in [10, 100]");
                if (!IPAddress.TryParse(host, out _)) throw new ArgumentException($"--remote-host '{host}' is not an IP address");
            }
            catch (Exception e) when (e is ArgumentException || e is FormatException || e is OverflowException)
            {
                Console.Error.WriteLine($"chain-drive: {e.Message}");
                Console.Error.WriteLine(Usage);
                return 64;
            }

            if (!simulated && !dryRun)
            {
                Console.Error.WriteLine("chain-drive: only a simulated robot (RobotHost --backend sim) may be driven until the serial-chain collision " +
                    "model exists (docs/adr/0026 decision 5). Pass --simulated-robot to say that is what this talks to.");
                return 64;
            }

            SerialChainProfile profile;
            try
            {
                profile = ImportUrdf.SerialChainProfileJson.Load(profilePath);
            }
            catch (Exception e) when (e is IOException || e is InvalidDataException || e is System.Text.Json.JsonException)
            {
                Console.Error.WriteLine($"chain-drive: {e.Message}");
                return 1;
            }

            var plan = new ChainDrivePlan(profile, amplitude, seed);
            Console.WriteLine($"[chain-drive] {profile.Name}: {plan.Motors} motors; joints wander +-{amplitude:0.##} rad from their start, legs 0.5-1.5 s, hands toggle; {seconds:0} s at {rateHz:0} Hz");
            if (dryRun)
            {
                plan.Start(SerialChainStartGuess(profile));
                for (double t = 0; t <= Math.Min(seconds, 5); t += 1)
                {
                    Console.WriteLine(FormattableString.Invariant($"  {t,3:0} s: ") + string.Join(" ", plan.Sample(t).Select(s => FormattableString.Invariant($"{s.MotorId}:{s.Angle:0.000}"))));
                }

                return 0;
            }

            return Drive(profile, plan, new IPEndPoint(IPAddress.Parse(host), port), seconds, rateHz, logPath, sessionId ?? "chain-drive");
        }

        private static int Drive(SerialChainProfile profile, ChainDrivePlan plan, IPEndPoint robot, double seconds, double rateHz, string? logPath, string sessionId)
        {
            long tps = Stopwatch.Frequency;
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            socket.Blocking = false;
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

            var buffer = new byte[JointDatagram.MaxBytes];
            var joints = new JointSample[JointStateCodec.MaxJointsPerMessage];
            var measured = new double[plan.Motors];
            var sensed = new bool[plan.Motors];
            long frames = 0, nextKeepalive = 0;

            void Pump(long now)
            {
                if (now >= nextKeepalive)
                {
                    if (JointStateCodec.TryEncodeSubscribe(now, tps, buffer, out int n)) socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                    nextKeepalive = now + tps / 2;
                }

                while (socket.Available > 0)
                {
                    EndPoint from = new IPEndPoint(IPAddress.Any, 0);
                    int got;
                    try
                    {
                        got = socket.ReceiveFrom(buffer, ref from);
                    }
                    catch (SocketException)
                    {
                        break;
                    }

                    if (!JointStateCodec.TryDecode(buffer.AsSpan(0, got), out _, out _, out _, out _, out _, out _, joints, out int count)) continue;
                    frames++;
                    for (int i = 0; i < count; i++)
                    {
                        int m = plan.IndexOf(joints[i].MotorId);
                        if (m < 0) continue;
                        measured[m] = joints[i].Value;
                        sensed[m] |= joints[i].Sensed;
                    }
                }
            }

            Console.WriteLine($"[chain-drive] subscribing to {robot}'s joint-state stream; waiting for every motor");
            long waitUntil = Stopwatch.GetTimestamp() + 5 * tps;
            while (sensed.Any(s => !s))
            {
                if (stop.IsSet) return 130;
                if (Stopwatch.GetTimestamp() > waitUntil)
                {
                    Console.Error.WriteLine("chain-drive: no frame with every motor in 5 s (is RobotHost running with this profile? just fr3-sim-host). Nothing was sent.");
                    return 1;
                }

                Pump(Stopwatch.GetTimestamp());
                Thread.Sleep(5);
            }

            double[] start = (double[])measured.Clone();
            plan.Start(start);
            using OperatorLog? log = logPath != null ? new OperatorLog(File.Create(logPath), tps, sessionId) : null;
            long interval = (long)(tps / rateHz), t0 = Stopwatch.GetTimestamp(), nextSend = t0, nextReport = t0 + 10 * tps;
            uint sequence = 0;
            double maxGap = 0;
            JointTarget[] last = plan.Sample(0);

            void Send(long now, JointTarget[] targets)
            {
                if (!JointCommandCodec.TryEncode(sequence, now, targets, buffer, out int written)) return;
                socket.SendTo(buffer, 0, written, SocketFlags.None, robot);
                log?.Command(now, sequence, now, targets);
                sequence++;
            }

            Console.WriteLine("[chain-drive] driving. Press Enter to stop.");
            while (!stop.IsSet)
            {
                long now = Stopwatch.GetTimestamp();
                double t = (now - t0) / (double)tps;
                if (t > seconds) break;
                Pump(now);
                for (int m = 0; m < plan.Motors; m++)
                {
                    if (!plan.IsHand(m)) maxGap = Math.Max(maxGap, Math.Abs(measured[m] - last[m].Angle));
                }

                if (now >= nextSend)
                {
                    last = plan.Sample(t);
                    Send(now, last);
                    nextSend += interval;
                    if (nextSend < now) nextSend = now + interval;
                }

                if (now >= nextReport)
                {
                    Console.WriteLine(FormattableString.Invariant($"[chain-drive] {t,5:0} s, {frames} frames, largest gap commanded-measured {maxGap * 180 / Math.PI:0.0} deg"));
                    nextReport += 10 * tps;
                }

                Thread.Sleep(1);
            }

            // Back to the start, then hold it for a moment.
            var home = plan.Home();
            for (int i = 0; i < 100; i++)
            {
                Send(Stopwatch.GetTimestamp(), home);
                Pump(Stopwatch.GetTimestamp());
                Thread.Sleep(20);
            }

            log?.Close();
            Console.WriteLine(FormattableString.Invariant(
                $"[chain-drive] done: {sequence} commands, {frames} frames, largest gap commanded-measured {maxGap * 180 / Math.PI:0.0} deg; back at the start pose") +
                (logPath != null ? $"; operator log {logPath}" : ""));
            return stop.IsSet ? 130 : 0;
        }

        /// <summary>For a dry run with no robot: each joint's soft-range middle, hands open.</summary>
        private static double[] SerialChainStartGuess(SerialChainProfile profile)
        {
            var start = new System.Collections.Generic.List<double>();
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint joint in arm.Joints) start.Add(0.5 * (joint.Limits.SoftLower + joint.Limits.SoftUpper));
                if (arm.HasGripper) start.Add(1.0);
            }

            return start.ToArray();
        }
    }

    /// <summary>chain-drive's seeded joint-space moves, one waypoint leg per joint at a time.</summary>
    internal sealed class ChainDrivePlan
    {
        private readonly byte[] _ids;
        private readonly bool[] _hand;
        private readonly double[] _softLower, _softUpper;
        private readonly double _amplitude;
        private SeededRng _rng;
        private double[] _start = Array.Empty<double>();
        private readonly double[] _from, _to, _legStart, _legEnd, _lo, _hi;

        public ChainDrivePlan(SerialChainProfile profile, double amplitude, ulong seed)
        {
            var ids = new System.Collections.Generic.List<byte>();
            var hand = new System.Collections.Generic.List<bool>();
            var lower = new System.Collections.Generic.List<double>();
            var upper = new System.Collections.Generic.List<double>();
            foreach (ChainArm arm in profile.Arms)
            {
                foreach (ChainJoint joint in arm.Joints)
                {
                    ids.Add(joint.MotorId);
                    hand.Add(false);
                    lower.Add(joint.Limits.SoftLower);
                    upper.Add(joint.Limits.SoftUpper);
                }

                if (arm.HasGripper)
                {
                    ids.Add(arm.Gripper.MotorId);
                    hand.Add(true);
                    lower.Add(0);
                    upper.Add(1);
                }
            }

            _ids = ids.ToArray();
            _hand = hand.ToArray();
            _softLower = lower.ToArray();
            _softUpper = upper.ToArray();
            _amplitude = amplitude;
            _rng = new SeededRng(seed);
            int n = _ids.Length;
            _from = new double[n];
            _to = new double[n];
            _legStart = new double[n];
            _legEnd = new double[n];
            _lo = new double[n];
            _hi = new double[n];
        }

        public int Motors => _ids.Length;

        public bool IsHand(int m) => _hand[m];

        public int IndexOf(byte motorId) => Array.IndexOf(_ids, motorId);

        /// <summary>Starts the plan from the measured pose, one value per motor in profile order.</summary>
        public void Start(double[] measured)
        {
            _start = (double[])measured.Clone();
            for (int m = 0; m < _ids.Length; m++)
            {
                _lo[m] = Math.Max(_softLower[m] + (_hand[m] ? 0 : LimitMarginOf), measured[m] - _amplitude);
                _hi[m] = Math.Min(_softUpper[m] - (_hand[m] ? 0 : LimitMarginOf), measured[m] + _amplitude);
                if (_lo[m] > _hi[m]) _lo[m] = _hi[m] = Math.Min(Math.Max(measured[m], _softLower[m]), _softUpper[m]);
                _to[m] = Math.Min(Math.Max(measured[m], _lo[m]), _hi[m]);
                NewLeg(m, 0);
            }
        }

        private const double LimitMarginOf = 0.1;

        /// <summary>The targets at plan time <paramref name="t"/> (s).</summary>
        public JointTarget[] Sample(double t)
        {
            var targets = new JointTarget[_ids.Length];
            for (int m = 0; m < _ids.Length; m++)
            {
                if (t >= _legEnd[m]) NewLeg(m, t);
                double value;
                if (_hand[m])
                {
                    value = _to[m];
                }
                else
                {
                    double u = (t - _legStart[m]) / (_legEnd[m] - _legStart[m]);
                    value = _from[m] + (_to[m] - _from[m]) * u;
                }

                targets[m] = new JointTarget(_ids[m], (float)value, 0f);
            }

            return targets;
        }

        /// <summary>The start pose as targets.</summary>
        public JointTarget[] Home() => _ids.Select((id, m) => new JointTarget(id, (float)_start[m], 0f)).ToArray();

        private void NewLeg(int m, double t)
        {
            _from[m] = _to[m];
            if (_hand[m])
            {
                _to[m] = _to[m] > 0.5 ? 0 : 1;
                _legEnd[m] = t + 2 + 4 * _rng.NextDouble();
            }
            else
            {
                _to[m] = _lo[m] + (_hi[m] - _lo[m]) * _rng.NextDouble();
                _legEnd[m] = t + 0.5 + _rng.NextDouble();
            }

            _legStart[m] = t;
        }
    }
}
