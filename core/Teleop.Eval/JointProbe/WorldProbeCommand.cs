using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointProbe
{
    /// <summary>
    /// <c>world-probe</c> (<c>just fr3-world-probe</c>, docs/adr/0031 amendment 2): subscribes to a RobotHost's joint port as the
    /// headset does and checks the world it streams. The status must arrive and name this scene file (its sha256), every object
    /// must be posed within 2 s, and world frames must come at 50 ± 5 Hz. With <c>--seed</c> the poses must be that seed's
    /// layout (<see cref="SceneLayout"/>); <c>--new-layout</c> then asks for the next one (motion is off: the probe sends no
    /// command) and checks it arrives. <c>--move id:x:y:yawDeg[:expected]</c> then asks to move one object (phase P5's edit,
    /// as the headset's EDIT OBJECTS sends it) and checks the status echoes it with the expected outcome (<c>applied</c> by
    /// default, or a refusal such as <c>overlaps-arm</c>), and, when applied, that the world frames show the object there.
    /// Sends only the keepalive and those edits, never a joint command, so no arm moves. Exits 0 only when every check passes.
    /// </summary>
    public static class WorldProbeCommand
    {
        public static int Run(string[] args)
        {
            string? host = null, scenePath = null;
            int port = 6002;
            double seconds = 3;
            ulong? seed = null;
            bool newLayout = false;
            string? moveSpec = null;
            for (int i = 1; i < args.Length; i++)
            {
                string Next() => i + 1 < args.Length ? args[++i] : throw new ArgumentException($"{args[i]} needs a value");
                switch (args[i])
                {
                    case "--remote-host": host = Next(); break;
                    case "--remote-port": port = int.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--seconds": seconds = double.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--scene": scenePath = Next(); break;
                    case "--seed": seed = ulong.Parse(Next(), CultureInfo.InvariantCulture); break;
                    case "--new-layout": newLayout = true; break;
                    case "--move": moveSpec = Next(); break;
                    default:
                        Console.Error.WriteLine("usage: world-probe --remote-host <ip> --scene <file> [--remote-port 6002] [--seconds 3] [--seed <n> [--new-layout]] [--move id:x:y:yawDeg[:expected]]");
                        return 64;
                }
            }

            if (host == null || !IPAddress.TryParse(host, out IPAddress? address) || scenePath == null)
            {
                Console.Error.WriteLine("world-probe: --remote-host <ip> and --scene <file> are required");
                return 64;
            }

            byte[] sceneBytes = File.ReadAllBytes(scenePath);
            WorldScene scene = JsonSerializer.Deserialize<SceneDocument>(sceneBytes, new JsonSerializerOptions { IncludeFields = true })!.ToScene();
            byte[] sha = SHA256.HashData(sceneBytes);
            var robot = new IPEndPoint(address, port);
            using var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            socket.Bind(new IPEndPoint(IPAddress.Any, 0));
            socket.ReceiveTimeout = 20;

            var failures = new List<string>();
            int phases = newLayout && seed.HasValue ? 2 : 1;
            for (int phase = 0; phase < phases; phase++)
            {
                ulong? expect = seed.HasValue ? seed.Value + (ulong)phase : (ulong?)null;
                if (phase == 1)
                {
                    var edit = new byte[WorldCodec.EditBytes];
                    WorldCodec.TryEncodeEdit(WorldEditOp.NewLayout, 0, Sequence(), Stopwatch.GetTimestamp(), default, edit, out int n);
                    for (int k = 0; k < 4; k++) socket.SendTo(edit, 0, n, SocketFlags.None, robot); // re-sent: applied once
                }

                // --seconds 0: no watch of the stream (another joint client is subscribed, so frames come and go); the move only.
                if (seconds > 0 || phase == 1) Watch(socket, robot, scene, sha, Math.Max(seconds, 2), expect, phase == 1 ? 2u : (uint?)null, failures, phase == 1 ? "after NEW LAYOUT" : "");
            }

            if (moveSpec != null) Move(socket, robot, scene, moveSpec, failures);

            if (failures.Count == 0)
            {
                Console.WriteLine("[world-probe] PASS");
                return 0;
            }

            foreach (string f in failures) Console.WriteLine("[world-probe] FAIL: " + f);
            return 1;
        }

        /// <summary>An edit sequence no earlier run against the same RobotHost used (it applies a re-sent sequence once).</summary>
        private static uint Sequence() => (uint)(DateTime.UtcNow.Ticks / 10_000 % uint.MaxValue) | 1u;

        private static readonly string[] Outcomes =
            { "applied", "overlaps-arm", "overlaps-object", "outside-region", "unknown-object", "not-movable", "arm-touching", "busy", "invalid-pose", "timed-out" };

        private static void Move(Socket socket, IPEndPoint robot, WorldScene scene, string spec, List<string> failures)
        {
            string[] f = spec.Split(':');
            int expected = f.Length > 4 ? Array.IndexOf(Outcomes, f[4]) : 0;
            if (f.Length < 4 || f.Length > 5 || expected < 0 || !byte.TryParse(f[0], out byte id))
            {
                failures.Add($"--move '{spec}': expected id:x:y:yawDeg[:{string.Join("|", Outcomes)}]");
                return;
            }

            double x = double.Parse(f[1], CultureInfo.InvariantCulture), y = double.Parse(f[2], CultureInfo.InvariantCulture);
            double yaw = double.Parse(f[3], CultureInfo.InvariantCulture) * Math.PI / 180;
            var pose = new ObjectPose(id, 0, (float)x, (float)y, 0.1f, (float)Math.Cos(yaw / 2), 0, 0, (float)Math.Sin(yaw / 2));
            uint sequence = Sequence() + 2;
            var edit = new byte[WorldCodec.EditBytes];
            WorldCodec.TryEncodeEdit(WorldEditOp.Move, id, sequence, Stopwatch.GetTimestamp(), pose, edit, out int n);
            var buffer = new byte[JointDatagram.MaxBytes];
            var part = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
            var statusSha = new byte[WorldCodec.Sha256Bytes];
            WorldStatus? echoed = null;
            ObjectPose? after = null;
            long freq = Stopwatch.Frequency, start = Stopwatch.GetTimestamp(), end = start + 3 * freq, nextSend = 0;
            while (Stopwatch.GetTimestamp() < end)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= nextSend)
                {
                    JointStateCodec.TryEncodeSubscribe(now, freq, buffer, out int k);
                    socket.SendTo(buffer, 0, k, SocketFlags.None, robot);
                    if (!echoed.HasValue) socket.SendTo(edit, 0, n, SocketFlags.None, robot); // re-sent until echoed, as the headset does
                    nextSend = now + freq / 4;
                }

                int got;
                try
                {
                    got = socket.Receive(buffer);
                }
                catch (SocketException)
                {
                    continue;
                }

                ReadOnlySpan<byte> d = buffer.AsSpan(0, got);
                if (WorldCodec.TryDecodeStatus(d, out WorldStatus s, statusSha, out _) && s.EditSequence == sequence) echoed = s;
                else if (echoed.HasValue && WorldCodec.TryDecodeFrame(d, out _, out _, out _, out _, part, out int count))
                {
                    for (int i = 0; i < count; i++) if (part[i].Id == id) after = part[i];
                }
            }

            string label = $"move {id} to ({x:F3}, {y:F3}, {f[3]}°)";
            if (!echoed.HasValue)
            {
                failures.Add(label + ": RobotHost never echoed it");
                return;
            }

            byte outcome = echoed.Value.EditOutcome;
            Console.WriteLine($"[world-probe] {label}: {(outcome < Outcomes.Length ? Outcomes[outcome] : outcome.ToString(CultureInfo.InvariantCulture))}");
            if (outcome != expected) failures.Add($"{label}: {(outcome < Outcomes.Length ? Outcomes[outcome] : "?")}, expected {Outcomes[expected]}");
            if (expected != 0) return;
            if (!after.HasValue)
            {
                failures.Add(label + ": no world frame showed the object after it");
                return;
            }

            ObjectPose a = after.Value;
            double gotYaw = Math.Atan2(2.0 * (a.Qw * a.Qz + a.Qx * a.Qy), 1.0 - 2.0 * (a.Qy * a.Qy + a.Qz * a.Qz));
            double off = Math.Max(Math.Abs(a.X - x), Math.Abs(a.Y - y)), turn = Math.Abs(Math.IEEERemainder(gotYaw - yaw, 2 * Math.PI));
            Console.WriteLine($"[world-probe] {label}: the frames show it {off * 1000:F3} mm and {turn * 180 / Math.PI:F3}° from there");
            if (off > 1e-4 || turn > 0.01) failures.Add($"{label}: the frames show it {off * 1000:F3} mm and {turn * 180 / Math.PI:F3}° away");
        }

        private static void Watch(Socket socket, IPEndPoint robot, WorldScene scene, byte[] sha, double seconds, ulong? seed, uint? layout,
            List<string> failures, string label)
        {
            var buffer = new byte[JointDatagram.MaxBytes];
            var part = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
            var latest = new Dictionary<byte, ObjectPose>();
            var statusSha = new byte[WorldCodec.Sha256Bytes];
            WorldStatus? status = null;
            long f = Stopwatch.Frequency, start = Stopwatch.GetTimestamp(), end = start + (long)(seconds * f), nextKeepalive = 0, allPosedAt = 0;
            var snapshots = new HashSet<uint>();
            long firstFrame = 0, lastFrame = 0;
            while (Stopwatch.GetTimestamp() < end)
            {
                long now = Stopwatch.GetTimestamp();
                if (now >= nextKeepalive)
                {
                    JointStateCodec.TryEncodeSubscribe(now, f, buffer, out int n);
                    socket.SendTo(buffer, 0, n, SocketFlags.None, robot);
                    nextKeepalive = now + f / 2;
                }

                int got;
                try
                {
                    got = socket.Receive(buffer);
                }
                catch (SocketException)
                {
                    continue;
                }

                ReadOnlySpan<byte> d = buffer.AsSpan(0, got);
                if (WorldCodec.TryDecodeStatus(d, out WorldStatus s, statusSha, out _))
                {
                    status = s;
                }
                else if (WorldCodec.TryDecodeFrame(d, out uint snapshot, out _, out _, out _, part, out int count))
                {
                    if (layout.HasValue && (!status.HasValue || status.Value.LayoutNumber < layout.Value)) continue; // the old layout's
                    for (int i = 0; i < count; i++) latest[part[i].Id] = part[i];
                    if (snapshots.Add(snapshot))
                    {
                        if (firstFrame == 0) firstFrame = now;
                        lastFrame = now;
                    }

                    if (allPosedAt == 0 && scene.Objects.All(o => latest.ContainsKey(o.Id))) allPosedAt = now;
                }
            }

            string at = label.Length > 0 ? " (" + label + ")" : "";
            if (!status.HasValue)
            {
                failures.Add("no world status" + at + ": is RobotHost running with --scene?");
                return;
            }

            WorldStatus st = status.Value;
            Console.WriteLine($"[world-probe]{at} status: running {st.Running}, physics {st.Physics}, edits {(st.EditsAllowed ? "allowed" : "refused")}, " +
                $"{st.ObjectCount} objects, layout {st.LayoutNumber} (seed {st.LayoutSeed}), last edit {st.EditSequence} -> {st.EditOutcome}");
            if (!statusSha.SequenceEqual(sha)) failures.Add("RobotHost runs another scene file than " + scene.Name + at);
            if (!st.Running) failures.Add("the world is not running" + at);
            if (allPosedAt == 0) failures.Add($"only {latest.Count} of {scene.Objects.Length} objects posed" + at);
            else if ((allPosedAt - start) / (double)Stopwatch.Frequency > 2.0) failures.Add("the objects took more than 2 s to be posed" + at);
            double rate = snapshots.Count > 1 ? (snapshots.Count - 1) / ((lastFrame - firstFrame) / (double)Stopwatch.Frequency) : 0;
            Console.WriteLine($"[world-probe]{at} {snapshots.Count} snapshot(s) at {rate:F1} Hz; {latest.Count} object(s) posed");
            if (rate < 45 || rate > 55) failures.Add($"world frames at {rate:F1} Hz, not 50 ± 5" + at);
            if (layout.HasValue && st.LayoutNumber != layout.Value) failures.Add($"layout {st.LayoutNumber}, expected {layout.Value}" + at);
            if (seed.HasValue && allPosedAt != 0)
            {
                if (st.LayoutSeed != seed.Value) failures.Add($"layout seed {st.LayoutSeed}, expected {seed.Value}" + at);
                var expected = new RigidTransform[scene.Objects.Length];
                SceneLayout.Draw(scene, seed.Value, expected);
                double worst = scene.Objects.Select((o, i) => Math.Max(Math.Abs(latest[o.Id].X - expected[i].X), Math.Abs(latest[o.Id].Y - expected[i].Y))).Max();
                Console.WriteLine($"[world-probe]{at} poses against seed {seed.Value}'s layout: worst {worst * 1000:F4} mm");
                if (worst > 1e-5) failures.Add($"the poses are {worst * 1000:F3} mm from seed {seed.Value}'s layout" + at);
            }
        }
    }
}
