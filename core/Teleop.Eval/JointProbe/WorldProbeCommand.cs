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
    /// command) and checks it arrives. Sends only the keepalive and that edit, never a joint command, so no arm moves. Exits 0
    /// only when every check passes.
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
                    default:
                        Console.Error.WriteLine("usage: world-probe --remote-host <ip> --scene <file> [--remote-port 6002] [--seconds 3] [--seed <n> [--new-layout]]");
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
                    WorldCodec.TryEncodeEdit(WorldEditOp.NewLayout, 0, 1, Stopwatch.GetTimestamp(), default, edit, out int n);
                    for (int k = 0; k < 4; k++) socket.SendTo(edit, 0, n, SocketFlags.None, robot); // re-sent: applied once
                }

                Watch(socket, robot, scene, sha, seconds, expect, phase == 1 ? 2u : (uint?)null, failures, phase == 1 ? "after NEW LAYOUT" : "");
            }

            if (failures.Count == 0)
            {
                Console.WriteLine("[world-probe] PASS");
                return 0;
            }

            foreach (string f in failures) Console.WriteLine("[world-probe] FAIL: " + f);
            return 1;
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
