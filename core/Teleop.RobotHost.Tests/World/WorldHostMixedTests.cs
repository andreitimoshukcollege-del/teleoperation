using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Tests.Backend;
using Teleop.RobotHost.World;

namespace Teleop.RobotHost.Tests.World
{
    /// <summary>
    /// RobotHost's world for a mixed pool (docs/adr/0038 decision 3, <c>core/Scenes/jetrover-mixed.json</c> and its held-out family):
    /// every placement a layout draws is applied, the objects raised on a stand or a plank's supports after what they rest on, and a
    /// refused part of a stand or a rig parks the whole of it. The placements of the first layouts are what Isaac's model must apply
    /// too (robot/isaac's test_world.py, from the golden file this writes).
    /// </summary>
    public class WorldHostMixedTests
    {
        private const long Tps = 1_000_000;
        private const long Ms = Tps / 1000;

        private static byte[] Bytes(string name) => File.ReadAllBytes(Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "..", "Scenes", name + ".json"));

        private static WorldScene Load(byte[] bytes) => JsonSerializer.Deserialize<SceneDocument>(bytes, new JsonSerializerOptions { IncludeFields = true })!.ToScene();

        /// <summary>StaticWorld, keeping every request in the order it came, and refusing every layout placement of one object.</summary>
        private sealed class Recording : IWorldBackend
        {
            private readonly StaticWorld _inner;
            private readonly byte[] _sha;
            private readonly Queue<(uint, PlaceOutcome, ObjectPose)> _refused = new Queue<(uint, PlaceOutcome, ObjectPose)>();
            public readonly List<(string Kind, ObjectPose Pose, float Scale)> Requests = new List<(string, ObjectPose, float)>();
            public byte RefuseId;

            public Recording(WorldScene scene, byte[] sha)
            {
                _inner = new StaticWorld(scene, sha, Tps);
                _sha = sha;
            }

            public bool HasWorld => true;
            public ReadOnlySpan<byte> SceneSha256 => _sha;
            public bool Physics => false;
            public void Poll(long now) => _inner.Poll(now);

            public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
            {
                Requests.Add((cause == PlaceCause.Park ? "park" : cause == PlaceCause.Layout ? "layout" : cause.ToString().ToLowerInvariant(), pose, 1f));
                if (pose.Id == RefuseId && cause == PlaceCause.Layout) _refused.Enqueue((request, PlaceOutcome.OverlapsArm, pose));
                else _inner.Place(request, cause, pose, hostTicks);
            }

            public void Resize(uint request, byte id, float scale, long hostTicks)
            {
                Requests.Add(("resize", new ObjectPose(id, 0, 0, 0, 0, 1, 0, 0, 0), scale));
                _inner.Resize(request, id, scale, hostTicks);
            }

            public bool TryTakeSizes(Span<byte> ids, Span<float> scales, out int count, out uint revision) => _inner.TryTakeSizes(ids, scales, out count, out revision);

            public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision) =>
                _inner.TryTakeSnapshot(poses, out count, out sensedTicks, out revision);

            public bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose)
            {
                if (_refused.Count > 0)
                {
                    (request, outcome, pose) = _refused.Dequeue();
                    return true;
                }

                return _inner.TryTakePlaced(out request, out outcome, out pose);
            }
        }

        private sealed class Rig
        {
            public readonly List<string> Records = new List<string>();
            public readonly WorldScene Scene;
            public readonly WorldHost Host;
            public readonly Recording Backend;
            public long Now;

            public Rig(string name, ulong seed, byte refuse = 0)
            {
                byte[] bytes = Bytes(name);
                Scene = Load(bytes);
                Backend = new Recording(Scene, SHA256.HashData(bytes)) { RefuseId = refuse };
                Host = new WorldHost(Scene, SHA256.HashData(bytes), Backend, Tps, seed, Records.Add);
            }

            public void Settle()
            {
                for (int i = 0; i < 40; i++)
                {
                    Now += Ms;
                    Backend.Poll(Now);
                    Host.Tick(Now, false, true, (b, n) => { });
                }
            }

            public void NewLayout(uint sequence)
            {
                Host.HandleEdit(WorldEditOp.NewLayout, sequence, default, true, Now);
                Settle();
            }

            /// <summary>The newest WO record's poses, by id.</summary>
            public Dictionary<byte, double[]> LastObjects()
            {
                string wo = Records.Last(r => r.StartsWith("WO|"));
                var byId = new Dictionary<byte, double[]>();
                foreach (string part in wo.Split('|').Last().Split(';'))
                {
                    string[] f = part.Split(':');
                    byId[byte.Parse(f[0], CultureInfo.InvariantCulture)] = f.Skip(1).Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray();
                }

                return byId;
            }
        }

        public static IEnumerable<object[]> Scenes() => new[] { new object[] { "jetrover-mixed" }, new object[] { "jetrover-mixed-holdout" } };

        [Theory]
        [MemberData(nameof(Scenes))]
        public void EveryPlacementALayoutDraws_IsApplied_TheRaisedOnesAfterWhatTheyRestOn(string name)
        {
            var rig = new Rig(name, seed: 1);
            rig.Settle();
            for (uint layout = 2; layout <= 60; layout++) rig.NewLayout(layout);
            // StaticWorld takes the rules Isaac takes but for the arm's overlap: no placement refused, none redrawn, nothing parked for it.
            string[] refused = rig.Records.Where(r => r.StartsWith("WP|") && !r.EndsWith("|applied")).ToArray();
            Assert.True(refused.Length == 0, string.Join("\n", refused.Take(5)));
            Assert.Equal(60u, rig.Host.LayoutNumber);

            // In each layout (a new one parks every object first), a raised object's placement comes after its stand's, a plank's after
            // its supports'. Each object's size is the last resize sent for it.
            WorldScene s = rig.Scene;
            int raisedSeen = 0;
            var placed = new HashSet<byte>();
            var scale = new Dictionary<byte, double>();
            foreach ((string kind, ObjectPose pose, float factor) in rig.Backend.Requests)
            {
                if (kind == "park" && pose.Id == s.Objects[0].Id) placed.Clear();
                if (kind == "resize") scale[pose.Id] = factor;
                if (kind != "layout") continue;
                int i = s.IndexOf(pose.Id);
                placed.Add(pose.Id);
                int rest = WorldScene.RestOf(pose.Qw, pose.Qx, pose.Qy, pose.Qz, out _);
                if (!s.IsElevated(i, pose.Z, scale.TryGetValue(pose.Id, out double f) ? f : 1.0, rest)) continue;
                raisedSeen++;
                if (s.Objects[i].Role == SceneRole.Plank) Assert.All(s.Objects[i].SupportIds, id => Assert.Contains(id, placed));
                else Assert.Contains(placed, id => s.Objects[s.IndexOf(id)].Role == SceneRole.Stand);
            }

            Assert.True(raisedSeen > 5, $"{raisedSeen} raised placements in 60 layouts");
        }

        [Fact]
        public void AStandsObject_AndAPlank_RestAtTheirHeights_InTheRecords()
        {
            for (ulong seed = 1; seed <= 60; seed++)
            {
                var rig = new Rig("jetrover-mixed", seed);
                rig.Settle();
                WorldScene s = rig.Scene;
                var expected = new RigidTransform[s.Objects.Length];
                var parked = new bool[s.Objects.Length];
                var scales = new double[s.Objects.Length];
                SceneLayout.Draw(s, seed, expected, parked, scales);
                Dictionary<byte, double[]> seen = rig.LastObjects();
                for (int i = 0; i < s.Objects.Length; i++)
                {
                    Assert.Equal(parked[i], rig.Host.IsParked(i));
                    if (parked[i]) continue;
                    // Where the layout drew it, at its height (float32 on the wire), raised ones too.
                    Assert.True(Math.Abs(seen[s.Objects[i].Id][2] - expected[i].Z) < 1e-4, $"seed {seed}: {s.Objects[i].Name} at z {seen[s.Objects[i].Id][2]}, not {expected[i].Z}");
                    Assert.Equal(expected[i].X, seen[s.Objects[i].Id][0], 4);
                    Assert.Equal(scales[i], rig.Host.ScaleOf(i), 4);
                }

                string wl = rig.Records.Last(r => r.StartsWith("WL|"));
                Assert.EndsWith("|" + string.Join(";", s.Objects.Where((o, i) => !parked[i]).Select(o => o.Id)), wl);
            }
        }

        [Fact]
        public void ARefusedSupport_ParksItsPlankAndItsOtherSupport_ARefusedStandWhatItHolds()
        {
            WorldScene s = Load(Bytes("jetrover-mixed"));
            (ulong seed, int index) Find(Func<int, bool[], RigidTransform[], bool> wanted)
            {
                for (ulong seed = 1; seed < 500; seed++)
                {
                    var poses = new RigidTransform[s.Objects.Length];
                    var parked = new bool[s.Objects.Length];
                    var scales = new double[s.Objects.Length];
                    SceneLayout.Draw(s, seed, poses, parked, scales);
                    for (int i = 0; i < poses.Length; i++)
                    {
                        if (!parked[i] && wanted(i, parked, poses)) return (seed, i);
                    }
                }

                throw new Xunit.Sdk.XunitException("no such layout");
            }

            // A bridge's support refused: the plank and both supports leave the layout; WL is written again without them.
            (ulong rigSeed, int support) = Find((i, parked, poses) => s.Objects[i].Role == SceneRole.Support &&
                s.Objects.Any(q => q.Role == SceneRole.Plank && q.SupportIds.Length == 2 && Array.IndexOf(q.SupportIds, s.Objects[i].Id) >= 0));
            var rig = new Rig("jetrover-mixed", rigSeed, s.Objects[support].Id);
            rig.Settle();
            SceneObject plank = s.Objects.First(q => q.Role == SceneRole.Plank && Array.IndexOf(q.SupportIds, s.Objects[support].Id) >= 0);
            Assert.True(rig.Host.IsParked(Array.IndexOf(s.Objects, plank)));
            Assert.All(plank.SupportIds, id => Assert.True(rig.Host.IsParked(s.IndexOf(id))));
            Assert.DoesNotContain(plank.Id.ToString(CultureInfo.InvariantCulture), rig.Records.Last(r => r.StartsWith("WL|")).Split('|').Last().Split(';'));
            Assert.Equal(1, rig.Records.Count(r => r.StartsWith("WP|") && !r.EndsWith("|applied"))); // no redraw of a part

            // A stand refused: it and the object on it leave; anything else stays.
            (ulong standSeed, int stand) = Find((i, parked, poses) => s.Objects[i].Role == SceneRole.Stand);
            rig = new Rig("jetrover-mixed", standSeed, s.Objects[stand].Id);
            rig.Settle();
            Assert.True(rig.Host.IsParked(stand));
            var drawn = new RigidTransform[s.Objects.Length];
            var off = new bool[s.Objects.Length];
            var sizes = new double[s.Objects.Length];
            SceneLayout.Draw(s, standSeed, drawn, off, sizes);
            int held = Enumerable.Range(0, s.Objects.Length).Single(k => !off[k] && s.HoldsOnTop(stand, drawn[stand].X, drawn[stand].Y, drawn[stand].Z, Yaw(drawn[stand]), sizes[stand], 0,
                k, drawn[k].X, drawn[k].Y, drawn[k].Z, sizes[k], WorldScene.RestOf(drawn[k])));
            Assert.True(rig.Host.IsParked(held));
            Assert.Equal(off.Count(p => !p) - 2, Enumerable.Range(0, s.Objects.Length).Count(k => !rig.Host.IsParked(k)));
        }

        private static double Yaw(in RigidTransform p)
        {
            WorldScene.RestOf(p, out double yaw);
            return yaw;
        }

        [Fact]
        public void TheLayoutsGoldenFile_IsWhatWorldHostSends()
        {
            // robot/isaac/tests/test_world.py replays these through Isaac's WorldModel, which must apply every one: what RobotHost
            // draws, the bridge takes (docs/adr/0038: no outside-region, no overlaps-object).
            string expected = LayoutsGoldenJson();
            string path = Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "..", "..", "robot", "isaac", "golden", "scene-layouts-jetrover-mixed.json");
            if (Environment.GetEnvironmentVariable("TELEOP_WRITE_GOLDEN") == "1")
            {
                File.WriteAllText(path, expected, new UTF8Encoding(false));
            }

            Assert.True(File.Exists(path), $"{path} is missing: run `just gen-isaac-goldens`");
            Assert.Equal(File.ReadAllText(path).Replace("\r\n", "\n"), expected);
        }

        /// <summary>
        /// The first 20 layouts of each mixed scene as WorldHost sends them: every park, resize and layout placement in order, each a
        /// row (kind, id, then a placement's pose x, y, z, qw, qx, qy, qz as float32 carries it, or a resize's factor).
        /// </summary>
        internal static string LayoutsGoldenJson()
        {
            var sb = new StringBuilder();
            sb.Append("{\n \"about\": \"docs/adr/0038 decision 3: the first 20 layouts of each mixed scene as RobotHost's WorldHost sends them, written by WorldHostMixedTests ");
            sb.Append("(just gen-isaac-goldens): every park, resize and layout placement in order, each [kind, id, ...] with a placement's pose (x, y, z, qw, qx, qy, qz, float32 as ");
            sb.Append("the wire carries it) or a resize's factor. robot/isaac/tests/test_world.py replays them through Isaac's WorldModel, which must apply every one.\",\n");
            string[] names = { "jetrover-mixed", "jetrover-mixed-holdout" };
            for (int n = 0; n < names.Length; n++)
            {
                byte[] bytes = Bytes(names[n]);
                var rig = new Rig(names[n], seed: 1);
                rig.Settle();
                for (uint layout = 2; layout <= 20; layout++) rig.NewLayout(layout);
                var rows = rig.Backend.Requests.Select(r => r.Kind == "resize"
                    ? $"[\"resize\", {r.Pose.Id}, {F(r.Scale)}]"
                    : $"[\"{r.Kind}\", {r.Pose.Id}, {F(r.Pose.X)}, {F(r.Pose.Y)}, {F(r.Pose.Z)}, {F(r.Pose.Qw)}, {F(r.Pose.Qx)}, {F(r.Pose.Qy)}, {F(r.Pose.Qz)}]");
                sb.Append($" \"{names[n]}\": {{\"sha256\": \"{Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant()}\", \"requests\": [\n  ");
                sb.Append(string.Join(",\n  ", rows)).Append(n < names.Length - 1 ? "\n ]},\n" : "\n ]}\n");
            }

            sb.Append("}\n");
            return sb.ToString();
        }

        private static string F(float v) => ((double)v).ToString("R", CultureInfo.InvariantCulture);
    }
}
