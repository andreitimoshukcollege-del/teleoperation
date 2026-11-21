using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Tests.Backend;
using Teleop.RobotHost.World;

namespace Teleop.RobotHost.Tests.World
{
    /// <summary>
    /// RobotHost's world for a scene whose objects are a pool (docs/adr/0037 decision 3, the JetRover bench): each layout puts a
    /// few on the table and parks the rest, the parked ones are flagged to the headset and out of its reach, and a spot the
    /// backend refuses ends in the object being parked, never left where it stood.
    /// </summary>
    public class WorldHostPoolTests
    {
        private const long Tps = 1_000_000;
        private const long Ms = Tps / 1000;

        private static readonly byte[] SceneBytes = File.ReadAllBytes(Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "..", "Scenes", "jetrover-bench.json"));
        private static readonly WorldScene Scene = JsonSerializer.Deserialize<SceneDocument>(SceneBytes, new JsonSerializerOptions { IncludeFields = true })!.ToScene();
        private static readonly byte[] Sha = SHA256.HashData(SceneBytes);

        /// <summary>StaticWorld, refusing every layout placement of one object, as Isaac does when an arm stands in the way.</summary>
        private sealed class Refusing : IWorldBackend
        {
            private readonly StaticWorld _inner = new StaticWorld(Scene, Sha, Tps);
            private readonly Queue<(uint, PlaceOutcome, ObjectPose)> _refused = new Queue<(uint, PlaceOutcome, ObjectPose)>();
            public byte RefuseId;
            public bool HasWorld => true;
            public ReadOnlySpan<byte> SceneSha256 => Sha;
            public bool Physics => false;
            public void Poll(long now) => _inner.Poll(now);

            public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
            {
                if (pose.Id == RefuseId && cause == PlaceCause.Layout) _refused.Enqueue((request, PlaceOutcome.OverlapsArm, pose));
                else _inner.Place(request, cause, pose, hostTicks);
            }

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
            public readonly List<byte[]> Sent = new List<byte[]>();
            public readonly WorldHost Host;
            public readonly IWorldBackend Backend;
            public long Now;

            public Rig(IWorldBackend? backend = null, ulong seed = 5)
            {
                Backend = backend ?? new StaticWorld(Scene, Sha, Tps);
                Host = new WorldHost(Scene, Sha, Backend, Tps, seed, Records.Add);
            }

            public void Tick(double ms)
            {
                Now += (long)(ms * Ms);
                (Backend as StaticWorld)?.Poll(Now);
                (Backend as Refusing)?.Poll(Now);
                Host.Tick(Now, true, true, (b, n) => Sent.Add(b.AsSpan(0, n).ToArray()));
            }

            /// <summary>The newest complete snapshot as the headset's WorldState has it, by id (parked marked from the pose).</summary>
            public Dictionary<byte, ObjectPose> LastSnapshot()
            {
                var poses = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
                var state = new WorldState(Scene, Sha);
                foreach (byte[] d in Sent.Where(x => x[0] == WorldCodec.FrameTag))
                {
                    Assert.True(WorldCodec.TryDecodeFrame(d, out _, out long sensed, out _, out _, poses, out int n));
                    for (int k = 0; k < n; k++) state.Apply(poses[k], sensed);
                }

                var byId = new Dictionary<byte, ObjectPose>();
                foreach (SceneObject o in Scene.Objects)
                {
                    if (state.TryGet(o.Id, out ObjectPose p, out _)) byId[o.Id] = p;
                }

                return byId;
            }

            public PlaceOutcome LastEditOutcome() => Records.Where(r => r.StartsWith("WE|")).Select(r => r.Split('|').Last()).Last() switch
            {
                "applied" => PlaceOutcome.Applied,
                "not-movable" => PlaceOutcome.NotMovable,
                var other => throw new Xunit.Sdk.XunitException("outcome " + other),
            };
        }

        [Fact]
        public void ALayout_PutsTheDrawnObjectsOnTheTable_AndParksTheRest_FlaggedToTheHeadset()
        {
            var rig = new Rig(seed: 5);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            var expected = new RigidTransform[Scene.Objects.Length];
            var parked = new bool[Scene.Objects.Length];
            var scales = new double[Scene.Objects.Length];
            SceneLayout.Draw(Scene, 5, expected, parked, scales);
            Dictionary<byte, ObjectPose> seen = rig.LastSnapshot();
            Assert.Equal(Scene.Objects.Length, seen.Count);
            bool resizes = Scene.DrawsSizes && Enumerable.Range(0, scales.Length).Any(Scene.Resizable);
            for (int i = 0; i < Scene.Objects.Length; i++)
            {
                ObjectPose p = seen[Scene.Objects[i].Id];
                Assert.Equal(parked[i], p.Parked);
                if (!parked[i]) Assert.Equal(scales[i], rig.Host.ScaleOf(i), 4); // resized on the shelf, then placed at that size
                if (!parked[i]) Assert.True(Math.Abs(expected[i].Z - p.Z) < 1e-4, $"{Scene.Objects[i].Name}: z {p.Z} not {expected[i].Z}"); // float32 on the wire
                Assert.Equal(parked[i], rig.Host.IsParked(i));
                Assert.Equal(expected[i].X, p.X, 4);
                Assert.Equal(expected[i].Y, p.Y, 4);
            }

            // The records: WL names the objects on the table. A pool that draws sizes parks every object first (sizes change on
            // the shelf), then places each drawn one with cause layout.
            string wl = rig.Records.First(r => r.StartsWith("WL|"));
            string active = string.Join(";", Scene.Objects.Where((o, i) => !parked[i]).Select(o => o.Id));
            Assert.EndsWith("|" + active, wl);
            Assert.Equal(resizes ? Scene.Objects.Length : parked.Count(p => p), rig.Records.Count(r => r.StartsWith("WP|") && r.Split('|')[3] == "park" && r.EndsWith("|applied")));
            Assert.Equal(parked.Count(p => !p), rig.Records.Count(r => r.StartsWith("WP|") && r.Split('|')[3] == "layout" && r.EndsWith("|applied")));
        }

        [Fact]
        public void NewLayout_ChangesWhichObjectsTakePart_AndResetPutsTheDrawnLayoutBack()
        {
            var rig = new Rig(seed: 5);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            bool[] first = Enumerable.Range(0, Scene.Objects.Length).Select(rig.Host.IsParked).ToArray();
            rig.Host.HandleEdit(WorldEditOp.NewLayout, 1, default, true, rig.Now);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            bool[] second = Enumerable.Range(0, Scene.Objects.Length).Select(rig.Host.IsParked).ToArray();
            Assert.NotEqual(first, second);
            Dictionary<byte, ObjectPose> laid = rig.LastSnapshot();

            int mover = Array.FindIndex(second, p => !p);
            byte id = Scene.Objects[mover].Id;
            RigidTransform there = Scene.PoseAt(mover, Scene.RegionXMin + 0.05, Scene.RegionYMax - 0.05, 0);
            rig.Host.HandleEdit(WorldEditOp.Move, 2, ObjectPose.Of(id, 0, there), true, rig.Now);
            for (int i = 0; i < 30; i++) rig.Tick(1);
            rig.Host.HandleEdit(WorldEditOp.ResetLayout, 3, default, true, rig.Now);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            Dictionary<byte, ObjectPose> reset = rig.LastSnapshot();
            foreach (SceneObject o in Scene.Objects)
            {
                if (laid[o.Id].Parked != reset[o.Id].Parked) Assert.Fail($"{o.Name}: parked {laid[o.Id].Parked} -> {reset[o.Id].Parked}");
                if (!reset[o.Id].Parked && o.Id != id) Assert.Equal(laid[o.Id].X, reset[o.Id].X, 4);
            }

            Assert.Equal(laid[id].X, reset[id].X, 4); // back where the layout drew it, not at its nominal pose
            Assert.Equal(laid[id].Y, reset[id].Y, 4);
        }

        [Fact]
        public void AParkedObject_CannotBeMovedOrResizedFromTheHeadset()
        {
            var rig = new Rig(seed: 5);
            for (int i = 0; i < 50; i++) rig.Tick(1);
            int index = Enumerable.Range(0, Scene.Objects.Length).First(i => rig.Host.IsParked(i) && Scene.Objects[i].OperatorMovable);
            byte id = Scene.Objects[index].Id;
            rig.Host.HandleEdit(WorldEditOp.Move, 1, ObjectPose.Of(id, 0, Scene.PoseAt(index, 0.28, 0.0, 0)), true, rig.Now);
            for (int i = 0; i < 5; i++) rig.Tick(1);
            Assert.Equal(PlaceOutcome.NotMovable, rig.LastEditOutcome());
            rig.Host.HandleEdit(WorldEditOp.Resize, 2, new ObjectPose(id, 0, 1.25f, 0, 0, 1, 0, 0, 0), true, rig.Now);
            for (int i = 0; i < 5; i++) rig.Tick(1);
            Assert.Equal(PlaceOutcome.NotMovable, rig.LastEditOutcome());
            Assert.True(rig.LastSnapshot()[id].Parked);
        }

        [Fact]
        public void AnObjectNoSpotIsTakenFor_IsParked_AfterItsRedraws()
        {
            var expected = new RigidTransform[Scene.Objects.Length];
            var parked = new bool[Scene.Objects.Length];
            SceneLayout.Draw(Scene, 5, expected, parked);
            int index = Array.FindIndex(parked, p => !p);
            var backend = new Refusing { RefuseId = Scene.Objects[index].Id };
            var rig = new Rig(backend, seed: 5);
            for (int i = 0; i < 80; i++) rig.Tick(1);
            Assert.True(rig.Host.IsParked(index));
            Assert.True(rig.LastSnapshot()[Scene.Objects[index].Id].Parked);
            // Each refusal is redrawn until MaxRedraws, or sooner when no other spot is clear (the reach band is small).
            Assert.InRange(rig.Records.Count(r => r.StartsWith("WP|") && r.Split('|')[4] == Scene.Objects[index].Id.ToString() && r.EndsWith("|overlaps-arm")), 1, WorldHost.MaxRedraws + 1);
            Assert.DoesNotContain(Scene.Objects[index].Id.ToString(), rig.Records.Last(r => r.StartsWith("WL|")).Split('|')[4].Split(';'));
        }

        [Fact]
        public void AClutterSpotTheBackendRefuses_IsRedrawnFlatInTheRegion_ThenParked()
        {
            // docs/adr/0037 "Raised clutter": a refused piece of clutter is redrawn as clutter (anywhere in the region, its own
            // rectangle at its yaw, flat), not in the reach band the pool's movable objects keep to; after the redraws it is parked.
            var expected = new RigidTransform[Scene.Objects.Length];
            var parked = new bool[Scene.Objects.Length];
            ulong seed = 0;
            int index = -1;
            for (ulong s = 1; s <= 50 && index < 0; s++)
            {
                SceneLayout.Draw(Scene, s, expected, parked);
                for (int i = 0; i < parked.Length && index < 0; i++)
                {
                    if (!parked[i] && Scene.Objects[i].Clutter) index = i;
                }

                seed = s;
            }

            Assert.True(index >= 0, "no layout of seeds 1-50 has clutter on the table");
            byte id = Scene.Objects[index].Id;
            var rig = new Rig(new Refusing { RefuseId = id }, seed);
            for (int i = 0; i < 80; i++) rig.Tick(1);
            Assert.True(rig.Host.IsParked(index));
            string[] refused = rig.Records.Where(r => r.StartsWith("WP|") && r.Split('|')[4] == id.ToString() && r.EndsWith("|overlaps-arm")).ToArray();
            Assert.InRange(refused.Length, 1, WorldHost.MaxRedraws + 1); // its layout spot, then each redraw until none is clear
            foreach (string r in refused)
            {
                double[] p = r.Split('|')[5].Split(':').Select(v => double.Parse(v, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
                Assert.InRange(p[0], Scene.RegionXMin, Scene.RegionXMax);
                Assert.InRange(p[1], Scene.RegionYMin, Scene.RegionYMax);
                Assert.True(Math.Abs(p[4]) < 1e-4 && Math.Abs(p[5]) < 1e-4, $"{r}: not flat");
                Assert.Equal(Scene.Table.Top + Scene.Objects[index].SizeZ / 2, p[2], 4);
            }
        }

        [Fact]
        public void PlaceCausePark_CrossesTheWire_AndResizeNeverDoes()
        {
            var buffer = new byte[128];
            var pose = new ObjectPose(3, 0, 0.3f, 0, 0, 1, 0, 0, 0);
            Assert.True(ArmServerWorld.TryEncodePlace(1, 2, 3, PlaceCause.Park, pose, buffer, out int n));
            Assert.True(ArmServerWorld.TryDecodePlace(buffer.AsSpan(0, n), out _, out _, out PlaceCause cause, out _));
            Assert.Equal(PlaceCause.Park, cause);
            Assert.True(ArmServerWorld.TryEncodePlace(1, 2, 3, PlaceCause.Resize, pose, buffer, out n));
            Assert.False(ArmServerWorld.TryDecodePlace(buffer.AsSpan(0, n), out _, out _, out _, out _));
        }
    }
}
