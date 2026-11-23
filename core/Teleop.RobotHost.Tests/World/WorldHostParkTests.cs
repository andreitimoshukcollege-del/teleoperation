using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text.Json;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Tests.Backend;
using Teleop.RobotHost.World;

namespace Teleop.RobotHost.Tests.World
{
    /// <summary>
    /// docs/adr/0038 decision 2: a replay's <c>remove</c> takes one object off the table with the park edit (op 6), any object, a
    /// fixture too; reset-layout puts the source's layout back. The headset never sends it, and the wire still refuses op 7.
    /// </summary>
    public class WorldHostParkTests
    {
        private const long Tps = 1_000_000;
        private const long Ms = Tps / 1000;

        private static readonly byte[] SceneBytes = File.ReadAllBytes(Path.Combine(Path.GetDirectoryName(SerialChainTestProfile.Path)!, "..", "Scenes", "jetrover-bench.json"));
        private static readonly WorldScene Scene = JsonSerializer.Deserialize<SceneDocument>(SceneBytes, new JsonSerializerOptions { IncludeFields = true })!.ToScene();
        private static readonly byte[] Sha = SHA256.HashData(SceneBytes);

        private readonly List<string> _records = new List<string>();
        private readonly List<byte[]> _sent = new List<byte[]>();
        private readonly StaticWorld _backend = new StaticWorld(Scene, Sha, Tps);
        private readonly WorldHost _host;
        private long _now;

        public WorldHostParkTests() => _host = new WorldHost(Scene, Sha, _backend, Tps, 5, _records.Add);

        private void Tick(int ms, bool motionOff = true)
        {
            for (int i = 0; i < ms; i++)
            {
                _now += Ms;
                _backend.Poll(_now);
                _host.Tick(_now, true, motionOff, (b, n) => _sent.Add(b.AsSpan(0, n).ToArray()));
            }
        }

        /// <summary>The newest pose of every object as joint-drive's WorldState has it (parked marked from the pose).</summary>
        private Dictionary<byte, ObjectPose> Seen()
        {
            var poses = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
            var state = new WorldState(Scene, Sha);
            foreach (byte[] d in _sent.Where(x => x[0] == WorldCodec.FrameTag))
            {
                Assert.True(WorldCodec.TryDecodeFrame(d, out _, out long sensed, out _, out _, poses, out int n));
                for (int k = 0; k < n; k++) state.Apply(poses[k], sensed);
            }

            var byId = new Dictionary<byte, ObjectPose>();
            foreach (SceneObject o in Scene.Objects) if (state.TryGet(o.Id, out ObjectPose p, out _)) byId[o.Id] = p;
            return byId;
        }

        private string LastEdit() => _records.Last(r => r.StartsWith("WE|"));

        [Fact]
        public void AParkEdit_TakesAnyObjectOffTheTable_AndResetLayoutPutsItBack()
        {
            Tick(50);
            Dictionary<byte, ObjectPose> laid = Seen();
            // One object of the layout of each kind: a fixture (the headset cannot move it) and a movable one.
            foreach (bool fixture in new[] { true, false })
            {
                int index = Enumerable.Range(0, Scene.Objects.Length).First(i => !_host.IsParked(i) && Scene.Objects[i].Movable != fixture && !Scene.Objects[i].Clutter);
                byte id = Scene.Objects[index].Id;
                uint sequence = fixture ? 11u : 21u;
                _host.HandleEdit(WorldEditOp.Park, sequence, new ObjectPose(id, 0, 0, 0, 0, 1, 0, 0, 0), true, _now);
                Tick(20);
                Assert.Equal($"park|{id}", string.Join("|", LastEdit().Split('|').Skip(3).Take(2)));
                Assert.EndsWith("|applied", LastEdit());
                Assert.True(Seen()[id].Parked, $"{Scene.Objects[index].Name} is still on the table");
                Assert.False(_host.IsParked(index)); // the layout as drawn: the edit is the replay's

                // A second park of it, or of an object the layout left out, is refused; so is any with motion on.
                int left = Enumerable.Range(0, Scene.Objects.Length).First(_host.IsParked);
                _host.HandleEdit(WorldEditOp.Park, sequence + 1, new ObjectPose(Scene.Objects[left].Id, 0, 0, 0, 0, 1, 0, 0, 0), true, _now);
                Tick(5);
                Assert.EndsWith("|not-movable", LastEdit());
                _host.HandleEdit(WorldEditOp.Park, sequence + 2, new ObjectPose(id, 0, 0, 0, 0, 1, 0, 0, 0), false, _now);
                Tick(5, motionOff: false);
                Assert.EndsWith("|busy", LastEdit());

                _host.HandleEdit(WorldEditOp.ResetLayout, sequence + 3, default, true, _now);
                Tick(30);
                Dictionary<byte, ObjectPose> reset = Seen();
                Assert.False(reset[id].Parked);
                Assert.Equal(laid[id].X, reset[id].X, 4);
                Assert.Equal(laid[id].Y, reset[id].Y, 4);
                Assert.Equal(laid[id].Z, reset[id].Z, 4);
            }
        }

        [Fact]
        public void TheParkOp_CrossesTheWire_AndNothingPastItDoes()
        {
            var buffer = new byte[WorldCodec.EditBytes];
            Assert.True(WorldCodec.TryEncodeEdit(WorldEditOp.Park, 41, 7, 9, new ObjectPose(41, 0, 0, 0, 0, 1, 0, 0, 0), buffer, out int n));
            Assert.True(WorldCodec.TryDecodeEdit(buffer.AsSpan(0, n), out WorldEditOp op, out uint sequence, out _, out ObjectPose pose));
            Assert.Equal((WorldEditOp.Park, 7u, (byte)41), (op, sequence, pose.Id));
            buffer[2] = 7;
            Assert.False(WorldCodec.TryDecodeEdit(buffer.AsSpan(0, n), out _, out _, out _, out _));
        }
    }
}
