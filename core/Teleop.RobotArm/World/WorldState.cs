using System;
using Teleop.RobotArm.Wire;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// The headset's view of the world (docs/adr/0031 amendment 2): the newest pose of each of the scene's objects, by the time
    /// RobotHost sensed it, as world frames release from the joint link's playout, and the newest world status. A part that
    /// arrives late (older than what is held) changes nothing; an id the scene doesn't have is counted and ignored. No clock
    /// of its own; allocates nothing after construction.
    /// </summary>
    public sealed class WorldState
    {
        private readonly bool[] _known = new bool[256];
        private readonly bool[] _posed = new bool[256];
        private readonly ObjectPose[] _pose = new ObjectPose[256];
        private readonly long[] _sensed = new long[256];
        private readonly byte[] _statusSha = new byte[WorldCodec.Sha256Bytes];
        private readonly byte[] _sceneSha;

        /// <param name="sceneSha256">The headset's own scene file's sha256: a status naming another scene is a mismatch.</param>
        public WorldState(WorldScene scene, ReadOnlySpan<byte> sceneSha256)
        {
            if (scene == null) throw new ArgumentNullException(nameof(scene));
            if (sceneSha256.Length != WorldCodec.Sha256Bytes) throw new ArgumentException("a 32-byte sha256", nameof(sceneSha256));
            Scene = scene;
            _sceneSha = sceneSha256.ToArray();
            foreach (SceneObject o in scene.Objects) _known[o.Id] = true;
        }

        public WorldScene Scene { get; }
        public long Superseded { get; private set; }
        public long UnknownIds { get; private set; }
        public bool HasStatus { get; private set; }
        public WorldStatus Status { get; private set; }

        /// <summary>Whether RobotHost runs a different scene file than the headset's (known once a status has come).</summary>
        public bool SceneMismatch
        {
            get
            {
                if (!HasStatus) return false;
                for (int i = 0; i < _sceneSha.Length; i++)
                {
                    if (_sceneSha[i] != _statusSha[i]) return true;
                }

                return false;
            }
        }

        /// <summary>Takes one object's pose sensed at <paramref name="sensedTicks"/>; false when it is older or not the scene's.</summary>
        public bool Apply(in ObjectPose pose, long sensedTicks)
        {
            if (!_known[pose.Id])
            {
                UnknownIds++;
                return false;
            }

            if (_posed[pose.Id] && sensedTicks < _sensed[pose.Id])
            {
                Superseded++;
                return false;
            }

            _pose[pose.Id] = pose;
            _sensed[pose.Id] = sensedTicks;
            _posed[pose.Id] = true;
            return true;
        }

        public void ApplyStatus(in WorldStatus status, ReadOnlySpan<byte> sceneSha256)
        {
            Status = status;
            sceneSha256.Slice(0, WorldCodec.Sha256Bytes).CopyTo(_statusSha);
            HasStatus = true;
        }

        /// <summary>The newest pose of object <paramref name="id"/>, and when it was sensed; false before its first.</summary>
        public bool TryGet(byte id, out ObjectPose pose, out long sensedTicks)
        {
            pose = _pose[id];
            sensedTicks = _sensed[id];
            return _posed[id];
        }

        public void Reset()
        {
            Array.Clear(_posed, 0, _posed.Length);
            Superseded = UnknownIds = 0;
            HasStatus = false;
            Status = default;
        }
    }
}
