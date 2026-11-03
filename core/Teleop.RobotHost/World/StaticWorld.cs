using System;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;

namespace Teleop.RobotHost.World
{
    /// <summary>
    /// The world beside the simulated arm (<c>just fr3-sim-host scene=...</c>, docs/adr/0031 amendment 2): objects stay where
    /// they were placed, and nothing collides with the arm. It applies the same placement rules as Isaac except the arm's
    /// overlap (the table and region, other objects, whether the headset may move it), and reports a snapshot every 20 ms, so
    /// the whole world path runs end to end without the GPU machine.
    /// </summary>
    public sealed class StaticWorld : IWorldBackend
    {
        private readonly WorldScene _scene;
        private readonly byte[] _sha;
        private readonly long _period;
        private readonly ObjectPose[] _poses;
        private readonly (uint Request, PlaceOutcome Outcome, ObjectPose Pose)[] _placed = new (uint, PlaceOutcome, ObjectPose)[64];
        private int _placedHead, _placedCount;
        private long _nextSnapshot = long.MinValue, _snapshotSensed;
        private bool _fresh;
        private uint _revision;

        public StaticWorld(WorldScene scene, ReadOnlySpan<byte> sceneSha256, long ticksPerSecond)
        {
            _scene = scene ?? throw new ArgumentNullException(nameof(scene));
            _sha = sceneSha256.ToArray();
            _period = ticksPerSecond / 50;
            _poses = new ObjectPose[scene.Objects.Length];
            for (int i = 0; i < _poses.Length; i++) _poses[i] = ObjectPose.Of(scene.Objects[i].Id, ObjectPose.FlagKinematic, scene.Objects[i].Nominal);
        }

        public bool HasWorld => true;
        public ReadOnlySpan<byte> SceneSha256 => _sha;
        public bool Physics => false;

        public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
        {
            PlaceOutcome outcome = Check(cause, pose, out int index, out ObjectPose upright);
            if (outcome == PlaceOutcome.Applied)
            {
                _poses[index] = upright;
                _revision++;
                _fresh = true;
                _snapshotSensed = hostTicks;
            }

            ObjectPose now = index >= 0 ? _poses[index] : pose;
            if (_placedCount < _placed.Length)
            {
                _placed[(_placedHead + _placedCount) % _placed.Length] = (request, outcome, now);
                _placedCount++;
            }
        }

        /// <summary>The snapshot clock: one every 20 ms.</summary>
        public void Poll(long hostTicks)
        {
            if (_nextSnapshot != long.MinValue && hostTicks < _nextSnapshot) return;
            _nextSnapshot = hostTicks + _period;
            _snapshotSensed = hostTicks;
            _fresh = true;
        }

        public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision)
        {
            count = 0;
            sensedTicks = _snapshotSensed;
            revision = _revision;
            if (!_fresh || poses.Length < _poses.Length) return false;
            _fresh = false;
            _poses.AsSpan().CopyTo(poses);
            count = _poses.Length;
            return true;
        }

        public bool TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose)
        {
            request = 0;
            outcome = 0;
            pose = default;
            if (_placedCount == 0) return false;
            (request, outcome, pose) = _placed[_placedHead];
            _placedHead = (_placedHead + 1) % _placed.Length;
            _placedCount--;
            return true;
        }

        private PlaceOutcome Check(PlaceCause cause, in ObjectPose pose, out int index, out ObjectPose upright)
        {
            upright = default;
            index = _scene.IndexOf(pose.Id);
            if (index < 0) return PlaceOutcome.UnknownObject;
            if (cause == PlaceCause.Headset && !_scene.Objects[index].OperatorMovable) return PlaceOutcome.NotMovable;
            if (!pose.IsValid()) return PlaceOutcome.InvalidPose;
            double yaw = WorldHost.YawOf(pose);
            if (!_scene.FitsOnTable(index, pose.X, pose.Y)) return PlaceOutcome.OutsideRegion;
            for (int k = 0; k < _poses.Length; k++)
            {
                if (k != index && _scene.Overlaps(index, pose.X, pose.Y, k, _poses[k].X, _poses[k].Y)) return PlaceOutcome.OverlapsObject;
            }

            upright = ObjectPose.Of(pose.Id, ObjectPose.FlagKinematic, _scene.PoseAt(index, pose.X, pose.Y, yaw));
            return PlaceOutcome.Applied;
        }
    }
}
