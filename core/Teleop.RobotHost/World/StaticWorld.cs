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
        private readonly float[] _scales;
        private uint _sizesRevision;
        private bool _sizesFresh = true;
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
            _scales = new float[scene.Objects.Length];
            for (int i = 0; i < _scales.Length; i++) _scales[i] = 1f;
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
            Answer(request, outcome, now);
        }

        /// <summary>Resizes one object, by the same rules at the new size (docs/adr/0034 amendment 6): stood up where it is.</summary>
        public void Resize(uint request, byte id, float scale, long hostTicks)
        {
            int index = _scene.IndexOf(id);
            PlaceOutcome outcome;
            double cx = 0, cy = 0;
            if (index < 0) outcome = PlaceOutcome.UnknownObject;
            else if (!_scene.Objects[index].OperatorMovable) outcome = PlaceOutcome.NotMovable;
            else if (!WorldScene.IsValidScale(scale)) outcome = PlaceOutcome.InvalidPose;
            else if (!_scene.TryClampOnTable(index, _poses[index].X, _poses[index].Y, scale, out cx, out cy)) outcome = PlaceOutcome.OutsideRegion;
            else
            {
                outcome = PlaceOutcome.Applied;
                for (int k = 0; k < _poses.Length; k++)
                {
                    if (k != index && _scene.Overlaps(index, cx, cy, k, _poses[k].X, _poses[k].Y, scale, _scales[k])) outcome = PlaceOutcome.OverlapsObject;
                }
            }

            if (outcome == PlaceOutcome.Applied)
            {
                ObjectPose p = _poses[index];
                _scales[index] = scale;
                _poses[index] = ObjectPose.Of(id, ObjectPose.FlagKinematic, _scene.PoseAt(index, cx, cy, WorldHost.YawOf(p), scale));
                _revision++;
                _sizesRevision++;
                _sizesFresh = true;
                _fresh = true;
                _snapshotSensed = hostTicks;
            }

            Answer(request, outcome, index >= 0 ? _poses[index] : new ObjectPose(id, 0, 0, 0, 0, 1, 0, 0, 0));
        }

        public bool TryTakeSizes(Span<byte> ids, Span<float> scales, out int count, out uint revision)
        {
            count = 0;
            revision = _sizesRevision;
            if (!_sizesFresh || ids.Length < _scales.Length || scales.Length < _scales.Length) return false;
            _sizesFresh = false;
            for (int i = 0; i < _scales.Length; i++)
            {
                ids[i] = _scene.Objects[i].Id;
                scales[i] = _scales[i];
            }

            count = _scales.Length;
            return true;
        }

        private void Answer(uint request, PlaceOutcome outcome, in ObjectPose now)
        {
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
            if (!_scene.FitsOnTable(index, pose.X, pose.Y, _scales[index])) return PlaceOutcome.OutsideRegion;
            for (int k = 0; k < _poses.Length; k++)
            {
                if (k != index && _scene.Overlaps(index, pose.X, pose.Y, k, _poses[k].X, _poses[k].Y, _scales[index], _scales[k])) return PlaceOutcome.OverlapsObject;
            }

            upright = ObjectPose.Of(pose.Id, ObjectPose.FlagKinematic, _scene.PoseAt(index, pose.X, pose.Y, yaw, _scales[index]));
            return PlaceOutcome.Applied;
        }
    }
}
