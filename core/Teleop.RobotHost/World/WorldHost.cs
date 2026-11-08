using System;
using Teleop.Core.Types;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.World
{
    /// <summary>
    /// RobotHost's side of the world (docs/adr/0031 amendment 2). It decides where objects go and tells the headset where they
    /// are; the backend (Isaac, or <see cref="StaticWorld"/>) does the rest.
    /// <list type="bullet">
    /// <item><b>Layouts.</b> <see cref="Start"/> draws layout 1 from the seed (<see cref="SceneLayout"/>) and places every
    /// object; NEW LAYOUT from the headset draws the next (seed + layout − 1), only while motion is off. A placement the backend
    /// refuses (it overlaps an arm, say) is redrawn up to <see cref="MaxRedraws"/> times within that object's jitter, and
    /// otherwise the object stays where it was. Layouts never change on their own.</item>
    /// <item><b>Edits.</b> Moves and resets from the headset (joint port <c>0x85</c>), each applied once however often it is
    /// re-sent; the status echoes its sequence and outcome once settled.</item>
    /// <item><b>Sizes</b> (docs/adr/0034 amendment 6). A resize from the headset (<see cref="WorldEditOp.Resize"/>) goes to the
    /// backend, which applies or refuses it by the placement rules at the new size. The backend's sizes are the truth: they go to
    /// the headset (<c>0x8B</c>) with the status, and into the records (<c>WZ</c>) when they change.</item>
    /// <item><b>To the headset.</b> The newest snapshot as world frames (<c>0x86</c>, eight objects a part) every 20 ms, and the
    /// status (<c>0x87</c>) once a second and on change, both only while someone subscribes.</item>
    /// <item><b>Records.</b> <c>WL</c>, <c>WP</c>, <c>WO</c> (a snapshot whenever something moved 0.1 mm or 0.05°, otherwise once a
    /// second), <c>WF</c> and <c>WE</c>, through the caller's recorder.</item>
    /// </list>
    /// No clock or socket of its own: the host passes the time and a sender.
    /// </summary>
    public sealed class WorldHost
    {
        public const int MaxRedraws = 8;
        public const double PlaceTimeoutSeconds = 1.0;

        private readonly WorldScene _scene;
        private readonly byte[] _sha;
        private readonly IWorldBackend _backend;
        private readonly long _tps;
        private readonly ulong _baseSeed;
        private readonly Action<string>? _record;
        private readonly Action<string>? _log;
        private readonly RigidTransform[] _targets;
        private readonly ObjectPose[] _snapshot;
        private readonly ObjectPose[] _lastRecorded;
        private readonly ObjectPose[] _part = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
        private readonly byte[] _datagram = new byte[JointDatagram.MaxBytes];
        private readonly Pending[] _pending = new Pending[64];
        private readonly byte[] _sizeIds;
        private readonly float[] _sizeScales;
        private readonly byte[] _takenIds = new byte[WorldScene.MaxObjects];
        private readonly float[] _takenScales = new float[WorldScene.MaxObjects];
        private uint _sizesRevision;
        private bool _sizesChanged;
        private int _snapshotCount;
        private long _snapshotSensed, _lastFrame = long.MinValue, _lastStatus = long.MinValue, _lastRecordedTicks = long.MinValue;
        private uint _snapshotRevision, _frameSnapshot, _request;
        private bool _hasSnapshot, _started, _recordedOnce;
        private WorldStatus _sentStatus;
        private uint _editSequence, _resolvedEditSequence;
        private byte _editOutcome, _editObject;
        private bool _shaChecked, _shaMatches;

        private struct Pending
        {
            public bool Active;
            public uint Request;
            public int Index;
            public PlaceCause Cause;
            public int Tries;
            public long Sent;
            public uint Edit;
        }

        public WorldHost(WorldScene scene, ReadOnlySpan<byte> sceneSha256, IWorldBackend backend, long ticksPerSecond, ulong seed,
            Action<string>? record = null, Action<string>? log = null)
        {
            _scene = scene ?? throw new ArgumentNullException(nameof(scene));
            _sha = sceneSha256.ToArray();
            _backend = backend ?? throw new ArgumentNullException(nameof(backend));
            _tps = ticksPerSecond;
            _baseSeed = seed;
            _record = record;
            _log = log;
            _targets = new RigidTransform[scene.Objects.Length];
            _snapshot = new ObjectPose[scene.Objects.Length];
            _lastRecorded = new ObjectPose[scene.Objects.Length];
            _sizeIds = new byte[scene.Objects.Length];
            _sizeScales = new float[scene.Objects.Length];
            for (int i = 0; i < _sizeIds.Length; i++)
            {
                _sizeIds[i] = scene.Objects[i].Id;
                _sizeScales[i] = 1f;
            }
        }

        /// <summary>Object <paramref name="index"/>'s size factor as the backend last reported it (1 before).</summary>
        public float ScaleOf(int index) => _sizeScales[index];

        public uint LayoutNumber { get; private set; }
        public ulong LayoutSeed { get; private set; }
        public long FramesSent { get; private set; }
        public long PlacementsRefused { get; private set; }

        /// <summary>Whether the backend runs this scene (known once it says which); a mismatch keeps the world off.</summary>
        public bool Running => _backend.HasWorld && ShaMatches();

        /// <summary>The upright yaw of a pose: its heading about the vertical.</summary>
        public static double YawOf(in ObjectPose p) =>
            Math.Atan2(2.0 * (p.Qw * p.Qz + p.Qx * p.Qy), 1.0 - 2.0 * (p.Qy * p.Qy + p.Qz * p.Qz));

        /// <summary>Draws and places layout 1, once the backend runs this scene. Called by <see cref="Tick"/> until it has.</summary>
        public void Start(long now)
        {
            if (_started || !Running) return;
            _started = true;
            NewLayout(now);
        }

        /// <summary>The session's preamble: the current layout and where the objects are, so every session stands alone.</summary>
        public void WritePreamble(long now)
        {
            if (!_started) return;
            _record?.Invoke(SessionLines.WorldLayout(now, LayoutNumber, LayoutSeed));
            _record?.Invoke(SessionLines.WorldSizes(now, _sizesRevision, _sizeIds, _sizeScales));
            if (_hasSnapshot) RecordSnapshot(now, force: true);
        }

        /// <summary>An edit request from the headset; <paramref name="motionOff"/> is the headset's last reported mode.</summary>
        public void HandleEdit(WorldEditOp op, uint sequence, in ObjectPose pose, bool motionOff, long now)
        {
            if (sequence == 0 || sequence == _editSequence) return; // re-sent until echoed: applied once
            _editSequence = sequence;
            _editObject = pose.Id;
            PlaceOutcome outcome;
            if (!Running || !_started) outcome = PlaceOutcome.Busy;
            else if (!motionOff) outcome = PlaceOutcome.Busy;
            else if (op == WorldEditOp.NewLayout)
            {
                NewLayout(now);
                outcome = PlaceOutcome.Applied;
            }
            else if (op == WorldEditOp.ResetLayout)
            {
                for (int i = 0; i < _targets.Length; i++)
                {
                    _targets[i] = _scene.Objects[i].Nominal;
                    SendPlace(i, PlaceCause.Reset, 0, now);
                }

                outcome = PlaceOutcome.Applied;
            }
            else if (op == WorldEditOp.Resize)
            {
                int index = _scene.IndexOf(pose.Id);
                if (index < 0) outcome = PlaceOutcome.UnknownObject;
                else if (!_scene.Objects[index].OperatorMovable) outcome = PlaceOutcome.NotMovable;
                else if (!WorldScene.IsValidScale(pose.X)) outcome = PlaceOutcome.InvalidPose;
                else
                {
                    SendResize(index, pose.X, sequence, now);
                    _record?.Invoke(SessionLines.WorldEdit(now, sequence, op, pose, null));
                    return; // settles when the backend answers
                }
            }
            else
            {
                int index = _scene.IndexOf(pose.Id);
                if (index < 0) outcome = PlaceOutcome.UnknownObject;
                else if (!_scene.Objects[index].OperatorMovable) outcome = PlaceOutcome.NotMovable;
                else if (op == WorldEditOp.Move && !pose.IsValid()) outcome = PlaceOutcome.InvalidPose;
                else
                {
                    _targets[index] = op == WorldEditOp.ResetObject ? _scene.Objects[index].Nominal : pose.ToTransform();
                    SendPlace(index, op == WorldEditOp.ResetObject ? PlaceCause.Reset : PlaceCause.Headset, sequence, now);
                    _record?.Invoke(SessionLines.WorldEdit(now, sequence, op, pose, null));
                    return; // settles when the backend answers
                }
            }

            Resolve(sequence, outcome, now, op, pose);
        }

        /// <summary>Everything since the last call: placement results, timeouts, the newest snapshot, frames and the status.</summary>
        public void Tick(long now, bool subscribed, bool motionOff, Action<byte[], int> send)
        {
            Start(now);
            while (_backend.TryTakePlaced(out uint request, out PlaceOutcome outcome, out ObjectPose pose)) Settle(request, outcome, pose, now);
            for (int p = 0; p < _pending.Length; p++)
            {
                if (_pending[p].Active && now - _pending[p].Sent > (long)(PlaceTimeoutSeconds * _tps))
                {
                    Settle(_pending[p].Request, PlaceOutcome.TimedOut, ObjectPose.Of(_scene.Objects[_pending[p].Index].Id, 0, _targets[_pending[p].Index]), now);
                }
            }

            if (_backend.TryTakeSizes(_takenIds, _takenScales, out int sizes, out uint sizesRevision))
            {
                for (int k = 0; k < sizes; k++)
                {
                    int index = _scene.IndexOf(_takenIds[k]);
                    if (index >= 0 && WorldScene.IsValidScale(_takenScales[k])) _sizeScales[index] = _takenScales[k];
                }

                _sizesRevision = sizesRevision;
                _sizesChanged = true;
                _record?.Invoke(SessionLines.WorldSizes(now, _sizesRevision, _sizeIds, _sizeScales));
            }

            if (_backend.TryTakeSnapshot(_snapshot, out int count, out long sensed, out uint revision))
            {
                _snapshotCount = count;
                _snapshotSensed = sensed;
                _snapshotRevision = revision;
                _hasSnapshot = true;
                _frameSnapshot++;
                RecordSnapshot(now, force: false);
            }

            if (!subscribed) return;
            if (_hasSnapshot && Running && (_lastFrame == long.MinValue || now - _lastFrame >= _tps / 50))
            {
                _lastFrame = now;
                int parts = (_snapshotCount + WorldCodec.MaxObjectsPerFrame - 1) / WorldCodec.MaxObjectsPerFrame;
                for (int part = 0; part < parts; part++)
                {
                    int from = part * WorldCodec.MaxObjectsPerFrame, n = Math.Min(WorldCodec.MaxObjectsPerFrame, _snapshotCount - from);
                    _snapshot.AsSpan(from, n).CopyTo(_part);
                    if (WorldCodec.TryEncodeFrame(_frameSnapshot, _snapshotSensed, (byte)part, (byte)parts, _part.AsSpan(0, n), _datagram, out int bytes))
                    {
                        send(_datagram, bytes);
                    }
                }

                FramesSent++;
                _record?.Invoke(SessionLines.WorldFrame(now, _frameSnapshot, _snapshotSensed, parts));
            }

            byte flags = (byte)((Running ? WorldStatus.FlagRunning : 0) | (_backend.Physics ? WorldStatus.FlagPhysics : 0) | (motionOff ? WorldStatus.FlagEditsAllowed : 0));
            var status = new WorldStatus(flags, (byte)_scene.Objects.Length, LayoutNumber, LayoutSeed, _resolvedEditSequence, _editOutcome, _editObject);
            if (_lastStatus == long.MinValue || now - _lastStatus >= _tps || !status.Equals(_sentStatus) || _sizesChanged)
            {
                if (WorldCodec.TryEncodeStatus(status, _sha, now, _datagram, out int bytes)) send(_datagram, bytes);
                if (Running && WorldCodec.TryEncodeSizes(_sizesRevision, _sizeIds, _sizeScales, _datagram, out int sizeBytes)) send(_datagram, sizeBytes);
                _sentStatus = status;
                _lastStatus = now;
                _sizesChanged = false;
            }
        }

        private bool ShaMatches()
        {
            if (!_shaChecked && _backend.HasWorld)
            {
                _shaChecked = true;
                _shaMatches = _backend.SceneSha256.SequenceEqual(_sha);
                if (!_shaMatches) _log?.Invoke("[world] the backend runs another scene file than RobotHost's: the world stays off (check both use the same core/Scenes file)");
            }

            return _shaMatches;
        }

        private void NewLayout(long now)
        {
            LayoutNumber++;
            LayoutSeed = unchecked(_baseSeed + LayoutNumber - 1);
            int nominal = SceneLayout.Draw(_scene, LayoutSeed, _targets);
            _record?.Invoke(SessionLines.WorldLayout(now, LayoutNumber, LayoutSeed));
            _log?.Invoke($"[world] layout {LayoutNumber} (seed {LayoutSeed}){(nominal > 0 ? $", {nominal} object(s) at their nominal pose" : "")}");
            for (int i = 0; i < _targets.Length; i++) SendPlace(i, PlaceCause.Layout, 0, now);
        }

        private void SendPlace(int index, PlaceCause cause, uint edit, long now, int tries = 0)
        {
            uint request = ++_request;
            int slot = Array.FindIndex(_pending, p => !p.Active);
            if (slot < 0) slot = (int)(request % (uint)_pending.Length);
            _pending[slot] = new Pending { Active = true, Request = request, Index = index, Cause = cause, Tries = tries, Sent = now, Edit = edit };
            _backend.Place(request, cause, ObjectPose.Of(_scene.Objects[index].Id, ObjectPose.FlagKinematic, _targets[index]), now);
        }

        private void SendResize(int index, float scale, uint edit, long now)
        {
            uint request = ++_request;
            int slot = Array.FindIndex(_pending, p => !p.Active);
            if (slot < 0) slot = (int)(request % (uint)_pending.Length);
            _pending[slot] = new Pending { Active = true, Request = request, Index = index, Cause = PlaceCause.Resize, Sent = now, Edit = edit };
            _backend.Resize(request, _scene.Objects[index].Id, scale, now);
        }

        private void Settle(uint request, PlaceOutcome outcome, in ObjectPose pose, long now)
        {
            int slot = -1;
            for (int p = 0; p < _pending.Length; p++)
            {
                if (_pending[p].Active && _pending[p].Request == request)
                {
                    slot = p;
                    break;
                }
            }

            if (slot < 0) return; // already timed out, or not ours
            Pending done = _pending[slot];
            _pending[slot].Active = false;
            _record?.Invoke(SessionLines.WorldPlace(now, request, done.Cause, pose, outcome));
            if (outcome != PlaceOutcome.Applied) PlacementsRefused++;
            if (outcome != PlaceOutcome.Applied && done.Cause == PlaceCause.Layout && outcome != PlaceOutcome.TimedOut && done.Tries < MaxRedraws &&
                Redraw(done.Index, done.Tries))
            {
                SendPlace(done.Index, PlaceCause.Layout, 0, now, done.Tries + 1);
                return;
            }

            if (done.Edit != 0) Resolve(done.Edit, outcome, now, done.Cause == PlaceCause.Resize ? WorldEditOp.Resize : WorldEditOp.Move, pose);
        }

        /// <summary>Another spot for object <paramref name="index"/> within its jitter, clear of the others' targets.</summary>
        private bool Redraw(int index, int tries)
        {
            SceneObject o = _scene.Objects[index];
            var rng = new SeededRng(unchecked(LayoutSeed * 0x9E3779B97F4A7C15UL + (ulong)index * 1000 + (ulong)tries));
            for (int t = 0; t < SceneLayout.MaxTries; t++)
            {
                double x = o.NominalX + (2 * rng.NextDouble() - 1) * o.JitterX;
                double y = o.NominalY + (2 * rng.NextDouble() - 1) * o.JitterY;
                double yaw = o.NominalYaw + (2 * rng.NextDouble() - 1) * o.JitterYaw;
                if (!_scene.FitsOnTable(index, x, y)) continue;
                bool clear = true;
                for (int k = 0; k < _targets.Length && clear; k++)
                {
                    if (k != index && _scene.Overlaps(index, x, y, k, _targets[k].X, _targets[k].Y)) clear = false;
                }

                if (!clear) continue;
                _targets[index] = _scene.PoseAt(index, x, y, yaw);
                return true;
            }

            return false;
        }

        private void Resolve(uint sequence, PlaceOutcome outcome, long now, WorldEditOp op, in ObjectPose pose)
        {
            _resolvedEditSequence = sequence;
            _editOutcome = (byte)outcome;
            _record?.Invoke(SessionLines.WorldEdit(now, sequence, op, pose, outcome));
        }

        private void RecordSnapshot(long now, bool force)
        {
            if (_record == null) return;
            bool moved = !_recordedOnce;
            for (int i = 0; i < _snapshotCount && !moved; i++)
            {
                ObjectPose a = _snapshot[i], b = _lastRecorded[i];
                double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
                double dot = Math.Abs(a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz);
                if (dx * dx + dy * dy + dz * dz > 1e-8 || 2 * Math.Acos(Math.Min(1.0, dot)) > 0.05 * Math.PI / 180) moved = true;
            }

            if (!force && !moved && now - _lastRecordedTicks < _tps) return;
            _snapshot.AsSpan(0, _snapshotCount).CopyTo(_lastRecorded);
            _recordedOnce = true;
            _lastRecordedTicks = now;
            _record(SessionLines.WorldObjects(now, _snapshotSensed, _snapshotRevision, _snapshot.AsSpan(0, _snapshotCount)));
        }
    }
}
