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
    /// <item><b>Pools</b> (docs/adr/0037 decision 3). For a scene whose objects are a pool, the layout also chooses which take
    /// part: the rest are sent to the backend with <see cref="PlaceCause.Park"/>. A layout spot the backend refuses is redrawn
    /// anywhere in the region (by the edge for an object drawn there, on its drawn rest: docs/adr/0037 amendment 6), and an
    /// object that finds none is parked. Reset puts the layout back as it was drawn, lying objects lying; a parked
    /// object cannot be moved or resized from the headset. A pool that draws sizes (docs/adr/0037 amendment 2) changes them
    /// on the shelf: a new layout parks every object, resizes there each one whose drawn size differs, then places the
    /// drawn ones, so the table is empty while it is filled.</item>
    /// <item><b>Edits.</b> Moves and resets from the headset (joint port <c>0x85</c>), each applied once however often it is
    /// re-sent; the status echoes its sequence and outcome once settled. <c>joint-drive</c>'s replays also park one object (op 6) or
    /// stand a parked one on the table (op 7, listed in <c>WL</c> until a reset parks it again), docs/adr/0038 decision 2.</item>
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
        private readonly RigidTransform[] _layoutTargets;
        private readonly bool[] _parked;
        private readonly bool[] _added; // a parked object stood on the table by an unpark (op 7) until a reset or a new layout
        private readonly double[] _layoutScales;
        private readonly byte[] _activeIds;
        private readonly ObjectPose[] _snapshot;
        private readonly ObjectPose[] _lastRecorded;
        private readonly ObjectPose[] _part = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
        private readonly byte[] _datagram = new byte[JointDatagram.MaxBytes];
        private readonly Pending[] _pending = new Pending[128];
        private readonly byte[] _sizeIds;
        private readonly float[] _sizeScales;
        private readonly int[] _parkGroup = new int[WorldScene.MaxObjects];
        private int _parkCount;
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
            public WorldEditOp Op; // the edit's op, for its record; Move for any placement that is not an edit's park, resize or unpark
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
            _layoutTargets = new RigidTransform[scene.Objects.Length];
            _parked = new bool[scene.Objects.Length];
            _added = new bool[scene.Objects.Length];
            _layoutScales = new double[scene.Objects.Length];
            _activeIds = new byte[scene.Objects.Length];
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

        /// <summary>Whether the current layout leaves object <paramref name="index"/> out (a pool's, docs/adr/0037 decision 3).</summary>
        public bool IsParked(int index) => _parked[index];

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
            _record?.Invoke(LayoutLine(now));
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
                bool wasAdded = false;
                for (int i = 0; i < _targets.Length; i++)
                {
                    _targets[i] = _scene.IsPool ? _layoutTargets[i] : _scene.Objects[i].Nominal;
                    SendPlace(i, _parked[i] ? PlaceCause.Park : PlaceCause.Reset, 0, now); // an unparked object (op 7) goes back to the shelf
                    wasAdded |= _added[i];
                    _added[i] = false;
                }

                if (wasAdded) _record?.Invoke(LayoutLine(now));
                outcome = PlaceOutcome.Applied;
            }
            else if (op == WorldEditOp.Unpark)
            {
                // docs/adr/0038, a replay's add when no object on the table will do: one object the layout left out stood on the table for
                // now, as a reset places one (the placement rules decide), and listed in WL while it stands there; reset-layout parks it
                // again. Only one the headset may move, and only from the shelf (an object on the table moves by op 1).
                int index = _scene.IndexOf(pose.Id);
                if (index < 0) outcome = PlaceOutcome.UnknownObject;
                else if (!_scene.Objects[index].OperatorMovable || !_parked[index]) outcome = PlaceOutcome.NotMovable;
                else if (!pose.IsValid()) outcome = PlaceOutcome.InvalidPose;
                else
                {
                    _targets[index] = pose.ToTransform();
                    SendPlace(index, PlaceCause.Reset, sequence, now, op: WorldEditOp.Unpark);
                    _record?.Invoke(SessionLines.WorldEdit(now, sequence, op, pose, null));
                    return; // settles when the backend answers
                }
            }
            else if (op == WorldEditOp.Park)
            {
                // docs/adr/0038 decision 2: one object off the table for a replay, stood on the shelf as a layout's parked objects
                // are (any object: a fixture too); reset-object or reset-layout brings it back. The layout stays as drawn.
                int index = _scene.IndexOf(pose.Id);
                if (index < 0) outcome = PlaceOutcome.UnknownObject;
                else if (_parked[index]) outcome = PlaceOutcome.NotMovable;
                else
                {
                    _targets[index] = _scene.ParkedPose(index, _sizeScales[index]);
                    SendPlace(index, PlaceCause.Park, sequence, now);
                    _record?.Invoke(SessionLines.WorldEdit(now, sequence, op, pose, null));
                    return; // settles when the backend answers
                }
            }
            else if (op == WorldEditOp.Resize)
            {
                int index = _scene.IndexOf(pose.Id);
                if (index < 0) outcome = PlaceOutcome.UnknownObject;
                else if (!_scene.Objects[index].OperatorMovable || _parked[index]) outcome = PlaceOutcome.NotMovable;
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
                else if (!_scene.Objects[index].OperatorMovable || _parked[index]) outcome = PlaceOutcome.NotMovable;
                else if (op == WorldEditOp.Move && !pose.IsValid()) outcome = PlaceOutcome.InvalidPose;
                else
                {
                    RigidTransform home = _scene.IsPool ? _layoutTargets[index] : _scene.Objects[index].Nominal;
                    _targets[index] = op == WorldEditOp.ResetObject ? home : pose.ToTransform();
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
            Array.Clear(_added, 0, _added.Length);
            LayoutNumber++;
            LayoutSeed = unchecked(_baseSeed + LayoutNumber - 1);
            int missed = SceneLayout.Draw(_scene, LayoutSeed, _targets, _parked, _layoutScales);
            _targets.AsSpan().CopyTo(_layoutTargets);
            _record?.Invoke(LayoutLine(now));
            string what = _scene.IsPool
                ? $", {_targets.Length - ParkedCount()} of {_targets.Length} objects on the table{(missed > 0 ? $" ({missed} found no spot)" : "")}"
                : missed > 0 ? $", {missed} object(s) at their nominal pose" : "";
            _log?.Invoke($"[world] layout {LayoutNumber} (seed {LayoutSeed}){what}");
            if (_scene.DrawsSizes)
            {
                // Sizes change on the shelf, never on the table: park every object, resize there, then place the drawn ones.
                // The backend takes these in order, so each placement meets only the objects already placed, at their new sizes.
                for (int i = 0; i < _targets.Length; i++) SendPlace(i, PlaceCause.Park, 0, now);
                for (int i = 0; i < _targets.Length; i++)
                {
                    if (!_parked[i] && _scene.Resizable(i) && Math.Abs(_layoutScales[i] - _sizeScales[i]) > 1e-4 * _layoutScales[i])
                    {
                        SendResize(i, (float)_layoutScales[i], 0, now);
                    }
                }

                PlaceDrawn(now);
                return;
            }

            if (_scene.Mixed)
            {
                for (int i = 0; i < _targets.Length; i++)
                {
                    if (_parked[i]) SendPlace(i, PlaceCause.Park, 0, now);
                }

                PlaceDrawn(now);
                return;
            }

            for (int i = 0; i < _targets.Length; i++) SendPlace(i, _parked[i] ? PlaceCause.Park : PlaceCause.Layout, 0, now);
        }

        /// <summary>
        /// Places the layout's objects that are on the table, in file order. In a mixed pool (docs/adr/0038) those standing on the table go
        /// first and those raised on them after (an obstacle on its stand, a plank on its supports), so each placement meets what it rests on.
        /// </summary>
        private void PlaceDrawn(long now)
        {
            for (int pass = 0; pass < (_scene.Mixed ? 2 : 1); pass++)
            {
                for (int i = 0; i < _targets.Length; i++)
                {
                    if (!_parked[i] && IsRaised(i) == (pass == 1)) SendPlace(i, PlaceCause.Layout, 0, now);
                }
            }
        }

        /// <summary>Whether the layout raises object <paramref name="index"/> above the table (docs/adr/0038): on a stand, or a plank on its supports.</summary>
        public bool IsRaised(int index)
        {
            int rest = WorldScene.RestOf(_layoutTargets[index]);
            return !_parked[index] && _scene.IsElevated(index, _layoutTargets[index].Z, ScaleFor(index), rest);
        }

        /// <summary>The size a layout redraw checks object <paramref name="index"/> at: the drawn one for a pool that draws sizes, else the backend's.</summary>
        private double ScaleFor(int index) => _scene.DrawsSizes && !_parked[index] ? _layoutScales[index] : _sizeScales[index];

        private int ParkedCount()
        {
            int n = 0;
            foreach (bool p in _parked) n += p ? 1 : 0;
            return n;
        }

        private string LayoutLine(long now)
        {
            if (!_scene.IsPool) return SessionLines.WorldLayout(now, LayoutNumber, LayoutSeed);
            int n = 0;
            for (int i = 0; i < _parked.Length; i++)
            {
                if (!_parked[i] || _added[i]) _activeIds[n++] = _scene.Objects[i].Id;
            }

            return SessionLines.WorldLayout(now, LayoutNumber, LayoutSeed, _activeIds.AsSpan(0, n));
        }

        private void SendPlace(int index, PlaceCause cause, uint edit, long now, int tries = 0, WorldEditOp op = WorldEditOp.Move)
        {
            uint request = ++_request;
            int slot = Array.FindIndex(_pending, p => !p.Active);
            if (slot < 0) slot = (int)(request % (uint)_pending.Length);
            _pending[slot] = new Pending { Active = true, Request = request, Index = index, Cause = cause, Tries = tries, Sent = now, Edit = edit, Op = op };
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
            if (outcome != PlaceOutcome.Applied && done.Cause == PlaceCause.Layout && outcome != PlaceOutcome.TimedOut && _scene.Mixed &&
                (_parked[done.Index] || ParkGroup(done.Index, now)))
            {
                // docs/adr/0038: part of an obstacle on its stand, or of a plank on its supports, parked whole; or the answer to one
                // its group's refusal parked already (an object sent onto a stand that was then refused).
                return;
            }

            if (outcome != PlaceOutcome.Applied && done.Cause == PlaceCause.Layout && outcome != PlaceOutcome.TimedOut && done.Tries < MaxRedraws &&
                Redraw(done.Index, done.Tries))
            {
                SendPlace(done.Index, PlaceCause.Layout, 0, now, done.Tries + 1);
                return;
            }

            if (outcome != PlaceOutcome.Applied && done.Cause == PlaceCause.Layout && outcome != PlaceOutcome.TimedOut && _scene.IsPool)
            {
                // No spot the backend takes: out of this layout, rather than wherever it stood before.
                _parked[done.Index] = true;
                _targets[done.Index] = _layoutTargets[done.Index] = _scene.ParkedPose(done.Index);
                _record?.Invoke(LayoutLine(now));
                SendPlace(done.Index, PlaceCause.Park, 0, now);
                return;
            }

            if (done.Op == WorldEditOp.Unpark)
            {
                // Standing on the table now, listed in the layout until a reset parks it; or refused, and still on the shelf.
                if (outcome == PlaceOutcome.Applied)
                {
                    _added[done.Index] = true;
                    _record?.Invoke(LayoutLine(now));
                }
                else
                {
                    _targets[done.Index] = _layoutTargets[done.Index];
                }
            }

            if (done.Edit != 0)
            {
                Resolve(done.Edit, outcome, now, done.Cause == PlaceCause.Resize ? WorldEditOp.Resize : done.Cause == PlaceCause.Park ? WorldEditOp.Park : done.Op, pose);
            }
        }

        /// <summary>
        /// In a mixed pool (docs/adr/0038), parks object <paramref name="index"/> with what it belongs to when the backend refused it and it
        /// is part of a group: a plank or a support (the plank and all its supports), a stand (with whatever the layout raised on it), or an
        /// object raised on a stand (itself: its stand stays, a box on the table). A redraw would split the group. False, parking nothing,
        /// for an object that stands alone on the table, which is redrawn as in any pool.
        /// </summary>
        private bool ParkGroup(int index, long now)
        {
            SceneObject o = _scene.Objects[index];
            int plank = o.Role == SceneRole.Plank ? index : -1;
            for (int k = 0; k < _targets.Length && o.Role == SceneRole.Support && plank < 0; k++)
            {
                if (_scene.Objects[k].Role == SceneRole.Plank && Array.IndexOf(_scene.Objects[k].SupportIds, o.Id) >= 0) plank = k;
            }

            bool grouped = plank >= 0 || o.Role == SceneRole.Stand || IsRaised(index);
            if (!grouped) return false;
            for (int k = 0; k < _targets.Length; k++)
            {
                if (_parked[k]) continue;
                bool member = k == index;
                if (plank >= 0) member |= k == plank || Array.IndexOf(_scene.Objects[plank].SupportIds, _scene.Objects[k].Id) >= 0;
                if (o.Role == SceneRole.Stand && k != index && IsRaised(k))
                {
                    int restK = WorldScene.RestOf(_layoutTargets[k]);
                    int restS = WorldScene.RestOf(_layoutTargets[index], out double yawS);
                    member |= _scene.HoldsOnTop(index, _layoutTargets[index].X, _layoutTargets[index].Y, _layoutTargets[index].Z, yawS, ScaleFor(index), restS,
                        k, _layoutTargets[k].X, _layoutTargets[k].Y, _layoutTargets[k].Z, ScaleFor(k), restK);
                }

                if (member) _parkGroup[_parkCount++] = k;
            }

            for (int q = 0; q < _parkCount; q++)
            {
                int k = _parkGroup[q];
                _parked[k] = true;
                _targets[k] = _layoutTargets[k] = _scene.ParkedPose(k);
            }

            _record?.Invoke(LayoutLine(now));
            for (int q = 0; q < _parkCount; q++) SendPlace(_parkGroup[q], PlaceCause.Park, 0, now);
            _parkCount = 0;
            return true;
        }

        /// <summary>
        /// Another spot for object <paramref name="index"/>, clear of the others' targets: within its jitter, or for a pool anywhere
        /// in the region (a movable one in the reach band, or by the edge if it was drawn there; clutter anywhere, as its own
        /// rectangle at its yaw, docs/adr/0037 "Raised clutter"), on the rest it was drawn on.
        /// </summary>
        private bool Redraw(int index, int tries)
        {
            SceneObject o = _scene.Objects[index];
            var rng = new SeededRng(unchecked(LayoutSeed * 0x9E3779B97F4A7C15UL + (ulong)index * 1000 + (ulong)tries));
            // Its drawn rest, and the bench's edge if it was drawn there (docs/adr/0037 amendment 6), are kept.
            int rest = WorldScene.RestOf(_layoutTargets[index]);
            bool atEdge = _scene.IsPool && o.Movable && _scene.InEdgeRegions(_layoutTargets[index].X, _layoutTargets[index].Y);
            for (int t = 0; t < (_scene.IsPool ? SceneLayout.MaxPoolTries : SceneLayout.MaxTries); t++)
            {
                double x, y, yaw;
                if (_scene.Mixed)
                {
                    // docs/adr/0038: an obstacle anywhere over the arm's reachable band, a distractor anywhere in its regions.
                    if (o.Role == SceneRole.Distractor) _scene.DrawDistractorSpot(ref rng, out x, out y, out yaw);
                    else _scene.DrawObstacleSpot(ref rng, out x, out y, out yaw);
                    double z = _scene.RestingZ(index, ScaleFor(index), rest);
                    if (!_scene.FitsOnTable(index, x, y, yaw, ScaleFor(index), rest)) continue;
                    bool free = true;
                    for (int k = 0; k < _targets.Length && free; k++)
                    {
                        if (k == index || _parked[k]) continue;
                        int restK = WorldScene.RestOf(_targets[k], out double yawK);
                        free = !_scene.OverlapsAtHeights(index, x, y, z, yaw, k, _targets[k].X, _targets[k].Y, _targets[k].Z, yawK, ScaleFor(index), ScaleFor(k), rest, restK);
                    }

                    if (!free) continue;
                    _targets[index] = _layoutTargets[index] = _scene.PoseAt(index, x, y, yaw, ScaleFor(index), rest);
                    return true;
                }

                if (o.Clutter)
                {
                    // Raised clutter (docs/adr/0037 "Raised clutter"): anywhere in the region, its own rectangle at its yaw.
                    if (!_scene.DrawClutterSpot(index, ref rng, out x, out y, out yaw)) continue;
                }
                else if (atEdge)
                {
                    if (!_scene.DrawEdgeSpot(index, ScaleFor(index), rest, ref rng, out x, out y, out yaw)) continue;
                }
                else if (_scene.IsPool)
                {
                    if (!_scene.DrawSpot(index, ScaleFor(index), rest, ref rng, out x, out y, out yaw)) continue;
                }
                else
                {
                    x = o.NominalX + (2 * rng.NextDouble() - 1) * o.JitterX;
                    y = o.NominalY + (2 * rng.NextDouble() - 1) * o.JitterY;
                    yaw = o.NominalYaw + (2 * rng.NextDouble() - 1) * o.JitterYaw;
                }

                if (!_scene.FitsOnTable(index, x, y, yaw, ScaleFor(index), rest) || (_scene.IsPool && !atEdge && !_scene.InReach(index, x, y))) continue;
                bool clear = true;
                for (int k = 0; k < _targets.Length && clear; k++)
                {
                    if (k == index || _parked[k]) continue;
                    int restK = WorldScene.RestOf(_targets[k], out double yawK);
                    if (_scene.Overlaps(index, x, y, yaw, k, _targets[k].X, _targets[k].Y, yawK, ScaleFor(index), ScaleFor(k), rest, restK)) clear = false;
                }

                if (!clear) continue;
                _targets[index] = _scene.PoseAt(index, x, y, yaw, ScaleFor(index), rest);
                if (_scene.IsPool) _layoutTargets[index] = _targets[index];
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
