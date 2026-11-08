using System;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Wire;
using Teleop.RobotArm.World;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.Operator
{
    /// <summary>
    /// What rides the joint port beside the arm's state, for any robot's operator link (docs/adr/0034 decision 7): the world
    /// (docs/adr/0031 amendment 2, <see cref="WorldCodec"/>) and scenario recording (docs/adr/0033, <see cref="RecordCodec"/>).
    /// The FR3's <see cref="SerialChainJointLink"/> and the JetRover's link both hand it their datagrams, so both robots' headsets
    /// see the same world and the same RECORD button.
    /// <list type="bullet">
    /// <item><b>World frames</b> wait out the link's added delay, as joint frames do, so objects and arms cross the disturbed network
    /// together (<see cref="Release"/>). The status applies at once.</item>
    /// <item><b>Requests</b> (a world edit, a record request) are re-sent every 0.25 s until the status echoes them, given up
    /// after 2 s, one of each at a time.</item>
    /// <item><b>Recording</b> status and list apply at once: control, not robot state.</item>
    /// </list>
    /// Allocation-free once constructed, a scenario list's names aside (made only when its revision changes).
    /// </summary>
    public sealed class JointPortSideChannels
    {
        private const int WorldQueueCapacity = 512;
        private readonly ITransport _transport;
        private readonly ITimeAuthority _clock;
        private readonly WorldState? _world;
        private readonly long[] _worldRelease = new long[WorldQueueCapacity];
        private readonly long[] _worldSensed = new long[WorldQueueCapacity];
        private readonly int[] _worldCounts = new int[WorldQueueCapacity];
        private readonly ObjectPose[] _worldPoses = new ObjectPose[WorldQueueCapacity * WorldCodec.MaxObjectsPerFrame];
        private readonly ObjectPose[] _worldPart = new ObjectPose[WorldCodec.MaxObjectsPerFrame];
        private readonly byte[] _worldSha = new byte[WorldCodec.Sha256Bytes];
        private readonly byte[] _editBuffer = new byte[WorldCodec.EditBytes];
        private readonly byte[] _sizeIds = new byte[WorldScene.MaxObjects];
        private readonly float[] _sizeScales = new float[WorldScene.MaxObjects];
        private int _worldHead, _worldSize;
        private uint _editSequence;
        private long _editFirstSent = long.MinValue, _editLastSent = long.MinValue;
        private WorldEditOp _editOp;
        private ObjectPose _editPose;

        private readonly RecordingState _recording = new RecordingState();
        private readonly byte[] _recordBuffer = new byte[RecordCodec.RequestBytes];
        private uint _recordSequence;
        private long _recordFirstSent = long.MinValue, _recordLastSent = long.MinValue;
        private RecordOp _recordOp;
        private byte _recordScenario;
        private ushort _recordRevision;

        public JointPortSideChannels(ITransport transport, ITimeAuthority clock, WorldState? world)
        {
            _transport = transport ?? throw new ArgumentNullException(nameof(transport));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _world = world;
        }

        /// <summary>The world as the headset sees it; null without a scene file.</summary>
        public WorldState? World => _world;

        public bool WorldEditPending => _editFirstSent != long.MinValue;

        /// <summary>The last edit that went unanswered for 2 s (0: none).</summary>
        public uint WorldEditTimedOut { get; private set; }

        /// <summary>Scenario recording as RobotHost reports it; never null, empty until a status arrives.</summary>
        public RecordingState Recording => _recording;

        public bool RecordRequestPending => _recordFirstSent != long.MinValue;

        /// <summary>The last record request that went unanswered for 2 s (0: none).</summary>
        public uint RecordRequestTimedOut { get; private set; }

        /// <summary>The sequence of the last record request asked (0: none yet).</summary>
        public uint RecordSequence => _recordSequence;

        /// <summary>Asks RobotHost for a world edit: false while another is waiting, or without a world.</summary>
        public bool RequestWorldEdit(WorldEditOp op, in ObjectPose pose, long now)
        {
            if (_world == null || WorldEditPending) return false;
            _editSequence = _editSequence == uint.MaxValue ? 1 : _editSequence + 1;
            _editOp = op;
            _editPose = pose;
            _editFirstSent = now;
            _editLastSent = long.MinValue;
            return true;
        }

        /// <summary>Asks RobotHost to start, stop or discard a take: false while another is waiting, or before the list matches the status.</summary>
        public bool RequestRecord(RecordOp op, int scenario, long now)
        {
            if (RecordRequestPending || !_recording.HasStatus) return false;
            if (op == RecordOp.Start && (!_recording.ListCurrent || scenario < 0 || scenario >= _recording.Count)) return false;
            _recordSequence = _recordSequence == uint.MaxValue ? 1 : _recordSequence + 1;
            _recordOp = op;
            _recordScenario = (byte)Math.Max(0, Math.Min(254, scenario));
            _recordRevision = _recording.ListRevision;
            _recordFirstSent = now;
            _recordLastSent = long.MinValue;
            return true;
        }

        /// <summary>Each tick, before receiving: re-sends what waits for an echo, and gives up on what waited 2 s.</summary>
        public void SendDue(long now)
        {
            long tps = _clock.TicksPerSecond;
            if (_editFirstSent != long.MinValue)
            {
                if (_world!.HasStatus && _world.Status.EditSequence == _editSequence)
                {
                    _editFirstSent = long.MinValue;
                }
                else if (now - _editFirstSent > 2 * tps)
                {
                    WorldEditTimedOut = _editSequence;
                    _editFirstSent = long.MinValue;
                }
                else if ((_editLastSent == long.MinValue || now - _editLastSent >= tps / 4) &&
                         WorldCodec.TryEncodeEdit(_editOp, _editPose.Id, _editSequence, now, _editPose, _editBuffer, out int editBytes))
                {
                    _transport.Send(_editBuffer.AsSpan(0, editBytes), now);
                    _editLastSent = now;
                }
            }

            if (_recordFirstSent != long.MinValue)
            {
                if (_recording.HasStatus && _recording.Status.Sequence == _recordSequence)
                {
                    _recordFirstSent = long.MinValue;
                }
                else if (now - _recordFirstSent > 2 * tps)
                {
                    RecordRequestTimedOut = _recordSequence;
                    _recordFirstSent = long.MinValue;
                }
                else if ((_recordLastSent == long.MinValue || now - _recordLastSent >= tps / 4) &&
                         RecordCodec.TryEncodeRequest(_recordOp, _recordScenario, _recordRevision, _recordSequence, now, _recordBuffer, out int recordBytes))
                {
                    _transport.Send(_recordBuffer.AsSpan(0, recordBytes), now);
                    _recordLastSent = now;
                }
            }
        }

        /// <summary>
        /// One received datagram: true when it was one of these (a world frame or status, a record status or list), so the link
        /// reads it no further. A world frame waits <paramref name="addedDelayMs"/> from <paramref name="arrival"/>.
        /// </summary>
        public bool TryHandle(ReadOnlySpan<byte> datagram, long arrival, double addedDelayMs)
        {
            if (datagram.Length == 0) return false;
            byte tag = datagram[0];
            if (tag == RecordCodec.StatusTag)
            {
                if (RecordCodec.TryDecodeStatus(datagram, out RecordStatus status, out long sent)) _recording.ApplyStatus(status, sent);
                return true;
            }

            if (tag == RecordCodec.ListTag)
            {
                _recording.ApplyListPart(datagram);
                return true;
            }

            if (_world == null) return false;
            if (tag == WorldCodec.FrameTag)
            {
                if (WorldCodec.TryDecodeFrame(datagram, out _, out long sensed, out _, out _, _worldPart, out int objects))
                {
                    EnqueueWorld(arrival + (long)(addedDelayMs / 1000.0 * _clock.TicksPerSecond), sensed, objects);
                }

                return true;
            }

            if (tag == WorldCodec.SizesTag)
            {
                if (WorldCodec.TryDecodeSizes(datagram, out uint revision, _sizeIds, _sizeScales, out int n))
                {
                    _world.ApplySizes(revision, _sizeIds.AsSpan(0, n), _sizeScales.AsSpan(0, n));
                }

                return true;
            }

            if (tag == WorldCodec.StatusTag)
            {
                if (WorldCodec.TryDecodeStatus(datagram, out WorldStatus status, _worldSha, out _)) _world.ApplyStatus(status, _worldSha);
                return true;
            }

            return false;
        }

        /// <summary>Each tick, after receiving: the world frames whose added delay is over reach the world.</summary>
        public void Release(long now)
        {
            while (_worldSize > 0 && _worldRelease[_worldHead] <= now)
            {
                int slot = _worldHead;
                for (int i = 0; i < _worldCounts[slot]; i++) _world!.Apply(_worldPoses[slot * WorldCodec.MaxObjectsPerFrame + i], _worldSensed[slot]);
                _worldHead = (_worldHead + 1) % WorldQueueCapacity;
                _worldSize--;
            }
        }

        private void EnqueueWorld(long releaseTicks, long sensedRobotTicks, int count)
        {
            if (_worldSize == WorldQueueCapacity)
            {
                _worldHead = (_worldHead + 1) % WorldQueueCapacity;
                _worldSize--;
            }

            int slot = (_worldHead + _worldSize) % WorldQueueCapacity;
            _worldRelease[slot] = releaseTicks;
            _worldSensed[slot] = sensedRobotTicks;
            _worldCounts[slot] = count;
            Array.Copy(_worldPart, 0, _worldPoses, slot * WorldCodec.MaxObjectsPerFrame, count);
            _worldSize++;
        }
    }
}
