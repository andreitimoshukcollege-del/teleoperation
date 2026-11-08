using System;
using System.Net.Sockets;
using System.Text;
using Teleop.Core.Types;
using Teleop.RobotArm.World;
using Teleop.RobotHost.Backend;
using Teleop.RobotHost.World;

namespace Teleop.RobotHost.ArmServer
{
    /// <summary>A datagram link to an arm server: send, and receive without blocking.</summary>
    public interface IArmServerLink : IDisposable
    {
        void Send(ReadOnlySpan<byte> datagram);

        /// <summary>The next datagram waiting into <paramref name="buffer"/>; false when none is.</summary>
        bool TryReceive(Span<byte> buffer, out int bytes);
    }

    /// <summary>
    /// The <see cref="IArmBackend"/> on the far side of a socket (docs/adr/0030 decision 4): whatever owns the arm, be it
    /// the real FR3's arm server or the Isaac bridge (docs/adr/0031), speaking <see cref="ArmServerWire"/>. It forwards
    /// targets while enabled and keeps the newest state the server reports, every 4 ms. That state is also the
    /// recording's truth, through <see cref="TruthSink"/>.
    ///
    /// <para>Its motors are the shaper's, in the shaper's order: <see cref="Connect"/> refuses a server whose Welcome
    /// names others, or whose clock is not this machine's (the server stamps readings with RobotHost's clock, docs/adr/0030
    /// decision 4).</para>
    /// </summary>
    public sealed class ArmServerClient : IArmBackend, IWorldBackend, IDisposable
    {
        /// <summary>How far the server's clock may be from ours when it welcomes us: they must be the same clock.</summary>
        public const double MaxClockDisagreementSeconds = 0.05;

        private readonly IArmServerLink _link;
        private readonly byte[] _motorIds;
        private readonly long _ticksPerSecond;
        private readonly byte[] _send = new byte[ArmServerWire.MaxBytes];
        private readonly byte[] _receive = new byte[ArmServerWire.MaxBytes];
        private readonly byte[] _stateIds = new byte[ArmServerWire.MaxMotors];
        private readonly float[] _statePositions = new float[ArmServerWire.MaxMotors];
        private readonly float[] _stateVelocities = new float[ArmServerWire.MaxMotors];
        private readonly float[] _stateCommanded = new float[ArmServerWire.MaxMotors];
        private readonly float[] _position;
        private readonly double[] _truth;
        private readonly int[] _slot = new int[256];
        private uint _sequence;
        private bool _haveState;

        // The world (docs/adr/0031 amendment 2): the scene the server runs, a snapshot being assembled from its parts, the
        // newest complete one, and placement results not yet taken.
        private readonly byte[] _sceneSha = new byte[ArmServerWorld.Sha256Bytes];
        private readonly ObjectPose[] _assembling = new ObjectPose[WorldScene.MaxObjects];
        private readonly ObjectPose[] _snapshot = new ObjectPose[WorldScene.MaxObjects];
        private readonly ObjectPose[] _worldPart = new ObjectPose[ArmServerWorld.MaxObjectsPerWorld];
        private readonly ArmServerContact[] _contacts = new ArmServerContact[ArmServerWorld.MaxContacts];
        private readonly (uint Request, PlaceOutcome Outcome, ObjectPose Pose)[] _placed = new (uint, PlaceOutcome, ObjectPose)[64];
        private int _placedHead, _placedCount, _assembledCount, _snapshotCount;
        private uint _assemblingSnapshot, _snapshotRevision;
        private long _assemblingSensed, _snapshotSensed;
        private ulong _partsSeen;
        private bool _hasWorld, _snapshotFresh;

        public ArmServerClient(IArmServerLink link, ReadOnlySpan<byte> motorIds, long ticksPerSecond)
        {
            _link = link ?? throw new ArgumentNullException(nameof(link));
            // No motors: a server of the world alone (the JetRover's Isaac world beside its relay, docs/adr/0034 decision 7).
            if (motorIds.Length > ArmServerWire.MaxMotors) throw new ArgumentException($"at most {ArmServerWire.MaxMotors} motors", nameof(motorIds));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _motorIds = motorIds.ToArray();
            _ticksPerSecond = ticksPerSecond;
            _position = new float[_motorIds.Length];
            _truth = new double[_motorIds.Length];
            for (int i = 0; i < _slot.Length; i++) _slot[i] = -1;
            for (int m = 0; m < _motorIds.Length; m++) _slot[_motorIds[m]] = m;
            Enabled = true;
        }

        /// <summary>What the server said it is, once welcomed.</summary>
        public ArmServerKind Kind { get; private set; }

        public string Description { get; private set; } = "";

        public bool Enabled { get; private set; }

        /// <summary>The newest state's header, and when it arrived (host ticks); <see cref="long.MinValue"/> before any.</summary>
        public ArmServerStateHeader LastState { get; private set; }

        public long LastStateArrivalTicks { get; private set; } = long.MinValue;

        public ArmServerHealth LastHealth { get; private set; }

        public long StatesReceived { get; private set; }

        /// <summary>Datagrams that were not states, health or a welcome, or named motors this host does not have.</summary>
        public long Rejected { get; private set; }

        public Action<long, double[]>? TruthSink { get; set; }

        /// <summary>The server's state report's positions in this host's motor order, as of the newest report.</summary>
        public ReadOnlySpan<float> Positions => _position;

        /// <summary>
        /// Says hello until the server welcomes us (or <paramref name="timeoutSeconds"/> pass), and checks the welcome: the
        /// shaper's motors in the shaper's order, and the server's clock within <see cref="MaxClockDisagreementSeconds"/> of
        /// ours. Then waits for the first state, so the arm's pose is known before anything is commanded. With
        /// <paramref name="requireState"/> false it is done at the welcome: a server of the world alone, with no motors (the
        /// JetRover's Isaac world beside its relay, docs/adr/0034 decision 7).
        /// </summary>
        public bool Connect(Func<long> now, string profileName, double timeoutSeconds, Action<int>? sleep, out string error, bool requireState = true)
        {
            long deadline = now() + (long)(timeoutSeconds * _ticksPerSecond);
            long nextHello = long.MinValue;
            bool welcomed = false;
            while (now() < deadline)
            {
                long t = now();
                if (!welcomed && t >= nextHello)
                {
                    if (ArmServerWire.TryEncodeHello(++_sequence, t, _ticksPerSecond, profileName, _send, out int n)) _link.Send(_send.AsSpan(0, n));
                    nextHello = t + _ticksPerSecond / 2;
                }

                while (_link.TryReceive(_receive, out int bytes))
                {
                    ReadOnlySpan<byte> d = _receive.AsSpan(0, bytes);
                    if (!welcomed && ArmServerWire.TryDecodeWelcome(d, out ArmServerKind kind, out long serverNow, out ReadOnlySpan<byte> ids, out ReadOnlySpan<byte> text))
                    {
                        if (!ids.SequenceEqual(_motorIds))
                        {
                            error = $"the server's motors ({string.Join(",", ids.ToArray())}) are not this profile's, in its order ({string.Join(",", _motorIds)})";
                            return false;
                        }

                        double apart = Math.Abs(serverNow - now()) / (double)_ticksPerSecond;
                        if (apart > MaxClockDisagreementSeconds)
                        {
                            error = $"the server's clock is {apart:0.000} s from ours: it must stamp readings with this machine's monotonic clock";
                            return false;
                        }

                        Kind = kind;
                        Description = Encoding.UTF8.GetString(text);
                        welcomed = true;
                        continue;
                    }

                    if (welcomed) Handle(d, now());
                }

                if (welcomed && (_haveState || !requireState))
                {
                    error = "";
                    return true;
                }

                sleep?.Invoke(5);
            }

            error = welcomed ? "the server welcomed us but sent no state" : $"no arm server answered within {timeoutSeconds:0} s";
            return false;
        }

        public void Send(ReadOnlySpan<JointSample> targets, long hostTicks)
        {
            if (!Enabled) return;
            if (ArmServerWire.TryEncodeTargets(++_sequence, hostTicks, targets, _send, out int n)) _link.Send(_send.AsSpan(0, n));
        }

        public int ReadState(long hostTicks, Span<JointSample> output, out long sensedTicks)
        {
            Poll(hostTicks);
            sensedTicks = LastState.SensedTicks;
            if (!_haveState) return 0;
            int n = Math.Min(output.Length, _position.Length);
            for (int m = 0; m < n; m++) output[m] = new JointSample(_motorIds[m], _position[m], true);
            return n;
        }

        public void SetEnabled(bool enabled, long hostTicks)
        {
            if (ArmServerWire.TryEncodeEnable(++_sequence, hostTicks, enabled, _send, out int n)) _link.Send(_send.AsSpan(0, n));
            Enabled = enabled;
        }

        public void Poll(long hostTicks)
        {
            while (_link.TryReceive(_receive, out int bytes)) Handle(_receive.AsSpan(0, bytes), hostTicks);
        }

        public string Summary =>
            $"{StatesReceived} state report(s) from the {Kind} server" +
            (LastHealth.Steps > 0 ? $", {LastHealth.Steps} steps at {LastHealth.RealTimeFactor:0.0}x real time, {LastHealth.LateSteps} late" : "") +
            (Rejected > 0 ? $", {Rejected} datagram(s) rejected" : "");

        public void Dispose() => _link.Dispose();

        public bool HasWorld => _hasWorld;

        public ReadOnlySpan<byte> SceneSha256 => _hasWorld ? _sceneSha : ReadOnlySpan<byte>.Empty;

        public bool Physics => Kind == ArmServerKind.Isaac;

        /// <summary>The server's contact reports, as they come: sensed ticks, then each contact. Null: none.</summary>
        public Action<long, ReadOnlyMemory<ArmServerContact>>? ContactsSink { get; set; }

        public void Place(uint request, PlaceCause cause, in ObjectPose pose, long hostTicks)
        {
            if (ArmServerWorld.TryEncodePlace(++_sequence, hostTicks, request, cause, pose, _send, out int n)) _link.Send(_send.AsSpan(0, n));
        }

        public bool TryTakeSnapshot(Span<ObjectPose> poses, out int count, out long sensedTicks, out uint revision)
        {
            count = 0;
            sensedTicks = _snapshotSensed;
            revision = _snapshotRevision;
            if (!_snapshotFresh || poses.Length < _snapshotCount) return false;
            _snapshotFresh = false;
            _snapshot.AsSpan(0, _snapshotCount).CopyTo(poses);
            count = _snapshotCount;
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

        private bool HandleWorld(byte kind, ReadOnlySpan<byte> datagram)
        {
            if (kind == ArmServerWorld.KindWorldInfo && ArmServerWorld.TryDecodeWorldInfo(datagram, out ReadOnlySpan<byte> sha, out _))
            {
                sha.CopyTo(_sceneSha);
                _hasWorld = true;
                return true;
            }

            if (kind == ArmServerWorld.KindWorld &&
                ArmServerWorld.TryDecodeWorld(datagram, out long sensed, out uint snapshot, out uint revision, out byte part, out byte parts, _worldPart, out int count))
            {
                if (snapshot != _assemblingSnapshot || _partsSeen == 0)
                {
                    _assemblingSnapshot = snapshot;
                    _assemblingSensed = sensed;
                    _partsSeen = 0;
                    _assembledCount = 0;
                }

                int from = part * ArmServerWorld.MaxObjectsPerWorld;
                if (from + count > _assembling.Length) return true;
                _worldPart.AsSpan(0, count).CopyTo(_assembling.AsSpan(from));
                _assembledCount = Math.Max(_assembledCount, from + count);
                _partsSeen |= 1UL << part;
                if (_partsSeen == (parts >= 64 ? ulong.MaxValue : (1UL << parts) - 1))
                {
                    _assembling.AsSpan(0, _assembledCount).CopyTo(_snapshot);
                    _snapshotCount = _assembledCount;
                    _snapshotSensed = _assemblingSensed;
                    _snapshotRevision = revision;
                    _snapshotFresh = true;
                    _partsSeen = 0;
                }

                return true;
            }

            if (kind == ArmServerWorld.KindPlaced && ArmServerWorld.TryDecodePlaced(datagram, out uint request, out PlaceOutcome outcome, out _, out ObjectPose pose))
            {
                if (_placedCount < _placed.Length)
                {
                    _placed[(_placedHead + _placedCount) % _placed.Length] = (request, outcome, pose);
                    _placedCount++;
                }

                return true;
            }

            if (kind == ArmServerWorld.KindContacts && ArmServerWorld.TryDecodeContacts(datagram, out long at, _contacts, out int n))
            {
                ContactsSink?.Invoke(at, new ReadOnlyMemory<ArmServerContact>(_contacts, 0, n));
                return true;
            }

            return false;
        }

        private void Handle(ReadOnlySpan<byte> datagram, long arrival)
        {
            byte kind = ArmServerWire.KindOf(datagram);
            if (kind == ArmServerWire.KindState &&
                ArmServerWire.TryDecodeState(datagram, out ArmServerStateHeader header, _stateIds, _statePositions, _stateVelocities, _stateCommanded, out int count))
            {
                for (int i = 0; i < count; i++)
                {
                    int m = _slot[_stateIds[i]];
                    if (m < 0)
                    {
                        Rejected++;
                        return;
                    }
                }

                for (int i = 0; i < count; i++) _position[_slot[_stateIds[i]]] = _statePositions[i];
                LastState = header;
                LastStateArrivalTicks = arrival;
                StatesReceived++;
                _haveState = true;
                if (TruthSink != null)
                {
                    for (int m = 0; m < _position.Length; m++) _truth[m] = _position[m];
                    TruthSink(header.SensedTicks, _truth);
                }

                return;
            }

            if (kind == ArmServerWire.KindHealth && ArmServerWire.TryDecodeHealth(datagram, out ArmServerHealth health))
            {
                LastHealth = health;
                return;
            }

            if (HandleWorld(kind, datagram)) return;
            if (kind != ArmServerWire.KindWelcome) Rejected++;
        }
    }

    /// <summary>
    /// A Unix datagram socket to the arm server on this machine (docs/adr/0030 decision 4), bound to its own path so the
    /// server can answer. The client's path is removed on dispose. Linux, and anything else with AF_UNIX datagrams.
    /// </summary>
    public sealed class UdsArmServerLink : IArmServerLink
    {
        private readonly Socket _socket;
        private readonly UnixDomainSocketEndPoint _server;
        private readonly string _clientPath;

        public UdsArmServerLink(string serverPath, string clientPath)
        {
            _server = new UnixDomainSocketEndPoint(serverPath);
            _clientPath = clientPath;
            if (System.IO.File.Exists(clientPath)) System.IO.File.Delete(clientPath);
            _socket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified) { Blocking = false };
            _socket.Bind(new UnixDomainSocketEndPoint(clientPath));
        }

        public void Send(ReadOnlySpan<byte> datagram)
        {
            try
            {
                _socket.SendTo(datagram, SocketFlags.None, _server);
            }
            catch (SocketException)
            {
                // The server is not up yet, or restarting: the next send tries again. Connect reports a server that never answers.
            }
        }

        public bool TryReceive(Span<byte> buffer, out int bytes)
        {
            bytes = 0;
            if (_socket.Available <= 0) return false;
            try
            {
                bytes = _socket.Receive(buffer, SocketFlags.None);
                return bytes > 0;
            }
            catch (SocketException)
            {
                return false;
            }
        }

        public void Dispose()
        {
            _socket.Dispose();
            try
            {
                if (System.IO.File.Exists(_clientPath)) System.IO.File.Delete(_clientPath);
            }
            catch (System.IO.IOException)
            {
            }
        }
    }
}
