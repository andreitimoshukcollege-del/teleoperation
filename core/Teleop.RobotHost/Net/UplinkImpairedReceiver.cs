using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;

namespace Teleop.RobotHost.Net
{
    /// <summary>
    /// A physically impaired uplink for recorded sessions (docs/adr/0021 decision 4): what the robot
    /// reads from the joint socket passes through Core's own <see cref="EmulatedTransport"/>, with a
    /// named profile from <see cref="NetworkProfileCatalog"/>, before the plant sees it. Uplink jitter
    /// and loss change which target the relay's newest-wins gate writes, so they cannot be synthesised
    /// afterwards from a LAN recording; they have to happen to the real arm.
    ///
    /// <para><b>Receive side only.</b> Wrapping the whole socket in an <c>EmulatedTransport</c> would
    /// also drop joint-state frames on their way out (loss is decided at send). Here the socket's
    /// arrivals are fed into an emulator over a loopback, stamped with when they were read, and taken
    /// out when the profile releases them; frames still go out through the socket untouched.</para>
    ///
    /// <para>Off (the default, or the profile <c>none</c> / <c>lan</c>), every datagram passes straight
    /// through, read and delivered at the same tick. Changing the profile by hand drops whatever the
    /// previous one was still holding: a profile switch is an outage, as on a real link that changes.
    /// A schedule's switch (<see cref="UplinkSchedule"/>, docs/adr/0021 decision 13) instead lets the
    /// previous profile deliver what it holds, when it is due: a datagram's fate is decided by the
    /// profile it was read under, which is how the trainer's simulator and its true conditions see a
    /// schedule, so a scheduled session has no outage at its switches that nothing else would
    /// have.</para>
    /// </summary>
    public sealed class UplinkImpairedReceiver
    {
        private const int StampBytes = 8;
        private const int Capacity = 4096;

        private readonly ITransport _socket;
        private readonly long _ticksPerSecond;
        private readonly int _maxDatagramBytes;
        private readonly byte[] _scratch;
        private EmulatedTransport? _emulator;
        private readonly List<EmulatedTransport> _draining = new List<EmulatedTransport>();

        public UplinkImpairedReceiver(ITransport socket, long ticksPerSecond, int maxDatagramBytes)
        {
            _socket = socket ?? throw new ArgumentNullException(nameof(socket));
            if (ticksPerSecond <= 0) throw new ArgumentOutOfRangeException(nameof(ticksPerSecond));
            _ticksPerSecond = ticksPerSecond;
            _maxDatagramBytes = maxDatagramBytes;
            _scratch = new byte[maxDatagramBytes + StampBytes];
        }

        /// <summary>The active profile's name, or <c>none</c>.</summary>
        public string Profile { get; private set; } = "none";

        public ulong Seed { get; private set; }

        /// <summary>Profiles switched away from that still hold datagrams.</summary>
        public int DrainingCount => _draining.Count;

        /// <summary>
        /// Switches the impairment. <c>none</c> or <c>lan</c> turns it off. Throws on a name Core's
        /// catalog does not know, so a typo never silently records an unimpaired session as impaired.
        /// With <paramref name="drainPrevious"/>, what the previous profile holds is still delivered
        /// when due; without it, it is dropped.
        /// </summary>
        public void SetProfile(string name, ulong seed, bool drainPrevious = false)
        {
            if (drainPrevious && _emulator != null && _emulator.InFlightCount > 0)
            {
                _draining.Add(_emulator);
            }

            if (name == "none" || name == "lan")
            {
                _emulator = null;
                Profile = "none";
                Seed = seed;
                return;
            }

            if (!NetworkProfileCatalog.TryResolveImpairments(name, _ticksPerSecond, out INetworkImpairment[] impairments, out string? error))
            {
                throw new ArgumentException(error ?? $"unknown network profile '{name}'", nameof(name));
            }

            _emulator = new EmulatedTransport(new LoopbackTransport(_maxDatagramBytes + StampBytes, Capacity), impairments, seed, Capacity);
            Profile = name;
            Seed = seed;
        }

        /// <summary>
        /// The next datagram the plant should see: <paramref name="socketTicks"/> is when it was read
        /// from the socket, <paramref name="deliverTicks"/> when the impairment released it (equal when
        /// off).
        /// </summary>
        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long socketTicks, out long deliverTicks)
        {
            // Datagrams a previous profile still holds come out when due, before anything newer.
            for (int i = 0; i < _draining.Count; i++)
            {
                if (TryTake(_draining[i], nowTicks, destination, out byteCount, out socketTicks, out deliverTicks))
                {
                    return true;
                }

                if (_draining[i].InFlightCount == 0)
                {
                    _draining.RemoveAt(i);
                    i--;
                }
            }

            if (_emulator == null)
            {
                bool got = _socket.TryReceive(nowTicks, destination, out byteCount, out long arrival);
                socketTicks = deliverTicks = arrival;
                return got;
            }

            // Everything that has arrived goes into the emulator, stamped with when it was read.
            while (_socket.TryReceive(nowTicks, _scratch.AsSpan(StampBytes), out int count, out long arrival))
            {
                BinaryPrimitives.WriteInt64LittleEndian(_scratch, arrival);
                _emulator.Send(_scratch.AsSpan(0, StampBytes + count), arrival);
            }

            return TryTake(_emulator, nowTicks, destination, out byteCount, out socketTicks, out deliverTicks);
        }

        private bool TryTake(EmulatedTransport emulator, long nowTicks, Span<byte> destination, out int byteCount,
                             out long socketTicks, out long deliverTicks)
        {
            byteCount = 0;
            socketTicks = deliverTicks = 0;
            if (!emulator.TryReceive(nowTicks, _scratch, out int stamped, out long _) || stamped < StampBytes)
            {
                return false;
            }

            socketTicks = BinaryPrimitives.ReadInt64LittleEndian(_scratch);
            // When the plant takes it (this loop), which is at or after the moment it was due.
            deliverTicks = nowTicks;
            byteCount = stamped - StampBytes;
            _scratch.AsSpan(StampBytes, byteCount).CopyTo(destination);
            return true;
        }
    }
}
