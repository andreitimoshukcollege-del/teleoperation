using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport;

// C# 9: block-scoped namespace only, like the rest of unity/.
namespace Teleop.Bridge
{
    /// <summary>
    /// A network disturbance on the headset's joint socket, in both directions (docs/adr/0023
    /// decision 3), switched on and off while running. Wraps the real UDP transport, so the joint
    /// commands, the keepalives (and so ClockSync's round trips) and the robot's frames all go
    /// through it.
    /// <list type="bullet">
    /// <item><b>Uplink:</b> each datagram sent goes into a Core <see cref="EmulatedTransport"/> over a
    /// <see cref="LoopbackTransport"/>, and leaves on the real socket when that emulator says it
    /// arrives. The robot then receives it late, or never.</item>
    /// <item><b>Downlink:</b> each frame received goes into a second emulator the same way, and is
    /// handed on when it emerges, with the emulated arrival as its arrival time.</item>
    /// </list>
    /// Every delay, jitter and drop decision is Core's (the impairments are built by
    /// <see cref="NetworkImpairmentSettings.CreateImpairments"/> or <see cref="NetworkProfileCatalog"/>);
    /// this class only routes datagrams through them, the same pass-through RobotHost's
    /// <c>UplinkImpairedReceiver</c> uses on the robot. A setting change keeps the previous emulator
    /// draining until it is empty, so what was in flight still arrives. Plain C#, not thread-safe,
    /// allocation-free per datagram (a setting change builds new emulators).
    /// </summary>
    public sealed class JointLinkDisturbance : ITransport
    {
        private const int Capacity = 512;

        private readonly ITransport _inner;
        private readonly byte[] _buffer;
        private Stage _up, _down, _upDraining, _downDraining;

        public JointLinkDisturbance(ITransport inner)
        {
            _inner = inner ?? throw new ArgumentNullException(nameof(inner));
            _buffer = new byte[inner.MaxPayloadBytes];
            Description = "off";
        }

        public int MaxPayloadBytes => _inner.MaxPayloadBytes;

        /// <summary>True while either direction is disturbed.</summary>
        public bool IsActive => _up != null || _down != null;

        /// <summary>What is applied, for the console and the logs ("off" when nothing).</summary>
        public string Description { get; private set; }

        /// <summary>The seed the current setting's random draws come from (0 when off).</summary>
        public ulong Seed { get; private set; }

        /// <summary>
        /// Applies <paramref name="uplink"/> to what the headset sends and <paramref name="downlink"/>
        /// to what it receives; null or empty for none. Each direction gets its own emulator and
        /// substream (seed and seed + 1). What was in flight under the previous setting still drains.
        /// </summary>
        public void Set(INetworkImpairment[] uplink, INetworkImpairment[] downlink, ulong seed, string description)
        {
            Retire(ref _up, ref _upDraining);
            Retire(ref _down, ref _downDraining);
            _up = uplink != null && uplink.Length > 0 ? new Stage(uplink, seed, _inner.MaxPayloadBytes) : null;
            _down = downlink != null && downlink.Length > 0 ? new Stage(downlink, seed + 1, _inner.MaxPayloadBytes) : null;
            Seed = IsActive ? seed : 0;
            Description = IsActive ? description : "off";
        }

        /// <summary>Switches both directions off; what is in flight still drains.</summary>
        public void Clear() => Set(null, null, 0, "off");

        public bool Send(ReadOnlySpan<byte> payload, long nowTicks)
        {
            if (_up == null) return _inner.Send(payload, nowTicks);
            // Loss is decided here; delay as the datagram emerges in Pump.
            return _up.Emulator.Send(payload, nowTicks);
        }

        public bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks)
        {
            PumpUplink(nowTicks);

            // Frames already in a retired downlink emulator come out first, as they were sent first.
            if (_downDraining != null)
            {
                if (_downDraining.Emulator.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks)) return true;
                if (_downDraining.Emulator.InFlightCount == 0) _downDraining = null;
            }

            if (_down == null)
            {
                return _inner.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);
            }

            while (_inner.TryReceive(nowTicks, _buffer, out int length, out long realArrival))
            {
                _down.Emulator.Send(_buffer.AsSpan(0, length), realArrival);
            }

            return _down.Emulator.TryReceive(nowTicks, destination, out byteCount, out arrivalTicks);
        }

        public void Reset()
        {
            _up?.Emulator.Reset();
            _down?.Emulator.Reset();
            _upDraining = null;
            _downDraining = null;
            _inner.Reset();
        }

        /// <summary>Sends every uplink datagram whose emulated arrival has come.</summary>
        private void PumpUplink(long nowTicks)
        {
            if (_upDraining != null)
            {
                while (_upDraining.Emulator.TryReceive(nowTicks, _buffer, out int length, out _)) _inner.Send(_buffer.AsSpan(0, length), nowTicks);
                if (_upDraining.Emulator.InFlightCount == 0) _upDraining = null;
            }

            if (_up == null) return;
            while (_up.Emulator.TryReceive(nowTicks, _buffer, out int length, out _)) _inner.Send(_buffer.AsSpan(0, length), nowTicks);
        }

        /// <summary>The current stage starts draining; one already draining is dropped (a third change in quick succession).</summary>
        private static void Retire(ref Stage current, ref Stage draining)
        {
            if (current == null) return;
            draining = current;
            current = null;
        }

        private sealed class Stage
        {
            public readonly EmulatedTransport Emulator;

            public Stage(INetworkImpairment[] impairments, ulong seed, int maxPayloadBytes)
            {
                Emulator = new EmulatedTransport(new LoopbackTransport(maxPayloadBytes, Capacity), impairments, seed, maxInFlight: Capacity);
            }
        }
    }
}
