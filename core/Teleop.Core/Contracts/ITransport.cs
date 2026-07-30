using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// A datagram channel, as Core sees it. Message-oriented and unreliable by design: what is
    /// sent may be delayed, dropped, duplicated, or reordered, and the mitigation of that is
    /// the research.
    ///
    /// Core contains no real I/O. The implementations in <c>Transport/</c> are in-process — a
    /// loopback, and an emulator that decorates any other <c>ITransport</c> with delay, jitter,
    /// loss and reordering from a seeded RNG. Sockets live in <c>Bridge/UdpTransport.cs</c>
    /// because I/O is a host concern; the same emulator decorates that one on a LAN and yields
    /// a reproducible impairment over a real socket.
    ///
    /// Receive is <b>poll-based, never callback- or thread-based</b>: Core has no threads, and
    /// a callback would deliver samples at a time the replay cannot reproduce. The host drains
    /// the channel once per step at a time it chooses, which is exactly what makes a replay
    /// bit-identical.
    ///
    /// Implementations are not thread-safe and are not required to be. Time is always a
    /// parameter, never a clock read.
    /// </summary>
    public interface ITransport
    {
        /// <summary>
        /// Largest datagram this transport carries, in bytes. Callers size their buffers from
        /// this rather than from a constant, and a codec must not produce more than this.
        /// </summary>
        int MaxPayloadBytes { get; }

        /// <summary>
        /// Hand a datagram to the channel at <paramref name="nowTicks"/>. The payload is copied
        /// before returning; the caller may reuse its buffer immediately.
        ///
        /// Returns false when the datagram will not be delivered — emulated loss, or a full
        /// send queue. False is an ordinary outcome on a lossy link, not an error, and callers
        /// must not retry on it or the loss model stops being the loss model.
        /// </summary>
        bool Send(ReadOnlySpan<byte> payload, long nowTicks);

        /// <summary>
        /// Take the next datagram that has arrived at or before <paramref name="nowTicks"/>,
        /// if any. Returns false when the channel is empty, which is the common case and not an
        /// error. Call in a loop until it returns false to drain a step; an implementation must
        /// never return the same datagram twice.
        ///
        /// Datagrams are returned in arrival order, which is not send order — reordering is
        /// part of what is being studied.
        ///
        /// <paramref name="destination"/> must be at least <see cref="MaxPayloadBytes"/> long.
        /// If it is too short for a pending datagram, this returns false, sets
        /// <paramref name="byteCount"/> to the length required, and leaves the datagram queued
        /// rather than throwing or allocating.
        /// </summary>
        /// <param name="nowTicks">Current time; nothing scheduled after it is visible yet.</param>
        /// <param name="destination">Caller-owned buffer the payload is copied into.</param>
        /// <param name="byteCount">
        /// Bytes written on success; on a too-short buffer, the bytes required; otherwise zero.
        /// </param>
        /// <param name="arrivalTicks">
        /// <c>t_recv</c> from docs/metrics.md: when the datagram actually arrived, which may be
        /// earlier than <paramref name="nowTicks"/> if the host polled late. Using the poll
        /// time instead folds frame time into measured one-way delay — a real source of wrong
        /// numbers in this project.
        /// </param>
        bool TryReceive(long nowTicks, Span<byte> destination, out int byteCount, out long arrivalTicks);

        /// <summary>
        /// Returns the transport to its as-constructed state: nothing in flight, nothing
        /// queued, impairment state and sequence counters cleared, and any injected RNG
        /// returned to its seed so the next trial reproduces the previous one. A decorator
        /// resets the transport it wraps. Sweeps reuse instances across trials.
        /// </summary>
        void Reset();
    }
}
