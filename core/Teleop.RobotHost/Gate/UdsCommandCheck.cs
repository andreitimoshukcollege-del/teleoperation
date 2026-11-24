using System;
using System.IO;
using System.Net.Sockets;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Gate
{
    /// <summary>
    /// A command check in another process on this machine (docs/adr/0038 N, <c>--command-gate socket:&lt;path&gt;</c>): each
    /// command goes to it as a <see cref="CommandGateWire"/> request over a Unix datagram socket (bound at <c>&lt;path&gt;.robothost</c>,
    /// as the relay client binds its own), and RobotHost waits for the reply with that seq for at most the deadline. Replies
    /// to earlier requests that came too late are read off and ignored. Nothing here retries: a miss is a miss.
    /// <para>The wait blocks RobotHost's loop, so a slow check delays every command after it by up to the deadline; a burst of
    /// N commands can wait N deadlines. Keep the deadline small (5 ms by default).</para>
    /// </summary>
    public sealed class UdsCommandCheck : ICommandCheck, IDisposable
    {
        private readonly Socket _socket;
        private readonly UnixDomainSocketEndPoint _peer;
        private readonly string _localPath;
        private readonly ITimeAuthority _clock;
        private readonly long _deadlineTicks;
        private readonly byte[] _request = new byte[CommandGateWire.RequestBytes(CommandGateWire.MaxTargets)];
        private readonly byte[] _reply = new byte[CommandGateWire.ReplyHeaderBytes + CommandGateWire.MaxReasonBytes + 16];

        public UdsCommandCheck(string localPath, string checkPath, double deadlineMs, ITimeAuthority clock)
        {
            if (!(deadlineMs > 0)) throw new ArgumentOutOfRangeException(nameof(deadlineMs));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            _deadlineTicks = Math.Max(1L, (long)(deadlineMs / 1000.0 * clock.TicksPerSecond));
            _localPath = localPath;
            _peer = new UnixDomainSocketEndPoint(checkPath);
            if (File.Exists(localPath)) File.Delete(localPath);
            _socket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
            _socket.Bind(new UnixDomainSocketEndPoint(localPath));
            _socket.Blocking = false;
        }

        public CommandVerdict Judge(uint sequence, long captureTicks, long nowTicks, ReadOnlySpan<JointTarget> targets, out string reason)
        {
            reason = "";
            long until = nowTicks + _deadlineTicks;
            DrainLate();
            int n = CommandGateWire.EncodeRequest(_request, sequence, captureTicks, nowTicks, _clock.TicksPerSecond, targets);
            try
            {
                _socket.SendTo(_request.AsSpan(0, n), SocketFlags.None, _peer);
            }
            catch (SocketException)
            {
                return CommandVerdict.NoAnswer; // no check listening
            }

            while (true)
            {
                long left = until - _clock.NowTicks;
                if (left <= 0) return CommandVerdict.NoAnswer;
                int micros = (int)Math.Max(1L, Math.Min(int.MaxValue, left * 1_000_000 / _clock.TicksPerSecond));
                if (!_socket.Poll(micros, SelectMode.SelectRead)) continue;
                int got;
                try
                {
                    got = _socket.Receive(_reply, SocketFlags.None);
                }
                catch (SocketException)
                {
                    continue;
                }

                if (CommandGateWire.TryDecodeReply(_reply.AsSpan(0, got), out uint seq, out bool hold, out string why) && seq == sequence)
                {
                    reason = why;
                    return hold ? CommandVerdict.Hold : CommandVerdict.Accept;
                }
            }
        }

        private void DrainLate()
        {
            try
            {
                while (_socket.Poll(0, SelectMode.SelectRead)) _socket.Receive(_reply, SocketFlags.None);
            }
            catch (SocketException)
            {
            }
        }

        public void Dispose()
        {
            _socket.Dispose();
            try
            {
                if (File.Exists(_localPath)) File.Delete(_localPath);
            }
            catch (IOException)
            {
            }
        }
    }

    /// <summary>
    /// Serves an <see cref="ICommandCheck"/> on a Unix datagram socket, answering each request to its sender (docs/adr/0038 N):
    /// how the example check runs as a process of its own, and what the tests drive <see cref="UdsCommandCheck"/> against. A
    /// real check (the trained one, later) implements the same datagrams in whatever language it is written in.
    /// </summary>
    public sealed class CommandCheckServer : IDisposable
    {
        private readonly Socket _socket;
        private readonly string _path;
        private readonly ICommandCheck _check;
        private readonly byte[] _buffer = new byte[CommandGateWire.RequestBytes(CommandGateWire.MaxTargets) + 16];
        private readonly byte[] _reply = new byte[CommandGateWire.ReplyHeaderBytes + CommandGateWire.MaxReasonBytes];
        private readonly JointTarget[] _targets = new JointTarget[CommandGateWire.MaxTargets];

        public CommandCheckServer(string path, ICommandCheck check)
        {
            _path = path;
            _check = check ?? throw new ArgumentNullException(nameof(check));
            if (File.Exists(path)) File.Delete(path);
            _socket = new Socket(AddressFamily.Unix, SocketType.Dgram, ProtocolType.Unspecified);
            _socket.Bind(new UnixDomainSocketEndPoint(path));
        }

        /// <summary>Answers requests until <paramref name="microseconds"/> pass with none; returns how many it answered.</summary>
        public int Serve(int microseconds)
        {
            int answered = 0;
            while (_socket.Poll(microseconds, SelectMode.SelectRead))
            {
                System.Net.EndPoint from = new UnixDomainSocketEndPoint(_path);
                int n = _socket.ReceiveFrom(_buffer, ref from);
                if (!CommandGateWire.TryDecodeRequest(_buffer.AsSpan(0, n), out uint seq, out long capture, out long robot, out _, _targets, out int count))
                {
                    continue;
                }

                CommandVerdict v = _check.Judge(seq, capture, robot, _targets.AsSpan(0, count), out string reason);
                int r = CommandGateWire.EncodeReply(_reply, seq, v == CommandVerdict.Hold, reason);
                _socket.SendTo(_reply.AsSpan(0, r), SocketFlags.None, from);
                answered++;
            }

            return answered;
        }

        public void Dispose()
        {
            _socket.Dispose();
            try
            {
                if (File.Exists(_path)) File.Delete(_path);
            }
            catch (IOException)
            {
            }
        }
    }
}
