using System;
using System.Buffers.Binary;
using System.Globalization;
using System.Text;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Gate
{
    /// <summary>What a command check said about one joint command (docs/adr/0038 N).</summary>
    public enum CommandVerdict
    {
        /// <summary>Let it through.</summary>
        Accept,

        /// <summary>Hold it: the arm keeps the last target it took.</summary>
        Hold,

        /// <summary>The check did not answer within the deadline (or could not be asked).</summary>
        NoAnswer,
    }

    /// <summary>
    /// Something that judges a joint command the robot is about to take (docs/adr/0038 N): in RobotHost's own process
    /// (<see cref="TableClearanceCheck"/>, the example) or another one over a socket (<see cref="UdsCommandCheck"/>).
    /// </summary>
    public interface ICommandCheck
    {
        /// <summary>
        /// Judges command <paramref name="sequence"/> (stamped <paramref name="captureTicks"/> on the sender's clock, received
        /// now, <paramref name="nowTicks"/> on RobotHost's). <paramref name="reason"/> is a short word for a hold, else empty.
        /// </summary>
        CommandVerdict Judge(uint sequence, long captureTicks, long nowTicks, ReadOnlySpan<JointTarget> targets, out string reason);
    }

    /// <summary>
    /// RobotHost's optional command gate (docs/adr/0038 N, request 6 of the world-models side): every joint command the arm
    /// would take is first put to an <see cref="ICommandCheck"/>; a hold drops it and the arm holds the last target it took.
    /// A check that does not answer in time lets the command through (fail-open, the default) or holds it (fail-closed).
    /// <para>Off unless <c>--command-gate</c> is given: then nothing asks it, and RobotHost adds no latency and writes no
    /// record. On, it writes <c>GC</c> at each session start, <c>GH</c> for every hold and <c>GT</c> for every miss
    /// (trainer/contract/SESSION_FORMAT.md). It never sees RobotHost's own park.</para>
    /// </summary>
    public sealed class CommandGate : IDisposable
    {
        private readonly ICommandCheck _check;
        private readonly ITimeAuthority _clock;
        private readonly Action<string>? _record;

        /// <param name="record">Takes a record line while a session is open (null: none is written).</param>
        public CommandGate(ICommandCheck check, string description, double deadlineMs, bool failClosed, ITimeAuthority clock, Action<string>? record)
        {
            _check = check ?? throw new ArgumentNullException(nameof(check));
            _clock = clock ?? throw new ArgumentNullException(nameof(clock));
            Description = description ?? throw new ArgumentNullException(nameof(description));
            DeadlineMs = deadlineMs;
            FailClosed = failClosed;
            _record = record;
        }

        public string Description { get; }
        public double DeadlineMs { get; }
        public bool FailClosed { get; }

        /// <summary>The sequence of the command about to be judged: the plant's hook carries no sequence, so Program sets it.</summary>
        public uint Sequence { get; set; }

        public int Holds { get; private set; }
        public int Misses { get; private set; }

        /// <summary>The plant's hook (<see cref="Plant.GenericArmPlant.CommandGate"/>): true holds the command.</summary>
        public bool ShouldHold(ReadOnlySpan<JointTarget> targets, long captureTicks)
        {
            long asked = _clock.NowTicks;
            CommandVerdict verdict = _check.Judge(Sequence, captureTicks, asked, targets, out string reason);
            long now = _clock.NowTicks;
            if (verdict == CommandVerdict.NoAnswer)
            {
                Misses++;
                _record?.Invoke(Recording.SessionLines.GateMiss(now, Sequence, (now - asked) * 1_000_000 / _clock.TicksPerSecond));
                if (!FailClosed) return false;
                reason = "no-answer";
            }
            else if (verdict == CommandVerdict.Accept)
            {
                return false;
            }

            Holds++;
            _record?.Invoke(Recording.SessionLines.GateHold(now, Sequence, reason.Length > 0 ? reason : "held"));
            return true;
        }

        public void Dispose() => (_check as IDisposable)?.Dispose();

        /// <summary>The <c>GC</c> record for a session's start.</summary>
        public string ConfigLine(long now) => Recording.SessionLines.GateConfig(now, Description, DeadlineMs, FailClosed);

        /// <summary>
        /// Builds the check <paramref name="spec"/> names: <c>socket:&lt;path&gt;</c> (another process, <see cref="UdsCommandCheck"/>)
        /// or <c>table:&lt;clearanceMm&gt;:&lt;tableTopZm&gt;</c> (the in-process example, <see cref="TableClearanceCheck"/>).
        /// </summary>
        public static bool TryCreateCheck(string spec, RobotArmProfile profile, double deadlineMs, ITimeAuthority clock,
                                          out ICommandCheck? check, out string? error)
        {
            check = null;
            if (!TryParseSpec(spec, out string kind, out string path, out double clearanceMm, out double tableTopZ, out error)) return false;
            try
            {
                check = kind == "socket"
                    ? new UdsCommandCheck(path + ".robothost", path, deadlineMs, clock)
                    : new TableClearanceCheck(profile, clearanceMm / 1000.0, tableTopZ);
                return true;
            }
            catch (Exception e) when (e is System.Net.Sockets.SocketException || e is PlatformNotSupportedException || e is ArgumentException)
            {
                error = $"--command-gate {spec}: {e.Message}";
                return false;
            }
        }

        /// <summary>Checks a <c>--command-gate</c> spec's form without building anything (RobotHostArgs).</summary>
        public static bool TryParseSpec(string spec, out string kind, out string path, out double clearanceMm, out double tableTopZ, out string? error)
        {
            kind = path = "";
            clearanceMm = tableTopZ = 0;
            error = null;
            if (spec.StartsWith("socket:", StringComparison.Ordinal) && spec.Length > "socket:".Length)
            {
                kind = "socket";
                path = spec.Substring("socket:".Length);
                return true;
            }

            string[] p = spec.Split(':');
            if (p.Length == 3 && p[0] == "table" &&
                double.TryParse(p[1], NumberStyles.Float, CultureInfo.InvariantCulture, out clearanceMm) && clearanceMm >= 0 && clearanceMm <= 500 &&
                double.TryParse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture, out tableTopZ) && Math.Abs(tableTopZ) <= 2)
            {
                kind = "table";
                return true;
            }

            error = $"--command-gate is socket:<path> or table:<clearanceMm 0-500>:<tableTopZ metres, base frame>, not '{spec}'";
            return false;
        }
    }

    /// <summary>
    /// The gate's datagrams to and from a check process (docs/adr/0038 N), little-endian:
    /// <list type="bullet">
    /// <item>request: <c>u8 'Q'</c>, <c>u8 version 1</c>, <c>u16 count</c>, <c>u32 seq</c>, <c>i64 captureTicks</c> (the sender's
    /// clock), <c>i64 robotTicks</c> (RobotHost's, when it was received), <c>i64 robotTicksPerSecond</c>, then per target
    /// <c>u8 motor</c>, <c>f32 angle</c> (radians; the gripper a 0-1 fraction), <c>f32 speed</c> (rad/s);</item>
    /// <item>reply: <c>u8 'V'</c>, <c>u8 version 1</c>, <c>u8 verdict</c> (0 accept, 1 hold), <c>u8 reasonLength</c>, <c>u32 seq</c>
    /// (the request's), then the reason, ASCII, at most <see cref="MaxReasonBytes"/>. A reply for another seq is ignored.</item>
    /// </list>
    /// </summary>
    public static class CommandGateWire
    {
        public const byte RequestTag = (byte)'Q', ReplyTag = (byte)'V', Version = 1;
        public const int RequestHeaderBytes = 32, TargetBytes = 9, ReplyHeaderBytes = 8, MaxReasonBytes = 48, MaxTargets = 32;

        public static int RequestBytes(int targets) => RequestHeaderBytes + targets * TargetBytes;

        public static int EncodeRequest(Span<byte> d, uint seq, long captureTicks, long robotTicks, long robotTps, ReadOnlySpan<JointTarget> targets)
        {
            int n = Math.Min(targets.Length, MaxTargets);
            d[0] = RequestTag;
            d[1] = Version;
            BinaryPrimitives.WriteUInt16LittleEndian(d.Slice(2), (ushort)n);
            BinaryPrimitives.WriteUInt32LittleEndian(d.Slice(4), seq);
            BinaryPrimitives.WriteInt64LittleEndian(d.Slice(8), captureTicks);
            BinaryPrimitives.WriteInt64LittleEndian(d.Slice(16), robotTicks);
            BinaryPrimitives.WriteInt64LittleEndian(d.Slice(24), robotTps);
            for (int i = 0; i < n; i++)
            {
                Span<byte> t = d.Slice(RequestHeaderBytes + i * TargetBytes);
                t[0] = targets[i].MotorId;
                BinaryPrimitives.WriteSingleLittleEndian(t.Slice(1), targets[i].Angle);
                BinaryPrimitives.WriteSingleLittleEndian(t.Slice(5), targets[i].Speed);
            }

            return RequestBytes(n);
        }

        public static bool TryDecodeRequest(ReadOnlySpan<byte> s, out uint seq, out long captureTicks, out long robotTicks, out long robotTps,
                                            Span<JointTarget> targets, out int count)
        {
            seq = 0;
            captureTicks = robotTicks = robotTps = 0;
            count = 0;
            if (s.Length < RequestHeaderBytes || s[0] != RequestTag || s[1] != Version) return false;
            int n = BinaryPrimitives.ReadUInt16LittleEndian(s.Slice(2));
            if (n > targets.Length || s.Length != RequestBytes(n)) return false;
            seq = BinaryPrimitives.ReadUInt32LittleEndian(s.Slice(4));
            captureTicks = BinaryPrimitives.ReadInt64LittleEndian(s.Slice(8));
            robotTicks = BinaryPrimitives.ReadInt64LittleEndian(s.Slice(16));
            robotTps = BinaryPrimitives.ReadInt64LittleEndian(s.Slice(24));
            for (int i = 0; i < n; i++)
            {
                ReadOnlySpan<byte> t = s.Slice(RequestHeaderBytes + i * TargetBytes);
                targets[i] = new JointTarget(t[0], BinaryPrimitives.ReadSingleLittleEndian(t.Slice(1)), BinaryPrimitives.ReadSingleLittleEndian(t.Slice(5)));
            }

            count = n;
            return true;
        }

        public static int EncodeReply(Span<byte> d, uint seq, bool hold, string reason)
        {
            string r = Clean(reason);
            d[0] = ReplyTag;
            d[1] = Version;
            d[2] = hold ? (byte)1 : (byte)0;
            d[3] = (byte)r.Length;
            BinaryPrimitives.WriteUInt32LittleEndian(d.Slice(4), seq);
            Encoding.ASCII.GetBytes(r, d.Slice(ReplyHeaderBytes));
            return ReplyHeaderBytes + r.Length;
        }

        public static bool TryDecodeReply(ReadOnlySpan<byte> s, out uint seq, out bool hold, out string reason)
        {
            seq = 0;
            hold = false;
            reason = "";
            if (s.Length < ReplyHeaderBytes || s[0] != ReplyTag || s[1] != Version || s[2] > 1) return false;
            int len = s[3];
            if (len > MaxReasonBytes || s.Length != ReplyHeaderBytes + len) return false;
            hold = s[2] == 1;
            seq = BinaryPrimitives.ReadUInt32LittleEndian(s.Slice(4));
            reason = Clean(Encoding.ASCII.GetString(s.Slice(ReplyHeaderBytes, len)));
            return true;
        }

        /// <summary>A reason as a record field: letters, digits and <c>. _ : -</c>, at most <see cref="MaxReasonBytes"/>.</summary>
        public static string Clean(string reason)
        {
            var b = new StringBuilder(Math.Min(reason.Length, MaxReasonBytes));
            foreach (char c in reason)
            {
                if (b.Length == MaxReasonBytes) break;
                b.Append(c < 128 && (char.IsLetterOrDigit(c) || c == '.' || c == '_' || c == ':' || c == '-') ? c : '_');
            }

            return b.ToString();
        }
    }
}
