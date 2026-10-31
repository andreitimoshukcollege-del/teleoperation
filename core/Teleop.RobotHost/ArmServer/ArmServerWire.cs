using System;
using System.Buffers.Binary;
using System.Text;
using Teleop.Core.Types;

namespace Teleop.RobotHost.ArmServer
{
    /// <summary>What owns the arm on the far side of the socket (docs/adr/0030 decision 5, 0031 decision 1).</summary>
    public enum ArmServerKind : byte
    {
        Unknown = 0,

        /// <summary>The FR3 Duo in Isaac Sim (robot/isaac/): physics, never the robot.</summary>
        Isaac = 1,

        /// <summary>The real FR3's arm server over libfranka (docs/adr/0030).</summary>
        Fci = 2,

        /// <summary>The arm server's own simulate mode, no robot and no physics.</summary>
        Simulate = 3,
    }

    /// <summary>One state report: when it was measured, the server's own time, and its flags.</summary>
    public readonly struct ArmServerStateHeader
    {
        public readonly long SensedTicks;
        public readonly long ServerTicks;
        public readonly byte Flags;

        public ArmServerStateHeader(long sensedTicks, long serverTicks, byte flags)
        {
            SensedTicks = sensedTicks;
            ServerTicks = serverTicks;
            Flags = flags;
        }

        public bool Enabled => (Flags & ArmServerWire.FlagEnabled) != 0;
        public bool Behind => (Flags & ArmServerWire.FlagBehind) != 0;
        public bool Error => (Flags & ArmServerWire.FlagError) != 0;
    }

    /// <summary>How the server keeps up with the wall clock, about once a second.</summary>
    public readonly struct ArmServerHealth
    {
        public readonly float RealTimeFactor;
        public readonly float LagMilliseconds;
        public readonly uint LateSteps;
        public readonly uint Steps;

        public ArmServerHealth(float realTimeFactor, float lagMilliseconds, uint lateSteps, uint steps)
        {
            RealTimeFactor = realTimeFactor;
            LagMilliseconds = lagMilliseconds;
            LateSteps = lateSteps;
            Steps = steps;
        }
    }

    /// <summary>
    /// The arm-server wire, version 1 (docs/adr/0030 decision 5): RobotHost and the process that owns the arm, over a Unix
    /// datagram socket on one machine. That process is the real FR3's arm server, its simulate mode, or the Isaac bridge
    /// (robot/isaac/wire.py, which must change with this). Little-endian. Every datagram starts with the same 8 bytes:
    /// <c>'A' 'S'</c>, <see cref="Version"/>, the kind, and a uint32 sequence.
    /// <list type="table">
    /// <item><term>1 Hello</term><description>RobotHost → server: int64 host ticks, int64 ticks per second, uint8 length and
    /// the profile's name (UTF-8). Sent until a Welcome comes back.</description></item>
    /// <item><term>2 Welcome</term><description>server → RobotHost: uint8 <see cref="ArmServerKind"/>, int64 the server's
    /// clock now (it must be RobotHost's clock), uint8 motor count and each motor id in its order, uint16 length and a
    /// description (UTF-8).</description></item>
    /// <item><term>3 Targets</term><description>RobotHost → server: int64 host ticks, uint8 count, then per motor uint8 id
    /// and float32 value (radians, or a hand's 0..1 opening).</description></item>
    /// <item><term>4 Enable</term><description>RobotHost → server: int64 host ticks, uint8 1 or 0.</description></item>
    /// <item><term>5 State</term><description>server → RobotHost: int64 sensed ticks (RobotHost's clock), int64 the server's
    /// own time (ns), uint8 flags (<see cref="FlagEnabled"/>, <see cref="FlagBehind"/>, <see cref="FlagError"/>), uint8
    /// count, then per motor uint8 id and float32 position, velocity and commanded position.</description></item>
    /// <item><term>6 Health</term><description>server → RobotHost: float32 real-time factor, float32 lag (ms), uint32 late
    /// steps, uint32 steps.</description></item>
    /// </list>
    /// Allocation-free: everything encodes into and decodes from spans the caller owns.
    /// </summary>
    public static class ArmServerWire
    {
        public const byte Version = 1;
        public const int HeaderBytes = 8;

        /// <summary>The largest datagram either side sends: room for 36 motors' states.</summary>
        public const int MaxBytes = 512;

        public const int MaxMotors = 36;
        public const int MaxNameBytes = 64;
        public const int MaxDescriptionBytes = 256;

        public const byte KindHello = 1;
        public const byte KindWelcome = 2;
        public const byte KindTargets = 3;
        public const byte KindEnable = 4;
        public const byte KindState = 5;
        public const byte KindHealth = 6;

        public const byte FlagEnabled = 1;
        public const byte FlagBehind = 2;
        public const byte FlagError = 4;

        private const int StateMotorBytes = 13;

        /// <summary>The datagram's kind, or 0 when it is not an arm-server datagram of this version.</summary>
        public static byte KindOf(ReadOnlySpan<byte> datagram) =>
            datagram.Length >= HeaderBytes && datagram[0] == (byte)'A' && datagram[1] == (byte)'S' && datagram[2] == Version ? datagram[3] : (byte)0;

        public static uint SequenceOf(ReadOnlySpan<byte> datagram) =>
            datagram.Length >= HeaderBytes ? BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(4)) : 0u;

        public static bool TryEncodeHello(uint sequence, long hostTicks, long ticksPerSecond, string profileName, Span<byte> output, out int written)
        {
            written = 0;
            int nameBytes = Encoding.UTF8.GetByteCount(profileName ?? "");
            int size = HeaderBytes + 8 + 8 + 1 + nameBytes;
            if (nameBytes > MaxNameBytes || output.Length < size) return false;
            Header(output, KindHello, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), hostTicks);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(16), ticksPerSecond);
            output[24] = (byte)nameBytes;
            Encoding.UTF8.GetBytes(profileName ?? "", output.Slice(25, nameBytes));
            written = size;
            return true;
        }

        /// <summary>A Hello's fields; the name's bytes are a slice of <paramref name="datagram"/>.</summary>
        public static bool TryDecodeHello(ReadOnlySpan<byte> datagram, out long hostTicks, out long ticksPerSecond, out ReadOnlySpan<byte> profileName)
        {
            hostTicks = ticksPerSecond = 0;
            profileName = default;
            if (KindOf(datagram) != KindHello || datagram.Length < HeaderBytes + 17) return false;
            int n = datagram[24];
            if (n > MaxNameBytes || datagram.Length != 25 + n) return false;
            hostTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(8));
            ticksPerSecond = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(16));
            profileName = datagram.Slice(25, n);
            return ticksPerSecond > 0;
        }

        public static bool TryEncodeWelcome(uint sequence, ArmServerKind kind, long serverNowTicks, ReadOnlySpan<byte> motorIds, string description,
            Span<byte> output, out int written)
        {
            written = 0;
            int descriptionBytes = Encoding.UTF8.GetByteCount(description ?? "");
            int size = HeaderBytes + 1 + 8 + 1 + motorIds.Length + 2 + descriptionBytes;
            if (motorIds.Length > MaxMotors || descriptionBytes > MaxDescriptionBytes || output.Length < size) return false;
            Header(output, KindWelcome, sequence);
            output[8] = (byte)kind;
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(9), serverNowTicks);
            output[17] = (byte)motorIds.Length;
            motorIds.CopyTo(output.Slice(18));
            int at = 18 + motorIds.Length;
            BinaryPrimitives.WriteUInt16LittleEndian(output.Slice(at), (ushort)descriptionBytes);
            Encoding.UTF8.GetBytes(description ?? "", output.Slice(at + 2, descriptionBytes));
            written = size;
            return true;
        }

        /// <summary>A Welcome's fields; the motor ids and description are slices of <paramref name="datagram"/>.</summary>
        public static bool TryDecodeWelcome(ReadOnlySpan<byte> datagram, out ArmServerKind kind, out long serverNowTicks,
            out ReadOnlySpan<byte> motorIds, out ReadOnlySpan<byte> description)
        {
            kind = ArmServerKind.Unknown;
            serverNowTicks = 0;
            motorIds = description = default;
            if (KindOf(datagram) != KindWelcome || datagram.Length < HeaderBytes + 12) return false;
            int motors = datagram[17];
            int at = 18 + motors;
            if (motors > MaxMotors || datagram.Length < at + 2) return false;
            int n = BinaryPrimitives.ReadUInt16LittleEndian(datagram.Slice(at));
            if (n > MaxDescriptionBytes || datagram.Length != at + 2 + n) return false;
            kind = (ArmServerKind)datagram[8];
            serverNowTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(9));
            motorIds = datagram.Slice(18, motors);
            description = datagram.Slice(at + 2, n);
            return true;
        }

        public static bool TryEncodeTargets(uint sequence, long hostTicks, ReadOnlySpan<JointSample> targets, Span<byte> output, out int written)
        {
            written = 0;
            int size = HeaderBytes + 8 + 1 + 5 * targets.Length;
            if (targets.Length > MaxMotors || output.Length < size) return false;
            Header(output, KindTargets, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), hostTicks);
            output[16] = (byte)targets.Length;
            for (int i = 0; i < targets.Length; i++)
            {
                int at = 17 + 5 * i;
                output[at] = targets[i].MotorId;
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(at + 1), targets[i].Value);
            }

            written = size;
            return true;
        }

        public static bool TryDecodeTargets(ReadOnlySpan<byte> datagram, out long hostTicks, Span<JointSample> targets, out int count)
        {
            hostTicks = 0;
            count = 0;
            if (KindOf(datagram) != KindTargets || datagram.Length < HeaderBytes + 9) return false;
            int n = datagram[16];
            if (n > MaxMotors || n > targets.Length || datagram.Length != 17 + 5 * n) return false;
            hostTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(8));
            for (int i = 0; i < n; i++)
            {
                int at = 17 + 5 * i;
                targets[i] = new JointSample(datagram[at], BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(at + 1)), false);
            }

            count = n;
            return true;
        }

        public static bool TryEncodeEnable(uint sequence, long hostTicks, bool enabled, Span<byte> output, out int written)
        {
            written = 0;
            const int size = HeaderBytes + 9;
            if (output.Length < size) return false;
            Header(output, KindEnable, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), hostTicks);
            output[16] = enabled ? (byte)1 : (byte)0;
            written = size;
            return true;
        }

        public static bool TryDecodeEnable(ReadOnlySpan<byte> datagram, out long hostTicks, out bool enabled)
        {
            hostTicks = 0;
            enabled = false;
            if (KindOf(datagram) != KindEnable || datagram.Length != HeaderBytes + 9 || datagram[16] > 1) return false;
            hostTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(8));
            enabled = datagram[16] == 1;
            return true;
        }

        /// <summary>A State of <paramref name="motorIds"/>.Length motors; the three value spans are parallel to it.</summary>
        public static bool TryEncodeState(uint sequence, in ArmServerStateHeader header, ReadOnlySpan<byte> motorIds,
            ReadOnlySpan<float> positions, ReadOnlySpan<float> velocities, ReadOnlySpan<float> commanded, Span<byte> output, out int written)
        {
            written = 0;
            int n = motorIds.Length;
            int size = HeaderBytes + 18 + StateMotorBytes * n;
            if (n > MaxMotors || positions.Length < n || velocities.Length < n || commanded.Length < n || output.Length < size) return false;
            Header(output, KindState, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), header.SensedTicks);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(16), header.ServerTicks);
            output[24] = header.Flags;
            output[25] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                int at = 26 + StateMotorBytes * i;
                output[at] = motorIds[i];
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(at + 1), positions[i]);
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(at + 5), velocities[i]);
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(at + 9), commanded[i]);
            }

            written = size;
            return true;
        }

        public static bool TryDecodeState(ReadOnlySpan<byte> datagram, out ArmServerStateHeader header, Span<byte> motorIds,
            Span<float> positions, Span<float> velocities, Span<float> commanded, out int count)
        {
            header = default;
            count = 0;
            if (KindOf(datagram) != KindState || datagram.Length < HeaderBytes + 18) return false;
            int n = datagram[25];
            if (n > MaxMotors || n > motorIds.Length || n > positions.Length || n > velocities.Length || n > commanded.Length) return false;
            if (datagram.Length != 26 + StateMotorBytes * n) return false;
            header = new ArmServerStateHeader(BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(8)),
                BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(16)), datagram[24]);
            for (int i = 0; i < n; i++)
            {
                int at = 26 + StateMotorBytes * i;
                motorIds[i] = datagram[at];
                positions[i] = BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(at + 1));
                velocities[i] = BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(at + 5));
                commanded[i] = BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(at + 9));
            }

            count = n;
            return true;
        }

        public static bool TryEncodeHealth(uint sequence, in ArmServerHealth health, Span<byte> output, out int written)
        {
            written = 0;
            const int size = HeaderBytes + 16;
            if (output.Length < size) return false;
            Header(output, KindHealth, sequence);
            BinaryPrimitives.WriteSingleLittleEndian(output.Slice(8), health.RealTimeFactor);
            BinaryPrimitives.WriteSingleLittleEndian(output.Slice(12), health.LagMilliseconds);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(16), health.LateSteps);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(20), health.Steps);
            written = size;
            return true;
        }

        public static bool TryDecodeHealth(ReadOnlySpan<byte> datagram, out ArmServerHealth health)
        {
            health = default;
            if (KindOf(datagram) != KindHealth || datagram.Length != HeaderBytes + 16) return false;
            health = new ArmServerHealth(BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(8)), BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(12)),
                BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(16)), BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(20)));
            return true;
        }

        private static void Header(Span<byte> output, byte kind, uint sequence)
        {
            output[0] = (byte)'A';
            output[1] = (byte)'S';
            output[2] = Version;
            output[3] = kind;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(4), sequence);
        }
    }
}
