using System;
using System.Buffers.Binary;
using Teleop.RobotArm.World;

namespace Teleop.RobotArm.Wire
{
    /// <summary>What the headset asks of the world (docs/adr/0031 amendment 2).</summary>
    public enum WorldEditOp : byte
    {
        Move = 1,
        NewLayout = 2,
        ResetObject = 3,
        ResetLayout = 4,

        /// <summary>
        /// Resize one object (docs/adr/0034 amendment 6): the edit's x carries the new size factor on the scene file's size, one
        /// of <see cref="World.WorldScene.ScaleOfStep"/>'s; the rest of its pose is unused. The object is stood up where it is at
        /// the new size.
        /// </summary>
        Resize = 5,
    }

    /// <summary>The world's state as RobotHost reports it to the headset, once a second and on change.</summary>
    public readonly struct WorldStatus : IEquatable<WorldStatus>
    {
        public const byte FlagRunning = 1;
        public const byte FlagPhysics = 2;
        public const byte FlagEditsAllowed = 4;

        public readonly byte Flags;
        public readonly byte ObjectCount;
        public readonly uint LayoutNumber;
        public readonly ulong LayoutSeed;
        public readonly uint EditSequence;
        public readonly byte EditOutcome;
        public readonly byte EditObjectId;

        public WorldStatus(byte flags, byte objectCount, uint layoutNumber, ulong layoutSeed, uint editSequence, byte editOutcome, byte editObjectId)
        {
            Flags = flags;
            ObjectCount = objectCount;
            LayoutNumber = layoutNumber;
            LayoutSeed = layoutSeed;
            EditSequence = editSequence;
            EditOutcome = editOutcome;
            EditObjectId = editObjectId;
        }

        public bool Running => (Flags & FlagRunning) != 0;
        public bool Physics => (Flags & FlagPhysics) != 0;
        public bool EditsAllowed => (Flags & FlagEditsAllowed) != 0;

        public bool Equals(WorldStatus o) => Flags == o.Flags && ObjectCount == o.ObjectCount && LayoutNumber == o.LayoutNumber &&
            LayoutSeed == o.LayoutSeed && EditSequence == o.EditSequence && EditOutcome == o.EditOutcome && EditObjectId == o.EditObjectId;

        public override bool Equals(object? obj) => obj is WorldStatus o && Equals(o);

        public override int GetHashCode() => HashCode.Combine(Flags, ObjectCount, LayoutNumber, LayoutSeed, EditSequence, EditOutcome, EditObjectId);
    }

    /// <summary>
    /// The world on the joint port (docs/adr/0031 amendment 2), little-endian; quaternions w, x, y, z. These ride the joint
    /// states' transport, so the headset's disturbance, <c>ClockSync</c> and playout apply to them. Every decoder on the joint
    /// port checks its first byte exactly, so an older peer ignores them.
    /// <list type="table">
    /// <item><term>0x85 edit</term><description>headset → RobotHost, 44 bytes: tag, version, op (<see cref="WorldEditOp"/>), object
    /// id (0 for a layout op), uint32 sequence (not 0), int64 operator send ticks, float32 x y z, float32 qw qx qy qz.</description></item>
    /// <item><term>0x86 world frame</term><description>RobotHost → headset, at most 249 bytes: tag, version, uint32 snapshot, int64
    /// sensed ticks (RobotHost's clock), uint8 part, uint8 parts, uint8 count (≤ <see cref="MaxObjectsPerFrame"/>), then per object
    /// uint8 id, float32 x y z, float32 qw qx qy qz. Each part is applied as it arrives.</description></item>
    /// <item><term>0x8B world sizes</term><description>RobotHost → headset (docs/adr/0034 amendment 6), with the status: tag,
    /// version, uint32 revision (counts resizes), uint8 count, then per object uint8 id, float32 size factor. Every object,
    /// once a second and on change; applied at once, as the status is.</description></item>
    /// <item><term>0x87 world status</term><description>RobotHost → headset, 64 bytes: tag, version, flags, object count, 32 bytes of
    /// the scene file's sha256, uint32 layout number, uint64 layout seed, uint32 the last edit's sequence, uint8 its outcome (the
    /// arm-server wire's codes), uint8 its object, uint16 reserved, int64 robot send ticks.</description></item>
    /// </list>
    /// Allocation-free.
    /// </summary>
    public static class WorldCodec
    {
        public const byte EditTag = 0x85;
        public const byte FrameTag = 0x86;
        public const byte StatusTag = 0x87;
        public const byte SizesTag = 0x8B;
        private const int SizesHeaderBytes = 7;
        private const int SizesObjectBytes = 5;
        public const byte Version = 1;
        public const int EditBytes = 44;
        public const int StatusBytes = 64;
        public const int MaxObjectsPerFrame = 8;
        public const int Sha256Bytes = 32;
        private const int FrameObjectBytes = 29;
        private const int FrameHeaderBytes = 17;

        public static int FrameBytes(int objects) => FrameHeaderBytes + FrameObjectBytes * objects;

        public static bool TryEncodeEdit(WorldEditOp op, byte objectId, uint sequence, long operatorSendTicks, in ObjectPose pose, Span<byte> output, out int written)
        {
            written = 0;
            if (output.Length < EditBytes || sequence == 0) return false;
            output[0] = EditTag;
            output[1] = Version;
            output[2] = (byte)op;
            output[3] = objectId;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(4), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(8), operatorSendTicks);
            WritePose(output.Slice(16), pose);
            written = EditBytes;
            return true;
        }

        /// <summary>An edit; the pose's id is the edit's object id.</summary>
        public static bool TryDecodeEdit(ReadOnlySpan<byte> source, out WorldEditOp op, out uint sequence, out long operatorSendTicks, out ObjectPose pose)
        {
            op = 0;
            sequence = 0;
            operatorSendTicks = 0;
            pose = default;
            if (source.Length != EditBytes || source[0] != EditTag || source[1] != Version || source[2] < 1 || source[2] > 5) return false;
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(4));
            if (sequence == 0) return false;
            op = (WorldEditOp)source[2];
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(8));
            pose = ReadPose(source.Slice(16), source[3]);
            return true;
        }

        public static bool TryEncodeFrame(uint snapshot, long sensedTicks, byte part, byte parts, ReadOnlySpan<ObjectPose> objects, Span<byte> output, out int written)
        {
            written = 0;
            int n = objects.Length;
            if (n > MaxObjectsPerFrame || parts == 0 || part >= parts || output.Length < FrameBytes(n)) return false;
            output[0] = FrameTag;
            output[1] = Version;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(2), snapshot);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(6), sensedTicks);
            output[14] = part;
            output[15] = parts;
            output[16] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                int at = FrameHeaderBytes + FrameObjectBytes * i;
                output[at] = objects[i].Id;
                WritePose(output.Slice(at + 1), objects[i]);
            }

            written = FrameBytes(n);
            return true;
        }

        public static bool TryDecodeFrame(ReadOnlySpan<byte> source, out uint snapshot, out long sensedTicks, out byte part, out byte parts, Span<ObjectPose> objects, out int count)
        {
            snapshot = 0;
            sensedTicks = 0;
            part = parts = 0;
            count = 0;
            if (source.Length < FrameHeaderBytes || source[0] != FrameTag || source[1] != Version) return false;
            int n = source[16];
            if (n > MaxObjectsPerFrame || n > objects.Length || source.Length != FrameBytes(n)) return false;
            part = source[14];
            parts = source[15];
            if (parts == 0 || part >= parts) return false;
            snapshot = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(2));
            sensedTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(6));
            for (int i = 0; i < n; i++)
            {
                int at = FrameHeaderBytes + FrameObjectBytes * i;
                objects[i] = ReadPose(source.Slice(at + 1), source[at]);
            }

            count = n;
            return true;
        }

        public static bool TryEncodeStatus(in WorldStatus s, ReadOnlySpan<byte> sceneSha256, long robotSendTicks, Span<byte> output, out int written)
        {
            written = 0;
            if (output.Length < StatusBytes || sceneSha256.Length != Sha256Bytes) return false;
            output[0] = StatusTag;
            output[1] = Version;
            output[2] = s.Flags;
            output[3] = s.ObjectCount;
            sceneSha256.CopyTo(output.Slice(4));
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(36), s.LayoutNumber);
            BinaryPrimitives.WriteUInt64LittleEndian(output.Slice(40), s.LayoutSeed);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(48), s.EditSequence);
            output[52] = s.EditOutcome;
            output[53] = s.EditObjectId;
            output[54] = output[55] = 0;
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(56), robotSendTicks);
            written = StatusBytes;
            return true;
        }

        /// <summary>A status; <paramref name="sceneSha256"/> receives the scene file's hash.</summary>
        public static bool TryDecodeStatus(ReadOnlySpan<byte> source, out WorldStatus status, Span<byte> sceneSha256, out long robotSendTicks)
        {
            status = default;
            robotSendTicks = 0;
            if (source.Length != StatusBytes || source[0] != StatusTag || source[1] != Version || sceneSha256.Length < Sha256Bytes) return false;
            source.Slice(4, Sha256Bytes).CopyTo(sceneSha256);
            status = new WorldStatus(source[2], source[3], BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(36)),
                BinaryPrimitives.ReadUInt64LittleEndian(source.Slice(40)), BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(48)), source[52], source[53]);
            robotSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(56));
            return true;
        }

        public static int SizesBytes(int objects) => SizesHeaderBytes + SizesObjectBytes * objects;

        /// <summary>Every object's size factor, <paramref name="ids"/> and <paramref name="scales"/> parallel.</summary>
        public static bool TryEncodeSizes(uint revision, ReadOnlySpan<byte> ids, ReadOnlySpan<float> scales, Span<byte> output, out int written)
        {
            written = 0;
            int n = ids.Length;
            if (n != scales.Length || n > World.WorldScene.MaxObjects || output.Length < SizesBytes(n)) return false;
            output[0] = SizesTag;
            output[1] = Version;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(2), revision);
            output[6] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                int at = SizesHeaderBytes + SizesObjectBytes * i;
                output[at] = ids[i];
                BinaryPrimitives.WriteInt32LittleEndian(output.Slice(at + 1), BitConverter.SingleToInt32Bits(scales[i]));
            }

            written = SizesBytes(n);
            return true;
        }

        public static bool TryDecodeSizes(ReadOnlySpan<byte> source, out uint revision, Span<byte> ids, Span<float> scales, out int count)
        {
            revision = 0;
            count = 0;
            if (source.Length < SizesHeaderBytes || source[0] != SizesTag || source[1] != Version) return false;
            int n = source[6];
            if (n > ids.Length || n > scales.Length || source.Length != SizesBytes(n)) return false;
            revision = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(2));
            for (int i = 0; i < n; i++)
            {
                int at = SizesHeaderBytes + SizesObjectBytes * i;
                ids[i] = source[at];
                scales[i] = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(at + 1)));
            }

            count = n;
            return true;
        }

        private static void WritePose(Span<byte> o, in ObjectPose p)
        {
            BinaryPrimitives.WriteInt32LittleEndian(o, BitConverter.SingleToInt32Bits(p.X));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(4), BitConverter.SingleToInt32Bits(p.Y));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(8), BitConverter.SingleToInt32Bits(p.Z));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(12), BitConverter.SingleToInt32Bits(p.Qw));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(16), BitConverter.SingleToInt32Bits(p.Qx));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(20), BitConverter.SingleToInt32Bits(p.Qy));
            BinaryPrimitives.WriteInt32LittleEndian(o.Slice(24), BitConverter.SingleToInt32Bits(p.Qz));
        }

        private static ObjectPose ReadPose(ReadOnlySpan<byte> s, byte id) => new ObjectPose(id, 0,
            BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s)), BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(4))), BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(8))),
            BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(12))), BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(16))),
            BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(20))), BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s.Slice(24))));
    }
}
