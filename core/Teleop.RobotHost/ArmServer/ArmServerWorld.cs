using System;
using System.Buffers.Binary;
using Teleop.RobotArm.World;

namespace Teleop.RobotHost.ArmServer
{
    /// <summary>Why RobotHost asked for a placement.</summary>
    public enum PlaceCause : byte
    {
        Layout = 1,
        Headset = 2,
        Reset = 3,

        /// <summary>A resize from the headset (docs/adr/0034 amendment 6): sent as RESIZE, never as PLACE; for the records.</summary>
        Resize = 4,
    }

    /// <summary>What happened to a placement (docs/adr/0031 amendment 2); the joint port's world status echoes the same codes.</summary>
    public enum PlaceOutcome : byte
    {
        Applied = 0,
        OverlapsArm = 1,
        OverlapsObject = 2,
        OutsideRegion = 3,
        UnknownObject = 4,
        NotMovable = 5,
        ArmTouching = 6,
        Busy = 7,
        InvalidPose = 8,
        TimedOut = 9,
    }

    /// <summary>One contact between an arm and an object (0 is the table), with its normal force (N).</summary>
    public readonly struct ArmServerContact
    {
        public readonly byte Arm;
        public readonly byte Link;
        public readonly byte Object;
        public readonly float ForceNewtons;

        public ArmServerContact(byte arm, byte link, byte obj, float forceNewtons)
        {
            Arm = arm;
            Link = link;
            Object = obj;
            ForceNewtons = forceNewtons;
        }
    }

    /// <summary>
    /// The arm-server wire's world messages, kinds 7–11 of version 1 (docs/adr/0031 amendment 2), after
    /// <see cref="ArmServerWire"/>'s header. robot/isaac/teleop_isaac/wire.py must change with this. An older RobotHost counts
    /// these kinds as rejected, and an older bridge ignores them.
    /// <list type="table">
    /// <item><term>7 WorldInfo</term><description>server → RobotHost: 32 bytes of the scene file's sha256, uint8 count, each object's id.</description></item>
    /// <item><term>8 World</term><description>server → RobotHost: int64 sensed ticks, uint32 snapshot, uint32 revision, uint8 part,
    /// uint8 parts, uint8 count (≤ <see cref="MaxObjectsPerWorld"/>), then per object uint8 id, uint8 flags, float32 x y z, float32
    /// qw qx qy qz.</description></item>
    /// <item><term>9 Place</term><description>RobotHost → server: int64 host ticks, uint32 request, uint8 <see cref="PlaceCause"/>,
    /// uint8 id, float32 x y z, float32 qw qx qy qz.</description></item>
    /// <item><term>10 Placed</term><description>server → RobotHost: uint32 request, uint8 id, uint8 <see cref="PlaceOutcome"/>, uint32
    /// revision, then the pose the object has now.</description></item>
    /// <item><term>12 Resize</term><description>RobotHost → server (docs/adr/0034 amendment 6): int64 host ticks, uint32 request,
    /// uint8 id, float32 the new size factor on the scene file's size. Answered with a Placed of the same request: the object
    /// is stood up where it is at the new size, or refused by the placement rules at that size.</description></item>
    /// <item><term>13 Sizes</term><description>server → RobotHost: uint32 revision (counts resizes), uint8 count, then per
    /// object uint8 id, float32 size factor. Every object, at start, on change and once a second.</description></item>
    /// <item><term>11 Contacts</term><description>server → RobotHost: int64 sensed ticks, uint8 count, then per contact uint8 arm,
    /// uint8 link, uint8 object, float32 normal force (N).</description></item>
    /// </list>
    /// Allocation-free.
    /// </summary>
    public static class ArmServerWorld
    {
        public const byte KindWorldInfo = 7;
        public const byte KindWorld = 8;
        public const byte KindPlace = 9;
        public const byte KindPlaced = 10;
        public const byte KindContacts = 11;
        public const byte KindResize = 12;
        public const byte KindSizes = 13;
        private const int ResizeBytes = H + 17;

        public const int Sha256Bytes = 32;
        public const int MaxObjectsPerWorld = 16;
        public const int MaxContacts = 64;
        private const int PoseBytes = 28;
        private const int WorldObjectBytes = 2 + PoseBytes;
        private const int ContactBytes = 7;

        private const int H = ArmServerWire.HeaderBytes;

        public static bool TryEncodeWorldInfo(uint sequence, ReadOnlySpan<byte> sceneSha256, ReadOnlySpan<byte> ids, Span<byte> output, out int written)
        {
            written = 0;
            int size = H + Sha256Bytes + 1 + ids.Length;
            if (sceneSha256.Length != Sha256Bytes || ids.Length > 255 || output.Length < size) return false;
            ArmServerWire.Header(output, KindWorldInfo, sequence);
            sceneSha256.CopyTo(output.Slice(H));
            output[H + Sha256Bytes] = (byte)ids.Length;
            ids.CopyTo(output.Slice(H + Sha256Bytes + 1));
            written = size;
            return true;
        }

        /// <summary>A WorldInfo's fields; both spans are slices of <paramref name="datagram"/>.</summary>
        public static bool TryDecodeWorldInfo(ReadOnlySpan<byte> datagram, out ReadOnlySpan<byte> sceneSha256, out ReadOnlySpan<byte> ids)
        {
            sceneSha256 = ids = default;
            if (ArmServerWire.KindOf(datagram) != KindWorldInfo || datagram.Length < H + Sha256Bytes + 1) return false;
            int n = datagram[H + Sha256Bytes];
            if (datagram.Length != H + Sha256Bytes + 1 + n) return false;
            sceneSha256 = datagram.Slice(H, Sha256Bytes);
            ids = datagram.Slice(H + Sha256Bytes + 1, n);
            return true;
        }

        public static bool TryEncodeWorld(uint sequence, long sensedTicks, uint snapshot, uint revision, byte part, byte parts,
            ReadOnlySpan<ObjectPose> objects, Span<byte> output, out int written)
        {
            written = 0;
            int n = objects.Length;
            int size = H + 19 + WorldObjectBytes * n;
            if (n > MaxObjectsPerWorld || parts == 0 || part >= parts || output.Length < size) return false;
            ArmServerWire.Header(output, KindWorld, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(H), sensedTicks);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H + 8), snapshot);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H + 12), revision);
            output[H + 16] = part;
            output[H + 17] = parts;
            output[H + 18] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                int at = H + 19 + WorldObjectBytes * i;
                output[at] = objects[i].Id;
                output[at + 1] = objects[i].Flags;
                WritePose(output.Slice(at + 2), objects[i]);
            }

            written = size;
            return true;
        }

        public static bool TryDecodeWorld(ReadOnlySpan<byte> datagram, out long sensedTicks, out uint snapshot, out uint revision,
            out byte part, out byte parts, Span<ObjectPose> objects, out int count)
        {
            sensedTicks = 0;
            snapshot = revision = 0;
            part = parts = 0;
            count = 0;
            if (ArmServerWire.KindOf(datagram) != KindWorld || datagram.Length < H + 19) return false;
            int n = datagram[H + 18];
            if (n > MaxObjectsPerWorld || n > objects.Length || datagram.Length != H + 19 + WorldObjectBytes * n) return false;
            part = datagram[H + 16];
            parts = datagram[H + 17];
            if (parts == 0 || part >= parts) return false;
            sensedTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(H));
            snapshot = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H + 8));
            revision = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H + 12));
            for (int i = 0; i < n; i++)
            {
                int at = H + 19 + WorldObjectBytes * i;
                objects[i] = ReadPose(datagram.Slice(at + 2), datagram[at], datagram[at + 1]);
            }

            count = n;
            return true;
        }

        public static bool TryEncodePlace(uint sequence, long hostTicks, uint request, PlaceCause cause, in ObjectPose pose, Span<byte> output, out int written)
        {
            written = 0;
            const int size = H + 14 + PoseBytes;
            if (output.Length < size) return false;
            ArmServerWire.Header(output, KindPlace, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(H), hostTicks);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H + 8), request);
            output[H + 12] = (byte)cause;
            output[H + 13] = pose.Id;
            WritePose(output.Slice(H + 14), pose);
            written = size;
            return true;
        }

        public static bool TryDecodePlace(ReadOnlySpan<byte> datagram, out long hostTicks, out uint request, out PlaceCause cause, out ObjectPose pose)
        {
            hostTicks = 0;
            request = 0;
            cause = 0;
            pose = default;
            if (ArmServerWire.KindOf(datagram) != KindPlace || datagram.Length != H + 14 + PoseBytes) return false;
            byte c = datagram[H + 12];
            if (c < 1 || c > 3) return false;
            hostTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(H));
            request = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H + 8));
            cause = (PlaceCause)c;
            pose = ReadPose(datagram.Slice(H + 14), datagram[H + 13], 0);
            return true;
        }

        public static bool TryEncodePlaced(uint sequence, uint request, PlaceOutcome outcome, uint revision, in ObjectPose pose, Span<byte> output, out int written)
        {
            written = 0;
            const int size = H + 10 + PoseBytes;
            if (output.Length < size) return false;
            ArmServerWire.Header(output, KindPlaced, sequence);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H), request);
            output[H + 4] = pose.Id;
            output[H + 5] = (byte)outcome;
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H + 6), revision);
            WritePose(output.Slice(H + 10), pose);
            written = size;
            return true;
        }

        public static bool TryDecodePlaced(ReadOnlySpan<byte> datagram, out uint request, out PlaceOutcome outcome, out uint revision, out ObjectPose pose)
        {
            request = revision = 0;
            outcome = 0;
            pose = default;
            if (ArmServerWire.KindOf(datagram) != KindPlaced || datagram.Length != H + 10 + PoseBytes || datagram[H + 5] > (byte)PlaceOutcome.TimedOut) return false;
            request = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H));
            outcome = (PlaceOutcome)datagram[H + 5];
            revision = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H + 6));
            pose = ReadPose(datagram.Slice(H + 10), datagram[H + 4], 0);
            return true;
        }

        public static bool TryEncodeResize(uint sequence, long hostTicks, uint request, byte id, float scale, Span<byte> output, out int written)
        {
            written = 0;
            if (output.Length < ResizeBytes) return false;
            ArmServerWire.Header(output, KindResize, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(H), hostTicks);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H + 8), request);
            output[H + 12] = id;
            BinaryPrimitives.WriteSingleLittleEndian(output.Slice(H + 13), scale);
            written = ResizeBytes;
            return true;
        }

        public static bool TryDecodeResize(ReadOnlySpan<byte> datagram, out long hostTicks, out uint request, out byte id, out float scale)
        {
            hostTicks = 0;
            request = 0;
            id = 0;
            scale = 0;
            if (ArmServerWire.KindOf(datagram) != KindResize || datagram.Length != ResizeBytes) return false;
            hostTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(H));
            request = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H + 8));
            id = datagram[H + 12];
            scale = BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(H + 13));
            return true;
        }

        public static bool TryEncodeSizes(uint sequence, uint revision, ReadOnlySpan<byte> ids, ReadOnlySpan<float> scales, Span<byte> output, out int written)
        {
            written = 0;
            int n = ids.Length;
            int size = H + 5 + 5 * n;
            if (n != scales.Length || n > 255 || output.Length < size) return false;
            ArmServerWire.Header(output, KindSizes, sequence);
            BinaryPrimitives.WriteUInt32LittleEndian(output.Slice(H), revision);
            output[H + 4] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                output[H + 5 + 5 * i] = ids[i];
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(H + 6 + 5 * i), scales[i]);
            }

            written = size;
            return true;
        }

        public static bool TryDecodeSizes(ReadOnlySpan<byte> datagram, out uint revision, Span<byte> ids, Span<float> scales, out int count)
        {
            revision = 0;
            count = 0;
            if (ArmServerWire.KindOf(datagram) != KindSizes || datagram.Length < H + 5) return false;
            int n = datagram[H + 4];
            if (n > ids.Length || n > scales.Length || datagram.Length != H + 5 + 5 * n) return false;
            revision = BinaryPrimitives.ReadUInt32LittleEndian(datagram.Slice(H));
            for (int i = 0; i < n; i++)
            {
                ids[i] = datagram[H + 5 + 5 * i];
                scales[i] = BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(H + 6 + 5 * i));
            }

            count = n;
            return true;
        }

        public static bool TryEncodeContacts(uint sequence, long sensedTicks, ReadOnlySpan<ArmServerContact> contacts, Span<byte> output, out int written)
        {
            written = 0;
            int n = contacts.Length;
            int size = H + 9 + ContactBytes * n;
            if (n > MaxContacts || output.Length < size) return false;
            ArmServerWire.Header(output, KindContacts, sequence);
            BinaryPrimitives.WriteInt64LittleEndian(output.Slice(H), sensedTicks);
            output[H + 8] = (byte)n;
            for (int i = 0; i < n; i++)
            {
                int at = H + 9 + ContactBytes * i;
                output[at] = contacts[i].Arm;
                output[at + 1] = contacts[i].Link;
                output[at + 2] = contacts[i].Object;
                BinaryPrimitives.WriteSingleLittleEndian(output.Slice(at + 3), contacts[i].ForceNewtons);
            }

            written = size;
            return true;
        }

        public static bool TryDecodeContacts(ReadOnlySpan<byte> datagram, out long sensedTicks, Span<ArmServerContact> contacts, out int count)
        {
            sensedTicks = 0;
            count = 0;
            if (ArmServerWire.KindOf(datagram) != KindContacts || datagram.Length < H + 9) return false;
            int n = datagram[H + 8];
            if (n > MaxContacts || n > contacts.Length || datagram.Length != H + 9 + ContactBytes * n) return false;
            sensedTicks = BinaryPrimitives.ReadInt64LittleEndian(datagram.Slice(H));
            for (int i = 0; i < n; i++)
            {
                int at = H + 9 + ContactBytes * i;
                contacts[i] = new ArmServerContact(datagram[at], datagram[at + 1], datagram[at + 2], BinaryPrimitives.ReadSingleLittleEndian(datagram.Slice(at + 3)));
            }

            count = n;
            return true;
        }

        private static void WritePose(Span<byte> o, in ObjectPose p)
        {
            BinaryPrimitives.WriteSingleLittleEndian(o, p.X);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(4), p.Y);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(8), p.Z);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(12), p.Qw);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(16), p.Qx);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(20), p.Qy);
            BinaryPrimitives.WriteSingleLittleEndian(o.Slice(24), p.Qz);
        }

        private static ObjectPose ReadPose(ReadOnlySpan<byte> s, byte id, byte flags) => new ObjectPose(id, flags,
            BinaryPrimitives.ReadSingleLittleEndian(s), BinaryPrimitives.ReadSingleLittleEndian(s.Slice(4)), BinaryPrimitives.ReadSingleLittleEndian(s.Slice(8)),
            BinaryPrimitives.ReadSingleLittleEndian(s.Slice(12)), BinaryPrimitives.ReadSingleLittleEndian(s.Slice(16)),
            BinaryPrimitives.ReadSingleLittleEndian(s.Slice(20)), BinaryPrimitives.ReadSingleLittleEndian(s.Slice(24)));
    }
}
