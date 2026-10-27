using System;
using System.Buffers.Binary;

namespace Teleop.RobotArm.Wire
{
    /// <summary>The course's task (docs/adr/0024 decision 2).</summary>
    public enum TrainingTask : byte
    {
        None = 0,

        /// <summary>Eight targets, one at a time: hold the wrist on each for a moment.</summary>
        Reach = 1,

        /// <summary>Follow a guide bead around a closed curve.</summary>
        Trace = 2,

        /// <summary>Touch two targets alternately, as fast as possible.</summary>
        Shuttle = 3,
    }

    /// <summary>Where the course is in a round.</summary>
    public enum TrainingPhase : byte
    {
        /// <summary>No round: the course is stopped, or between rounds.</summary>
        Idle = 0,

        /// <summary>A round is running: the robot is on and holding.</summary>
        Active = 1,

        /// <summary>A round is under way but its clock is stopped: the robot is paused, off, or waiting for torque.</summary>
        Suspended = 2,

        /// <summary>The round has ended; the score is final.</summary>
        Done = 3,
    }

    /// <summary>What the data recorded during a round is for (docs/adr/0024 decision 6).</summary>
    public enum TrainingPurpose : byte
    {
        Train = 0,

        /// <summary>Held out: recorded after a model is frozen, never trained on.</summary>
        Test = 1,
    }

    /// <summary>
    /// The course state, operator → robot on the joint port (docs/adr/0024 decision 7), little-endian,
    /// 34 bytes: <c>u8 0x83 | u8 version (1) | u8 task | u8 phase | u8 purpose | u8 flags |
    /// u32 courseId | u32 round | f32 speedLevel | f32 score | u32 sequence | i64 operatorSendTicks</c>.
    /// <c>flags</c> bit 0 is <see cref="DisturbedFlag"/>; <c>speedLevel</c> is rad/s, NaN for a varied
    /// round. It is state, not an event: the headset sends it once a second and at every change.
    /// Every decoder on the joint port checks its first byte exactly, so an older RobotHost ignores it.
    /// </summary>
    public static class TrainingTaskCodec
    {
        public const byte Tag = 0x83;
        public const byte Version = 1;
        public const int Size = 6 + 4 + 4 + 4 + 4 + 4 + 8;

        /// <summary>The headset's network disturbance or added delay was on (docs/adr/0024 decision 8).</summary>
        public const byte DisturbedFlag = 0x01;

        public static bool IsTask(ReadOnlySpan<byte> source) => source.Length >= 1 && source[0] == Tag;

        public static bool TryEncode(in TrainingState state, uint sequence, long operatorSendTicks, Span<byte> destination, out int bytesWritten)
        {
            bytesWritten = Size;
            if (destination.Length < Size)
            {
                return false;
            }

            destination[0] = Tag;
            destination[1] = Version;
            destination[2] = (byte)state.Task;
            destination[3] = (byte)state.Phase;
            destination[4] = (byte)state.Purpose;
            destination[5] = state.Disturbed ? DisturbedFlag : (byte)0;
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(6), state.CourseId);
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(10), state.Round);
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(14), BitConverter.SingleToInt32Bits(state.SpeedLevel));
            BinaryPrimitives.WriteInt32LittleEndian(destination.Slice(18), BitConverter.SingleToInt32Bits(state.Score));
            BinaryPrimitives.WriteUInt32LittleEndian(destination.Slice(22), sequence);
            BinaryPrimitives.WriteInt64LittleEndian(destination.Slice(26), operatorSendTicks);
            return true;
        }

        /// <summary>
        /// Reads a course state. False for a short record, another tag or version, or a task, phase or
        /// purpose byte this build does not define: a label it cannot name is not recorded.
        /// </summary>
        public static bool TryDecode(ReadOnlySpan<byte> source, out TrainingState state, out uint sequence, out long operatorSendTicks)
        {
            state = default;
            sequence = 0;
            operatorSendTicks = 0;
            if (source.Length < Size || source[0] != Tag || source[1] != Version ||
                source[2] > (byte)TrainingTask.Shuttle || source[3] > (byte)TrainingPhase.Done || source[4] > (byte)TrainingPurpose.Test)
            {
                return false;
            }

            state = new TrainingState(
                (TrainingTask)source[2], (TrainingPhase)source[3], (TrainingPurpose)source[4], (source[5] & DisturbedFlag) != 0,
                BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(6)),
                BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(10)),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(14))),
                BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(source.Slice(18))));
            sequence = BinaryPrimitives.ReadUInt32LittleEndian(source.Slice(22));
            operatorSendTicks = BinaryPrimitives.ReadInt64LittleEndian(source.Slice(26));
            return true;
        }

        public static string Name(TrainingTask task) => task switch
        {
            TrainingTask.Reach => "reach",
            TrainingTask.Trace => "trace",
            TrainingTask.Shuttle => "shuttle",
            _ => "none",
        };

        public static string Name(TrainingPhase phase) => phase switch
        {
            TrainingPhase.Active => "active",
            TrainingPhase.Suspended => "suspended",
            TrainingPhase.Done => "done",
            _ => "idle",
        };

        public static string Name(TrainingPurpose purpose) => purpose == TrainingPurpose.Test ? "test" : "train";
    }

    /// <summary>The course state the headset reports: what is being done, at what speed, for what.</summary>
    public readonly struct TrainingState : IEquatable<TrainingState>
    {
        public readonly TrainingTask Task;
        public readonly TrainingPhase Phase;
        public readonly TrainingPurpose Purpose;
        public readonly bool Disturbed;
        public readonly uint CourseId;
        public readonly uint Round;

        /// <summary>The round's speed level, rad/s; NaN for a varied round.</summary>
        public readonly float SpeedLevel;

        /// <summary>The round's score so far (final when <see cref="Phase"/> is <see cref="TrainingPhase.Done"/>).</summary>
        public readonly float Score;

        public TrainingState(TrainingTask task, TrainingPhase phase, TrainingPurpose purpose, bool disturbed, uint courseId, uint round,
            float speedLevel, float score)
        {
            Task = task;
            Phase = phase;
            Purpose = purpose;
            Disturbed = disturbed;
            CourseId = courseId;
            Round = round;
            SpeedLevel = speedLevel;
            Score = score;
        }

        /// <summary>
        /// Whether a recording should note the change: everything but the running score, which changes
        /// with every touch (the final score arrives with <see cref="TrainingPhase.Done"/>).
        /// </summary>
        public bool SameLabel(in TrainingState other) =>
            Task == other.Task && Phase == other.Phase && Purpose == other.Purpose && Disturbed == other.Disturbed &&
            CourseId == other.CourseId && Round == other.Round &&
            (SpeedLevel == other.SpeedLevel || (float.IsNaN(SpeedLevel) && float.IsNaN(other.SpeedLevel)));

        public bool Equals(TrainingState other) => SameLabel(other) && Score.Equals(other.Score);

        public override bool Equals(object? obj) => obj is TrainingState other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(Task, Phase, Purpose, CourseId, Round);
    }
}
