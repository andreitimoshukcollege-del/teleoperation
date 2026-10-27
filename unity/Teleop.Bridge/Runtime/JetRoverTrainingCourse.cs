using System;
using Teleop.Core.Contracts;
using Teleop.RobotArm.Training;
using Teleop.RobotArm.Types;
using Teleop.RobotArm.Wire;
using UnityEngine;
using CoreVec = System.Numerics.Vector3;

namespace Teleop.Bridge
{
    /// <summary>The course's task, for code that sees Bridge only.</summary>
    public enum JetRoverCourseTask { None, Reach, Trace, Shuttle }

    /// <summary>Where the course is in a round: announcing the next one, running, stopped for the robot, or showing the result.</summary>
    public enum JetRoverCoursePhase { Stopped, Announcing, Active, Suspended, Done }

    /// <summary>What a reach target asks of the gripper.</summary>
    public enum JetRoverGripperGoal { Any, Open, Closed }

    /// <summary>
    /// The headset's training course (docs/adr/0024), for the bench's controls. An adapter only: every
    /// decision (where targets go, what counts, which speed) is <c>Teleop.RobotArm</c>'s
    /// <see cref="TrainingCourse"/>; this converts its arm-frame positions into the scene through the
    /// arm's base anchor and <see cref="CoordConversion"/>, and gives plain types to code that sees
    /// Bridge only. <see cref="JetRoverOperatorBridge"/> drives it every frame with the commanded wrist
    /// and sends its state to the robot's recording.
    /// </summary>
    public sealed class JetRoverTrainingCourse
    {
        private readonly TrainingCourse _course;
        private readonly Transform _armBase;
        private readonly ITimeAuthority _clock;

        internal JetRoverTrainingCourse(RobotArmProfile profile, float minHeightMeters, Transform armBase, ITimeAuthority clock)
        {
            _course = new TrainingCourse(profile, minHeightMeters);
            _armBase = armBase;
            _clock = clock;
        }

        public bool Running => _course.Running;

        /// <summary>Whether the next round's data is held out for testing (docs/adr/0024 decision 6).</summary>
        public bool Test
        {
            get => _course.Purpose == TrainingPurpose.Test;
            set => _course.Purpose = value ? TrainingPurpose.Test : TrainingPurpose.Train;
        }

        /// <summary>Whether the running round is labelled test (it keeps the purpose it started with).</summary>
        public bool RoundIsTest => _course.State.Purpose == TrainingPurpose.Test;

        public uint CourseId => _course.CourseId;
        public uint Round => _course.Round;

        public JetRoverCourseTask Task => (JetRoverCourseTask)(int)_course.Task;

        public JetRoverCoursePhase Phase => !_course.Running ? JetRoverCoursePhase.Stopped : _course.Phase switch
        {
            TrainingPhase.Active => JetRoverCoursePhase.Active,
            TrainingPhase.Suspended => JetRoverCoursePhase.Suspended,
            TrainingPhase.Done => JetRoverCoursePhase.Done,
            _ => JetRoverCoursePhase.Announcing,
        };

        /// <summary>The speed the course sends now, rad/s; NaN when stopped.</summary>
        public float SpeedRadPerSec => _course.SpeedRadPerSec;

        /// <summary>Whether the running round changes its speed as it goes.</summary>
        public bool Varied => _course.Level == TrainingCourse.VariedLevel;

        public float Score => _course.Score;
        public double SecondsLeft => _course.SecondsLeft;
        public bool Disturbed => _course.State.Disturbed;

        public int TargetCount => _course.TargetCount;
        public int ActiveTarget => _course.ActiveTarget;
        public float HoldProgress => _course.HoldProgress;
        public Vector3 TargetWorld(int i) => ToWorld(_course.Target(i));
        public JetRoverGripperGoal Goal(int i) => (JetRoverGripperGoal)(int)_course.Goal(i);

        public int PathCount => _course.PathCount;
        public Vector3 PathWorld(int i) => ToWorld(_course.PathPoint(i));
        public Vector3 BeadWorld => ToWorld(_course.Bead);

        /// <summary>The fixed levels, rad/s, in the order <see cref="CoverageSeconds"/> counts them; the varied one is index 4.</summary>
        public static int LevelCount => TrainingCourse.LevelCount;
        public static float Level(int i) => i < TrainingCourse.SpeedLevels.Length ? TrainingCourse.SpeedLevels[i] : float.NaN;
        public float CoverageSeconds(bool test, int level) => _course.CoverageSeconds(test ? TrainingPurpose.Test : TrainingPurpose.Train, level);

        /// <summary>The coverage counts, for the host to persist between sessions.</summary>
        public float[] SaveCoverage() => _course.Coverage.ToArray();
        public void LoadCoverage(float[] counts) { if (counts != null) _course.LoadCoverage(counts); }

        /// <summary>Starts the course; its seed (and id in the recording) comes from the clock.</summary>
        public void Start()
        {
            long now = _clock.NowTicks;
            uint id = (uint)(now ^ (now >> 32));
            _course.Start(id == 0 ? 1u : id, Seconds(now));
            Debug.Log($"[course] started, course {id}, {(Test ? "test" : "train")}");
        }

        public void Stop()
        {
            _course.Stop();
            Debug.Log("[course] stopped");
        }

        internal void Update(long now, CoreVec wrist, float gripperFraction, bool robotReady, bool disturbed) =>
            _course.Update(Seconds(now), wrist, gripperFraction, robotReady, disturbed);

        internal TrainingState State => _course.State;

        private double Seconds(long ticks) => ticks / (double)_clock.TicksPerSecond;

        private Vector3 ToWorld(CoreVec arm) => _armBase.TransformPoint(arm.ToUnity());
    }
}
