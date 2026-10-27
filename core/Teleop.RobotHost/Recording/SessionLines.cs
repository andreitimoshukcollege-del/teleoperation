using System;
using System.Globalization;
using System.Text;
using Teleop.Core.Types;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// RobotHost's records in the trainer's session format v1 (trainer/contract/SESSION_FORMAT.md,
    /// docs/adr/0021 decision 3): one <c>|</c>-separated line per record, lists as
    /// <c>a:b;c:d</c>, floats in round-trip form with an invariant <c>.</c>. Every time is this
    /// process's <c>Stopwatch</c> ticks, which on the Jetson are CLOCK_MONOTONIC nanoseconds: the
    /// same clock the driver's <c>time.monotonic_ns()</c> reads, so the two logs line up.
    /// </summary>
    public static class SessionLines
    {
        /// <summary>Which way <c>GenericArmPlant.CommandJointAngles</c> took a joint command.</summary>
        public static char Outcome(JointCommandOutcome outcome) => outcome switch
        {
            JointCommandOutcome.Accepted => 'A',
            JointCommandOutcome.Stale => 'S',
            JointCommandOutcome.Refused => 'R',
            JointCommandOutcome.Parked => 'P',
            _ => '?',
        };

        public static string Keepalive(long t, long operatorTicks, long operatorTicksPerSecond) =>
            "KA|" + I(t) + "|" + I(operatorTicks) + "|" + I(operatorTicksPerSecond);

        public static string JointCommand(long t, long tDeliver, uint sequence, long captureTicks, JointCommandOutcome outcome,
            ReadOnlySpan<JointTarget> targets)
        {
            var b = new StringBuilder(32 + targets.Length * 24);
            b.Append("JC|").Append(I(t)).Append('|').Append(I(tDeliver)).Append('|').Append(sequence.ToString(CultureInfo.InvariantCulture))
                .Append('|').Append(I(captureTicks)).Append('|').Append(Outcome(outcome)).Append('|');
            for (int i = 0; i < targets.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(targets[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(F(targets[i].Angle))
                    .Append(':').Append(F(targets[i].Speed));
            }

            return b.ToString();
        }

        /// <summary>Targets sent to the relay: the pulse as sent (a float) and the pulses per second.</summary>
        public static string RelayOut(long t, ReadOnlySpan<JointTarget> targets)
        {
            var b = new StringBuilder(16 + targets.Length * 20);
            b.Append("RO|").Append(I(t)).Append('|');
            for (int i = 0; i < targets.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(targets[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(F(targets[i].Angle))
                    .Append(':').Append(F(targets[i].Speed));
            }

            return b.ToString();
        }

        public static string RelayIn(long t, ReadOnlySpan<JointFeedbackEntry> entries)
        {
            var b = new StringBuilder(16 + entries.Length * 16);
            b.Append("RI|").Append(I(t)).Append('|');
            for (int i = 0; i < entries.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(entries[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(entries[i].Valid ? '1' : '0')
                    .Append(':').Append(F(entries[i].Pulse));
            }

            return b.ToString();
        }

        public static string Frame(long t, uint sequence, long robotSendTicks, long sensedTicks, long echoOperatorTicks,
            long echoRobotRecvTicks, ReadOnlySpan<JointSample> joints)
        {
            var b = new StringBuilder(64 + joints.Length * 20);
            b.Append("FR|").Append(I(t)).Append('|').Append(sequence.ToString(CultureInfo.InvariantCulture)).Append('|')
                .Append(I(robotSendTicks)).Append('|').Append(I(sensedTicks)).Append('|').Append(I(echoOperatorTicks)).Append('|')
                .Append(I(echoRobotRecvTicks)).Append('|');
            for (int i = 0; i < joints.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(joints[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(joints[i].Sensed ? '1' : '0')
                    .Append(':').Append(F(joints[i].Value));
            }

            return b.ToString();
        }

        /// <summary>A torque switch sent to the relay (docs/adr/0023 decision 5): 1 on, 0 off.</summary>
        public static string Torque(long t, bool on) => "TQ|" + I(t) + "|" + (on ? '1' : '0');

        /// <summary>An arm-power request from the headset (docs/adr/0023 decision 5).</summary>
        public static string PowerRequest(long t, uint sequence, ArmPowerRequest request, long operatorTicks) =>
            "PQ|" + I(t) + "|" + sequence.ToString(CultureInfo.InvariantCulture) + "|" + RequestName(request) + "|" + I(operatorTicks);

        /// <summary>The arm-power state after a change, or a refused relax (holding with a reason).</summary>
        public static string PowerState(long t, ArmPowerState state, ArmPowerReason reason) =>
            "PS|" + I(t) + "|" + StateName(state) + "|" + ReasonName(reason);

        /// <summary>A park waypoint as given to the plant: <c>motor:angleRad:speedRadPerSec;...</c>.</summary>
        public static string Park(long t, ReadOnlySpan<JointTarget> targets)
        {
            var b = new StringBuilder(16 + targets.Length * 24);
            b.Append("PK|").Append(I(t)).Append('|');
            for (int i = 0; i < targets.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(targets[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(F(targets[i].Angle))
                    .Append(':').Append(F(targets[i].Speed));
            }

            return b.ToString();
        }

        /// <summary>The operator's client kind and motion mode, from its heartbeat.</summary>
        public static string OperatorMode(long t, string kind, string mode, string endpoint) =>
            "OM|" + I(t) + "|" + Clean(kind) + "|" + Clean(mode) + "|" + Clean(endpoint);

        /// <summary>
        /// The headset's training course (docs/adr/0024 decision 7):
        /// <c>TK|t|courseId|round|task|phase|purpose|speedLevel|score|disturbed</c>, the level in rad/s or
        /// <c>varied</c>, the score as the headset counts it (final in phase <c>done</c>), disturbed 0 or 1.
        /// </summary>
        public static string Training(long t, in TrainingState s) =>
            "TK|" + I(t) + "|" + s.CourseId.ToString(CultureInfo.InvariantCulture) + "|" + s.Round.ToString(CultureInfo.InvariantCulture) +
            "|" + TrainingTaskCodec.Name(s.Task) + "|" + TrainingTaskCodec.Name(s.Phase) + "|" + TrainingTaskCodec.Name(s.Purpose) +
            "|" + (float.IsNaN(s.SpeedLevel) ? "varied" : F(s.SpeedLevel)) + "|" + F(s.Score) + "|" + (s.Disturbed ? "1" : "0");

        /// <summary>
        /// What the headset does to its own link and view (docs/adr/0023 amendment):
        /// <c>HD|t|lagMs|jitterMs|lossPercent|addedDelayMs|seed</c>, the lag <c>NaN</c> for a named profile.
        /// Any non-zero value means the session is not a real-network measurement.
        /// </summary>
        public static string HeadsetConditions(long t, in HeadsetConditions c) =>
            "HD|" + I(t) + "|" + F(c.LagMs) + "|" + F(c.JitterMs) + "|" + F(c.LossPercent) + "|" + F(c.AddedDelayMs) + "|" +
            c.Seed.ToString(CultureInfo.InvariantCulture);

        public static string KindName(OperatorClientKind kind) => kind switch
        {
            OperatorClientKind.Headset => "headset",
            OperatorClientKind.Tool => "tool",
            _ => "?" + ((byte)kind).ToString(CultureInfo.InvariantCulture),
        };

        public static string ModeName(OperatorMotionMode mode) => mode switch
        {
            OperatorMotionMode.Off => "off",
            OperatorMotionMode.Paused => "paused",
            OperatorMotionMode.On => "on",
            _ => "?",
        };

        public static string RequestName(ArmPowerRequest request) => request switch
        {
            ArmPowerRequest.None => "none",
            ArmPowerRequest.Relax => "relax",
            ArmPowerRequest.Hold => "hold",
            _ => "?" + ((byte)request).ToString(CultureInfo.InvariantCulture),
        };

        public static string StateName(ArmPowerState state) => state switch
        {
            ArmPowerState.Holding => "holding",
            ArmPowerState.Parking => "parking",
            ArmPowerState.Relaxed => "relaxed",
            _ => "?",
        };

        public static string ReasonName(ArmPowerReason reason) => reason switch
        {
            ArmPowerReason.None => "none",
            ArmPowerReason.NoRestPose => "no-rest-pose",
            ArmPowerReason.PathCollides => "path-collides",
            ArmPowerReason.ParkTimedOut => "park-timed-out",
            ArmPowerReason.BoardNotAnswering => "board-not-answering",
            ArmPowerReason.NoCollisionModel => "no-collision-model",
            ArmPowerReason.UnknownRequest => "unknown-request",
            ArmPowerReason.HoldRequested => "hold-requested",
            _ => "?" + ((byte)reason).ToString(CultureInfo.InvariantCulture),
        };

        public static string Subscriber(long t, string endpoint) => "SS|" + I(t) + "|" + Clean(endpoint);

        public static string Sender(long t, string endpoint) => "JS|" + I(t) + "|" + Clean(endpoint);

        public static string Uplink(long t, string profile, ulong seed) =>
            "UP|" + I(t) + "|" + Clean(profile) + "|" + seed.ToString(CultureInfo.InvariantCulture);

        private static string I(long value) => value.ToString(CultureInfo.InvariantCulture);

        private static string F(float value) => value.ToString("R", CultureInfo.InvariantCulture);

        /// <summary>A free-text field may not carry the separators.</summary>
        private static string Clean(string text) => text.Replace('|', '/').Replace(';', ',').Replace('\n', ' ').Replace('\r', ' ');
    }
}
