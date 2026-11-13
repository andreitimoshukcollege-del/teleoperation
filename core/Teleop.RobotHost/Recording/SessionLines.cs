using System;
using Teleop.RobotHost.ArmServer;
using Teleop.RobotArm.World;
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

        /// <summary>
        /// Targets a serial-chain plant sent its arm backend (session v2, docs/adr/0027 decision 6):
        /// <c>AO|t|motor:value;...</c>, radians for an arm joint, the 0..1 fraction for a hand, after the limits.
        /// </summary>
        public static string ArmOut(long t, ReadOnlySpan<JointSample> targets) => "AO|" + I(t) + "|" + Samples(targets);

        /// <summary>
        /// The arm's measured state as the backend reported it (session v2's truth): <c>QS|t|sensed|motor:value;...</c>,
        /// <c>sensed</c> on the host's clock, written every few milliseconds rather than at the controller's 1 kHz.
        /// </summary>
        public static string ControllerState(long t, long sensedTicks, ReadOnlySpan<byte> motorIds, ReadOnlySpan<double> values)
        {
            var b = new StringBuilder(32 + values.Length * 22);
            b.Append("QS|").Append(I(t)).Append('|').Append(I(sensedTicks)).Append('|');
            for (int i = 0; i < values.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(motorIds[i].ToString(CultureInfo.InvariantCulture)).Append(':').Append(values[i].ToString("R", CultureInfo.InvariantCulture));
            }

            return b.ToString();
        }

        private static string Samples(ReadOnlySpan<JointSample> samples)
        {
            var b = new StringBuilder(samples.Length * 16);
            for (int i = 0; i < samples.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(samples[i].MotorId.ToString(CultureInfo.InvariantCulture)).Append(':').Append(F(samples[i].Value));
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
        /// Any non-zero value means the session is not a real-network measurement. When the headset reports its hand
        /// filter (docs/adr/0029 amendment 7), four more fields follow:
        /// <c>|handMinCutoffHz|handPositionBeta|handRotationBeta|handDerivativeCutoffHz</c>, the min cutoff 0 for raw hands.
        /// </summary>
        public static string HeadsetConditions(long t, in HeadsetConditions c) =>
            "HD|" + I(t) + "|" + F(c.LagMs) + "|" + F(c.JitterMs) + "|" + F(c.LossPercent) + "|" + F(c.AddedDelayMs) + "|" +
            c.Seed.ToString(CultureInfo.InvariantCulture) +
            (c.ReportsHandFilter
                ? "|" + F(c.HandMinCutoffHz) + "|" + F(c.HandPositionBeta) + "|" + F(c.HandRotationBeta) + "|" + F(c.HandDerivativeCutoffHz)
                : "");

        // ---- the world (docs/adr/0031 amendment 2) ----

        /// <summary><c>WL|t|layout|seed</c>: a layout began (drawn from <c>seed</c> by Core's <c>SceneLayout</c>).</summary>
        public static string WorldLayout(long t, uint layout, ulong seed) =>
            "WL|" + I(t) + "|" + layout.ToString(CultureInfo.InvariantCulture) + "|" + seed.ToString(CultureInfo.InvariantCulture);

        /// <summary><c>WP|t|request|cause|id|x:y:z:qw:qx:qy:qz|outcome</c>: a placement settled; the pose is the one the object has now.</summary>
        public static string WorldPlace(long t, uint request, PlaceCause cause, in ObjectPose pose, PlaceOutcome outcome) =>
            "WP|" + I(t) + "|" + request.ToString(CultureInfo.InvariantCulture) + "|" + CauseName(cause) + "|" + pose.Id.ToString(CultureInfo.InvariantCulture) +
            "|" + Pose(pose) + "|" + OutcomeName(outcome);

        /// <summary><c>WO|t|sensed|revision|id:x:y:z:qw:qx:qy:qz;...</c>: where every object was, sensed on the host's clock.</summary>
        public static string WorldObjects(long t, long sensed, uint revision, ReadOnlySpan<ObjectPose> poses)
        {
            var b = new StringBuilder("WO|").Append(I(t)).Append('|').Append(I(sensed)).Append('|').Append(revision.ToString(CultureInfo.InvariantCulture)).Append('|');
            for (int i = 0; i < poses.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(poses[i].Id.ToString(CultureInfo.InvariantCulture)).Append(':').Append(Pose(poses[i]));
            }

            return b.ToString();
        }

        /// <summary><c>WF|t|snapshot|sensed|parts</c>: a world frame sent to the subscriber.</summary>
        public static string WorldFrame(long t, uint snapshot, long sensed, int parts) =>
            "WF|" + I(t) + "|" + snapshot.ToString(CultureInfo.InvariantCulture) + "|" + I(sensed) + "|" + parts.ToString(CultureInfo.InvariantCulture);

        /// <summary><c>WE|t|sequence|op|id|x:y:z:qw:qx:qy:qz|outcome</c>: an edit from the headset; outcome <c>pending</c> until it settles.</summary>
        public static string WorldEdit(long t, uint sequence, WorldEditOp op, in ObjectPose pose, PlaceOutcome? outcome) =>
            "WE|" + I(t) + "|" + sequence.ToString(CultureInfo.InvariantCulture) + "|" + EditName(op) + "|" + pose.Id.ToString(CultureInfo.InvariantCulture) +
            "|" + Pose(pose) + "|" + (outcome.HasValue ? OutcomeName(outcome.Value) : "pending");

        /// <summary><c>WZ|t|revision|id:scale;...</c>: every object's size factor on the scene file's size (docs/adr/0034 amendment 6), at the start and on change.</summary>
        public static string WorldSizes(long t, uint revision, ReadOnlySpan<byte> ids, ReadOnlySpan<float> scales)
        {
            var b = new StringBuilder("WZ|").Append(I(t)).Append('|').Append(revision.ToString(CultureInfo.InvariantCulture)).Append('|');
            for (int i = 0; i < ids.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(ids[i].ToString(CultureInfo.InvariantCulture)).Append(':').Append(scales[i].ToString("0.####", CultureInfo.InvariantCulture));
            }

            return b.ToString();
        }

        /// <summary><c>WC|t|sensed|arm:link:object:forceN;...</c>: contacts between the arms and objects (object 0 is the table); empty when they end.</summary>
        public static string WorldContacts(long t, long sensed, ReadOnlySpan<ArmServerContact> contacts)
        {
            var b = new StringBuilder("WC|").Append(I(t)).Append('|').Append(I(sensed)).Append('|');
            for (int i = 0; i < contacts.Length; i++)
            {
                if (i > 0) b.Append(';');
                b.Append(contacts[i].Arm.ToString(CultureInfo.InvariantCulture)).Append(':').Append(contacts[i].Link.ToString(CultureInfo.InvariantCulture))
                    .Append(':').Append(contacts[i].Object.ToString(CultureInfo.InvariantCulture)).Append(':').Append(F(contacts[i].ForceNewtons));
            }

            return b.ToString();
        }

        private static string Pose(in ObjectPose p) =>
            F(p.X) + ":" + F(p.Y) + ":" + F(p.Z) + ":" + F(p.Qw) + ":" + F(p.Qx) + ":" + F(p.Qy) + ":" + F(p.Qz);

        public static string CauseName(PlaceCause cause) => cause switch
        {
            PlaceCause.Layout => "layout",
            PlaceCause.Headset => "headset",
            PlaceCause.Reset => "reset",
            PlaceCause.Resize => "resize",
            _ => "?",
        };

        public static string EditName(WorldEditOp op) => op switch
        {
            WorldEditOp.Move => "move",
            WorldEditOp.NewLayout => "new-layout",
            WorldEditOp.ResetObject => "reset-object",
            WorldEditOp.ResetLayout => "reset-layout",
            WorldEditOp.Resize => "resize",
            _ => "?",
        };

        public static string OutcomeName(PlaceOutcome o) => o switch
        {
            PlaceOutcome.Applied => "applied",
            PlaceOutcome.OverlapsArm => "overlaps-arm",
            PlaceOutcome.OverlapsObject => "overlaps-object",
            PlaceOutcome.OutsideRegion => "outside-region",
            PlaceOutcome.UnknownObject => "unknown-object",
            PlaceOutcome.NotMovable => "not-movable",
            PlaceOutcome.ArmTouching => "arm-touching",
            PlaceOutcome.Busy => "busy",
            PlaceOutcome.InvalidPose => "invalid-pose",
            PlaceOutcome.TimedOut => "timed-out",
            _ => "?",
        };

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
