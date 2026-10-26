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
