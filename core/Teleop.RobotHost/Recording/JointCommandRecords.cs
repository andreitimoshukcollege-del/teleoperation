using System;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Plant;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// The records of one joint command the JetRover's plant has judged (trainer/contract/SESSION_FORMAT.md): its <c>JC</c>, and
    /// for a clamped one (outcome <c>C</c>, docs/adr/0020 amendment 2026-11-24) the <c>JK</c> after it. The <c>RO</c> of what
    /// went to the relay is written by <see cref="RecordingRelayClient"/> as the plant sends it, so it comes first.
    /// </summary>
    public static class JointCommandRecords
    {
        /// <param name="t">When the socket was read.</param>
        /// <param name="tDeliver">When the command reached the plant; also the <c>JK</c>'s time.</param>
        /// <param name="clamp">The plant's <see cref="GenericArmPlant.LastClamp"/> for this command.</param>
        public static void Write(Action<string> record, long t, long tDeliver, uint sequence, long captureTicks, JointCommandOutcome outcome,
                                 ReadOnlySpan<JointTarget> targets, in LineClampResult clamp)
        {
            record(SessionLines.JointCommand(t, tDeliver, sequence, captureTicks, outcome, targets));
            if (outcome == JointCommandOutcome.Clamped)
            {
                record(SessionLines.Clamp(tDeliver, sequence, clamp.Alpha));
            }
        }
    }
}
