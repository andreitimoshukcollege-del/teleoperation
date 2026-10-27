using System;

namespace Teleop.RobotHost.Recording
{
    /// <summary>The robot-side uplink impairment when a session opens (<c>conditions.network</c>).</summary>
    public sealed class UplinkCondition
    {
        public UplinkCondition(string profile, ulong seed, string? schedule = null)
        {
            Profile = profile ?? throw new ArgumentNullException(nameof(profile));
            Seed = seed;
            Schedule = schedule;
        }

        /// <summary>A Core profile name, or <c>none</c>.</summary>
        public string Profile { get; }

        public ulong Seed { get; }

        /// <summary>The schedule's spec when one is running (docs/adr/0021 decision 13), else null.</summary>
        public string? Schedule { get; }

        public static readonly UplinkCondition None = new UplinkCondition("none", 0);
    }

    /// <summary>What <see cref="AutoSessionRecorder"/> knows when it opens a session; the writer adds the rest (builds, profile).</summary>
    public sealed class AutoSessionSpec
    {
        public AutoSessionSpec(DateTime startedUtc, string sender, UplinkCondition uplink, string reason)
        {
            StartedUtc = startedUtc;
            Sender = sender ?? throw new ArgumentNullException(nameof(sender));
            Uplink = uplink ?? throw new ArgumentNullException(nameof(uplink));
            Reason = reason ?? throw new ArgumentNullException(nameof(reason));
        }

        public DateTime StartedUtc { get; }

        /// <summary>The joint-command sender, <c>ip:port</c>; <c>conditions.operator</c> is <c>headset &lt;sender&gt;</c> (only a headset opens one).</summary>
        public string Sender { get; }

        public UplinkCondition Uplink { get; }

        /// <summary>Why it opened (first command after idle, a split, a rotation), for the log and <c>notes</c>.</summary>
        public string Reason { get; }
    }

    /// <summary>
    /// What <see cref="AutoSessionRecorder"/> drives: <see cref="SessionRecorder"/>, or a fake in tests.
    /// Every member is called from RobotHost's main loop and must not block or do I/O.
    /// </summary>
    public interface IAutoSessionSink
    {
        /// <summary>True while the control file names a session this recorder did not open itself (a manual one): the automatic recorder stays out.</summary>
        bool ControlFileBusy { get; }

        /// <summary>Opens an automatic session: records go to it from now on. Returns its generation (never 0), or 0 when refused.</summary>
        int BeginAuto(AutoSessionSpec spec);

        /// <summary>True while the session <paramref name="generation"/> is still the open one. It can end without <see cref="EndAuto"/>: a manual session took over, or an I/O error closed it.</summary>
        bool IsSessionActive(int generation);

        /// <summary>Closes the session <paramref name="generation"/>, if it is still open.</summary>
        void EndAuto(int generation, string reason);
    }
}
