using System;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// The state records of one session's <c>robothost.jlog</c>: <c>SS</c> (the joint-state
    /// subscriber), <c>JS</c> (the joint-command sender), <c>UP</c> (the robot-side uplink) and <c>OM</c>
    /// (the operator's client kind and motion mode, from its heartbeat), each
    /// written when it differs from what <i>this session's</i> file already says. <see cref="Reset"/>
    /// at the start of every session, then the current state, makes every file self-contained
    /// (docs/adr/0023 decision 4): a session that opens while the headset is already connected still
    /// names its subscriber and sender. <c>UP none</c> is the format's starting state, so it is written
    /// only as a change. Main loop only.
    /// </summary>
    public sealed class SessionStateLines
    {
        private readonly Action<string> _record;
        private string? _subscriber;
        private string? _sender;
        private string _uplink = "none";
        private ulong _uplinkSeed;
        private string? _operator;

        public SessionStateLines(Action<string> record)
        {
            _record = record ?? throw new ArgumentNullException(nameof(record));
        }

        /// <summary>A new session: its file says nothing yet.</summary>
        public void Reset()
        {
            _subscriber = null;
            _sender = null;
            _uplink = "none";
            _uplinkSeed = 0;
            _operator = null;
        }

        /// <summary>The operator message's client kind and mode from <paramref name="endpoint"/> (docs/adr/0023 decisions 4 and 5).</summary>
        public void Operator(long t, string kind, string mode, string? endpoint)
        {
            if (endpoint == null) return;
            string key = kind + " " + mode + " " + endpoint;
            if (string.Equals(key, _operator, StringComparison.Ordinal)) return;
            _operator = key;
            _record(SessionLines.OperatorMode(t, kind, mode, endpoint));
        }

        public void Subscriber(long t, string? endpoint)
        {
            if (endpoint == null || string.Equals(endpoint, _subscriber, StringComparison.Ordinal)) return;
            _subscriber = endpoint;
            _record(SessionLines.Subscriber(t, endpoint));
        }

        public void Sender(long t, string? endpoint)
        {
            if (endpoint == null || string.Equals(endpoint, _sender, StringComparison.Ordinal)) return;
            _sender = endpoint;
            _record(SessionLines.Sender(t, endpoint));
        }

        public void Uplink(long t, string profile, ulong seed)
        {
            if (profile == null) throw new ArgumentNullException(nameof(profile));
            if (profile == _uplink && (profile == "none" || seed == _uplinkSeed)) return;
            _uplink = profile;
            _uplinkSeed = seed;
            _record(SessionLines.Uplink(t, profile, seed));
        }
    }
}
