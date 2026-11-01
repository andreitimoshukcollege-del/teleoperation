using System;
using Teleop.RobotArm.Wire;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// The state records of one session's <c>robothost.jlog</c>: <c>SS</c> (the joint-state
    /// subscriber), <c>JS</c> (the joint-command sender), <c>UP</c> (the robot-side uplink) and <c>OM</c>
    /// (the operator's client kind and motion mode, from its heartbeat) and <c>TK</c> (the headset's
    /// training course, docs/adr/0024), each
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
        private TrainingState? _training;
        private HeadsetConditions? _headset;

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
            _training = null;
            _headset = null;
        }

        /// <summary>
        /// The headset's disturbance, added delay and hand filter: written when they change, and at the start of a
        /// session whenever anything is on or the hand filter is reported (all off and nothing said about the hands is
        /// the format's starting state).
        /// </summary>
        public void Headset(long t, in HeadsetConditions conditions)
        {
            if (_headset.HasValue ? _headset.Value.Equals(conditions) : !conditions.Disturbed && !conditions.ReportsHandFilter)
            {
                return;
            }

            _headset = conditions;
            _record(SessionLines.HeadsetConditions(t, conditions));
        }

        /// <summary>
        /// The course state from the headset: written when its label changes (the running score aside;
        /// the final score comes with phase done). A new session's file starts with nothing, so a course
        /// that is stopped (no task, idle) is written only as a change.
        /// </summary>
        public void Training(long t, in TrainingState state)
        {
            if (_training.HasValue ? _training.Value.SameLabel(state) : state.Task == TrainingTask.None && state.Phase == TrainingPhase.Idle)
            {
                return;
            }

            _training = state;
            _record(SessionLines.Training(t, state));
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
