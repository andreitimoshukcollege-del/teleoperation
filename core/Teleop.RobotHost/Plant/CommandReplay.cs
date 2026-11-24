using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Relay;

namespace Teleop.RobotHost.Plant
{
    /// <summary>
    /// Runs a recorded <c>robothost.jlog</c>'s joint commands (<c>JC</c>) back through a <see cref="GenericArmPlant"/>, with its
    /// relay feedback (<c>RI</c>) given to the plant where it was recorded, so each command is judged from the measured pose it
    /// was judged from then (docs/adr/0020 amendment 2026-11-24, <c>Teleop.RobotHost --replay-commands</c>). With
    /// <see cref="HostCheckOnFail.Refuse"/> every outcome must be the recorded one; with <see cref="HostCheckOnFail.Clamp"/> it
    /// says which refusals become clamps, how far along, and which stay held.
    /// <para>Open loop: the feedback is what the recorded arm did, so after a clamp the arm is still measured where it held,
    /// not moving toward the clamped targets. Each command's outcome is right for the pose it was measured in; a run of
    /// refusals would be shorter in a live clamp, where the arm follows. Commands ignored while parking (<c>P</c>) never reached
    /// the plant and are skipped; RobotHost's own park waypoints are not replayed (they do not move a measured joint).</para>
    /// </summary>
    public sealed class CommandReplay
    {
        private readonly ReplayRelay _relay = new ReplayRelay();
        private readonly GenericArmPlant _plant;
        private readonly JointTarget[] _targets = new JointTarget[64];
        private readonly JointFeedbackEntry[] _feedback = new JointFeedbackEntry[RelayProtocol.MaxJointsPerMessage];
        private long _lastStep;

        public CommandReplay(GenericArmPlantConfig config, HostCheckOnFail onFail)
        {
            _plant = new GenericArmPlant(config, _relay) { OnHostCheckFail = onFail };
        }

        /// <summary>(recorded outcome, replayed outcome) → count.</summary>
        public SortedDictionary<string, int> Outcomes { get; } = new SortedDictionary<string, int>(StringComparer.Ordinal);

        /// <summary>Every clamped command's alpha, in order.</summary>
        public List<float> Alphas { get; } = new List<float>();

        /// <summary>How many checks each clamp asked.</summary>
        public List<int> ClampChecks { get; } = new List<int>();

        /// <summary>How many checks each clamp search that found nothing (replayed <c>R</c>) asked: 1 when the line's start failed.</summary>
        public List<int> HeldChecks { get; } = new List<int>();

        /// <summary>Wall time of each command's <see cref="GenericArmPlant.CommandJointAngles"/>, microseconds, by replayed outcome letter.</summary>
        public Dictionary<char, List<double>> Micros { get; } = new Dictionary<char, List<double>>();

        /// <summary>Commands whose replayed outcome differs from the recorded one, the first few as "seq recorded→replayed".</summary>
        public List<string> FirstMismatches { get; } = new List<string>();

        public int Mismatches { get; private set; }
        public int Commands { get; private set; }
        public GenericArmPlant Plant => _plant;

        /// <summary>Takes one line of the file, in the file's order.</summary>
        public void Line(string line)
        {
            if (line.StartsWith("RI|", StringComparison.Ordinal))
            {
                string[] f = line.Split('|');
                if (f.Length < 3) return;
                long t = long.Parse(f[1], CultureInfo.InvariantCulture);
                int n = 0;
                if (f[2].Length > 0)
                {
                    foreach (string entry in f[2].Split(';'))
                    {
                        string[] p = entry.Split(':');
                        if (p.Length != 3 || n == _feedback.Length) continue;
                        _feedback[n++] = new JointFeedbackEntry(byte.Parse(p[0], CultureInfo.InvariantCulture), p[1] == "1",
                            float.Parse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture));
                    }
                }

                _relay.Set(_feedback, n);
                // The plant's Step ignores a time not after its last; the recording's RI times only go forward.
                _lastStep = Math.Max(_lastStep + 1, t);
                _plant.Step(_lastStep);
            }
            else if (line.StartsWith("JC|", StringComparison.Ordinal))
            {
                string[] f = line.Split('|');
                if (f.Length < 7 || f[5].Length != 1) return;
                char recorded = f[5][0];
                if (recorded == 'P') return;
                long capture = long.Parse(f[4], CultureInfo.InvariantCulture);
                int n = 0;
                if (f[6].Length > 0)
                {
                    foreach (string target in f[6].Split(';'))
                    {
                        string[] p = target.Split(':');
                        if (p.Length != 3 || n == _targets.Length) continue;
                        _targets[n++] = new JointTarget(byte.Parse(p[0], CultureInfo.InvariantCulture),
                            float.Parse(p[1], NumberStyles.Float, CultureInfo.InvariantCulture), float.Parse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture));
                    }
                }

                Commands++;
                long started = Stopwatch.GetTimestamp();
                JointCommandOutcome outcome = _plant.CommandJointAngles(_targets.AsSpan(0, n), capture);
                double micros = (Stopwatch.GetTimestamp() - started) * 1e6 / Stopwatch.Frequency;
                char replayed = Recording.SessionLines.Outcome(outcome);
                if (!Micros.TryGetValue(replayed, out List<double>? list)) Micros[replayed] = list = new List<double>();
                list.Add(micros);

                string key = recorded + ">" + replayed;
                Outcomes[key] = Outcomes.TryGetValue(key, out int count) ? count + 1 : 1;
                if (outcome == JointCommandOutcome.Clamped)
                {
                    Alphas.Add(_plant.LastClamp.Alpha);
                    ClampChecks.Add(_plant.LastClamp.Checks);
                }
                else if (outcome == JointCommandOutcome.Refused && _plant.LastClamp.Checks > 0)
                {
                    HeldChecks.Add(_plant.LastClamp.Checks);
                }

                if (recorded != replayed)
                {
                    Mismatches++;
                    if (FirstMismatches.Count < 5) FirstMismatches.Add(f[3] + " " + recorded + ">" + replayed);
                }
            }
        }

        /// <summary>A relay that hands the plant the one feedback datagram the replay last set, once.</summary>
        private sealed class ReplayRelay : IRelayClient
        {
            private JointFeedbackEntry[] _entries = Array.Empty<JointFeedbackEntry>();
            private int _count = -1;

            public void Set(JointFeedbackEntry[] entries, int count)
            {
                _entries = entries;
                _count = count;
            }

            public void Send(ReadOnlySpan<JointTarget> targets)
            {
            }

            public void SendTorque(bool on)
            {
            }

            public bool TryReceiveFeedback(Span<JointFeedbackEntry> entriesBuffer, out int entryCount)
            {
                entryCount = 0;
                if (_count < 0) return false;
                _entries.AsSpan(0, _count).CopyTo(entriesBuffer);
                entryCount = _count;
                _count = -1;
                return true;
            }
        }
    }
}
