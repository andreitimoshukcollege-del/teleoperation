using System;
using System.Collections.Generic;
using Teleop.Core.Contracts;
using Teleop.Core.Registry;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Prediction
{
    /// <summary>
    /// One named entry of a <see cref="JointPredictorBank"/>: a label to show, a
    /// <see cref="Registries.JointPredictors"/> key, and the config to build it with. Two presets may
    /// share a key and differ in their parameters (two fits of <c>relay-replay-servo</c>, say).
    /// </summary>
    public sealed class JointPredictorPreset
    {
        public string Label { get; }
        public string RegistryKey { get; }
        public JointPredictorConfig Config { get; }

        public JointPredictorPreset(string label, string registryKey, JointPredictorConfig config)
        {
            if (string.IsNullOrEmpty(label)) throw new ArgumentException("a preset needs a label", nameof(label));
            if (string.IsNullOrEmpty(registryKey)) throw new ArgumentException("a preset needs a registry key", nameof(registryKey));
            Label = label;
            RegistryKey = registryKey;
            Config = config ?? throw new ArgumentNullException(nameof(config));
        }
    }

    /// <summary>
    /// Several joint predictors fed the same stream, one of them drawn (docs/adr/0023 decision 1).
    /// Every <see cref="ObserveCommand"/> and <see cref="ObserveState"/> goes to each <i>enabled</i>
    /// member; <see cref="TryPredict"/> answers with the <i>active</i> one, with the same signatures as
    /// <see cref="ICommandAwarePredictor"/>, so a host can call it where it called one predictor. It
    /// does not implement that interface: it is a container of predictors, not one, and
    /// <c>Teleop.Eval audit</c> rightly requires every implementation to be in the registry.
    ///
    /// <para><b>Switching</b> (<see cref="TrySetActive"/>) only changes which enabled member answers:
    /// every member keeps its history, so a switch is instant and shows a warmed-up prediction.
    /// <b>Enabling</b> a disabled member resets it: it missed the stream while off, and a predictor
    /// that silently skipped input would be wrong in a way nothing reports. Until it has seen a state
    /// again, <see cref="HasState"/> is false (the console shows it as warming up). The active member
    /// cannot be disabled. <see cref="Generation"/> counts each member's resets, so a scorer can tell
    /// that its numbers belong to an earlier run of that member.</para>
    ///
    /// <para>Allocates only when built. Feeding, predicting, switching and toggling allocate nothing
    /// (invariant 8).</para>
    /// </summary>
    public sealed class JointPredictorBank
    {
        private readonly string[] _labels;
        private readonly ICommandAwarePredictor[] _members;
        private readonly bool[] _enabled;
        private readonly bool[] _hasState;
        private readonly int[] _generation;
        private int _active;

        public JointPredictorBank(IReadOnlyList<string> labels, IReadOnlyList<ICommandAwarePredictor> members, int activeIndex)
        {
            if (labels == null) throw new ArgumentNullException(nameof(labels));
            if (members == null) throw new ArgumentNullException(nameof(members));
            if (members.Count == 0) throw new ArgumentException("a bank needs at least one member", nameof(members));
            if (labels.Count != members.Count) throw new ArgumentException("one label per member", nameof(labels));
            if (activeIndex < 0 || activeIndex >= members.Count) throw new ArgumentOutOfRangeException(nameof(activeIndex));

            int n = members.Count;
            _labels = new string[n];
            _members = new ICommandAwarePredictor[n];
            _enabled = new bool[n];
            _hasState = new bool[n];
            _generation = new int[n];
            for (int i = 0; i < n; i++)
            {
                _labels[i] = labels[i] ?? throw new ArgumentException($"label {i} is null", nameof(labels));
                _members[i] = members[i] ?? throw new ArgumentException($"member {i} is null", nameof(members));
                for (int j = 0; j < i; j++)
                {
                    if (string.Equals(_labels[j], _labels[i], StringComparison.Ordinal))
                    {
                        throw new ArgumentException($"two members are labelled '{_labels[i]}'", nameof(labels));
                    }
                }

                _enabled[i] = true;
            }

            _active = activeIndex;
        }

        /// <summary>
        /// A bank built from <paramref name="presets"/> through <see cref="Registries.JointPredictors"/>,
        /// all members enabled. An unknown registry key throws, naming the keys that exist.
        /// </summary>
        public static JointPredictorBank Create(IReadOnlyList<JointPredictorPreset> presets, int activeIndex)
        {
            if (presets == null) throw new ArgumentNullException(nameof(presets));
            var labels = new string[presets.Count];
            var members = new ICommandAwarePredictor[presets.Count];
            for (int i = 0; i < presets.Count; i++)
            {
                JointPredictorPreset preset = presets[i] ?? throw new ArgumentException($"preset {i} is null", nameof(presets));
                if (!Registries.JointPredictors.TryGetValue(preset.RegistryKey, out Func<JointPredictorConfig, ICommandAwarePredictor>? factory))
                {
                    throw new ArgumentException(
                        $"preset '{preset.Label}': no joint predictor '{preset.RegistryKey}' (registered: {string.Join(", ", Registries.JointPredictors.Keys)})",
                        nameof(presets));
                }

                labels[i] = preset.Label;
                members[i] = factory(preset.Config);
            }

            return new JointPredictorBank(labels, members, activeIndex);
        }

        public int Count => _members.Length;

        public int ActiveIndex => _active;

        public string ActiveLabel => _labels[_active];

        public string LabelAt(int index) => _labels[index];

        public ICommandAwarePredictor MemberAt(int index) => _members[index];

        public bool IsEnabled(int index) => _enabled[index];

        /// <summary>The member has observed a state since it was last reset: false means warming up.</summary>
        public bool HasState(int index) => _hasState[index];

        /// <summary>How many times the member has been reset (by <see cref="Reset"/> or by being enabled).</summary>
        public int Generation(int index) => _generation[index];

        /// <summary>The index of the member labelled <paramref name="label"/>, or -1.</summary>
        public int IndexOf(string label)
        {
            for (int i = 0; i < _labels.Length; i++)
            {
                if (string.Equals(_labels[i], label, StringComparison.Ordinal)) return i;
            }

            return -1;
        }

        /// <summary>Draws with member <paramref name="index"/> from now on. False (and no change) when it is disabled.</summary>
        public bool TrySetActive(int index)
        {
            if (index < 0 || index >= _members.Length || !_enabled[index]) return false;
            _active = index;
            return true;
        }

        /// <summary>
        /// Turns member <paramref name="index"/> on or off. Enabling a disabled member resets it.
        /// False (and no change) for the active member being disabled, or an index out of range.
        /// </summary>
        public bool TrySetEnabled(int index, bool enabled)
        {
            if (index < 0 || index >= _members.Length) return false;
            if (!enabled && index == _active) return false;
            if (enabled && !_enabled[index])
            {
                ResetMember(index);
            }

            _enabled[index] = enabled;
            return true;
        }

        public void ObserveCommand(long effectiveTicks, ReadOnlySpan<JointSample> targets, float speedRadPerSec)
        {
            for (int i = 0; i < _members.Length; i++)
            {
                if (_enabled[i]) _members[i].ObserveCommand(effectiveTicks, targets, speedRadPerSec);
            }
        }

        public void ObserveState(long sensedTicks, ReadOnlySpan<JointSample> joints)
        {
            for (int i = 0; i < _members.Length; i++)
            {
                if (!_enabled[i]) continue;
                _members[i].ObserveState(sensedTicks, joints);
                _hasState[i] = true;
            }
        }

        public bool TryPredict(long targetTicks, Span<JointSample> output, out int count) =>
            _members[_active].TryPredict(targetTicks, output, out count);

        /// <summary>Member <paramref name="index"/>'s prediction; false when it is disabled or has no state yet.</summary>
        public bool TryPredictMember(int index, long targetTicks, Span<JointSample> output, out int count)
        {
            count = 0;
            if (index < 0 || index >= _members.Length || !_enabled[index]) return false;
            return _members[index].TryPredict(targetTicks, output, out count);
        }

        /// <summary>Resets every member, enabled or not, and keeps the active one and the enabled set.</summary>
        public void Reset()
        {
            for (int i = 0; i < _members.Length; i++)
            {
                ResetMember(i);
            }
        }

        /// <summary>The active member's diagnostics.</summary>
        public PredictorDiagnostics Diagnostics => _members[_active].Diagnostics;

        private void ResetMember(int index)
        {
            _members[index].Reset();
            _hasState[index] = false;
            _generation[index]++;
        }
    }
}
