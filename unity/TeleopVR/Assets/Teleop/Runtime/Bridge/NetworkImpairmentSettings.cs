using System;
using Teleop.Core.Types;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// The whole network-disturbance configuration: one <see cref="NetworkImpairment"/> per axis,
    /// and the one place that folds them into a Core <see cref="NetworkProfile"/>.
    ///
    /// <b>This type holds no impairment logic and must never grow any.</b> Every delay, drop and
    /// reorder decision is made by <c>Teleop.Core.Transport.EmulatedTransport</c> from the profile
    /// this produces; the axes describe, they do not compute. That keeps this on the right side of
    /// Teleop/CLAUDE.md's rule that a Bridge file containing a coefficient or a buffering decision
    /// is a bug.
    ///
    /// <b>Adding an axis</b> is a new file in <c>Impairments/</c> plus a field here and one line in
    /// each of <see cref="ToProfile"/>, <see cref="Describe"/>, <see cref="ValueEquals"/> and
    /// <see cref="CopyTo"/> — all four of which are flat lists over <see cref="AllAxes"/>, so in
    /// practice it is the field and the array entry. Existing axes are not touched.
    ///
    /// The axes are iterated through an explicitly-built array rather than found by reflection:
    /// IL2CPP strips what nothing references and has no runtime codegen (root CLAUDE.md invariant
    /// 5), so a reflective scan would work in the Editor and fail on device. This is the same
    /// static-registration discipline <c>Registry/Registries.cs</c> follows for the same reason.
    ///
    /// <b>Relationship to the frozen profile suite.</b> The impairment runs through the identical
    /// <c>EmulatedTransport</c> the sweeps use, so the mechanism is shared; only the authoring
    /// differs. <see cref="NetworkProfilePresets"/> closes that in the safe direction — a preset
    /// loads a frozen profile's exact values from <c>NetworkProfileCatalog</c> into these axes, so
    /// "feel what `300ms-60j-2loss-bursty` is like" uses the numbers the sweep citing that name
    /// used. The suite itself stays frozen and is never authored from here: its numbers must not
    /// move, because recorded results cite those names (docs/adr/0004, 0005, 0006). A preset is a
    /// starting point the axes then override freely, and the moment one is edited
    /// <see cref="NetworkImpairmentController"/> reports it as "(modified)" rather than continuing
    /// to claim the name.
    /// </summary>
    [Serializable]
    public sealed class NetworkImpairmentSettings
    {
        [Header("Delay source -- a recorded trace supersedes the two axes below")]
        public DelayTraceImpairment DelayTrace = new DelayTraceImpairment();

        [Header("Axes")]
        public DelayImpairment Delay = new DelayImpairment();

        public JitterImpairment Jitter = new JitterImpairment();

        public LossImpairment Loss = new LossImpairment();

        public ReorderImpairment Reorder = new ReorderImpairment();

        /// <summary>
        /// Every axis, in display order. Rebuilt lazily and cached, because Unity deserializes the
        /// fields above without running any constructor that could populate it, so this cannot be
        /// built once at construction and trusted.
        /// </summary>
        private NetworkImpairment[] _allAxes;

        public NetworkImpairment[] AllAxes
        {
            get
            {
                if (_allAxes == null || _allAxes.Length != 5 || _allAxes[0] != DelayTrace)
                {
                    _allAxes = new NetworkImpairment[] { DelayTrace, Delay, Jitter, Loss, Reorder };
                }

                return _allAxes;
            }
        }

        /// <summary>
        /// True when at least one axis is enabled. When false the caller should install no emulator
        /// at all rather than one configured to a neutral profile — see
        /// <see cref="NetworkImpairmentController"/>, which relies on this to keep the
        /// impairment-off path byte-identical to the unimpaired build rather than merely equivalent
        /// to it.
        /// </summary>
        public bool AnyEnabled
        {
            get
            {
                NetworkImpairment[] axes = AllAxes;
                for (int i = 0; i < axes.Length; i++)
                {
                    if (axes[i].Enabled)
                    {
                        return true;
                    }
                }

                return false;
            }
        }

        /// <summary>True when delay is being replayed from a trace, so the caller must install trace mode.</summary>
        public bool UsesDelayTrace => DelayTrace.Enabled;

        /// <summary>
        /// Composes the enabled axes into a Core profile. A disabled axis contributes nothing — its
        /// neutral value, never a default — and <see cref="NetworkProfileDraft.Build"/> does the
        /// clamping, so no axis can forget to.
        /// </summary>
        public NetworkProfile ToProfile(long ticksPerSecond)
        {
            var draft = default(NetworkProfileDraft);

            NetworkImpairment[] axes = AllAxes;
            for (int i = 0; i < axes.Length; i++)
            {
                if (axes[i].Enabled)
                {
                    axes[i].Contribute(ref draft, ticksPerSecond);
                }
            }

            return draft.Build();
        }

        /// <summary>
        /// One-line summary of what is switched on, for the HUD and for the Play-mode log line that
        /// records what the operator actually had enabled — impairment state is not otherwise
        /// recoverable from a <c>.tlog</c>, and a session recorded under unknown conditions is not a
        /// result.
        ///
        /// A trace-superseded delay or jitter axis is omitted even when still ticked, because
        /// <see cref="ToProfile"/> zeroed it: a summary listing an axis the run is not applying is
        /// how a recording ends up described wrongly.
        /// </summary>
        public string Describe()
        {
            if (!AnyEnabled)
            {
                return "none";
            }

            bool traced = DelayTrace.Enabled;
            string result = string.Empty;

            NetworkImpairment[] axes = AllAxes;
            for (int i = 0; i < axes.Length; i++)
            {
                NetworkImpairment axis = axes[i];
                if (!axis.Enabled)
                {
                    continue;
                }

                if (traced && (axis == Delay || axis == Jitter))
                {
                    continue;
                }

                if (result.Length > 0)
                {
                    result += " ";
                }

                result += axis.DescribeSettings();
            }

            return result;
        }

        /// <summary>
        /// Value equality across every axis, so <see cref="NetworkImpairmentController"/> can detect
        /// an Inspector edit without rebuilding transports every frame.
        /// </summary>
        public bool ValueEquals(NetworkImpairmentSettings other)
        {
            if (other == null)
            {
                return false;
            }

            NetworkImpairment[] mine = AllAxes;
            NetworkImpairment[] theirs = other.AllAxes;
            for (int i = 0; i < mine.Length; i++)
            {
                if (!mine[i].ValueEquals(theirs[i]))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>True when any axis's <b>checkbox</b> differs, ignoring numeric fields.</summary>
        public bool EnabledStateEquals(NetworkImpairmentSettings other)
        {
            if (other == null)
            {
                return false;
            }

            NetworkImpairment[] mine = AllAxes;
            NetworkImpairment[] theirs = other.AllAxes;
            for (int i = 0; i < mine.Length; i++)
            {
                if (mine[i].Enabled != theirs[i].Enabled)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>Field-by-field copy into <paramref name="destination"/>, avoiding an allocation per change check.</summary>
        public void CopyTo(NetworkImpairmentSettings destination)
        {
            NetworkImpairment[] mine = AllAxes;
            NetworkImpairment[] theirs = destination.AllAxes;
            for (int i = 0; i < mine.Length; i++)
            {
                mine[i].CopyTo(theirs[i]);
            }
        }
    }
}
