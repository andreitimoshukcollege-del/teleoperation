using System;
using Teleop.Core.Contracts;

namespace Teleop.Bridge
{
    /// <summary>
    /// One axis of network impairment: an on/off flag, whatever parameters that axis needs, and the
    /// one method that folds them into the profile handed to <c>EmulatedTransport</c>.
    ///
    /// <b>An impairment here never impairs anything, and after docs/adr/0013 it does not even model
    /// anything.</b> It is an Inspector surface: serializable fields plus one method that constructs
    /// the real <c>Teleop.Core.Transport.Impairments</c> object. Every delay, drop and reorder
    /// decision, and every parameter clamp, now lives in Core.
    ///
    /// <b>Why this layer still exists at all</b> rather than putting Core's impairments directly in
    /// the Inspector: Unity serialization needs parameterless constructors and mutable public
    /// fields, while Core validates at construction and holds its parameters readonly. `[Range]`,
    /// `[Min]` and `[Tooltip]` are `UnityEngine` and can never appear in Core. And
    /// `[SerializeReference]` loses managed references when a type is renamed or moved, so binding
    /// scene data to Core type identities would make any later Core refactor a silent data-loss
    /// event here. <see cref="RobotArmProfileData"/> exists for exactly the same reasons.
    ///
    /// <b>Adding an axis.</b> Implement <c>INetworkImpairment</c> in Core, then add one file here
    /// deriving from this plus one field and one array entry in
    /// <see cref="NetworkImpairmentSettings"/>. No existing axis is touched, and the ceiling that
    /// used to make this only half-true is gone: a new *kind* of impairment is now a new Core file
    /// rather than a widening of a fixed six-field struct plus an edit at every construction site
    /// (docs/adr/0013).
    /// </summary>
    [Serializable]
    public abstract class NetworkImpairment
    {
        /// <summary>The checkbox. When false the axis contributes nothing — its neutral value, never a default.</summary>
        public bool Enabled;

        /// <summary>Short lower-case label used in log lines and the HUD, e.g. "delay".</summary>
        public abstract string AxisName { get; }

        /// <summary>
        /// Builds the Core impairment this axis's Inspector values describe. Called only when
        /// <see cref="Enabled"/> is true, so implementations never need to check it.
        ///
        /// <paramref name="loadedTrace"/> is the delay trace the host has already read off disk, or
        /// null when none is loaded. Only the trace axis uses it; every other axis ignores it. It is
        /// passed rather than fetched because Core cannot do I/O and an impairment must not know
        /// what a file is.
        ///
        /// Returning null is legal and means "nothing to install" -- the trace axis does that when
        /// its file could not be read, rather than silently substituting a different link.
        /// </summary>
        public abstract INetworkImpairment ToCoreImpairment(long ticksPerSecond, long[] loadedTrace);

        /// <summary>This axis's settings as one human-readable fragment, e.g. "delay 50ms". Enabled-only.</summary>
        public abstract string DescribeSettings();

        /// <summary>Value equality against another instance of the same concrete type, including <see cref="Enabled"/>.</summary>
        public abstract bool ValueEquals(NetworkImpairment other);

        /// <summary>Field-by-field copy into another instance of the same concrete type.</summary>
        public abstract void CopyTo(NetworkImpairment destination);
    }

    /// <summary>Shared millisecond-to-tick conversion, so every axis rounds identically.</summary>
    public static class ImpairmentUnits
    {
        public static long MsToTicks(float milliseconds, long ticksPerSecond)
        {
            if (milliseconds <= 0f)
            {
                return 0L;
            }

            return (long)(milliseconds / 1000.0 * ticksPerSecond);
        }

        public static float TicksToMs(long ticks, long ticksPerSecond) =>
            (float)(ticks * 1000.0 / ticksPerSecond);
    }
}
