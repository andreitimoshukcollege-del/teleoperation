using System;
using Teleop.Core.Contracts;
using Teleop.Core.Transport.Impairments;
using UnityEngine;

namespace Teleop.Bridge
{
    /// <summary>
    /// The numbers behind the network-disturbance checkboxes, and the one place that turns them into
    /// the Core impairment set <c>EmulatedTransport</c> consumes.
    ///
    /// <b>This holds no impairment model, and since docs/adr/0013 it holds no impairment *types*
    /// either.</b> The impairments live in <c>core/Teleop.Core/Transport/Impairments/</c> — one file
    /// each, where a new kind of disturbance is actually added. What is here is a set of plain
    /// serializable fields and one method that builds Core objects from them.
    ///
    /// <b>Why any of this exists, given Unity uses Core directly everywhere else.</b> It does —
    /// Bridge constructs <c>EmulatedTransport</c>, <c>OperatorEndpoint</c>, <c>ClockSync</c>, and the
    /// Core impairments themselves. There is exactly one thing Unity cannot do with a Core object:
    /// draw it in the Inspector and save it into a scene. Unity's serializer only persists public
    /// mutable fields, and Core's impairments keep their parameters private and readonly precisely
    /// so they can be validated once at construction and never be wrong afterwards. So this type is
    /// the editable surface — the boxes an operator types into — and Core is the model.
    /// <see cref="RobotArmProfileData"/> exists for the same reason.
    ///
    /// This was briefly a per-axis class hierarchy under <c>Impairments/</c>, mirroring Core's
    /// file-per-impairment layout. That was removed: a plain number does not need its own class or
    /// its own file, and the duplicated shape read as though the impairments had never moved to Core
    /// at all. The split belongs in Core, where it earns its keep.
    ///
    /// <b>Adding an axis</b> is therefore a new file in Core, plus a field and one <c>if</c> here.
    /// </summary>
    [Serializable]
    public sealed class NetworkImpairmentSettings
    {
        /// <summary>
        /// Replay delay from a recorded trace instead of drawing it from
        /// <see cref="BaseDelayMs"/> + <see cref="JitterMs"/>.
        ///
        /// This is the only way to reach burst *delay* structure: `synthetic-burst`'s bursts live in
        /// its recorded samples and are not expressible as base + jitter, which is exactly why it
        /// matters — the Buffering result rests on that structure.
        ///
        /// Since docs/adr/0013 a trace no longer *supersedes* delay and jitter. With all three on
        /// they compose, meaning a recorded link plus an extra fixed hop, and the delays sum. That
        /// is a legal configuration rather than a contradiction to validate away.
        /// </summary>
        [Header("Delay source")]
        public bool EnableDelayTrace;

        /// <summary>
        /// Trace to replay, without the <c>.trace</c> extension. Resolved by
        /// <see cref="DelayTraceLoader"/>; the source of truth is <c>core/testdata/traces/</c> and no
        /// copy is committed under <c>unity/</c>, so install it with <c>just install-traces</c>
        /// (Editor) or <c>adb push</c> (device).
        /// </summary>
        public string TraceName = "synthetic-burst";

        [Header("Lag -- fixed one-way delay added to every datagram")]
        public bool EnableDelay;

        [Min(0f)]
        public float BaseDelayMs = 50f;

        [Header("Jitter -- uniform +/- variation on top of the delay")]
        public bool EnableJitter;

        /// <summary>
        /// Half-width, not full width: the draw is uniform over <c>[-JitterMs, +JitterMs]</c>, so 10
        /// here means a 20 ms spread. Legal with delay off, but the emulator clamps a negative total
        /// to zero rather than delivering a datagram before it was sent, so jitter alone is
        /// half-sided.
        /// </summary>
        [Min(0f)]
        public float JitterMs = 10f;

        [Header("Loss")]
        public bool EnableLoss;

        /// <summary>Probability a datagram is dropped, given the previous one got through.</summary>
        [Range(0f, 100f)]
        public float LossPercent = 1f;

        /// <summary>
        /// When off, loss is plain Bernoulli. When on, a drop makes the next drop far likelier —
        /// what real links do, and what actually breaks a jitter buffer, since burst *length* rather
        /// than average rate is the thing that hurts.
        /// </summary>
        public bool EnableBurstLoss;

        /// <summary>
        /// Probability the drop continues, given the previous datagram was dropped. Expected burst
        /// length is <c>1 / (1 - p)</c>, so 70% is bursts of ~3.3 and 90% is bursts of ~10.
        ///
        /// Capped below 100% by <see cref="MaxBurstContinuation"/>: Core treats exactly 1.0 as an
        /// absorbing state — once anything drops, everything drops until the transport is rebuilt.
        /// A legitimate total-outage model, but reaching it by dragging a slider to the end would
        /// read as a hang rather than a setting.
        /// </summary>
        [Range(0f, 100f)]
        public float BurstContinuationPercent = 70f;

        [Header("Reorder")]
        public bool EnableReorder;

        [Range(0f, 100f)]
        public float ReorderPercent = 2f;

        /// <summary>
        /// Extra delay applied to the datagrams reordering selects.
        ///
        /// <b>This must exceed the interval between sends to reorder anything at all.</b> Below that
        /// it is a delay spike on one datagram with everything still arriving in order. At the 48Hz
        /// this project sends at the interval is ~21 ms, so 30 ms is the smallest round number that
        /// actually reorders — lower it and the checkbox appears to do nothing.
        /// </summary>
        [Min(0f)]
        public float ReorderDelayMs = 30f;

        /// <summary>See <see cref="BurstContinuationPercent"/> for why this is not 1.0.</summary>
        public const double MaxBurstContinuation = 0.99;

        /// <summary>
        /// True when at least one axis is on. When false the caller installs no emulator at all
        /// rather than one configured to do nothing, which keeps the impairment-off path identical
        /// to the unimpaired build rather than merely equivalent to it.
        /// </summary>
        public bool AnyEnabled =>
            EnableDelayTrace || EnableDelay || EnableJitter || EnableLoss || EnableReorder;

        /// <summary>True when delay comes from a trace, so the caller must have one loaded.</summary>
        public bool UsesDelayTrace => EnableDelayTrace;

        /// <summary>
        /// Builds the Core impairment set. A disabled axis is omitted entirely, not included at a
        /// neutral value — legal because each Core impairment draws a constant number of random
        /// numbers per datagram regardless of its parameters, which makes a neutral axis
        /// indistinguishable from its absence (docs/adr/0013).
        ///
        /// <paramref name="loadedTrace"/> is the trace <see cref="DelayTraceLoader"/> already read,
        /// or null. Core cannot do file I/O, so the samples are passed in rather than fetched; a
        /// null trace with the axis enabled simply omits it, and the caller has already reported why.
        ///
        /// <b>Fresh Core instances on every call.</b> A Core impairment owns mutable model state and
        /// one RNG substream and may be bound to exactly one transport, so a link's uplink and
        /// downlink must each get their own set. Sharing them would either throw or make both
        /// directions lose and delay the same datagrams together.
        ///
        /// Values are clamped here because an Inspector field is operator-typed and Core's
        /// constructors throw on an out-of-range probability — a checkbox click must not surface as
        /// an exception.
        /// </summary>
        public INetworkImpairment[] CreateImpairments(long ticksPerSecond, long[] loadedTrace)
        {
            bool trace = EnableDelayTrace && loadedTrace != null;

            int count = (trace ? 1 : 0) + (EnableDelay ? 1 : 0) + (EnableJitter ? 1 : 0)
                + (EnableLoss ? 1 : 0) + (EnableReorder ? 1 : 0);
            var result = new INetworkImpairment[count];

            int at = 0;
            if (trace)
            {
                result[at++] = new TraceDelayImpairment(loadedTrace);
            }

            if (EnableDelay)
            {
                result[at++] = new FixedDelayImpairment(MsToTicks(BaseDelayMs, ticksPerSecond));
            }

            if (EnableJitter)
            {
                result[at++] = new UniformJitterImpairment(MsToTicks(JitterMs, ticksPerSecond));
            }

            if (EnableLoss)
            {
                double afterDelivered = Clamp01(LossPercent / 100.0);

                // Equal after-delivered/after-lost degenerates the Gilbert-Elliott chain to plain
                // Bernoulli -- the same identity docs/adr/0004 relies on for `150ms-20j-0.5loss`.
                // So "bursty off" needs no special case, here or in Core.
                double afterLost = EnableBurstLoss
                    ? Math.Min(Clamp01(BurstContinuationPercent / 100.0), MaxBurstContinuation)
                    : afterDelivered;

                result[at++] = new GilbertElliottLossImpairment(afterDelivered, afterLost);
            }

            if (EnableReorder)
            {
                result[at++] = new Teleop.Core.Transport.Impairments.ReorderImpairment(
                    Clamp01(ReorderPercent / 100.0), MsToTicks(ReorderDelayMs, ticksPerSecond));
            }

            return result;
        }

        /// <summary>
        /// One-line summary of what is on, for the HUD and for the Play-mode log line that records
        /// what the operator actually had enabled — impairment state is not otherwise recoverable
        /// from a <c>.tlog</c>, and a session recorded under unknown conditions is not a result.
        ///
        /// Every enabled axis is listed, including delay and jitter alongside a trace: since
        /// docs/adr/0013 those compose rather than conflict, so all three genuinely apply and a
        /// summary hiding two of them would describe the run wrongly.
        /// </summary>
        public string Describe()
        {
            if (!AnyEnabled)
            {
                return "none";
            }

            string result = string.Empty;

            if (EnableDelayTrace)
            {
                result = Append(result, $"delay-trace '{TraceName}'");
            }

            if (EnableDelay)
            {
                result = Append(result, $"delay {BaseDelayMs:0.#}ms");
            }

            if (EnableJitter)
            {
                result = Append(result, $"jitter ±{JitterMs:0.#}ms");
            }

            if (EnableLoss)
            {
                result = Append(result, EnableBurstLoss
                    ? $"loss {LossPercent:0.##}% bursty({BurstContinuationPercent:0.#}%)"
                    : $"loss {LossPercent:0.##}%");
            }

            if (EnableReorder)
            {
                result = Append(result, $"reorder {ReorderPercent:0.##}%@{ReorderDelayMs:0.#}ms");
            }

            return result;
        }

        /// <summary>
        /// Value equality over every field, so the controller can detect an Inspector edit without
        /// rebuilding transports every frame.
        /// </summary>
        public bool ValueEquals(NetworkImpairmentSettings other)
        {
            return other != null
                && EnableDelayTrace == other.EnableDelayTrace
                && TraceName == other.TraceName
                && EnableDelay == other.EnableDelay
                && EnableJitter == other.EnableJitter
                && EnableLoss == other.EnableLoss
                && EnableBurstLoss == other.EnableBurstLoss
                && EnableReorder == other.EnableReorder
                && Mathf.Approximately(BaseDelayMs, other.BaseDelayMs)
                && Mathf.Approximately(JitterMs, other.JitterMs)
                && Mathf.Approximately(LossPercent, other.LossPercent)
                && Mathf.Approximately(BurstContinuationPercent, other.BurstContinuationPercent)
                && Mathf.Approximately(ReorderPercent, other.ReorderPercent)
                && Mathf.Approximately(ReorderDelayMs, other.ReorderDelayMs);
        }

        /// <summary>
        /// True when every checkbox matches, ignoring the numeric fields — a toggle applies
        /// instantly while a dragged slider debounces.
        /// </summary>
        public bool EnabledStateEquals(NetworkImpairmentSettings other)
        {
            return other != null
                && EnableDelayTrace == other.EnableDelayTrace
                && EnableDelay == other.EnableDelay
                && EnableJitter == other.EnableJitter
                && EnableLoss == other.EnableLoss
                && EnableBurstLoss == other.EnableBurstLoss
                && EnableReorder == other.EnableReorder;
        }

        /// <summary>Field-by-field copy, avoiding an allocation per change check.</summary>
        public void CopyTo(NetworkImpairmentSettings destination)
        {
            destination.EnableDelayTrace = EnableDelayTrace;
            destination.TraceName = TraceName;
            destination.EnableDelay = EnableDelay;
            destination.BaseDelayMs = BaseDelayMs;
            destination.EnableJitter = EnableJitter;
            destination.JitterMs = JitterMs;
            destination.EnableLoss = EnableLoss;
            destination.LossPercent = LossPercent;
            destination.EnableBurstLoss = EnableBurstLoss;
            destination.BurstContinuationPercent = BurstContinuationPercent;
            destination.EnableReorder = EnableReorder;
            destination.ReorderPercent = ReorderPercent;
            destination.ReorderDelayMs = ReorderDelayMs;
        }

        /// <summary>Turns every axis off in one call -- the "back to a clean link" button.</summary>
        public void ClearAll()
        {
            EnableDelayTrace = false;
            EnableDelay = false;
            EnableJitter = false;
            EnableLoss = false;
            EnableBurstLoss = false;
            EnableReorder = false;
        }

        private static string Append(string existing, string fragment) =>
            existing.Length == 0 ? fragment : existing + " " + fragment;

        private static double Clamp01(double value)
        {
            if (value < 0.0)
            {
                return 0.0;
            }

            return value > 1.0 ? 1.0 : value;
        }

        /// <summary>Shared with <see cref="NetworkProfilePresets"/>, so both round identically.</summary>
        internal static long MsToTicks(float milliseconds, long ticksPerSecond)
        {
            if (milliseconds <= 0f)
            {
                return 0L;
            }

            return (long)(milliseconds / 1000.0 * ticksPerSecond);
        }

        /// <summary>Shared with <see cref="NetworkProfilePresets"/>, so both round identically.</summary>
        internal static float TicksToMs(long ticks, long ticksPerSecond) =>
            (float)(ticks * 1000.0 / ticksPerSecond);
    }
}
