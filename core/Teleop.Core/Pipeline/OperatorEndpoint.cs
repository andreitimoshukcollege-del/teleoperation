using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
using Teleop.Core.Transport;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Pipeline
{
    /// <summary>
    /// The operator side of the loopback: capture a pose, send it as a command, match the
    /// robot's eventual reply back to a <see cref="LatencyTrace"/>, and maintain a live estimate
    /// of the robot's current state via an injected predictor/reconciler pair. This is the
    /// composition layer the root <c>CLAUDE.md</c> describes as "the wiring diagram, expressed
    /// in code" — it holds no algorithm of its own, only the sequencing that ties
    /// <see cref="ICommandCodec"/>, <see cref="ITransport"/>, <see cref="ClockSync"/>,
    /// <see cref="IPredictor{TState}"/>, and <see cref="IReconciler{TState}"/> together.
    ///
    /// The predictor/reconciler are <b>required</b> constructor dependencies, never defaulted
    /// internally: this type "holds no algorithm of its own" is a real constraint, not a
    /// figure of speech, so the zero-mitigation configuration (a passthrough predictor plus a
    /// snap reconciler) must be visible at the call site, not hidden inside this class. Only
    /// operator-side prediction is wired here — estimating the robot's current state from stale
    /// downlink samples. Robot-side prediction (estimating operator intent from stale commands)
    /// is a different problem with different signal statistics per
    /// <see cref="IPredictor{TState}"/>'s own doc, and nothing here attempts it; see
    /// <see cref="RobotEndpoint"/>, which is unchanged.
    ///
    /// <see cref="ITimeAuthority"/> is used only for <c>TicksPerSecond</c> (a fixed conversion
    /// constant) — never <c>NowTicks</c>. Every "when is it now" arrives as an explicit
    /// <paramref name="nowTicks"/>-shaped parameter, the same "time is always a parameter"
    /// discipline <see cref="ITransport"/> and <c>IRobotPlant</c> already enforce. That rate is
    /// also what this endpoint hands <see cref="ClockSync"/> as the operator side of every
    /// round trip; the robot's own rate arrives on each <see cref="RobotStateFrame"/> rather
    /// than being assumed equal to ours
    /// (docs/adr/0008-clocksync-cross-rate-normalization.md).
    ///
    /// <b>Receive is two-phase</b>, because <see cref="IPlayoutPolicy{TState}"/> is the first thing
    /// in the pipeline that holds a sample across time
    /// (docs/adr/0012-playout-policy-wiring.md). <see cref="TryReceiveState"/> does the work that
    /// belongs to <i>arrival</i> — decode, <see cref="ClockSync"/>, <c>owd_uplink_ms</c> and
    /// <c>owd_downlink_ms</c> — and then hands the sample to the policy.
    /// <see cref="TryPlayoutState"/> drains the policy: it stamps <c>t_playout</c>, folds the
    /// released sample into the predictor and reconciler, and returns the completed trace. A host
    /// drains both, in that order, once per step; both replaced the single hardcoded
    /// <c>t_playout = t_operatorRecv</c> line this class used to carry as a stand-in for the
    /// missing axis.
    ///
    /// <b>Nothing reaches the predictor at arrival any more.</b> A host that drains
    /// <see cref="TryReceiveState"/> and forgets <see cref="TryPlayoutState"/> sees a frozen robot
    /// estimate and a silently empty Reconciliation axis — the same failure mode as forgetting
    /// <see cref="EstimateRobotState"/>, which is why both are asserted by
    /// <c>LoopbackPipelineIntegrationTests</c> rather than left to this comment.
    ///
    /// Allocation-free per call: the send/receive buffers and the in-flight trace ring are all
    /// preallocated in the constructor.
    /// </summary>
    public sealed class OperatorEndpoint
    {
        private readonly ICommandCodec _commandCodec;
        private readonly RobotStateFrameCodec _stateCodec;
        private readonly ITransport _uplinkTransport;
        private readonly ITransport _downlinkTransport;
        private readonly long _ticksPerSecond;
        private readonly IMetricSink _metrics;
        private readonly ClockSync _clockSync;
        private readonly IPredictor<Pose> _robotStatePredictor;
        private readonly IReconciler<Pose> _robotStateReconciler;
        private readonly IPlayoutPolicy<Pose> _playoutPolicy;

        /// <summary>
        /// Optional, and null in every configuration that does not ask for it. The one
        /// <c>docs/metrics.md</c> §3 vantage that exists nowhere else: reordering and RFC 3550
        /// interarrival jitter need a decoded frame's sequence number and its sender's raw send
        /// stamp for the <b>same</b> datagram, and <see cref="ITransport"/> deliberately exposes
        /// neither (<c>DatagramFate</c> documents the absence of a sequence field as intentional).
        /// The loss half of §3 does not come through here — it is observed at the transport
        /// boundary by <c>MeasuredTransport</c>, which needs no decode.
        /// </summary>
        private readonly NetworkObserver? _downlinkNetworkObserver;

        private readonly byte[] _sendBuffer;
        private readonly byte[] _recvBuffer;

        private readonly uint[] _inFlightSequences;
        private readonly LatencyTrace[] _inFlightTraces;
        private readonly bool[] _inFlightOccupied;
        private int _inFlightNextIndex;

        private uint _nextSequence;
        private uint _lastAckSequence;

        public OperatorEndpoint(
            ICommandCodec commandCodec,
            RobotStateFrameCodec stateCodec,
            ITransport uplinkTransport,
            ITransport downlinkTransport,
            ITimeAuthority operatorClock,
            IMetricSink metrics,
            ClockSync clockSync,
            IPredictor<Pose> robotStatePredictor,
            IReconciler<Pose> robotStateReconciler,
            IPlayoutPolicy<Pose> playoutPolicy,
            int inFlightCapacity,
            NetworkObserver? downlinkNetworkObserver = null)
        {
            if (commandCodec.MaxEncodedBytes > uplinkTransport.MaxPayloadBytes)
            {
                throw new ArgumentException(
                    "commandCodec.MaxEncodedBytes exceeds uplinkTransport.MaxPayloadBytes -- wiring error.");
            }

            _commandCodec = commandCodec;
            _stateCodec = stateCodec;
            _uplinkTransport = uplinkTransport;
            _downlinkTransport = downlinkTransport;
            _ticksPerSecond = operatorClock.TicksPerSecond;
            _metrics = metrics;
            _clockSync = clockSync;
            _robotStatePredictor = robotStatePredictor;
            _robotStateReconciler = robotStateReconciler;
            _playoutPolicy = playoutPolicy;
            _downlinkNetworkObserver = downlinkNetworkObserver;

            _sendBuffer = new byte[commandCodec.MaxEncodedBytes];
            _recvBuffer = new byte[downlinkTransport.MaxPayloadBytes];

            _inFlightSequences = new uint[inFlightCapacity];
            _inFlightTraces = new LatencyTrace[inFlightCapacity];
            _inFlightOccupied = new bool[inFlightCapacity];
        }

        /// <summary>
        /// Captures a pose and sends it as a command, in one atomic call -- matching
        /// docs/setup.md's callback-placement table, which groups "capture poses" and
        /// "SubmitCommand" into the same host callback. Assigns the next <see cref="CommandFrame.Sequence"/>,
        /// opens a <see cref="LatencyTrace"/> with <c>CaptureTicks</c> and <c>UplinkSendTicks</c>
        /// both equal to <paramref name="nowTicks"/>, and stores it in a fixed-capacity ring
        /// keyed by sequence for <see cref="TryReceiveState"/> to complete later. Allocation-free.
        /// </summary>
        public LatencyTrace SubmitCommand(Pose pose, Vector3 linearVelocity, Vector3 angularVelocity, float gripper, long nowTicks)
        {
            uint sequence = _nextSequence;
            _nextSequence = unchecked(_nextSequence + 1);

            var frame = new CommandFrame(sequence, _lastAckSequence, nowTicks, pose, linearVelocity, angularVelocity, gripper);

            if (_commandCodec.TryEncode(frame, _sendBuffer, out int bytesWritten))
            {
                _uplinkTransport.Send(_sendBuffer.AsSpan(0, bytesWritten), nowTicks);
            }

            LatencyTrace trace = LatencyTrace.ForSequence(sequence)
                .WithCaptureTicks(nowTicks)
                .WithUplinkSendTicks(nowTicks);

            InsertInFlight(sequence, trace);
            return trace;
        }

        /// <summary>
        /// Drains the downlink transport and, if a reply matching an in-flight
        /// <see cref="LatencyTrace"/> arrives, completes it: converts the robot's raw
        /// timestamps into operator domain via <see cref="ClockSync"/>, feeds the round trip
        /// back into <see cref="ClockSync"/> for the next estimate, records one-way-delay
        /// metrics, and folds the robot's reported state into the predictor/reconciler pair
        /// (see <see cref="EstimateRobotState"/>). A reply for an unknown or already-evicted
        /// sequence is an ordinary, silently skipped outcome -- the same "false is ordinary"
        /// spirit as the rest of <see cref="ITransport"/>. Returns false when nothing completed
        /// this call. Call in a loop until it returns false to drain a step. Allocation-free.
        /// </summary>
        public bool TryReceiveState(long nowTicks, out LatencyTrace completedTrace)
        {
            while (_downlinkTransport.TryReceive(nowTicks, _recvBuffer, out int byteCount, out long arrivalTicks))
            {
                if (!_stateCodec.TryDecode(_recvBuffer.AsSpan(0, byteCount), out RobotStateFrame stateFrame))
                {
                    continue;
                }

                // docs/metrics.md §3, and nothing else: purely additive, no control flow, no RNG,
                // no existing metric touched. Placed here, immediately after decode and before any
                // branch, so it observes every reply the link actually delivered -- including a
                // duplicate and including one whose in-flight trace has been evicted. Deliberately
                // NOT placed on the trace-completion path: InsertInFlight still overwrites an
                // occupied ring slot without checking, and that eviction is delay-correlated, so
                // any network statistic computed from completed traces would be censored hardest
                // exactly where the link is worst.
                //
                // Raw robot ticks and the robot's own rate go in, not the ClockSync-corrected
                // stamps computed below: RFC 3550's estimator differences out a clock offset by
                // construction, so feeding it a corrected stamp would report the movement of the
                // offset estimate as network jitter.
                if (_downlinkNetworkObserver != null)
                {
                    _downlinkNetworkObserver.OnSequencedArrival(
                        stateFrame.Sequence,
                        stateFrame.DownlinkSendTicks,
                        stateFrame.TicksPerSecond,
                        arrivalTicks);
                }

                // A reply with no matching in-flight trace still carries valid robot state, and it
                // must reach the predictor and reconciler. Only the *latency bookkeeping* needs the
                // trace: ClockSync.AddRoundTrip needs the uplink send stamp, and the OWD metrics are
                // computed from the trace's own timestamps. The pose does not -- its playout stamp
                // comes from stateFrame.DownlinkSendTicks, converted through ClockSync, and is
                // available whether or not the trace survived.
                //
                // Skipping the whole frame here was silently censoring the estimator's input, and
                // censoring it in the worst possible way: the in-flight ring evicts oldest-first, so
                // the frames dropped were exactly the most-delayed ones. On
                // 300ms-60j-2loss-bursty only 57.8% of round trips completed against ~4% profile
                // loss, and every prediction and reconciliation number recorded on the impaired
                // profiles was measured through that filter.
                long uplinkSendTicks = 0;
                bool hasTrace = TryPeekInFlight(stateFrame.Sequence, out int slot);
                if (hasTrace)
                {
                    // A trace that already carries t_operatorRecv belongs to a reply that has
                    // already been accounted for and is merely waiting to play out. A second reply
                    // for that sequence is a duplicate: dropping it here is what stops it feeding
                    // ClockSync twice and emitting a second pair of owd_* samples. Before the
                    // playout split this fell out for free, because the trace left the ring the
                    // instant its reply arrived.
                    if (_inFlightTraces[slot].TryGetOperatorRecvTicks(out _))
                    {
                        continue;
                    }

                    hasTrace = _inFlightTraces[slot].TryGetUplinkSendTicks(out uplinkSendTicks);
                }

                if (!hasTrace)
                {
                    // Converted here rather than once above the branch: ToOperatorTicks reads the
                    // current offset estimate, and the other branch deliberately converts *after*
                    // AddRoundTrip has folded this round trip in. Hoisting it would silently stamp
                    // every sample with the previous estimate.
                    _playoutPolicy.Enqueue(
                        stateFrame.Sequence,
                        new Stamped<Pose>(
                            _clockSync.ToOperatorTicks(
                                stateFrame.DownlinkSendTicks, stateFrame.TicksPerSecond, _ticksPerSecond),
                            stateFrame.Pose),
                        arrivalTicks);
                    continue;
                }

                // Both tick rates go in explicitly: the robot's arrives on the frame itself, since
                // its clock need not tick at the same rate as ours (10 MHz vs 1 GHz across a real
                // Windows/Linux pair) -- docs/adr/0008-clocksync-cross-rate-normalization.md.
                _clockSync.AddRoundTrip(
                    uplinkSendTicks, _ticksPerSecond,
                    stateFrame.RobotRecvTicks, stateFrame.DownlinkSendTicks, stateFrame.TicksPerSecond,
                    arrivalTicks);

                long robotRecvOperatorDomain = _clockSync.ToOperatorTicks(
                    stateFrame.RobotRecvTicks, stateFrame.TicksPerSecond, _ticksPerSecond);
                long downlinkSendOperatorDomain = _clockSync.ToOperatorTicks(
                    stateFrame.DownlinkSendTicks, stateFrame.TicksPerSecond, _ticksPerSecond);
                ClockSyncDiagnostics syncDiagnostics = _clockSync.Diagnostics;

                // Arrival-complete, not complete: t_playout is stamped by TryPlayoutState, whenever
                // the policy decides this sample is due. The trace stays in its ring slot until then
                // rather than moving to a second one -- the ring already overwrites oldest-first, and
                // a trace lost to that only costs the playout attribution for one sample. The state
                // itself is never censored by this structure, which is the invariant the in-flight
                // ring was fixed to restore.
                completedTrace = _inFlightTraces[slot]
                    .WithRobotRecvTicks(robotRecvOperatorDomain)
                    .WithDownlinkSendTicks(downlinkSendOperatorDomain)
                    .WithOperatorRecvTicks(arrivalTicks)
                    .WithClockSync(syncDiagnostics.OffsetTicks, syncDiagnostics.OffsetUncertaintyTicks);
                _inFlightTraces[slot] = completedTrace;

                _lastAckSequence = stateFrame.Sequence;

                // Emitted here, at arrival, and deliberately unchanged by the playout split: OWD is
                // an arrival-side quantity, so keeping it here is what makes the buffer's own cost a
                // separate, addable stage (playout_delay_ms) rather than a silent inflation of
                // one-way delay -- docs/metrics.md section 2's stage breakdown.
                RecordOneWayDelayMetrics(completedTrace, nowTicks);

                _playoutPolicy.Enqueue(
                    stateFrame.Sequence,
                    new Stamped<Pose>(downlinkSendOperatorDomain, stateFrame.Pose),
                    arrivalTicks);
                return true;
            }

            completedTrace = default;
            return false;
        }

        /// <summary>
        /// Drains the playout policy: releases every sample due at <paramref name="nowTicks"/>,
        /// stamps <c>t_playout</c> on its trace, and folds it into the predictor and reconciler.
        /// Returns false when nothing more is due -- the common case, not an error. Call in a loop
        /// until it returns false, after <see cref="TryReceiveState"/> and before
        /// <see cref="EstimateRobotState"/>. Allocation-free.
        ///
        /// <b>This is the call that feeds the estimator.</b> <see cref="TryReceiveState"/> no
        /// longer does; skipping this leaves the predictor with no observations at all.
        ///
        /// A released sample whose trace has been evicted from the in-flight ring still reaches the
        /// predictor and reconciler -- only the latency bookkeeping is lost -- and is skipped over
        /// rather than returned, the same way <see cref="TryReceiveState"/> handles a reply with no
        /// matching trace.
        /// </summary>
        public bool TryPlayoutState(long nowTicks, out LatencyTrace completedTrace)
        {
            while (_playoutPolicy.TryDequeue(nowTicks, out uint sequence, out Pose pose, out long playoutTicks))
            {
                // Order matters: the trace must be taken before ObserveRobotState, because taking it
                // frees the ring slot and Observe can run arbitrary predictor code. Reading the
                // capture stamp from the policy's own release rather than from the trace keeps the
                // two paths -- with and without a trace -- fed from one source.
                bool hasTrace = TryTakeInFlight(sequence, out LatencyTrace trace);

                long captureTicks = hasTrace && trace.TryGetDownlinkSendTicks(out long downlinkSend)
                    ? downlinkSend
                    : playoutTicks;

                ObserveRobotState(pose, captureTicks);

                if (!hasTrace)
                {
                    continue;
                }

                completedTrace = trace.WithPlayoutTicks(playoutTicks);
                return true;
            }

            completedTrace = default;
            return false;
        }

        /// <summary>
        /// The live estimate of the robot's current state: <c>Reconcile(Predict(nowTicks), nowTicks)</c>.
        /// Named for, and intended to be called from, docs/setup.md's <c>Application.onBeforeRender</c>
        /// callback slot ("<c>EstimateRobotState</c> → write Transforms") -- the last hook before
        /// rendering, so the estimate is as fresh as possible at the moment it's used.
        /// Allocation-free.
        /// </summary>
        public Pose EstimateRobotState(long nowTicks) =>
            _robotStateReconciler.Reconcile(_robotStatePredictor.Predict(nowTicks), nowTicks);

        /// <summary>
        /// Folds one robot-state sample into the predictor and reconciler, in the order that
        /// makes <see cref="IReconciler{TState}.Observe"/>'s <c>predictedAtCapture</c> parameter
        /// correct: the prediction for <paramref name="captureTicks"/> is read <b>before</b> the
        /// new sample is folded into the predictor, so it reflects what was actually displayed
        /// for that instant, not a prediction contaminated by the truth that just arrived.
        /// </summary>
        private void ObserveRobotState(Pose robotPose, long captureTicks)
        {
            Pose predictedAtCapture = _robotStatePredictor.Predict(captureTicks);
            var sample = new Stamped<Pose>(captureTicks, robotPose);

            _robotStatePredictor.Observe(sample);
            _robotStateReconciler.Observe(sample, predictedAtCapture, _robotStatePredictor.Diagnostics);
        }

        private void RecordOneWayDelayMetrics(in LatencyTrace trace, long nowTicks)
        {
            if (trace.TryGetUplinkSendTicks(out long uplinkSend) && trace.TryGetRobotRecvTicks(out long robotRecv))
            {
                _metrics.Record("owd_uplink_ms", TicksToMilliseconds(robotRecv - uplinkSend), nowTicks);
            }

            if (trace.TryGetDownlinkSendTicks(out long downlinkSend) && trace.TryGetOperatorRecvTicks(out long operatorRecv))
            {
                _metrics.Record("owd_downlink_ms", TicksToMilliseconds(operatorRecv - downlinkSend), nowTicks);
            }
        }

        private double TicksToMilliseconds(long ticks) => ticks * 1000.0 / _ticksPerSecond;

        private void InsertInFlight(uint sequence, LatencyTrace trace)
        {
            _inFlightSequences[_inFlightNextIndex] = sequence;
            _inFlightTraces[_inFlightNextIndex] = trace;
            _inFlightOccupied[_inFlightNextIndex] = true;
            _inFlightNextIndex = (_inFlightNextIndex + 1) % _inFlightSequences.Length;
        }

        /// <summary>
        /// Locates an occupied slot without freeing it, so <see cref="TryReceiveState"/> can write
        /// the arrival-completed trace back and leave it for <see cref="TryPlayoutState"/> to take
        /// when the sample actually plays.
        /// </summary>
        private bool TryPeekInFlight(uint sequence, out int slot)
        {
            for (int i = 0; i < _inFlightSequences.Length; i++)
            {
                if (_inFlightOccupied[i] && _inFlightSequences[i] == sequence)
                {
                    slot = i;
                    return true;
                }
            }

            slot = -1;
            return false;
        }

        private bool TryTakeInFlight(uint sequence, out LatencyTrace trace)
        {
            for (int i = 0; i < _inFlightSequences.Length; i++)
            {
                if (_inFlightOccupied[i] && _inFlightSequences[i] == sequence)
                {
                    trace = _inFlightTraces[i];
                    _inFlightOccupied[i] = false;
                    return true;
                }
            }

            trace = default;
            return false;
        }

        /// <summary>
        /// Returns the endpoint to its as-constructed state: no in-flight traces, sequence
        /// counters reset. Does not reset <see cref="ClockSync"/>, the transports, or the
        /// injected predictor/reconciler/playout policy -- those are injected dependencies with
        /// their own <c>Reset()</c>, called separately by whatever owns them. A caller that resets
        /// this endpoint without resetting the policy leaves samples buffered for sequence numbers
        /// the endpoint is about to reissue.
        /// </summary>
        public void Reset()
        {
            _nextSequence = 0;
            _lastAckSequence = 0;
            _inFlightNextIndex = 0;
            Array.Clear(_inFlightOccupied, 0, _inFlightOccupied.Length);
        }
    }
}
