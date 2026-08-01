using System;
using System.Numerics;
using Teleop.Core.Contracts;
using Teleop.Core.Time;
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
    /// discipline <see cref="ITransport"/> and <c>IRobotPlant</c> already enforce.
    ///
    /// <c>t_playout</c> is set equal to <c>t_operatorRecv</c> in <see cref="TryReceiveState"/> —
    /// the explicit stand-in for the not-yet-built <c>ImmediatePlayout</c>
    /// (<c>Buffering/CLAUDE.md</c>'s "zero buffer" baseline). This is a deliberately temporary
    /// shortcut: once <c>IPlayoutPolicy</c> has an implementation, this inline assignment is
    /// replaced by a real call to it.
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
            int inFlightCapacity)
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

                if (!TryTakeInFlight(stateFrame.Sequence, out LatencyTrace trace))
                {
                    continue;
                }

                if (!trace.TryGetUplinkSendTicks(out long uplinkSendTicks))
                {
                    continue;
                }

                _clockSync.AddRoundTrip(uplinkSendTicks, stateFrame.RobotRecvTicks, stateFrame.DownlinkSendTicks, arrivalTicks);

                long robotRecvOperatorDomain = _clockSync.ToOperatorTicks(stateFrame.RobotRecvTicks);
                long downlinkSendOperatorDomain = _clockSync.ToOperatorTicks(stateFrame.DownlinkSendTicks);
                ClockSyncDiagnostics syncDiagnostics = _clockSync.Diagnostics;

                completedTrace = trace
                    .WithRobotRecvTicks(robotRecvOperatorDomain)
                    .WithDownlinkSendTicks(downlinkSendOperatorDomain)
                    .WithOperatorRecvTicks(arrivalTicks)
                    .WithPlayoutTicks(arrivalTicks) // Phase-4 stand-in for IPlayoutPolicy -- see type doc.
                    .WithClockSync(syncDiagnostics.OffsetTicks, syncDiagnostics.OffsetUncertaintyTicks);

                _lastAckSequence = stateFrame.Sequence;

                RecordOneWayDelayMetrics(completedTrace, nowTicks);
                ObserveRobotState(stateFrame.Pose, downlinkSendOperatorDomain);
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
        /// injected predictor/reconciler -- those are injected dependencies with their own
        /// <c>Reset()</c>, called separately by whatever owns them.
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
