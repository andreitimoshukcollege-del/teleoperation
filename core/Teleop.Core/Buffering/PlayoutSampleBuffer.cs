using Teleop.Core.Types;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Buffering
{
    /// <summary>
    /// The mechanics every <c>IPlayoutPolicy</c> shares: hold samples, reject duplicates, release
    /// in capture-time order at a due instant the owning policy chooses, and count what that cost.
    /// A policy supplies the delay budget; this supplies everything else.
    ///
    /// It lives in one place for the same reason
    /// <see cref="Reconciliation.DisplayedJerkEstimator"/> does. Late-arrival rate and underrun
    /// count are the terms playout policies are <i>compared on</i>: a fixed budget's whole claim
    /// against an adaptive one is "same loss, more delay", so two implementations of "same loss"
    /// would agree in the common case and diverge exactly in the corners — a full buffer, a
    /// duplicate of an already-released sequence, a stream that stops — and the comparison would
    /// become one of buffer implementations as much as of policies.
    ///
    /// Does not implement <c>IPlayoutPolicy{TState}</c> and is therefore correctly invisible to
    /// <c>Teleop.Eval -- audit</c>'s registry-completeness check, which reflects for implementors
    /// of the contract interfaces.
    ///
    /// Deterministic and allocation-free: every array is sized once in the constructor. Not
    /// thread-safe, matching the policies that own one.
    /// </summary>
    /// <remarks>
    /// <b>Two definitions here are decisions, not details</b>, and both are argued in
    /// docs/adr/0012-playout-policy-wiring.md.
    ///
    /// <b>Late</b> is defined by order, not by arithmetic on the budget. The obvious rule — late
    /// iff <c>t_recv &gt; t_capture + budget</c> — scores <c>immediate</c> at 100% late, because
    /// its budget is zero and arrival is always after capture. The rule used instead is: a sample
    /// is late iff it can no longer be released in capture-time order, i.e. its capture stamp is
    /// not newer than the last released sample's. On a dense stream the two coincide exactly (a
    /// sample whose delay exceeds the budget lands after the buffer has already released something
    /// newer), so this is the same latency/loss curve <c>analysis/playout_bounds.py</c> measures —
    /// stated in a way that also holds at zero budget and on a sparse stream.
    ///
    /// <b>Underrun</b> is a drain that released nothing <i>and</i> had nothing to release — not a
    /// <c>TryDequeue</c> that returned false. False is how a drain loop terminates;
    /// <c>IPlayoutPolicy.TryDequeue</c>'s own doc calls it "the common case and not an error".
    /// Each half of the rule rules out one wrong answer: counting an empty buffer alone would score
    /// <c>immediate</c> at 100%, since a zero buffer is empty after every release; counting "gave
    /// nothing this drain" alone would score <c>fixed</c> at 100%, since holding a sample that is
    /// not yet due is exactly its designed steady state. Together they count the case the contract
    /// actually names: the pipeline asked, and there was nothing to have.
    /// </remarks>
    internal sealed class PlayoutSampleBuffer
    {
        private readonly int _capacity;

        private readonly uint[] _sequences;
        private readonly Stamped<Pose>[] _samples;
        private readonly long[] _dueTicks;
        private readonly long[] _arrivalTicks;
        private readonly bool[] _occupied;

        /// <summary>
        /// Sequences already released, so a duplicate arriving after its original played out is
        /// rejected rather than replayed — <c>IPlayoutPolicy</c> clause 2 forbids releasing the
        /// same sequence twice, and checking the live buffer alone cannot catch that case. Sized
        /// with the buffer: remembering exactly as far back as the buffer can hold is the same
        /// bound <see cref="PlayoutPolicyConfig.HistoryCapacity"/> already places on how far
        /// out-of-order a sample may arrive and still be reinserted.
        /// </summary>
        private readonly uint[] _releasedSequences;
        private readonly bool[] _releasedOccupied;
        private int _releasedNextIndex;

        private long _lastReleasedCaptureTicks;
        private bool _hasReleased;

        /// <summary>
        /// Whether anything has been released since the last <see cref="TryRelease"/> that found
        /// nothing. Drives the underrun rule in the type remarks: one underrun per drain that gave
        /// the pipeline nothing, not one per false return.
        /// </summary>
        private bool _releasedSinceLastMiss;

        private int _bufferedCount;
        private int _enqueuedCount;
        private int _lateCount;
        private int _underrunCount;
        private int _duplicatesRejected;

        internal PlayoutSampleBuffer(int capacity)
        {
            _capacity = capacity;

            _sequences = new uint[capacity];
            _samples = new Stamped<Pose>[capacity];
            _dueTicks = new long[capacity];
            _arrivalTicks = new long[capacity];
            _occupied = new bool[capacity];

            _releasedSequences = new uint[capacity];
            _releasedOccupied = new bool[capacity];
        }

        internal int BufferedCount => _bufferedCount;

        internal float OccupancyFraction => (float)_bufferedCount / _capacity;

        /// <summary>
        /// Fraction of enqueued samples discarded as too late to play in order. Zero before the
        /// first <see cref="Enqueue"/> rather than <c>NaN</c>: a policy that has seen nothing has
        /// induced no loss, and a rate that starts at <c>NaN</c> poisons every aggregate computed
        /// over a trial's first steps.
        /// </summary>
        internal double LateArrivalRate => _enqueuedCount == 0 ? 0.0 : (double)_lateCount / _enqueuedCount;

        internal int UnderrunCount => _underrunCount;

        internal int DuplicatesRejected => _duplicatesRejected;

        /// <summary>
        /// Offers a sample for playout at <paramref name="dueTicks"/>. Returns
        /// <see cref="PlayoutAdmission"/> so the owning policy can emit the right metric without
        /// re-deriving why the sample was refused; the counters behind
        /// <see cref="LateArrivalRate"/> and <see cref="DuplicatesRejected"/> are updated here
        /// either way. Allocation-free.
        /// </summary>
        internal PlayoutAdmission Enqueue(uint sequence, Stamped<Pose> sample, long arrivalTicks, long dueTicks)
        {
            if (IsBuffered(sequence) || WasReleased(sequence))
            {
                _duplicatesRejected++;
                return PlayoutAdmission.Duplicate;
            }

            _enqueuedCount++;

            // Out of order past the point of no return: something at or after this capture instant
            // has already been played, so releasing this would move the displayed state backwards.
            if (_hasReleased && sample.CaptureTicks <= _lastReleasedCaptureTicks)
            {
                _lateCount++;
                return PlayoutAdmission.Late;
            }

            int slot = FindFreeSlot();
            if (slot < 0)
            {
                // The buffer is the bound on how far out of order a sample may arrive and still be
                // reinserted (PlayoutPolicyConfig.HistoryCapacity). Past it, the sample is late for
                // the same reason and is counted the same way -- a silently dropped sample here
                // would make a too-small capacity look like a well-behaved policy.
                _lateCount++;
                return PlayoutAdmission.Late;
            }

            _sequences[slot] = sequence;
            _samples[slot] = sample;
            _dueTicks[slot] = dueTicks;
            _arrivalTicks[slot] = arrivalTicks;
            _occupied[slot] = true;
            _bufferedCount++;

            return PlayoutAdmission.Buffered;
        }

        /// <summary>
        /// Releases the buffered sample with the earliest capture stamp among those due at or
        /// before <paramref name="nowTicks"/>. Ordering is by capture time rather than by due time
        /// so that a policy which lowers its budget mid-stream cannot invert two samples it has
        /// already accepted. Returns false when nothing is due — see the type remarks for why that
        /// is not by itself an underrun. Allocation-free.
        /// </summary>
        internal bool TryRelease(
            long nowTicks, out uint sequence, out Stamped<Pose> sample, out long playoutTicks, out long arrivalTicks)
        {
            int best = -1;
            for (int i = 0; i < _capacity; i++)
            {
                if (!_occupied[i] || _dueTicks[i] > nowTicks)
                {
                    continue;
                }

                if (best < 0 || _samples[i].CaptureTicks < _samples[best].CaptureTicks)
                {
                    best = i;
                }
            }

            if (best < 0)
            {
                // Both conditions, and the second is the one that took a failing test to find:
                // a drain that released nothing but is still *holding* something has not starved,
                // it is doing its job. Requiring only the first counted `fixed`'s designed steady
                // state as an underrun on the step after every release.
                if (!_releasedSinceLastMiss && _bufferedCount == 0)
                {
                    _underrunCount++;
                }

                _releasedSinceLastMiss = false;
                sequence = 0;
                sample = default;
                playoutTicks = 0;
                arrivalTicks = 0;
                return false;
            }

            sequence = _sequences[best];
            sample = _samples[best];
            arrivalTicks = _arrivalTicks[best];

            // t_playout is the instant the policy *scheduled*, not nowTicks: reporting the poll
            // time would fold host frame time into every downstream playout figure, which is the
            // failure mode IPlayoutPolicy.TryDequeue's own doc calls out. A sample that was already
            // due when it was enqueued is scheduled for its arrival instant, never earlier -- a
            // playout stamp before the sample existed locally would make t_playout - t_recv
            // negative.
            long scheduled = _dueTicks[best];
            playoutTicks = scheduled < arrivalTicks ? arrivalTicks : scheduled;

            _occupied[best] = false;
            _bufferedCount--;

            _lastReleasedCaptureTicks = sample.CaptureTicks;
            _hasReleased = true;
            _releasedSinceLastMiss = true;
            RecordReleased(sequence);

            return true;
        }

        /// <summary>
        /// Back to as-constructed: buffer empty, release history forgotten, every counter zeroed.
        /// The arrays themselves survive, since sweeps reuse policy instances across trials.
        /// </summary>
        internal void Reset()
        {
            for (int i = 0; i < _capacity; i++)
            {
                _occupied[i] = false;
                _releasedOccupied[i] = false;
            }

            _releasedNextIndex = 0;
            _lastReleasedCaptureTicks = 0;
            _hasReleased = false;
            _releasedSinceLastMiss = false;
            _bufferedCount = 0;
            _enqueuedCount = 0;
            _lateCount = 0;
            _underrunCount = 0;
            _duplicatesRejected = 0;
        }

        private bool IsBuffered(uint sequence)
        {
            for (int i = 0; i < _capacity; i++)
            {
                if (_occupied[i] && _sequences[i] == sequence)
                {
                    return true;
                }
            }

            return false;
        }

        private bool WasReleased(uint sequence)
        {
            for (int i = 0; i < _capacity; i++)
            {
                if (_releasedOccupied[i] && _releasedSequences[i] == sequence)
                {
                    return true;
                }
            }

            return false;
        }

        private void RecordReleased(uint sequence)
        {
            _releasedSequences[_releasedNextIndex] = sequence;
            _releasedOccupied[_releasedNextIndex] = true;
            _releasedNextIndex = (_releasedNextIndex + 1) % _capacity;
        }

        private int FindFreeSlot()
        {
            for (int i = 0; i < _capacity; i++)
            {
                if (!_occupied[i])
                {
                    return i;
                }
            }

            return -1;
        }
    }

    /// <summary>
    /// What <see cref="PlayoutSampleBuffer.Enqueue"/> did with a sample. Returned so the owning
    /// policy emits the metric matching the outcome without re-deriving the decision — the
    /// alternative, comparing counters before and after, is how two policies end up disagreeing
    /// about their own late rate.
    /// </summary>
    internal enum PlayoutAdmission
    {
        /// <summary>Accepted; it will be released when due.</summary>
        Buffered,

        /// <summary>
        /// Too late to play in capture order, or the buffer was full. Counted in
        /// <see cref="PlayoutPolicyDiagnostics.LateArrivalRate"/>; emits <c>playout_late</c>.
        /// </summary>
        Late,

        /// <summary>
        /// Already buffered or already released. Counted in
        /// <see cref="PlayoutPolicyDiagnostics.DuplicatesRejected"/>; emits nothing, since a
        /// duplicate is a transport property rather than an operating point (docs/metrics.md §3
        /// owns reordering and duplication, and is uninstrumented).
        /// </summary>
        Duplicate,
    }
}
