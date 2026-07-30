using System;
using Teleop.Core.Contracts;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Metrics
{
    /// <summary>
    /// Retains recent samples in a fixed-capacity ring buffer so a test or an
    /// `algorithm-implementer` session can assert "this component emitted metric X ≈ Y" without
    /// wiring up a real CSV file. This is a test/inspection aid, not a full-run historian — it
    /// deliberately overwrites its oldest entry once full rather than growing, since recording an
    /// entire sweep's worth of metrics durably is <c>CsvMetricSink</c>'s job (a host-side class,
    /// since writing a file is I/O and I/O is not Core's, per <see cref="IMetricSink"/>'s own
    /// doc comment).
    ///
    /// Allocation-free after construction: the backing array is sized once, and <see cref="Record"/>
    /// never appends, resizes, or boxes — <paramref name="name"/> comparisons in
    /// <see cref="TryGetLatest"/> are ordinary <see cref="string"/> equality, which does not
    /// allocate.
    /// </summary>
    public sealed class InMemoryMetricTracker : IMetricSink
    {
        private readonly struct Entry
        {
            public readonly string Name;
            public readonly double Value;
            public readonly long Ticks;

            public Entry(string name, double value, long ticks)
            {
                Name = name;
                Value = value;
                Ticks = ticks;
            }
        }

        private readonly Entry[] _entries;
        private int _count;
        private int _nextWriteIndex;

        /// <summary>Preallocates room for <paramref name="capacity"/> samples.</summary>
        public InMemoryMetricTracker(int capacity)
        {
            _entries = new Entry[capacity];
            _count = 0;
            _nextWriteIndex = 0;
        }

        /// <summary>Samples currently retained, at most <see cref="Capacity"/>.</summary>
        public int Count => _count;

        /// <summary>Capacity given at construction.</summary>
        public int Capacity => _entries.Length;

        /// <summary>
        /// Records one sample, overwriting the oldest retained sample once <see cref="Capacity"/>
        /// is reached. Allocation-free.
        /// </summary>
        public void Record(string name, double value, long ticks)
        {
            _entries[_nextWriteIndex] = new Entry(name, value, ticks);
            _nextWriteIndex = (_nextWriteIndex + 1) % _entries.Length;
            if (_count < _entries.Length)
            {
                _count++;
            }
        }

        /// <summary>
        /// Sample at <paramref name="index"/>, oldest-retained-first (index 0 is the oldest
        /// sample still in the buffer, not necessarily the oldest ever recorded). Throws if
        /// <paramref name="index"/> is outside <c>[0, Count)</c>.
        /// </summary>
        public (string Name, double Value, long Ticks) this[int index]
        {
            get
            {
                if (index < 0 || index >= _count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }

                int oldestIndex = _count < _entries.Length ? 0 : _nextWriteIndex;
                int actualIndex = (oldestIndex + index) % _entries.Length;
                Entry e = _entries[actualIndex];
                return (e.Name, e.Value, e.Ticks);
            }
        }

        /// <summary>
        /// Most recently recorded sample named <paramref name="name"/>, searching newest-first.
        /// Returns false if no retained sample has that name — either none was ever recorded, or
        /// it has since been overwritten.
        /// </summary>
        public bool TryGetLatest(string name, out double value, out long ticks)
        {
            for (int i = 0; i < _count; i++)
            {
                int idx = (_nextWriteIndex - 1 - i + _entries.Length) % _entries.Length;
                if (_entries[idx].Name == name)
                {
                    value = _entries[idx].Value;
                    ticks = _entries[idx].Ticks;
                    return true;
                }
            }

            value = 0;
            ticks = 0;
            return false;
        }

        /// <summary>Returns the tracker to its as-constructed state: no samples retained.</summary>
        public void Reset()
        {
            _count = 0;
            _nextWriteIndex = 0;
        }
    }
}
