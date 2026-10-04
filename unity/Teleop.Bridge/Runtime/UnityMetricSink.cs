using System;
using System.IO;
using System.Text;
using Teleop.Core.Contracts;
using Teleop.Core.Metrics;
using Teleop.Core.Recording;
using Teleop.Core.Types;

namespace Teleop.Bridge
{
    /// <summary>
    /// The host-side <see cref="IMetricSink"/> for a Play-mode session. Two jobs, matching
    /// Teleop.Eval's split of <c>CsvMetricSink</c> (named samples) and
    /// <c>Recording/TlogFileWriter</c> (the session recording) -- combined into one class here
    /// because Bridge's file budget is small (Teleop/CLAUDE.md: "roughly a dozen files") and this
    /// phase only ever runs one recording at a time.
    ///
    /// <b>Named samples</b> (<c>owd_uplink_ms</c>, <c>owd_downlink_ms</c>, <c>m2p_ms</c>) go into
    /// an <see cref="InMemoryMetricTracker"/> -- a Core class, no I/O -- so <c>LatencyHud</c> can
    /// read the latest value every frame via <see cref="TryGetLatest"/>.
    ///
    /// <b>The <c>.tlog</c> recording</b> is Gate 4's "recorded session" (docs/setup.md): every
    /// completed, render/photon-stamped <see cref="LatencyTrace"/> is written as one line, via
    /// <see cref="Teleop.Core.Recording.SessionWriter"/> -- the actual <c>FileStream</c> handle
    /// lives here because Core's <c>Recording/</c> is forbidden from touching one.
    /// </summary>
    public sealed class UnityMetricSink : IMetricSink, IDisposable
    {
        private readonly InMemoryMetricTracker _hud;
        private readonly StreamWriter _stream;
        private readonly SessionWriter _sessionWriter;
        private readonly byte[] _lineBuffer;

        public UnityMetricSink(int hudCapacity, string tlogPath, long ticksPerSecond, ulong sessionId)
        {
            _hud = new InMemoryMetricTracker(hudCapacity);
            _stream = new StreamWriter(tlogPath, append: false, Encoding.ASCII) { NewLine = "\n" };
            _sessionWriter = new SessionWriter();
            _lineBuffer = new byte[RecordFormat.MaxLineBytes];

            WriteLine(_sessionWriter.TryWriteHeader(ticksPerSecond, sessionId, _lineBuffer, out int n), n, "header");
        }

        /// <inheritdoc/>
        public void Record(string name, double value, long ticks) => _hud.Record(name, value, ticks);

        /// <summary>Most recent sample named <paramref name="name"/>. See <see cref="InMemoryMetricTracker.TryGetLatest"/>.</summary>
        public bool TryGetLatest(string name, out double value, out long ticks) =>
            _hud.TryGetLatest(name, out value, out ticks);

        /// <summary>Appends one fully-stamped <see cref="LatencyTrace"/> line to the session recording.</summary>
        public void WriteLatencyTrace(in LatencyTrace trace) =>
            WriteLine(_sessionWriter.TryWriteLatencyTrace(trace, _lineBuffer, out int n), n, "LatencyTrace");

        /// <summary>Closes the recording with the checksum trailer and flushes the file.</summary>
        public void Dispose()
        {
            WriteLine(_sessionWriter.TryWriteEndOfSession(_lineBuffer, out int n), n, "EndOfSession");
            _stream.Flush();
            _stream.Dispose();
        }

        private void WriteLine(bool ok, int length, string what)
        {
            if (!ok)
            {
                throw new InvalidOperationException($"{what} line exceeded RecordFormat.MaxLineBytes");
            }

            _stream.WriteLine(Encoding.ASCII.GetString(_lineBuffer, 0, length));
        }
    }
}
