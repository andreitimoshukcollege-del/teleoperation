using System;
using System.Globalization;
using System.IO;
using System.Text;
using Teleop.Core.Contracts;

namespace Teleop.Eval.Metrics
{
    /// <summary>
    /// Writes every recorded sample to a <c>metrics.csv</c>. Lives here, not in Core, because
    /// writing a file is I/O and I/O is not Core's -- per <c>IMetricSink</c>'s own doc comment.
    /// Core's in-process equivalents are <c>NullMetricSink</c> and <c>InMemoryMetricTracker</c>.
    /// </summary>
    public sealed class CsvMetricSink : IMetricSink, IDisposable
    {
        private readonly StreamWriter _writer;

        public CsvMetricSink(string path)
        {
            _writer = new StreamWriter(path, append: false, Encoding.ASCII) { NewLine = "\n" };
            _writer.WriteLine("name,value,ticks");
        }

        public void Record(string name, double value, long ticks)
        {
            _writer.WriteLine($"{name},{value.ToString(CultureInfo.InvariantCulture)},{ticks}");
        }

        public void Dispose()
        {
            _writer.Flush();
            _writer.Dispose();
        }
    }
}
