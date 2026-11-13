using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading;

namespace Teleop.RobotHost.Recording
{
    /// <summary>
    /// Completes a closed automatic session (docs/adr/0023 decision 4) as <c>just record-stop</c>
    /// completes a manual one: waits for the driver's <c>END</c> trailer (with a timeout), stamps
    /// <c>stoppedUtc</c>, sets the slider from the speeds the joint commands carried, notes an uplink
    /// that changed during the session, and writes <c>files.sha256</c>. On its own thread, so hashing a
    /// 200 MB log never holds up the writer of the next session.
    /// </summary>
    public sealed class SessionFinalizer : IDisposable
    {
        private readonly BlockingCollection<string> _jobs = new BlockingCollection<string>();
        private readonly double _driverWaitSeconds;
        private readonly IReadOnlyList<WriterWait> _writers;
        private readonly Func<DateTime> _utcNow;
        private readonly Action<string> _log;
        private readonly Thread _thread;
        private int _pending;

        public SessionFinalizer(double driverWaitSeconds, Func<DateTime>? utcNow = null, Action<string>? log = null,
            IReadOnlyList<WriterWait>? writers = null)
        {
            _driverWaitSeconds = driverWaitSeconds;
            _writers = writers?.ToArray() ?? Array.Empty<WriterWait>();
            _utcNow = utcNow ?? (() => DateTime.UtcNow);
            _log = log ?? Console.WriteLine;
            _thread = new Thread(Run) { IsBackground = true, Name = "session-finalizer" };
            _thread.Start();
        }

        /// <summary>Sessions queued or being completed.</summary>
        public int Pending => Volatile.Read(ref _pending);

        public void Enqueue(string directory)
        {
            Interlocked.Increment(ref _pending);
            _jobs.Add(directory);
        }

        /// <summary>Finishes what is queued (up to <paramref name="timeout"/>), then stops.</summary>
        public void Dispose(TimeSpan timeout)
        {
            _jobs.CompleteAdding();
            _thread.Join(timeout);
        }

        public void Dispose() => Dispose(TimeSpan.FromSeconds(_driverWaitSeconds + _writers.Sum(w => w.Seconds) + 30));

        private void Run()
        {
            foreach (string directory in _jobs.GetConsumingEnumerable())
            {
                try
                {
                    Complete(directory, _driverWaitSeconds, _utcNow, _log, _writers);
                }
                catch (Exception e)
                {
                    // Disk full, a file removed underneath: the session stays as it is, without files.sha256.
                    _log($"[record] could not complete {directory}: {e.GetType().Name}: {e.Message}");
                }
                finally
                {
                    Interlocked.Decrement(ref _pending);
                }
            }
        }

        private static readonly JsonSerializerOptions Indented = new JsonSerializerOptions
        {
            WriteIndented = true,
            Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
        };

        /// <summary>Writes <paramref name="node"/> as indented JSON through a temporary file, so a reader never sees half of it.</summary>
        public static void WriteJsonAtomically(string path, JsonNode node)
        {
            string temp = path + ".tmp";
            File.WriteAllText(temp, node.ToJsonString(Indented) + "\n", new UTF8Encoding(false));
            File.Move(temp, path, overwrite: true);
        }

        /// <summary>Completes one session directory; see the class summary. Synchronous (tests call it directly).</summary>
        public static void Complete(string directory, double driverWaitSeconds, Func<DateTime> utcNow, Action<string> log,
            IReadOnlyList<WriterWait>? writers = null)
        {
            writers ??= Array.Empty<WriterWait>();
            bool driverClosed = WaitForTrailer(directory, "driver", driverWaitSeconds);
            var writerNotes = new List<string>();
            foreach (WriterWait w in writers)
            {
                bool closed = WaitForTrailer(directory, w.Name, w.Seconds);
                List<string> logs = LogFiles(directory, w.Name).ToList();
                if (logs.Count == 0) writerNotes.Add($"{w.Name}.jlog is missing: that writer recorded nothing");
                else if (!closed) writerNotes.Add(FormattableString.Invariant($"{w.Name}.jlog had no END trailer {w.Seconds} s after RobotHost closed the session"));
                else writerNotes.AddRange(CheckFileLines(directory, logs));
            }

            string metaPath = Path.Combine(directory, "session.json");
            JsonNode meta = JsonNode.Parse(File.ReadAllText(metaPath)) ?? throw new InvalidDataException("session.json is empty");
            meta["stoppedUtc"] = utcNow().ToString("yyyy-MM-dd'T'HH:mm:ss'Z'", CultureInfo.InvariantCulture);

            var speeds = new SortedSet<double>();
            var uplinks = new SortedSet<string>(StringComparer.Ordinal);
            bool headsetDisturbed = false;
            foreach (string log1 in LogFiles(directory, "robothost"))
            {
                headsetDisturbed |= ScanRobotHostLog(log1, speeds, uplinks);
            }

            var notes = new List<string>();
            JsonNode conditions = meta["conditions"] ?? (meta["conditions"] = new JsonObject());
            // The slider is what the commands carried (motor 10, the gripper, aside; 0 means none was
            // sent), read as just record-stop and robot/pilot/onboard.py read it.
            if (speeds.Count == 1) conditions["speedSliderRadPerSec"] = speeds.Min;
            else if (speeds.Count > 1) conditions["speedSliderRadPerSec"] = "varied";

            // RobotHost's own uplink impairment: one profile all session long, or it changed (UP).
            JsonNode? network = conditions["network"];
            string opened = network?["uplinkProfile"]?.GetValue<string>() ?? "lan";
            if (opened != "schedule" && network != null)
            {
                string openedName = opened == "lan" ? "none" : opened;
                if (uplinks.Any(u => u != openedName))
                {
                    network["uplinkProfile"] = "varied";
                    notes.Add("the robot-side uplink changed during the session (" + string.Join(", ", new[] { openedName }.Concat(uplinks).Distinct()) + "); its UP records say when");
                }
            }

            // The headset's own disturbance or added delay (HD records, docs/adr/0023 amendment).
            if (network != null)
            {
                network["headsetDisturbed"] = headsetDisturbed;
                if (headsetDisturbed) notes.Add("the headset's network disturbance or added delay was on during the session (HD records): not a real-network measurement");
            }

            // A wait of 0 with no driver log is a host without a driver (the serial-chain hosts): nothing was missing.
            bool driverExpected = driverWaitSeconds > 0 || LogFiles(directory, "driver").Any();
            if (!driverClosed && driverExpected)
            {
                notes.Add(FormattableString.Invariant($"driver.jlog had no END trailer {driverWaitSeconds} s after RobotHost closed the session"));
                log($"[record] {Path.GetFileName(directory)}: the driver's log was not closed within {driverWaitSeconds} s; files.sha256 written anyway");
            }

            notes.AddRange(writerNotes);
            foreach (string note in writerNotes) log($"[record] {Path.GetFileName(directory)}: {note}");

            if (notes.Count > 0)
            {
                string old = meta["notes"]?.GetValue<string>() ?? "";
                meta["notes"] = (old.Length > 0 ? old + "; " : "") + string.Join("; ", notes);
            }

            WriteJsonAtomically(metaPath, meta);
            WriteChecksums(directory, writers.Select(w => w.Name));
            log($"[record] completed {Path.GetFileName(directory)}: slider {conditions["speedSliderRadPerSec"]?.ToJsonString()}, files.sha256 written");
        }

        /// <summary>
        /// <c>sha256sum</c> format, as record-stop writes it: session.json, robot-profile.json, scene.json, every <c>.jlog</c>, then
        /// every file under each writer's folder (<c>isaac/head.rgb.frames</c>, ...), so <c>sha256sum -c</c> checks them all.
        /// </summary>
        public static void WriteChecksums(string directory, IEnumerable<string>? writerFolders = null)
        {
            var names = new List<string> { "session.json" };
            if (File.Exists(Path.Combine(directory, "robot-profile.json"))) names.Add("robot-profile.json");
            if (File.Exists(Path.Combine(directory, "scene.json"))) names.Add("scene.json");
            names.AddRange(Directory.GetFiles(directory, "*.jlog").Select(Path.GetFileName).Select(n => n!).OrderBy(n => n, StringComparer.Ordinal));
            foreach (string folder in writerFolders ?? Array.Empty<string>())
            {
                string root = Path.Combine(directory, folder);
                if (!Directory.Exists(root)) continue;
                names.AddRange(Directory.GetFiles(root, "*", SearchOption.AllDirectories)
                    .Select(p => Path.GetRelativePath(directory, p).Replace('\\', '/'))
                    .Where(n => !n.EndsWith(".part", StringComparison.Ordinal))
                    .OrderBy(n => n, StringComparer.Ordinal));
            }

            var text = new StringBuilder();
            foreach (string name in names)
            {
                using FileStream stream = File.OpenRead(Path.Combine(directory, name));
                text.Append(Convert.ToHexString(SHA256.HashData(stream)).ToLowerInvariant()).Append("  ").Append(name).Append('\n');
            }

            string path = Path.Combine(directory, "files.sha256");
            File.WriteAllText(path + ".tmp", text.ToString(), new UTF8Encoding(false));
            File.Move(path + ".tmp", path, overwrite: true);
        }

        /// <summary><c>&lt;writer&gt;.jlog</c> and the segments a restart started (<c>&lt;writer&gt;.2.jlog</c>, ...).</summary>
        private static IEnumerable<string> LogFiles(string directory, string writer)
        {
            if (!Directory.Exists(directory)) return Array.Empty<string>();
            return Directory.GetFiles(directory, writer + "*.jlog").OrderBy(p => p, StringComparer.Ordinal);
        }

        /// <summary>
        /// Every <c>FS|name|bytes|sha256</c> line of a writer's logs against the file it names: a problem for each that is missing,
        /// another size, or another hash. A name must stay inside the session.
        /// </summary>
        internal static IEnumerable<string> CheckFileLines(string directory, IEnumerable<string> logs)
        {
            string root = Path.GetFullPath(directory) + Path.DirectorySeparatorChar;
            foreach (string log in logs)
            {
                foreach (string line in File.ReadLines(log, Encoding.ASCII))
                {
                    if (!line.StartsWith("FS|", StringComparison.Ordinal)) continue;
                    string[] f = line.Split('|');
                    if (f.Length != 4 || !long.TryParse(f[2], NumberStyles.None, CultureInfo.InvariantCulture, out long bytes))
                    {
                        yield return $"{Path.GetFileName(log)} has a malformed FS line: {line}";
                        continue;
                    }

                    string path = Path.GetFullPath(Path.Combine(directory, f[1]));
                    if (!path.StartsWith(root, StringComparison.Ordinal))
                    {
                        yield return $"{Path.GetFileName(log)} names a file outside the session: {f[1]}";
                        continue;
                    }

                    if (!File.Exists(path))
                    {
                        yield return $"{f[1]} is missing (named by {Path.GetFileName(log)})";
                        continue;
                    }

                    long size = new FileInfo(path).Length;
                    string sha;
                    using (FileStream stream = File.OpenRead(path)) sha = Convert.ToHexString(SHA256.HashData(stream)).ToLowerInvariant();
                    if (size != bytes || sha != f[3]) yield return $"{f[1]} is not what {Path.GetFileName(log)} says ({size} bytes, sha256 {sha[..12]})";
                }
            }
        }

        private static bool WaitForTrailer(string directory, string writer, double seconds)
        {
            DateTime deadline = DateTime.UtcNow + TimeSpan.FromSeconds(seconds);
            while (true)
            {
                var logs = LogFiles(directory, writer).ToList();
                if (logs.Count > 0 && logs.All(p => LastLine(p).StartsWith("END|", StringComparison.Ordinal))) return true;
                if (DateTime.UtcNow >= deadline) return false;
                Thread.Sleep(100);
            }
        }

        /// <summary>The last non-empty line of a file, from its last 300 bytes.</summary>
        public static string LastLine(string path)
        {
            try
            {
                using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
                long start = Math.Max(0, stream.Length - 300);
                stream.Seek(start, SeekOrigin.Begin);
                var buffer = new byte[stream.Length - start];
                int read = 0;
                while (read < buffer.Length)
                {
                    int n = stream.Read(buffer, read, buffer.Length - read);
                    if (n == 0) break;
                    read += n;
                }

                string[] lines = Encoding.ASCII.GetString(buffer, 0, read).TrimEnd('\n').Split('\n');
                return lines.Length > 0 ? lines[lines.Length - 1] : "";
            }
            catch (IOException)
            {
                return "";
            }
            catch (UnauthorizedAccessException)
            {
                return "";
            }
        }

        /// <summary>Collects the speeds and uplinks; true when an HD record had anything on.</summary>
        private static bool ScanRobotHostLog(string path, SortedSet<double> speeds, SortedSet<string> uplinks)
        {
            bool disturbed = false;
            using var reader = new StreamReader(new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite), Encoding.ASCII);
            string? line;
            while ((line = reader.ReadLine()) != null)
            {
                if (line.StartsWith("JC|", StringComparison.Ordinal))
                {
                    string[] fields = line.Split('|');
                    if (fields.Length < 7 || fields[6].Length == 0) continue;
                    foreach (string target in fields[6].Split(';'))
                    {
                        string[] parts = target.Split(':');
                        if (parts.Length != 3 || parts[0] == "10") continue;
                        if (double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double speed) && speed > 0)
                        {
                            speeds.Add(Math.Round(speed, 2));
                        }
                    }
                }
                else if (line.StartsWith("UP|", StringComparison.Ordinal))
                {
                    string[] fields = line.Split('|');
                    if (fields.Length >= 3) uplinks.Add(fields[2]);
                }
                else if (line.StartsWith("HD|", StringComparison.Ordinal))
                {
                    string[] fields = line.Split('|');
                    for (int i = 2; i <= 5 && i < fields.Length; i++)
                    {
                        // NaN (a named profile) and any positive value are on.
                        if (!double.TryParse(fields[i], NumberStyles.Float, CultureInfo.InvariantCulture, out double v) || double.IsNaN(v) || v > 0)
                        {
                            disturbed = true;
                        }
                    }
                }
            }

            return disturbed;
        }
    }
}
