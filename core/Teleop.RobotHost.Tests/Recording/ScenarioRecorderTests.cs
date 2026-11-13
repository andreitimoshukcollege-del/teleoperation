using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json.Nodes;
using System.Threading;
using Teleop.RobotArm.Wire;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// Takes of named scenarios from the headset's RECORD button (docs/adr/0033): the list file's rules, when a take starts
    /// and stops, the automatic recorder held off meanwhile, discarding, and the files the real writer makes for a take.
    /// </summary>
    public class ScenarioRecorderTests
    {
        private const long Tps = 1_000_000_000;
        private const long GB = 1024L * 1024 * 1024;
        private const string Headset = "10.0.0.7:6012";

        private sealed class FakeSink : IAutoSessionSink
        {
            private int _next = 1;
            public int Open;
            public bool ControlFileBusy { get; set; }
            public readonly List<AutoSessionSpec> Begun = new List<AutoSessionSpec>();
            public readonly List<string> Ended = new List<string>();

            public int BeginAuto(AutoSessionSpec spec)
            {
                Assert.Equal(0, Open);
                if (ControlFileBusy) return 0;
                Begun.Add(spec);
                Open = _next++;
                return Open;
            }

            public bool IsSessionActive(int generation) => generation != 0 && generation == Open;

            public void EndAuto(int generation, string reason)
            {
                if (generation != Open) return;
                Ended.Add(reason);
                Open = 0;
            }
        }

        private sealed class FakeFiles : IScenarioFiles
        {
            public string? List = "free-play | anything\nknock-over | knock each object over\n";
            public DateTime? Stamp = new DateTime(2026, 11, 6, 9, 0, 0, DateTimeKind.Utc);
            public readonly Dictionary<string, (int, int)> TakeCounts = new Dictionary<string, (int, int)>();
            public readonly HashSet<string> Complete = new HashSet<string>();
            public readonly List<(string, string)> Moved = new List<(string, string)>();
            public int Reads;

            public string? ReadList()
            {
                Reads++;
                return List;
            }

            public DateTime? ListStamp() => Stamp;
            public (int Kept, int Highest) Takes(string scenario) => TakeCounts.TryGetValue(scenario, out var t) ? t : (0, 0);
            public bool IsComplete(string directory) => Complete.Contains(directory);

            public bool MoveAside(string directory, string aside)
            {
                Moved.Add((directory, aside));
                return true;
            }
        }

        private sealed class Rig
        {
            public readonly FakeSink Sink = new FakeSink();
            public readonly FakeFiles Files = new FakeFiles();
            public readonly AutoSessionRecorder Auto;
            public readonly ScenarioRecorder Scenarios;
            public readonly List<string> Log = new List<string>();
            public long? Free = 400 * GB;
            public uint Sequence;

            public Rig()
            {
                var utc = new DateTime(2026, 11, 6, 15, 30, 0, DateTimeKind.Utc);
                Auto = new AutoSessionRecorder(new AutoRecordOptions(), Sink, Tps, () => utc, () => Free, () => UplinkCondition.None, Log.Add);
                Scenarios = new ScenarioRecorder("/data/scenarios", Sink, Auto, Files, Tps, () => utc, () => Free, () => UplinkCondition.None, Log.Add);
                Scenarios.Tick(0);
            }

            public RecordOutcome Ask(RecordOp op, int scenario, long now, ushort? revision = null)
            {
                Scenarios.HandleRequest(op, (byte)scenario, revision ?? Scenarios.ListRevision, ++Sequence, Headset, now);
                return Scenarios.Status(now).Outcome;
            }
        }

        private static long T(double s) => (long)(s * Tps);

        [Fact]
        public void TheListFile_SkipsWhatBreaksARule_AndIsNeverEmpty()
        {
            var problems = new List<string>();
            IReadOnlyList<Scenario> list = ScenarioList.Parse("# a comment\n\npush-can | push the can\nbad name\npush-can\n_hidden\nstack # trailing comment\n", problems);
            Assert.Equal(new[] { "push-can", "stack" }, list.Select(s => s.Name));
            Assert.Equal("push the can", list[0].Description);
            Assert.Equal(3, problems.Count);
            Assert.Equal(new[] { ScenarioList.DefaultName }, ScenarioList.Parse(null, problems).Select(s => s.Name));
            Assert.Equal(16, ScenarioList.Parse(string.Join("\n", Enumerable.Range(0, 20).Select(i => $"s{i}")), problems).Count);
            Assert.Equal(new[] { "free-play", "knock-over", "push-to-edge" }, ScenarioList.Parse(ScenarioList.Template, new List<string>()).Select(s => s.Name));
        }

        [Fact]
        public void ATake_OpensInItsScenariosFolder_HoldsOffTheAutomaticRecorder_AndStopsWhenAsked()
        {
            var rig = new Rig();
            rig.Files.TakeCounts["knock-over"] = (3, 5);
            rig.Auto.OnHeadset(0, Headset);
            rig.Auto.OnJointCommand(0, Headset); // an automatic session is open
            Assert.True(rig.Auto.IsOpen);

            Assert.Equal(RecordOutcome.Started, rig.Ask(RecordOp.Start, 1, T(1)));
            Assert.False(rig.Auto.IsOpen); // closed for the take
            Assert.True(rig.Scenarios.IsRecording);
            ScenarioTake take = rig.Sink.Begun.Last().Take!;
            Assert.Equal(("knock-over", 6), (take.Scenario, take.Take));
            Assert.Equal(Path.Combine(Path.GetFullPath("/data/scenarios"), "knock-over", "20261106T153000Z-take006"), take.Directory);
            RecordStatus s = rig.Scenarios.Status(T(31));
            Assert.Equal((RecordState.Recording, (byte)1, (ushort)6, 30_000u), (s.State, s.Scenario, s.Take, s.ElapsedMs));

            // Commands go on: the automatic recorder opens nothing meanwhile.
            for (double t = 1; t < 40; t += 0.5)
            {
                rig.Auto.OnHeadset(T(t), Headset);
                rig.Auto.OnJointCommand(T(t), Headset);
                rig.Scenarios.OnHeard(Headset, T(t));
                rig.Scenarios.Tick(T(t));
            }

            Assert.Equal(2, rig.Sink.Begun.Count); // the automatic one, then the take

            Assert.Equal(RecordOutcome.Busy, rig.Ask(RecordOp.Start, 0, T(40)));
            Assert.Equal(RecordOutcome.Saved, rig.Ask(RecordOp.Stop, 0, T(41)));
            Assert.False(rig.Scenarios.IsRecording);
            Assert.False(rig.Auto.Held);
            Assert.Equal(4, rig.Scenarios.Status(T(41)).TakesKept);
            Assert.Equal(RecordOutcome.Busy, rig.Ask(RecordOp.Stop, 0, T(42))); // nothing to stop
        }

        [Fact]
        public void ARequest_ResentUntilEchoed_IsAnsweredOnce()
        {
            var rig = new Rig();
            rig.Scenarios.HandleRequest(RecordOp.Start, 0, rig.Scenarios.ListRevision, 5, Headset, T(1));
            rig.Scenarios.HandleRequest(RecordOp.Start, 0, rig.Scenarios.ListRevision, 5, Headset, T(1.25));
            rig.Scenarios.HandleRequest(RecordOp.Start, 0, rig.Scenarios.ListRevision, 5, Headset, T(1.5));
            Assert.Single(rig.Sink.Begun);
            Assert.Equal((5u, RecordOutcome.Started), (rig.Scenarios.Status(T(2)).Sequence, rig.Scenarios.Status(T(2)).Outcome));
        }

        [Fact]
        public void ARestartedRobotHost_ListIsTakenAsNew_NotTheHeadsetsStaleOne()
        {
            // Every process started at revision 0, so its first list was revision 1, the number the headset held for the
            // previous process's list: the headset kept the old names and called them current, and a take could go to the
            // wrong scenario (the 2026-11 bug hunt, E-02).
            var utc = new DateTime(2026, 11, 13, 9, 0, 0, DateTimeKind.Utc);
            ScenarioRecorder Host(string list)
            {
                var files = new FakeFiles { List = list };
                var host = new ScenarioRecorder("/data/scenarios", new FakeSink(), null, files, Tps, () => utc, () => 400 * GB, () => UplinkCondition.None);
                host.Tick(0);
                return host;
            }

            var headset = new Teleop.RobotArm.Operator.RecordingState();
            void Hear(ScenarioRecorder host, long now) => host.Send(now, (bytes, n) =>
            {
                var datagram = new ReadOnlySpan<byte>(bytes, 0, n);
                if (RecordCodec.TryDecodeStatus(datagram, out RecordStatus status, out long ticks)) headset.ApplyStatus(status, ticks);
                else headset.ApplyListPart(datagram);
            });

            Hear(Host("pick-can\nstack\n"), T(1));
            Assert.Equal("pick-can", headset.Name(0));

            // The restarted host reads a different list.
            Hear(Host("other\npick-can\nstack\n"), T(2));
            Assert.True(headset.ListCurrent);
            Assert.Equal((3, "other"), (headset.Count, headset.Name(0)));
        }

        [Fact]
        public void EachProcess_StartsItsListRevisionsSomewhereNew()
        {
            ushort FirstListRevision(ushort? first)
            {
                var host = new ScenarioRecorder("/data/scenarios", new FakeSink(), null, new FakeFiles(), Tps, () => DateTime.UtcNow,
                    () => 400 * GB, () => UplinkCondition.None, firstRevision: first);
                host.Tick(0);
                return host.ListRevision;
            }

            Assert.True(Enumerable.Range(0, 20).Select(_ => FirstListRevision(null)).Distinct().Count() > 1, "twenty processes all started at the same revision");
            Assert.Equal(1, FirstListRevision(0)); // the old start, still available to a test that needs it fixed
        }

        [Fact]
        public void AnIndexIntoAnOlderList_OrPastTheEnd_IsRefused()
        {
            var rig = new Rig();
            ushort old = rig.Scenarios.ListRevision;
            rig.Files.List = "other\nknock-over\n";
            rig.Files.Stamp = rig.Files.Stamp!.Value.AddMinutes(1);
            rig.Scenarios.Tick(T(3));
            Assert.NotEqual(old, rig.Scenarios.ListRevision);
            Assert.Equal(RecordOutcome.UnknownScenario, rig.Ask(RecordOp.Start, 0, T(4), old));
            Assert.Equal(RecordOutcome.UnknownScenario, rig.Ask(RecordOp.Start, 2, T(4)));
            Assert.Equal(RecordOutcome.Started, rig.Ask(RecordOp.Start, 0, T(4)));
            Assert.Equal("other", rig.Sink.Begun.Single().Take!.Scenario);
        }

        [Fact]
        public void TheListFile_IsReadOnlyWhenItChanges()
        {
            var rig = new Rig();
            int reads = rig.Files.Reads;
            for (double t = 0; t < 30; t += 0.001 * 50) rig.Scenarios.Tick(T(t));
            Assert.Equal(reads, rig.Files.Reads);
        }

        [Fact]
        public void ATake_StopsByItself_WhenTheHeadsetGoesQuiet_OrTheDiskFills_OrItRunsTooLong()
        {
            var rig = new Rig();
            rig.Ask(RecordOp.Start, 0, T(1));
            rig.Scenarios.Tick(T(10.9));
            Assert.True(rig.Scenarios.IsRecording);
            rig.Scenarios.Tick(T(11.1));
            Assert.Equal(RecordOutcome.StoppedByItself, rig.Scenarios.Status(T(12)).Outcome);

            rig.Ask(RecordOp.Start, 0, T(20));
            rig.Free = 20 * GB;
            rig.Scenarios.OnHeard(Headset, T(23));
            rig.Scenarios.Tick(T(23));
            Assert.Equal(RecordOutcome.LowDisk, rig.Scenarios.Status(T(23)).Outcome);
            Assert.Equal(RecordOutcome.LowDisk, rig.Ask(RecordOp.Start, 0, T(30))); // and does not start below 25 GB

            rig.Free = 400 * GB;
            rig.Scenarios.MaxMinutes = 1;
            rig.Ask(RecordOp.Start, 0, T(40));
            for (double t = 40; t < 102; t += 1)
            {
                rig.Scenarios.OnHeard(Headset, T(t));
                rig.Scenarios.Tick(T(t));
            }

            Assert.False(rig.Scenarios.IsRecording);
            Assert.Contains(rig.Sink.Ended, e => e.Contains("1 min"));
        }

        [Fact]
        public void DiscardLast_SetsTheLastTakeAside_OnceComplete_AndOnlyOnce()
        {
            var rig = new Rig();
            Assert.Equal(RecordOutcome.NothingToDiscard, rig.Ask(RecordOp.DiscardLast, 0, T(1)));
            rig.Ask(RecordOp.Start, 1, T(2));
            Assert.Equal(RecordOutcome.Busy, rig.Ask(RecordOp.DiscardLast, 0, T(3))); // while recording
            rig.Ask(RecordOp.Stop, 0, T(4));
            string dir = rig.Scenarios.LastTake!;
            Assert.Equal(RecordOutcome.Busy, rig.Ask(RecordOp.DiscardLast, 0, T(5))); // not yet completed
            rig.Files.Complete.Add(dir);
            Assert.Equal(RecordOutcome.Discarded, rig.Ask(RecordOp.DiscardLast, 0, T(6)));
            (string from, string to) = rig.Files.Moved.Single();
            Assert.Equal(dir, from);
            Assert.Equal(Path.Combine(Path.GetFullPath("/data/scenarios"), "_discarded", "knock-over", Path.GetFileName(dir)), to);
            Assert.Equal(RecordOutcome.NothingToDiscard, rig.Ask(RecordOp.DiscardLast, 0, T(7)));
        }

        [Fact]
        public void TheHeadsetHears_TheStatusAndTheList()
        {
            var rig = new Rig();
            var sent = new List<byte[]>();
            rig.Scenarios.Send(T(1), (b, n) => sent.Add(b.AsSpan(0, n).ToArray()));
            rig.Scenarios.Send(T(1.5), (b, n) => sent.Add(b.AsSpan(0, n).ToArray())); // nothing new
            Assert.Equal(2, sent.Count);
            var state = new Teleop.RobotArm.Operator.RecordingState();
            Assert.True(RecordCodec.TryDecodeStatus(sent[0], out RecordStatus status, out _));
            state.ApplyStatus(status, 0);
            Assert.True(state.ApplyListPart(sent[1]));
            Assert.True(state.ListCurrent);
            Assert.Equal(new[] { "free-play", "knock-over" }, new[] { state.Name(0), state.Name(1) });
        }

        [Fact]
        public void TheRealWriter_PutsATakeInItsFolder_WithTheScenarioInSessionJson()
        {
            string root = Path.Combine(Path.GetTempPath(), "teleop-scenario-" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(root);
            try
            {
                var options = new AutoRecordOptions { DriverWaitSeconds = 0, SessionFormat = "teleop-session", SessionVersion = 2, Source = "isaac", RecordingsRoot = Path.Combine(root, "auto") };
                using var recorder = new SessionRecorder(Path.Combine(root, ".active"), Tps, SessionRecorder.DefaultCapacity, 0.02, options, () => DateTime.UtcNow, _ => { });
                var folder = new ScenarioFolder(Path.Combine(root, "scenarios"));
                folder.EnsureExists();
                long free = 400 * GB;
                var scenarios = new ScenarioRecorder(Path.Combine(root, "scenarios"), recorder, null, folder, Tps, () => new DateTime(2026, 11, 6, 15, 30, 0, DateTimeKind.Utc),
                    () => free, () => UplinkCondition.None);
                scenarios.Tick(0);
                scenarios.HandleRequest(RecordOp.Start, 1, scenarios.ListRevision, 1, Headset, T(1));
                Assert.True(scenarios.IsRecording);
                recorder.Record("QS|1|2|1:0.5");
                Thread.Sleep(200);
                scenarios.HandleRequest(RecordOp.Stop, 0, scenarios.ListRevision, 2, Headset, T(2));
                string dir = Path.Combine(root, "scenarios", "knock-over", "20261106T153000Z-take001");
                var sw = Stopwatch.StartNew();
                while (!File.Exists(Path.Combine(dir, "files.sha256")) && sw.Elapsed.TotalSeconds < 10) Thread.Sleep(20);
                Assert.True(File.Exists(Path.Combine(dir, "files.sha256")), "the take was not completed");
                var meta = JsonNode.Parse(File.ReadAllText(Path.Combine(dir, "session.json")))!;
                Assert.Equal("scenario-knock-over", (string?)meta["label"]);
                Assert.Equal("scenario:knock-over", (string?)meta["conditions"]!["motion"]);
                Assert.Equal("knock-over", (string?)meta["scenario"]!["name"]);
                Assert.Equal(1, (int)meta["scenario"]!["take"]!);
                Assert.Equal("20261106T153000Z-take001", (string?)meta["id"]);
                Assert.Contains("QS|1|2|1:0.5", File.ReadAllText(Path.Combine(dir, "robothost.jlog")));
                Assert.Equal((1, 1), folder.Takes("knock-over"));

                // Discarded, it moves aside; the next take is still number 2.
                scenarios.HandleRequest(RecordOp.DiscardLast, 0, scenarios.ListRevision, 3, Headset, T(3));
                Assert.Equal(RecordOutcome.Discarded, scenarios.Status(T(3)).Outcome);
                Assert.True(Directory.Exists(Path.Combine(root, "scenarios", "_discarded", "knock-over", "20261106T153000Z-take001")));
                Assert.Equal((0, 1), folder.Takes("knock-over"));
            }
            finally
            {
                try { Directory.Delete(root, true); } catch (IOException) { }
            }
        }
    }
}
