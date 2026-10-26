using System;
using System.Collections.Generic;
using System.Linq;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>
    /// The rules of recording whenever the robot is commanded (docs/adr/0023 decision 4), on an
    /// injected clock, disk and writer: when a session opens, what closes it, and when the next one
    /// opens. <see cref="SessionRecorderAutoTests"/> covers the files the real writer makes.
    /// </summary>
    public class AutoSessionRecorderTests
    {
        private const long Tps = 1_000_000_000;
        private const long GB = 1024L * 1024 * 1024;

        private sealed class FakeSink : IAutoSessionSink
        {
            private int _next = 1;
            public int Open;
            public bool ControlFileBusy { get; set; }
            public readonly List<AutoSessionSpec> Begun = new List<AutoSessionSpec>();
            public readonly List<string> Ended = new List<string>();

            public int BeginAuto(AutoSessionSpec spec)
            {
                Assert.Equal(0, Open); // never two at once
                if (ControlFileBusy) return 0;
                Begun.Add(spec);
                Open = _next++;
                return Open;
            }

            public bool IsSessionActive(int generation) => generation != 0 && generation == Open;

            public void EndAuto(int generation, string reason)
            {
                Assert.Equal(Open, generation);
                Ended.Add(reason);
                Open = 0;
            }

            /// <summary>What the writer does on its own: a manual session took over, or a write failed.</summary>
            public void EndedByWriter() => Open = 0;
        }

        private sealed class Rig
        {
            public readonly FakeSink Sink = new FakeSink();
            public long? Free = 10 * GB;
            public int DiskReads;
            public readonly List<string> Log = new List<string>();
            public UplinkCondition Uplink = UplinkCondition.None;
            public readonly AutoSessionRecorder Auto;
            public DateTime Utc = new DateTime(2026, 10, 26, 12, 0, 0, DateTimeKind.Utc);

            public Rig(AutoRecordOptions? options = null)
            {
                Auto = new AutoSessionRecorder(options ?? new AutoRecordOptions(), Sink, Tps, () => Utc,
                    () => { DiskReads++; return Free; }, () => Uplink, Log.Add);
            }

            /// <summary>Commands from <paramref name="sender"/> every 20 ms over [from, to) seconds, ticking as the main loop does.</summary>
            public void Command(double from, double to, string sender = "10.0.0.5:6002")
            {
                for (double t = from; t < to; t += 0.02)
                {
                    Auto.Tick(T(t));
                    Auto.OnJointCommand(T(t), sender);
                }
            }

            public void Idle(double from, double to)
            {
                for (double t = from; t < to; t += 0.1) Auto.Tick(T(t));
            }
        }

        private static long T(double seconds) => (long)Math.Round(seconds * Tps);

        [Fact]
        public void TheFirstCommand_OpensASession_WithWhatIsKnown()
        {
            var rig = new Rig();
            rig.Uplink = new UplinkCondition("50ms-5j", 7);
            rig.Auto.OnJointCommand(T(1), "10.0.0.5:6002");
            Assert.True(rig.Auto.IsOpen);
            AutoSessionSpec spec = Assert.Single(rig.Sink.Begun);
            Assert.Equal("10.0.0.5:6002", spec.Sender);
            Assert.Equal("50ms-5j", spec.Uplink.Profile);
            Assert.Equal(7UL, spec.Uplink.Seed);
            Assert.Equal(rig.Utc, spec.StartedUtc);
            rig.Command(1.02, 20);
            Assert.Single(rig.Sink.Begun); // one session while the commands keep coming
        }

        [Fact]
        public void ItClosesAfterTheIdleTime_AndOpensAgainOnlyAfterAnIdleGap()
        {
            var rig = new Rig();
            rig.Command(0, 10);
            rig.Idle(10, 39.9);
            Assert.True(rig.Auto.IsOpen); // 29.9 s without a command
            rig.Idle(39.9, 40.2);
            Assert.False(rig.Auto.IsOpen);
            Assert.Contains("no joint command for 30 s", rig.Sink.Ended.Single());
            rig.Command(41, 45);
            Assert.Equal(2, rig.Sink.Begun.Count);
            Assert.StartsWith("first joint command after 30 s", rig.Sink.Begun[1].Reason);
        }

        [Fact]
        public void ACommandAfterAShortGap_DoesNotOpen_WhenTheSessionWasEndedElsewhere()
        {
            var rig = new Rig();
            rig.Command(0, 5);
            rig.Sink.EndedByWriter(); // e.g. just record-start took over, or the disk refused a write
            rig.Command(5, 20);
            Assert.False(rig.Auto.IsOpen);
            Assert.Single(rig.Sink.Begun);
            rig.Idle(20, 51);
            rig.Command(51, 52);
            Assert.True(rig.Auto.IsOpen);
            Assert.Equal(2, rig.Sink.Begun.Count);
        }

        [Fact]
        public void ANewSender_ClosesTheSession_AndOpensANewOne_OnItsFirstCommand()
        {
            var rig = new Rig();
            rig.Command(0, 10, "10.0.0.5:6002");
            rig.Command(10, 11, "10.0.0.5:40000"); // the app restarted on a new port
            Assert.Equal(2, rig.Sink.Begun.Count);
            Assert.Contains("sender changed from 10.0.0.5:6002 to 10.0.0.5:40000", rig.Sink.Ended.Single());
            Assert.Equal("10.0.0.5:40000", rig.Sink.Begun[1].Sender);
            Assert.True(rig.Auto.IsOpen);
        }

        [Fact]
        public void ANewSubscriber_ClosesTheSession_AndTheNextCommandOpensOne()
        {
            var rig = new Rig();
            rig.Auto.OnSubscriber(T(0), "10.0.0.5:6002");
            rig.Command(0, 10);
            rig.Auto.OnSubscriber(T(10), "10.0.0.5:6002"); // the same one: nothing
            Assert.Empty(rig.Sink.Ended);
            rig.Auto.OnSubscriber(T(10.01), "10.0.0.9:51000"); // joint-probe, say
            Assert.False(rig.Auto.IsOpen);
            Assert.Contains("subscriber changed from 10.0.0.5:6002 to 10.0.0.9:51000", rig.Sink.Ended.Single());
            rig.Command(10.02, 11);
            Assert.Equal(2, rig.Sink.Begun.Count);
        }

        [Fact]
        public void ALapsedSubscriberComingBack_DoesNotSplit()
        {
            var rig = new Rig();
            rig.Auto.OnSubscriber(T(0), "10.0.0.5:6002");
            rig.Command(0, 10);
            rig.Auto.OnSubscriber(T(10), null);
            rig.Auto.OnSubscriber(T(12), "10.0.0.5:6002");
            Assert.Empty(rig.Sink.Ended);
        }

        [Fact]
        public void TheFirstSubscriberOfASessionThatHadNone_DoesNotSplit()
        {
            var rig = new Rig();
            rig.Command(0, 1);
            rig.Auto.OnSubscriber(T(1), "10.0.0.5:6002");
            Assert.True(rig.Auto.IsOpen);
            Assert.Empty(rig.Sink.Ended);
        }

        [Fact]
        public void TwoEndpointsTakingTurns_SplitOnce_ThenNotAgainWithinTheMinimum()
        {
            var rig = new Rig();
            rig.Auto.OnSubscriber(T(0), "a:1");
            rig.Command(0, 10);
            for (int i = 0; i < 100; i++)
            {
                double t = 10 + i * 0.03;
                rig.Auto.OnSubscriber(T(t), i % 2 == 0 ? "b:2" : "a:1");
                rig.Auto.OnJointCommand(T(t + 0.01), "10.0.0.5:6002");
            }

            Assert.Single(rig.Sink.Ended);
            Assert.Equal(2, rig.Sink.Begun.Count);
            Assert.Contains(rig.Log, l => l.Contains("not splitting"));
            // Once the minimum has passed, a change splits again.
            rig.Auto.OnSubscriber(T(20), "c:3");
            Assert.Equal(2, rig.Sink.Ended.Count);
        }

        [Fact]
        public void AfterTheMaximumLength_ItRotates_AndTheNextCommandOpensTheNextSession()
        {
            var rig = new Rig(new AutoRecordOptions { MaxMinutes = 1 });
            rig.Command(0, 59.9);
            Assert.Single(rig.Sink.Begun);
            rig.Command(59.9, 60.5);
            Assert.Equal(2, rig.Sink.Begun.Count);
            Assert.Contains("rotation after 60 s", rig.Sink.Ended.Single());
            Assert.Contains("rotation", rig.Sink.Begun[1].Reason);
        }

        [Fact]
        public void BelowTheStartThreshold_NothingOpens_AndItSaysWhy_ThenItOpensOnceThereIsRoom()
        {
            var rig = new Rig();
            rig.Free = (long)(1.5 * GB);
            rig.Command(0, 5);
            Assert.False(rig.Auto.IsOpen);
            Assert.Single(rig.Log, l => l.Contains("not opening an automatic session: only 1.50 GB free, below 2 GB"));
            Assert.True(rig.DiskReads <= 4); // read every 2 s, not on every command
            rig.Free = 3 * GB; // fetched and pruned while commanding
            rig.Command(5, 8);
            Assert.True(rig.Auto.IsOpen);
            Assert.Equal("free space recovered", rig.Sink.Begun.Single().Reason);
        }

        [Fact]
        public void BelowTheStopThreshold_AnOpenSessionCloses()
        {
            var rig = new Rig();
            rig.Command(0, 5);
            Assert.True(rig.Auto.IsOpen);
            rig.Free = (long)(0.9 * GB);
            rig.Command(5, 8);
            Assert.False(rig.Auto.IsOpen);
            Assert.Contains("only 0.90 GB free, below 1 GB", rig.Sink.Ended.Single());
            rig.Free = (long)(1.5 * GB); // above the stop threshold, still below the start one
            rig.Command(8, 12);
            Assert.False(rig.Auto.IsOpen);
        }

        [Fact]
        public void AnUnreadableDisk_DoesNotStopRecording()
        {
            var rig = new Rig { Free = null };
            rig.Command(0, 5);
            Assert.True(rig.Auto.IsOpen);
        }

        [Fact]
        public void WhileAManualSessionRecords_TheAutomaticRecorderStaysOut()
        {
            var rig = new Rig();
            rig.Sink.ControlFileBusy = true;
            rig.Command(0, 10);
            Assert.Empty(rig.Sink.Begun);
            Assert.Single(rig.Log, l => l.Contains("manual session is recording"));
            // The manual session ends while commands keep coming: no hand-over until an idle gap.
            rig.Sink.ControlFileBusy = false;
            rig.Command(10, 20);
            Assert.Empty(rig.Sink.Begun);
            rig.Idle(20, 50.5);
            rig.Command(50.5, 51);
            Assert.Single(rig.Sink.Begun);
        }

        [Fact]
        public void EveryThreshold_IsTheOptionsOne()
        {
            var rig = new Rig(new AutoRecordOptions { IdleSeconds = 5, MinSplitSeconds = 0, StartFreeGigabytes = 4, StopFreeGigabytes = 3 });
            rig.Command(0, 1);
            rig.Idle(1, 6.2);
            Assert.False(rig.Auto.IsOpen);
            rig.Command(6.2, 7); // 5.2 s later: a gap
            Assert.Equal(2, rig.Sink.Begun.Count);
            rig.Free = (long)(3.5 * GB);
            rig.Idle(7, 9.5);
            Assert.True(rig.Auto.IsOpen); // above the stop threshold
            rig.Idle(15, 16); // idle-closes
            rig.Command(30, 31);
            Assert.Equal(2, rig.Sink.Begun.Count); // 3.5 GB is under the start threshold of 4
        }

        [Fact]
        public void InvalidOptions_AreRefused()
        {
            Assert.Throws<ArgumentException>(() => new Rig(new AutoRecordOptions { IdleSeconds = 0 }));
            Assert.Throws<ArgumentException>(() => new Rig(new AutoRecordOptions { StartFreeGigabytes = 1, StopFreeGigabytes = 2 }));
        }
    }
}
