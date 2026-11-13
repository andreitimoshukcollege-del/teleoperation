using System;
using System.IO;
using Teleop.RobotHost.Recording;

namespace Teleop.RobotHost.Tests.Recording
{
    /// <summary>What both hosts start the same way (HostRecording): the RECORD button's folder and its disk thresholds.</summary>
    public sealed class HostRecordingTests
    {
        private const long Tps = 1_000_000_000;

        [Fact]
        public void Takes_StartOnlyWithARecorderAndAFolder()
        {
            Assert.Null(HostRecording.StartScenarios(null, null, null, null, null, Tps, () => new UplinkCondition("none", 0), _ => { }));
            Assert.Null(HostRecording.StartAuto(null, null, Tps, () => new UplinkCondition("none", 0), _ => { }));
        }

        [Fact]
        public void Takes_GetTheGivenThresholds_OrKeepTheDefaults()
        {
            string root = Path.Combine(Path.GetTempPath(), "host-recording-" + Guid.NewGuid().ToString("N"));
            try
            {
                Directory.CreateDirectory(root);
                using var recorder = new SessionRecorder(Path.Combine(root, ".active"), Tps, pollSeconds: 0.02);
                var log = new System.Collections.Generic.List<string>();
                ScenarioRecorder? jetson = HostRecording.StartScenarios(Path.Combine(root, "scenarios"), 8, 4, recorder, null, Tps,
                    () => new UplinkCondition("none", 0), log.Add);
                Assert.NotNull(jetson);
                Assert.Equal((8.0, 4.0), (jetson!.StartFreeGigabytes, jetson.StopFreeGigabytes));
                Assert.True(Directory.Exists(Path.Combine(root, "scenarios")));
                Assert.Contains(log, l => l.Contains("starts with 8 GB free and stops below 4 GB", StringComparison.Ordinal));

                ScenarioRecorder? gpu = HostRecording.StartScenarios(Path.Combine(root, "scenarios2"), null, null, recorder, null, Tps,
                    () => new UplinkCondition("none", 0), _ => { });
                var defaults = new ScenarioRecorder(Path.Combine(root, "scenarios3"), recorder, null, new ScenarioFolder(Path.Combine(root, "scenarios3")), Tps,
                    () => DateTime.UtcNow, () => null, () => new UplinkCondition("none", 0));
                Assert.Equal((defaults.StartFreeGigabytes, defaults.StopFreeGigabytes), (gpu!.StartFreeGigabytes, gpu.StopFreeGigabytes));
            }
            finally
            {
                try { Directory.Delete(root, recursive: true); } catch (IOException) { }
            }
        }
    }
}
