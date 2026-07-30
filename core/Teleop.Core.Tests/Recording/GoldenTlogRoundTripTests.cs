using System.IO;
using Teleop.Core.Recording;

namespace Teleop.Core.Tests.Recording;

/// <summary>
/// Reads the committed golden fixture directly (does not share code with
/// <c>Teleop.Eval.Tooling.GoldenSessionBuilder</c> that generated it -- Tests never depends on
/// Eval in this project). Complements <c>Teleop.Eval -- verify</c>'s own byte-identical replay
/// check with assertions on the fixture's actual decoded content, so a change that breaks
/// decoding semantics (not just byte-identity) is caught here too.
/// </summary>
public class GoldenTlogRoundTripTests
{
    private static string GoldenPath => Path.Combine(AppContext.BaseDirectory, "testdata", "golden", "basic-session.tlog");

    [Fact]
    public void GoldenFixture_IsPresentInTestOutput()
    {
        Assert.True(File.Exists(GoldenPath), $"Golden fixture not found at {GoldenPath}");
    }

    [Fact]
    public void GoldenFixture_HeaderDecodes_AndReportsSupportedVersion()
    {
        string[] lines = File.ReadAllLines(GoldenPath);
        var reader = new SessionReader();

        var result = reader.TryReadHeader(System.Text.Encoding.ASCII.GetBytes(lines[0]), out long ticksPerSecond, out ulong seed);

        Assert.Equal(SessionOpenResult.Ok, result);
        Assert.Equal(10_000_000, ticksPerSecond);
        Assert.Equal(12345UL, seed);
    }

    [Fact]
    public void GoldenFixture_EveryDataLineDecodesSuccessfully_AndChecksumVerifies()
    {
        string[] lines = File.ReadAllLines(GoldenPath);
        Assert.True(lines.Length > 2, "golden fixture should have header + data + trailer");

        var reader = new SessionReader();
        var headerResult = reader.TryReadHeader(System.Text.Encoding.ASCII.GetBytes(lines[0]), out _, out _);
        Assert.Equal(SessionOpenResult.Ok, headerResult);
        reader.AccumulateChecksum(System.Text.Encoding.ASCII.GetBytes(lines[0]));

        int commandFrameCount = 0, stampedPoseCount = 0, latencyTraceCount = 0;
        ulong? trailerChecksum = null;

        for (int i = 1; i < lines.Length; i++)
        {
            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(lines[i]);

            if (lines[i].StartsWith("CF|"))
            {
                Assert.True(reader.TryReadCommandFrame(bytes, out _), $"line {i} failed to decode as CommandFrame: {lines[i]}");
                reader.AccumulateChecksum(bytes);
                commandFrameCount++;
            }
            else if (lines[i].StartsWith("SP|"))
            {
                Assert.True(reader.TryReadStampedPose(bytes, out _), $"line {i} failed to decode as StampedPose: {lines[i]}");
                reader.AccumulateChecksum(bytes);
                stampedPoseCount++;
            }
            else if (lines[i].StartsWith("LT|"))
            {
                Assert.True(reader.TryReadLatencyTrace(bytes, out _), $"line {i} failed to decode as LatencyTrace: {lines[i]}");
                reader.AccumulateChecksum(bytes);
                latencyTraceCount++;
            }
            else if (lines[i].StartsWith("EOS|"))
            {
                Assert.True(reader.TryReadEndOfSession(bytes, out ulong checksum));
                trailerChecksum = checksum;
            }
            else
            {
                Assert.Fail($"line {i} has an unrecognized tag: {lines[i]}");
            }
        }

        Assert.Equal(51, commandFrameCount); // 50 steady frames + 1 near-uint.MaxValue frame
        Assert.Equal(50, stampedPoseCount);
        Assert.Equal(10, latencyTraceCount);
        Assert.NotNull(trailerChecksum);
        Assert.True(reader.TryVerifyChecksum(trailerChecksum!.Value));
    }

    [Fact]
    public void GoldenFixture_ContainsASequenceNearUIntMaxValue()
    {
        string[] lines = File.ReadAllLines(GoldenPath);
        var reader = new SessionReader();

        bool foundNearMax = false;
        foreach (string line in lines)
        {
            if (!line.StartsWith("CF|")) continue;
            if (reader.TryReadCommandFrame(System.Text.Encoding.ASCII.GetBytes(line), out var frame) &&
                frame.Sequence > uint.MaxValue - 100)
            {
                foundNearMax = true;
                break;
            }
        }

        Assert.True(foundNearMax, "expected a CommandFrame with Sequence near uint.MaxValue");
    }

    [Fact]
    public void GoldenFixture_ContainsANegativeClockOffset()
    {
        string[] lines = File.ReadAllLines(GoldenPath);
        var reader = new SessionReader();

        bool foundNegativeOffset = false;
        foreach (string line in lines)
        {
            if (!line.StartsWith("LT|")) continue;
            if (reader.TryReadLatencyTrace(System.Text.Encoding.ASCII.GetBytes(line), out var trace) &&
                trace.TryGetClockOffsetTicks(out long offset) && offset < 0)
            {
                foundNegativeOffset = true;
                break;
            }
        }

        Assert.True(foundNegativeOffset, "expected a LatencyTrace with a negative ClockOffsetTicks");
    }
}
