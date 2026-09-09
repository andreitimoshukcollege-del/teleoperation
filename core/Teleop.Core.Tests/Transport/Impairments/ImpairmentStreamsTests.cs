using Teleop.Core.Transport.Impairments;

namespace Teleop.Core.Tests.Transport.Impairments;

/// <summary>
/// Pins the substream derivation to hardcoded literals.
///
/// <b>Why literals rather than comparing two instances.</b> The hazard this guards against is
/// <c>string.GetHashCode()</c> creeping into <see cref="ImpairmentStreams.StreamIdFromName"/>. .NET
/// Core randomises string hashing per process, so a <c>GetHashCode</c>-derived stream id is stable
/// within one run and different on the next. A test that hashed the same name twice in one process,
/// or compared two impairments constructed side by side, would pass every single time while
/// reproducibility across runs was completely gone. Only a constant the test does not itself compute
/// can catch that.
///
/// The same literals also catch an axis rename (which silently changes every stream that axis
/// produces, making new runs incomparable with old ones) and any change to the hash or the mixer.
/// If one of these fails, the correct response is almost never to update the constant — it is to
/// work out which of those three things happened and whether it was intended.
///
/// The expected values were cross-checked against an independent implementation written in Python
/// before this test was written, so they are not merely a recording of whatever the C# happened to
/// produce.
/// </summary>
public class ImpairmentStreamsTests
{
    [Theory]
    [InlineData("delay", 0x6AAF4991CE52F9F0UL)]
    [InlineData("jitter", 0xC89A145674B04B69UL)]
    [InlineData("loss", 0x29AC3008E7D38666UL)]
    [InlineData("reorder", 0x03FE96C1BBF2C45CUL)]
    [InlineData("delay-trace", 0x32175612FA92EEFCUL)]
    public void StreamIdFromName_MatchesFrozenValues(string axisName, ulong expected)
    {
        Assert.Equal(expected, ImpairmentStreams.StreamIdFromName(axisName));
    }

    [Theory]
    [InlineData("delay", 0x709969726A2F3B11UL)]
    [InlineData("jitter", 0x366C596945A783F0UL)]
    [InlineData("loss", 0x45EE7ACE7E65C817UL)]
    [InlineData("reorder", 0xF8B2BECE4940EF18UL)]
    [InlineData("delay-trace", 0x53BE07F952524999UL)]
    public void DeriveForAxis_MatchesFrozenValues(string axisName, ulong expected)
    {
        Assert.Equal(expected, ImpairmentStreams.DeriveForAxis(555UL, axisName));
    }

    /// <summary>
    /// The property the whole design rests on: two axes under one trial seed must not share a
    /// stream. Two impairments drawing identical numbers would correlate loss with jitter, which no
    /// real link does and which nothing in the source would look wrong.
    /// </summary>
    [Fact]
    public void Derive_DistinctAxes_ProduceDistinctSubstreamSeeds()
    {
        string[] axes = { "delay", "jitter", "loss", "reorder", "delay-trace" };
        var seen = new HashSet<ulong>();

        foreach (string axis in axes)
        {
            Assert.True(
                seen.Add(ImpairmentStreams.DeriveForAxis(1234UL, axis)),
                $"'{axis}' collided with an earlier axis's substream seed.");
        }
    }

    /// <summary>
    /// <c>SweepCommand</c> gives the uplink <c>seed</c> and the downlink <c>seed + 1</c>. If the
    /// mixer did not avalanche, those two would produce near-identical substreams and the two
    /// directions of the link would lose and delay the same datagrams together.
    /// </summary>
    [Fact]
    public void Derive_AdjacentTrialSeeds_ProduceUnrelatedSubstreamSeeds()
    {
        ulong a = ImpairmentStreams.DeriveForAxis(1000UL, "loss");
        ulong b = ImpairmentStreams.DeriveForAxis(1001UL, "loss");

        Assert.NotEqual(a, b);

        // Avalanche: a one-bit input change should flip roughly half of the 64 output bits. A
        // threshold of 16 is far below the expected ~32 and far above what a weak mixer would give,
        // so this fails loudly on a broken mixer without being flaky about the exact count.
        int differingBits = System.Numerics.BitOperations.PopCount(a ^ b);
        Assert.InRange(differingBits, 16, 48);
    }

    /// <summary>
    /// Names differing only in case or by one character must not collide. Ordinal, no culture
    /// folding -- the same guarantee `Registries` gets from its explicit `StringComparer.Ordinal`.
    /// </summary>
    [Fact]
    public void StreamIdFromName_IsCaseSensitiveAndOrdinal()
    {
        Assert.NotEqual(
            ImpairmentStreams.StreamIdFromName("delay"),
            ImpairmentStreams.StreamIdFromName("Delay"));

        Assert.NotEqual(
            ImpairmentStreams.StreamIdFromName("delay"),
            ImpairmentStreams.StreamIdFromName("delay-trace"));
    }
}
