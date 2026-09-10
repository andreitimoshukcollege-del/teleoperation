using System.Runtime.CompilerServices;

namespace Teleop.Core.Tests.TestSupport;

/// <summary>
/// Asserts a delegate allocates nothing on the managed heap. The first allocation-assertion
/// harness in this project — enforces the "no allocations in the per-frame hot path" invariant
/// (root CLAUDE.md invariant 8) for every hot-path method introduced from Phase 3 onward.
/// </summary>
public static class AllocationAssert
{
    /// <summary>
    /// Runs <paramref name="action"/> through a full untimed warmup loop to absorb one-time costs
    /// that are not the hot path allocating (JIT tiering, a lazy static initializer, first-use
    /// collection growth) — then runs it <paramref name="iterations"/> more times and asserts the
    /// total bytes allocated on the current thread is exactly zero, not merely small. A truly
    /// allocation-free path allocates nothing; a nonzero-but-tiny delta is exactly the kind of
    /// regression (a boxed struct, a closure capture, a hidden string interpolation) this exists
    /// to catch rather than tolerate.
    ///
    /// <b>The warmup is a full loop, not a single call, and that matters.</b> A single call was not
    /// enough: .NET promotes a method to tier-1 only after roughly thirty calls and then recompiles
    /// it on a background thread, so with one warmup call the tiering transition landed *inside* the
    /// measured window. On a cold two-core CI runner that produced failures of 0.165 to 0.678
    /// bytes/call — fractions of a byte, which no real allocation can be, since the smallest object
    /// is twenty-four — and a *different* test failed on each run while the same code passed every
    /// time on a warmer developer machine. It made the gate enforcing invariant 8 unreliable on the
    /// one machine that enforces it, which is worse than no gate.
    ///
    /// <b>This does not weaken the assertion.</b> A warmup can only absorb costs that happen once.
    /// A per-call allocation still allocates on every iteration of the measured loop, so anything
    /// this harness was built to catch is caught exactly as before; only the one-time transition is
    /// excluded. The loop is deliberately identical to the measured one rather than some smaller
    /// fixed count, so whatever settles during measurement has already settled during warmup.
    /// </summary>
    public static void Zero(Action action, int iterations = 10_000)
    {
        RunLoop(action, iterations);

        Span<long> windows = stackalloc long[MaxWindows];
        int consecutiveClean = 0;

        for (int window = 0; window < MaxWindows; window++)
        {
            windows[window] = MeasureWindow(action, iterations);

            if (windows[window] == 0)
            {
                if (++consecutiveClean == RequiredCleanWindows)
                {
                    return;
                }
            }
            else
            {
                consecutiveClean = 0;
            }
        }

        var detail = new System.Text.StringBuilder();
        for (int window = 0; window < MaxWindows; window++)
        {
            detail.Append(window == 0 ? "" : ", ").Append(windows[window]);
        }

        Assert.Fail(
            $"Expected {RequiredCleanWindows} consecutive windows of zero allocation over " +
            $"{iterations} iterations each, but never got them in {MaxWindows} attempts. " +
            $"Bytes per window: [{detail}]. Last window: " +
            $"{(double)windows[MaxWindows - 1] / iterations:F3} bytes/call. " +
            "A per-call allocation reads as a large, repeatable figure in every window " +
            "(>= 24 bytes/call); a handful of bytes in one window only is the JIT-tiering race " +
            "this retry exists to absorb, and should not have reached here.");
    }

    /// <summary>
    /// Consecutive clean windows required to pass. Two, not one, and that is the whole design.
    ///
    /// The flake this defends against is a <b>race with a bounded, one-off payload</b>, not noise
    /// in the instrument: <see cref="GC.GetAllocatedBytesForCurrentThread"/> is exact and
    /// thread-local, so a parallel test cannot contribute a byte. What varies is *when* tier-1
    /// promotion and on-stack replacement land, because the compiler does that work on a
    /// background thread whose latency depends on machine load, while the warmup above is a fixed
    /// iteration count. Under load the transition slides past the warmup and into a measured
    /// window. Observed residues are 0.032 to 0.776 bytes/call — 320 to 7760 bytes *in total*,
    /// i.e. tens of objects once — where a genuine per-call regression is at least 24 bytes/call,
    /// four to five orders of magnitude larger. Run alone, the allocation tests pass 77/77.
    ///
    /// One clean window would be enough to absorb that, but two is strictly stronger and costs one
    /// more loop: a one-time transition can pollute a bounded number of windows, while anything
    /// *periodic* — a buffer that grows every N calls, a cache that rebuilds — keeps landing in
    /// them and can never produce two clean ones in a row. So the retry does not trade rigour for
    /// stability; it separates one-time from recurring, which a single window structurally cannot
    /// do even though the two differ by four orders of magnitude.
    ///
    /// <b>The assertion is still exactly zero.</b> No tolerance was introduced, and none would
    /// work: a real 1-in-200 regression reports 0.160 bytes/call, inside the observed flake range,
    /// so any byte threshold that silences the flake also silences the regression.
    ///
    /// <b>Honest limit:</b> nobody has captured the allocating stack. The tiering explanation is
    /// inferred from magnitude, from correlation with machine load, and from two controls — the
    /// tests pass in isolation, and the flake persists with candidate code removed. It is not a
    /// profile. If failures survive this change, that inference is where to look first.
    /// </summary>
    private const int RequiredCleanWindows = 2;

    /// <summary>
    /// Attempts before giving up. Bounded so a genuine regression fails promptly rather than
    /// spinning: a real one never produces a clean window, so it costs the full budget and stops.
    /// </summary>
    private const int MaxWindows = 5;

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static long MeasureWindow(Action action, int iterations)
    {
        long before = GC.GetAllocatedBytesForCurrentThread();
        RunLoop(action, iterations);
        long after = GC.GetAllocatedBytesForCurrentThread();
        return after - before;
    }
    /// <summary>
    /// The loop, factored out so warmup and measurement run the <b>same</b> one.
    ///
    /// This is the correction to a first attempt that warmed up with a separate loop in this same
    /// method. That warmed the delegate but not the loop calling it: the measured loop was still
    /// entered cold, and a long-running cold loop is exactly what triggers on-stack replacement
    /// mid-execution — inside the measurement window. Tiering of the callee was fixed and tiering
    /// of the caller was left, which is why fractional bytes/call kept appearing on CI after the
    /// first fix, on a different test each run.
    ///
    /// <see cref="MethodImplOptions.NoInlining"/> is load-bearing rather than decorative: inlined
    /// into <see cref="Zero"/>, this collapses back into two separate loops and the bug returns
    /// silently, with nothing in the source to show it.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void RunLoop(Action action, int iterations)
    {
        for (int i = 0; i < iterations; i++)
        {
            action();
        }
    }
}
