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
        for (int i = 0; i < iterations; i++)
        {
            action();
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < iterations; i++)
        {
            action();
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        long allocated = after - before;
        Assert.True(
            allocated == 0,
            $"Expected zero allocation over {iterations} iterations, but {allocated} bytes " +
            $"were allocated ({(double)allocated / iterations:F3} bytes/call).");
    }
}
