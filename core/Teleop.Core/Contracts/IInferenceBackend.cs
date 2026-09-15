using System;

// C# 9: block-scoped namespace only. File-scoped namespaces (namespace X;) are C# 10
// and will not compile in Unity 2022.3.
namespace Teleop.Core.Contracts
{
    /// <summary>
    /// Runs a learned model on flat float buffers. Core never links an ML runtime: Sentis needs
    /// Unity 2023.2+ and this project is pinned to 2022.3, and in any case a NuGet or plugin
    /// dependency in Core would break the one-assembly-two-compilers property. A backend is
    /// chosen at Phase 7 and supplied by the host — ONNX Runtime headless, whatever ships
    /// on-device for the Quest — behind this interface.
    ///
    /// <b>There are no consumers and no implementations today.</b> This comment used to name
    /// <c>Prediction/SequenceModelPredictor.cs</c> as "the one consumer today"; that file has
    /// never existed — <c>seq-model</c> is a "planned, not built" row in
    /// <c>Prediction/CLAUDE.md</c>, deferred because it depends on a Phase-7 backend decision
    /// nobody has made. Corrected because this was the only place in the repo asserting it in
    /// the present tense, and anyone following the world-model direction in root
    /// <c>CLAUDE.md</c> would come here first.
    ///
    /// The intended consumer still shapes the contract, and the requirement it implies is real:
    /// a predictor calls this from inside <c>Predict</c>, never an ONNX library directly, so
    /// inference must be repeatable without allocating — the model is loaded once at
    /// construction and every call reads and writes caller-owned spans over preallocated
    /// buffers.
    ///
    /// A backend must be deterministic for a fixed input, or results from it are not
    /// comparable across runs. Backends that are not (nondeterministic GPU reductions) must say
    /// so, and a result produced by one is not reproducible in the sense this project means.
    /// </summary>
    public interface IInferenceBackend
    {
        /// <summary>
        /// Identifier of the loaded model — the exported artifact's name and version, as it
        /// appears in the run manifest. A figure produced by a model that cannot be named is
        /// not reproducible.
        /// </summary>
        string ModelId { get; }

        /// <summary>
        /// Number of floats one inference consumes. Fixed at construction; Core does not do
        /// dynamic shapes, because a variable-size input would mean a variable-size buffer and
        /// an allocation on the hot path.
        /// </summary>
        int InputLength { get; }

        /// <summary>Number of floats one inference produces. Fixed at construction.</summary>
        int OutputLength { get; }

        /// <summary>
        /// Run one inference. <paramref name="input"/> must be exactly
        /// <see cref="InputLength"/> long and <paramref name="output"/> at least
        /// <see cref="OutputLength"/>; a mismatch returns false and leaves
        /// <paramref name="output"/> untouched rather than throwing, so a caller on the frame
        /// path can fall back to a non-learned predictor instead of dropping a frame.
        ///
        /// Returns false on backend failure too. The caller must have a fallback; a model that
        /// fails at 90 Hz cannot take the loop down with it.
        ///
        /// Synchronous by contract. No threads, no async, no GPU fence waits hidden behind an
        /// await: the caller is inside a per-frame predictor and needs a value now.
        /// Allocation-free.
        /// </summary>
        bool TryRun(ReadOnlySpan<float> input, Span<float> output);

        /// <summary>
        /// Clears any state carried between inferences — recurrent hidden state, a warm-start
        /// buffer, an internal history window. The loaded model itself survives, since
        /// reloading it would allocate. A stateless backend implements this as a no-op and says
        /// so. Sweeps reuse instances across trials, and hidden state leaking between trials is
        /// invisible in the output but destroys the comparison.
        /// </summary>
        void Reset();
    }
}
