# Teleop.Core.Tests

xUnit tests for `Teleop.Core`. `net8.0`, unconstrained by Core's `netstandard2.1`/`LangVersion 9.0`
rules -- this project never compiles under Unity, so file-scoped namespaces and modern C# are
fine here.

## `TestSupport/AllocationAssert.cs`

The project's allocation-assertion harness: `AllocationAssert.Zero(Action action, int iterations)`
warms up once, then asserts zero bytes allocated across `iterations` more calls via
`GC.GetAllocatedBytesForCurrentThread()`. Use this for every hot-path method on a new
implementation (`Predict`, `Reconcile`, `Send`/`TryReceive`, `Record`, and so on) rather than
inventing an ad hoc per-file check -- this is what root CLAUDE.md's "there are
allocation-assertion tests; keep them passing" refers to.

## Fixtures

`core/testdata/golden/*.tlog` is copied into the test output directory via an explicit
`<None Include=... Link=...>` item in this project's `.csproj` (a glob with `Link` transforms
was considered and rejected -- MSBuild's default handling of a `..`-relative `Include` path does
not reliably preserve the `testdata/golden/` structure under the output directory). Read it via
`Path.Combine(AppContext.BaseDirectory, "testdata", "golden", "basic-session.tlog")`.
