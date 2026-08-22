using System.Runtime.CompilerServices;

// Lets Teleop.Eval.Tests exercise internal-only entry points (e.g. BuildProfileCommand's
// TextReader/TextWriter-injectable overload) without making them public API of an Exe project
// nothing else references.
[assembly: InternalsVisibleTo("Teleop.Eval.Tests")]
