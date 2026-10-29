using System;
using System.Collections.Generic;

namespace Teleop.RobotArm.Collision
{
    /// <summary>
    /// The file form of a <see cref="SerialChainCollisionModel"/> (<c>core/RobotProfiles/&lt;profile&gt;.collision.json</c>,
    /// docs/adr/0026 amendment), written by <c>Teleop.Eval fit-capsules</c> and never by hand. Like the profile's
    /// document, it has plain fields named as the JSON names them, for <c>System.Text.Json</c> (with fields) and Unity's
    /// <c>JsonUtility</c> alike.
    /// </summary>
    [Serializable]
    public sealed class SerialChainCollisionDocument
    {
        public string profile = "";
        public string kind = "serial-chain-collision";
        public double margin = 0.01;

        /// <summary>The URDF the capsules were fitted to, and its sha256.</summary>
        public string urdf = "";
        public string sha256 = "";

        public ChainCapsuleDocument[] capsules = Array.Empty<ChainCapsuleDocument>();
        public DisabledPairDocument[] disabled = Array.Empty<DisabledPairDocument>();

        public SerialChainCollisionModel ToModel()
        {
            if (kind != "serial-chain-collision") throw new FormatException($"collision model has kind '{kind}', not 'serial-chain-collision'");
            var caps = new ChainCapsule[capsules.Length];
            var index = new Dictionary<string, int>(StringComparer.Ordinal);
            for (int i = 0; i < capsules.Length; i++)
            {
                ChainCapsuleDocument c = capsules[i];
                if (c.a == null || c.a.Length != 3 || c.b == null || c.b.Length != 3) throw new FormatException($"capsule '{c.name}': a and b need three numbers each");
                if (index.ContainsKey(c.name)) throw new FormatException($"capsule '{c.name}' is listed twice");
                index[c.name] = i;
                caps[i] = new ChainCapsule(c.name, c.arm, c.frame, c.a[0], c.a[1], c.a[2], c.b[0], c.b[1], c.b[2], c.radius);
            }

            var pairs = new List<(int, int)>();
            foreach (DisabledPairDocument p in disabled)
            {
                if (!index.TryGetValue(p.a, out int a) || !index.TryGetValue(p.b, out int b)) throw new FormatException($"disabled pair ({p.a}, {p.b}) names a capsule that is not listed");
                pairs.Add((a, b));
            }

            return new SerialChainCollisionModel(profile, caps, pairs.ToArray(), margin);
        }
    }

    [Serializable]
    public sealed class ChainCapsuleDocument
    {
        public string name = "";

        /// <summary>The arm's index, or -1 for the robot's fixed body.</summary>
        public int arm = -1;

        /// <summary>-1: the arm's base (or, for the body, the root); k: joint k's moving frame.</summary>
        public int frame = -1;

        public double[] a = { 0, 0, 0 };
        public double[] b = { 0, 0, 0 };
        public double radius;
    }

    [Serializable]
    public sealed class DisabledPairDocument
    {
        public string a = "";
        public string b = "";

        /// <summary>Why the pair is never checked: <c>adjacent</c> (joined by a joint, or both on the mount) or <c>touching-at-ready</c>.</summary>
        public string reason = "";
    }
}
