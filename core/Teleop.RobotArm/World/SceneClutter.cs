using System;
using Teleop.Core.Types;

// C# 9: block-scoped namespace only, like the rest of Teleop.RobotArm.
namespace Teleop.RobotArm.World
{
    /// <summary>
    /// The clutter pass of a pool's layout (docs/adr/0037 "Raised clutter"): desk clutter tall enough for the arm to hit (a roll of
    /// tape, tools, a phone, a pen), stood after every other object of the layout. Flat clutter, 6 mm or thinner, is the
    /// renderer's alone (robot/isaac's <c>clutter.py</c>): the arm's floor keeps it out of reach. This clutter is real scene
    /// objects, so Isaac collides with it, the safety scripts plan around it, and the contact labels see it.
    /// <list type="bullet">
    /// <item><b>Its own generator.</b> A <see cref="SeededRng"/> made from the layout's seed by <see cref="SeedOf"/>, never the one
    /// the rest of the layout draws from: a pool without clutter draws exactly what it drew before, and with clutter every other
    /// object stands where it would without it (the clutter only fills in around them).</item>
    /// <item><b>The draw.</b> How many (uniform in <see cref="WorldScene.ClutterMin"/> to <see cref="WorldScene.ClutterMax"/>),
    /// which (a partial shuffle of the clutter objects in file order), then each one's spot: anywhere in the placement region
    /// where its footprint fits, clear of the robot's keep-out and of every object standing, yaw uniform, flat as the file has
    /// it (its natural rest, never a side), up to <see cref="SceneLayout.MaxPoolTries"/> tries. One that finds no spot stays
    /// parked. No reach band: some clutter lands in the arm's reach (an obstacle, and sometimes a target), some in the cameras'
    /// foreground.</item>
    /// </list>
    /// Allocates nothing.
    /// </summary>
    public static class SceneClutter
    {
        /// <summary>What the layout's seed is xored with to seed the clutter pass's generator ("clutter!" in ASCII).</summary>
        public const ulong SeedSalt = 0x636C757474657221UL;

        /// <summary>The seed of the clutter pass's generator for the layout drawn from <paramref name="layoutSeed"/>.</summary>
        public static ulong SeedOf(ulong layoutSeed) => layoutSeed ^ SeedSalt;

        /// <summary>
        /// Stands the layout's clutter among <paramref name="clutter"/> (indices of the scene's clutter objects, in file order; the
        /// chosen ones are moved to its front), each clear of everything <paramref name="standing"/> holds. Returns how many chosen
        /// found no spot. Draws nothing for a pool without clutter.
        /// </summary>
        internal static int Stand(WorldScene scene, ulong layoutSeed, Span<int> clutter, ref SceneLayout.Standing standing)
        {
            if (scene.ClutterMax <= 0 || clutter.Length == 0) return 0;
            var rng = new SeededRng(SeedOf(layoutSeed));
            int take = SceneLayout.Count(ref rng, scene.ClutterMin, Math.Min(scene.ClutterMax, clutter.Length));
            SceneLayout.Choose(ref rng, clutter, take);
            int failed = 0;
            for (int k = 0; k < take; k++) failed += standing.StandClutter(clutter[k], ref rng) ? 0 : 1;
            return failed;
        }
    }
}
