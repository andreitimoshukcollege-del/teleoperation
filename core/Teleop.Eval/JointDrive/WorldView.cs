using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text.Json;
using Teleop.RobotArm.World;

namespace Teleop.Eval.JointDrive
{
    /// <summary>
    /// joint-drive's view of the scene's objects (docs/adr/0036 amendment 1): the scene file it was given, and where RobotHost
    /// says each object is now (its world frames and sizes on the joint port, as the headset takes them), as the boxes the
    /// safety scripts plan around.
    /// </summary>
    public static class WorldView
    {
        private static readonly ConditionalWeakTable<WorldScene, ObjectShapes> Shapes = new ConditionalWeakTable<WorldScene, ObjectShapes>();

        /// <summary>
        /// A scene file as RobotHost loads it (<c>core/Scenes/*.json</c>), and the sha256 of its bytes (the world status names the
        /// one RobotHost runs). The scene's shapes file beside it (<see cref="ObjectShapes.PathFor"/>), when there is one, comes
        /// with it (<see cref="ShapesOf"/>): its objects' own shapes, which collide_and_recover touches.
        /// </summary>
        public static WorldScene LoadScene(string path, out byte[] sha256)
        {
            byte[] bytes = File.ReadAllBytes(path);
            SceneDocument doc = JsonSerializer.Deserialize<SceneDocument>(bytes, new JsonSerializerOptions { IncludeFields = true })
                ?? throw new InvalidDataException($"{path} is not a scene file");
            sha256 = SHA256.HashData(bytes);
            WorldScene scene = doc.ToScene();
            string shapes = ObjectShapes.PathFor(path);
            if (File.Exists(shapes)) Shapes.AddOrUpdate(scene, ObjectShapes.Load(shapes));
            return scene;
        }

        /// <summary>The shapes file loaded with <paramref name="scene"/> (docs/adr/0037 amendment 5, third revision), or null.</summary>
        public static ObjectShapes? ShapesOf(WorldScene scene) => Shapes.TryGetValue(scene, out ObjectShapes? shapes) ? shapes : null;

        /// <summary>
        /// Object <paramref name="o"/>'s box in the arm base frame, at <paramref name="scale"/> times its file size, posed at
        /// (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>) with the rotation w, x, y, z, turned as the object
        /// is (a cylinder or sphere is its box). Its axis-aligned bounds were the whole object until 2026-11-16, and a touch at a
        /// turned object's corner stopped up to 3 cm short of it. With <paramref name="shapes"/> (the scene's, <see cref="ShapesOf"/>),
        /// a scanned object also has its own shape in that box, at its size.
        /// </summary>
        public static SafetyObject Bounds(SceneObject o, double scale, double x, double y, double z, double qw, double qx, double qy, double qz,
            ObjectShapes? shapes = null, bool scenery = false)
        {
            var rotation = Quaternion.Normalize(new Quaternion((float)qx, (float)qy, (float)qz, (float)qw));
            var half = new Vector3((float)(o.SizeX * scale / 2), (float)(o.SizeY * scale / 2), (float)(o.SizeZ * scale / 2));
            var centre = new Vector3((float)x, (float)y, (float)z);

            // Mass at size, as Isaac's bodies have it; tippy from the proportions as it rests (a uniform size keeps them).
            return new SafetyObject(o.Id, centre, rotation, half, o.Movable, o.Mass * scale * scale * scale, IsTippy(o, qw, qx, qy, qz, shapes?.For(o)), shapes?.For(o)?.At(half),
                scenery);
        }

        /// <summary>
        /// Whether object <paramref name="o"/>, resting with the rotation w, x, y, z, is taller than <see cref="SafetyObject.TippyAspect"/>
        /// times its narrower side: upright, its file's height over its footprint; lying on a side (docs/adr/0037 amendment 6), the
        /// side that is up over the two that lie flat. A marker on its side is not tippy; standing, it is.
        /// </summary>
        public static bool IsTippy(SceneObject o, double qw, double qx, double qy, double qz, AssetShape? shape = null)
        {
            int rest = WorldScene.RestOf(qw, qx, qy, qz, out _) % 3;
            // Lying on a round side it rolls when pushed, which the take check counts as a tip: no steady target for a touch
            // (the 2026-11-21 check: a touch rolled the mug, lying, 34 degrees). Without its shape only a cylinder is known round.
            if (rest != 0 && (o.Shape == SceneShape.Cylinder || (shape != null && shape.Round)))
                return true;

            double up, a, b;
            switch (rest)
            {
                case 1: up = o.SizeY; a = o.SizeX; b = o.SizeZ; break;
                case 2: up = o.SizeX; a = o.SizeZ; b = o.SizeY; break;
                default: up = o.SizeZ; a = o.SizeX; b = o.SizeY; break;
            }

            return up > SafetyObject.TippyAspect * Math.Min(a, b);
        }

        /// <summary>
        /// Every object of <paramref name="state"/>'s scene where it last was, at its size, but those the layout parked (a pool's,
        /// docs/adr/0037 decision 3): they are off-stage, out of every plan. With the shapes loaded with the scene, each scanned
        /// object has its own. False (and nothing) until every one has been posed.
        /// </summary>
        public static bool TryObjects(WorldState state, out List<SafetyObject> objects)
        {
            objects = new List<SafetyObject>();
            ObjectShapes? shapes = ShapesOf(state.Scene);
            foreach (SceneObject o in state.Scene.Objects)
            {
                if (!state.TryGet(o.Id, out ObjectPose p, out _)) return false;
                if (p.Parked) continue;
                // A movable object standing by the bench's edge is scenery (docs/adr/0037 amendment 6), an obstacle and never a target.
                objects.Add(Bounds(o, state.Scale(o.Id), p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz, shapes, o.Movable && state.Scene.InEdgeRegions(p.X, p.Y)));
            }

            return true;
        }
    }
}
