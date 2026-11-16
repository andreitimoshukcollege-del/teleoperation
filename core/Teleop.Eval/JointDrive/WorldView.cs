using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
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
        /// <summary>A scene file as RobotHost loads it (<c>core/Scenes/*.json</c>), and the sha256 of its bytes (the world status names the one RobotHost runs).</summary>
        public static WorldScene LoadScene(string path, out byte[] sha256)
        {
            byte[] bytes = File.ReadAllBytes(path);
            SceneDocument doc = JsonSerializer.Deserialize<SceneDocument>(bytes, new JsonSerializerOptions { IncludeFields = true })
                ?? throw new InvalidDataException($"{path} is not a scene file");
            sha256 = SHA256.HashData(bytes);
            return doc.ToScene();
        }

        /// <summary>
        /// Object <paramref name="o"/>'s axis-aligned bounds in the arm base frame, at <paramref name="scale"/> times its file size,
        /// posed at (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>) with the rotation w, x, y, z: the box's
        /// eight corners turned and bounded, so a tipped or turned object is a little larger than it is (a cylinder or sphere is
        /// its box).
        /// </summary>
        public static SafetyObject Bounds(SceneObject o, double scale, double x, double y, double z, double qw, double qx, double qy, double qz)
        {
            var rotation = Quaternion.Normalize(new Quaternion((float)qx, (float)qy, (float)qz, (float)qw));
            var half = new Vector3((float)(o.SizeX * scale / 2), (float)(o.SizeY * scale / 2), (float)(o.SizeZ * scale / 2));
            var centre = new Vector3((float)x, (float)y, (float)z);
            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            for (int k = 0; k < 8; k++)
            {
                var corner = new Vector3((k & 1) == 0 ? -half.X : half.X, (k & 2) == 0 ? -half.Y : half.Y, (k & 4) == 0 ? -half.Z : half.Z);
                Vector3 p = centre + Vector3.Transform(corner, rotation);
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }

            return new SafetyObject(o.Id, min, max, o.Movable);
        }

        /// <summary>
        /// Every object of <paramref name="state"/>'s scene where it last was, at its size, but those the layout parked (a pool's,
        /// docs/adr/0037 decision 3): they are off-stage, out of every plan. False (and nothing) until every one has been posed.
        /// </summary>
        public static bool TryObjects(WorldState state, out List<SafetyObject> objects)
        {
            objects = new List<SafetyObject>();
            foreach (SceneObject o in state.Scene.Objects)
            {
                if (!state.TryGet(o.Id, out ObjectPose p, out _)) return false;
                if (p.Parked) continue;
                objects.Add(Bounds(o, state.Scale(o.Id), p.X, p.Y, p.Z, p.Qw, p.Qx, p.Qy, p.Qz));
            }

            return true;
        }
    }
}
