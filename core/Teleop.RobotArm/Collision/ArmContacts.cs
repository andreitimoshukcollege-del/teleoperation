using System.Collections.Generic;
using Teleop.RobotArm.Types;

namespace Teleop.RobotArm.Collision
{
    /// <summary>
    /// A pose that rests on something touches the collision model's fixed boxes by definition: the arm
    /// parked on its support (docs/adr/0023 decision 5), or sagged onto it with its torque off. To move
    /// such a pose at all, the contacts it already has are allowed, and nothing else.
    /// </summary>
    public static class ArmContacts
    {
        /// <summary>
        /// <paramref name="model"/> with every (body, fixed box) contact <paramref name="pose"/> has
        /// ignored, widened to every body on the same link; nothing else changes. The model itself when
        /// the pose is clear. Null when the pose's hit is not with a fixed box (the floor, another link, a
        /// joint range), which no support explains. <paramref name="contacts"/> names what was allowed,
        /// or the hit that could not be.
        /// </summary>
        public static ArmCollisionModel? AllowPoseContacts(in RobotArmProfile profile, ArmCollisionModel model, in ArmPose pose, out string contacts)
        {
            var pairs = new List<(int, int)>();
            var boxes = new List<(int, int)>();
            int n = model.Bodies.Length, m = model.Boxes.Length;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++) if (model.IgnoresBodyPair(i, j)) pairs.Add((i, j));
                for (int k = 0; k < m; k++) if (model.IgnoresBodyBox(i, k)) boxes.Add((i, k));
            }

            var found = new List<string>();
            ArmCollisionModel current = model;
            for (int round = 0; round <= m; round++)
            {
                CollisionHit hit = ArmCollision.Check(profile, current, pose);
                if (!hit.IsHit)
                {
                    contacts = string.Join(", ", found);
                    return current;
                }

                if (hit.Kind != CollisionHitKind.BodyBox)
                {
                    found.Add(hit.Describe(model));
                    contacts = string.Join(", ", found);
                    return null;
                }

                ArmLink link = model.Bodies[hit.A].Link;
                for (int b = 0; b < n; b++) if (model.Bodies[b].Link == link) boxes.Add((b, hit.B));
                found.Add($"{link} on {model.Boxes[hit.B].Name}");
                current = new ArmCollisionModel(model.Name + " (pose contacts allowed)", model.Bodies, model.Boxes, model.FloorZ, model.Margin,
                    model.BaseYaw, model.Proximal, model.Distal, model.Wrist, model.Roll, model.Home, pairs.ToArray(), boxes.ToArray(), model.Source);
            }

            contacts = string.Join(", ", found);
            return null;
        }
    }
}
