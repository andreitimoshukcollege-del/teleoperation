# robot/franka/fr3_duo/ — the FR3 Duo robot description

Franka Robotics' own URDF for the fixed dual-arm FR3 Duo (docs/adr/0026). It is two FR3 v2 7-joint
arms, each with a Franka Hand, on the v0.3 fixed mount, with the v0.2 head. Nothing here is edited
by hand. It is the input that `Teleop.Eval import-urdf` turns into `core/RobotProfiles/fr3-duo.json`,
and that Unity and capsule fitting read meshes from.

## Provenance

| | |
|---|---|
| Upstream | [`frankarobotics/franka_description`](https://github.com/frankarobotics/franka_description) @ `7aeeddc449edf8d62b594f9e36a81da53e7796f9` (release 2.9.0, 2026-08-17), `robots/fr3_duo/fr3_duo.urdf.xacro` |
| Expanded by | [`hyyyyyyz/awesome-robot-arm-zoo`](https://github.com/hyyyyyyz/awesome-robot-arm-zoo) @ `ca0cabe27dfe1d5151fd58659e8cb981a878a98e`, `models/franka/fr3_duo/` (xacro 2.1.1) |
| `urdf/fr3_duo.urdf` sha256 | `5f1ab744deb0097f014163395aa828a7988edcc72526fd8e5cb0e6e6a6494162` |
| Copied | 2026-10-28 |
| License | Apache-2.0, © Franka Robotics GmbH (`urdf/LICENSE`, `urdf/NOTICE`) |

The expansion came from a third-party collection, so it was checked against upstream on 2026-10-28:
- **Meshes:** all 25 files under `urdf/resources/franka_description/meshes/` have the same git blob
  hash as upstream at that commit.
- **The mount:** its transforms match upstream's `accessories/fr3_duo_mount_v0_3/kinematics.yaml`.
- **The arm joint origins:** they reproduce Franka's published Craig DH table (d1 0.333, d3 0.316,
  a4 0.0825, a5 −0.0825 / d5 0.384, a7 0.088, flange 0.107).

`preview.png` is the collection's MuJoCo render. To replace the description, copy a newer expansion
and re-run that check. Then regenerate the profile, because the profile records this file's sha256.

## What is in the URDF

- **The root** is `base`. The mount origin is (0.037, 0, 0.05) from it.
- **The arm mounts** are at y = ±0.05018 m, with rpy (∓0.8933, −0.1746, ∓0.4633). The two arms'
  bases are 10 cm apart and tilted outward, so arm-to-arm collision is a real constraint.
- **The arms:**
  - joints `{left,right}_fr3v2_joint1..7` are revolute;
  - each joint has `limit`, `safety_controller` and `position_based_velocity_limits`;
  - joint 8, the flange, is fixed at +0.107 m;
  - the hand is rotated −45° about z;
  - the TCP `*_fr3v2_hand_tcp` is +0.1034 m from the hand.
- **The fingers:**
  - `*_fr3v2_finger_joint1` is prismatic, 0–0.04 m, at 0.2 m/s;
  - `finger_joint2` mimics joint 1.
- **The meshes:** visual meshes are COLLADA (`.dae`, 58 MB, in LFS). Collision meshes are STL
  (`.stl`, 180 KB, plain git, so they can be read headlessly without LFS).
- **The head** carries a camera mounting point. Nothing in this repo uses it yet.
