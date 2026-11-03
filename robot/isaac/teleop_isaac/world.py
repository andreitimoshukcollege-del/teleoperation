"""The world beside the arms in Isaac (docs/adr/0031 amendment 2): the scene file's surfaces and fixed obstacles.

RobotHost decides where objects go (``WorldHost``, from the seeded layouts) and asks with PLACE; this module applies or
refuses each placement and reports where the objects are. The rules are RobotHost's ``StaticWorld``'s, in its order, plus
what only physics can see: whether an arm is in the way.

1. an id the scene does not have: UNKNOWN_OBJECT;
2. from the headset, an object it may not move: NOT_MOVABLE;
3. a pose that is not finite or not a unit quaternion: INVALID_POSE;
4. from the headset, an object an arm is touching now: ARM_TOUCHING;
5. off the table or the placement region: OUTSIDE_REGION;
6. closer to another object than the clearance (footprints, as Core's ``WorldScene.Overlaps``): OVERLAPS_OBJECT;
7. an arm within the clearance of the object's shape at the new pose (a PhysX overlap query): OVERLAPS_ARM;
8. otherwise the object is stood upright on the table at the pose's x, y and yaw (``Scene.pose_at``): APPLIED.

:class:`WorldModel` is the bookkeeping and the rules, with the arm check passed in, so it runs in the plain-Python tests.
:class:`IsaacWorld` builds the prims and answers the arm check; it imports Isaac only when it is made.
"""

from __future__ import annotations

import math
import re
from typing import Callable, List, Optional, Sequence, Tuple

from . import wire
from .scene import Scene

Pose = Tuple[float, float, float, float, float, float, float]  # x, y, z, qw, qx, qy, qz

# An arm check: (object index, pose, margin metres) -> whether an arm is within the margin of that object there.
ArmCheck = Callable[[int, Pose, float], bool]

TOUCH_MARGIN = 0.005  # metres: an arm this close to an object counts as touching it (rule 4)


def prim_name(prefix: str, name: str) -> str:
    """A USD prim name for a scene name: scene names may hold '-' and '.', prim names may not; the prefix keeps two
    names that differ only there apart (``o1_post_left``)."""
    return f"{prefix}_{re.sub(r'[^A-Za-z0-9_]', '_', name)}"


def yaw_of(p: Sequence[float]) -> float:
    """The turn about the vertical, as RobotHost's ``WorldHost.YawOf``."""
    _, _, _, w, x, y, z = p
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_valid(p: Sequence[float]) -> bool:
    """As Core's ``ObjectPose.IsValid``: finite, and a unit quaternion within 0.2 %."""
    if len(p) != 7 or not all(math.isfinite(v) for v in p):
        return False
    n = p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6]
    return 0.998 <= n <= 1.002


class WorldModel:
    """Where every object of the scene is, and the placement rules. Objects start at their nominal poses."""

    def __init__(self, scene: Scene):
        self.scene = scene
        self.poses: List[Pose] = [tuple(o.nominal) for o in scene.objects]
        self.revision = 0
        self.snapshot = 0

    def index_of(self, object_id: int) -> int:
        for i, o in enumerate(self.scene.objects):
            if o.id == object_id:
                return i
        return -1

    def check(self, cause: int, object_id: int, pose: Sequence[float], arm_near: ArmCheck) -> Tuple[int, int, Optional[Pose]]:
        """(outcome, object index or -1, the upright pose to apply when the outcome is APPLIED)."""
        i = self.index_of(object_id)
        if i < 0:
            return wire.UNKNOWN_OBJECT, -1, None
        o = self.scene.objects[i]
        if cause == wire.CAUSE_HEADSET and not o.operator_movable:
            return wire.NOT_MOVABLE, i, None
        if not is_valid(pose):
            return wire.INVALID_POSE, i, None
        if cause == wire.CAUSE_HEADSET and arm_near(i, self.poses[i], TOUCH_MARGIN):
            return wire.ARM_TOUCHING, i, None
        x, y = pose[0], pose[1]
        if not self.scene.fits_on_table(i, x, y):
            return wire.OUTSIDE_REGION, i, None
        for k, other in enumerate(self.poses):
            if k != i and self.scene.overlaps(i, x, y, k, other[0], other[1]):
                return wire.OVERLAPS_OBJECT, i, None
        upright = tuple(self.scene.pose_at(i, x, y, yaw_of(pose)))
        if arm_near(i, upright, self.scene.clearance):
            return wire.OVERLAPS_ARM, i, None
        return wire.APPLIED, i, upright

    def apply(self, index: int, pose: Pose) -> None:
        self.poses[index] = pose
        self.revision += 1

    def objects(self) -> List[wire.ObjectPose]:
        return [wire.ObjectPose(o.id, wire.FLAG_KINEMATIC, self.poses[i]) for i, o in enumerate(self.scene.objects)]

    def world_messages(self, sequence: Callable[[], int], sensed_ticks: int,
                       objects: Optional[Sequence[wire.ObjectPose]] = None) -> List[bytes]:
        """One snapshot as WORLD messages, at most 16 objects each."""
        objects = list(objects if objects is not None else self.objects())
        self.snapshot += 1
        per = wire.MAX_OBJECTS_PER_WORLD
        parts = max(1, (len(objects) + per - 1) // per)
        return [wire.encode_world(sequence(), sensed_ticks, self.snapshot, self.revision, k, parts, objects[k * per:(k + 1) * per])
                for k in range(parts)]


class IsaacWorld:
    """The scene in Isaac's stage: surfaces as static colliders, obstacles as kinematic rigid bodies, on the session layer
    so the robot's USD is never changed. Call :meth:`build` before the simulation plays and :meth:`attach` after."""

    ROOT = "/World/Scene"

    def __init__(self, scene: Scene, robot_path: str):
        self.model = WorldModel(scene)
        self.robot_path = robot_path.rstrip("/")
        self.object_paths = [f"{self.ROOT}/Objects/{prim_name(f'o{o.id}', o.name)}" for o in scene.objects]
        self._bodies = None
        self._query = None

    # --- before play ------------------------------------------------------------------------------------------------

    def build(self, stage) -> None:
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

        def material(path: str, static: float, dynamic: float):
            mat = UsdShade.Material.Define(stage, path)
            api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(static)
            api.CreateDynamicFrictionAttr(dynamic)
            api.CreateRestitutionAttr(0.0)
            return mat

        def bind_physics(prim, mat) -> None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

        scene = self.model.scene
        fixed = self.fixed_bodies(stage)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform.Define(stage, self.ROOT)
            UsdGeom.Xform.Define(stage, f"{self.ROOT}/Surfaces")
            UsdGeom.Xform.Define(stage, f"{self.ROOT}/Objects")
            UsdGeom.Scope.Define(stage, f"{self.ROOT}/Materials")
            for k, s in enumerate(scene.surfaces):
                if s.headset == "builtin" and not s.collides:
                    continue
                name = prim_name(f"s{k}", s.id)
                cube = UsdGeom.Cube.Define(stage, f"{self.ROOT}/Surfaces/{name}")
                cube.CreateSizeAttr(1.0)
                x = UsdGeom.Xformable(cube)
                x.AddTranslateOp().Set(Gf.Vec3d(*s.center))
                x.AddScaleOp().Set(Gf.Vec3f(*s.size))
                cube.CreateDisplayColorAttr([Gf.Vec3f(*s.colour)])
                if s.collides:
                    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                    # The robot's mount stands in the bench; a contact with a body no joint moves only fights the base's pin.
                    UsdPhysics.FilteredPairsAPI.Apply(cube.GetPrim()).CreateFilteredPairsRel().SetTargets([Sdf.Path(b) for b in fixed])
                    bind_physics(cube.GetPrim(), material(f"{self.ROOT}/Materials/{name}", s.static_friction, s.dynamic_friction))
            for o, path, pose in zip(scene.objects, self.object_paths, self.model.poses):
                body = UsdGeom.Xform.Define(stage, path)
                x = UsdGeom.Xformable(body)
                x.AddTranslateOp().Set(Gf.Vec3d(*pose[:3]))
                x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(pose[3], pose[4], pose[5], pose[6]))
                rb = UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
                rb.CreateKinematicEnabledAttr(True)
                sx, sy, sz = o.size
                if o.shape == "box":
                    g = UsdGeom.Cube.Define(stage, f"{path}/Shape")
                    g.CreateSizeAttr(1.0)
                    UsdGeom.Xformable(g).AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
                elif o.shape == "cylinder":
                    g = UsdGeom.Cylinder.Define(stage, f"{path}/Shape")
                    g.CreateRadiusAttr(sx / 2)
                    g.CreateHeightAttr(sz)
                    g.CreateAxisAttr(UsdGeom.Tokens.z)
                else:
                    g = UsdGeom.Sphere.Define(stage, f"{path}/Shape")
                    g.CreateRadiusAttr(sx / 2)
                g.CreateDisplayColorAttr([Gf.Vec3f(*o.colour)])
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                bind_physics(g.GetPrim(), material(f"{self.ROOT}/Materials/{path.rsplit('/', 1)[1]}", o.static_friction, o.dynamic_friction))

    def fixed_bodies(self, stage) -> List[str]:
        """The robot's rigid bodies no joint moves (its mount and what is bolted to it): those reached from the pinned base
        through fixed joints alone, in either direction. The base is the body a joint pins to the world (a joint with
        one side empty), or the articulation root's body."""
        from pxr import Usd, UsdPhysics

        robot = stage.GetPrimAtPath(self.robot_path)
        bodies, fixed_links, start = set(), {}, set()
        for prim in Usd.PrimRange(robot):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.add(str(prim.GetPath()))
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    start.add(str(prim.GetPath()))
            if prim.IsA(UsdPhysics.Joint):
                j = UsdPhysics.Joint(prim)
                b0 = [str(t) for t in j.GetBody0Rel().GetTargets()]
                b1 = [str(t) for t in j.GetBody1Rel().GetTargets()]
                if not b0 or not b1:
                    start.update(b0 + b1)
                elif prim.IsA(UsdPhysics.FixedJoint):
                    for a in b0:
                        for b in b1:
                            fixed_links.setdefault(a, []).append(b)
                            fixed_links.setdefault(b, []).append(a)
        seen, todo = set(), [b for b in start if b in bodies]
        while todo:
            b = todo.pop()
            if b not in seen:
                seen.add(b)
                todo.extend(fixed_links.get(b, []))
        return sorted(seen)

    # --- after play -------------------------------------------------------------------------------------------------

    def attach(self) -> None:
        import omni.physx
        from isaacsim.core.experimental.prims import RigidPrim

        self._bodies = RigidPrim(self.object_paths)
        self._query = omni.physx.get_physx_scene_query_interface()

    def arm_near(self, index: int, pose: Pose, margin: float) -> bool:
        """Whether any link of the robot is within ``margin`` of object ``index``'s shape at ``pose``. Boxes and
        cylinders are tested as their boxes, which errs towards refusing."""
        import carb

        o = self.model.scene.objects[index]
        robot = self.robot_path + "/"
        hit = []

        def report(h) -> bool:
            body = str(getattr(h, "rigid_body", "") or getattr(h, "collision", ""))
            if body.startswith(robot):
                hit.append(body)
                return False  # one is enough
            return True

        position = carb.Float3(pose[0], pose[1], pose[2])
        if o.shape == "sphere":
            self._query.overlap_sphere(o.size[0] / 2 + margin, position, report, False)
        else:
            half = carb.Float3(o.size[0] / 2 + margin, o.size[1] / 2 + margin, o.size[2] / 2 + margin)
            rotation = carb.Float4(pose[4], pose[5], pose[6], pose[3])  # x, y, z, w
            self._query.overlap_box(half, position, rotation, report, False)
        return bool(hit)

    def robot_on_surfaces(self) -> List[Tuple[str, List[str]]]:
        """Each colliding surface with the robot's bodies that overlap it now (a PhysX overlap query of its box)."""
        import carb

        robot = self.robot_path + "/"
        out = []
        for s in self.model.scene.surfaces:
            if not s.collides:
                continue
            hits: List[str] = []

            def report(h) -> bool:
                body = str(getattr(h, "rigid_body", "") or "")
                if body.startswith(robot) and body not in hits:
                    hits.append(body)
                return True

            self._query.overlap_box(carb.Float3(*(v / 2 for v in s.size)), carb.Float3(*s.center), carb.Float4(0, 0, 0, 1), report, False)
            out.append((s.id, hits))
        return out

    def place(self, cause: int, object_id: int, pose: Sequence[float]) -> Tuple[int, Pose]:
        """Applies or refuses one placement: (outcome, the object's pose now)."""
        import numpy as np

        outcome, i, upright = self.model.check(cause, object_id, pose, self.arm_near)
        if outcome == wire.APPLIED:
            self._bodies.set_world_poses(positions=np.array([upright[:3]]), orientations=np.array([upright[3:]]), indices=[i])
            self.model.apply(i, upright)
        now = self.model.poses[i] if i >= 0 else tuple(pose)
        return outcome, now

    def read(self) -> List[wire.ObjectPose]:
        """Every object's pose as PhysX has it (not USD's, which the bridge does not write back)."""
        import numpy as np

        positions, orientations = self._bodies.get_world_poses()
        p = np.asarray(positions.numpy() if hasattr(positions, "numpy") else positions).reshape(-1, 3)
        q = np.asarray(orientations.numpy() if hasattr(orientations, "numpy") else orientations).reshape(-1, 4)
        return [wire.ObjectPose(o.id, wire.FLAG_KINEMATIC, (*map(float, p[i]), *map(float, q[i])))
                for i, o in enumerate(self.model.scene.objects)]
