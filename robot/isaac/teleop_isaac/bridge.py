"""The Isaac bridge (docs/adr/0031 decision 2): the FR3 Duo in Isaac Sim, behind RobotHost over the arm-server wire.

Run on the GPU machine with Isaac's Python (``just isaac-host`` does it):

    ~/isaacsim/venv/bin/python -m teleop_isaac.bridge --socket /tmp/teleop-isaac.sock

Every millisecond, paced to the wall clock: take RobotHost's newest targets, step the golden-pinned shaper once per
motor, give its output to the joints' position drives, and step PhysX once. Every 4 ms, the arm's measured state goes
back. About once a second, how well the loop keeps up.

What stands in for the FR3 (docs/adr/0031 decision 2), each a parameter written into the Welcome's description:
- the joints' position drives (stiffness, damping) play Franka's internal controller;
- the arms' own links feel no gravity, because the real FR3 compensates its own weight; objects will;
- PhysX steps at 1 kHz with no results written back to USD, which is what makes 1 kHz affordable.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

from . import profile as profile_module
from . import wire
from .shaper import ShapedState, shaper_for_profile, step

READY_POSE = (0.0, -0.785398163397448, 0.0, -2.356194490192345, 0.0, 1.570796326794897, 0.785398163397448)


def start_pose(p: profile_module.Profile) -> List[float]:
    """RobotHost's ``SerialChainHost.StartPose``: Franka's ready pose for each arm, mirrored for every arm after the
    first (joints 1, 3, 5 and 7 negated), clamped into the soft limits; hands open. Profile order."""
    out: List[float] = []
    for a, arm in enumerate(p.arms):
        for j, joint in enumerate(arm.joints):
            q = READY_POSE[j] if j < len(READY_POSE) else 0.0
            if a % 2 == 1 and j % 2 == 0:
                q = -q
            out.append(min(max(q, joint.soft_lower), joint.soft_upper))
        if arm.gripper is not None:
            out.append(1.0)
    return out


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))
    a = argparse.ArgumentParser(description="The FR3 Duo in Isaac Sim, behind RobotHost (docs/adr/0031)")
    a.add_argument("--profile", default=os.path.join(repo, "core", "RobotProfiles", "fr3-duo.json"))
    a.add_argument("--urdf", default=os.path.join(repo, "robot", "franka", "fr3_duo", "urdf", "fr3_duo.urdf"))
    a.add_argument("--socket", default="/tmp/teleop-isaac.sock", help="the Unix datagram socket RobotHost talks to")
    a.add_argument("--work", default=os.path.expanduser("~/isaacsim/work"), help="where the URDF's USD conversion is kept")
    a.add_argument("--stiffness", type=float, default=16000.0, help="arm joint drive stiffness, Nm/rad")
    a.add_argument("--damping", type=float, default=250.0, help="arm joint drive damping, Nm s/rad")
    a.add_argument("--finger-stiffness", type=float, default=2000.0, help="finger drive stiffness, N/m")
    a.add_argument("--finger-damping", type=float, default=100.0, help="finger drive damping, N s/m")
    a.add_argument("--iterations", type=int, default=8, help="the articulation solver's position iterations")
    a.add_argument("--state-every", type=int, default=4, help="send the arm's state every this many 1 ms steps")
    a.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: until stopped)")
    a.add_argument("--gui", action="store_true", help="open Isaac's window (needs a display)")
    return a.parse_args(argv)


def convert_urdf(urdf: str, work: str) -> str:
    """The URDF as Isaac's USD, converted once per URDF content (its sha256 names the folder)."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    with open(urdf, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()[:12]
    out = os.path.join(work, f"fr3_duo-{digest}")
    name = os.path.splitext(os.path.basename(urdf))[0]
    usd = os.path.join(out, name, f"{name}.usda")
    if not os.path.isfile(usd):
        os.makedirs(out, exist_ok=True)
        usd = URDFImporter(URDFImporterConfig(urdf_path=urdf, usd_path=out, fix_base=True,
                                              joint_drive_type="force", joint_target_type="position")).import_urdf()
    return usd


class DofMap:
    """Where each profile motor lives in Isaac's DOF list: an arm joint by name, a hand by both its fingers."""

    def __init__(self, p: profile_module.Profile, dof_names: List[str]):
        index = {n: i for i, n in enumerate(dof_names)}
        self.joint_dof: Dict[int, int] = {}
        self.finger_dofs: Dict[int, Tuple[int, ...]] = {}
        self.finger_travel: Dict[int, float] = {}
        missing = []
        for arm in p.arms:
            for j in arm.joints:
                if j.name in index:
                    self.joint_dof[j.motor_id] = index[j.name]
                else:
                    missing.append(j.name)
            if arm.gripper is not None:
                g = arm.gripper
                second = g.name[:-1] + "2" if g.name.endswith("1") else None
                fingers = [index[n] for n in (g.name, second) if n is not None and n in index]
                if g.name not in index:
                    missing.append(g.name)
                self.finger_dofs[g.motor_id] = tuple(fingers)
                self.finger_travel[g.motor_id] = g.finger_travel
        if missing:
            raise RuntimeError(f"Isaac's articulation lacks the profile's joints: {', '.join(missing)}")


def check_limits(p: profile_module.Profile, dofs: DofMap, lower: List[float], upper: List[float], tolerance: float = 1e-3) -> None:
    """The import's joint limits are the profile's, or the bridge refuses to start (docs/adr/0031 decision 2)."""
    wrong = []
    for arm in p.arms:
        for j in arm.joints:
            i = dofs.joint_dof[j.motor_id]
            if abs(lower[i] - j.lower) > tolerance or abs(upper[i] - j.upper) > tolerance:
                wrong.append(f"{j.name} [{lower[i]:.4f}, {upper[i]:.4f}] against the profile's [{j.lower:.4f}, {j.upper:.4f}]")
    if wrong:
        raise RuntimeError("Isaac's joint limits are not the profile's: " + "; ".join(wrong))


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    sys.argv = sys.argv[:1]  # Kit reads the command line too; it gets none of ours

    p = profile_module.load(args.profile)
    shaper = shaper_for_profile(p)
    motor_ids = p.motor_ids()
    limits = [shaper.joints[shaper.index_of(m)] for m in motor_ids]
    start = start_pose(p)
    states = [ShapedState.at_rest(q) for q in start]
    targets = list(start)

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    try:
        import carb
        import numpy as np
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from importlib.metadata import version
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import PhysxSchema, UsdPhysics

        usd = convert_urdf(args.urdf, args.work)
        stage_utils.open_stage(usd)
        while stage_utils.is_stage_loading():
            app.update()
        stage = stage_utils.get_current_stage()
        robot = stage.GetDefaultPrim()
        robot.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
        for _ in range(3):
            app.update()
        roots = [str(x.GetPath()) for x in stage.Traverse() if x.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError(f"expected one articulation in {usd}, found {roots}")
        art_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(roots[0]))
        art_api.CreateSolverPositionIterationCountAttr(args.iterations)
        art_api.CreateSolverVelocityIterationCountAttr(1)
        art_api.CreateEnabledSelfCollisionsAttr(False)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)

        settings = carb.settings.get_settings()
        for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateResidualsToUsd"):
            settings.set(key, False)
        dt = shaper.step_seconds
        SimulationManager.setup_simulation(dt=dt, device="cpu")
        app_utils.play()
        app.update()

        art = Articulation(roots[0])
        names = list(art.dof_names)
        dofs = DofMap(p, names)
        lo, hi = (np.asarray(x.numpy()).reshape(-1).tolist() for x in art.get_dof_limits())
        check_limits(p, dofs, lo, hi)
        n = len(names)
        kp = np.array([[args.finger_stiffness if "finger" in nm else args.stiffness for nm in names]])
        kd = np.array([[args.finger_damping if "finger" in nm else args.damping for nm in names]])
        art.set_dof_gains(kp, kd)

        dof_target = np.zeros((1, n))

        def apply_targets() -> None:
            for k, m in enumerate(motor_ids):
                if m in dofs.joint_dof:
                    dof_target[0, dofs.joint_dof[m]] = states[k].q
                else:
                    width = states[k].q * dofs.finger_travel[m]
                    for i in dofs.finger_dofs[m]:
                        dof_target[0, i] = width

        apply_targets()
        art.set_dof_positions(dof_target)
        art.set_dof_position_targets(dof_target)

        description = (f"Isaac Sim {version('isaacsim')}, PhysX {1 / dt:.0f} Hz on CPU, drives {args.stiffness:g}/{args.damping:g} "
                       f"(fingers {args.finger_stiffness:g}/{args.finger_damping:g}), robot gravity off, {args.iterations} solver iterations")
        if os.path.exists(args.socket):
            os.unlink(args.socket)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.bind(args.socket)
        sock.setblocking(False)
        print(f"[isaac] {description}; {n} DOFs; listening on {args.socket}", flush=True)

        client = None
        enabled = True
        sequence = 0
        steps = late = 0
        lag_ms = 0.0
        step_ns = int(round(dt * 1e9))
        next_tick = time.monotonic_ns()
        window_start, window_steps = next_tick, 0
        stop_at = next_tick + int(args.seconds * 1e9) if args.seconds > 0 else None

        while stop_at is None or time.monotonic_ns() < stop_at:
            while True:
                try:
                    data, sender = sock.recvfrom(wire.MAX_BYTES)
                except BlockingIOError:
                    break
                kind = wire.kind_of(data)
                if kind == wire.HELLO and wire.decode_hello(data) is not None:
                    hello = wire.decode_hello(data)
                    if hello.profile != p.name:
                        print(f"[isaac] a hello for profile {hello.profile!r}, not {p.name!r}: not answered", flush=True)
                        continue
                    if sender != client:
                        print(f"[isaac] RobotHost at {sender}", flush=True)
                    client = sender
                    sequence += 1
                    try:
                        sock.sendto(wire.encode_welcome(sequence, wire.SERVER_ISAAC, time.monotonic_ns(), motor_ids, description), client)
                    except OSError:
                        client = None  # it left before the answer; it says hello again
                elif kind == wire.TARGETS and sender == client:
                    t = wire.decode_targets(data)
                    if t is not None and enabled:
                        for m, v in t.targets:
                            if m in motor_ids:
                                targets[motor_ids.index(m)] = v
                elif kind == wire.ENABLE and sender == client:
                    e = wire.decode_enable(data)
                    if e is not None:
                        if not e.enabled:
                            targets = [s.q for s in states]  # come to rest where the shaper has the arm
                        enabled = e.enabled

            for k in range(len(states)):
                step(states[k], targets[k], limits[k], shaper)
            apply_targets()
            art.set_dof_position_targets(dof_target)
            SimulationManager.step(steps=1)
            steps += 1
            window_steps += 1

            if client is not None and steps % args.state_every == 0:
                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                v = np.asarray(art.get_dof_velocities().numpy()).reshape(-1)
                sensed = time.monotonic_ns()
                report = []
                for k, m in enumerate(motor_ids):
                    if m in dofs.joint_dof:
                        i = dofs.joint_dof[m]
                        report.append((m, float(q[i]), float(v[i]), states[k].q))
                    else:
                        i = dofs.finger_dofs[m][0]
                        travel = dofs.finger_travel[m]
                        report.append((m, float(q[i]) / travel, float(v[i]) / travel, states[k].q))
                flags = (wire.FLAG_ENABLED if enabled else 0) | (wire.FLAG_BEHIND if lag_ms > 1.0 else 0)
                sequence += 1
                try:
                    sock.sendto(wire.encode_state(sequence, sensed, steps * step_ns, flags, report), client)
                except OSError:
                    pass  # RobotHost went away; it says hello again when it is back

            next_tick += step_ns
            now = time.monotonic_ns()
            behind = now - next_tick
            if behind > step_ns:
                late += 1
                if behind > 50 * step_ns:  # too far behind to catch up: count it and start again from now
                    lag_ms += behind / 1e6
                    next_tick = now
            else:
                while next_tick - time.monotonic_ns() > 300_000:
                    time.sleep(0.0002)
                while time.monotonic_ns() < next_tick:
                    pass

            if now - window_start >= 1_000_000_000:
                factor = window_steps * dt / ((now - window_start) / 1e9)
                if client is not None:
                    sequence += 1
                    try:
                        sock.sendto(wire.encode_health(sequence, factor, lag_ms, late, steps), client)
                    except OSError:
                        pass
                window_start, window_steps = now, 0

        sock.close()
        os.unlink(args.socket)
        print(f"[isaac] stopped after {steps} steps, {late} late", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
