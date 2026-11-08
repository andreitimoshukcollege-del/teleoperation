"""The JetRover's arm in Isaac Sim, standing in for its ROS servo driver (docs/adr/0034 decision 1).

Run on the GPU machine with Isaac's Python (``just isaac-jetrover-host`` does it, J2):

    ~/isaacsim/venv/bin/python -m teleop_isaac.jetrover_bridge --socket /tmp/teleop-jetrover-relay.sock

It binds the Unix datagram socket the JetRover's RobotHost names with ``--relay-socket`` and speaks relay protocol v5, the ROS
relay's own module: targets in (pulse and pulses per second per motor), torque on or off, and feedback out to whoever sent the
last command (before any, to ``--client-socket``, RobotHost's own relay socket, as the real relay sends it). Every millisecond,
paced to the wall clock:

- each servo's own setpoint ramps toward its target over ``|delta| / pulses_per_second`` (``servo.TimedSetpoint``), and the
  joint's PhysX position drive follows it, no harder than the servo's stall torque (decision 3);
- the arm feels gravity, so it sags and slows under load, and with torque off the drives go limp and it falls, as the real
  one does when it relaxes;
- PhysX steps once.

Every 20 ms a cycle reads motors 1 to 5 back to back in whole pulses, as the real driver does, and every 20 ms the newest
reading of each goes back, valid while under 0.2 s old; the gripper (10) is never valid (decision 4, ``servo.FeedbackEmulator``). The base is fixed (decision 5).

With ``--scene`` (docs/adr/0034 decision 7, J4) the scene file's surfaces and objects are in the stage too (``world.py``, as
the FR3's), served to RobotHost on a second socket (``--world-socket``) over the arm-server wire's world messages: a Welcome
with no motors and WORLD_INFO after each Hello, WORLD every 20 ms from PhysX, PLACE applied or refused. ``--tap`` copies the
arm's state (an arm-server STATE of joint values) and every WORLD to the renderer.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import signal
import socket
import sys
import time
import traceback
from typing import Dict, List, Optional

from . import scene as scene_module
from . import servo
from . import wire

HOME = (0.0, 1.0472, -1.5708, 0.5236, 0.0)  # JetRoverCollisionModel's home: yaw, proximal, distal, wrist, roll


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    a = argparse.ArgumentParser(description="The JetRover's arm in Isaac Sim, behind its own RobotHost (docs/adr/0034)")
    a.add_argument("--sim", default=servo.DEFAULT_SIM, help="the sidecar (robot/jetrover/urdf/jetrover.sim.json)")
    a.add_argument("--socket", default="/tmp/teleop-jetrover-relay.sock", help="the relay socket RobotHost's --relay-socket names")
    a.add_argument("--client-socket", default="", help="RobotHost's --local-relay-socket, where feedback goes every 50 ms whether or not "
                                                     "commands come, as the real relay sends it (default: --socket plus .robothost)")
    a.add_argument("--work", default=os.path.expanduser("~/isaacsim/work"), help="where the URDF's USD conversion is kept")
    a.add_argument("--stiffness", type=float, default=60.0, help="joint drive stiffness, Nm/rad: a servo holding its setpoint")
    a.add_argument("--damping", type=float, default=1.5, help="joint drive damping, Nm s/rad")
    a.add_argument("--finger-stiffness", type=float, default=2000.0, help="finger drive stiffness, N/m")
    a.add_argument("--finger-damping", type=float, default=40.0, help="finger drive damping, N s/m")
    a.add_argument("--limp-damping", type=float, default=0.02, help="joint damping with torque off (Nm s/rad): a relaxed servo's gearbox")
    a.add_argument("--iterations", type=int, default=16, help="the articulation solver's position iterations")
    a.add_argument("--dt", type=float, default=0.001, help="the physics step, s")
    a.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0: until stopped)")
    a.add_argument("--torque-off-at-start", action="store_true", help="start relaxed, as the robot does after ROBOT OFF")
    a.add_argument("--scene", default="", help="a scene file (core/Scenes/*.json) for the robot's bench: its surfaces and objects")
    a.add_argument("--world-socket", default="", help="where RobotHost's world client talks (default: --socket plus .world)")
    a.add_argument("--world-every", type=int, default=20, help="send the objects' poses every this many steps")
    a.add_argument("--tap", default="", help="the renderer's socket: a copy of the arm's state every 4 steps and of every WORLD")
    a.add_argument("--gui", action="store_true", help="open Isaac's window (needs a display)")
    return a.parse_args(argv)


def convert_urdf(urdf: str, work: str) -> str:
    """The URDF as Isaac's USD, converted once per URDF content (its sha256 names the folder)."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    with open(urdf, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()[:12]
    out = os.path.join(work, f"jetrover-{digest}")
    name = os.path.splitext(os.path.basename(urdf))[0]
    usd = os.path.join(out, name, f"{name}.usda")
    if not os.path.isfile(usd):
        os.makedirs(out, exist_ok=True)
        usd = URDFImporter(URDFImporterConfig(urdf_path=urdf, usd_path=out, fix_base=True,
                                              joint_drive_type="force", joint_target_type="position")).import_urdf()
    return usd


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    sys.argv = sys.argv[:1]  # Kit reads the command line too; it gets none of ours
    model = servo.SimModel.load(args.sim)
    rp = servo.relay_protocol()
    urdf = os.path.join(os.path.dirname(os.path.abspath(args.sim)), model.urdf)
    pulses = servo.home_pulses(model, HOME)
    setpoints: Dict[int, servo.TimedSetpoint] = {m: servo.TimedSetpoint(p) for m, p in pulses.items()}
    feedback = servo.FeedbackEmulator(model)
    scene = scene_module.load(args.scene) if args.scene else None
    if scene is not None and scene.robot_profile != "jetrover":
        raise SystemExit(f"error: the scene {args.scene} is for profile {scene.robot_profile!r}, not 'jetrover'")
    scene_sha = __import__("hashlib").sha256(open(args.scene, "rb").read()).digest() if scene is not None else b""

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

        usd = convert_urdf(urdf, args.work)
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
        # Gravity stays on for the arm (decision 3): the real servos hold it up, and sag under load.
        from .world import IsaacWorld

        world = IsaacWorld(scene, str(robot.GetPath())) if scene is not None else None
        if world is not None:
            world.build(stage)
            fixed = world.fixed_bodies(stage)
            print(f"[jetrover] the robot's fixed bodies ignore the surfaces: {', '.join(b.rsplit('/', 1)[-1] for b in fixed)}", flush=True)

        settings = carb.settings.get_settings()
        for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/updateParticlesToUsd", "/physics/updateResidualsToUsd"):
            settings.set(key, False)
        SimulationManager.setup_simulation(dt=args.dt, device="cpu")
        app_utils.play()
        app.update()

        art = Articulation(roots[0])
        names = list(art.dof_names)
        index = {n: i for i, n in enumerate(names)}
        missing = [m.joint for m in model.motors.values() if not m.is_gripper and m.joint not in index]
        missing += [f for m in model.motors.values() for f in m.fingers if f not in index]
        if missing:
            raise RuntimeError(f"Isaac's articulation lacks the sidecar's joints: {', '.join(missing)} (has {', '.join(names)})")
        n = len(names)
        finger = np.array([("finger" in nm or nm.startswith("gripper")) for nm in names])
        kp_on = np.where(finger, args.finger_stiffness, args.stiffness).reshape(1, -1)
        kd_on = np.where(finger, args.finger_damping, args.damping).reshape(1, -1)
        kp_off = np.zeros((1, n))
        kd_off = np.where(finger, args.finger_damping, args.limp_damping).reshape(1, -1)
        efforts = np.asarray(art.get_dof_max_efforts().numpy()).reshape(-1).copy()
        for m in model.motors.values():
            if not m.is_gripper:
                efforts[index[m.joint]] = m.stall_torque
        art.set_dof_max_efforts(efforts.reshape(1, -1))
        if world is not None:
            world.attach()

        target = np.zeros((1, n))

        def apply_setpoints() -> None:
            for mid, sp in setpoints.items():
                m = model.motors[mid]
                v = model.value(mid, sp.pulse)
                if m.is_gripper:
                    for f in m.fingers:
                        target[0, index[f]] = v
                else:
                    target[0, index[m.joint]] = v

        apply_setpoints()
        art.set_dof_positions(target)
        art.set_dof_position_targets(target)
        torque = not args.torque_off_at_start
        art.set_dof_gains(kp_on if torque else kp_off, kd_on if torque else kd_off)

        description = (f"Isaac Sim {version('isaacsim')}, PhysX {1 / args.dt:.0f} Hz on CPU, servo drives {args.stiffness:g}/{args.damping:g} "
                       f"within {'/'.join(f'{m.stall_torque:g}' for m in model.motors.values() if not m.is_gripper)} Nm, arm gravity on, "
                       f"timed-ramp setpoints, feedback motors {','.join(str(x) for x in feedback.order)} read back to back ({feedback.read_seconds * 1000:.1f} ms each) "
                       f"every {feedback.cycle_seconds * 1000:.0f} ms, sent every {model.feedback_seconds * 1000:.0f} ms")
        if os.path.exists(args.socket):
            os.unlink(args.socket)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.bind(args.socket)
        sock.setblocking(False)
        wsock = None
        world_path = args.world_socket or args.socket + ".world"
        if world is not None:
            movable = sum(1 for o in scene.objects if o.movable)
            world_description = (f"Isaac Sim {version('isaacsim')}, the JetRover's world: scene {scene.name} ({scene_sha.hex()[:12]}, "
                                 f"{len(scene.objects) - movable} fixed and {movable} movable objects)")[:wire.MAX_DESCRIPTION_BYTES]
            description += f", scene {scene.name} ({scene_sha.hex()[:12]}, {len(scene.objects) - movable} fixed and {movable} movable objects)"
            if os.path.exists(world_path):
                os.unlink(world_path)
            wsock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            wsock.bind(world_path)
            wsock.setblocking(False)
        print(f"[jetrover] {description}; {n} DOFs ({', '.join(names)}); torque {'on' if torque else 'off'}; listening on {args.socket}"
              + (f", world on {world_path}" if wsock is not None else ""), flush=True)
        world_client = None
        wseq = [0]

        def next_wseq() -> int:
            wseq[0] += 1
            return wseq[0]

        tapped = tap_dropped = 0

        def tap(message: bytes) -> None:
            nonlocal tapped, tap_dropped
            if not args.tap:
                return
            try:
                sock.sendto(message, args.tap)
                tapped += 1
            except OSError:
                tap_dropped += 1

        motor_order = sorted(model.motors)

        stopping = []
        signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
        signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
        client = args.client_socket or args.socket + ".robothost"
        commands = torques = rejected_total = sent = 0
        steps = late = 0
        step_ns = int(round(args.dt * 1e9))
        start_ns = next_tick = time.monotonic_ns()
        stop_at = start_ns + int(args.seconds * 1e9) if args.seconds > 0 else None
        report_at = start_ns + 5_000_000_000

        while not stopping and (stop_at is None or time.monotonic_ns() < stop_at):
            batch = []
            while True:
                try:
                    data, sender = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                batch.append(data)
                if sender:
                    client = sender  # whoever commands is where feedback goes
            if batch:
                actions, rejected = rp.sort_datagrams(batch)
                rejected_total += rejected
                for kind, payload in actions:
                    if kind == "torque":
                        torques += 1
                        if payload != torque:
                            torque = payload
                            if torque:
                                # Back on, a servo holds where the arm lies now.
                                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                                for mid, sp in setpoints.items():
                                    m = model.motors[mid]
                                    sp.hold(model.pulse(mid, q[index[m.fingers[0] if m.is_gripper else m.joint]]))
                            art.set_dof_gains(kp_on if torque else kp_off, kd_on if torque else kd_off)
                            print(f"[jetrover] torque {'on' if torque else 'off'}", flush=True)
                    else:
                        commands += 1
                        for mid, pulse, pps in payload:
                            if mid in setpoints:
                                setpoints[mid].move(model.clamp(pulse), pps)

            while wsock is not None:
                try:
                    data, sender = wsock.recvfrom(wire.MAX_BYTES)
                except BlockingIOError:
                    break
                kind = wire.kind_of(data)
                if kind == wire.HELLO and wire.decode_hello(data) is not None:
                    hello = wire.decode_hello(data)
                    if hello.profile != "jetrover":
                        print(f"[jetrover] a world hello for profile {hello.profile!r}: not answered", flush=True)
                        continue
                    if sender != world_client:
                        print(f"[jetrover] RobotHost's world client at {sender}", flush=True)
                    world_client = sender
                    try:
                        wsock.sendto(wire.encode_welcome(next_wseq(), wire.SERVER_ISAAC, time.monotonic_ns(), [], world_description), world_client)
                        wsock.sendto(wire.encode_world_info(next_wseq(), scene_sha, [o.id for o in scene.objects]), world_client)
                    except OSError:
                        world_client = None
                elif kind == wire.PLACE and sender == world_client:
                    place = wire.decode_place(data)
                    if place is not None:
                        _, request, cause, object_id, pose = place
                        outcome, now_pose = world.place(cause, object_id, pose)
                        if outcome != wire.APPLIED or cause != wire.CAUSE_LAYOUT:
                            print(f"[jetrover] place {object_id} (cause {cause}): outcome {outcome}", flush=True)
                        try:
                            wsock.sendto(wire.encode_placed(next_wseq(), request, object_id, outcome, world.model.revision, now_pose), world_client)
                        except OSError:
                            pass

            seconds = args.dt
            for sp in setpoints.values():
                sp.step(seconds)
            if torque:
                apply_setpoints()
                art.set_dof_position_targets(target)
            SimulationManager.step(steps=1)
            steps += 1
            now_s = steps * args.dt

            if args.tap and steps % 4 == 0:
                # The renderer poses the arm from these: every motor's joint value (radians; the gripper's opening 0..1).
                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                report = []
                for m_id in motor_order:
                    m = model.motors[m_id]
                    if m.is_gripper:
                        v = q[index[m.fingers[0]]] / m.finger_travel if m.finger_travel else 0.0
                    else:
                        v = float(q[index[m.joint]])
                    report.append((m_id, float(v), 0.0, float(v)))
                tap(wire.encode_state(next_wseq(), time.monotonic_ns(), steps * step_ns, wire.FLAG_ENABLED if torque else 0, report))
            if world is not None and (world_client is not None or args.tap) and steps % args.world_every == 0:
                objects = world.read()
                for message in world.model.world_messages(next_wseq, time.monotonic_ns(), objects):
                    if world_client is not None:
                        try:
                            wsock.sendto(message, world_client)
                        except OSError:
                            pass
                    tap(message)

            mid = feedback.due_read(now_s)
            if mid is not None:
                q = np.asarray(art.get_dof_positions().numpy()).reshape(-1)
                feedback.record(mid, model.pulse(mid, q[index[model.motors[mid].joint]]), now_s)
            entries = feedback.due_feedback(now_s)
            if entries is not None:
                try:
                    sock.sendto(rp.encode_feedback(entries), client)
                    sent += 1
                except OSError:
                    pass  # RobotHost is not there (yet): the real relay's feedback is lost the same way

            next_tick += step_ns
            now = time.monotonic_ns()
            behind = now - next_tick
            if behind > step_ns:
                late += 1
                if behind > 50 * step_ns:
                    next_tick = now
            else:
                while next_tick - time.monotonic_ns() > 300_000:
                    time.sleep(0.0002)
                while time.monotonic_ns() < next_tick:
                    pass
            if now >= report_at:
                report_at = now + 5_000_000_000
                print(f"[jetrover] {steps} steps, {late} late; {commands} commands, {torques} torque switches, {rejected_total} rejected; "
                      f"{sent} feedback sent; torque {'on' if torque else 'off'}", flush=True)

        sock.close()
        os.unlink(args.socket)
        if wsock is not None:
            wsock.close()
            os.unlink(world_path)
        print(f"[jetrover] stopped after {steps} steps, {late} late; {commands} commands, {sent} feedback sent", flush=True)
        return 0
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
