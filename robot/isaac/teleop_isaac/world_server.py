"""The world half of the arm-server wire, served the same way by both Isaac bridges (docs/adr/0031 amendment 2,
docs/adr/0034 amendment 6).

After each Welcome, which scene runs (WORLD_INFO); each PLACE and RESIZE applied or refused and answered (PLACED); the
objects' sizes at start, on a change and once a second (SIZES); and every ``world_every`` steps where the objects are
(WORLD). Every message also goes to the renderer's tap. The FR3 bridge serves it on its arm socket, sharing the arm's
sequence; the JetRover's on a socket of its own.

``world`` is an :class:`~teleop_isaac.world.IsaacWorld`, or None for an FR3 bridge started without ``--scene``: then
nothing is published, and every PLACE and RESIZE is answered UNKNOWN_OBJECT. No Isaac import here, so the tests run it
with a stand-in world.
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

from . import wire

NO_POSE = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)


class WorldServer:
    def __init__(self, world, scene_sha: bytes, object_ids: Sequence[int], sendto: Callable[[bytes, object], None],
                 next_sequence: Callable[[], int], tap: Callable[[bytes], None], steps_per_second: int, world_every: int,
                 tag: str, clock_ns: Callable[[], int] = time.monotonic_ns):
        self.world = world
        self.scene_sha = scene_sha
        self.object_ids = list(object_ids)
        self.sendto = sendto
        self.next_sequence = next_sequence
        self.tap = tap
        self.steps_per_second = steps_per_second
        self.world_every = world_every
        self.tag = tag
        self.clock_ns = clock_ns
        self.sizes_due = True
        self.last_sizes = 0

    def _send(self, message: bytes, client) -> None:
        if client is None:
            return
        try:
            self.sendto(message, client)
        except OSError:
            pass  # RobotHost went away; it says hello again when it is back

    def world_info(self) -> Optional[bytes]:
        """What follows a Welcome (None without a world); the sizes go out again with the next publish."""
        if self.world is None:
            return None
        self.sizes_due = True
        return wire.encode_world_info(self.next_sequence(), self.scene_sha, self.object_ids)

    def handle(self, kind: int, data: bytes, client) -> bool:
        """A PLACE or RESIZE from the client, applied or refused and answered; False for any other kind."""
        if kind == wire.PLACE:
            place = wire.decode_place(data)
            if place is not None:
                _, request, cause, object_id, pose = place
                if self.world is not None:
                    outcome, now_pose = self.world.place(cause, object_id, pose)
                    revision = self.world.model.revision
                else:
                    outcome, now_pose, revision = wire.UNKNOWN_OBJECT, tuple(pose), 0
                if outcome != wire.APPLIED or cause != wire.CAUSE_LAYOUT:
                    print(f"[{self.tag}] place {object_id} (cause {cause}): outcome {outcome}", flush=True)
                self._send(wire.encode_placed(self.next_sequence(), request, object_id, outcome, revision, now_pose), client)
            return True
        if kind == wire.RESIZE:
            rz = wire.decode_resize(data)
            if rz is not None:
                _, request, object_id, scale = rz
                if self.world is not None:
                    outcome, now_pose = self.world.resize(object_id, scale)
                    revision = self.world.model.revision
                else:
                    outcome, now_pose, revision = wire.UNKNOWN_OBJECT, NO_POSE, 0
                print(f"[{self.tag}] resize {object_id} to {scale:.3g}x: outcome {outcome}", flush=True)
                self.sizes_due = self.sizes_due or outcome == wire.APPLIED
                self._send(wire.encode_placed(self.next_sequence(), request, object_id, outcome, revision, now_pose), client)
            return True
        return False

    def publish(self, steps: int, client, tapping: bool) -> None:
        """After step ``steps``: the sizes when due, and the objects' poses every ``world_every`` steps."""
        if self.world is None or (client is None and not tapping):
            return
        if self.sizes_due or steps - self.last_sizes >= self.steps_per_second:
            self.sizes_due, self.last_sizes = False, steps
            message = self.world.model.sizes_message(self.next_sequence())
            self._send(message, client)
            self.tap(message)
        if steps % self.world_every == 0:
            objects = self.world.read()
            for message in self.world.model.world_messages(self.next_sequence, self.clock_ns(), objects):
                self._send(message, client)
                self.tap(message)
