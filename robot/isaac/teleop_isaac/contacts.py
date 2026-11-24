"""Which of the arm's links touch which of the scene's objects (docs/adr/0036 amendment 1), and the table (docs/adr/0038
decision 4): PhysX's contact report, as the arm-server wire's contacts message (``wire.encode_contacts``) that RobotHost
records as ``WC``.

PhysX reports contact events per pair of shapes: ``FOUND`` when two start touching, ``PERSIST`` while they do, ``LOST``
when they part. :class:`ContactTracker` keeps the pairs between a robot link and an object body that are touching now,
and answers by (link, object): one contact however many shapes of the two touch, its normal force the sum of theirs.
Given ``surfaces``, a robot link against one of the scene's surfaces is a contact too, as object :data:`TABLE` (0), which
the wire has always reserved for the table (the bench and the floor are reported as it: every surface the arm can stand
on). Every other pair (an object on the table or on another object) is not a contact here.

PhysX reports a pair from the moment the two shapes come within their contact offsets of each other (both shapes'
summed), before they touch (docs/adr/0036 amendment 4). So a pair counts as touching on a step only when it has a contact
point no more than :data:`TOUCH_SEPARATION_M` apart, or PhysX's solver pushed the two apart that step (an impulse); a
pair inside the offsets but further apart is near, and not a contact.

No Isaac import here, so the tests run it with made-up events; ``world.IsaacWorld.contact_events`` turns PhysX's
report into the events this takes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

FOUND, PERSIST, LOST = 0, 1, 2
UNKNOWN_LINK = 255  # a robot body that is not one of the URDF's links (it should not happen; it is still a contact)
JETROVER_ARM = 0  # the JetRover has one arm
TABLE = 0  # the object id of the table on the wire and in WC (docs/adr/0038): every scene surface the arm touches

# A touch (docs/adr/0036 amendment 4): a contact point at most this far apart (m), or any impulse.
TOUCH_SEPARATION_M = 0.001
# The colliders' offsets, set on the robot's links and on every object and surface when contacts are reported
# (world.IsaacWorld.build). PhysX generates a pair's contacts within the two shapes' contact offsets summed (10 mm here):
# that must exceed TOUCH_SEPARATION_M, or a touch would never be reported, and the most two bodies close in one 1 ms
# step (a link at 8 rad/s, 0.3 m out: 2.4 mm; a falling object about the same), or a fast one would pass into another
# before PhysX sees the pair. Rest offset 0, PhysX's own for rigid bodies, pinned: bodies at rest touch at separation 0,
# so the 1 mm means a 1 mm gap.
CONTACT_OFFSET_M = 0.005
REST_OFFSET_M = 0.0


@dataclass(frozen=True)
class ContactEvent:
    """One PhysX contact event between two shapes: the rigid bodies they belong to, the shapes themselves, what happened,
    the impulse PhysX applied between them over the step (N s), and the smallest separation of its contact points (m;
    negative when they overlap, infinite when the event carries none, as a ``LOST`` does)."""

    body0: str
    body1: str
    shape0: str
    shape1: str
    kind: int
    impulse: float
    separation: float = math.inf

    @property
    def touches(self) -> bool:
        """A contact point within :data:`TOUCH_SEPARATION_M`, or the solver pushed the shapes apart this step."""
        return self.kind != LOST and (self.separation <= TOUCH_SEPARATION_M or self.impulse > 0.0)


class ContactTracker:
    """The arm's contacts with the scene's objects, from PhysX's contact events step by step.

    ``robot_path`` is the robot's prim (its links are the bodies under it); ``links`` the URDF's link names in order (a
    contact's link is the index of the last path element that names one); ``objects`` each object body's prim path and
    the object id it stands for (a resizable object has several bodies, one per size, all its id); ``dt`` the physics
    step (s), to turn an impulse into a force; ``surfaces`` the prim paths of the scene's colliding surfaces, each a static
    collider whose own path PhysX reports as its body (matched on the shape's path too), reported as :data:`TABLE`."""

    def __init__(self, robot_path: str, links: Sequence[str], objects: Mapping[str, int], dt: float, arm: int = JETROVER_ARM,
                 surfaces: Iterable[str] = ()):
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.robot = robot_path.rstrip("/") + "/"
        self.links = {name: i for i, name in enumerate(links)}
        self.objects = dict(objects)
        self.surfaces = frozenset(surfaces)
        self.dt = dt
        self.arm = arm
        # (link, object, robot shape, object shape) -> force (N): the shape pairs touching now.
        self._pairs: Dict[Tuple[int, int, str, str], float] = {}
        self._last: Tuple[Tuple[int, int], ...] = ()
        # What the filter saw, for the bridge's report: robot-object pair events inside the offsets but not touching,
        # and the separations PhysX reported for robot-object pairs (the largest is about the contact offsets in use).
        self.near_events = 0
        self.max_separation = -math.inf
        self.min_separation = math.inf

    def _link_of(self, body: str) -> int:
        for part in reversed(body[len(self.robot):].split("/")):
            if part in self.links:
                return self.links[part]
        return UNKNOWN_LINK

    def _classify(self, e: ContactEvent):
        """(link, object, robot shape, object shape) for a robot-object or robot-surface pair (object :data:`TABLE`), else None."""
        for robot, other, robot_shape, other_shape in ((e.body0, e.body1, e.shape0, e.shape1), (e.body1, e.body0, e.shape1, e.shape0)):
            if not robot.startswith(self.robot) or other.startswith(self.robot):
                continue
            if other in self.objects:
                return self._link_of(robot), self.objects[other], robot_shape, other_shape
            if other in self.surfaces or other_shape in self.surfaces:
                return self._link_of(robot), TABLE, robot_shape, other_shape
        return None

    def update(self, events: Iterable[ContactEvent]) -> bool:
        """Applies one step's events; True when the set of touching (link, object) pairs changed."""
        for e in events:
            key = self._classify(e)
            if key is None:
                continue
            if math.isfinite(e.separation):
                self.max_separation = max(self.max_separation, e.separation)
                self.min_separation = min(self.min_separation, e.separation)
            if e.touches:
                self._pairs[key] = abs(e.impulse) / self.dt
            else:
                # LOST, or still reported but no longer (or not yet) touching: near is not a contact.
                self._pairs.pop(key, None)
                if e.kind != LOST:
                    self.near_events += 1
        now = self.touching()
        changed = now != self._last
        self._last = now
        return changed

    def forget(self, object_id: int) -> None:
        """Drops every contact with ``object_id``: after it is placed or resized elsewhere PhysX may never report them lost."""
        self._pairs = {k: f for k, f in self._pairs.items() if k[1] != object_id}

    def clear(self) -> None:
        self._pairs.clear()

    def touching(self) -> Tuple[Tuple[int, int], ...]:
        """The (link, object) pairs touching now, sorted."""
        return tuple(sorted({(k[0], k[1]) for k in self._pairs}))

    def contacts(self) -> List[Tuple[int, int, int, float]]:
        """The wire's contacts, (arm, link, object, normal force N), one per touching (link, object) pair, sorted."""
        force: Dict[Tuple[int, int], float] = {}
        for (link, obj, _, _), f in self._pairs.items():
            force[(link, obj)] = force.get((link, obj), 0.0) + f
        return [(self.arm, link, obj, force[(link, obj)]) for link, obj in sorted(force)]


def urdf_links(path: str) -> List[str]:
    """The URDF's link names, in the file's order: a contact's link number is its index here."""
    import xml.etree.ElementTree as ET

    return [link.get("name") for link in ET.parse(path).getroot().findall("link")]
