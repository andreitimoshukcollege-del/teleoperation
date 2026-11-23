"""Each take's look (docs/adr/0038 decision 5, workstream A): the scanned objects' colours, the light, and a small jitter of
every camera's pose, drawn again at every change of the room (each take is a session, and the renderer changes the room
after each), so a model has to use the scene's geometry rather than memorised colours or exact pixel positions. All of it is
recorded (``OA``, ``LT``, ``CJ``, and ``CM`` with the pose each camera actually had).

The scene file's optional ``appearance`` section turns it on. **A scene without one renders exactly as before.** Its parts,
each optional (docs/adr/0038 "A, appearance" has a complete example):

- ``objects``: every scanned object (``shape`` ``mesh``) has its own colour varied, within bounds that keep it the object it
  is. ``brightness`` [lo, hi] multiplies its albedo; ``desaturation`` [lo, hi] mixes the albedo toward its own luminance;
  ``tint`` (a strength, at most this) gives it a slight colour cast of a random hue, at unit luminance, so the cast moves
  the hue and not the brightness. Simple shapes keep the looks the scene's ``materials`` give them.
- ``lighting``:
  - ``colourTemperatureK`` [lo, hi]: the colour temperature of every light the scene file lists (the HDRI dome);
  - ``exposure`` [lo, hi]: the brightness (0-255) the renderer's metering aims for, instead of the environment's
    ``targetBrightness``;
  - ``key``: a distant light added to the scene. ``azimuthDeg`` and ``elevationDeg`` say where it shines from, in the robot's
    base frame (azimuth 0 is +x, beyond the table; 90 is +y, the robot's left); ``intensity`` is a multiple of the
    environment's ``meanRadiance`` (of the first dome's intensity in a scene without an environment); ``angleDeg`` is its
    angular size, the softness of its shadows; ``colourTemperatureK`` its own colour.
- ``cameras``: ``room`` (the cameras fixed in the room) and ``mounted`` (those on a link: the wrist), each ``translationM``
  and ``rotationDeg``. Each take, every camera is moved by up to ``translationM`` along each of its own axes and turned by up
  to ``rotationDeg`` about each of them: x (tilt), then y (pan), then z (roll).

**Every draw is a pure function** of the seed (``BK``'s), the change and what it is for, each from its own
``random.Random`` key, so adding a draw never moves another. Values are rounded before they are applied, so a record holds
exactly what was applied.

The functions at the end change a scanned object's materials on the renderer's USD stage (they import pxr when called):
an OmniPBR material through its own albedo inputs, a MaterialX one through two nodes put in front of its base colour, a
UsdPreviewSurface through its colour texture's scale. Each object's materials are changed in the network the renderer uses,
the first of Kit's render contexts (``mdl``, ``mtlx``, then the universal one) that the material has.
"""

from __future__ import annotations

import colorsys
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from . import kinematics

SECTION = "appearance"
LUMA = (0.2126, 0.7152, 0.0722)  # Rec. 709, linear light: the tint keeps this luminance at 1
DEFAULT_RENDER_CONTEXTS = ("mdl", "mtlx", "")  # Kit's /persistent/app/hydra/material/renderContexts

# What a value may be, so a section stays realistic: (lowest, highest).
LIMITS = {
    "brightness": (0.5, 1.5),
    "desaturation": (0.0, 1.0),
    "tint": (0.0, 0.3),
    "colourTemperatureK": (1500.0, 15000.0),
    "exposure": (1.0, 254.0),
    "intensity": (0.0, 20.0),
    "azimuthDeg": (-360.0, 720.0),
    "elevationDeg": (5.0, 89.0),
    "angleDeg": (0.0, 30.0),
    "translationM": (0.0, 0.05),
    "rotationDeg": (0.0, 10.0),
}


# --- the section ---------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class ObjectsSpec:
    brightness: Tuple[float, float]
    desaturation: Tuple[float, float]
    tint: float


@dataclass(frozen=True)
class KeySpec:
    intensity: Tuple[float, float]
    azimuth_deg: Tuple[float, float]
    elevation_deg: Tuple[float, float]
    angle_deg: Tuple[float, float]
    kelvin: Optional[Tuple[float, float]]


@dataclass(frozen=True)
class LightingSpec:
    kelvin: Optional[Tuple[float, float]]
    exposure: Optional[Tuple[float, float]]
    key: Optional[KeySpec]


@dataclass(frozen=True)
class JitterSpec:
    translation_m: float
    rotation_deg: float


@dataclass(frozen=True)
class Spec:
    objects: Optional[ObjectsSpec]
    lighting: Optional[LightingSpec]
    room: Optional[JitterSpec]
    mounted: Optional[JitterSpec]

    def jitter_for(self, mounted: bool) -> Optional[JitterSpec]:
        return self.mounted if mounted else self.room


def _number(v, where: str, name: str) -> float:
    if not (isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)):
        raise ValueError(f"appearance {where}: {name} is a number")
    lo, hi = LIMITS[name]
    if not lo <= float(v) <= hi:
        raise ValueError(f"appearance {where}: {name} {v} is outside {lo}..{hi}")
    return float(v)


def _range(d: Mapping, where: str, name: str, required: bool = True) -> Optional[Tuple[float, float]]:
    if name not in d:
        if required:
            raise ValueError(f"appearance {where}: {name} [low, high] is missing")
        return None
    v = d[name]
    if not (isinstance(v, list) and len(v) == 2):
        raise ValueError(f"appearance {where}: {name} is [low, high]")
    lo, hi = _number(v[0], where, name), _number(v[1], where, name)
    if lo > hi:
        raise ValueError(f"appearance {where}: {name} [low, high] has its low above its high")
    if name == "azimuthDeg" and hi - lo > 360.0:
        raise ValueError(f"appearance {where}: azimuthDeg spans more than 360 degrees")
    return lo, hi


def _object(d, where: str, fields: Sequence[str]) -> Mapping:
    if not isinstance(d, dict):
        raise ValueError(f"appearance {where}: an object")
    unknown = sorted(set(d) - set(fields))
    if unknown:
        raise ValueError(f"appearance {where}: no field {', '.join(unknown)}")
    return d


def _jitter(d, where: str) -> JitterSpec:
    d = _object(d, where, ("translationM", "rotationDeg"))
    return JitterSpec(_number(d.get("translationM", 0.0), where, "translationM"), _number(d.get("rotationDeg", 0.0), where, "rotationDeg"))


def parse(section) -> Spec:
    """The ``appearance`` section, checked (every field known, every value within :data:`LIMITS`)."""
    s = _object(section, "", ("objects", "lighting", "cameras"))
    objects = lighting = room = mounted = None
    if "objects" in s:
        o = _object(s["objects"], "objects", ("brightness", "desaturation", "tint"))
        objects = ObjectsSpec(_range(o, "objects", "brightness", False) or (1.0, 1.0),
                              _range(o, "objects", "desaturation", False) or (0.0, 0.0),
                              _number(o.get("tint", 0.0), "objects", "tint"))
    if "lighting" in s:
        lt = _object(s["lighting"], "lighting", ("colourTemperatureK", "exposure", "key"))
        key = None
        if "key" in lt:
            k = _object(lt["key"], "lighting key", ("intensity", "azimuthDeg", "elevationDeg", "angleDeg", "colourTemperatureK"))
            key = KeySpec(_range(k, "lighting key", "intensity"), _range(k, "lighting key", "azimuthDeg"),
                          _range(k, "lighting key", "elevationDeg"), _range(k, "lighting key", "angleDeg", False) or (0.53, 0.53),
                          _range(k, "lighting key", "colourTemperatureK", False))
        lighting = LightingSpec(_range(lt, "lighting", "colourTemperatureK", False), _range(lt, "lighting", "exposure", False), key)
    if "cameras" in s:
        c = _object(s["cameras"], "cameras", ("room", "mounted"))
        room = _jitter(c["room"], "cameras room") if "room" in c else None
        mounted = _jitter(c["mounted"], "cameras mounted") if "mounted" in c else None
    return Spec(objects, lighting, room, mounted)


def spec(scene) -> Optional[Spec]:
    """The scene file's ``appearance`` section, checked, or None when it has none (then nothing here runs)."""
    section = scene.extra.get(SECTION)
    return None if section is None else parse(section)


# --- the draws -----------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class ObjectColour:
    brightness: float
    desaturation: float
    tint: Tuple[float, float, float]

    def gain(self) -> Tuple[float, float, float]:
        """What multiplies the (desaturated) albedo, each channel: the tint times the brightness."""
        return tuple(round(c * self.brightness, 5) for c in self.tint)


NEUTRAL = ObjectColour(1.0, 0.0, (1.0, 1.0, 1.0))


def tint_rgb(hue_deg: float, strength: float) -> Tuple[float, float, float]:
    """A colour cast of hue ``hue_deg`` and ``strength`` (0: none), as a gain per channel whose luminance (:data:`LUMA`) is 1."""
    c = colorsys.hsv_to_rgb((hue_deg % 360.0) / 360.0, 1.0, 1.0)
    t = [(1.0 - strength) + strength * x for x in c]
    lum = sum(w * x for w, x in zip(LUMA, t))
    return tuple(round(x / lum, 4) for x in t)


def object_colour(s: ObjectsSpec, seed: int, change: int, object_id: int) -> ObjectColour:
    """Object ``object_id``'s colour from change ``change``."""
    r = random.Random(f"teleop-object-colour:{seed}:{change}:{object_id}")
    brightness = round(r.uniform(*s.brightness), 4)
    desaturation = round(r.uniform(*s.desaturation), 4)
    hue, strength = r.uniform(0.0, 360.0), r.uniform(0.0, s.tint)
    return ObjectColour(brightness, desaturation, tint_rgb(hue, strength))


@dataclass(frozen=True)
class KeyLight:
    azimuth_deg: float
    elevation_deg: float
    intensity: float  # in the light's own units: the drawn multiple of the reference
    kelvin: Optional[float]
    angle_deg: float

    def orientation(self) -> Tuple[float, float, float, float]:
        """The USD orientation (w, x, y, z) of a distant light, which shines along its -z, coming from this direction."""
        return key_orientation(self.azimuth_deg, self.elevation_deg)


@dataclass(frozen=True)
class Lighting:
    kelvin: Optional[float]     # every scene light's colour temperature; None: as the scene file has it
    exposure: Optional[float]   # the metering's target brightness; None: the environment's targetBrightness
    key: Optional[KeyLight]


def lighting(s: LightingSpec, seed: int, change: int, reference: float) -> Lighting:
    """The light of change ``change``; ``reference`` is what the key's drawn ``intensity`` multiplies."""
    def r(what: str) -> random.Random:
        return random.Random(f"teleop-lighting:{seed}:{change}:{what}")

    kelvin = round(r("kelvin").uniform(*s.kelvin)) if s.kelvin else None
    exposure = round(r("exposure").uniform(*s.exposure), 1) if s.exposure else None
    key = None
    if s.key is not None:
        k = r("key")
        azimuth = round(k.uniform(*s.key.azimuth_deg) % 360.0, 2)
        elevation = round(k.uniform(*s.key.elevation_deg), 2)
        intensity = round(k.uniform(*s.key.intensity) * reference, 1)
        angle = round(k.uniform(*s.key.angle_deg), 2)
        key_kelvin = round(k.uniform(*s.key.kelvin)) if s.key.kelvin else None
        key = KeyLight(azimuth, elevation, intensity, key_kelvin, angle)
    return Lighting(kelvin, exposure, key)


def key_orientation(azimuth_deg: float, elevation_deg: float) -> Tuple[float, float, float, float]:
    """A distant light's orientation (w, x, y, z) so that its -z, the way its light travels, points from the direction at
    ``azimuth_deg`` (from +x toward +y) and ``elevation_deg`` (above the horizon) toward the origin: a turn about y by
    90° - elevation, then about z by the azimuth."""
    tilt = kinematics.q_axis((0.0, 1.0, 0.0), math.radians(90.0 - elevation_deg))
    turn = kinematics.q_axis((0.0, 0.0, 1.0), math.radians(azimuth_deg))
    return kinematics.q_mul(turn, tilt)


@dataclass(frozen=True)
class CameraJitter:
    dx: float  # m, along the camera's own x (right)
    dy: float  # m, along its y (up)
    dz: float  # m, along its z (backward: a USD camera looks along its -z)
    pitch_deg: float  # about its x (tilt)
    yaw_deg: float    # about its y (pan)
    roll_deg: float   # about its z (roll)

    def rotation(self) -> Tuple[float, float, float, float]:
        """The turn, in the camera's own frame: about x, then y, then z (each about the axis as the turns before left it)."""
        q = kinematics.q_axis((1.0, 0.0, 0.0), math.radians(self.pitch_deg))
        q = kinematics.q_mul(q, kinematics.q_axis((0.0, 1.0, 0.0), math.radians(self.yaw_deg)))
        return kinematics.q_mul(q, kinematics.q_axis((0.0, 0.0, 1.0), math.radians(self.roll_deg)))


NO_JITTER = CameraJitter(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def camera_jitter(s: Optional[JitterSpec], seed: int, change: int, camera_id: str) -> CameraJitter:
    """Camera ``camera_id``'s offset from its pose in the scene file, for change ``change`` (none without a spec)."""
    if s is None:
        return NO_JITTER
    r = random.Random(f"teleop-camera-jitter:{seed}:{change}:{camera_id}")
    t, a = s.translation_m, s.rotation_deg
    return CameraJitter(*(round(r.uniform(-t, t), 5) for _ in range(3)), *(round(r.uniform(-a, a), 4) for _ in range(3)))


def turn_deg(q1: Sequence[float], q2: Sequence[float]) -> float:
    """The angle in degrees of the turn from orientation ``q1`` to ``q2`` (w, x, y, z), well conditioned at small angles: the
    relative quaternion's vector length against its scalar part, through atan2. Twice the arc cosine of the two quaternions'
    dot product is not: near 1 it turns a rounding error of 1e-7 into hundredths of a degree, so 7-decimal ``CM`` quaternions
    a hundred-thousandth of a degree apart measured 0.01-0.05 degrees that way (docs/adr/0038 "A, appearance", the twin
    check of 2026-11-23)."""
    n1 = math.sqrt(sum(c * c for c in q1))
    n2 = math.sqrt(sum(c * c for c in q2))
    a = tuple(c / n1 for c in q1)
    b = tuple(c / n2 for c in q2)
    r = kinematics.q_mul((a[0], -a[1], -a[2], -a[3]), b)
    return math.degrees(2.0 * math.atan2(math.sqrt(r[1] * r[1] + r[2] * r[2] + r[3] * r[3]), abs(r[0])))


def jittered(pose: Tuple[Sequence[float], Sequence[float]], j: CameraJitter):
    """A camera's pose (position, w-x-y-z orientation, in whatever frame it is given) moved by ``j`` in its own frame."""
    p, q = pose
    d = kinematics.q_rotate(tuple(q), (j.dx, j.dy, j.dz))
    q2 = kinematics.q_mul(tuple(q), j.rotation())
    n = math.sqrt(sum(c * c for c in q2))
    return tuple(a + b for a, b in zip(p, d)), tuple(c / n for c in q2)


# --- the records ---------------------------------------------------------------------------------------------------------

def oa_items(colours: Mapping[int, Tuple[str, ObjectColour]]) -> List[list]:
    """``OA``'s items, by object id: ``id:method:brightness:desaturation:r:g:b``, the tint's r, g, b. ``method`` is how its
    materials were changed (``omnipbr``, ``mtlx``, ``preview``, several joined by ``+``); a ``preview`` material takes no
    desaturation, so its record says 0."""
    return [[oid, method, c.brightness, c.desaturation, *c.tint] for oid, (method, c) in sorted(colours.items())]


def cj_items(jitters: Sequence[Tuple[str, CameraJitter]]) -> List[list]:
    """``CJ``'s items, in ``CM`` order: ``camera:dx:dy:dz:pitchDeg:yawDeg:rollDeg``."""
    return [[cid, j.dx, j.dy, j.dz, j.pitch_deg, j.yaw_deg, j.roll_deg] for cid, j in jitters]


def lt_fields(lt: Lighting) -> tuple:
    """``LT``'s fields after the change: ``domeKelvin`` ``exposure`` ``key`` (``azimuthDeg:elevationDeg:intensity:kelvin:angleDeg``),
    each empty when the section does not vary it."""
    key = [] if lt.key is None else [[lt.key.azimuth_deg, lt.key.elevation_deg, lt.key.intensity,
                                      "" if lt.key.kelvin is None else lt.key.kelvin, lt.key.angle_deg]]
    return ("" if lt.kelvin is None else lt.kelvin, "" if lt.exposure is None else lt.exposure, key)


# --- the materials, on the renderer's stage (pxr) --------------------------------------------------------------------

OMNIPBR_INPUTS = ("diffuse_tint", "albedo_desaturation", "albedo_brightness")
MTLX_SATURATE, MTLX_TINT = "TeleopColourSaturate", "TeleopColourTint"


def render_contexts(value) -> Tuple[str, ...]:
    """The render contexts in the order the renderer takes a material's network (Kit's setting
    ``/persistent/app/hydra/material/renderContexts``), or Kit's default when the setting is not there."""
    if isinstance(value, (list, tuple)) and value and all(isinstance(v, str) for v in value):
        return tuple(value)
    return DEFAULT_RENDER_CONTEXTS


def key_reference(scene) -> float:
    """What a key light's drawn ``intensity`` multiplies: the environment's ``meanRadiance``, or the first dome's intensity
    (the renderer's 800 when it gives none), or 1000 in a scene with no dome."""
    env = scene.extra.get("environment") or {}
    radiance = env.get("meanRadiance")
    if isinstance(radiance, (int, float)) and not isinstance(radiance, bool) and radiance > 0:
        return float(radiance)
    for light in scene.extra.get("lights") or []:
        if light.get("kind") == "dome":
            return float(light.get("intensity", 800))
    return 1000.0


@dataclass
class ColourTarget:
    """One material of a scanned object whose albedo can be varied, and the inputs that vary it."""
    material: str
    method: str  # "omnipbr", "mtlx" or "preview"
    inputs: Dict[str, object]  # name -> UsdShade.Input
    base: Optional[Tuple[float, ...]] = None  # a preview material's unconnected colour, which the gain multiplies


def _source_shader(material, context: str):
    """The shader a material's surface output for ``context`` is connected to, or None. Only that context's own output: no
    fallback to the universal one (``ComputeSurfaceSource`` would)."""
    from pxr import UsdShade

    out = material.GetSurfaceOutput(context) if context else material.GetSurfaceOutput()
    if not out or not out.HasConnectedSource():
        return None
    for src in out.GetConnectedSources()[0]:
        prim = src.source.GetPrim()
        if prim.IsA(UsdShade.Shader):
            return UsdShade.Shader(prim)
    return None


def _is_omnipbr(shader) -> bool:
    try:
        asset = shader.GetSourceAsset("mdl")
        sub = shader.GetSourceAssetSubIdentifier("mdl")
    except Exception:  # noqa: BLE001 -- not an MDL shader
        return False
    path = getattr(asset, "path", "") or str(asset)
    return sub == "OmniPBR" and path.replace("@", "").endswith("OmniPBR.mdl")


def _prepare_omnipbr(shader) -> Dict[str, object]:
    from pxr import Sdf

    return {"diffuse_tint": shader.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f),
            "albedo_desaturation": shader.CreateInput("albedo_desaturation", Sdf.ValueTypeNames.Float),
            "albedo_brightness": shader.CreateInput("albedo_brightness", Sdf.ValueTypeNames.Float)}


def _prepare_mtlx(stage, material, surface) -> Optional[Dict[str, object]]:
    """A MaterialX surface's ``base_color`` gets two nodes in front of it, neutral until :func:`apply_colour` sets them:
    ``ND_saturate_color3`` (amount 1 - desaturation), then ``ND_multiply_color3`` by the gain. None when the surface has no
    ``base_color``."""
    from pxr import Sdf, UsdShade

    base = surface.GetInput("base_color")
    if not base:
        return None
    path = material.GetPath()
    sat = UsdShade.Shader.Define(stage, path.AppendChild(MTLX_SATURATE))
    sat.CreateIdAttr("ND_saturate_color3")
    sat_in = sat.CreateInput("in", Sdf.ValueTypeNames.Color3f)
    if base.HasConnectedSource():
        src = base.GetConnectedSources()[0][0]
        sat_in.ConnectToSource(src.source, src.sourceName, src.sourceType)
    else:
        value = base.Get()
        sat_in.Set(value if value is not None else (0.8, 0.8, 0.8))
    amount = sat.CreateInput("amount", Sdf.ValueTypeNames.Float)
    amount.Set(1.0)
    sat_out = sat.CreateOutput("out", Sdf.ValueTypeNames.Color3f)
    mul = UsdShade.Shader.Define(stage, path.AppendChild(MTLX_TINT))
    mul.CreateIdAttr("ND_multiply_color3")
    mul.CreateInput("in1", Sdf.ValueTypeNames.Color3f).ConnectToSource(sat_out)
    gain = mul.CreateInput("in2", Sdf.ValueTypeNames.Color3f)
    gain.Set((1.0, 1.0, 1.0))
    base.ConnectToSource(mul.CreateOutput("out", Sdf.ValueTypeNames.Color3f))
    return {"amount": amount, "gain": gain}


def _prepare_preview(surface) -> Tuple[Optional[Dict[str, object]], Optional[Tuple[float, ...]]]:
    """A UsdPreviewSurface's ``diffuseColor``: the ``scale`` of the UsdUVTexture it reads, or its own value."""
    from pxr import Sdf, UsdShade

    diffuse = surface.GetInput("diffuseColor")
    if not diffuse:
        return None, None
    if diffuse.HasConnectedSource():
        src = diffuse.GetConnectedSources()[0][0]
        tex = UsdShade.Shader(src.source.GetPrim())
        if not tex or tex.GetIdAttr().Get() != "UsdUVTexture":
            return None, None
        return {"scale": tex.CreateInput("scale", Sdf.ValueTypeNames.Float4)}, None
    value = diffuse.Get()
    return {"diffuseColor": diffuse}, tuple(value) if value is not None else (0.18, 0.18, 0.18)


def prepare_colour(stage, root_path: str, contexts: Sequence[str] = DEFAULT_RENDER_CONTEXTS) -> List[ColourTarget]:
    """Every material under ``root_path`` (a scanned object's ``Shape``) whose albedo can be varied, readied once: for each,
    the first of ``contexts`` it has a surface for decides how (an OmniPBR MDL shader, a MaterialX surface with a
    ``base_color``, a UsdPreviewSurface); another MDL material is left alone. A material inside an instance cannot be edited,
    and is left alone too. Nothing looks different until :func:`apply_colour`."""
    from pxr import Usd, UsdShade

    root = stage.GetPrimAtPath(root_path)
    targets: List[ColourTarget] = []
    if not root or not root.IsValid():
        return targets
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdShade.Material) or prim.IsInstanceProxy():
            continue
        material = UsdShade.Material(prim)
        for context in contexts:
            shader = _source_shader(material, context)
            if shader is None:
                continue
            if context == "mdl":
                if _is_omnipbr(shader):
                    targets.append(ColourTarget(str(prim.GetPath()), "omnipbr", _prepare_omnipbr(shader)))
            elif context == "mtlx":
                got = _prepare_mtlx(stage, material, shader)
                if got is not None:
                    targets.append(ColourTarget(str(prim.GetPath()), "mtlx", got))
            elif shader.GetIdAttr().Get() == "UsdPreviewSurface":
                got, base = _prepare_preview(shader)
                if got is not None:
                    targets.append(ColourTarget(str(prim.GetPath()), "preview", got, base))
            break  # the renderer uses this context's network; the others are never drawn
    return targets


def apply_colour(targets: Sequence[ColourTarget], colour: ObjectColour) -> Tuple[str, ObjectColour]:
    """``colour`` on every target; (the methods used, joined by ``+``, and the colour as applied: a preview material takes no
    desaturation)."""
    from pxr import Gf

    gain = colour.gain()
    methods = []
    desaturated = False
    for t in targets:
        if t.method == "omnipbr":
            t.inputs["diffuse_tint"].Set(Gf.Vec3f(*colour.tint))
            t.inputs["albedo_desaturation"].Set(float(colour.desaturation))
            t.inputs["albedo_brightness"].Set(float(colour.brightness))
            desaturated = True
        elif t.method == "mtlx":
            t.inputs["amount"].Set(float(round(1.0 - colour.desaturation, 4)))
            t.inputs["gain"].Set(Gf.Vec3f(*gain))
            desaturated = True
        elif t.method == "preview":
            if "scale" in t.inputs:
                t.inputs["scale"].Set(Gf.Vec4f(gain[0], gain[1], gain[2], 1.0))
            else:
                t.inputs["diffuseColor"].Set(Gf.Vec3f(*(min(1.0, b * g) for b, g in zip(t.base, gain))))
        if t.method not in methods:
            methods.append(t.method)
    applied = colour if desaturated or not targets else ObjectColour(colour.brightness, 0.0, colour.tint)
    return "+".join(methods), applied
