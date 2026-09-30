"""
The vocabulary every layer shares.

    Pose    where the arm is / should be:  x y z mm, rx ry rz degrees
    Pixel   what the vision model saw:     u v, optional depth
    Point   a place in 3D, in a NAMED frame
    Result  what the sniffer said about one point

Frames are strings ("camera", "base") because they end up in log lines and
config files -- but they are checked, so a typo raises instead of quietly
sending the arm half a metre away.

Nothing in this file imports a camera, a robot or a vendor SDK. That is
deliberate: it is the only module every other module is allowed to depend on.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Sequence

# ------------------------------------------------------------------- frames
CAMERA = "camera"
BASE = "base"
FLANGE = "flange"
TOOL = "tool"

FRAMES = (CAMERA, BASE, FLANGE, TOOL)


def check_frame(name: str) -> str:
    if name not in FRAMES:
        raise ValueError(f"unknown frame {name!r}; use one of {FRAMES}")
    return name


# ------------------------------------------------------------------- errors
class CellError(Exception):
    """Something the operator can act on. Printed as a sentence, not a stack."""


class Unreachable(CellError):
    """The arm cannot make this pose -- skip the point, do not stop the cycle."""


class Refused(CellError):
    """The controller said no. `code` is its own number when we have one."""

    def __init__(self, message: str, code: int | None = None):
        super().__init__(message)
        self.code = code


# -------------------------------------------------------------- finding files
def find_file(path, what: str = "file"):
    """
    Turn a relative path into a real one, looking where a person would look.

    `handeye.json` in settings.py means the file NEXT TO settings.py -- not
    next to whatever folder the program happened to be started from. Without
    this, everything works when you run from the project folder and breaks from
    a desktop shortcut, a scheduled task or an IDE with its own working
    directory. Same bug, three disguises.

    When nothing is found it names every place it looked, because "no such
    file" with a single relative path is a guessing game.
    """
    import sys
    from pathlib import Path

    p = Path(str(path)).expanduser()
    if p.is_absolute():
        if p.exists():
            return p
        raise CellError(f"no {what} at {p}")

    tried = [Path.cwd() / p]
    here = Path(__file__).resolve().parent                 # the cobot_kit folder
    tried += [here / p, here.parent / p]
    main = sys.modules.get("__main__")
    script = getattr(main, "__file__", None)
    if script:
        d = Path(script).resolve().parent
        tried += [d / p, d.parent / p]

    for candidate in tried:
        if candidate.exists():
            return candidate.resolve()

    places = "\n  ".join(str(c) for c in dict.fromkeys(tried))
    raise CellError(f"no {what} called '{path}'. I looked in:\n  {places}")


# --------------------------------------------------------------------- pose
@dataclass(frozen=True)
class Pose:
    """
    x y z in millimetres, rx ry rz in degrees -- exactly what the pendant shows.

    Rotation convention is Rz @ Ry @ Rx (see core/geometry.py). A controller
    that uses a different one converts inside its own driver, never here.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    @classmethod
    def of(cls, values: Sequence[float]) -> "Pose":
        v = [float(n) for n in values]
        if len(v) != 6:
            raise ValueError(f"a pose is six numbers, got {len(v)}")
        return cls(*v)

    def list(self) -> list[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def rpy(self) -> tuple[float, float, float]:
        return (self.rx, self.ry, self.rz)

    @property
    def reach(self) -> float:
        """Straight-line distance from the base origin."""
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def at(self, xyz: Sequence[float]) -> "Pose":
        """Same orientation, new position -- the commonest edit there is."""
        x, y, z = (float(v) for v in xyz)
        return replace(self, x=x, y=y, z=z)

    def turned(self, rpy: Sequence[float]) -> "Pose":
        rx, ry, rz = (float(v) for v in rpy)
        return replace(self, rx=rx, ry=ry, rz=rz)

    def shifted(self, dx=0.0, dy=0.0, dz=0.0) -> "Pose":
        return replace(self, x=self.x + dx, y=self.y + dy, z=self.z + dz)

    def distance_to(self, other: "Pose") -> float:
        return math.dist(self.xyz, other.xyz)

    def __str__(self) -> str:
        return (f"({self.x:7.1f}, {self.y:7.1f}, {self.z:7.1f} | "
                f"{self.rx:6.1f}, {self.ry:6.1f}, {self.rz:6.1f})")


# -------------------------------------------------------------------- pixel
@dataclass(frozen=True)
class Pixel:
    """
    What a detector saw, before anything 3D happens.

    A detector NEVER returns millimetres. It returns pixels and, if it happens
    to know one, a depth. Keeping it that way is what lets the same model run
    on a different camera, or on a camera bolted to a different robot.
    """

    u: float
    v: float
    depth_m: float | None = None
    label: str = ""
    confidence: float = 1.0
    meta: dict = field(default_factory=dict)

    def __str__(self) -> str:
        d = f", {self.depth_m * 1000:.0f} mm" if self.depth_m else ""
        return f"[{self.label or 'point'} @ {self.u:.0f},{self.v:.0f}{d}]"


# -------------------------------------------------------------------- point
@dataclass(frozen=True)
class Point:
    """
    A place in 3D that carries the name of the frame it is expressed in.

    `normal` is optional and is a DIRECTION, not a position: it points from the
    target back towards where the tool should come from. The sniffer uses it to
    find the service port's axis.
    """

    x: float
    y: float
    z: float
    frame: str = CAMERA
    label: str = ""
    confidence: float = 1.0
    normal: tuple | None = None
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        check_frame(self.frame)

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def moved(self, xyz: Sequence[float], frame: str) -> "Point":
        x, y, z = (float(v) for v in xyz)
        return replace(self, x=x, y=y, z=z, frame=check_frame(frame))

    def with_normal(self, normal) -> "Point":
        n = tuple(float(v) for v in normal)
        return replace(self, normal=n)

    def distance_to(self, other: "Point") -> float:
        if other.frame != self.frame:
            raise ValueError(f"cannot measure {self.frame} against {other.frame}")
        return math.dist(self.xyz, other.xyz)

    def __str__(self) -> str:
        return (f"{self.label or 'point'} "
                f"({self.x:.1f}, {self.y:.1f}, {self.z:.1f}) {self.frame}")


# -------------------------------------------------------------------- batch
@dataclass
class Batch:
    """
    The points from ONE look, plus the flange pose at the instant of the look.

    The flange pose is the load-bearing part. The camera rides the arm, so a
    camera-frame point only means something together with where the arm was
    standing when the shutter fired. Carrying that pose with the points is what
    makes it impossible to transform a batch with a pose read a second later,
    after the arm has moved -- an error of tens of millimetres that looks
    exactly like a bad calibration.
    """

    points: list = field(default_factory=list)
    flange: Pose | None = None
    when: float = field(default_factory=time.time)
    dropped: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    @property
    def frame(self) -> str:
        return self.points[0].frame if self.points else CAMERA


# ------------------------------------------------------------------- result
@dataclass
class Result:
    """One point, visited and sniffed."""

    index: int
    label: str
    point: Point
    pose: Pose | None = None
    reached: bool = False
    leak: str = "not tested"        # "pass" | "fail" | "not tested" | "error"
    value: float | None = None
    note: str = ""
    seconds: float = 0.0

    @property
    def ok(self) -> bool:
        return self.reached and self.leak == "pass"

    def line(self) -> str:
        mark = {"pass": "PASS", "fail": "FAIL"}.get(self.leak, "----")
        val = f" {self.value:.3g}" if self.value is not None else ""
        return (f"  {self.index:>2}. {self.label or 'point':<16} "
                f"{'reached' if self.reached else 'MISSED ':<8} {mark}{val}"
                + (f"   {self.note}" if self.note else ""))

    def dict(self) -> dict:
        return {"index": self.index, "label": self.label,
                "point": list(self.point.xyz), "frame": self.point.frame,
                "pose": self.pose.list() if self.pose else None,
                "reached": self.reached, "leak": self.leak,
                "value": self.value, "note": self.note,
                "seconds": round(self.seconds, 3)}
