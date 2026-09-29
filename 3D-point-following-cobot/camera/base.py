"""
What every camera in this project has to be.

A camera's whole job ends at one operation:

    a pixel + whatever depth this device has  ->  a point in the CAMERA frame

Everything before that line is the camera's problem (one driver). Everything
after it is the robot's problem (one hand-eye transform). Neither side is ever
edited to work with a new counterpart -- that is the entire reason the two are
split into separate packages here.

To add a camera: copy one of the driver files, fill in open/close/read, add it
to REGISTRY in camera/__init__.py. Nothing else in the project changes.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from core import CAMERA, CellError, Pixel, Point


@dataclass
class Lens:
    """
    The pinhole model: plain numbers, so a calibration can live in a JSON file
    and a test can run with no camera attached.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    def deproject(self, u: float, v: float, depth_mm: float):
        """Pixel + depth -> (x, y, z) in the camera frame, millimetres."""
        return ((float(u) - self.cx) / self.fx * depth_mm,
                (float(v) - self.cy) / self.fy * depth_mm,
                float(depth_mm))

    def project(self, xyz):
        """The other way -- for drawing a known point back onto the image."""
        x, y, z = (float(v) for v in xyz)
        if z <= 1e-6:
            raise CellError("cannot project a point at or behind the lens")
        return (x / z * self.fx + self.cx, y / z * self.fy + self.cy)

    def dict(self) -> dict:
        return {"width": self.width, "height": self.height,
                "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
                "distortion": list(self.distortion)}

    def __str__(self):
        return (f"{self.width}x{self.height}  fx {self.fx:.1f} fy {self.fy:.1f} "
                f"cx {self.cx:.1f} cy {self.cy:.1f}")


@dataclass
class Frame:
    """One instant. `depth` is METRES, aligned to colour, or None."""

    color: np.ndarray | None = None
    depth: np.ndarray | None = None
    lens: Lens | None = None
    when: float = field(default_factory=time.time)

    @property
    def has_depth(self) -> bool:
        return self.depth is not None

    @property
    def size(self):
        if self.color is None:
            return (0, 0)
        h, w = self.color.shape[:2]
        return (w, h)

    def depth_coverage(self) -> float:
        """Fraction of pixels with a real depth reading. Below ~0.5 on the work
        area means shiny metal or a bad exposure, not a calibration problem."""
        if self.depth is None:
            return 0.0
        return float(np.count_nonzero(self.depth > 0) / self.depth.size)


class Camera:
    """
    Base class. Implement open(), close(), read() and the `lens` property.

    Vendor SDKs are imported INSIDE methods, never at the top of a driver file,
    so this whole package imports fine on a PC that has none of them installed.
    """

    name = "camera"
    has_depth = False

    #: A 2D camera can still be used when the work sits at a known distance.
    #: Set it and every pixel is deprojected at this range. It is an
    #: assumption, and it is written here rather than hidden inside a detector.
    assume_depth_m: float | None = None

    #: Median over a patch, not a single pixel. Depth images are noisy and one
    #: reading on an edge lands on the background -- half a metre out.
    patch = 5

    def open(self) -> None: ...

    def close(self) -> None: ...

    def read(self, timeout_s: float = 5.0) -> Frame:
        raise NotImplementedError

    @property
    def lens(self) -> Lens:
        raise NotImplementedError

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # -- the one operation the rest of the project needs -------------------
    def depth_at(self, frame: Frame, u: float, v: float) -> float | None:
        """
        Depth in metres at a pixel, or None when there is no usable answer.

        Zeros in a depth image mean "no reading" -- shiny weld, too close, in
        shadow -- not "at the lens". Averaging them in drags a good point
        towards the camera by however many neighbours failed, which is exactly
        the silent few-millimetre error that is hardest to find later.
        """
        if frame.depth is None:
            return self.assume_depth_m
        h, w = frame.depth.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        r = max(0, self.patch // 2)
        window = frame.depth[max(0, vi - r):vi + r + 1,
                             max(0, ui - r):ui + r + 1]
        good = window[window > 0]
        if good.size < max(3, (2 * r + 1) ** 2 // 4):
            return None
        return float(np.median(good))

    def to_point(self, frame: Frame, pixel: Pixel) -> Point | None:
        """
        A detection -> a point in the camera frame, or None.

        None means "no depth worth trusting here", which is normal on a shiny
        weld and must NOT be an exception: one bad pixel out of six is a point
        to skip and mention, not a failed cycle.
        """
        depth = (pixel.depth_m if pixel.depth_m is not None
                 else self.depth_at(frame, pixel.u, pixel.v))
        if depth is None or depth <= 0:
            return None
        lens = frame.lens or self.lens
        meta = dict(pixel.meta, uv=(float(pixel.u), float(pixel.v)),
                    depth_m=round(depth, 5))
        if frame.depth is None and self.assume_depth_m is not None:
            meta["depth_assumed"] = True
        return Point(*lens.deproject(pixel.u, pixel.v, depth * 1000.0),
                     frame=CAMERA, label=pixel.label,
                     confidence=pixel.confidence, meta=meta)

    def to_points(self, frame: Frame, pixels):
        """(converted, dropped). The dropped ones get reported, not hidden."""
        good, bad = [], []
        for px in pixels:
            p = self.to_point(frame, px)
            (good.append(p) if p is not None else bad.append(px))
        return good, bad

    def describe(self) -> str:
        if self.has_depth:
            return f"{self.name} (colour + depth)"
        if self.assume_depth_m:
            return f"{self.name} (2D, assuming {self.assume_depth_m} m)"
        return f"{self.name} (2D, no depth)"
