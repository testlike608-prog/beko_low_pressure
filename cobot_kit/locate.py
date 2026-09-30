"""
Pixels -> x, y, z the cobot understands.

    from cobot_kit import Locator

    locator = Locator()                        # handeye.json from settings.py
    points = locator.to_robot(shot, pixels)    # [Point(x, y, z, frame="base"), ...]

Two steps, and the second is the only thing that joins the camera half to the
robot half:

    pixel + depth           -> point in the CAMERA frame   (the lens)
    camera point + flange   -> point in the ROBOT BASE     (handeye.json)

The flange pose comes from the Shot -- the pose at the instant of the picture,
not a fresh read. Using a pose read after the arm moved is a tens-of-mm error
that looks exactly like a bad calibration.

A pixel with no usable depth (shiny weld, shadow) is SKIPPED and reported, not
turned into an exception: one bad point out of six is a point to mention, not
a failed cycle.
"""

from __future__ import annotations

from . import settings
from .datatypes import CAMERA, Batch, Point
from .handeye import HandEye


class Locator:
    """
        Locator()                          settings.HANDEYE
        Locator("handeye.json", patch=7)   bigger depth window on noisy surfaces
    """

    def __init__(self, handeye=None, patch=5, warn=True):
        self.handeye = HandEye(handeye or settings.HANDEYE)
        self.patch = int(patch)
        if warn:
            for w in self.handeye.warnings():
                print(f"  !! {w}")

    def to_camera(self, shot, pixels=None) -> tuple[list, list]:
        """(camera-frame points, pixels that had no depth)."""
        frame = shot.frame
        lens = frame.lens
        pixels = shot.pixels if pixels is None else pixels
        good, dropped = [], []
        for px in pixels:
            depth = px.depth_m if px.depth_m is not None else frame.depth_at(
                px.u, px.v, self.patch)
            if not depth or depth <= 0:
                dropped.append(px)
                continue
            x, y, z = lens.deproject(px.u, px.v, depth * 1000.0)
            good.append(Point(x, y, z, frame=CAMERA, label=px.label,
                              confidence=px.confidence,
                              meta=dict(px.meta, uv=(px.u, px.v),
                                        depth_m=round(depth, 5),
                                        shot=shot.name)))
        return good, dropped

    def to_robot(self, shot, pixels=None) -> list[Point]:
        """
        Pixels of one Shot -> base-frame points, ready for the cycle.
        Also stored on shot.points.
        """
        cam_points, dropped = self.to_camera(shot, pixels)
        if dropped:
            print(f"  !! {len(dropped)} point(s) had no usable depth and were "
                  f"skipped: " + ", ".join(str(p) for p in dropped[:5]))
        base = self.handeye.to_base(Batch(points=cam_points, flange=shot.flange))
        shot.points = base
        return base

    def describe(self) -> str:
        return self.handeye.describe()
