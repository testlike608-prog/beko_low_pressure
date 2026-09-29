"""
What finds the welding points.

A detector takes an IMAGE and returns PIXELS. It never returns millimetres and
it never touches a robot.

That one rule is what keeps the AI model replaceable. Train a better model, or
move from YOLO to something else, and only a file in this package changes --
the camera does not know a model exists, the robot does not know a camera
exists, and the hand-eye transform in core/ works on whatever comes out.

    class MyDetector(Detector):
        def find(self, frame) -> list[Pixel]:
            ...
"""

from __future__ import annotations

from core import Pixel


class Detector:
    name = "detector"

    #: Set by a model that also produces a surface normal or a port axis. The
    #: black rubber service port has to be entered ALONG its axis, so a model
    #: that reports one lets the cell plan an insertion instead of a poke.
    gives_normals = False

    def open(self) -> None:
        """Load weights, warm up. Called once by vision.start()."""

    def close(self) -> None:
        """Release whatever open() took."""

    def find(self, frame) -> list[Pixel]:
        """One frame -> the welding points it sees, in pixels."""
        raise NotImplementedError

    def describe(self) -> str:
        return self.name

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()


def sort_reading_order(pixels, row_tolerance=40.0) -> list[Pixel]:
    """
    Left-to-right, top-to-bottom -- the order a person would name them.

    Worth having as a default: a model returns detections in confidence order,
    which changes between frames, so the same physical weld gets a different
    number every cycle and the report becomes impossible to compare. Sorting by
    position makes point 3 the same weld tomorrow.
    """
    pixels = list(pixels)
    if not pixels:
        return []
    rows: list[list] = []
    for px in sorted(pixels, key=lambda p: p.v):
        for row in rows:
            if abs(row[0].v - px.v) <= row_tolerance:
                row.append(px)
                break
        else:
            rows.append([px])
    out = []
    for row in rows:
        out.extend(sorted(row, key=lambda p: p.u))
    return out


def drop_duplicates(pixels, min_gap_px=15.0) -> list[Pixel]:
    """
    Two detections on one weld become two robot visits and two leak tests.
    Highest confidence wins; the loser is dropped, not averaged -- averaging
    two boxes that landed on different features invents a point on neither.
    """
    kept: list[Pixel] = []
    for px in sorted(pixels, key=lambda p: -p.confidence):
        if all((px.u - k.u) ** 2 + (px.v - k.v) ** 2 > min_gap_px ** 2
               for k in kept):
            kept.append(px)
    return kept
