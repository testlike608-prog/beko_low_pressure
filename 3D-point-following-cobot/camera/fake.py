"""
A camera made of saved files, or of nothing at all.

This is the reason the whole project can be built, tested and demonstrated on
a laptop: a detector change or a planner tweak is checked in a second, instead
of on an arm that has to be booked, cleared and powered.

    Fake()                     a plain synthetic scene at a fixed distance
    Fake(folder="captures")    replays real images; a .npy of the same name
                               beside an image is used as its depth (metres)

`camera.save(frame, "captures")` writes exactly that pair, so today's real
capture becomes tomorrow's regression test.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from core import CellError, find_file

from .base import Camera, Frame, Lens

# Matches the D435I on this cell, so a fake run and a real run produce
# comparable millimetres.
D435I = dict(width=1280, height=720, fx=911.70, fy=911.85, cx=644.02, cy=368.64)


class Fake(Camera):
    name = "fake"
    has_depth = True

    def __init__(self, folder=None, width=D435I["width"], height=D435I["height"],
                 fx=D435I["fx"], fy=D435I["fy"], cx=None, cy=None,
                 depth_m=0.45, loop=True, fps=None, serial=None):
        # fps and serial are accepted and ignored, so switching CAMERA["kind"]
        # from "realsense" to "fake" is a one-word edit rather than a rewrite.
        self.folder = Path(folder) if folder else None
        self.flat_depth, self.loop = float(depth_m), loop
        self._lens = Lens(width, height, fx, fy,
                          width / 2.0 if cx is None else cx,
                          height / 2.0 if cy is None else cy)
        self._files, self._i = [], 0

    def open(self) -> None:
        if self.folder is None:
            return
        self.folder = find_file(self.folder, "frame folder")
        if not self.folder.is_dir():
            raise CellError(f"{self.folder} is not a folder")
        self._files = sorted(p for p in self.folder.iterdir()
                             if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
        if not self._files:
            raise CellError(f"{self.folder} holds no images")
        print(f"  camera: fake, {len(self._files)} frames from {self.folder}")

    def close(self) -> None:
        self._i = 0

    def read(self, timeout_s: float = 5.0) -> Frame:
        if self.folder is None:
            w, h = self._lens.width, self._lens.height
            color = np.full((h, w, 3), 60, np.uint8)
            color[h // 3:2 * h // 3, w // 3:2 * w // 3] = 140    # a "part"
            return Frame(color, np.full((h, w), self.flat_depth, np.float32),
                         self._lens)

        import cv2
        if self._i >= len(self._files):
            if not self.loop:
                raise CellError("the recording is finished")
            self._i = 0
        path = self._files[self._i]
        self._i += 1

        color = cv2.imread(str(path))
        if color is None:
            raise CellError(f"{path} could not be read as an image")

        npy = path.with_suffix(".npy")
        depth = (np.load(npy).astype(np.float32) if npy.exists()
                 else np.full(color.shape[:2], self.flat_depth, np.float32))

        h, w = color.shape[:2]
        lens = self._lens
        if (w, h) != (lens.width, lens.height):
            # Scale the model with the image rather than silently deprojecting
            # with the wrong focal length -- an error that looks like a
            # calibration problem and is not one.
            sx, sy = w / lens.width, h / lens.height
            lens = Lens(w, h, lens.fx * sx, lens.fy * sy,
                        lens.cx * sx, lens.cy * sy, lens.distortion)
        return Frame(color, depth, lens)

    @property
    def lens(self) -> Lens:
        return self._lens
