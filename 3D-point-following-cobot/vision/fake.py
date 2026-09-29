"""
Detectors that need neither a model nor a person.

    Fixed([(640, 360), (700, 400)])     always the same pixels
    Fixed(count=6)                      six spread over the middle of the image
    FromFile("points.csv")              u,v[,label] per line

This is what makes a full cycle testable end to end on a laptop: a fake camera,
a fake detector, a simulator arm, and the real hand-eye maths in the middle.
The only untested part is then the hardware itself, which is exactly the part
you want to spend cell time on.
"""

from __future__ import annotations

from core import CellError, Pixel, find_file

from .base import Detector


class Fixed(Detector):
    name = "fixed"

    def __init__(self, pixels=None, count=4, label="weld", depth_m=None,
                 expect=None):
        # `expect` is what the click detector calls the same idea, accepted
        # here so VISION["kind"] can be switched with a one-word edit.
        self.pixels = list(pixels) if pixels else None
        self.count = int(expect if expect is not None else count)
        self.label = label
        self.depth_m = depth_m

    def find(self, frame) -> list[Pixel]:
        if self.pixels is not None:
            return [Pixel(float(p[0]), float(p[1]), self.depth_m,
                          label=f"{self.label}{i}", meta={"detector": "fixed"})
                    for i, p in enumerate(self.pixels, 1)]

        w, h = frame.size if frame.color is not None else (1280, 720)
        out = []
        for i in range(self.count):
            u = w * (0.35 + 0.30 * i / max(1, self.count - 1))
            v = h * (0.40 + 0.20 * (i % 2))
            out.append(Pixel(u, v, self.depth_m, label=f"{self.label}{i + 1}",
                             meta={"detector": "fixed"}))
        return out

    def describe(self) -> str:
        n = len(self.pixels) if self.pixels is not None else self.count
        return f"fixed ({n} synthetic points)"


class FromFile(Detector):
    """
    Pixels from a CSV: `u,v` or `u,v,label` or `u,v,label,depth_m` per line.
    Lines starting with # are comments.

    Use it to replay the exact points from a run that went wrong.
    """

    name = "file"

    def __init__(self, path="points.csv"):
        self.path = path
        self._pixels: list[Pixel] = []

    def open(self) -> None:
        file = find_file(self.path, "points file")
        out = []
        for n, raw in enumerate(file.read_text(encoding="utf-8").splitlines(), 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            try:
                u, v = float(parts[0]), float(parts[1])
            except (ValueError, IndexError):
                raise CellError(f"{file}:{n} is not 'u,v[,label[,depth_m]]': "
                                f"{raw!r}") from None
            label = parts[2] if len(parts) > 2 and parts[2] else f"point{len(out) + 1}"
            depth = float(parts[3]) if len(parts) > 3 and parts[3] else None
            out.append(Pixel(u, v, depth, label=label, meta={"detector": "file"}))
        if not out:
            raise CellError(f"{file} holds no points")
        self._pixels = out
        print(f"  vision: {len(out)} points from {file.name}")

    def find(self, frame) -> list[Pixel]:
        return list(self._pixels)

    def describe(self) -> str:
        return f"from file ({self.path})"
