"""
The trained model: Ultralytics YOLO, detect / segment / pose.

This is the file that changes when the weld-point model improves. Nothing else
in the project does.

Three task types, because the two targets on the fridge are different problems:

    detect      a box per weld -> its centre is the point.
                Fine for the brazed copper joints, which are visible on the
                surface and only need a position.

    pose        keypoints per object -> each keypoint is a point, and two
                keypoints give a DIRECTION. This is what the black rubber
                service port needs: the sniffer has to go in ALONG the port's
                axis, so a model that marks the mouth and the far end lets the
                cell plan an insertion instead of a poke at a surface.

    segment     a mask per weld -> its centroid is the point, and the mask's
                long axis is a usable direction for a seam.

Ultralytics is imported inside open(), so this file imports on a PC with no
torch installed -- which is every PC running the test suite.
"""

from __future__ import annotations

import numpy as np

from core import CellError, Pixel, find_file

from .base import Detector, drop_duplicates, sort_reading_order


class Yolo(Detector):
    """
        Yolo("weld.pt")                                boxes
        Yolo("weld-pose.pt", task="pose")              keypoints + axis
        Yolo("weld.pt", classes=["braze", "port"], min_confidence=0.55)
    """

    name = "yolo"

    def __init__(self, weights, task="detect", min_confidence=0.4,
                 classes=None, device=None, image_size=None,
                 axis_from_keypoints=(0, 1), min_gap_px=15.0):
        self.weights = weights
        self.task = task
        self.min_confidence = float(min_confidence)
        self.classes = list(classes) if classes else None
        self.device = device
        self.image_size = image_size
        self.axis_from_keypoints = axis_from_keypoints
        self.min_gap_px = float(min_gap_px)
        self.model = None
        self.gives_normals = (task == "pose")

    def open(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise CellError(
                "ultralytics is not installed (pip install ultralytics). "
                "Until the model is trained, set VISION = {'kind': 'clicks'} "
                "in settings.py and pick the points by hand.") from e
        path = find_file(self.weights, "model weights")
        self.model = YOLO(str(path))
        print(f"  vision: yolo {self.task}, {path.name}"
              + (f", {len(self.model.names)} classes" if self.model.names else ""))

    def close(self) -> None:
        self.model = None

    # ------------------------------------------------------------------
    def _wanted(self, class_id) -> bool:
        if not self.classes:
            return True
        names = getattr(self.model, "names", {}) or {}
        return names.get(int(class_id), str(class_id)) in self.classes

    def _label(self, class_id) -> str:
        names = getattr(self.model, "names", {}) or {}
        return str(names.get(int(class_id), f"class{int(class_id)}"))

    def find(self, frame) -> list[Pixel]:
        if self.model is None:
            raise CellError("the model has not been loaded -- call vision.start()")

        kw = dict(verbose=False, conf=self.min_confidence)
        if self.device:
            kw["device"] = self.device
        if self.image_size:
            kw["imgsz"] = self.image_size

        result = self.model(frame.color, **kw)[0]
        pixels: list[Pixel] = []

        keypoints = getattr(result, "keypoints", None)
        boxes = getattr(result, "boxes", None)
        masks = getattr(result, "masks", None)

        if self.task == "pose" and keypoints is not None and keypoints.xy is not None:
            for i, kp in enumerate(keypoints.xy):
                kp = np.asarray(kp, dtype=float)
                if kp.size == 0:
                    continue
                conf = self._box_conf(boxes, i)
                cls = self._box_cls(boxes, i)
                if not self._wanted(cls):
                    continue
                a, b = self.axis_from_keypoints
                meta = {"detector": "yolo-pose", "object": i}
                if 0 <= a < len(kp) and 0 <= b < len(kp):
                    d = kp[b] - kp[a]
                    if float(np.hypot(*d)) > 3.0:
                        # A 2D direction only. It becomes a real 3D axis once
                        # both keypoints have depth -- done in vision/__init__.
                        meta["axis_uv"] = (float(kp[a][0]), float(kp[a][1]),
                                           float(kp[b][0]), float(kp[b][1]))
                for j, (u, v) in enumerate(kp):
                    pixels.append(Pixel(float(u), float(v), None,
                                        label=f"{self._label(cls)}{i}.{j}",
                                        confidence=conf, meta=dict(meta, kp=j)))

        elif self.task == "segment" and masks is not None and masks.xy is not None:
            for i, poly in enumerate(masks.xy):
                poly = np.asarray(poly, dtype=float)
                if poly.size == 0:
                    continue
                cls = self._box_cls(boxes, i)
                if not self._wanted(cls):
                    continue
                u, v = poly.mean(axis=0)
                pixels.append(Pixel(float(u), float(v), None,
                                    label=self._label(cls),
                                    confidence=self._box_conf(boxes, i),
                                    meta={"detector": "yolo-seg",
                                          "area_px": float(len(poly))}))

        elif boxes is not None:
            for i in range(len(boxes)):
                cls = self._box_cls(boxes, i)
                if not self._wanted(cls):
                    continue
                x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i])
                pixels.append(Pixel((x1 + x2) / 2.0, (y1 + y2) / 2.0, None,
                                    label=self._label(cls),
                                    confidence=self._box_conf(boxes, i),
                                    meta={"detector": "yolo",
                                          "box": (x1, y1, x2, y2)}))

        if self.task != "pose":     # keypoints of one object are not duplicates
            pixels = drop_duplicates(pixels, self.min_gap_px)
        return sort_reading_order(pixels)

    @staticmethod
    def _box_conf(boxes, i) -> float:
        try:
            return float(boxes.conf[i])
        except Exception:
            return 1.0

    @staticmethod
    def _box_cls(boxes, i) -> int:
        try:
            return int(boxes.cls[i])
        except Exception:
            return 0

    def describe(self) -> str:
        return f"yolo {self.task} ({self.weights})"
