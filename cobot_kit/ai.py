"""
Who picks the points. Every picker has the same three methods:

    picker.start()                 load weights / nothing
    pixels = picker.find(image)    image -> [Pixel(u, v, label, confidence), ...]
    picker.stop()

`image` can be a numpy BGR image, a Frame, or a Shot -- whatever is at hand.

A picker returns PIXELS, never millimetres, and never touches the robot. That
is what makes them interchangeable: the old workflow (a person clicks) and the
new one (the AI picks) feed exactly the same next step.

    AiModel("weld.pt")             the trained YOLO model   <- the new workflow
    ClickPicker()                  a person clicks          <- the old workflow
    FunctionPicker(my_function)    ANY model of yours: image -> [(u, v), ...]
    FixedPoints([(640, 360)])      testing without a model
"""

from __future__ import annotations

import numpy as np

from . import settings
from .datatypes import CellError, Pixel, find_file


def _image_of(thing):
    """numpy image, Frame, or Shot -> numpy BGR image."""
    if thing is None:
        raise CellError("there is no image to look at")
    if isinstance(thing, np.ndarray):
        return thing
    frame = getattr(thing, "frame", None)          # a Shot
    if frame is not None:
        return frame.color
    color = getattr(thing, "color", None)          # a Frame
    if color is not None:
        return color
    raise CellError(f"cannot find an image in a {type(thing).__name__}")


def sort_reading_order(pixels, row_tolerance=40.0) -> list:
    """
    Left-to-right, top-to-bottom. A model returns detections in confidence
    order, which changes between frames; sorting by position makes "point 3"
    the same weld every cycle.
    """
    rows: list[list] = []
    for px in sorted(pixels, key=lambda p: p.v):
        for row in rows:
            if abs(row[0].v - px.v) <= row_tolerance:
                row.append(px)
                break
        else:
            rows.append([px])
    return [px for row in rows for px in sorted(row, key=lambda p: p.u)]


def drop_duplicates(pixels, min_gap_px=15.0) -> list:
    """Two detections on one weld = two visits. Highest confidence wins."""
    kept: list = []
    for px in sorted(pixels, key=lambda p: -p.confidence):
        if all((px.u - k.u) ** 2 + (px.v - k.v) ** 2 > min_gap_px ** 2 for k in kept):
            kept.append(px)
    return kept


class Picker:
    name = "picker"

    def start(self):
        return self

    def stop(self) -> None: ...

    def find(self, image) -> list:
        raise NotImplementedError

    def describe(self) -> str:
        return self.name

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


# ======================================================== the trained model
class AiModel(Picker):
    """
        AiModel()                                     settings.AI
        AiModel("weld.pt", min_confidence=0.55)
        AiModel("weld.pt", classes=["weld"])          keep only these classes
        AiModel("weld-seg.pt", task="segment")        mask centroid = the point
        AiModel("weld-pose.pt", task="pose")          every keypoint = a point
    """

    name = "yolo"

    def __init__(self, weights=None, task=None, min_confidence=None,
                 classes=None, device=None, image_size=None, min_gap_px=15.0):
        cfg = settings.AI
        self.weights = weights or cfg.get("weights")
        self.task = task or cfg.get("task", "detect")
        self.min_confidence = float(cfg.get("min_confidence", 0.4)
                                    if min_confidence is None else min_confidence)
        self.classes = list(classes) if classes else None
        self.device, self.image_size = device, image_size
        self.min_gap_px = float(min_gap_px)
        self.model = None

    def start(self):
        if self.model is not None:
            return self
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise CellError("ultralytics is not installed (pip install "
                            "ultralytics). Until the model exists use "
                            "ClickPicker() -- same interface.") from e
        path = find_file(self.weights, "model weights")
        self.model = YOLO(str(path))
        print(f"  ai: yolo {self.task}, {path.name}")
        return self

    def stop(self) -> None:
        self.model = None

    def _name(self, cls) -> str:
        names = getattr(self.model, "names", {}) or {}
        return str(names.get(int(cls), f"class{int(cls)}"))

    def find(self, image) -> list:
        if self.model is None:
            raise CellError("the model is not loaded -- call ai.start()")
        kw = dict(verbose=False, conf=self.min_confidence)
        if self.device:
            kw["device"] = self.device
        if self.image_size:
            kw["imgsz"] = self.image_size
        result = self.model(_image_of(image), **kw)[0]

        boxes = getattr(result, "boxes", None)
        conf = lambda i: float(boxes.conf[i]) if boxes is not None else 1.0
        cls = lambda i: int(boxes.cls[i]) if boxes is not None else 0
        wanted = lambda i: not self.classes or self._name(cls(i)) in self.classes

        pixels = []
        keypoints = getattr(result, "keypoints", None)
        masks = getattr(result, "masks", None)
        if self.task == "pose" and keypoints is not None and keypoints.xy is not None:
            for i, kp in enumerate(keypoints.xy):
                if not wanted(i):
                    continue
                for j, (u, v) in enumerate(np.asarray(kp, dtype=float)):
                    pixels.append(Pixel(float(u), float(v), None,
                                        label=f"{self._name(cls(i))}{i}.{j}",
                                        confidence=conf(i)))
            return sort_reading_order(pixels)
        if self.task == "segment" and masks is not None and masks.xy is not None:
            for i, poly in enumerate(masks.xy):
                poly = np.asarray(poly, dtype=float)
                if poly.size and wanted(i):
                    u, v = poly.mean(axis=0)
                    pixels.append(Pixel(float(u), float(v), None,
                                        label=self._name(cls(i)), confidence=conf(i)))
        elif boxes is not None:
            for i in range(len(boxes)):
                if wanted(i):
                    x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i])
                    pixels.append(Pixel((x1 + x2) / 2, (y1 + y2) / 2, None,
                                        label=self._name(cls(i)), confidence=conf(i),
                                        meta={"box": (x1, y1, x2, y2)}))
        return sort_reading_order(drop_duplicates(pixels, self.min_gap_px))

    def describe(self) -> str:
        return f"yolo {self.task} ({self.weights})"


# ============================================================= any function
class FunctionPicker(Picker):
    """
    Wrap ANY model you already have:

        def my_model(image):                  # numpy BGR
            ...
            return [(u, v), (u, v, "label"), (u, v, "label", 0.93)]

        ai = FunctionPicker(my_model)
    """

    name = "function"

    def __init__(self, fn, name=None):
        self.fn = fn
        self.name = name or getattr(fn, "__name__", "function")

    def find(self, image) -> list:
        out = []
        for i, item in enumerate(self.fn(_image_of(image)) or [], 1):
            if isinstance(item, Pixel):
                out.append(item)
                continue
            u, v = float(item[0]), float(item[1])
            label = str(item[2]) if len(item) > 2 else f"point{i}"
            c = float(item[3]) if len(item) > 3 else 1.0
            out.append(Pixel(u, v, None, label=label, confidence=c))
        return out


# ========================================================== a person clicks
class ClickPicker(Picker):
    """
    The original workflow. Left click = add, right click = undo,
    ENTER = done, ESC = cancel (returns nothing).
    Still useful after the model exists: click a weld yourself and you have
    separated a vision problem from a calibration problem in thirty seconds.
    """

    name = "clicks"

    def __init__(self, expect=None, window="click the points", max_width=1280):
        self.expect, self.window, self.max_width = expect, window, int(max_width)

    def find(self, image) -> list:
        import cv2
        img = _image_of(image)
        h, w = img.shape[:2]
        scale = min(1.0, self.max_width / float(w))
        shown = cv2.resize(img, None, fx=scale, fy=scale) if scale < 1 else img.copy()
        picked: list = []

        def redraw():
            canvas = shown.copy()
            for i, (u, v) in enumerate(picked, 1):
                p = (int(u * scale), int(v * scale))
                cv2.circle(canvas, p, 7, (0, 255, 0), 2)
                cv2.putText(canvas, str(i), (p[0] + 10, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(canvas, f"{len(picked)} points  ENTER=done  "
                                f"right=undo  ESC=cancel", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(self.window, canvas)

        def on_mouse(event, x, y, *_):
            if event == cv2.EVENT_LBUTTONDOWN:
                picked.append((x / scale, y / scale))
                redraw()
            elif event == cv2.EVENT_RBUTTONDOWN and picked:
                picked.pop()
                redraw()

        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.window, on_mouse)
        redraw()
        cancelled = False
        try:
            while True:
                key = cv2.waitKey(30) & 0xFF
                if key in (13, 10):
                    break
                if key == 27 or cv2.getWindowProperty(
                        self.window, cv2.WND_PROP_VISIBLE) < 1:
                    cancelled = True
                    break
                if self.expect and len(picked) >= self.expect:
                    break
        finally:
            cv2.destroyWindow(self.window)
            cv2.waitKey(1)
        if cancelled:
            return []
        return [Pixel(u, v, None, label=f"point{i}") for i, (u, v) in
                enumerate(picked, 1)]


# ============================================================ for testing
class FixedPoints(Picker):
    """Always the same pixels -- a full cycle on a laptop, no model needed."""

    name = "fixed"

    def __init__(self, pixels=((640, 360), (740, 380), (540, 380))):
        self.pixels = [tuple(p) for p in pixels]

    def find(self, image) -> list:
        return [Pixel(float(p[0]), float(p[1]), None,
                      label=str(p[2]) if len(p) > 2 else f"point{i}")
                for i, p in enumerate(self.pixels, 1)]


def draw(image, pixels, path=None):
    """The picked points drawn on a copy of the image; saved when path is given."""
    import cv2
    canvas = _image_of(image).copy()
    for i, px in enumerate(pixels, 1):
        p = (int(px.u), int(px.v))
        cv2.circle(canvas, p, 8, (0, 255, 0), 2)
        cv2.putText(canvas, f"{i} {px.label}", (p[0] + 10, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    if path:
        cv2.imwrite(str(path), canvas)
    return canvas
