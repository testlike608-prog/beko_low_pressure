"""
The vision side of the cell, behind the same two functions as everything else.

    import vision

    vision.start()                 load the model (or set up the click window)
    pixels = vision.find(frame)    one image -> welding points, in PIXELS
    vision.stop()

`look()` is the step above: it takes a camera frame AND the flange pose, and
returns a Batch of camera-frame points with the flange pose attached -- which
is the only form the hand-eye transform will accept.

ADDING A DETECTOR (three steps, one new file):

    1. copy vision/fake.py to vision/mymodel.py and implement find(frame)
    2. add one line to REGISTRY below
    3. set VISION = {"kind": "mymodel", ...} in settings.py
"""

from __future__ import annotations

import numpy as np

from core import Batch, CellError, Pose

from .base import Detector, drop_duplicates, sort_reading_order

# ---------------------------------------------------------------- registry
REGISTRY = {
    "yolo": ("vision.yolo", "Yolo"),
    "clicks": ("vision.clicks", "Clicks"),
    "fixed": ("vision.fake", "Fixed"),
    "file": ("vision.fake", "FromFile"),
}


def kinds() -> list[str]:
    return sorted(REGISTRY)


def build(config: dict | None = None) -> Detector:
    if config is None:
        import settings
        config = settings.VISION
    config = dict(config)
    kind = config.pop("kind", "fixed")
    if kind not in REGISTRY:
        raise CellError(f"unknown detector kind {kind!r}. "
                        f"Known kinds: {', '.join(kinds())}")
    module_name, class_name = REGISTRY[kind]
    import importlib
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise CellError(f"the {kind} detector could not be imported: {e}") from e
    try:
        return getattr(module, class_name)(**config)
    except TypeError as e:
        raise CellError(f"{kind} does not accept these settings: {e}") from e


# ------------------------------------------------------------- the singleton
_detector: Detector | None = None


def start(config: dict | None = None) -> Detector:
    global _detector
    if _detector is not None:
        return _detector
    det = build(config)
    det.open()
    _detector = det
    return det


def stop() -> None:
    global _detector
    if _detector is None:
        return
    try:
        _detector.close()
    except Exception as e:
        print(f"  !! closing the detector raised: {e}")
    finally:
        _detector = None


def current() -> Detector:
    if _detector is None:
        raise CellError("the detector has not been started -- call vision.start()")
    return _detector


def is_running() -> bool:
    return _detector is not None


def find(frame) -> list:
    """One frame -> pixels. No millimetres, no robot."""
    return current().find(frame)


# --------------------------------------------------------------------- look
def look(frame, flange: Pose | None, camera=None) -> Batch:
    """
    The whole vision step: detect, deproject, attach the flange pose.

        batch = vision.look(frame, robot.flange())
        points = handeye.to_base(batch)

    `flange` is read by the CALLER, immediately around the capture, and travels
    with the batch from here on. Passing it in rather than reading it inside is
    deliberate: it makes it obvious at the call site that the pose has to come
    from the moment of the shutter, not from a second later.
    """
    import camera as camera_module
    cam = camera or camera_module.current()

    pixels = find(frame)
    points, dropped = cam.to_points(frame, pixels)
    points = _attach_normals(points, pixels, frame, cam)

    batch = Batch(points=points, flange=flange, dropped=dropped,
                  meta={"detector": current().name,
                        "camera": cam.name,
                        "image": list(frame.size)})
    if dropped:
        print(f"  !! {len(dropped)} of {len(pixels)} detections had no usable "
              f"depth and were skipped: "
              + ", ".join(str(p) for p in dropped[:4])
              + (" ..." if len(dropped) > 4 else ""))
    return batch


def _attach_normals(points, pixels, frame, cam):
    """
    Turn a detector's 2D axis hint into a real 3D direction.

    A pose model marks the mouth and the far end of the service port. Both are
    pixels; each needs its own depth before the line between them means
    anything in space. When either depth is missing the point simply keeps no
    normal -- a made-up axis would send the sniffer into the rubber at an angle
    and bend it.
    """
    by_index = {}
    for point in points:
        uv = point.meta.get("uv")
        if uv:
            by_index[(round(uv[0], 1), round(uv[1], 1))] = point

    out = []
    for point in points:
        axis = point.meta.get("axis_uv")
        if not axis:
            out.append(point)
            continue
        u1, v1, u2, v2 = axis
        d1 = cam.depth_at(frame, u1, v1)
        d2 = cam.depth_at(frame, u2, v2)
        if not d1 or not d2:
            out.append(point)
            continue
        lens = frame.lens or cam.lens
        a = np.array(lens.deproject(u1, v1, d1 * 1000.0))
        b = np.array(lens.deproject(u2, v2, d2 * 1000.0))
        d = b - a
        n = float(np.linalg.norm(d))
        out.append(point.with_normal(d / n) if n > 1e-6 else point)
    return out


def describe(config: dict | None = None) -> str:
    if _detector is not None:
        return _detector.describe()
    try:
        return build(config).describe() + " (not loaded)"
    except CellError as e:
        return f"vision misconfigured: {e}"


__all__ = ["Detector", "REGISTRY", "kinds", "build", "start", "stop",
           "current", "is_running", "find", "look", "describe",
           "sort_reading_order", "drop_duplicates"]
