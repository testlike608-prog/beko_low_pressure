"""
The camera side of the cell, behind two functions.

    import camera

    camera.adopt(cam)            open a camera someone else built  <- main.py
    camera.start()               or build one from settings.CAMERA
    frame = camera.read()        one colour (+depth) frame
    camera.stop()                release the device

Everything else in the project talks to the camera through this module, so
swapping a RealSense for a Hikrobot or a GigE camera never reaches cell.py, the
model or the arm.

WHO PICKS THE CAMERA: main.py does, in its own CAMERA block, which builds the
camera out of camera_hub.py and hands it here with adopt(). start() is the
other route -- settings.CAMERA -- and it is what the tests use.

ADDING A CAMERA (three steps, one new file):

    0. or, if the driver already exists in camera_hub.py, skip all of this and
       set main.py's CAMERA = {"backend": "realsense"}
    1. copy camera/webcam.py to camera/mycam.py and fill in
       open() / close() / read() / lens
    2. add one line to REGISTRY below
    3. name it: main.py's CAMERA = {"backend": "mycam", ...}

Nothing else -- not the robot, not the vision model, not cell.py -- is touched.
"""

from __future__ import annotations

import time
from pathlib import Path

from core import CellError

from .base import Camera, Frame, Lens

# ---------------------------------------------------------------- registry
#: kind -> (module, class). Imported lazily, so a PC with no pyrealsense2
#: installed can still run the fake camera and the whole test suite.
REGISTRY = {
    "realsense": ("camera.realsense", "RealSense"),
    "webcam": ("camera.webcam", "Webcam"),
    "fake": ("camera.fake", "Fake"),
    # camera_hub.py at the project root -- its RealSense / OpenCV / UseePlus
    # drivers, wrapped. See camera/hub.py for why read() is not get_frame().
    "hub": ("camera.hub", "Hub"),
}


def kinds() -> list[str]:
    return sorted(REGISTRY)


def build(config: dict | None = None) -> Camera:
    """
    A config dict -> a Camera object, without opening it.

        build({"kind": "realsense", "width": 1280, "height": 720, "fps": 6})

    Used by start(), and directly by tests that want two cameras at once.
    """
    if config is None:
        import settings
        config = settings.CAMERA
    config = dict(config)
    kind = config.pop("kind", "fake")
    if kind not in REGISTRY:
        raise CellError(f"unknown camera kind {kind!r}. "
                        f"Known kinds: {', '.join(kinds())}")
    module_name, class_name = REGISTRY[kind]
    import importlib
    try:
        module = importlib.import_module(module_name)
    except ImportError as e:
        raise CellError(f"the {kind} driver could not be imported: {e}") from e
    try:
        return getattr(module, class_name)(**config)
    except TypeError as e:
        raise CellError(f"{kind} does not accept these settings: {e}") from e


# ------------------------------------------------------------- the singleton
_camera: Camera | None = None


def start(config: dict | None = None) -> Camera:
    """
    Open the camera. Safe to call twice -- the second call is a no-op.

    Returns the Camera object for anyone who wants it, but the usual code never
    needs it: camera.read() is enough.
    """
    return adopt(build(config))


def adopt(cam: Camera) -> Camera:
    """
    Open a Camera the CALLER built, and serve it from this module.

    start() is the settings.py route. This is the other one: main.py builds its
    camera itself -- straight out of camera_hub.py -- and hands it over here so
    that cell.py's camera.read() still finds it. Same singleton, same stop(),
    the choice just happened somewhere visible instead of in a config dict.

    Safe to call twice: the second call keeps the first camera and returns it,
    exactly like start().
    """
    global _camera
    if _camera is not None:
        return _camera
    print(f"  opening {cam.name} ...")
    cam.open()
    _camera = cam
    return cam


def stop() -> None:
    """
    Close the camera and forget it. Safe to call when nothing was started, and
    safe to call twice -- which is what makes it usable in a `finally:`.
    """
    global _camera
    if _camera is None:
        return
    try:
        _camera.close()
        print(f"  {_camera.name} closed")
    except Exception as e:
        print(f"  !! closing the camera raised: {e}")
    finally:
        _camera = None


def current() -> Camera:
    if _camera is None:
        raise CellError("the camera has not been started -- call camera.start()")
    return _camera


def is_running() -> bool:
    return _camera is not None


def read(timeout_s: float = 5.0) -> Frame:
    """One frame from the started camera."""
    return current().read(timeout_s)


def to_points(frame: Frame, pixels):
    """Detections -> camera-frame points. Returns (points, dropped)."""
    return current().to_points(frame, pixels)


# --------------------------------------------------------------------- tools
def save(frame: Frame, folder="captures", stem=None) -> Path:
    """
    Write a frame to disk as image + .npy depth, ready to be replayed by the
    fake camera. This is how a real capture becomes a permanent test case.
    """
    import cv2
    import numpy as np

    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    stem = stem or time.strftime("%Y%m%d-%H%M%S")
    image_path = out / f"{stem}.png"
    cv2.imwrite(str(image_path), frame.color)
    if frame.depth is not None:
        np.save(out / f"{stem}.npy", frame.depth)
    return image_path


def describe(config: dict | None = None) -> str:
    """One line about the configured camera, without opening it."""
    if _camera is not None:
        return _camera.describe()
    try:
        return build(config).describe() + " (not open)"
    except CellError as e:
        return f"camera misconfigured: {e}"


__all__ = ["Camera", "Frame", "Lens", "REGISTRY", "kinds", "build",
           "start", "adopt", "stop", "current", "is_running", "read", "to_points",
           "save", "describe"]
