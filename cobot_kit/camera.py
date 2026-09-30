"""
The camera, behind start() and stop().

    from cobot_kit import Camera

    camera = Camera()                       # settings.CAMERA (RealSense via camera_hub)
    camera = Camera(backend="fake")         # no hardware
    camera.start()
    frame = camera.read()                   # a FRESH colour + depth frame
    shot  = camera.shoot(robot, name="pos1")  # frame + flange pose + saved to capture/
    camera.stop()

A Shot is the unit the rest of the kit works with: the image the AI looks at,
the depth that turns its pixels into millimetres, and the flange pose from the
instant of the picture -- which the hand-eye maths needs, because the camera
rides the arm.

camera_hub.py (copied as-is) holds the real drivers. This file adds the one
thing a PULL cell needs from a PUSH camera: read() waits for a frame NEWER than
the last one, so a picture taken while the arm was still moving can never be
handed back after it stops.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import settings
from .datatypes import CellError, Pose


# ------------------------------------------------------------------ data
@dataclass
class Lens:
    """Pinhole model, plain numbers."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    def deproject(self, u, v, depth_mm):
        """Pixel + depth -> (x, y, z) mm in the camera frame."""
        return ((float(u) - self.cx) / self.fx * depth_mm,
                (float(v) - self.cy) / self.fy * depth_mm,
                float(depth_mm))

    def project(self, xyz):
        x, y, z = (float(v) for v in xyz)
        if z <= 1e-6:
            raise CellError("cannot project a point at or behind the lens")
        return (x / z * self.fx + self.cx, y / z * self.fy + self.cy)


@dataclass
class Frame:
    """One instant. `color` is BGR, `depth` is METRES aligned to colour (or None)."""

    color: np.ndarray | None = None
    depth: np.ndarray | None = None
    lens: Lens | None = None
    when: float = field(default_factory=time.time)

    @property
    def size(self):
        if self.color is None:
            return (0, 0)
        h, w = self.color.shape[:2]
        return (w, h)

    def depth_at(self, u, v, patch=5):
        """
        Depth in metres at a pixel, or None. Median over a patch, zeros ignored:
        a zero means "no reading" (shiny weld, shadow), not "at the lens", and
        averaging it in drags a good point towards the camera.
        """
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        r = max(0, patch // 2)
        window = self.depth[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1]
        good = window[window > 0]
        if good.size < max(3, (2 * r + 1) ** 2 // 4):
            return None
        return float(np.median(good))


@dataclass
class Shot:
    """One picture from one capture position -- everything later steps need."""

    frame: Frame
    flange: Pose | None            # base <- flange at the moment of the picture
    name: str = ""
    image_path: Path | None = None
    pixels: list = field(default_factory=list)   # filled by the AI step
    points: list = field(default_factory=list)   # filled by the locate step

    @property
    def image(self):
        return self.frame.color


# ---------------------------------------------------------------- camera
class Camera:
    """
        Camera()                                    settings.CAMERA
        Camera(backend="realsense", fps=6)          override any value
        Camera(backend="opencv", camera_index=0, fov_deg=68, assume_depth_m=0.45)
        Camera(backend="fake")                      synthetic scene, no hardware
        Camera(backend="fake", folder="capture")    replay saved shots
    """

    HUB_BACKENDS = {"realsense": "RealSense", "opencv": "OpenCV",
                    "useeplus": "UseePlus"}

    def __init__(self, backend=None, capture_folder=None, lens=None,
                 fov_deg=None, assume_depth_m=None, patch=5, warmup_s=10.0,
                 folder=None, depth_m=0.45, **driver_options):
        config = dict(settings.CAMERA)
        if backend is not None and backend != config.get("backend"):
            # Switching backend: the settings' driver knobs are for the other one.
            config = {"camera_index": config.get("camera_index", 0)}
        config.update(driver_options)
        config.pop("backend", None)
        self.backend = backend or settings.CAMERA.get("backend", "fake")
        self.driver_options = config
        self.capture_folder = capture_folder or settings.CAPTURE_FOLDER
        self.lens_config, self.fov_deg = lens, fov_deg
        self.assume_depth_m, self.patch, self.warmup_s = assume_depth_m, patch, warmup_s
        self.fake_folder, self.fake_depth_m = folder, float(depth_m)
        self._hub = None
        self._lens = None
        self._last_seq = 0
        self._fake_files, self._fake_i = None, 0
        self._running = False

    # ------------------------------------------------------------ lifecycle
    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> "Camera":
        if self._running:
            return self
        if self.backend == "fake":
            self._start_fake()
        elif self.backend in self.HUB_BACKENDS:
            self._start_hub()
        else:
            raise CellError(f"unknown camera backend {self.backend!r}. Use one "
                            f"of: fake, {', '.join(self.HUB_BACKENDS)}")
        self._running = True
        return self

    def stop(self) -> None:
        """Release the device. Safe twice, safe without start(). Never raises."""
        if not self._running:
            return
        self._running = False
        if self._hub is not None:
            try:
                self._hub.stop()
            except Exception as e:
                print(f"  !! closing the camera raised: {e}")
            self._hub = None
        print(f"  camera {self.backend} closed")

    # ----------------------------------------------------------- hub driver
    def _start_hub(self):
        try:
            from .camera_hub import CameraHub
        except ImportError as e:
            raise CellError(f"camera_hub could not be imported: {e}") from e
        cls = getattr(CameraHub, self.HUB_BACKENDS[self.backend], None)
        if cls is None:
            raise CellError(f"this camera_hub.py has no {self.backend} driver")
        try:
            hub = cls(**self.driver_options)
        except TypeError as e:
            raise CellError(f"the {self.backend} driver does not accept "
                            f"these settings: {e}") from e
        hub.start()
        if not hub.wait_for_frame(timeout=self.warmup_s):
            hub.stop()
            raise CellError(
                f"the {self.backend} camera delivered no frame in "
                f"{self.warmup_s:.0f} s -- held by another program "
                f"(realsense-viewer?) or not plugged in.")
        self._hub = hub
        self._last_seq = hub.frame_seq
        self._lens = self._hub_lens(hub)
        depth = getattr(hub, "get_depth", lambda: None)() is not None
        prof = getattr(hub, "profile", None)
        print(f"  camera: {self.backend}, {'colour+depth' if depth else 'colour'}"
              + (f" {prof[0]}x{prof[1]}@{prof[2]}" if prof else ""))

    def _hub_lens(self, hub) -> Lens:
        """Best source first: typed-in > measured by the device > from a FOV figure."""
        if self.lens_config:
            return Lens(**self.lens_config)
        k = getattr(hub, "intrinsics", None)
        if k:
            return Lens(k["width"], k["height"], k["fx"], k["fy"], k["cx"], k["cy"],
                        tuple(k.get("distortion") or ()) or (0.0,) * 5)
        img = hub.get_frame()
        h, w = img.shape[:2]
        if self.fov_deg:
            fx = (w / 2.0) / math.tan(math.radians(self.fov_deg) / 2.0)
            print(f"  !! lens ESTIMATED from fov_deg={self.fov_deg} -- "
                  f"millimetres from this camera are approximate")
            return Lens(w, h, fx, fx, w / 2.0, h / 2.0)
        raise CellError(f"the {self.backend} camera reports no intrinsics -- "
                        f"give Camera(lens={{...}}) or Camera(fov_deg=68)")

    # ---------------------------------------------------------- fake driver
    def _start_fake(self):
        # Same numbers as the D435I on this cell, so fake mm ~ real mm.
        self._lens = Lens(1280, 720, 911.70, 911.85, 644.02, 368.64)
        if self.fake_folder:
            from .datatypes import find_file
            folder = find_file(self.fake_folder, "frame folder")
            self._fake_files = sorted(
                p for p in folder.iterdir()
                if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
            if not self._fake_files:
                raise CellError(f"{folder} holds no images")
            print(f"  camera: fake, replaying {len(self._fake_files)} images")
        else:
            print("  camera: fake, synthetic scene")

    def _read_fake(self) -> Frame:
        lens = self._lens
        if not self._fake_files:
            w, h = lens.width, lens.height
            color = np.full((h, w, 3), 60, np.uint8)
            color[h // 3:2 * h // 3, w // 3:2 * w // 3] = 140
            return Frame(color, np.full((h, w), self.fake_depth_m, np.float32), lens)
        import cv2
        path = self._fake_files[self._fake_i % len(self._fake_files)]
        self._fake_i += 1
        color = cv2.imread(str(path))
        if color is None:
            raise CellError(f"{path} could not be read")
        npy = path.with_suffix(".npy")
        depth = (np.load(npy).astype(np.float32) if npy.exists()
                 else np.full(color.shape[:2], self.fake_depth_m, np.float32))
        h, w = color.shape[:2]
        if (w, h) != (lens.width, lens.height):
            sx, sy = w / lens.width, h / lens.height
            lens = Lens(w, h, lens.fx * sx, lens.fy * sy, lens.cx * sx, lens.cy * sy)
        return Frame(color, depth, lens)

    # --------------------------------------------------------------- reading
    @property
    def lens(self) -> Lens:
        if self._lens is None:
            raise CellError("the lens is only known after camera.start()")
        return self._lens

    def read(self, timeout_s: float = 5.0) -> Frame:
        """One FRESH frame -- never one grabbed before this call."""
        if not self._running:
            raise CellError("the camera is not started -- call camera.start()")
        if self._hub is None:
            return self._read_fake()
        hub = self._hub
        if not hub.wait_for_new_frame(self._last_seq, timeout_s):
            raise CellError(f"no new frame from the {self.backend} camera within "
                            f"{timeout_s:.0f} s")
        if hasattr(hub, "get_frames"):
            color, depth = hub.get_frames()
        else:
            color, depth = hub.get_frame(), None
        self._last_seq = hub.frame_seq
        if color is None:
            raise CellError("a read returned no colour image")
        if depth is None and self.assume_depth_m:
            depth = np.full(color.shape[:2], float(self.assume_depth_m), np.float32)
        return Frame(color, depth, self._lens)

    def save(self, frame: Frame, name: str | None = None, folder=None) -> Path:
        """image.png (+ image.npy depth) -- replayable later with backend='fake'."""
        import cv2
        out = Path(folder or self.capture_folder)
        if not out.is_absolute():
            out = _app_folder() / out
        out.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("%Y%m%d-%H%M%S") + (f"_{name}" if name else "")
        path = out / f"{stem}.png"
        cv2.imwrite(str(path), frame.color)
        if frame.depth is not None:
            np.save(out / f"{stem}.npy", frame.depth)
        return path

    def shoot(self, robot=None, name: str = "", save: bool = True,
              max_drift_mm: float = 1.0) -> Shot:
        """
        Take the picture the rest of the cycle uses.

        The flange pose is read right before AND right after the frame. If the
        arm moved more than max_drift_mm in between, the picture was taken while
        something was still moving and every point from it would be off by about
        that much -- so it raises instead of quietly producing wrong points.
        """
        before = robot.flange() if robot is not None else None
        frame = self.read()
        if robot is not None:
            drift = before.distance_to(robot.flange())
            if drift > max_drift_mm:
                raise CellError(f"the arm moved {drift:.1f} mm during the picture "
                                f"-- raise settings.SETTLE_S and look again")
        shot = Shot(frame, before, name=name)
        if save:
            try:
                shot.image_path = self.save(frame, name)
            except Exception as e:
                print(f"  !! the picture could not be saved: {e}")
        return shot

    def describe(self) -> str:
        return f"camera {self.backend}" + ("" if self._running else " (not open)")

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def _app_folder() -> Path:
    """The folder app.py lives in = the one this kit sits inside."""
    return Path(__file__).resolve().parent.parent
