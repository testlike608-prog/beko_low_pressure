"""
camera_hub.py as a camera of this project.

One file, so every driver in camera_hub -- RealSense, OpenCV, UseePlus, and
whatever you add there next -- becomes usable here without either side knowing
about the other. camera_hub stays a standalone module you can copy to any
project; this file is the only place that knows both shapes.

    CAMERA = {"kind": "hub", "backend": "realsense"}
    CAMERA = {"kind": "hub", "backend": "useeplus", "fov_deg": 70,
              "assume_depth_m": 0.25}

THE ONE THING THIS FILE EXISTS TO FIX -- STALE FRAMES.

camera_hub is push: a background thread grabs frames forever and get_frame()
hands back the newest one. This project is pull: cell.py moves the arm to the
capture pose and THEN asks for a frame. Put the two together naively and the
frame you get can be one the thread grabbed while the arm was still moving --
a sharp, correct-looking image of the wrong place.

The pose guard in cell.py does not catch it. That guard compares the flange
pose before and after the read; by then the arm is parked and both readings
agree, so a frame from two seconds ago passes cleanly.

So read() never takes whatever is lying there. It waits for a frame NEWER than
the last one it returned -- camera_hub.frame_seq counts them -- and raises if
none arrives in time. A frozen stream becomes a timeout you can see instead of
a wrong number you cannot.
"""

from __future__ import annotations

from core import CellError

from .base import Camera, Frame, Lens


class Hub(Camera):
    """
        Hub()                                   RealSense through camera_hub
        Hub(backend="opencv", camera_index=0, fov_deg=68, assume_depth_m=0.45)
        Hub(backend="useeplus", upscale=False)  extra kwargs go to the driver

    Anything this class does not recognise is passed straight to the camera_hub
    driver, so a knob that exists there needs no edit here to be usable.
    """

    name = "hub"
    has_depth = False          # decided per backend in open()

    #: settings name -> the attribute on CameraHub. Adding a driver over there
    #: is one line here, and nothing else in this project changes.
    BACKENDS = {
        "realsense": "RealSense",
        "opencv": "OpenCV",
        "useeplus": "UseePlus",
    }

    #: Which of them carry depth. Used ONLY by describe() before the device is
    #: opened -- once it is open, the answer comes from the device itself.
    DEPTH_BACKENDS = {"realsense"}

    def __init__(self, backend="realsense", camera_index=0, lens=None,
                 fov_deg=None, assume_depth_m=None, patch=5, warmup_s=10.0,
                 **backend_kwargs):
        """
        backend         : which camera_hub driver -- see BACKENDS
        lens            : explicit intrinsics dict, wins over everything.
                          {"width":.., "height":.., "fx":.., "fy":..,
                           "cx":.., "cy":..}
        fov_deg         : horizontal field of view, for a 2D camera with no
                          calibration. An ESTIMATE -- it says so at startup.
        assume_depth_m  : the work sits at this fixed range (2D cameras)
        warmup_s        : how long to wait for the first frame after start()
        """
        self.backend = backend
        self.camera_index = camera_index
        self.lens_config = lens
        self.fov_deg = fov_deg
        self.assume_depth_m = assume_depth_m
        self.patch = patch
        self.warmup_s = warmup_s
        self.backend_kwargs = backend_kwargs

        self._hub = None
        self._lens = None
        self._last_seq = 0

    # -- the module, imported late so this file loads on a PC without it ----
    @staticmethod
    def _camera_hub():
        try:
            from camera_hub import CameraHub
        except ImportError as e:
            raise CellError(
                "camera_hub.py was not found. Put it beside main.py "
                "(it is a single file -- copy it in)."
            ) from e
        return CameraHub

    def _driver_class(self):
        CameraHub = self._camera_hub()
        if self.backend not in self.BACKENDS:
            raise CellError(
                f"unknown camera_hub backend {self.backend!r}. "
                f"Known: {', '.join(sorted(self.BACKENDS))}")
        cls = getattr(CameraHub, self.BACKENDS[self.backend], None)
        if cls is None:
            raise CellError(
                f"camera_hub has no {self.BACKENDS[self.backend]} interface. "
                "An older copy of camera_hub.py, most likely.")
        return cls

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        cls = self._driver_class()
        try:
            hub = cls(camera_index=self.camera_index, **self.backend_kwargs)
        except TypeError as e:
            raise CellError(
                f"the {self.backend} driver does not accept these settings: {e}"
            ) from e

        hub.start()
        if not hub.wait_for_frame(timeout=self.warmup_s):
            hub.stop()
            raise CellError(
                f"the {self.backend} camera delivered no frame in "
                f"{self.warmup_s:.0f} s. The driver logged why -- usually the "
                "device is held by another program, or not plugged in.")

        self._hub = hub
        self._last_seq = hub.frame_seq
        self.has_depth = getattr(hub, "get_depth", lambda: None)() is not None
        self._lens = self._build_lens(hub)

        kind = "colour+depth" if self.has_depth else "colour"
        extra = f" {hub.profile}" if getattr(hub, "profile", None) else ""
        print(f"  camera: {self.backend} via camera_hub, {kind}{extra}")

    def _build_lens(self, hub) -> Lens:
        """
        Intrinsics, best source first. The order matters: a number measured by
        the device beats a number you typed, and both beat a number derived
        from a field-of-view figure off a spec sheet.
        """
        if self.lens_config:
            return Lens(**self.lens_config)

        k = getattr(hub, "intrinsics", None)
        if k:
            return Lens(k["width"], k["height"], k["fx"], k["fy"],
                        k["cx"], k["cy"], tuple(k.get("distortion") or ()) or
                        (0.0, 0.0, 0.0, 0.0, 0.0))

        frame = hub.get_frame()
        if frame is None:
            raise CellError("no frame to size the lens model from")
        h, w = frame.shape[:2]

        if self.fov_deg:
            import math
            fx = (w / 2.0) / math.tan(math.radians(self.fov_deg) / 2.0)
            print(f"  !! {self.backend}: no intrinsics from the device -- "
                  f"estimating fx={fx:.0f} from fov_deg={self.fov_deg}. "
                  "Millimetres from this camera are approximate.")
            return Lens(w, h, fx, fx, w / 2.0, h / 2.0)

        raise CellError(
            f"the {self.backend} backend reports no intrinsics, so this "
            "project cannot turn a pixel into millimetres. Give the camera "
            'either "lens": {"width":.., "height":.., "fx":.., "fy":.., '
            '"cx":.., "cy":..} or, as a rough stand-in, "fov_deg": 68.')

    def close(self) -> None:
        if self._hub is not None:
            try:
                self._hub.stop()
            finally:
                self._hub = None

    def read(self, timeout_s: float = 5.0) -> Frame:
        if self._hub is None:
            raise CellError("the camera has not been opened -- call camera.start()")

        # A NEW frame, never whatever the thread happens to be holding.
        if not self._hub.wait_for_new_frame(self._last_seq, timeout_s):
            running = self._hub.is_running()
            raise CellError(
                f"no new frame from the {self.backend} camera within "
                f"{timeout_s:.0f} s "
                + ("-- the capture thread has stopped; see the log above."
                   if not running else
                   "-- the stream is frozen while the thread is still alive."))

        if hasattr(self._hub, "get_frames"):
            color, depth = self._hub.get_frames()
        else:
            color, depth = self._hub.get_frame(), None
        self._last_seq = self._hub.frame_seq

        if color is None:
            raise CellError("a read returned no colour image")
        return Frame(color, depth, self._lens)

    @property
    def lens(self) -> Lens:
        if self._lens is None:
            raise CellError("the lens model is only known after the camera opens")
        return self._lens

    def describe(self) -> str:
        if self._hub is not None:
            p = getattr(self._hub, "profile", None)
            return (f"{self.backend} via camera_hub "
                    f"({self._depth_phrase(self.has_depth)}"
                    f"{f', {p[0]}x{p[1]}@{p[2]}' if p else ''})")
        # Not open yet, so the device has not been asked. Say what this backend
        # is known to carry rather than reporting the not-yet-set default as
        # fact -- "2D, no depth" about a RealSense is a wrong answer, not a
        # cautious one.
        return (f"{self.backend} via camera_hub "
                f"({self._depth_phrase(self.backend in self.DEPTH_BACKENDS)})")

    def _depth_phrase(self, has_depth: bool) -> str:
        if has_depth:
            return "colour + depth"
        if self.assume_depth_m:
            return f"2D, assuming {self.assume_depth_m} m"
        return "2D, no depth"
