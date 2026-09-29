"""
Intel RealSense D4xx -- the D435I on this cell.

Two details here, both learned from a camera that rides a moving arm:

PROFILES ARE NEGOTIATED, NOT ASSUMED. "Couldn't resolve requests" means the
exact size/fps you asked for is not on this model's list -- and that list
differs between a D405, a D435 and a D455, between firmware versions, and
between USB 3 and a port a long cable has quietly degraded to USB 2. (This
cell's D435I runs at USB 2.1, which is why 1280x720 only offers 6 fps.) Asking
the device what it supports turns a guess into a fact.

DEPTH IS ALIGNED TO COLOUR, ALWAYS. Detection happens on the colour image, and
a depth reading at the same pixel of an unaligned image is a reading of
something else entirely -- the two sensors are centimetres apart.
"""

from __future__ import annotations

import numpy as np

from core import CellError

from .base import Camera, Frame, Lens


class RealSense(Camera):
    """
        RealSense()                      pick the best colour+depth profile
        RealSense(width=1280, height=720, fps=6)      ask for exactly this
        RealSense(serial="123456789")    when two are plugged in
    """

    name = "realsense"
    has_depth = True

    def __init__(self, serial=None, width=None, height=None, fps=None,
                 prefer_max_width=1280, patch=5, warmup_frames=5):
        self.serial = serial
        self.want = (width, height, fps)
        self.prefer_max_width = prefer_max_width
        self.patch = patch
        self.warmup_frames = warmup_frames
        self._pipe = self._align = self._lens = None
        self.profile_used = None

    # -- SDK ---------------------------------------------------------------
    @staticmethod
    def _sdk():
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise CellError("pyrealsense2 is not installed on this PC "
                            "(pip install pyrealsense2)") from e
        return rs

    def _options(self):
        """Every colour+depth profile this device actually offers, best first."""
        rs = self._sdk()
        devices = list(rs.context().query_devices())
        if not devices:
            raise CellError(
                "no RealSense is visible to this PC at all.\n"
                "  1. close realsense-viewer -- it holds the camera exclusively\n"
                "  2. kill any other python still holding the pipeline\n"
                "  3. unplug and replug, into a blue / SS port")
        dev = devices[0]
        if self.serial:
            dev = next((d for d in devices
                        if d.get_info(rs.camera_info.serial_number) == self.serial),
                       None)
            if dev is None:
                have = [d.get_info(rs.camera_info.serial_number) for d in devices]
                raise CellError(f"no RealSense with serial {self.serial}; "
                                f"this PC sees {have}")

        color, depth = set(), set()
        for sensor in dev.sensors:
            for p in sensor.get_stream_profiles():
                if not p.is_video_stream_profile():
                    continue
                v = p.as_video_stream_profile()
                key = (v.width(), v.height(), p.fps())
                if p.stream_type() == rs.stream.color and p.format() == rs.format.bgr8:
                    color.add(key)
                elif p.stream_type() == rs.stream.depth and p.format() == rs.format.z16:
                    depth.add(key)
        if not color or not depth:
            raise CellError("the camera reports no usable colour+depth profiles")

        w, h, f = self.want
        if w and h and (w, h, f or 30) in color & depth:
            return [(w, h, f or 30)]
        rank = lambda t: (t[0] > self.prefer_max_width, -(t[0] * t[1]), -t[2])
        both = sorted(color & depth, key=rank)
        return both or [sorted(color, key=rank)[0]]

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        rs = self._sdk()
        last = None
        for (w, h, fps) in self._options()[:4]:
            pipe = rs.pipeline()
            cfg = rs.config()
            if self.serial:
                cfg.enable_device(self.serial)
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
            try:
                profile = pipe.start(cfg)
                for _ in range(self.warmup_frames):   # first frames are exposure
                    pipe.wait_for_frames(5000)
                i = (profile.get_stream(rs.stream.color)
                     .as_video_stream_profile().get_intrinsics())
                self._lens = Lens(i.width, i.height, i.fx, i.fy, i.ppx, i.ppy,
                                  tuple(i.coeffs))
                self._pipe, self._align = pipe, rs.align(rs.stream.color)
                self.profile_used = (w, h, fps)
                print(f"  camera: colour+depth {w}x{h}@{fps}")
                return
            except Exception as e:
                last = e
                print(f"  {w}x{h}@{fps}: {e}")
                try:
                    pipe.stop()
                except Exception:
                    pass
        raise CellError(f"no supported profile delivered frames ({last})")

    def close(self) -> None:
        if self._pipe is not None:
            try:
                self._pipe.stop()
            except Exception:
                pass
            self._pipe = None

    def read(self, timeout_s: float = 5.0) -> Frame:
        if self._pipe is None:
            raise CellError("the camera has not been opened -- call camera.start()")
        try:
            frames = self._align.process(
                self._pipe.wait_for_frames(int(timeout_s * 1000)))
        except Exception as e:
            raise CellError(f"no frame within {timeout_s:.0f} s: {e}") from e
        cf, df = frames.get_color_frame(), frames.get_depth_frame()
        if not cf:
            raise CellError("a frame arrived with no colour image")
        depth = None
        if df:
            scale = df.get_units() if hasattr(df, "get_units") else 0.001
            depth = np.asanyarray(df.get_data()).astype(np.float32) * float(scale)
        return Frame(np.asanyarray(cf.get_data()), depth, self._lens)

    @property
    def lens(self) -> Lens:
        if self._lens is None:
            raise CellError("the lens model is only known after the camera opens")
        return self._lens
