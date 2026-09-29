"""
Any camera OpenCV can open: a USB webcam, a GigE stream, an RTSP URL, a file.

This is here as the worked EXAMPLE of adding a second camera -- it is about
120 lines and touches nothing outside this file.

It has no depth, and that is stated rather than worked around. With
`assume_depth_m` set, every pixel is deprojected at a known working distance
and every resulting point is tagged `depth_assumed` in its meta. A cell where
the parts sit on a fixture of known height is exactly that case. Without it,
this camera produces no points and says why.

A webcam does not report its own lens, so give it a calibration file or, for a
rough answer, the horizontal field of view.
"""

from __future__ import annotations

import numpy as np

from core import CellError, find_file

from .base import Camera, Frame, Lens


class Webcam(Camera):
    """
        Webcam(device=0, assume_depth_m=0.45, lens_file="webcam_lens.json")
        Webcam(device="rtsp://192.168.1.50/stream", fov_deg=68)
    """

    name = "webcam"
    has_depth = False

    def __init__(self, device=0, width=None, height=None, assume_depth_m=None,
                 lens_file=None, fov_deg=None):
        self.device, self.width, self.height = device, width, height
        self.assume_depth_m = assume_depth_m
        self.lens_file, self.fov_deg = lens_file, fov_deg
        self._cap = self._lens = None

    def _make_lens(self, w: int, h: int) -> Lens:
        if self.lens_file:
            import json
            path = find_file(self.lens_file, "lens file")
            d = json.loads(path.read_text(encoding="utf-8"))
            if "fx" in d:
                return Lens(d.get("width", w), d.get("height", h),
                            d["fx"], d["fy"], d["cx"], d["cy"],
                            tuple(d.get("distortion", (0.0,) * 5)))
            if "camera_matrix" in d:                      # an OpenCV calibration
                K = np.asarray(d["camera_matrix"], dtype=float).reshape(3, 3)
                dist = np.asarray(d.get("distortion_coefficients", [0.0] * 5),
                                  dtype=float).ravel()
                return Lens(w, h, float(K[0, 0]), float(K[1, 1]),
                            float(K[0, 2]), float(K[1, 2]), tuple(dist))
            raise CellError(f"{path} has neither 'fx' nor 'camera_matrix'")

        if self.fov_deg:
            f = (w / 2.0) / np.tan(np.radians(self.fov_deg) / 2.0)
            print(f"  !! {self.name}: lens estimated from a {self.fov_deg} deg "
                  f"field of view -- good to a few percent, nowhere near good "
                  f"enough for a sniffer. Calibrate before trusting millimetres.")
            return Lens(w, h, float(f), float(f), w / 2.0, h / 2.0)

        raise CellError(
            f"{self.name} has no lens model. Give it lens_file= (a calibration) "
            f"or fov_deg= (a rough estimate). Without one, a pixel cannot "
            f"become millimetres at all.")

    def open(self) -> None:
        import cv2
        cap = cv2.VideoCapture(self.device)
        if self.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            raise CellError(f"camera {self.device!r} opened but delivered no frame")
        h, w = frame.shape[:2]
        self._lens, self._cap = self._make_lens(w, h), cap
        print(f"  camera: {self.name} {w}x{h}, no depth"
              + (f", assuming {self.assume_depth_m} m" if self.assume_depth_m else ""))

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def read(self, timeout_s: float = 5.0) -> Frame:
        if self._cap is None:
            raise CellError("the camera has not been opened -- call camera.start()")
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise CellError("the camera stopped delivering frames")
        return Frame(frame, None, self._lens)

    @property
    def lens(self) -> Lens:
        if self._lens is None:
            raise CellError("the lens model is only known after the camera opens")
        return self._lens
