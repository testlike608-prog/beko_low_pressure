"""
A person, pointing.

Until the weld-point model is trained this is the source of points -- and it
stays useful afterwards, because it is how you check a suspicious result: click
the weld yourself, watch where the arm goes, and you have separated a vision
problem from a calibration problem in thirty seconds.

Left click   add a point
Right click  remove the last one
ENTER        done
ESC          cancel (returns nothing)
"""

from __future__ import annotations

from core import CellError, Pixel

from .base import Detector


class Clicks(Detector):
    """
        Clicks()                        click, press ENTER
        Clicks(expect=6)                stop automatically after six
        Clicks(window="weld points")
    """

    name = "clicks"

    def __init__(self, expect=None, window="click the welding points",
                 max_width=1280, label="weld"):
        self.expect = expect
        self.window = window
        self.max_width = int(max_width)
        self.label = label

    def find(self, frame) -> list[Pixel]:
        try:
            import cv2
        except ImportError as e:
            raise CellError("opencv is not installed (pip install "
                            "opencv-python)") from e
        if frame.color is None:
            raise CellError("there is no image to click on")

        image = frame.color
        h, w = image.shape[:2]
        scale = min(1.0, self.max_width / float(w))
        shown = cv2.resize(image, None, fx=scale, fy=scale) if scale < 1.0 else image.copy()

        picked: list[tuple[float, float]] = []

        def redraw():
            canvas = shown.copy()
            for i, (u, v) in enumerate(picked, 1):
                p = (int(u * scale), int(v * scale))
                cv2.circle(canvas, p, 7, (0, 255, 0), 2)
                cv2.putText(canvas, str(i), (p[0] + 10, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            need = f" / {self.expect}" if self.expect else ""
            cv2.putText(canvas, f"{len(picked)}{need} points   "
                                f"ENTER = done, right click = undo, ESC = cancel",
                        (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow(self.window, canvas)

        def on_mouse(event, x, y, flags, _):
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
                if key in (13, 10):                       # ENTER
                    break
                if key == 27:                             # ESC
                    cancelled = True
                    break
                if self.expect and len(picked) >= self.expect:
                    break
                if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                    cancelled = True
                    break
        finally:
            cv2.destroyWindow(self.window)
            cv2.waitKey(1)

        if cancelled:
            return []
        return [Pixel(u, v, None, label=f"{self.label}{i}",
                      meta={"detector": "clicks"})
                for i, (u, v) in enumerate(picked, 1)]

    def describe(self) -> str:
        return f"clicks (a person picks{f', expecting {self.expect}' if self.expect else ''})"
