"""
camera/hub.py -- the bridge to camera_hub.py.

No hardware anywhere here. Two halves:

  * the real CameraHub base class, driven by a stub _capture_loop, so the
    threading and the frame counter are the genuine ones
  * camera.hub.Hub over a hand-written stub, so every branch (stale frame,
    frozen stream, lens priority, bad settings) is reachable on purpose
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

import camera
from camera.hub import Hub
from camera_hub import CameraHub
from core import CellError


# --------------------------------------------------------------- the real base
class Ticker(CameraHub):
    """A camera_hub driver that invents a frame every 20 ms."""

    def _capture_loop(self, camera_index: int):
        n = 0
        try:
            while not self._stop_event.is_set():
                n += 1
                self._set_frame(np.full((8, 8, 3), n % 256, np.uint8))
                time.sleep(0.02)
        finally:
            self._clear_frame()


def test_frame_seq_counts_real_frames():
    cam = Ticker()
    cam.start()
    try:
        assert cam.wait_for_frame(timeout=2.0)
        first = cam.frame_seq
        assert first >= 1
        assert cam.wait_for_new_frame(first, timeout=2.0)
        assert cam.frame_seq > first
    finally:
        cam.stop()


def test_wait_for_new_frame_gives_up_when_the_thread_is_gone():
    cam = Ticker()
    cam.start()
    cam.wait_for_frame(timeout=2.0)
    seq = cam.frame_seq
    cam.stop()
    # Not a five second hang: a stopped camera is an answer, not a wait.
    started = time.time()
    assert cam.wait_for_new_frame(seq, timeout=5.0) is False
    assert time.time() - started < 1.0


def test_get_frame_hands_back_a_copy():
    cam = Ticker()
    cam.start()
    try:
        cam.wait_for_frame(timeout=2.0)
        f = cam.get_frame()
        f[:] = 0
        assert cam.get_frame().any()          # the original is untouched
    finally:
        cam.stop()


# ------------------------------------------------------------------- the stub
class StubDriver:
    """The camera_hub surface Hub uses, with the clock under our thumb."""

    def __init__(self, camera_index=0, depth=True, intrinsics=True, **kw):
        self.camera_index = camera_index
        self.kw = kw
        self.started = self.stopped = False
        self.frame_seq = 0
        self.profile = (640, 480, 30)
        self.intrinsics = ({"width": 640, "height": 480, "fx": 600.0,
                            "fy": 600.0, "cx": 320.0, "cy": 240.0,
                            "distortion": [0.0] * 5} if intrinsics else None)
        self._depth = depth
        self._color = np.zeros((480, 640, 3), np.uint8)
        self._d = np.full((480, 640), 0.5, np.float32)
        self.frozen = False

    # -- lifecycle
    def start(self):
        self.started = True
        self.tick()

    def stop(self):
        self.stopped = True

    def is_running(self):
        return self.started and not self.stopped

    def tick(self):
        self.frame_seq += 1

    # -- reading
    def wait_for_frame(self, timeout=5.0):
        return self.frame_seq > 0

    def wait_for_new_frame(self, since, timeout=5.0):
        if self.frozen:
            return False
        if self.frame_seq > since:
            return True
        self.tick()                      # a live stream produces one
        return True

    def get_frame(self):
        return self._color.copy()

    def get_frames(self):
        return self._color.copy(), (self._d.copy() if self._depth else None)

    def get_depth(self):
        return self._d.copy() if self._depth else None


@pytest.fixture
def hub(monkeypatch):
    """A Hub wired to StubDriver, with the driver reachable as .driver."""
    def make(**kwargs):
        driver_kwargs = kwargs.pop("_driver", {})
        h = Hub(**kwargs)
        monkeypatch.setattr(
            h, "_driver_class",
            lambda: (lambda camera_index=0, **kw: StubDriver(
                camera_index=camera_index, **{**driver_kwargs, **kw})))
        return h
    return make


def test_open_close_round_trip(hub):
    h = hub()
    h.open()
    assert h._hub.started and h.has_depth
    h.close()
    assert h._hub is None


def test_read_returns_a_frame_with_colour_depth_and_lens(hub):
    h = hub()
    h.open()
    f = h.read()
    assert f.color.shape == (480, 640, 3)
    assert f.has_depth
    assert f.lens.fx == 600.0
    h.close()


def test_read_refuses_to_return_the_same_frame_twice(hub):
    """The whole reason this wrapper exists."""
    h = hub()
    h.open()
    h.read()
    seq_after_first = h._last_seq
    h.read()
    assert h._last_seq > seq_after_first
    h.close()


def test_a_frozen_stream_raises_instead_of_serving_a_stale_frame(hub):
    h = hub()
    h.open()
    h.read()
    h._hub.frozen = True
    with pytest.raises(CellError) as e:
        h.read(timeout_s=0.1)
    assert "frozen" in str(e.value)
    h.close()


def test_a_dead_capture_thread_says_so(hub):
    h = hub()
    h.open()
    h._hub.frozen = True
    h._hub.stopped = True
    with pytest.raises(CellError) as e:
        h.read(timeout_s=0.1)
    assert "stopped" in str(e.value)
    h.close()


def test_reading_before_open_is_a_sentence_not_a_crash(hub):
    with pytest.raises(CellError) as e:
        hub().read()
    assert "not been opened" in str(e.value)


# ------------------------------------------------------------------ the lens
def test_device_intrinsics_are_used_when_offered(hub):
    h = hub()
    h.open()
    assert (h.lens.fx, h.lens.cx) == (600.0, 320.0)
    h.close()


def test_an_explicit_lens_outranks_the_device(hub):
    h = hub(lens={"width": 640, "height": 480, "fx": 111.0, "fy": 111.0,
                  "cx": 1.0, "cy": 2.0})
    h.open()
    assert h.lens.fx == 111.0
    h.close()


def test_fov_estimates_a_lens_for_a_camera_that_has_none(hub, capsys):
    h = hub(fov_deg=90, _driver={"intrinsics": False, "depth": False})
    h.open()
    # 90 deg over 640 px -> fx = 320
    assert h.lens.fx == pytest.approx(320.0, abs=0.5)
    assert "approximate" in capsys.readouterr().out
    h.close()


def test_no_intrinsics_and_no_fov_is_refused_with_instructions(hub):
    h = hub(_driver={"intrinsics": False, "depth": False})
    with pytest.raises(CellError) as e:
        h.open()
    assert "fov_deg" in str(e.value)


def test_a_2d_backend_still_yields_points_at_an_assumed_range(hub):
    h = hub(assume_depth_m=0.4, fov_deg=90,
            _driver={"intrinsics": False, "depth": False})
    h.open()
    frame = h.read()
    assert not frame.has_depth
    assert h.depth_at(frame, 320, 240) == 0.4      # the assumption, stated
    h.close()


# --------------------------------------------------------------- the settings
def test_hub_is_in_the_registry():
    assert "hub" in camera.kinds()


def test_build_accepts_the_documented_config():
    cam = camera.build({"kind": "hub", "backend": "realsense",
                        "camera_index": 0, "frame_width": 1280})
    assert isinstance(cam, Hub)
    assert cam.backend_kwargs["frame_width"] == 1280   # passed on untouched


def test_an_unknown_backend_names_the_known_ones():
    cam = camera.build({"kind": "hub", "backend": "nope"})
    with pytest.raises(CellError) as e:
        cam.open()
    assert "realsense" in str(e.value)


def test_describe_works_before_anything_is_opened():
    text = camera.describe({"kind": "hub", "backend": "useeplus"})
    assert "useeplus" in text


def test_describe_does_not_call_a_realsense_2d_before_it_opens():
    assert "colour + depth" in camera.describe({"kind": "hub",
                                                "backend": "realsense"})


def test_describe_after_open_reports_the_real_profile(hub):
    h = hub()
    h.open()
    assert "640x480@30" in h.describe()
    h.close()
