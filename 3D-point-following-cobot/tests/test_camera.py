"""The camera layer, on the fake driver. No hardware."""

import numpy as np
import pytest

import camera
from camera.base import Camera, Frame, Lens
from core import CAMERA, CellError, Pixel


@pytest.fixture(autouse=True)
def _closed():
    camera.stop()
    yield
    camera.stop()


# --------------------------------------------------------------------- lens
def test_deproject_and_project_round_trip():
    lens = Lens(1280, 720, 911.7, 911.85, 644.0, 368.6)
    xyz = lens.deproject(700.0, 400.0, 450.0)
    u, v = lens.project(xyz)
    assert abs(u - 700.0) < 1e-9 and abs(v - 400.0) < 1e-9


def test_the_principal_point_deprojects_onto_the_axis():
    lens = Lens(1280, 720, 900.0, 900.0, 640.0, 360.0)
    x, y, z = lens.deproject(640.0, 360.0, 500.0)
    assert (abs(x), abs(y), z) == (0.0, 0.0, 500.0)


def test_projecting_something_behind_the_lens_is_an_error():
    lens = Lens(640, 480, 500.0, 500.0, 320.0, 240.0)
    with pytest.raises(CellError):
        lens.project((10.0, 10.0, 0.0))


# -------------------------------------------------------------------- depth
def test_zeros_in_the_depth_image_are_not_averaged_in():
    """A hole in the depth map must make depth_at return None, not a value
    dragged towards the lens. This is the bug that produces points a few
    millimetres out and looks like a calibration problem."""
    cam = camera.build({"kind": "fake"})
    depth = np.zeros((100, 100), np.float32)
    depth[50, 50] = 0.45                       # exactly one good pixel
    frame = Frame(np.zeros((100, 100, 3), np.uint8), depth, cam.lens)
    assert cam.depth_at(frame, 50, 50) is None


def test_a_solid_patch_gives_the_median():
    cam = camera.build({"kind": "fake"})
    depth = np.full((100, 100), 0.5, np.float32)
    frame = Frame(np.zeros((100, 100, 3), np.uint8), depth, cam.lens)
    assert abs(cam.depth_at(frame, 50, 50) - 0.5) < 1e-6


def test_a_pixel_outside_the_image_gives_none():
    cam = camera.build({"kind": "fake"})
    frame = Frame(np.zeros((10, 10, 3), np.uint8),
                  np.full((10, 10), 0.5, np.float32), cam.lens)
    assert cam.depth_at(frame, 999, 999) is None


def test_no_depth_and_no_assumption_produces_no_point():
    class Flat(Camera):
        name = "flat"
        _lens = Lens(100, 100, 100.0, 100.0, 50.0, 50.0)

        @property
        def lens(self):
            return self._lens

    cam = Flat()
    frame = Frame(np.zeros((100, 100, 3), np.uint8), None, cam.lens)
    good, bad = cam.to_points(frame, [Pixel(50, 50)])
    assert good == [] and len(bad) == 1


def test_an_assumed_depth_is_used_and_marked():
    class Flat(Camera):
        name = "flat"
        assume_depth_m = 0.45
        _lens = Lens(100, 100, 100.0, 100.0, 50.0, 50.0)

        @property
        def lens(self):
            return self._lens

    cam = Flat()
    frame = Frame(np.zeros((100, 100, 3), np.uint8), None, cam.lens)
    good, bad = cam.to_points(frame, [Pixel(50, 50)])
    assert not bad
    assert good[0].meta["depth_assumed"] is True
    assert abs(good[0].z - 450.0) < 1e-6


def test_a_point_comes_out_in_the_camera_frame():
    cam = camera.build({"kind": "fake"})
    frame = cam.read()
    good, bad = cam.to_points(frame, [Pixel(700, 400, label="w1")])
    assert not bad
    assert good[0].frame == CAMERA
    assert good[0].label == "w1"


# ---------------------------------------------------------------- lifecycle
def test_start_is_idempotent():
    a = camera.start({"kind": "fake"})
    b = camera.start({"kind": "fake"})
    assert a is b


def test_stop_without_start_is_fine():
    camera.stop()
    camera.stop()


def test_reading_before_start_says_so():
    with pytest.raises(CellError):
        camera.read()


def test_an_unknown_kind_names_the_known_ones():
    with pytest.raises(CellError) as e:
        camera.build({"kind": "hikrobot"})
    assert "realsense" in str(e.value)


def test_a_bad_setting_key_is_caught_before_opening():
    with pytest.raises(CellError):
        camera.build({"kind": "fake", "colour_temperature": 5600})


def test_realsense_imports_without_the_sdk():
    """The whole registry must import on a PC with no vendor SDKs -- otherwise
    the test suite cannot run anywhere but the cell."""
    from camera.realsense import RealSense
    cam = RealSense()
    assert cam.has_depth
    with pytest.raises(CellError):
        cam.lens                                 # not open yet


# ------------------------------------------------------------------ replay
def test_the_fake_camera_replays_saved_frames(tmp_path):
    cv2 = pytest.importorskip("cv2")
    folder = tmp_path / "shots"
    folder.mkdir()
    cv2.imwrite(str(folder / "a.png"), np.full((120, 160, 3), 80, np.uint8))
    np.save(folder / "a.npy", np.full((120, 160), 0.33, np.float32))

    cam = camera.start({"kind": "fake", "folder": str(folder)})
    frame = camera.read()
    assert frame.size == (160, 120)
    assert abs(cam.depth_at(frame, 80, 60) - 0.33) < 1e-6
    # The lens model scaled with the image instead of silently keeping 1280x720
    assert frame.lens.width == 160


def test_save_writes_an_image_and_a_depth_file(tmp_path):
    pytest.importorskip("cv2")
    camera.start({"kind": "fake"})
    path = camera.save(camera.read(), tmp_path, "shot")
    assert path.exists()
    assert (path.parent / "shot.npy").exists()
