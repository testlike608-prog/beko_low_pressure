"""The maths. No hardware, no I/O, no settings."""

import math

import numpy as np
import pytest

from core import (BASE, CAMERA, Batch, CellError, HandEye, Pixel, Point, Pose,
                  Result, angle_between, check_frame, invert, matrix_to_pose,
                  matrix_to_rpy, move_points, pose_facing, pose_to_matrix,
                  rpy_to_matrix, tool_axis)


# ----------------------------------------------------------------- rotations
@pytest.mark.parametrize("rpy", [
    (0, 0, 0), (180, 0, 175), (-30, 45, 120), (178, -3, 175), (1, 2, 3),
])
def test_rpy_round_trips(rpy):
    back = matrix_to_rpy(rpy_to_matrix(*rpy))
    assert np.allclose(np.array(back), np.array(rpy), atol=1e-8)


def test_rotation_is_orthonormal():
    R = rpy_to_matrix(37.0, -12.0, 155.0)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(R) - 1.0) < 1e-12


def test_gimbal_lock_does_not_make_nan():
    rpy = matrix_to_rpy(rpy_to_matrix(0.0, 90.0, 40.0))
    assert not any(math.isnan(v) for v in rpy)
    assert np.allclose(rpy_to_matrix(*rpy), rpy_to_matrix(0.0, 90.0, 40.0),
                       atol=1e-9)


def test_pose_matrix_round_trip():
    p = Pose(734.0, -426.0, -283.0, 178.0, -3.0, 175.0)
    back = matrix_to_pose(pose_to_matrix(p))
    assert np.allclose(back.list(), p.list(), atol=1e-9)


def test_invert_is_exact():
    T = pose_to_matrix(Pose(100.0, -200.0, 300.0, 20.0, -40.0, 60.0))
    assert np.allclose(invert(T) @ T, np.eye(4), atol=1e-12)


def test_tool_axis_is_the_z_column():
    p = Pose(0, 0, 0, 180, 0, 0)          # tool pointing straight down
    assert np.allclose(tool_axis(p), [0, 0, -1], atol=1e-9)


def test_pose_facing_points_the_tool_along_the_normal():
    direction = np.array([0.3, -0.5, -0.8])
    direction = direction / np.linalg.norm(direction)
    pose = pose_facing((100, 200, 300), direction)
    assert angle_between(tool_axis(pose), direction) < 1e-6


# -------------------------------------------------------------------- poses
def test_pose_of_rejects_wrong_length():
    with pytest.raises(ValueError):
        Pose.of([1, 2, 3])


def test_pose_at_keeps_orientation():
    p = Pose(1, 2, 3, 10, 20, 30)
    q = p.at((9, 9, 9))
    assert q.xyz == (9, 9, 9)
    assert q.rpy == (10, 20, 30)


def test_pose_shifted():
    assert Pose(0, 0, 0).shifted(dz=30).z == 30


# ------------------------------------------------------------------- points
def test_unknown_frame_raises():
    with pytest.raises(ValueError):
        Point(0, 0, 0, frame="worldish")
    assert check_frame(BASE) == BASE


def test_moving_a_point_carries_the_rotation_to_the_normal_only():
    # A pure translation must leave the normal alone.
    T = pose_to_matrix(Pose(1000, 0, 0, 0, 0, 0))
    p = Point(0, 0, 100, CAMERA, normal=(0, 0, 1))
    moved = move_points(T, [p], BASE)[0]
    assert moved.frame == BASE
    assert np.allclose(moved.xyz, (1000, 0, 100))
    assert np.allclose(moved.normal, (0, 0, 1))


def test_a_rotation_does_turn_the_normal():
    T = pose_to_matrix(Pose(0, 0, 0, 0, 0, 90))
    p = Point(0, 0, 0, CAMERA, normal=(1, 0, 0))
    moved = move_points(T, [p], BASE)[0]
    assert np.allclose(moved.normal, (0, 1, 0), atol=1e-9)


def test_distance_across_frames_is_refused():
    with pytest.raises(ValueError):
        Point(0, 0, 0, CAMERA).distance_to(Point(0, 0, 0, BASE))


# ----------------------------------------------------------------- hand-eye
def test_handeye_loads_and_describes(tmp_path):
    he = HandEye("handeye.json")
    assert he.mode == "eye_in_hand"
    assert "hand-eye" in he.describe()
    x, y, z = he.offset_mm
    assert -50 < x < -20 and -60 < y < -30 and 60 < z < 100


def test_handeye_needs_the_flange_pose():
    he = HandEye("handeye.json")
    batch = Batch(points=[Point(0, 0, 450, CAMERA)], flange=None)
    with pytest.raises(CellError):
        he.to_base(batch)


def test_handeye_is_reversible():
    he = HandEye("handeye.json")
    flange = Pose(600, -300, 100, 180, 0, 175)
    batch = Batch(points=[Point(12.0, -8.0, 450.0, CAMERA, "w1")], flange=flange)
    base_point = he.to_base(batch)[0]
    assert base_point.frame == BASE
    back = he.to_camera(base_point, flange)
    assert np.allclose(back.xyz, (12.0, -8.0, 450.0), atol=1e-6)


def test_handeye_moves_with_the_arm():
    """The same pixel seen from two flange poses must land in two different
    base positions -- that is what eye-in-hand MEANS."""
    he = HandEye("handeye.json")
    p = Point(0, 0, 450, CAMERA)
    a = he.to_base(Batch([p], flange=Pose(600, -300, 100, 180, 0, 175)))[0]
    b = he.to_base(Batch([p], flange=Pose(700, -300, 100, 180, 0, 175)))[0]
    assert abs(a.distance_to(b) - 100.0) < 1e-6


def test_handeye_passes_base_points_through_untouched():
    he = HandEye("handeye.json")
    p = Point(1, 2, 3, BASE)
    assert he.to_base(Batch([p], flange=None))[0] is p


def test_handeye_rejects_a_file_that_is_not_one(tmp_path):
    bad = tmp_path / "nope.json"
    bad.write_text('{"hello": 1}')
    with pytest.raises(CellError):
        HandEye(bad)


def test_handeye_warns_about_an_all_odd_board():
    """The 9x7 board really did flip its origin between captures -- the
    warning that explains it must not get lost in a refactor."""
    he = HandEye("handeye.json")
    warnings = " ".join(he.warnings())
    assert "9x7" in warnings or "odd" in warnings


# ------------------------------------------------------------------ results
def test_result_line_and_dict():
    r = Result(1, "weld1", Point(1, 2, 3, BASE), Pose(), reached=True, leak="pass")
    assert r.ok
    assert "PASS" in r.line()
    assert r.dict()["leak"] == "pass"


def test_pixel_str_is_readable():
    assert "weld1" in str(Pixel(10, 20, 0.45, label="weld1"))
