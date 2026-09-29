"""
The vision layer and the whole cycle, end to end on fakes.

Everything here runs on any laptop: a fake camera, a fake detector, a
simulator arm, a fake sniffer -- and the REAL hand-eye maths in the middle.
That is the part worth testing without hardware; the hardware itself is what
check.py's ladder is for.
"""

import numpy as np
import pytest

import camera
import cell
import robot
import sniffer
import vision
from core import BASE, CAMERA, Batch, CellError, Pixel, Point, Pose
from vision.base import drop_duplicates, sort_reading_order

FAKE = {
    "CAMERA": {"kind": "fake"},
    "ROBOT": {"kind": "simulator", "speed_factor": 0.0, "quiet": True},
    "VISION": {"kind": "fixed", "count": 4},
    "SNIFFER": {"kind": "fake", "seconds": 0.0},
}


@pytest.fixture
def fake_cell(monkeypatch, tmp_path):
    """A whole cell made of fakes, with settings.py monkeypatched."""
    import settings

    for name, value in FAKE.items():
        monkeypatch.setattr(settings, name, value)
    monkeypatch.setattr(settings, "CAPTURE_POSE", None)
    monkeypatch.setattr(settings, "DWELL_S", 0.0)
    monkeypatch.setattr(settings, "SAVE_CAPTURES", None)
    monkeypatch.setattr(settings, "RESULTS_FILE", str(tmp_path / "results.jsonl"))
    monkeypatch.setattr(settings, "HOME_ON_START", False)
    monkeypatch.setattr(settings, "HOME_ON_STOP", False)

    robot.stop(); camera.stop(); vision.stop(); sniffer.stop()
    robot.start(home=False, verify=False)
    camera.start()
    vision.start()
    sniffer.start()
    yield settings
    sniffer.stop(); vision.stop(); camera.stop(); robot.stop()


# ------------------------------------------------------------------- vision
def test_a_detector_returns_pixels_not_millimetres():
    det = vision.build({"kind": "fixed", "count": 3})
    cam = camera.build({"kind": "fake"})
    pixels = det.find(cam.read())
    assert len(pixels) == 3
    assert all(isinstance(p, Pixel) for p in pixels)


def test_reading_order_is_left_to_right_top_to_bottom():
    pixels = [Pixel(500, 100, label="c"), Pixel(100, 100, label="a"),
              Pixel(300, 400, label="d"), Pixel(300, 100, label="b")]
    assert [p.label for p in sort_reading_order(pixels)] == ["a", "b", "c", "d"]


def test_duplicates_are_dropped_by_confidence_not_averaged():
    """Averaging two boxes that landed on different features invents a point on
    neither. The lower-confidence detection is dropped instead."""
    kept = drop_duplicates([Pixel(100, 100, confidence=0.9, label="keep"),
                            Pixel(103, 101, confidence=0.4, label="drop"),
                            Pixel(400, 400, confidence=0.5, label="far")],
                           min_gap_px=15)
    labels = {p.label for p in kept}
    assert labels == {"keep", "far"}


def test_look_attaches_the_flange_pose_to_the_batch():
    camera.stop(); vision.stop()
    camera.start({"kind": "fake"})
    vision.start({"kind": "fixed", "count": 2})
    try:
        flange = Pose(600, -300, 100, 180, 0, 175)
        batch = vision.look(camera.read(), flange)
        assert isinstance(batch, Batch)
        assert batch.flange == flange
        assert len(batch) == 2
        assert all(p.frame == CAMERA for p in batch)
    finally:
        vision.stop(); camera.stop()


def test_points_without_depth_are_dropped_and_counted():
    """One bad pixel out of six is a point to skip and mention, not an
    exception that kills the cycle."""
    import camera as camera_module

    cam = camera_module.build({"kind": "fake"})
    cam.open()
    frame = cam.read()
    frame.depth[:] = 0.0                       # no depth anywhere
    frame.depth[300:420, 600:700] = 0.45       # except one island

    good, bad = cam.to_points(frame, [Pixel(650, 360), Pixel(50, 50)])
    assert len(good) == 1 and len(bad) == 1


def test_an_unknown_detector_kind_names_the_known_ones():
    with pytest.raises(CellError) as e:
        vision.build({"kind": "sam"})
    assert "yolo" in str(e.value) and "clicks" in str(e.value)


def test_the_yolo_driver_imports_without_torch():
    from vision.yolo import Yolo
    det = Yolo("weld.pt", task="pose")
    assert det.gives_normals
    with pytest.raises(CellError):
        det.find(object())                     # not loaded, and it says so


# --------------------------------------------------------------------- plan
def test_plan_turns_camera_points_into_base_poses(fake_cell):
    frame, batch = cell.capture(save=False)
    targets = cell.plan(batch)
    assert len(targets) == len(batch)
    assert all(t.point.frame == BASE for t in targets)


def test_the_approach_stands_off_along_the_tool_axis(fake_cell, monkeypatch):
    import settings
    monkeypatch.setattr(settings, "APPROACH_MM", 60.0)
    monkeypatch.setattr(settings, "CLEARANCE_MM", 0.0)

    frame, batch = cell.capture(save=False)
    target = cell.plan(batch)[0]
    assert target.approach is not None
    gap = target.approach.distance_to(target.pose)
    assert abs(gap - 60.0) < 1e-6


def test_clearance_stops_short_of_the_weld(fake_cell, monkeypatch):
    import settings
    monkeypatch.setattr(settings, "CLEARANCE_MM", 0.0)
    frame, batch = cell.capture(save=False)
    touching = cell.plan(batch)[0].pose

    monkeypatch.setattr(settings, "CLEARANCE_MM", 10.0)
    standing_off = cell.plan(batch)[0].pose
    assert abs(touching.distance_to(standing_off) - 10.0) < 1e-6


def test_a_point_with_a_normal_turns_the_tool(fake_cell):
    """The service port has to be entered along its axis, so a point carrying a
    normal must produce a pose whose tool axis follows it."""
    from core import angle_between, tool_axis

    normal = np.array([0.0, 0.0, 1.0])
    batch = Batch(points=[Point(700, -400, -200, BASE, "port",
                                normal=tuple(normal))],
                  flange=robot.flange())
    target = cell.plan(batch)[0]
    assert angle_between(tool_axis(target.pose), -normal) < 1e-3


def test_an_out_of_reach_point_is_flagged_not_raised(fake_cell, monkeypatch):
    import settings
    monkeypatch.setattr(settings, "MAX_REACH_MM", 100.0)
    frame, batch = cell.capture(save=False)
    targets = cell.plan(batch)
    assert all(not t.reachable for t in targets)
    assert all("limit" in t.why_not or "solution" in t.why_not for t in targets)


def test_too_few_points_fails_the_cycle(fake_cell, monkeypatch):
    import settings
    monkeypatch.setattr(settings, "MIN_POINTS", 99)
    frame, batch = cell.capture(save=False)
    with pytest.raises(CellError):
        cell.check_count(cell.plan(batch))


def test_the_wrong_number_for_the_recipe_fails(fake_cell):
    frame, batch = cell.capture(save=False)
    targets = cell.plan(batch)
    cell.check_count(targets, expected=len(targets))          # fine
    with pytest.raises(CellError):
        cell.check_count(targets, expected=len(targets) + 1)


# ---------------------------------------------------------------------- run
def test_a_dry_run_moves_nothing(fake_cell):
    arm = robot.current()
    before = len(arm.log)
    results = cell.run(dry=True)
    assert len(arm.log) == before
    assert results and all(not r.reached for r in results)


def test_a_real_run_visits_every_point_and_tests_it(fake_cell):
    results = cell.run()
    assert len(results) == 4
    assert all(r.reached for r in results)
    assert all(r.leak == "pass" for r in results)


def test_a_failing_weld_is_a_result_not_an_exception(fake_cell):
    sniffer.stop()
    sniffer.start({"kind": "fake", "seconds": 0.0, "fail_labels": ["weld2"]})
    results = cell.run()
    failed = [r for r in results if r.leak == "fail"]
    assert len(failed) == 1 and failed[0].label == "weld2"
    assert all(r.reached for r in results)     # the cycle carried on


def test_an_unreachable_point_does_not_stop_the_others(fake_cell, monkeypatch):
    import settings
    monkeypatch.setattr(settings, "MAX_REACH_MM", 1e9)

    arm = robot.current()
    real_solve = arm.solve
    from core import Unreachable

    def refuse_the_second(pose, seed=None):
        if abs(pose.y - cell_target_y[0]) < 1e-6:
            raise Unreachable("pretend this one is impossible")
        return real_solve(pose, seed)

    frame, batch = cell.capture(save=False)
    targets = cell.plan(batch)
    cell_target_y = [targets[1].pose.y]
    monkeypatch.setattr(arm, "solve", refuse_the_second)

    results = [cell.visit(t) for t in targets]
    assert sum(r.reached for r in results) == len(targets) - 1
    assert any("unreachable" in r.note for r in results)


def test_the_report_is_written(fake_cell, tmp_path):
    import json

    import settings
    results = cell.run()
    cell.report(results)
    lines = open(settings.RESULTS_FILE, encoding="utf-8").read().splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["passed"] == 4 and record["failed"] == 0


def test_capture_refuses_a_frame_taken_while_the_arm_moved(fake_cell, monkeypatch):
    """Transforming a batch with a pose that was never true is a
    tens-of-millimetres error that looks exactly like a bad calibration."""
    arm = robot.current()
    poses = iter([Pose(600, -300, 200, 180, 0, 175),
                  Pose(650, -300, 200, 180, 0, 175)])
    monkeypatch.setattr(arm, "flange_pose", lambda: next(poses))
    with pytest.raises(CellError) as e:
        cell.capture(save=False)
    assert "moved" in str(e.value)


# ------------------------------------------------------------------ sniffer
def test_the_digital_sniffer_drives_the_controller_io(fake_cell):
    arm = robot.current()
    arm.io = {0: 1, 1: 1}                      # ready, and a pass verdict
    sniffer.stop()
    sniffer.start({"kind": "digital", "trigger_out": 3, "ready_in": 0,
                   "pass_in": 1, "fail_in": 2, "settle_s": 0.0, "poll_s": 0.0})
    verdict, value = sniffer.test("weld1", timeout_s=1.0)
    assert verdict == "pass"
    assert arm.io[3] == 0                      # the trigger was pulsed, not left on


def test_a_silent_tester_is_an_error_not_a_pass(fake_cell):
    arm = robot.current()
    arm.io = {0: 1}                            # ready, but never answers
    sniffer.stop()
    sniffer.start({"kind": "digital", "settle_s": 0.0, "poll_s": 0.0})
    verdict, _ = sniffer.test("weld1", timeout_s=0.05)
    assert verdict == "error"
