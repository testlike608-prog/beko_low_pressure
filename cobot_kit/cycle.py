"""
The sequence, one function per line of app.py's _start_sequance comments.

    shots  = capture(robot, camera)            # move to capture positions + pictures
    shots  = detect(shots, ai)                 # AI -> x, y pixels on every picture
    points = locate(shots, locator)            # pixels -> x, y, z for the cobot

Or all three at once:   points = look(robot, camera, ai, locator)

Visiting the points is NOT in here -- that is app.py's _robot_cycle, written
out step by step so outputs / inputs can go between the moves. The two helpers
at the bottom only do the pose maths for it:

    pose     = point_pose(point, orientation)     # where the tip goes
    approach = approach_pose(pose, 60)            # 60 mm back along the tool axis
"""

from __future__ import annotations

import math
import time

import numpy as np

from . import settings
from .datatypes import CellError, Pose
from .geometry import tool_axis


# ============================================================ 1. capture
def capture(robot, camera, positions=None, settle_s=None, save=True) -> list:
    """
    Visit every capture position and take one picture at each.

    positions: list of {"joints": [...]} or {"pose": [...]} (or a bare list of
    six numbers = joints). None -> settings.CAPTURE_POSITIONS; an empty list
    means "one picture from wherever the arm stands now".
    Returns a list of Shots; each image is saved to the capture folder.
    """
    positions = settings.CAPTURE_POSITIONS if positions is None else positions
    settle_s = settings.SETTLE_S if settle_s is None else settle_s
    if not positions:
        return [camera.shoot(robot, name="here", save=save)]

    shots = []
    for i, pos in enumerate(positions, 1):
        name = f"pos{i}"
        if isinstance(pos, dict) and "pose" in pos:
            robot.move_to(pos["pose"], label=f"capture {name}")
        else:
            joints = pos["joints"] if isinstance(pos, dict) else pos
            robot.move_joints(joints, label=f"capture {name}")
        time.sleep(settle_s)
        shot = camera.shoot(robot, name=name, save=save)
        print(f"  picture {name}"
              + (f" -> {shot.image_path.name}" if shot.image_path else ""))
        shots.append(shot)
    return shots


# ============================================================= 2. detect
def detect(shots, ai, draw_to_file=True) -> list:
    """Run the AI on every picture. Fills shot.pixels. Returns the shots."""
    from .ai import draw
    for shot in shots:
        shot.pixels = ai.find(shot)
        print(f"  {shot.name}: the AI picked {len(shot.pixels)} point(s)")
        if draw_to_file and shot.image_path is not None:
            try:
                draw(shot, shot.pixels,
                     shot.image_path.with_name(shot.image_path.stem + "_ai.png"))
            except Exception as e:
                print(f"  !! could not save the annotated picture: {e}")
    return shots


# ============================================================= 3. locate
def locate(shots, locator, merge_mm=5.0) -> list:
    """
    Every shot's pixels -> base-frame points, merged into one list.
    The same weld seen from two capture positions is kept ONCE (merge_mm).
    """
    points = []
    for shot in shots:
        points += locator.to_robot(shot)
    return merge_close(points, merge_mm) if merge_mm else points


def merge_close(points, min_gap_mm=5.0) -> list:
    """Drop a point closer than min_gap_mm to one already kept (best confidence wins)."""
    kept: list = []
    for p in sorted(points, key=lambda p: -p.confidence):
        if all(math.dist(p.xyz, k.xyz) > min_gap_mm for k in kept):
            kept.append(p)
    order = {id(p): i for i, p in enumerate(points)}
    return sorted(kept, key=lambda p: order[id(p)])


def look(robot, camera, ai, locator, positions=None) -> list:
    """capture + detect + locate. -> base-frame points for _robot_cycle()."""
    return locate(detect(capture(robot, camera, positions), ai), locator)


# ========================================================= pose helpers
def point_pose(point, orientation, clearance_mm=0.0) -> Pose:
    """
    A base-frame Point -> the pose the TIP goes to.

    orientation: [rx, ry, rz] -- usually robot.pose().rpy, i.e. keep the tool
    pointing the way it points now. clearance_mm stops that far SHORT of the
    point along the tool axis (the sniffer does not touch).
    """
    if getattr(point, "frame", "base") != "base":
        raise CellError(f"{point} is in the {point.frame} frame -- run it "
                        f"through cycle.locate() / Locator.to_robot() first")
    xyz = point.xyz if hasattr(point, "xyz") else tuple(point[:3])
    pose = Pose(*xyz, *[float(v) for v in orientation])
    return approach_pose(pose, clearance_mm) if clearance_mm else pose


def approach_pose(pose, distance_mm) -> Pose:
    """
    The same pose backed off distance_mm along the TOOL axis (not along z-up:
    the tool is not always vertical, and backing off the wrong way is how a
    sniffer meets the part it was avoiding).
    """
    pose = pose if isinstance(pose, Pose) else Pose.of(pose)
    return pose.at(np.asarray(pose.xyz) - tool_axis(pose) * float(distance_mm))
