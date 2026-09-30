"""
The whole sequence with NO hardware: simulator arm, fake camera, fixed points.

    python -m cobot_kit.selftest          (run from the folder that holds app.py)

If this passes, the modules and the maths are fine, and anything that goes
wrong on the real cell is the hardware, the pendant, or the calibration.
"""

from __future__ import annotations

import sys
import tempfile
import threading
import time


def main() -> int:
    from cobot_kit import (Camera, CellError, FixedPoints, FunctionPicker,
                           Locator, Robot, cycle)

    failures = []

    def check(name, ok, detail=""):
        print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
        if not ok:
            failures.append(name)

    tmp = tempfile.mkdtemp(prefix="cobot_kit_")
    robot = Robot(kind="simulator", speed_factor=0.0)
    camera = Camera(backend="fake", capture_folder=tmp)
    ai = FixedPoints([(640, 360), (740, 380), (540, 380)])
    locator = Locator(warn=False)

    try:
        robot.start(); camera.start(); ai.start()
        robot.start()                                   # twice = no-op
        check("start twice is harmless", robot.is_connected)

        positions = [{"joints": [0, -20, -90, -70, 90, 0]},
                     {"joints": [5, -20, -90, -70, 90, 0]}]
        shots = cycle.capture(robot, camera, positions)
        check("one picture per capture position", len(shots) == 2)
        check("pictures saved", all(s.image_path and s.image_path.exists()
                                    for s in shots))
        check("each shot carries its flange pose", all(s.flange for s in shots))

        cycle.detect(shots, ai)
        check("AI pixels on every shot", all(len(s.pixels) == 3 for s in shots))

        points = cycle.locate(shots, locator)
        check("same weld from 2 positions kept once", len(points) == 3,
              f"{len(points)} points")
        check("points are in the robot base frame",
              all(p.frame == "base" for p in points))
        print("\n  base-frame points:")
        for p in points:
            print(f"     {p}")

        # the maths, checked backwards: base point -> camera -> pixel again
        shot = shots[0]
        back = locator.handeye.to_camera(points[0], shot.flange)
        u, v = shot.frame.lens.project(back.xyz)
        check("pixel -> base -> pixel round trip", abs(u - 640) < 0.5 and abs(v - 360) < 0.5,
              f"({u:.2f}, {v:.2f})")

        # ---- the pose helpers _robot_cycle uses
        from cobot_kit import Point, approach_pose, point_pose
        pose = point_pose(points[0], robot.pose().rpy)
        check("point_pose puts the tip on the point",
              max(abs(a - b) for a, b in zip(pose.xyz, points[0].xyz)) < 1e-9)
        app_pose = approach_pose(pose, 60)
        check("approach is 60 mm back along the tool axis (up, tool facing down)",
              abs(app_pose.distance_to(pose) - 60) < 1e-6 and app_pose.z > pose.z,
              f"dz {app_pose.z - pose.z:+.1f}")
        try:
            point_pose(Point(1, 2, 3, frame="camera"), (180, 0, 0))
            check("a camera-frame point is refused", False)
        except CellError:
            check("a camera-frame point is refused", True)

        # ---- app.py's _robot_cycle, exactly as written in example_app.py
        from types import SimpleNamespace
        from cobot_kit.example_app import App
        outs = []
        robot.write_output = lambda i, v: outs.append((i, int(v)))  # record, do not echo into DI
        robot.arm.io[0] = 1                                         # the tester answers at once
        app = SimpleNamespace(robot=robot, robot_is_connected=True)
        far = Point(5000, 0, 0, frame="base", label="far")
        res = App._robot_cycle(app, points[:2] + [far] + points[2:])
        check("every reachable point done, in order",
              [r["point"] for r in res if r["ok"]] == [1, 2, 4])
        check("an unreachable point is skipped, the cycle carries on",
              not res[2]["ok"] and len(res) == 4)
        check("output 1 then 0 at every reached point",
              outs.count((0, 1)) == 3 and outs[:2] == [(0, 1), (0, 0)])
        check("the tip ended backed off above the last point",
              robot.pose().distance_to(approach_pose(point_pose(points[2],
                                                                pose.rpy), 60)) < 1e-6)

        # a Stop while waiting for the input must not hang for WAIT_S
        robot.arm.io[0] = 0
        app.robot_is_connected = True
        threading.Timer(0.3, lambda: setattr(app, "robot_is_connected", False)).start()
        t0 = time.time()
        res = App._robot_cycle(app, points)
        check("stop while waiting for the input returns at once",
              time.time() - t0 < 5 and len(res) == 1 and not res[0]["ok"],
              f"{time.time() - t0:.1f} s")
        del robot.write_output

        fp = FunctionPicker(lambda img: [(640, 360, "weld", 0.9)])
        check("FunctionPicker wraps any model", fp.find(shot)[0].label == "weld")

        try:
            Robot(kind="nope")
            check("bad robot kind raises a sentence", False)
        except CellError:
            check("bad robot kind raises a sentence", True)

    finally:
        ai.stop(); camera.stop(); robot.stop()
        robot.stop(); camera.stop()                     # twice = no-op

    check("stop twice is harmless", not robot.is_connected)
    print(f"\n  {'ALL GOOD' if not failures else f'{len(failures)} FAILED'}"
          f"   (pictures in {tmp})\n")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
